// Copyright 2026 FiveAges Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "arxlift2s_ros2_control/arx_lift_hardware.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <stdexcept>
#include <thread>

namespace arxlift2s_ros2_control
{
namespace
{
std::string get_hw_param(
  const hardware_interface::HardwareInfo & info, const std::string & key,
  const std::string & default_value = "")
{
  const auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) {
    return default_value;
  }
  return it->second;
}

double get_hw_param_double(
  const hardware_interface::HardwareInfo & info, const std::string & key,
  double default_value)
{
  const auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) {
    return default_value;
  }
  try {
    return std::stod(it->second);
  } catch (const std::exception &) {
    return default_value;
  }
}
}  // namespace

void ArxLiftHardware::applyLiftSdkConfig()
{
  if (!lift_) {
    return;
  }
  lift_->config_.lift_kp = lift_kp_;
  lift_->config_.lift_max_vel = lift_max_vel_;
  lift_->config_.gravity_compensation_torque = gravity_compensation_torque_;
  lift_->config_.lift_max_torque = lift_max_torque_;
  RCLCPP_INFO(
    get_logger(),
    "LiftHeadControlLoop config: lift_kp=%.3f gravity=%.3f max_vel=%.3f "
    "max_torque=%.3f max_height_m=%.3f height_to_motor=%.2f",
    lift_kp_, gravity_compensation_torque_, lift_max_vel_, lift_max_torque_,
    max_height_m_, height_to_motor_);
}

double ArxLiftHardware::sdkHeightToMeters(double sdk_height) const
{
  // getHeight() returns negated motor radians; convert to meters.
  if (height_to_motor_ <= 0.0) {
    return 0.0;
  }
  return std::clamp(std::abs(sdk_height) / height_to_motor_, 0.0, max_height_m_);
}

double ArxLiftHardware::metersToSdkHeight(double height_m) const
{
  // setHeight() expects motor radians (not meters).
  return std::clamp(height_m, 0.0, max_height_m_) * height_to_motor_;
}

ArxLiftHardware::~ArxLiftHardware()
{
  loop_running_ = false;
  if (loop_thread_.joinable()) {
    loop_thread_.join();
  }
  lift_.reset();
}

hardware_interface::CallbackReturn ArxLiftHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 1) {
    RCLCPP_ERROR(
      get_logger(),
      "ArxLiftHardware expects exactly 1 joint, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  lift_joint_name_ = info_.joints.front().name;
  can_name_ = get_hw_param(info_, "can_name", "can5");
  const std::string robot_type_str = get_hw_param(info_, "robot_type", "0");
  try {
    robot_type_ = std::stoi(robot_type_str);
  } catch (const std::exception &) {
    RCLCPP_ERROR(
      get_logger(), "Invalid robot_type parameter: '%s'",
      robot_type_str.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (robot_type_ < 0 || robot_type_ > 2) {
    RCLCPP_ERROR(
      get_logger(), "robot_type must be 0(LIFT)/1(X7S)/2(LIFTS), got %d",
      robot_type_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  lift_position_ = 0.0;
  lift_velocity_ = 0.0;
  lift_effort_ = 0.0;
  lift_position_command_ = 0.0;
  lift_velocity_command_ = 0.0;
  lift_effort_command_ = 0.0;
  lift_kp_command_ = 0.0;
  lift_kd_command_ = 0.0;
  control_mode_ = get_hw_param(info_, "control_mode", "full_control");
  if (control_mode_ != "full_control") {
    RCLCPP_WARN(
      get_logger(),
      "ArxLiftHardware ignores control_mode='%s'; Lift2S lift is full_control-only",
      control_mode_.c_str());
    control_mode_ = "full_control";
  }
  lift_kp_ = get_hw_param_double(info_, "lift_kp", 8.0);
  lift_max_vel_ = get_hw_param_double(info_, "lift_max_vel", 0.10);
  lift_max_torque_ = get_hw_param_double(info_, "lift_max_torque", 15.0);
  max_height_m_ = get_hw_param_double(info_, "max_height_m", 0.48);
  height_to_motor_ = get_hw_param_double(info_, "height_to_motor", 41.54);
  // LIFTS (robot_type=2) field default; override via xacro if needed.
  const double default_gravity = (robot_type_ == 2) ? -1.8 : -2.0;
  gravity_compensation_torque_ =
    get_hw_param_double(info_, "gravity_compensation_torque", default_gravity);

  lift_.reset();

  RCLCPP_INFO(
    get_logger(),
    "ArxLiftHardware init: joint=%s can=%s robot_type=%d control_mode=%s "
    "lift_kp=%.1f max_vel=%.2f",
    lift_joint_name_.c_str(), can_name_.c_str(), robot_type_,
    control_mode_.c_str(), lift_kp_, lift_max_vel_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
ArxLiftHardware::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
  state_interfaces.push_back(
    std::make_shared<hardware_interface::StateInterface>(
      lift_joint_name_, hardware_interface::HW_IF_POSITION, &lift_position_));
  state_interfaces.push_back(
    std::make_shared<hardware_interface::StateInterface>(
      lift_joint_name_, hardware_interface::HW_IF_VELOCITY, &lift_velocity_));
  state_interfaces.push_back(
    std::make_shared<hardware_interface::StateInterface>(
      lift_joint_name_, hardware_interface::HW_IF_EFFORT, &lift_effort_));
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
ArxLiftHardware::on_export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
  // Always export MIX so ocs2_wbc full_control can claim; write() uses position.
  command_interfaces.push_back(
    std::make_shared<hardware_interface::CommandInterface>(
      lift_joint_name_, hardware_interface::HW_IF_POSITION,
      &lift_position_command_));
  command_interfaces.push_back(
    std::make_shared<hardware_interface::CommandInterface>(
      lift_joint_name_, hardware_interface::HW_IF_VELOCITY,
      &lift_velocity_command_));
  command_interfaces.push_back(
    std::make_shared<hardware_interface::CommandInterface>(
      lift_joint_name_, hardware_interface::HW_IF_EFFORT,
      &lift_effort_command_));
  command_interfaces.push_back(
    std::make_shared<hardware_interface::CommandInterface>(
      lift_joint_name_, "kp", &lift_kp_command_));
  command_interfaces.push_back(
    std::make_shared<hardware_interface::CommandInterface>(
      lift_joint_name_, "kd", &lift_kd_command_));
  return command_interfaces;
}

hardware_interface::CallbackReturn ArxLiftHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (loop_thread_.joinable()) {
    loop_running_ = false;
    loop_thread_.join();
  }

  try {
    const auto type =
      static_cast<arx::LiftHeadControlLoop::RobotType>(robot_type_);
    lift_ = std::make_shared<arx::LiftHeadControlLoop>(can_name_.c_str(), type);
    applyLiftSdkConfig();

    // Park chassis once (official timeout/stop semantics)
    lift_->setChassisCmd(0.0, 0.0, 0.0, 2);

    lift_position_ = sdkHeightToMeters(lift_->getHeight());
    lift_position_command_ = lift_position_;
    lift_->setHeight(metersToSdkHeight(lift_position_command_));
    lift_velocity_ = 0.0;
    lift_effort_ = 0.0;

    loop_running_ = true;
    loop_thread_ = std::thread([this]() {
      // Align with official lift_controller Rate(400)
      using namespace std::chrono_literals;
      while (loop_running_.load()) {
        if (lift_) {
          lift_->loop();
        }
        std::this_thread::sleep_for(2500us);  // ~400 Hz
      }
    });

    RCLCPP_INFO(
      get_logger(), "ArxLiftHardware activated on %s (height=%.3f m)",
      can_name_.c_str(), lift_position_);
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to activate ArxLiftHardware: %s", e.what());
    loop_running_ = false;
    if (loop_thread_.joinable()) {
      loop_thread_.join();
    }
    lift_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn ArxLiftHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  loop_running_ = false;
  if (loop_thread_.joinable()) {
    loop_thread_.join();
  }

  if (lift_) {
    try {
      lift_->setChassisCmd(0.0, 0.0, 0.0, 2);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(), "Chassis park on deactivate failed: %s", e.what());
    }
  }

  lift_.reset();
  RCLCPP_INFO(get_logger(), "ArxLiftHardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArxLiftHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!lift_) {
    return hardware_interface::return_type::ERROR;
  }

  try {
    const double sdk_height = lift_->getHeight();
    lift_position_ = sdkHeightToMeters(sdk_height);
    lift_velocity_ = 0.0;
    lift_effort_ = 0.0;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "ArxLiftHardware cmd=%.4f m (sdk=%.3f) feedback=%.4f m (sdk_raw=%.3f)",
      lift_position_command_, metersToSdkHeight(lift_position_command_),
      lift_position_, sdk_height);
    return hardware_interface::return_type::OK;
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "ArxLiftHardware read failed: %s",
      e.what());
    return hardware_interface::return_type::ERROR;
  }
}

hardware_interface::return_type ArxLiftHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!lift_) {
    return hardware_interface::return_type::ERROR;
  }

  try {
    lift_->setHeight(metersToSdkHeight(lift_position_command_));
    return hardware_interface::return_type::OK;
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "ArxLiftHardware write failed: %s",
      e.what());
    return hardware_interface::return_type::ERROR;
  }
}

}  // namespace arxlift2s_ros2_control

PLUGINLIB_EXPORT_CLASS(
  arxlift2s_ros2_control::ArxLiftHardware, hardware_interface::SystemInterface)
