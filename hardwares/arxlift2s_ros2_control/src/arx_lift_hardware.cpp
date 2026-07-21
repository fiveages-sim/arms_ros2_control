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

#include <chrono>
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
}  // namespace

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
  lift_.reset();

  RCLCPP_INFO(
    get_logger(),
    "ArxLiftHardware init: joint=%s can=%s robot_type=%d",
    lift_joint_name_.c_str(), can_name_.c_str(), robot_type_);

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
  command_interfaces.push_back(
    std::make_shared<hardware_interface::CommandInterface>(
      lift_joint_name_, hardware_interface::HW_IF_POSITION,
      &lift_position_command_));
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

    // Park chassis once (official timeout/stop semantics)
    lift_->setChassisCmd(0.0, 0.0, 0.0, 2);

    lift_position_ = lift_->getHeight();
    lift_position_command_ = lift_position_;
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
    lift_position_ = lift_->getHeight();
    lift_velocity_ = 0.0;
    lift_effort_ = 0.0;
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
    lift_->setHeight(lift_position_command_);
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
