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
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <stdexcept>
#include <thread>

namespace arxlift2s_ros2_control
{
namespace
{
constexpr const char * kHybridKpParam = "arx_lift.hybrid_kp";
constexpr const char * kHybridKdParam = "arx_lift.hybrid_kd";
// MIT Type3 pack clamps kd to this (see libarx_lift_src.so).
constexpr double kHybridKdMax = 5.0;

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

bool parse_double_param(
  const hardware_interface::HardwareInfo & info, const std::string & key,
  double default_value, double & out, const rclcpp::Logger & logger)
{
  const std::string raw = get_hw_param(info, key, "");
  if (raw.empty()) {
    out = default_value;
    return true;
  }
  try {
    out = std::stod(raw);
    return true;
  } catch (const std::exception &) {
    RCLCPP_ERROR(
      logger, "Invalid %s parameter: '%s'", key.c_str(), raw.c_str());
    return false;
  }
}
}  // namespace

void ArxLiftHardware::setupDynamicParameters(double hybrid_kp, double hybrid_kd)
{
  auto node = get_node();
  if (!node) {
    RCLCPP_WARN(
      get_logger(),
      "No ROS node; arx_lift.hybrid_* params will not be dynamically settable");
    return;
  }

  auto declare_or_set_double = [&](const char * name, double value) {
    if (!node->has_parameter(name)) {
      node->declare_parameter<double>(name, value);
    } else {
      node->set_parameter(rclcpp::Parameter(name, value));
    }
  };

  declare_or_set_double(kHybridKpParam, hybrid_kp);
  declare_or_set_double(kHybridKdParam, hybrid_kd);

  auto parse_nonneg = [](const rclcpp::Parameter & p, double & out,
                         std::string & reason, const char * label) -> bool {
    if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
      p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
    {
      reason = std::string(label) + " must be a number";
      return false;
    }
    out = p.as_double();
    if (!(out >= 0.0) || !std::isfinite(out)) {
      reason = std::string(label) + " must be finite and >= 0";
      return false;
    }
    return true;
  };

  param_cb_ = node->add_on_set_parameters_callback(
    [this, parse_nonneg](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : params) {
        const auto & name = p.get_name();
        if (name == kHybridKpParam) {
          double v = 0.0;
          if (!parse_nonneg(p, v, result.reason, kHybridKpParam)) {
            result.successful = false;
            return result;
          }
          hybrid_kp_.store(v);
          RCLCPP_INFO(get_logger(), "%s -> %.3f", kHybridKpParam, v);
        } else if (name == kHybridKdParam) {
          double v = 0.0;
          if (!parse_nonneg(p, v, result.reason, kHybridKdParam)) {
            result.successful = false;
            return result;
          }
          if (v > kHybridKdMax) {
            RCLCPP_WARN(
              get_logger(),
              "%s=%.3f exceeds Hybrid MIT pack max %.1f; will clamp when sending",
              kHybridKdParam, v, kHybridKdMax);
          }
          hybrid_kd_.store(v);
          RCLCPP_INFO(get_logger(), "%s -> %.3f", kHybridKdParam, v);
        }
      }
      return result;
    });

  RCLCPP_INFO(
    get_logger(), "Dynamic Hybrid params: %s=%.3f %s=%.3f", kHybridKpParam,
    hybrid_kp, kHybridKdParam, hybrid_kd);
}

void ArxLiftHardware::sendHybridHoldOrTrack(
  double q_target_sdk, double /*v_unused*/, double dt_s)
{
  if (!lift_) {
    return;
  }

  if (!ramp_initialized_) {
    ramp_q_sdk_ = sdk_get_height_.load();
    ramp_initialized_ = true;
  }

  const double v_ramp_sdk = cmd_ramp_vel_mps_ * height_rad_per_meter_;
  const double err = q_target_sdk - ramp_q_sdk_;
  double v_d = 0.0;
  constexpr double kPosEps = 1e-4;
  if (std::abs(err) <= kPosEps) {
    ramp_q_sdk_ = q_target_sdk;
    v_d = 0.0;
  } else {
    const double max_step = v_ramp_sdk * dt_s;
    const double step = std::copysign(std::min(std::abs(err), max_step), err);
    ramp_q_sdk_ += step;
    v_d = step / dt_s;
  }
  ramp_q_sdk_ = std::clamp(ramp_q_sdk_, 0.0, sdk_max_rad_);

  double hy_kp = hybrid_kp_.load();
  double hy_kd = hybrid_kd_.load();
  // Gains: live ROS params only (ocs2 may claim kp/kd IF but leave 0 — do not override).

  double t_ff = gravity_compensation_torque_;
  if (std::isfinite(lift_effort_command_)) {
    t_ff += lift_effort_command_;
  }
  t_ff = std::clamp(t_ff, -lift_max_torque_, lift_max_torque_);

  lift_->read();
  lift_->exchangeLiftMotorMsg();
  const double p_motor = -ramp_q_sdk_;
  const double v_motor = -v_d;
  const double kd_send = std::min(hy_kd, kHybridKdMax);
  lift_->sendLiftHybrid(hy_kp, kd_send, p_motor, v_motor, t_ff);

  last_written_height_.store(ramp_q_sdk_);
  last_written_vel_.store(v_d);
}

void ArxLiftHardware::stop_loop_thread()
{
  command_enabled_ = false;
  loop_running_ = false;
  if (loop_thread_.joinable()) {
    loop_thread_.join();
  }
}

ArxLiftHardware::~ArxLiftHardware()
{
  stop_loop_thread();
  param_cb_.reset();
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
  const std::string robot_type_str = get_hw_param(info_, "robot_type", "2");
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

  double gravity = -1.8;
  double max_torque = 15.0;
  double hybrid_kp = 5.0;
  double hybrid_kd = 2.0;
  double height_scale = 41.54;
  double span_m = 0.48;
  double sdk_max = 20.0;
  double ramp_vel = 0.04;

  if (!parse_double_param(
      info_, "gravity_compensation_torque", -1.8, gravity, get_logger()) ||
    !parse_double_param(
      info_, "lift_max_torque", 15.0, max_torque, get_logger()) ||
    !parse_double_param(info_, "sdk_max_rad", 20.0, sdk_max, get_logger()) ||
    !parse_double_param(info_, "cmd_ramp_vel", 0.04, ramp_vel, get_logger()) ||
    !parse_double_param(info_, "hybrid_kp", 5.0, hybrid_kp, get_logger()))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // meters ↔ motor rad (prefer height_rad_per_meter; alias height_to_motor)
  if (!get_hw_param(info_, "height_rad_per_meter", "").empty()) {
    if (!parse_double_param(
        info_, "height_rad_per_meter", 41.54, height_scale, get_logger()))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else if (!parse_double_param(
      info_, "height_to_motor", 41.54, height_scale, get_logger()))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!get_hw_param(info_, "height_span_m", "").empty()) {
    if (!parse_double_param(info_, "height_span_m", 0.48, span_m, get_logger())) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else if (!parse_double_param(info_, "max_height_m", 0.48, span_m, get_logger())) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Prefer explicit hybrid_kd; alias kd.
  if (!get_hw_param(info_, "hybrid_kd", "").empty()) {
    if (!parse_double_param(info_, "hybrid_kd", 2.0, hybrid_kd, get_logger())) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else if (!get_hw_param(info_, "kd", "").empty()) {
    if (!parse_double_param(info_, "kd", 2.0, hybrid_kd, get_logger())) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  auto ok_gain = [](double v) { return std::isfinite(v) && v >= 0.0; };
  if (!ok_gain(hybrid_kp) || !ok_gain(hybrid_kd)) {
    RCLCPP_ERROR(
      get_logger(),
      "hybrid_kp/hybrid_kd must be finite and >= 0 (got %.3f / %.3f)",
      hybrid_kp, hybrid_kd);
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (height_scale <= 0.0 || span_m <= 0.0 || sdk_max <= 0.0 || ramp_vel <= 0.0) {
    RCLCPP_ERROR(
      get_logger(),
      "height scale / span / sdk_max / cmd_ramp_vel must be > 0");
    return hardware_interface::CallbackReturn::ERROR;
  }

  gravity_compensation_torque_ = gravity;
  lift_max_torque_ = max_torque;
  height_rad_per_meter_ = height_scale;
  height_span_m_ = span_m;
  sdk_max_rad_ = sdk_max;
  cmd_ramp_vel_mps_ = ramp_vel;
  hybrid_kp_.store(hybrid_kp);
  hybrid_kd_.store(hybrid_kd);

  lift_position_ = 0.0;
  lift_velocity_ = 0.0;
  lift_effort_ = 0.0;
  lift_position_command_ = 0.0;
  lift_velocity_command_ = 0.0;
  lift_effort_command_ = 0.0;
  lift_kp_command_ = 0.0;
  lift_kd_command_ = 0.0;
  command_enabled_ = false;
  last_written_height_ = 0.0;
  last_written_vel_ = 0.0;
  lift_.reset();

  RCLCPP_INFO(
    get_logger(),
    "ArxLiftHardware init: joint=%s can=%s robot_type=%d mode=hybrid "
    "hybrid_kp=%.3f hybrid_kd=%.3f gravity=%.3f ramp=%.3f m/s",
    lift_joint_name_.c_str(), can_name_.c_str(), robot_type_, hybrid_kp,
    hybrid_kd, gravity_compensation_torque_, cmd_ramp_vel_mps_);

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
  // MIX so ocs2_wbc full_control can claim; Hybrid drive uses position (+ optional kp/kd/effort).
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
  stop_loop_thread();
  param_cb_.reset();

  try {
    const auto type =
      static_cast<arx::LiftHeadControlLoop::RobotType>(robot_type_);
    lift_ = std::make_shared<arx::LiftHeadControlLoop>(can_name_.c_str(), type);

    // Soft-P config unused for motion; keep gravity/torque for documentation parity.
    lift_->config_.gravity_compensation_torque = gravity_compensation_torque_;
    lift_->config_.lift_max_torque = lift_max_torque_;
    lift_->config_.lift_kp = 0.0;
    lift_->config_.lift_max_vel = 0.0;

    lift_->setChassisCmd(0.0, 0.0, 0.0, 2);

    setupDynamicParameters(hybrid_kp_.load(), hybrid_kd_.load());

    if (auto node = get_node()) {
      motor_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/arx_lift/motor_status", rclcpp::SystemDefaultsQoS());
    }

    loop_running_ = true;
    command_enabled_ = false;
    ramp_initialized_ = false;
    using namespace std::chrono_literals;
    loop_thread_ = std::thread([this]() {
      auto last = std::chrono::steady_clock::now();
      while (loop_running_.load()) {
        try {
          if (!lift_) {
            std::this_thread::sleep_for(5ms);
            continue;
          }

          const auto now = std::chrono::steady_clock::now();
          double dt_s = std::chrono::duration<double>(now - last).count();
          last = now;
          if (dt_s < 1e-4) {
            dt_s = 1e-4;
          } else if (dt_s > 0.05) {
            dt_s = 0.05;
          }

          if (!command_enabled_.load()) {
            // Warmup / calib: Hybrid hold at current height (no Soft-P).
            const double hold = sdk_get_height_.load();
            if (!ramp_initialized_) {
              ramp_q_sdk_ = hold;
              ramp_initialized_ = true;
            }
            sendHybridHoldOrTrack(ramp_q_sdk_, 0.0, dt_s);
          } else {
            const double q_target = rosToSdk(lift_position_command_);
            sendHybridHoldOrTrack(q_target, 0.0, dt_s);
          }

          const double sdk_h = lift_->getHeight();
          const auto motor = lift_->getLiftMotorStatus();
          sdk_get_height_.store(sdk_h);
          motor_position_.store(motor.position);
          motor_velocity_.store(motor.velocity);
          motor_torque_.store(motor.torque);
          motor_current_.store(motor.current);
          motor_online_.store(lift_->isLiftMotorOnline() ? 1 : 0);
          motor_error_.store(lift_->getLiftMotorErrorCode());

          lift_position_ = sdkToRos(sdk_h);
          lift_velocity_ = sdkToRos(-motor.velocity);
          lift_effort_ = motor.torque;
        } catch (const std::exception & e) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "ArxLiftHardware loop thread failed: %s", e.what());
        }
        std::this_thread::sleep_for(2500us);  // ~400 Hz
      }
    });

    constexpr auto kCalibWait = std::chrono::milliseconds(2500);
    std::this_thread::sleep_for(kCalibWait);

    lift_position_command_ = lift_position_;
    ramp_q_sdk_ = rosToSdk(lift_position_command_);
    ramp_initialized_ = true;
    last_written_height_.store(ramp_q_sdk_);
    last_written_vel_.store(0.0);
    command_enabled_ = true;

    RCLCPP_INFO(
      get_logger(),
      "ArxLiftHardware activated on %s (ros_height=%.3f m, sdk_rad=%.3f, "
      "mode=hybrid kp=%.3f kd=%.3f gravity=%.3f)",
      can_name_.c_str(), lift_position_, sdk_get_height_.load(),
      hybrid_kp_.load(), hybrid_kd_.load(), gravity_compensation_torque_);
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to activate ArxLiftHardware: %s", e.what());
    stop_loop_thread();
    param_cb_.reset();
    lift_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn ArxLiftHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop_loop_thread();
  param_cb_.reset();

  if (lift_) {
    try {
      lift_->setChassisCmd(0.0, 0.0, 0.0, 2);
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(), "Chassis park on deactivate failed: %s", e.what());
    }
  }

  lift_.reset();
  motor_pub_.reset();
  RCLCPP_INFO(get_logger(), "ArxLiftHardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArxLiftHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!lift_) {
    return hardware_interface::return_type::ERROR;
  }
  // State is refreshed in the Hybrid loop thread.
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArxLiftHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!lift_) {
    return hardware_interface::return_type::ERROR;
  }

  const double err = lift_position_command_ - lift_position_;
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "ArxLiftHardware hybrid cmd=%.4f m fb=%.4f m err=%.4f m | "
    "sdk=%.4f motor_pos=%.4f vel=%.4f torq=%.4f online=%d | "
    "kp=%.3f kd=%.3f (last q=%.4f v=%.4f rad/s)",
    lift_position_command_, lift_position_, err, sdk_get_height_.load(),
    motor_position_.load(), motor_velocity_.load(), motor_torque_.load(),
    motor_online_.load(), hybrid_kp_.load(), hybrid_kd_.load(),
    last_written_height_.load(), last_written_vel_.load());

  if (motor_pub_) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
      motor_position_.load(),
      motor_velocity_.load(),
      motor_torque_.load(),
      motor_current_.load(),
      sdk_get_height_.load(),
      static_cast<double>(motor_online_.load()),
      static_cast<double>(motor_error_.load()),
      -motor_position_.load(),
      hybrid_kp_.load(),
      hybrid_kd_.load()};
    motor_pub_->publish(msg);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace arxlift2s_ros2_control

PLUGINLIB_EXPORT_CLASS(
  arxlift2s_ros2_control::ArxLiftHardware, hardware_interface::SystemInterface)
