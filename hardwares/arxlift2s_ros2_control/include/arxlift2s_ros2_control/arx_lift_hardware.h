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

#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "arx_lift_src/lift_head_control_loop.h"

#include <algorithm>
#include <atomic>
#include <memory>
#include <string>
#include <thread>

namespace arxlift2s_ros2_control
{

/**
 * @brief LIFT2S lift-column ros2_control hardware interface.
 *
 * Wraps official arx::LiftHeadControlLoop (libarx_lift_src.so) on can5.
 *
 * Unit contract:
 * - ros2_control / URDF: **meters** (~0–0.48)
 * - SDK getHeight / Hybrid pack: **motor radians** (~0–20)
 * - Conversion: sdk_rad = meters * height_rad_per_meter (default 41.54)
 *
 * Motor modes (URDF ``lift_motor_mode`` + live ``arx_lift.motor_mode``):
 * - ``soft_p`` — Soft-P ``setHeight`` / ``loop()``; gain ``arx_lift.soft_p_kp``
 * - ``hybrid`` — Hybrid MIT ``sendLiftHybrid``; gains ``arx_lift.hybrid_kp/kd``
 *   (kd pack-clamped ≤5)
 *
 * Exports MIX command IFs so ocs2_wbc can claim full_control; Soft-P / Hybrid
 * drive uses position (+ effort as τ feedforward add in Hybrid).
 */
class ArxLiftHardware : public hardware_interface::SystemInterface
{
public:
  enum class MotorMode : int
  {
    SoftP = 0,
    Hybrid = 1,
  };

  ArxLiftHardware() = default;
  ~ArxLiftHardware() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface::ConstSharedPtr>
  on_export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void stop_loop_thread();
  static bool parseMotorMode(const std::string & raw, MotorMode & out);
  static const char * motorModeName(MotorMode mode);
  void setupDynamicParameters(
    const std::string & initial_mode, double soft_p_kp, double hybrid_kp,
    double hybrid_kd);
  void sendHybridHoldOrTrack(double q_target_sdk, double dt_s);

  double rosToSdk(double ros_m) const
  {
    const double m = std::clamp(ros_m, 0.0, height_span_m_);
    const double sdk = m * height_rad_per_meter_;
    return std::clamp(sdk, 0.0, sdk_max_rad_);
  }
  double sdkToRos(double sdk_rad) const
  {
    if (height_rad_per_meter_ <= 0.0) {
      return 0.0;
    }
    return sdk_rad / height_rad_per_meter_;
  }

  std::string lift_joint_name_;
  double lift_position_{0.0};
  double lift_velocity_{0.0};
  double lift_effort_{0.0};
  double lift_position_command_{0.0};
  double lift_velocity_command_{0.0};
  double lift_effort_command_{0.0};
  double lift_kp_command_{0.0};
  double lift_kd_command_{0.0};

  std::string can_name_{"can5"};
  int robot_type_{0};
  double gravity_compensation_torque_{-1.8};
  double lift_max_vel_{0.20};
  double lift_max_torque_{15.0};
  double cmd_ramp_vel_mps_{0.04};

  // Separate Soft-P / Hybrid gains (live: arx_lift.soft_p_kp / hybrid_kp / hybrid_kd).
  std::atomic<double> soft_p_kp_{8.0};
  std::atomic<double> hybrid_kp_{5.0};
  std::atomic<double> hybrid_kd_{2.0};

  std::string motor_mode_param_{"soft_p"};
  std::atomic<int> motor_mode_{static_cast<int>(MotorMode::SoftP)};
  int last_applied_mode_{-1};

  double height_rad_per_meter_{41.54};
  double height_span_m_{0.48};
  double sdk_max_rad_{20.0};

  double ramp_q_sdk_{0.0};
  bool ramp_initialized_{false};

  std::shared_ptr<arx::LiftHeadControlLoop> lift_;

  std::thread loop_thread_;
  std::atomic<bool> loop_running_{false};
  std::atomic<bool> command_enabled_{false};
  std::atomic<double> last_written_height_{0.0};
  std::atomic<double> last_written_vel_{0.0};

  std::atomic<double> motor_position_{0.0};
  std::atomic<double> motor_velocity_{0.0};
  std::atomic<double> motor_torque_{0.0};
  std::atomic<double> motor_current_{0.0};
  std::atomic<int> motor_online_{0};
  std::atomic<int> motor_error_{0};
  std::atomic<double> sdk_get_height_{0.0};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace arxlift2s_ros2_control
