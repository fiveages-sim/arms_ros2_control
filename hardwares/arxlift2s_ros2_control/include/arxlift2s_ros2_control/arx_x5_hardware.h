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
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include "app/joint_controller.h"

#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace arxlift2s_ros2_control
{

/**
 * @brief Single X5 arm SystemInterface (Stanford arx5-sdk / Arx5JointController).
 *
 * Lift2S dual-arm: instantiate twice (can1 / can3). Official InterfacesThread
 * arm path is retired from this package.
 *
 * Contract (aligned with panthera-ht / arx-ros2-control): URDF always declares
 * MIX position/velocity/effort/kp/kd; ``control_mode`` only changes ``write()``.
 *
 * Modes (URDF + live ROS param ``control_mode``, same rqt path as gains):
 * - ``full_control`` — MIX pos+vel+effort (OCS2)
 * - ``position`` — position only (vel=0); by default torque/effort=0 (strict reference).
 *   Optionally, set ``position_forward_effort=true`` to forward OCS2 effort as torque feedforward.
 *
 * MIT kp/kd from live ``joint_k_gains`` / ``joint_d_gains`` on this HI node
 * (xacro defaults at start; rqt / ``ros2 param set`` hot-tune).
 *
 * Params: robot_model, can_interface (alias: can_name), control_mode,
 * joint_k_gains, joint_d_gains, gripper_kp, gripper_kd.
 */
class ArxX5Hardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArxX5Hardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface::ConstSharedPtr>
  on_export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::optional<rclcpp::Logger> logger_;
  rclcpp::Logger get_logger() const { return logger_.value(); }

  std::shared_ptr<arx::Arx5JointController> controller_;
  bool hardware_connected_{false};
  bool control_active_{false};

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
  param_callback_handle_;

  std::vector<double> joint_k_gains_;
  std::vector<double> joint_d_gains_;

  // Two gain sets kept in memory; active gains follow control_mode_.
  std::vector<double> joint_k_gains_full_control_;
  std::vector<double> joint_d_gains_full_control_;
  std::vector<double> joint_k_gains_position_;
  std::vector<double> joint_d_gains_position_;

  mutable std::mutex gains_mutex_;
  double gripper_kp_{5.0};
  double gripper_kd_{0.2};

  std::vector<double> last_applied_kp_;
  std::vector<double> last_applied_kd_;
  double last_applied_gripper_kp_{-1.0};
  double last_applied_gripper_kd_{-1.0};

  std::string robot_model_;
  std::string can_interface_;
  std::string control_mode_{"full_control"};

  size_t joint_count_{0};
  std::vector<std::string> joint_names_;

  std::vector<double> position_states_;
  std::vector<double> velocity_states_;
  std::vector<double> effort_states_;
  std::vector<double> position_commands_;
  std::vector<double> velocity_commands_;
  std::vector<double> effort_commands_;
  std::vector<double> kp_commands_;
  std::vector<double> kd_commands_;

  bool has_gripper_{false};
  std::vector<std::string> gripper_joint_names_;
  std::vector<double> gripper_position_states_;
  std::vector<double> gripper_velocity_states_;
  std::vector<double> gripper_effort_states_;
  std::vector<double> gripper_position_commands_;

  std::optional<arx::JointState> cmd_buffer_;

  template<typename T>
  T get_node_param(const std::string & name, const T & default_val)
  {
    if (!node_->has_parameter(name)) {
      node_->declare_parameter<T>(name, default_val);
    }
    return node_->get_parameter(name).template get_value<T>();
  }

  void declare_node_parameters();
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & params);
  void applyGains(
    const std::vector<double> & kp, const std::vector<double> & kd,
    double gripper_kp, double gripper_kd, bool force = false);

  /** Normalize raw mode string; returns empty if unsupported. */
  static std::string normalizeControlMode(const std::string & raw);
  bool isFullControl() const;

  /** Periodic status / gain logs (live: ``status_debug`` on arm HI node). */
  std::atomic<bool> status_debug_{false};

  /**
   * If true, in ``position`` mode we forward OCS2 effort as torque feedforward.
   * If false, we match arx-ros2-control / panthera-ht "position" semantics (torque=0).
   */
  std::atomic<bool> position_forward_effort_{false};
};

}  // namespace arxlift2s_ros2_control
