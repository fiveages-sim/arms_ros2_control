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

#include "arx_lift_src/lift_head_control_loop.h"

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
 * Exports a single lift joint (height in meters). Does not export waist,
 * head, or chassis interfaces. On activate, chassis is parked once via
 * setChassisCmd(0,0,0,2); a background thread must call loop() periodically.
 * Does not wrap the topic-based lift_controller node.
 */
class ArxLiftHardware : public hardware_interface::SystemInterface
{
public:
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
  // ---------- joint buffers (single DOF; scalars bound to export) ----------
  std::string lift_joint_name_;
  double lift_position_{0.0};
  double lift_velocity_{0.0};  // no direct SDK quantity; keep 0 until derived
  double lift_effort_{0.0};    // same
  double lift_position_command_{0.0};

  // ---------- config ----------
  std::string can_name_{"can5"};
  int robot_type_{0};  // arx::LiftHeadControlLoop::RobotType

  // ---------- SDK ----------
  std::shared_ptr<arx::LiftHeadControlLoop> lift_;

  // ---------- loop thread (SDK requires periodic loop(); decoupled from CM rate) ----------
  std::thread loop_thread_;
  std::atomic<bool> loop_running_{false};
};

}  // namespace arxlift2s_ros2_control
