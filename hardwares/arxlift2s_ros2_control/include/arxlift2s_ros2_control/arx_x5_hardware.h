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

#include "arx_x5_src/interfaces/InterfacesThread.hpp"

#include <memory>
#include <string>
#include <vector>

namespace arxlift2s_ros2_control
{

/**
 * @brief Single ARX X5 arm ros2_control hardware interface.
 *
 * Wraps official arx::x5::InterfacesThread (libarx_x5_src.so).
 * For LIFT2S dual-arm, instantiate this plugin twice (can1 / can3).
 * Does not wrap the topic-based X5Controller node.
 *
 * Gripper: ROS position (m, URDF 0~0.044) <-> SDK setCatch / pos[6] (0~5).
 * setCatch is only sent when a controller writes a finite gripper command.
 */
class ArxX5Hardware : public hardware_interface::SystemInterface
{
public:
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
  // ---------- joint buffers (match info_.joints) ----------
  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_efforts_;
  std::vector<double> joint_position_commands_;

  // ---------- gripper (has_gripper_=false if URDF has no gripper joint) ----------
  bool has_gripper_{false};
  int gripper_joint_index_{-1};  // index in joint_*; SDK feedback is getJointPositons()[6]

  // ---------- config (URDF <param>) ----------
  std::string can_name_;   // can1 / can3
  std::string urdf_path_;  // required by InterfacesThread ctor
  int end_type_{0};

  // ---------- SDK (= dobot commander_) ----------
  std::shared_ptr<arx::x5::InterfacesThread> arm_;

  static constexpr size_t kArmDof = 6;
};

}  // namespace arxlift2s_ros2_control
