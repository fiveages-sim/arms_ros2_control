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

#include "arxlift2s_ros2_control/arx_x5_hardware.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <pluginlib/class_list_macros.hpp>
#include <stdexcept>

namespace arxlift2s_ros2_control
{
namespace
{
bool is_gripper_joint_name(const std::string & name)
{
  std::string lower = name;
  std::transform(
    lower.begin(), lower.end(), lower.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return lower.find("gripper") != std::string::npos ||
         lower.find("hand") != std::string::npos;
}

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

// URDF gripper_joint upper=0.044 m; official setCatch 0~5 (0~80 mm).
static constexpr double kGripperUrdfMax = 0.044;
static constexpr double kCatchMax = 5.0;

double catch_to_urdf_m(const double catch_val)
{
  return catch_val / kCatchMax * kGripperUrdfMax;
}

double urdf_m_to_catch(const double urdf_m)
{
  return std::clamp(urdf_m / kGripperUrdfMax * kCatchMax, 0.0, kCatchMax);
}
}  // namespace

hardware_interface::CallbackReturn ArxX5Hardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  can_name_ = get_hw_param(info_, "can_name");
  urdf_path_ = get_hw_param(info_, "urdf_path");
  const std::string end_type_str = get_hw_param(info_, "end_type", "0");
  try {
    end_type_ = std::stoi(end_type_str);
  } catch (const std::exception &) {
    RCLCPP_ERROR(
      get_logger(), "Invalid end_type parameter: '%s'", end_type_str.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (can_name_.empty() || urdf_path_.empty()) {
    RCLCPP_ERROR(
      get_logger(),
      "Required hardware_parameters missing: can_name='%s' urdf_path='%s'",
      can_name_.c_str(), urdf_path_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.clear();
  has_gripper_ = false;
  gripper_joint_index_ = -1;
  size_t arm_joint_count = 0;

  for (const auto & joint : info_.joints) {
    const int index = static_cast<int>(joint_names_.size());
    joint_names_.push_back(joint.name);
    if (is_gripper_joint_name(joint.name)) {
      if (has_gripper_) {
        RCLCPP_ERROR(
          get_logger(),
          "Multiple gripper/hand joints found; V1 supports at most one");
        return hardware_interface::CallbackReturn::ERROR;
      }
      has_gripper_ = true;
      gripper_joint_index_ = index;
    } else {
      ++arm_joint_count;
    }
  }

  if (arm_joint_count != kArmDof) {
    RCLCPP_ERROR(
      get_logger(),
      "Expected %zu non-gripper arm joints, got %zu (total joints %zu)",
      kArmDof, arm_joint_count, joint_names_.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (has_gripper_) {
    if (
      joint_names_.size() != kArmDof + 1 || gripper_joint_index_ < 0 ||
      static_cast<size_t>(gripper_joint_index_) >= joint_names_.size())
    {
      RCLCPP_ERROR(
        get_logger(),
        "Invalid gripper layout: total=%zu gripper_index=%d",
        joint_names_.size(), gripper_joint_index_);
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else if (joint_names_.size() != kArmDof) {
    RCLCPP_ERROR(
      get_logger(), "Expected %zu joints without gripper, got %zu", kArmDof,
      joint_names_.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const size_t n = joint_names_.size();
  joint_positions_.assign(n, 0.0);
  joint_velocities_.assign(n, 0.0);
  joint_efforts_.assign(n, 0.0);
  joint_position_commands_.assign(n, 0.0);
  // Official X5Controller only setCatch on command; leave NaN until a
  // controller writes a finite gripper position (avoids drive-on-bringup).
  if (has_gripper_ && gripper_joint_index_ >= 0) {
    joint_position_commands_[static_cast<size_t>(gripper_joint_index_)] =
      std::numeric_limits<double>::quiet_NaN();
  }
  arm_.reset();

  RCLCPP_INFO(
    get_logger(),
    "ArxX5Hardware init: can=%s end_type=%d joints=%zu has_gripper=%s "
    "(gripper ROS m 0~%.3f <-> SDK catch 0~%.0f)",
    can_name_.c_str(), end_type_, n, has_gripper_ ? "true" : "false",
    kGripperUrdfMax, kCatchMax);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
ArxX5Hardware::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
  state_interfaces.reserve(joint_names_.size() * 3);

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.push_back(
      std::make_shared<hardware_interface::StateInterface>(
        joint_names_[i], hardware_interface::HW_IF_POSITION,
        &joint_positions_[i]));
    state_interfaces.push_back(
      std::make_shared<hardware_interface::StateInterface>(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY,
        &joint_velocities_[i]));
    state_interfaces.push_back(
      std::make_shared<hardware_interface::StateInterface>(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &joint_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
ArxX5Hardware::on_export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
  command_interfaces.reserve(joint_names_.size());

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.push_back(
      std::make_shared<hardware_interface::CommandInterface>(
        joint_names_[i], hardware_interface::HW_IF_POSITION,
        &joint_position_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn ArxX5Hardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    arm_ = std::make_shared<arx::x5::InterfacesThread>(
      urdf_path_, can_name_, end_type_);
    arm_->arx_x(500, 2000, 10);
    arm_->setArmStatus(arx::x5::InterfacesThread::state::POSITION_CONTROL);

    const std::vector<double> pos = arm_->getJointPositons();
    const std::vector<double> vel = arm_->getJointVelocities();
    const std::vector<double> cur = arm_->getJointCurrent();

    if (pos.size() < kArmDof) {
      RCLCPP_ERROR(
        get_logger(), "SDK returned %zu joint positions, expected >= %zu",
        pos.size(), kArmDof);
      arm_.reset();
      return hardware_interface::CallbackReturn::ERROR;
    }

    size_t arm_i = 0;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (has_gripper_ && static_cast<int>(i) == gripper_joint_index_) {
        if (pos.size() <= kArmDof) {
          RCLCPP_ERROR(get_logger(), "SDK missing gripper feedback at index 6");
          arm_.reset();
          return hardware_interface::CallbackReturn::ERROR;
        }
        joint_positions_[i] = catch_to_urdf_m(pos[kArmDof]);
        joint_velocities_[i] = (vel.size() > kArmDof) ? vel[kArmDof] : 0.0;
        joint_efforts_[i] = (cur.size() > kArmDof) ? cur[kArmDof] : 0.0;
        // Do not seed gripper command — keep NaN so write() skips setCatch
        joint_position_commands_[i] = std::numeric_limits<double>::quiet_NaN();
      } else {
        joint_positions_[i] = pos[arm_i];
        joint_velocities_[i] = (arm_i < vel.size()) ? vel[arm_i] : 0.0;
        joint_efforts_[i] = (arm_i < cur.size()) ? cur[arm_i] : 0.0;
        joint_position_commands_[i] = joint_positions_[i];
        ++arm_i;
      }
    }

    RCLCPP_INFO(get_logger(), "ArxX5Hardware activated on %s", can_name_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to activate ArxX5Hardware: %s", e.what());
    arm_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::CallbackReturn ArxX5Hardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  arm_.reset();
  RCLCPP_INFO(get_logger(), "ArxX5Hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArxX5Hardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!arm_) {
    return hardware_interface::return_type::ERROR;
  }

  try {
    const std::vector<double> pos = arm_->getJointPositons();
    const std::vector<double> vel = arm_->getJointVelocities();
    const std::vector<double> cur = arm_->getJointCurrent();

    if (pos.size() < kArmDof) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "SDK joint position size %zu < %zu", pos.size(), kArmDof);
      return hardware_interface::return_type::ERROR;
    }

    size_t arm_i = 0;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (has_gripper_ && static_cast<int>(i) == gripper_joint_index_) {
        if (pos.size() <= kArmDof) {
          return hardware_interface::return_type::ERROR;
        }
        joint_positions_[i] = catch_to_urdf_m(pos[kArmDof]);
        joint_velocities_[i] = (vel.size() > kArmDof) ? vel[kArmDof] : 0.0;
        joint_efforts_[i] = (cur.size() > kArmDof) ? cur[kArmDof] : 0.0;
      } else {
        joint_positions_[i] = pos[arm_i];
        joint_velocities_[i] = (arm_i < vel.size()) ? vel[arm_i] : 0.0;
        joint_efforts_[i] = (arm_i < cur.size()) ? cur[arm_i] : 0.0;
        ++arm_i;
      }
    }
    return hardware_interface::return_type::OK;
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "ArxX5Hardware read failed: %s",
      e.what());
    return hardware_interface::return_type::ERROR;
  }
}

hardware_interface::return_type ArxX5Hardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!arm_) {
    return hardware_interface::return_type::ERROR;
  }

  try {
    std::vector<double> arm_cmd(kArmDof, 0.0);
    size_t arm_i = 0;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (has_gripper_ && static_cast<int>(i) == gripper_joint_index_) {
        continue;
      }
      if (arm_i >= kArmDof) {
        break;
      }
      arm_cmd[arm_i++] = joint_position_commands_[i];
    }

    arm_->setJointPositions(arm_cmd);

    if (has_gripper_ && gripper_joint_index_ >= 0) {
      const double gripper_cmd =
        joint_position_commands_[static_cast<size_t>(gripper_joint_index_)];
      if (std::isfinite(gripper_cmd)) {
        arm_->setCatch(urdf_m_to_catch(gripper_cmd));
      }
    }

    return hardware_interface::return_type::OK;
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "ArxX5Hardware write failed: %s",
      e.what());
    return hardware_interface::return_type::ERROR;
  }
}

}  // namespace arxlift2s_ros2_control

PLUGINLIB_EXPORT_CLASS(
  arxlift2s_ros2_control::ArxX5Hardware, hardware_interface::SystemInterface)
