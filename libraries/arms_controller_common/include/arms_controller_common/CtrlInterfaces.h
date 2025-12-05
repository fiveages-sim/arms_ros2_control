//
// Common Control Interfaces for Arm Controllers
//
#pragma once

#include <vector>
#include <string>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <arms_ros2_control_msgs/msg/inputs.hpp>

namespace arms_controller_common
{
    // Control mode enum for automatic detection
    enum class ControlMode
    {
        POSITION,    // Position control only
        MIX,         // Mixed control with kp, kd, velocity, effort, position
        AUTO         // Automatic detection based on available interfaces
    };

    // Control interfaces structure for arm control
    struct CtrlInterfaces
    {
        // Command interfaces - position control (required)
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_position_command_interface_;

        // Optional command interfaces for force/mixed control
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_force_command_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_velocity_command_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_kp_command_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_kd_command_interface_;

        // State interfaces - position and velocity (required)
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> 
        joint_position_state_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> 
        joint_velocity_state_interface_;

        // Optional state interface for force
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> 
        joint_force_state_interface_;

        // Control input
        arms_ros2_control_msgs::msg::Inputs control_inputs_;
        int frequency_ = 1000;

        // Control mode - automatically detected based on available interfaces
        ControlMode control_mode_ = ControlMode::POSITION;
        bool mode_detected_ = false;

        // Force control gains [kp, kd]
        std::vector<double> default_gains_;

        // Auto mode detection function - called once during initialization
        void detectAndSetControlMode()
        {
            if (mode_detected_) return;

            // Check if command interfaces include kp, kd, velocity, effort, position
            bool has_kp_cmd = !joint_kp_command_interface_.empty();
            bool has_kd_cmd = !joint_kd_command_interface_.empty();
            bool has_velocity_cmd = !joint_velocity_command_interface_.empty();
            bool has_effort_cmd = !joint_force_command_interface_.empty();
            bool has_position_cmd = !joint_position_command_interface_.empty();

            // Check if state interfaces include velocity, effort, position
            bool has_velocity_state = !joint_velocity_state_interface_.empty();
            bool has_effort_state = !joint_force_state_interface_.empty();
            bool has_position_state = !joint_position_state_interface_.empty();

            // Determine control mode based on available interfaces
            if (has_kp_cmd && has_kd_cmd && has_velocity_cmd && has_effort_cmd && has_position_cmd &&
                has_velocity_state && has_effort_state && has_position_state)
            {
                control_mode_ = ControlMode::MIX;
            }
            else
            {
                control_mode_ = ControlMode::POSITION;
            }

            mode_detected_ = true;
        }

        void clear()
        {
            joint_position_command_interface_.clear();
            joint_force_command_interface_.clear();
            joint_velocity_command_interface_.clear();
            joint_kp_command_interface_.clear();
            joint_kd_command_interface_.clear();
            joint_position_state_interface_.clear();
            joint_velocity_state_interface_.clear();
            joint_force_state_interface_.clear();
        }
    };
} // namespace arms_controller_common

