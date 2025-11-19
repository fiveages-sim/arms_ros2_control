//
// Created for OCS2 Arm Controller - StateHome
//

#include "ocs2_arm_controller/FSM/StateHome.h"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

namespace ocs2::mobile_manipulator
{
    StateHome::StateHome(CtrlInterfaces& ctrl_interfaces, const std::vector<double>& target_pos,
                         const std::shared_ptr<CtrlComponent>& ctrl_comp, double duration)
        : FSMState(FSMStateName::HOME, "home"),
          ctrl_interfaces_(ctrl_interfaces),
          ctrl_comp_(ctrl_comp),
          target_pos_(target_pos),
          duration_(duration), // Interpolation duration in seconds
          percent_(0.0), // Start from 0%
          has_rest_pose_(false),
          is_rest_pose_(false)
    {
        current_target_ = target_pos_;
    }

    void StateHome::enter()
    {
        // Always start with home pose when entering the state
        is_rest_pose_ = false;
        current_target_ = target_pos_;

        // Get current joint positions as starting positions
        start_pos_.clear();
        for (auto i : ctrl_interfaces_.joint_position_state_interface_)
        {
            start_pos_.push_back(i.get().get_value());
        }

        // Reset interpolation progress
        percent_ = 0.0;

        // Set kp and kd gains for force control if available (only once when entering state)
        if (ctrl_interfaces_.control_mode_ == ControlMode::MIX && ctrl_interfaces_.default_gains_.size() >= 2)
        {
            double kp = ctrl_interfaces_.default_gains_[0]; // Position gain
            double kd = ctrl_interfaces_.default_gains_[1]; // Velocity gain

            // Set kp gains
            for (auto& kp_interface : ctrl_interfaces_.joint_kp_command_interface_)
            {
                kp_interface.get().set_value(kp);
            }

            // Set kd gains
            for (auto& kd_interface : ctrl_interfaces_.joint_kd_command_interface_)
            {
                kd_interface.get().set_value(kd);
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("StateHome"),
                    "Starting linear interpolation to home pose over %.1f seconds", duration_);
    }

    void StateHome::run(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        // Check for pose switching command with debouncing
        bool current_switch_command = (ctrl_interfaces_.control_inputs_.command == 4);

        // Detect rising edge (command just became 4)
        if (current_switch_command && !last_switch_command_ && !switch_debounced_)
        {
            switchPose();
            switch_debounced_ = true; // Prevent further triggers
        }

        // Reset debounce when command is no longer 4
        if (!current_switch_command)
        {
            switch_debounced_ = false;
        }

        // Update last command state
        last_switch_command_ = current_switch_command;

        // Update interpolation progress based on actual controller frequency
        double controller_frequency = ctrl_interfaces_.frequency_;
        percent_ += 1.0 / (duration_ * controller_frequency);

        // Calculate interpolation phase using tanh for smooth transition
        double phase = std::tanh(percent_);

        // Apply interpolated position to joints
        for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size() &&
             i < current_target_.size() && i < start_pos_.size(); ++i)
        {
            double interpolated_value = phase * current_target_[i] + (1.0 - phase) * start_pos_[i];
            ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(interpolated_value);
        }

        // In force control mode, calculate static torques for interpolated position
        if (ctrl_interfaces_.control_mode_ == ControlMode::MIX && ctrl_comp_)
        {
            // Get interpolated joint positions
            vector_t interpolated_positions(ctrl_interfaces_.joint_position_command_interface_.size());
            for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size(); ++i)
            {
                interpolated_positions(i) = ctrl_interfaces_.joint_position_command_interface_[i].get().get_value();
            }

            // Calculate static torques for current position using CtrlComponent
            vector_t static_torques = ctrl_comp_->calculateStaticTorques();

            // Set effort commands (torques) for force control
            for (size_t i = 0; i < ctrl_interfaces_.joint_force_command_interface_.size() && i < static_torques.size();
                 ++i)
            {
                ctrl_interfaces_.joint_force_command_interface_[i].get().set_value(static_torques(i));
            }
        }
    }

    void StateHome::exit(){}

    FSMStateName StateHome::checkChange()
    {
        // Check control input to determine state transition
        switch (ctrl_interfaces_.control_inputs_.command)
        {
        case 2:
            return FSMStateName::HOLD;
        default:
            return FSMStateName::HOME;
        }
    }

    void StateHome::switchPose()
    {
        // Only allow switching if rest pose is configured
        if (!has_rest_pose_)
        {
            RCLCPP_WARN(rclcpp::get_logger("StateHome"),
                        "Cannot switch pose: rest pose not configured");
            return;
        }

        // Toggle between home and rest pose
        is_rest_pose_ = !is_rest_pose_;

        // Update current target
        current_target_ = is_rest_pose_ ? rest_pos_ : target_pos_;

        // Start new interpolation
        startInterpolation();

        RCLCPP_INFO(rclcpp::get_logger("StateHome"),
                    "Switching to %s pose", is_rest_pose_ ? "rest" : "home");
    }

    void StateHome::startInterpolation()
    {
        // Get current joint positions as new starting positions
        start_pos_.clear();
        for (auto i : ctrl_interfaces_.joint_position_state_interface_)
        {
            start_pos_.push_back(i.get().get_value());
        }

        // Reset interpolation progress
        percent_ = 0.0;

        RCLCPP_INFO(rclcpp::get_logger("StateHome"),
                    "Starting interpolation to %s pose over %.1f seconds",
                    is_rest_pose_ ? "rest" : "home", duration_);
    }

    void StateHome::setRestPose(const std::vector<double>& rest_pos)
    {
        rest_pos_ = rest_pos;
        has_rest_pose_ = true;

        RCLCPP_INFO(rclcpp::get_logger("StateHome"),
                    "Rest pose configured with %zu joints", rest_pos_.size());
    }
} // namespace ocs2::mobile_manipulator
