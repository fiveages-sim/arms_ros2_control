//
// Common StateMoveJ Implementation
//
#include "arms_controller_common/FSM/StateMoveJ.h"
#include <cmath>

namespace arms_controller_common
{
    StateMoveJ::StateMoveJ(CtrlInterfaces& ctrl_interfaces,
                          const rclcpp::Logger& logger,
                          double duration,
                          std::shared_ptr<GravityCompensation> gravity_compensation)
        : FSMState(FSMStateName::MOVEJ, "movej", ctrl_interfaces),
          logger_(logger),
          gravity_compensation_(gravity_compensation),
          duration_(duration)
    {
    }

    void StateMoveJ::enter()
    {
        // Get current joint positions as starting positions
        start_pos_.clear();
        for (auto i : ctrl_interfaces_.joint_position_state_interface_)
        {
            auto value = i.get().get_optional();
            start_pos_.push_back(value.value_or(0.0));
        }

        // Reset interpolation progress
        percent_ = 0.0;
        interpolation_active_ = false;

        // Check if we have a target position
        std::lock_guard lock(target_mutex_);
        if (has_target_ && target_pos_.size() == start_pos_.size())
        {
            interpolation_active_ = true;
            RCLCPP_INFO(logger_,
                       "Starting linear interpolation to target position over %.1f seconds", duration_);
        }
        else
        {
            RCLCPP_WARN(logger_,
                       "No target position set, waiting for target position...");
        }
    }

    void StateMoveJ::run(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        std::lock_guard lock(target_mutex_);

        // If no target or target size mismatch, maintain current position
        if (!has_target_ || target_pos_.size() != start_pos_.size())
        {
            // Maintain current position
            for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size() && 
                 i < start_pos_.size(); ++i)
            {
                std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(start_pos_[i]);
            }
            return;
        }

        // If interpolation just started, update start position to current position
        if (!interpolation_active_)
        {
            start_pos_.clear();
            for (auto i : ctrl_interfaces_.joint_position_state_interface_)
            {
                auto value = i.get().get_optional();
                start_pos_.push_back(value.value_or(0.0));
            }
            percent_ = 0.0;
            interpolation_active_ = true;
            RCLCPP_INFO(logger_,
                       "Target position received, starting interpolation from current position");
        }

        // Update interpolation progress based on actual controller frequency
        double controller_frequency = ctrl_interfaces_.frequency_;
        percent_ += 1.0 / (duration_ * controller_frequency);

        // Clamp percent to [0, 1]
        if (percent_ > 1.0)
        {
            percent_ = 1.0;
        }

        // Calculate interpolation phase using tanh for smooth transition
        double phase = std::tanh(percent_ * 3.0); // Scale for faster convergence

        // Apply interpolated position to joints
        for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size() &&
             i < target_pos_.size() && i < start_pos_.size(); ++i)
        {
            double interpolated_value = phase * target_pos_[i] + (1.0 - phase) * start_pos_[i];
            std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(interpolated_value);
        }

        // In force control mode, calculate static torques
        if (ctrl_interfaces_.control_mode_ == ControlMode::MIX && gravity_compensation_)
        {
            // Get interpolated joint positions
            std::vector<double> interpolated_positions;
            for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size(); ++i)
            {
                interpolated_positions.push_back(
                    ctrl_interfaces_.joint_position_command_interface_[i].get().get_value());
            }

            // Calculate static torques
            std::vector<double> static_torques = 
                gravity_compensation_->calculateStaticTorques(interpolated_positions);

            // Set effort commands
            for (size_t i = 0; i < ctrl_interfaces_.joint_force_command_interface_.size() && 
                 i < static_torques.size(); ++i)
            {
                std::ignore = ctrl_interfaces_.joint_force_command_interface_[i].get().set_value(static_torques[i]);
            }
        }
    }

    void StateMoveJ::exit()
    {
        std::lock_guard lock(target_mutex_);
        percent_ = 0.0;
        interpolation_active_ = false;
    }

    FSMStateName StateMoveJ::checkChange()
    {
        // Check control input to determine state transition
        // Default implementation: can transition to HOME or HOLD
        // Derived classes or controllers can override this behavior
        switch (ctrl_interfaces_.control_inputs_.command)
        {
        case 1:
            return FSMStateName::HOME;
        case 2:
            return FSMStateName::HOLD;
        default:
            return FSMStateName::MOVEJ;
        }
    }

    void StateMoveJ::setTargetPosition(const std::vector<double>& target_pos)
    {
        std::lock_guard lock(target_mutex_);
        
        // Check if the new target is the same as the current target_pos_
        bool is_same_target = false;
        if (has_target_ && target_pos_.size() == target_pos.size())
        {
            is_same_target = true;
            for (size_t i = 0; i < target_pos.size(); ++i)
            {
                if (std::abs(target_pos_[i] - target_pos[i]) > TARGET_EPSILON)
                {
                    is_same_target = false;
                    break;
                }
            }
        }
        
        // If target is the same, skip re-interpolation
        if (is_same_target && interpolation_active_)
        {
            RCLCPP_DEBUG(logger_, "Received same target position, skipping re-interpolation");
            return;
        }
        
        // Update target position
        target_pos_ = target_pos;
        has_target_ = true;
        
        // Reset interpolation if we're already in the state
        if (interpolation_active_)
        {
            // Update start position to current position for new interpolation
            start_pos_.clear();
            for (auto i : ctrl_interfaces_.joint_position_state_interface_)
            {
                auto value = i.get().get_optional();
                start_pos_.push_back(value.value_or(0.0));
            }
            percent_ = 0.0;
            RCLCPP_INFO(logger_, "New target position received, restarting interpolation");
        }
    }
} // namespace arms_controller_common

