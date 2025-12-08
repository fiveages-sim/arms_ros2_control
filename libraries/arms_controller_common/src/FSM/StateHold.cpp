//
// Common StateHold Implementation
//
#include "arms_controller_common/FSM/StateHold.h"
#include <cmath>

namespace arms_controller_common
{
    StateHold::StateHold(CtrlInterfaces& ctrl_interfaces,
                        const rclcpp::Logger& logger,
                        double position_threshold,
                        std::shared_ptr<GravityCompensation> gravity_compensation)
        : FSMState(FSMStateName::HOLD, "HOLD", ctrl_interfaces),
          logger_(logger),
          gravity_compensation_(gravity_compensation),
          joint_position_threshold_(position_threshold)
    {
        RCLCPP_INFO(logger_,
                   "StateHold initialized with position threshold: %.4f rad (%.2f degrees)",
                   joint_position_threshold_, joint_position_threshold_ * 180.0 / M_PI);
    }

    void StateHold::enter()
    {
        // Record current position on first run or if position difference exceeds threshold
        if (hold_positions_.empty())
        {
            // Get current joint positions
            hold_positions_.resize(ctrl_interfaces_.joint_position_state_interface_.size());
            for (size_t i = 0; i < ctrl_interfaces_.joint_position_state_interface_.size(); ++i)
            {
                auto value = ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional();
                hold_positions_[i] = value.value_or(0.0);
            }

            RCLCPP_INFO(logger_,
                      "HOLD state entered, current positions recorded for %zu joints", 
                      hold_positions_.size());
        }
        else
        {
            // Check if current joint positions are within threshold of recorded positions
            bool within_threshold = true;
            double max_diff = 0.0;

            for (size_t i = 0; i < ctrl_interfaces_.joint_position_state_interface_.size() && 
                 i < hold_positions_.size(); ++i)
            {
                auto value = ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional();
                double current_pos = value.value_or(0.0);
                double diff = std::abs(current_pos - hold_positions_[i]);

                if (diff > max_diff)
                {
                    max_diff = diff;
                }

                if (diff > joint_position_threshold_)
                {
                    within_threshold = false;
                }
            }

            if (within_threshold)
            {
                RCLCPP_INFO(logger_,
                           "HOLD state entered, using previously recorded positions (max diff: %.4f rad < %.4f rad)",
                           max_diff, joint_position_threshold_);
            }
            else
            {
                // Update hold positions to current positions when difference exceeds threshold
                size_t num_joints = ctrl_interfaces_.joint_position_state_interface_.size();
                hold_positions_.resize(num_joints);
                for (size_t i = 0; i < num_joints; ++i)
                {
                    auto value = ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional();
                    hold_positions_[i] = value.value_or(0.0);
                }

                RCLCPP_WARN(logger_,
                           "HOLD state entered, position difference (max: %.4f rad) exceeds threshold (%.4f rad). "
                           "Updating hold positions to current positions for safety.",
                           max_diff, joint_position_threshold_);
            }
        }
    }

    void StateHold::run(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
    {
        // HOLD state maintains the recorded positions
        // Set position commands to hold positions
        for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size() &&
             i < hold_positions_.size(); ++i)
        {
            std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(hold_positions_[i]);
        }

        // In force control mode, calculate static torques
        if (ctrl_interfaces_.control_mode_ == ControlMode::MIX && gravity_compensation_)
        {
            // Get current joint positions
            std::vector<double> current_positions;
            for (size_t i = 0; i < ctrl_interfaces_.joint_position_state_interface_.size(); ++i)
            {
                auto value = ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional();
                current_positions.push_back(value.value_or(0.0));
            }

            // Calculate static torques
            std::vector<double> static_torques = 
                gravity_compensation_->calculateStaticTorques(current_positions);

            // Set effort commands
            for (size_t i = 0; i < ctrl_interfaces_.joint_force_command_interface_.size() && 
                 i < static_torques.size(); ++i)
            {
                std::ignore = ctrl_interfaces_.joint_force_command_interface_[i].get().set_value(static_torques[i]);
            }

            // Set kp and kd gains for force control if available
            if (ctrl_interfaces_.default_gains_.size() >= 2)
            {
                double kp = ctrl_interfaces_.default_gains_[0];
                double kd = ctrl_interfaces_.default_gains_[1];

                for (auto& kp_interface : ctrl_interfaces_.joint_kp_command_interface_)
                {
                    std::ignore = kp_interface.get().set_value(kp);
                }

                for (auto& kd_interface : ctrl_interfaces_.joint_kd_command_interface_)
                {
                    std::ignore = kd_interface.get().set_value(kd);
                }
            }
        }
    }

    void StateHold::exit()
    {
        // Nothing to do on exit
    }

    FSMStateName StateHold::checkChange()
    {
        // Check control inputs for state transition
        switch (ctrl_interfaces_.control_inputs_.command)
        {
        case 1:
            return FSMStateName::HOME;
        case 3:
            // Could be MOVE or OCS2 depending on controller
            return FSMStateName::INVALID;  // Let derived classes handle this
        default:
            return FSMStateName::HOLD;
        }
    }
} // namespace arms_controller_common

