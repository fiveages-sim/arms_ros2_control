//
// Created for Basic Joint Controller - StateHold
//

#include "basic_joint_controller/FSM/StateHold.h"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <tuple>

namespace basic_joint_controller
{
    StateHold::StateHold(CtrlInterfaces& ctrl_interfaces, const rclcpp::Logger& logger, const double position_threshold)
        : FSMState(FSMStateName::HOLD, "HOLD"),
          ctrl_interfaces_(ctrl_interfaces),
          logger_(logger),
          joint_position_threshold_(position_threshold)
    {
        RCLCPP_INFO(logger_,
                     "StateHold initialized with position threshold: %.4f rad (%.2f degrees)",
                     joint_position_threshold_, joint_position_threshold_ * 180.0 / M_PI);
    }

    void StateHold::enter()
    {
        // HOLD state maintains current position
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
                        "HOLD state entered, current positions recorded for %zu joints", hold_positions_.size());
        }
        else
        {
            // Check if current joint positions are within threshold of recorded positions
            bool within_threshold = true;
            double max_diff = 0.0;

            for (size_t i = 0; i < ctrl_interfaces_.joint_position_state_interface_.size() && i < hold_positions_.size()
                 ; ++i)
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
            return FSMStateName::MOVE;
        default:
            return FSMStateName::HOLD;
        }
    }
} // namespace basic_joint_controller
