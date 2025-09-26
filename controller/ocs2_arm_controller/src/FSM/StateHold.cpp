//
// Created for OCS2 Arm Controller - StateHold
//

#include "ocs2_arm_controller/FSM/StateHold.h"
#include <rclcpp/rclcpp.hpp>

namespace ocs2::mobile_manipulator
{
    StateHold::StateHold(CtrlInterfaces& ctrl_interfaces, const std::shared_ptr<CtrlComponent>& ctrl_comp)
        : FSMState(FSMStateName::HOLD, "HOLD"),
          ctrl_interfaces_(ctrl_interfaces),
          ctrl_comp_(ctrl_comp)
    {
    }

    void StateHold::enter()
    {
        // HOLD state does not send any commands, let robot maintain current position
        // Record current position on first run (only for logging)
        if (hold_positions_.empty())
        {
            // Get current joint positions
            hold_positions_.resize(ctrl_interfaces_.joint_position_state_interface_.size());
            for (size_t i = 0; i < ctrl_interfaces_.joint_position_state_interface_.size(); ++i)
            {
                hold_positions_[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
            }

            RCLCPP_INFO(rclcpp::get_logger("StateHold"),
                        "HOLD state entered, current positions recorded for %zu joints", hold_positions_.size());
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("StateHold"), "HOLD state entered, using previously recorded positions");
        }
    }

    void StateHold::run(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
    {
        // HOLD state does not send any commands
        // Robot will maintain its current position

        // In force control mode, calculate static torques to maintain current position
        if (ctrl_interfaces_.control_mode_ == ControlMode::MIX && ctrl_comp_)
        {
            // Get current joint positions
            vector_t current_positions(ctrl_interfaces_.joint_position_state_interface_.size());
            for (size_t i = 0; i < ctrl_interfaces_.joint_position_state_interface_.size(); ++i)
            {
                current_positions(i) = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
            }

            // Calculate static torques needed to maintain current position using CtrlComponent
            vector_t static_torques = ctrl_comp_->calculateStaticTorques();

            // Set effort commands (torques) for force control
            for (size_t i = 0; i < ctrl_interfaces_.joint_force_command_interface_.size() && i < static_torques.size();
                 ++i)
            {
                ctrl_interfaces_.joint_force_command_interface_[i].get().set_value(static_torques(i));
            }

            // Set kp and kd gains for force control if available
            if (ctrl_interfaces_.default_gains_.size() >= 2)
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
        }
    }

    void StateHold::exit(){}

    FSMStateName StateHold::checkChange()
    {
        // Check control inputs for state transition
        switch (ctrl_interfaces_.control_inputs_.command)
        {
        case 3: return FSMStateName::OCS2;
        case 1: return FSMStateName::HOME;
        default: return FSMStateName::HOLD;
        }
    }
} // namespace ocs2_arm_controller
