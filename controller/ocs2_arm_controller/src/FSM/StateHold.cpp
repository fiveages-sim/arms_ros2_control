//
// Created for OCS2 Arm Controller - StateHold
//

#include "ocs2_arm_controller/FSM/StateHold.h"
#include <rclcpp/rclcpp.hpp>

namespace ocs2::mobile_manipulator
{
    StateHold::StateHold(CtrlInterfaces& ctrl_interfaces)
        : FSMState(FSMStateName::HOLD, "HOLD"),
          ctrl_interfaces_(ctrl_interfaces)
    {
    }

    void StateHold::enter()
    {
        RCLCPP_INFO(rclcpp::get_logger("StateHold"), "Entering HOLD state");
        
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
    }

    void StateHold::exit()
    {
        RCLCPP_INFO(rclcpp::get_logger("StateHold"), "Exiting HOLD state");
    }

    FSMStateName StateHold::checkChange()
    {
        // Check control inputs for state transition
        switch (ctrl_interfaces_.control_inputs_.command)
        {
        case 1: return FSMStateName::OCS2;
        case 3: return FSMStateName::HOME;
        default: return FSMStateName::HOLD;
        }
    }
} // namespace ocs2_arm_controller
