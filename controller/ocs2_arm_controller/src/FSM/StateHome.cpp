//
// Created for OCS2 Arm Controller - StateHome
//

#include "ocs2_arm_controller/FSM/StateHome.h"
#include <rclcpp/rclcpp.hpp>

namespace ocs2::mobile_manipulator
{
    StateHome::StateHome(CtrlInterfaces& ctrl_interfaces, const std::vector<double>& target_pos)
        : FSMState(FSMStateName::HOME, "home"),
          ctrl_interfaces_(ctrl_interfaces),
          target_pos_(target_pos)
    {
    }

    void StateHome::enter()
    {
        RCLCPP_INFO(rclcpp::get_logger("StateHome"), "Entering HOME state");
    }

    void StateHome::run(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        // Move arm to home position using position control
        // Simple position control: set target position directly
        for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size() && i < target_pos_.size(); ++i)
        {
            ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(target_pos_[i]);
        }
    }

    void StateHome::exit()
    {
        RCLCPP_INFO(rclcpp::get_logger("StateHome"), "Exiting HOME state");
    }

    FSMStateName StateHome::checkChange()
    {
        // Check control input to determine state transition
        // Allow transition to ZERO (command 2)
        switch (ctrl_interfaces_.control_inputs_.command)
        {
        case 1:
            return FSMStateName::ZERO;
        case 3:
            return FSMStateName::OCS2;
        default:
            return FSMStateName::HOME;
        }
    }
} // namespace ocs2_arm_controller
