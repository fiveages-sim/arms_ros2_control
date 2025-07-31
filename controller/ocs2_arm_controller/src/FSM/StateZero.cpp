//
// Created for OCS2 Arm Controller - StateZero
//

#include "ocs2_arm_controller/FSM/StateZero.h"
#include <rclcpp/rclcpp.hpp>

namespace ocs2_arm_controller {

StateZero::StateZero(CtrlInterfaces& ctrl_interfaces, const std::vector<double>& target_pos)
    : FSMState(FSMStateName::ZERO, "zero"), 
      ctrl_interfaces_(ctrl_interfaces), 
      target_pos_(target_pos) {
}

void StateZero::enter() {
    RCLCPP_INFO(rclcpp::get_logger("StateZero"), "Entering ZERO state");
}

void StateZero::run(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    // Move arm to zero position using position control
    // Simple position control: set target position directly
    for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size() && i < target_pos_.size(); ++i) {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(target_pos_[i]);
    }
}

void StateZero::exit() {
    RCLCPP_INFO(rclcpp::get_logger("StateZero"), "Exiting ZERO state");
}

FSMStateName StateZero::checkChange() {
    // Check control input to determine state transition
    // Allow transition to HOME (command 1)
    if (ctrl_interfaces_.control_inputs_.command == 2) {
        return FSMStateName::HOME;
    }
    // Stay in ZERO state for any other command
    return FSMStateName::ZERO;
}

} // namespace ocs2_arm_controller 