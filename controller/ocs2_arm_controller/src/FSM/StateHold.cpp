//
// Created for OCS2 Arm Controller - StateHold
//

#include "ocs2_arm_controller/FSM/StateHold.h"
#include <rclcpp/rclcpp.hpp>

namespace ocs2::mobile_manipulator {

StateHold::StateHold(CtrlInterfaces& ctrl_interfaces)
    : FSMState(FSMStateName::HOLD, "HOLD"), 
      ctrl_interfaces_(ctrl_interfaces) {
}

void StateHold::enter() {
    RCLCPP_INFO(rclcpp::get_logger("StateHold"), "Entering HOLD state");
    positions_recorded_ = false;
}

void StateHold::run(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
    // 第一次运行时记录当前位置
    if (!positions_recorded_) {
        hold_positions_.clear();
        hold_positions_.reserve(ctrl_interfaces_.joint_position_state_interface_.size());
        
        for (const auto& interface : ctrl_interfaces_.joint_position_state_interface_) {
            hold_positions_.push_back(interface.get().get_value());
        }
        
        positions_recorded_ = true;
        RCLCPP_INFO(rclcpp::get_logger("StateHold"), "Recorded hold positions for %zu joints", hold_positions_.size());
    }
    
    // 保持记录的位置
    for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size() && i < hold_positions_.size(); ++i) {
        ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(hold_positions_[i]);
    }
}

void StateHold::exit() {
    RCLCPP_INFO(rclcpp::get_logger("StateHold"), "Exiting HOLD state");
    positions_recorded_ = false;
}

FSMStateName StateHold::checkChange() {
    // 检查控制输入进行状态切换
    switch (ctrl_interfaces_.control_inputs_.command) {
        case 1: return FSMStateName::HOME;
        case 2: return FSMStateName::ZERO;
        case 4: return FSMStateName::OCS2;
        default: return FSMStateName::HOLD;
    }
}

} // namespace ocs2_arm_controller 