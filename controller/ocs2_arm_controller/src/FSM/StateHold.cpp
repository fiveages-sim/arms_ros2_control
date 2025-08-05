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
        positions_recorded_ = false;
    }

    void StateHold::run(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        // HOLD状态不发送任何指令，让机器人保持当前位置
        // 第一次运行时记录当前位置（仅用于日志）
        if (!positions_recorded_)
        {
            hold_positions_.clear();
            hold_positions_.reserve(ctrl_interfaces_.joint_position_state_interface_.size());

            for (const auto& interface : ctrl_interfaces_.joint_position_state_interface_)
            {
                hold_positions_.push_back(interface.get().get_value());
            }

            positions_recorded_ = true;
            RCLCPP_INFO(rclcpp::get_logger("StateHold"), "HOLD state active - no commands sent, maintaining current position for %zu joints",
                        hold_positions_.size());
        }
    }

    void StateHold::exit()
    {
        RCLCPP_INFO(rclcpp::get_logger("StateHold"), "Exiting HOLD state");
        positions_recorded_ = false;
    }

    FSMStateName StateHold::checkChange()
    {
        // 检查控制输入进行状态切换
        switch (ctrl_interfaces_.control_inputs_.command)
        {
        case 1: return FSMStateName::HOME;
        case 3: return FSMStateName::OCS2;
        default: return FSMStateName::HOLD;
        }
    }
} // namespace ocs2_arm_controller
