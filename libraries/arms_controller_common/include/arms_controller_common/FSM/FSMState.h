//
// Common FSM State Base Class for Arm Controllers
//
#pragma once

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "arms_controller_common/CtrlInterfaces.h"

namespace arms_controller_common
{
    // FSM State names enum
    enum class FSMStateName
    {
        INVALID,
        HOME,
        HOLD,
        MOVEJ,  // Move Joint - for basic_joint_controller
        OCS2    // For ocs2_arm_controller
    };

    // FSM Mode enum
    enum class FSMMode
    {
        NORMAL,
        CHANGE
    };

    // Base FSM State class
    class FSMState
    {
    public:
        virtual ~FSMState() = default;

        FSMState(FSMStateName state_name, std::string state_name_string, CtrlInterfaces& ctrl_interfaces)
            : state_name(state_name), 
              state_name_string(std::move(state_name_string)),
              ctrl_interfaces_(ctrl_interfaces)
        {
        }

        virtual void enter() = 0;
        virtual void run(const rclcpp::Time& time, const rclcpp::Duration& period) = 0;
        virtual void exit() = 0;
        virtual FSMStateName checkChange() { return FSMStateName::INVALID; }

        FSMStateName state_name;
        std::string state_name_string;

    protected:
        CtrlInterfaces& ctrl_interfaces_;
    };
} // namespace arms_controller_common

