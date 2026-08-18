//
// Basic Joint Controller - StateMoveJ Implementation
//
#include "basic_joint_controller/FSM/StateMoveJ.h"

namespace basic_joint_controller
{
    arms_controller_common::FSMStateName StateMoveJ::checkChange()
    {
        // MOVEJ 只能回到 HOLD；HOME 必须经 HOLD，与 FSMStateTransitionValidator 一致
        switch (ctrl_interfaces_.fsm_command_)
        {
        case 2:
            return arms_controller_common::FSMStateName::HOLD;
        default:
            return arms_controller_common::FSMStateName::MOVEJ;
        }
    }
} // namespace basic_joint_controller

