//
// OCS2 Arm Controller - StateMoveJ Implementation
//
#include "ocs2_arm_controller/FSM/StateMoveJ.h"

namespace ocs2::mobile_manipulator
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
} // namespace ocs2::mobile_manipulator

