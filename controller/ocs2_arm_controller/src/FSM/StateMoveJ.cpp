//
// OCS2 Arm Controller - StateMoveJ Implementation
//
#include "ocs2_arm_controller/FSM/StateMoveJ.h"

namespace ocs2::mobile_manipulator
{
    arms_controller_common::FSMStateName StateMoveJ::checkChange()
    {
        // Check FSM command for state transition
        // Supports: MOVEJ -> HOME (command 1), MOVEJ -> HOLD (command 2)
        switch (ctrl_interfaces_.fsm_command_)
        {
        case 1:
            return arms_controller_common::FSMStateName::HOME;
        case 2:
            return arms_controller_common::FSMStateName::HOLD;
        default:
            return arms_controller_common::FSMStateName::MOVEJ;
        }
    }
} // namespace ocs2::mobile_manipulator

