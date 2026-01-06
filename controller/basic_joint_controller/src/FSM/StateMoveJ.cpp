//
// Basic Joint Controller - StateMoveJ Implementation
//
#include "basic_joint_controller/FSM/StateMoveJ.h"

namespace basic_joint_controller
{
    arms_controller_common::FSMStateName StateMoveJ::checkChange()
    {
        // Check FSM command for state transition
        // Supports: MOVEJ -> HOME (command 1), MOVEJ -> HOLD (command 2)
        switch (ctrl_interfaces_.fsm_command_)
        {
        case 2:
            return arms_controller_common::FSMStateName::HOLD;
        default:
            return arms_controller_common::FSMStateName::MOVEJ;
        }
    }
} // namespace basic_joint_controller

