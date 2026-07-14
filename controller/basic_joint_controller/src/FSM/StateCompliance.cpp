//
// Basic Joint Controller - StateCompliance Implementation
//
#include "basic_joint_controller/FSM/StateCompliance.h"

namespace basic_joint_controller
{
    arms_controller_common::FSMStateName StateCompliance::checkChange()
    {
        // COMPLIANCE -> HOLD on command 2; stays in COMPLIANCE otherwise.
        // basic_joint_controller has no OCS2 state, so command 3 is ignored here.
        switch (ctrl_interfaces_.fsm_command_)
        {
        case 2:
            return arms_controller_common::FSMStateName::HOLD;
        default:
            return arms_controller_common::FSMStateName::COMPLIANCE;
        }
    }
} // namespace basic_joint_controller
