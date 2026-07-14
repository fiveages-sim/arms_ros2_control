//
// OCS2 Arm Controller - StateCompliance Implementation
//
#include "ocs2_arm_controller/FSM/StateCompliance.h"

namespace ocs2::mobile_manipulator
{
    arms_controller_common::FSMStateName StateCompliance::checkChange()
    {
        // COMPLIANCE -> HOLD on command 2; stays in COMPLIANCE otherwise.
        switch (ctrl_interfaces_.fsm_command_)
        {
        case 2:
            return arms_controller_common::FSMStateName::HOLD;
        default:
            return arms_controller_common::FSMStateName::COMPLIANCE;
        }
    }
} // namespace ocs2::mobile_manipulator
