//
// OCS2 Arm Controller - StateHold Implementation
//
#include "ocs2_arm_controller/FSM/StateHold.h"

namespace ocs2::mobile_manipulator
{
    arms_controller_common::FSMStateName StateHold::checkChange()
    {
        // Check FSM command for state transition
        // Supports: HOLD -> HOME (1), HOLD -> OCS2 (3), HOLD -> MOVEJ (4), HOLD -> COMPLIANCE (5)
        switch (ctrl_interfaces_.fsm_command_)
        {
        case 1:
            return arms_controller_common::FSMStateName::HOME;
        case 3:
            return arms_controller_common::FSMStateName::OCS2;
        case 4:
            return arms_controller_common::FSMStateName::MOVEJ;
        case 5:
            return arms_controller_common::FSMStateName::COMPLIANCE;
        default:
            return arms_controller_common::FSMStateName::HOLD;
        }
    }
} // namespace ocs2::mobile_manipulator
