//
// OCS2 Arm Controller - StateHold Implementation
//
#include "ocs2_arm_controller/FSM/StateHold.h"

namespace ocs2::mobile_manipulator
{
    arms_controller_common::FSMStateName StateHold::checkChange()
    {
        // Check control inputs for state transition
        // Supports: HOLD -> HOME (command 1), HOLD -> OCS2 (command 3), HOLD -> MOVEJ (command 4)
        switch (ctrl_interfaces_.control_inputs_.command)
        {
        case 1:
            return arms_controller_common::FSMStateName::HOME;
        case 3:
            return arms_controller_common::FSMStateName::OCS2;
        case 4:
            return arms_controller_common::FSMStateName::MOVEJ;
        default:
            return arms_controller_common::FSMStateName::HOLD;
        }
    }
} // namespace ocs2::mobile_manipulator
