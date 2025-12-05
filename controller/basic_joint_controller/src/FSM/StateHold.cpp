//
// Basic Joint Controller - StateHold Implementation
//
#include "basic_joint_controller/FSM/StateHold.h"

namespace basic_joint_controller
{
    arms_controller_common::FSMStateName StateHold::checkChange()
    {
        // Check control inputs for state transition
        // Supports: HOLD -> HOME (command 1), HOLD -> MOVEJ (command 3)
        switch (ctrl_interfaces_.control_inputs_.command)
        {
        case 1:
            return arms_controller_common::FSMStateName::HOME;
        case 3:
        case 4:
            return arms_controller_common::FSMStateName::MOVEJ;
        default:
            return arms_controller_common::FSMStateName::HOLD;
        }
    }
} // namespace basic_joint_controller
