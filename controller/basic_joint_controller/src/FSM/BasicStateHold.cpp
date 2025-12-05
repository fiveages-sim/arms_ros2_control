//
// Basic Joint Controller - StateHold Implementation
//
#include "basic_joint_controller/FSM/BasicStateHold.h"

namespace basic_joint_controller
{
    arms_controller_common::FSMStateName BasicStateHold::checkChange()
    {
        // Check control inputs for state transition
        // Supports: HOLD -> HOME (command 1), HOLD -> MOVE (command 3)
        switch (ctrl_interfaces_.control_inputs_.command)
        {
        case 1:
            return arms_controller_common::FSMStateName::HOME;
        case 3:
            return arms_controller_common::FSMStateName::MOVE;
        default:
            return arms_controller_common::FSMStateName::HOLD;
        }
    }
} // namespace basic_joint_controller

