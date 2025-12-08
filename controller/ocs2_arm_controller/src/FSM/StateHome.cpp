//
// OCS2 Arm Controller - StateHome Implementation
//
#include "ocs2_arm_controller/FSM/StateHome.h"

namespace ocs2::mobile_manipulator
{
    arms_controller_common::FSMStateName StateHome::checkChange()
    {
        // Check control inputs for state transition
        // Supports: HOME -> HOLD (command 2)
        switch (ctrl_interfaces_.control_inputs_.command)
        {
        case 2:
            return arms_controller_common::FSMStateName::HOLD;
        default:
            return arms_controller_common::FSMStateName::HOME;
        }
    }
} // namespace ocs2::mobile_manipulator
