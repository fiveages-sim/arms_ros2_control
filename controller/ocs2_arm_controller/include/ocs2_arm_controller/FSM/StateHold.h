//
// OCS2 Arm Controller - StateHold (extends arms_controller_common::StateHold)
//
#pragma once

#include <arms_controller_common/FSM/StateHold.h>
#include <arms_controller_common/CtrlInterfaces.h>
#include <rclcpp/rclcpp.hpp>

namespace ocs2::mobile_manipulator
{
    /**
     * @brief StateHold for ocs2_arm_controller
     * 
     * Extends arms_controller_common::StateHold for custom state transition logic
     */
    class StateHold : public arms_controller_common::StateHold
    {
    public:
        StateHold(arms_controller_common::CtrlInterfaces& ctrl_interfaces,
                 const rclcpp::Logger& logger,
                 double position_threshold = 0.1,
                 std::shared_ptr<arms_controller_common::GravityCompensation> gravity_compensation = nullptr)
            : arms_controller_common::StateHold(ctrl_interfaces, logger, position_threshold, gravity_compensation)
        {
        }

        /**
         * @brief Override checkChange for custom state transition logic
         * @return Next state name
         */
        arms_controller_common::FSMStateName checkChange() override;
    };
} // namespace ocs2::mobile_manipulator
