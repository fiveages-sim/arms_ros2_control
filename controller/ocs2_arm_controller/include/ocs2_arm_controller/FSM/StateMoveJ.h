//
// OCS2 Arm Controller - StateMoveJ (extends arms_controller_common::StateMoveJ)
//
#pragma once

#include <arms_controller_common/FSM/StateMoveJ.h>
#include <arms_controller_common/CtrlInterfaces.h>
#include <rclcpp/rclcpp.hpp>

namespace ocs2::mobile_manipulator
{
    /**
     * @brief StateMoveJ for ocs2_arm_controller
     * 
     * Extends arms_controller_common::StateMoveJ for custom state transition logic
     */
    class StateMoveJ : public arms_controller_common::StateMoveJ
    {
    public:
        StateMoveJ(arms_controller_common::CtrlInterfaces& ctrl_interfaces,
                  const rclcpp::Logger& logger,
                  double duration = 3.0,
                  std::shared_ptr<arms_controller_common::GravityCompensation> gravity_compensation = nullptr)
            : arms_controller_common::StateMoveJ(ctrl_interfaces, logger, duration, gravity_compensation)
        {
        }

        /**
         * @brief Override checkChange for custom state transition logic
         * @return Next state name
         */
        arms_controller_common::FSMStateName checkChange() override;
    };
} // namespace ocs2::mobile_manipulator

