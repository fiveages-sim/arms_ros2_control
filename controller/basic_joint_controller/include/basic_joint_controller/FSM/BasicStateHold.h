//
// Basic Joint Controller - StateHold (extends arms_controller_common::StateHold)
//
#pragma once

#include <arms_controller_common/FSM/StateHold.h>
#include <arms_controller_common/CtrlInterfaces.h>
#include <rclcpp/rclcpp.hpp>

namespace basic_joint_controller
{
    /**
     * @brief StateHold for basic_joint_controller
     * 
     * Extends arms_controller_common::StateHold to support MOVE state transition
     */
    class BasicStateHold : public arms_controller_common::StateHold
    {
    public:
        BasicStateHold(arms_controller_common::CtrlInterfaces& ctrl_interfaces,
                      const rclcpp::Logger& logger,
                      double position_threshold = 0.1,
                      std::shared_ptr<arms_controller_common::GravityCompensation> gravity_compensation = nullptr)
            : arms_controller_common::StateHold(ctrl_interfaces, logger, position_threshold, gravity_compensation)
        {
        }

        /**
         * @brief Override checkChange to support MOVE state transition
         * @return Next state name
         */
        arms_controller_common::FSMStateName checkChange() override;
    };
} // namespace basic_joint_controller

