//
// Basic Joint Controller - StateCompliance (extends arms_controller_common::StateCompliance)
//
#pragma once

#include <arms_controller_common/FSM/StateCompliance.h>
#include <arms_controller_common/CtrlInterfaces.h>
#include <rclcpp/rclcpp.hpp>

namespace basic_joint_controller
{
    /**
     * @brief StateCompliance for basic_joint_controller
     *
     * Same hybrid API as ocs2; without ArmKinematics degrades to soft joint
     * hold. checkChange aware of basic_joint_controller FSM (no OCS2).
     */
    class StateCompliance : public arms_controller_common::StateCompliance
    {
    public:
        StateCompliance(arms_controller_common::CtrlInterfaces& ctrl_interfaces,
                        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = nullptr,
                        std::shared_ptr<arms_controller_common::GravityCompensation> gravity_compensation = nullptr,
                        std::shared_ptr<arms_controller_common::ArmKinematics> kinematics = nullptr)
            : arms_controller_common::StateCompliance(ctrl_interfaces, node, gravity_compensation, kinematics)
        {
        }

        arms_controller_common::FSMStateName checkChange() override;
    };
} // namespace basic_joint_controller
