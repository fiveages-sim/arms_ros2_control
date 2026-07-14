//
// OCS2 Arm Controller - StateCompliance (extends arms_controller_common::StateCompliance)
//
#pragma once

#include <memory>

#include <arms_controller_common/FSM/StateCompliance.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace ocs2::mobile_manipulator
{
    /**
     * @brief StateCompliance for ocs2_arm_controller
     *
     * Inherits common compliance behavior (force feed-forward + admittance).
     * checkChange follows ocs2_arm_controller's state graph
     * (HOLD on cmd 2; OCS2 not reachable from COMPLIANCE).
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
} // namespace ocs2::mobile_manipulator
