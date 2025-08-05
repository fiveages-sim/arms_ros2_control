//
// Created for OCS2 Arm Controller - StateOCS2
//

#ifndef STATEOCS2_H
#define STATEOCS2_H

#include <memory>
#include <atomic>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <ocs2_core/Types.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include "ocs2_arm_controller/Ocs2ArmController.h"
#include "ocs2_arm_controller/control/CtrlComponent.h"

namespace ocs2::mobile_manipulator
{
    class StateOCS2 : public FSMState
    {
    public:
        StateOCS2(CtrlInterfaces& ctrl_interfaces,
                  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                  const std::shared_ptr<CtrlComponent>& ctrl_comp = nullptr);

        ~StateOCS2() override = default;

        void enter() override;
        void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        void exit() override;
        FSMStateName checkChange() override;

    private:

        // CtrlComponent reference
        std::shared_ptr<CtrlComponent> ctrl_comp_;
        
        // Control interfaces and node
        CtrlInterfaces& ctrl_interfaces_;
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        
        // State variables
        std::vector<std::string> joint_names_;
        double mpc_period_;
        rclcpp::Time last_mpc_time_;
        
        // MPC running flag
        std::atomic_bool mpc_running_{false};
    };
}

#endif // STATEOCS2_H
