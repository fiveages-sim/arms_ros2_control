//
// Created for OCS2 Arm Controller - StateOCS2
//

#ifndef STATEOCS2_H
#define STATEOCS2_H

#include <memory>
#include <atomic>
#include <string>
#include <vector>
#include <thread>

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

        ~StateOCS2() override;

        void enter() override;
        void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        void exit() override;
        FSMStateName checkChange() override;

    private:
        // MPC update thread function
        void mpcUpdateThread();
        
        // Stop MPC update thread
        void stopMpcThread();

        // CtrlComponent reference
        std::shared_ptr<CtrlComponent> ctrl_comp_;
        
        // Control interfaces and node
        CtrlInterfaces& ctrl_interfaces_;
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        
        // State variables
        std::vector<std::string> joint_names_;
        double mpc_period_;
        rclcpp::Time last_mpc_time_;
        int thread_sleep_duration_ms_; // Thread sleep duration in milliseconds
        
        // MPC thread related
        std::thread mpc_thread_;
        std::atomic_bool mpc_running_{false};
        std::atomic_bool mpc_thread_should_stop_{false};
        std::atomic_bool mpc_update_requested_{false};

        // Collision detection
        bool collision_detected_{false};  // Flag indicating collision was detected
    };
}

#endif // STATEOCS2_H
