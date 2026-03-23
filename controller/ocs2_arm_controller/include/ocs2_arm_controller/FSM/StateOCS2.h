//
// Created for OCS2 Arm Controller - StateOCS2
//

#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <arms_controller_common/FSM/FSMState.h>

#include "ocs2_arm_controller/control/CtrlComponent.h"

namespace ocs2::mobile_manipulator
{
    using FSMState = arms_controller_common::FSMState;
    using FSMStateName = arms_controller_common::FSMStateName;
    using ControlMode = arms_controller_common::ControlMode;

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
        void mpcUpdateThread();
        void stopMpcThread();

        std::shared_ptr<CtrlComponent> ctrl_comp_;
        CtrlInterfaces& ctrl_interfaces_;
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

        std::vector<std::string> joint_names_;
        double mpc_period_{};
        rclcpp::Time last_mpc_time_;
        int thread_sleep_duration_ms_{};

        std::thread mpc_thread_;
        std::atomic_bool mpc_running_{false};
        std::atomic_bool mpc_thread_should_stop_{false};
        std::atomic_bool mpc_update_requested_{false};

        bool collision_detected_{false};
    };
}
