//
// Created for OCS2 Arm Controller - StateHold
//
#pragma once

#include <vector>
#include <memory>

#include "ocs2_arm_controller/Ocs2ArmController.h"

namespace ocs2::mobile_manipulator
{
    class StateHold : public FSMState
    {
    public:
        StateHold(CtrlInterfaces& ctrl_interfaces,
                  const std::shared_ptr<CtrlComponent>& ctrl_comp = nullptr);

        void enter() override;
        void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        void exit() override;
        FSMStateName checkChange() override;

    private:
        CtrlInterfaces& ctrl_interfaces_;
        std::shared_ptr<CtrlComponent> ctrl_comp_;  // CtrlComponent reference for torque calculation
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;  // Node reference for logging and parameters
        // State variables
        std::vector<double> hold_positions_; // Positions recorded when entering
        bool positions_recorded_{false};
        // Threshold for joint position difference check (in radians)
        double joint_position_threshold_; // Configurable threshold, default 0.1 rad (~5.7 degrees)
    };
} // namespace ocs2_arm_controller