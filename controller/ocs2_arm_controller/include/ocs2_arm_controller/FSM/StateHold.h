//
// Created for OCS2 Arm Controller - StateHold
//

#ifndef STATEHOLD_H
#define STATEHOLD_H

#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ocs2_arm_controller/Ocs2ArmController.h"

namespace ocs2::mobile_manipulator
{
    class StateHold : public FSMState
    {
    public:
        StateHold(CtrlInterfaces& ctrl_interfaces);

        void enter() override;
        void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        void exit() override;
        FSMStateName checkChange() override;

    private:
        CtrlInterfaces& ctrl_interfaces_;
        std::vector<double> hold_positions_; // 进入时记录的位置
        bool positions_recorded_{false};
    };
} // namespace ocs2_arm_controller

#endif // STATEHOLD_H
