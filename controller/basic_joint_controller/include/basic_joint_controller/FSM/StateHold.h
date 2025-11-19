//
// Created for Basic Joint Controller - StateHold
//
#pragma once

#include <vector>
#include "basic_joint_controller/BasicJointController.h"

namespace basic_joint_controller
{
    class StateHold : public FSMState
    {
    public:
        StateHold(CtrlInterfaces& ctrl_interfaces, const rclcpp::Logger& logger, double position_threshold = 0.1);

        void enter() override;
        void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        void exit() override;
        FSMStateName checkChange() override;

    private:
        CtrlInterfaces& ctrl_interfaces_;
        rclcpp::Logger logger_;           // Logger from controller
        std::vector<double> hold_positions_; // Positions recorded when entering
        double joint_position_threshold_; // Threshold for joint position difference check (in radians)
    };
} // namespace basic_joint_controller

