//
// Created for Basic Joint Controller - StateMove
//
#pragma once

#include <vector>
#include <mutex>
#include "basic_joint_controller/BasicJointController.h"

namespace basic_joint_controller
{
    class StateMove : public FSMState
    {
    public:
        StateMove(CtrlInterfaces& ctrl_interfaces, const rclcpp::Logger& logger, double duration = 3.0);

        void enter() override;
        void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        void exit() override;
        FSMStateName checkChange() override;

        // Set target position (thread-safe)
        void setTargetPosition(const std::vector<double>& target_pos);

    private:
        CtrlInterfaces& ctrl_interfaces_;
        rclcpp::Logger logger_;           // Logger from controller
        double duration_;                 // Interpolation duration in seconds
        double percent_{0.0};             // Interpolation progress (0.0 to 1.0)
        
        std::vector<double> start_pos_;   // Starting position when entering state or starting new movement
        std::vector<double> target_pos_;  // Target position to move to
        
        std::mutex target_mutex_;         // Mutex for thread-safe target position updates
        bool has_target_{false};          // Whether a target position has been set
        bool interpolation_active_{false}; // Whether interpolation is currently active
    };
} // namespace basic_joint_controller

