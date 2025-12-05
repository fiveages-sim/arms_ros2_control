//
// Common StateMoveJ for Arm Controllers
//
#pragma once

#include "arms_controller_common/FSM/FSMState.h"
#include "arms_controller_common/utils/GravityCompensation.h"
#include <vector>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

namespace arms_controller_common
{
    /**
     * @brief StateMoveJ - Moves arm joints to target position with smooth interpolation
     * 
     * Supports:
     * - Smooth interpolation to target joint positions
     * - Thread-safe target position updates
     * - Optional gravity compensation (if hardware supports)
     * - Configurable interpolation duration
     */
    class StateMoveJ : public FSMState
    {
    public:
        /**
         * @brief Constructor
         * @param ctrl_interfaces Control interfaces
         * @param logger ROS logger
         * @param duration Interpolation duration in seconds
         * @param gravity_compensation Optional gravity compensation utility (nullptr if not needed)
         */
        explicit StateMoveJ(CtrlInterfaces& ctrl_interfaces,
                           const rclcpp::Logger& logger,
                           double duration = 3.0,
                           std::shared_ptr<GravityCompensation> gravity_compensation = nullptr);

        void enter() override;
        void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        void exit() override;
        FSMStateName checkChange() override;

        /**
         * @brief Set target position (thread-safe)
         * @param target_pos Target joint positions
         */
        void setTargetPosition(const std::vector<double>& target_pos);

    private:
        rclcpp::Logger logger_;
        std::shared_ptr<GravityCompensation> gravity_compensation_;
        
        double duration_;                 // Interpolation duration in seconds
        double percent_{0.0};             // Interpolation progress (0.0 to 1.0)
        
        std::vector<double> start_pos_;   // Starting position when entering state or starting new movement
        std::vector<double> target_pos_;  // Target position to move to
        
        std::mutex target_mutex_;         // Mutex for thread-safe target position updates
        bool has_target_{false};          // Whether a target position has been set
        bool interpolation_active_{false}; // Whether interpolation is currently active
        
        static constexpr double TARGET_EPSILON = 1e-6;  // Tolerance for comparing target positions
    };
} // namespace arms_controller_common

