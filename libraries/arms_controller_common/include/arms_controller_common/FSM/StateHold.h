//
// Common StateHold for Arm Controllers
//
#pragma once

#include "arms_controller_common/FSM/FSMState.h"
#include "arms_controller_common/utils/GravityCompensation.h"
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace arms_controller_common
{
    /**
     * @brief StateHold - Holds current position
     * 
     * Supports:
     * - Position holding (sends position commands)
     * - Optional gravity compensation (if hardware supports)
     * - Automatic position threshold checking
     */
    class StateHold : public FSMState
    {
    public:
        /**
         * @brief Constructor
         * @param ctrl_interfaces Control interfaces
         * @param logger ROS logger
         * @param position_threshold Position difference threshold (radians). If <= 0, threshold checking is automatically disabled.
         * @param gravity_compensation Optional gravity compensation utility (nullptr if not needed)
         */
        StateHold(CtrlInterfaces& ctrl_interfaces,
                 const rclcpp::Logger& logger,
                 double position_threshold = 0.1,
                 std::shared_ptr<GravityCompensation> gravity_compensation = nullptr);

        void enter() override;
        void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        void exit() override;
        FSMStateName checkChange() override;

    private:
        rclcpp::Logger logger_;
        std::shared_ptr<GravityCompensation> gravity_compensation_;
        std::vector<double> hold_positions_;           // Positions recorded when entering
        double joint_position_threshold_;              // Threshold for joint position difference (radians). If <= 0, checking is disabled.
    };
} // namespace arms_controller_common

