//
// Common StateHold for Arm Controllers
//
#pragma once

#include "arms_controller_common/FSM/FSMState.h"
#include "arms_controller_common/utils/GravityCompensation.h"
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace arms_controller_common
{
    /**
     * @brief StateHold - Holds current position
     * 
     * Supports:
     * - Position holding (sends position commands)
     * - Optional gravity compensation (if hardware supports)
     * - One-time startup position selection and runtime deviation reporting
     */
    class StateHold : public FSMState
    {
    public:
        /**
         * @brief Constructor
         * @param ctrl_interfaces Control interfaces
         * @param node ROS lifecycle node for parameter access
         * @param gravity_compensation Optional gravity compensation utility (nullptr if not needed)
         */
        explicit StateHold(CtrlInterfaces& ctrl_interfaces,
                         std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = nullptr,
                         const std::shared_ptr<GravityCompensation>& gravity_compensation = nullptr);

        void enter() override;
        void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        void exit() override;
        FSMStateName checkChange() override;

    private:
        void updateParam();

        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        std::shared_ptr<GravityCompensation> gravity_compensation_;
        std::vector<double> hold_positions_;                  // Positions captured once when entering
        double first_check_position_threshold_{0.1};          // Threshold for the initial hold position check (radians). If <= 0, checking is disabled.
        double joint_position_diff_threshold_{0.1};           // Runtime warning threshold (radians); never reanchors. If <= 0, warnings are disabled.
    };
} // namespace arms_controller_common
