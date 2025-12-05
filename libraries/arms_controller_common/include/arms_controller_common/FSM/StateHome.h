//
// Common StateHome for Arm Controllers
//
#pragma once

#include "arms_controller_common/FSM/FSMState.h"
#include "arms_controller_common/utils/GravityCompensation.h"
#include <vector>
#include <memory>
#include <functional>
#include <rclcpp/rclcpp.hpp>

namespace arms_controller_common
{
    /**
     * @brief StateHome - Moves arm to home position(s) with smooth interpolation
     * 
     * Supports:
     * - Single or multiple home configurations
     * - Optional gravity compensation (if hardware supports)
     * - Configurable interpolation duration
     * - Flexible configuration switching
     */
    class StateHome : public FSMState
    {
    public:
        /**
         * @brief Constructor
         * @param ctrl_interfaces Control interfaces
         * @param logger ROS logger
         * @param duration Interpolation duration in seconds
         * @param gravity_compensation Optional gravity compensation utility (nullptr if not needed)
         */
        explicit StateHome(CtrlInterfaces& ctrl_interfaces,
                          const rclcpp::Logger& logger,
                          double duration = 3.0,
                          std::shared_ptr<GravityCompensation> gravity_compensation = nullptr);

        /**
         * @brief Set single home configuration
         * @param home_pos Home position vector
         */
        void setHomePosition(const std::vector<double>& home_pos);

        /**
         * @brief Add multiple home configurations
         * @param home_configs Vector of home configurations
         */
        void setHomeConfigurations(const std::vector<std::vector<double>>& home_configs);

        /**
         * @brief Set rest pose (alternative to home)
         * @param rest_pos Rest position vector
         */
        void setRestPose(const std::vector<double>& rest_pos);

        /**
         * @brief Set configuration switch command base (for multi-config switching)
         * @param switch_command_base Base command value (default: 100)
         */
        void setSwitchCommandBase(int32_t switch_command_base) { switch_command_base_ = switch_command_base; }

        /**
         * @brief Set pose switch command (for home/rest switching)
         * @param pose_switch_command Command value for pose switching (default: 4)
         */
        void setPoseSwitchCommand(int32_t pose_switch_command) { pose_switch_command_ = pose_switch_command; }

        void enter() override;
        void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        void exit() override;
        FSMStateName checkChange() override;

        /**
         * @brief Select specific configuration by index (0-based)
         * @param config_index Configuration index
         */
        void selectConfiguration(size_t config_index);

    private:
        void switchConfiguration();
        void switchPose();
        void startInterpolation();

        rclcpp::Logger logger_;
        std::shared_ptr<GravityCompensation> gravity_compensation_;

        // Home configurations
        std::vector<std::vector<double>> home_configs_;  // Multiple home configurations
        std::vector<double> rest_pos_;                  // Rest pose (optional)
        std::vector<double> start_pos_;                  // Starting position
        std::vector<double> current_target_;            // Current target configuration
        size_t current_config_index_{0};                // Current configuration index
        bool has_rest_pose_{false};                     // Whether rest pose is configured
        bool is_rest_pose_{false};                      // Current pose state (false: home, true: rest)

        // Interpolation
        double duration_;                               // Interpolation duration in seconds
        double percent_{0.0};                          // Interpolation progress (0.0 to 1.0)

        // Configuration switching
        int32_t switch_command_base_{100};             // Base command for multi-config switching
        int32_t pose_switch_command_{4};               // Command for home/rest switching
        int32_t last_command_{0};                      // Last command value for edge detection
        bool has_multiple_configs_{false};             // Whether multiple configurations are available
    };
} // namespace arms_controller_common

