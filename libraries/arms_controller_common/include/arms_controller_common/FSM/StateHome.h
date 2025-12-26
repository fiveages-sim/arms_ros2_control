//
// Common StateHome for Arm Controllers
//
#pragma once

#include "arms_controller_common/FSM/FSMState.h"
#include "arms_controller_common/utils/Interpolation.h"
#include "arms_controller_common/utils/GravityCompensation.h"
#include "lina_planning/planning/path_planner/movej.h"
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
         * @brief Initialize home configurations from parameters (home_1, home_2, home_3, etc.)
         * 
         * This method should be called in controller's on_init() to use auto_declare.
         * It automatically reads multiple home configurations using naming pattern: home_1, home_2, home_3, etc.
         * 
         * @param auto_declare_func A callable that takes (param_name, default_value) and returns the parameter value
         *                          Typically: [this](const std::string& name, const std::vector<double>& default_value) {
         *                              return this->auto_declare<std::vector<double>>(name, default_value);
         *                          }
         * @return true if at least one home configuration (home_1) was found and loaded, false otherwise
         */
        template<typename AutoDeclareFunc>
        bool init(AutoDeclareFunc auto_declare_func)
        {
            // Read multiple home configurations using naming pattern: home_1, home_2, home_3, etc.
            // Stop when we encounter an empty or missing configuration (configurations should be consecutive)
            for (int i = 1; i <= 10; ++i)  // Support up to 10 configurations
            {
                std::string param_name = "home_" + std::to_string(i);
                std::vector<double> default_config;
                std::vector<double> config = auto_declare_func(param_name, default_config);
                
                if (!config.empty())
                {
                    home_configs_.push_back(config);
                    RCLCPP_INFO(logger_,
                                "Found home configuration %d with %zu joints", i, config.size());
                }
                else
                {
                    // If this configuration is empty, stop checking further (configurations should be consecutive)
                    if (i == 1)
                    {
                        // No configurations found at all
                        RCLCPP_WARN(logger_,
                                    "No home configurations found (home_1, home_2, etc.)");
                    }
                    else
                    {
                        // Found some configurations but reached the end
                        RCLCPP_INFO(logger_,
                                    "Found %zu home configuration(s) (home_1 to home_%d)", 
                                    home_configs_.size(), i - 1);
                    }
                    break;
                }
            }
            
            // Validate configurations
            bool configs_loaded = !home_configs_.empty();
            
            if (configs_loaded)
            {
                // Only set target and flags if configurations were actually loaded
                current_target_ = home_configs_[0];
                current_config_index_ = 0;
                has_multiple_configs_ = home_configs_.size() > 1;
                
                RCLCPP_INFO(logger_,
                            "StateHome initialized with %zu configuration(s)", home_configs_.size());
            }
            // Note: Warning message is already logged in the loop above when home_1 is not found
            // No need to log again here to avoid duplicate warnings
            
            return configs_loaded;
        }

        /**
         * @brief Set rest pose as the second configuration (index 1) in home_configs_
         * 
         * Rest pose is now integrated into the home configurations system.
         * It will be added as the second configuration (index 1) in home_configs_.
         * If only one home config exists, rest_pos will be added as the second config.
         * If multiple configs exist, the second config (index 1) will be replaced with rest_pos.
         * 
         * @param rest_pos Rest position vector
         */
        void setRestPose(const std::vector<double>& rest_pos);

        /**
         * @brief Set configuration switch command base (for multi-config switching)
         * @param switch_command_base Base command value (default: 100)
         */
        void setSwitchCommandBase(int32_t switch_command_base) { switch_command_base_ = switch_command_base; }

        void enter() override;
        void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        void exit() override;
        FSMStateName checkChange() override;

        /**
         * @brief Select specific configuration by index (0-based)
         * @param config_index Configuration index
         */
        void selectConfiguration(size_t config_index);

        /**
         * @brief Select interpolation type used to compute phase from percent.
         * @param type "tanh" or "linear" (case-insensitive). Unknown values fall back to "tanh".
         */
        void setInterpolationType(const std::string& type);

        /**
         * @brief Set tanh scale used when interpolation type is TANH.
         * @note Larger values => faster start, slower tail. Must be > 0, otherwise it will be clamped to a default.
         */
        void setTanhScale(double scale);

        void initMoveJPlanner();
    private:
        void switchConfiguration();
        void startInterpolation();

        rclcpp::Logger logger_;
        std::shared_ptr<GravityCompensation> gravity_compensation_;

        // Home configurations
        std::vector<std::vector<double>> home_configs_;  // Multiple home configurations (rest pose is index 1)
        std::vector<double> start_pos_;                  // Starting position
        std::vector<double> current_target_;            // Current target configuration
        size_t current_config_index_{0};                // Current configuration index

        // Interpolation
        double duration_;                               // Interpolation duration in seconds
        double percent_{0.0};                          // Interpolation progress (0.0 to 1.0)
        InterpolationType interpolation_type_{InterpolationType::TANH};
        double tanh_scale_{3.0};

        // Configuration switching
        int32_t switch_command_base_{100};             // Base command for multi-config switching
        int32_t last_command_{0};                      // Last command value for edge detection
        bool has_multiple_configs_{false};             // Whether multiple configurations are available
        planning::moveJ movej_planner;                 //Joint interpolation planner
    };
} // namespace arms_controller_common

