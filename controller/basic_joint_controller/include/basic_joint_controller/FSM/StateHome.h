//
// Created for Basic Joint Controller - StateHome
//
#pragma once

#include "basic_joint_controller/BasicJointController.h"
#include <vector>

namespace basic_joint_controller
{
    class StateHome final : public FSMState
    {
    public:
        // Constructor - configurations will be loaded via init() method
        explicit StateHome(CtrlInterfaces& ctrl_interfaces,
                          const rclcpp::Logger& logger,
                          double duration = 3.0,
                          int32_t switch_command_base = 100);

        // Initialize home configurations from parameters (home_1, home_2, home_3, etc.)
        // This method should be called in controller's on_init() to use auto_declare
        // auto_declare_func should be a callable that takes (param_name, default_value) and returns the parameter value
        template<typename AutoDeclareFunc>
        void init(AutoDeclareFunc auto_declare_func)
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
            if (home_configs_.empty())
            {
                RCLCPP_WARN(logger_,
                            "No home configurations found, using empty default configuration");
                home_configs_.emplace_back();
            }
            
            current_target_ = home_configs_[0];
            current_config_index_ = 0;
            has_multiple_configs_ = home_configs_.size() > 1;
            
            RCLCPP_INFO(logger_,
                        "StateHome initialized with %zu configuration(s)", home_configs_.size());
        }

        void enter() override;
        void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        void exit() override;
        FSMStateName checkChange() override;

        // Select configuration by index (0-based)
        void selectConfiguration(size_t config_index);

    private:
        // Switch to next configuration or specific configuration
        void switchConfiguration();
        
        // Start interpolation to new target configuration
        void startInterpolation();

        CtrlInterfaces& ctrl_interfaces_;
        rclcpp::Logger logger_;           // Logger from controller
        std::vector<std::vector<double>> home_configs_;  // Multiple home configurations
        std::vector<double> start_pos_;   // Starting position when entering state or switching
        std::vector<double> current_target_; // Current target configuration
        double duration_;                 // Interpolation duration in seconds
        double percent_{0.0};            // Interpolation progress (0.0 to 1.0)
        size_t current_config_index_{0}; // Current configuration index
        bool has_multiple_configs_{false}; // Whether multiple configurations are available
        int32_t switch_command_base_{100}; // Base command value for configuration switching (default: 100)
        
        // Configuration switching: track last command value to detect changes
        int32_t last_command_{0};  // Last command value to detect changes
    };
} // namespace basic_joint_controller
