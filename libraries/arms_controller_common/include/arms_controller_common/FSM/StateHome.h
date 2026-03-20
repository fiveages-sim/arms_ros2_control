//
// Common StateHome for Arm Controllers
//
#pragma once

#include "arms_controller_common/FSM/FSMState.h"
#include "arms_controller_common/utils/Interpolation.h"
#include "arms_controller_common/utils/GravityCompensation.h"
#include "arms_controller_common/utils/JointTrajectoryManager.h"
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

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
         * @param gravity_compensation Optional gravity compensation utility (nullptr if not needed)
         * @param node ROS node for dynamic parameter access and logger (required)
         */
        explicit StateHome(CtrlInterfaces& ctrl_interfaces,
                          const std::shared_ptr<GravityCompensation>& gravity_compensation = nullptr,
                          const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node = nullptr);

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
         * @brief Initialize home configurations from YAML parameter overrides.
         *
         * Reads home_1, home_2, ... home_N directly from the node's parameter-override
         * map (the values loaded from --params-file before any declare_parameter call).
         * Each found key is then declared with its actual value so it remains visible
         * via `ros2 param get`.
         *
         * Why overrides instead of declare_parameter(name, {})?
         *   Calling declare_parameter<std::vector<double>>(name, {}) with an empty-vector
         *   default for a key that does NOT exist in the YAML can corrupt glibc's heap
         *   top-chunk on certain rclcpp/glibc version combinations, triggering a
         *   malloc assertion (malloc.c sysmalloc) on the very next allocation.
         *   Reading from overrides is safe because it never touches the allocator for
         *   absent keys.
         *
         * @param auto_declare_func  Callable (param_name, actual_value) → actual_value.
         *                           Used only for keys that exist in the YAML, so the
         *                           default is always the real value, never empty {}.
         *                           Typically:
         *                             [this](const std::string& n, const std::vector<double>& v) {
         *                                 return this->auto_declare<std::vector<double>>(n, v);
         *                             }
         * @return true if at least one configuration (home_1) was found.
         */
        template<typename AutoDeclareFunc>
        bool init(AutoDeclareFunc auto_declare_func)
        {
            const auto& overrides =
                node_->get_node_parameters_interface()->get_parameter_overrides();

            for (int i = 1; i <= 10; ++i)
            {
                const std::string param_name = "home_" + std::to_string(i);

                // 1. Check existence in YAML overrides — no allocator involvement.
                auto it = overrides.find(param_name);
                if (it == overrides.end())
                {
                    if (i == 1)
                    {
                        RCLCPP_WARN(node_->get_logger(),
                                    "No home configurations found (home_1, home_2, etc.)");
                    }
                    else
                    {
                        RCLCPP_INFO(node_->get_logger(),
                                    "Found %zu home configuration(s) (home_1 to home_%d)",
                                    home_configs_.size(), i - 1);
                    }
                    break;
                }

                // 2. Read the actual value directly from the override map.
                std::vector<double> config;
                try
                {
                    config = it->second.get<std::vector<double>>();
                }
                catch (const rclcpp::exceptions::InvalidParameterTypeException& e)
                {
                    RCLCPP_ERROR(node_->get_logger(),
                                 "Parameter '%s' has wrong type, skipping: %s",
                                 param_name.c_str(), e.what());
                    break;
                }

                if (config.empty())
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "Parameter '%s' is an empty array, stopping here",
                                param_name.c_str());
                    break;
                }

                // 3. Declare with the real value so `ros2 param get` works.
                //    auto_declare_func is safe here: default == actual value, never {}.
                auto_declare_func(param_name, config);

                home_configs_.push_back(config);
                RCLCPP_INFO(node_->get_logger(),
                            "Found home configuration %d with %zu joints", i, config.size());
            }

            bool configs_loaded = !home_configs_.empty();
            if (configs_loaded)
            {
                current_target_       = home_configs_[0];
                current_config_index_ = 0;
                has_multiple_configs_ = home_configs_.size() > 1;
                RCLCPP_INFO(node_->get_logger(),
                            "StateHome initialized with %zu configuration(s)",
                            home_configs_.size());
            }
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
         * @brief Get configuration by index (0-based)
         * @param config_index Configuration index
         * @return Configuration vector, or empty vector if index is invalid
         */
        std::vector<double> getConfiguration(size_t config_index) const;

        /**
         * @brief Update parameters from ROS node
         * Updates duration, tanh_scale, and interpolation_type from node parameters.
         * This method can be extended to update additional parameters in the future.
         */
        void updateParam();

    private:
        void switchConfiguration();
        void startInterpolation();

        std::shared_ptr<GravityCompensation> gravity_compensation_;

        // Home configurations
        std::vector<std::vector<double>> home_configs_;     // Multiple home configurations (rest pose is index 1)
        std::vector<double> start_pos_;                     // Starting position
        std::vector<double> current_target_;                // Current target configuration
        size_t current_config_index_{0};                    // Current configuration index

        // Interpolation
        double duration_{3.0};                              // Interpolation duration in seconds (default value, will be updated by updateParam())
        InterpolationType interpolation_type_{InterpolationType::TANH};
        double tanh_scale_{3.0};
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_; // ROS node for parameter access

        // Unified trajectory manager
        JointTrajectoryManager trajectory_manager_;

        // Configuration switching
        long switch_command_base_{100};                     // Base command for multi-config switching
        int32_t last_command_{0};                           // Last command value for edge detection
        bool has_multiple_configs_{false};                  // Whether multiple configurations are available
    };
} // namespace arms_controller_common

