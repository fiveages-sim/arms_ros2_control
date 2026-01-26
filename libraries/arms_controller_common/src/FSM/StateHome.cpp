//
// Common StateHome Implementation
//
#include "arms_controller_common/FSM/StateHome.h"
#include <cmath>

namespace arms_controller_common
{
    StateHome::StateHome(CtrlInterfaces& ctrl_interfaces,
                         const std::shared_ptr<GravityCompensation>& gravity_compensation,
                         const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node)
        : FSMState(FSMStateName::HOME, "home", ctrl_interfaces),
          gravity_compensation_(gravity_compensation),
          node_(node),
          trajectory_manager_(node->get_logger())
    {
        // Get switch_command_base from parameter server
        switch_command_base_ = node->get_parameter("switch_command_base").as_int();
    }

    void StateHome::setHomePosition(const std::vector<double>& home_pos)
    {
        home_configs_.clear();
        home_configs_.push_back(home_pos);
        current_target_ = home_pos;
        current_config_index_ = 0;
        has_multiple_configs_ = false;
    }

    void StateHome::setHomeConfigurations(const std::vector<std::vector<double>>& home_configs)
    {
        home_configs_ = home_configs;
        if (!home_configs_.empty())
        {
            current_target_ = home_configs_[0];
            current_config_index_ = 0;
            has_multiple_configs_ = home_configs_.size() > 1;
        }
    }

    void StateHome::setRestPose(const std::vector<double>& rest_pos)
    {
        if (rest_pos.empty())
        {
            return;
        }

        if (home_configs_.size() == 1)
        {
            home_configs_.push_back(rest_pos);
            has_multiple_configs_ = true;
            RCLCPP_INFO(node_->get_logger(),
                        "Rest pose added as second configuration (index 1). Total configurations: %zu",
                        home_configs_.size());
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "Cannot set rest pose: expected exactly 1 home configuration, but found %zu",
                         home_configs_.size());
        }
    }

    void StateHome::enter()
    {
        // Always start with first configuration when entering the state
        if (!home_configs_.empty())
        {
            current_config_index_ = 0;
            current_target_ = home_configs_[0];
        }

        // Get current joint positions as starting positions
        start_pos_.clear();
        for (auto i : ctrl_interfaces_.joint_position_state_interface_)
        {
            auto value = i.get().get_optional();
            start_pos_.push_back(value.value_or(0.0));
        }

        // Update parameters from node
        updateParam();

        // Initialize trajectory manager
        if (!start_pos_.empty() && !current_target_.empty() && start_pos_.size() == current_target_.size())
        {
            if (!trajectory_manager_.initSingleNode(
                start_pos_,
                current_target_,
                duration_,
                interpolation_type_,
                ctrl_interfaces_.frequency_,
                tanh_scale_))
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "Failed to initialize trajectory manager in StateHome::enter()");
            }
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Cannot initialize trajectory manager: start_pos size (%zu) != target_pos size (%zu)",
                        start_pos_.size(), current_target_.size());
        }

        // Set kp and kd gains for force control if available
        if (ctrl_interfaces_.control_mode_ == ControlMode::MIX &&
            ctrl_interfaces_.default_gains_.size() >= 2)
        {
            double kp = ctrl_interfaces_.default_gains_[0];
            double kd = ctrl_interfaces_.default_gains_[1];

            for (auto& kp_interface : ctrl_interfaces_.joint_kp_command_interface_)
            {
                std::ignore = kp_interface.get().set_value(kp);
            }

            for (auto& kd_interface : ctrl_interfaces_.joint_kd_command_interface_)
            {
                std::ignore = kd_interface.get().set_value(kd);
            }
        }

        RCLCPP_INFO(node_->get_logger(),
                    "Starting interpolation to home configuration %zu over %.1f seconds (type=%s)",
                    current_config_index_,
                    duration_,
                    toString(interpolation_type_));
    }

    void StateHome::run(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        int32_t current_command = ctrl_interfaces_.fsm_command_;
        bool command_changed = (current_command != last_command_);

        // Handle multi-configuration switching (command >= switch_command_base_)
        bool is_switch_command = current_command >= switch_command_base_;
        if (is_switch_command && command_changed && has_multiple_configs_)
        {
            if (current_command == switch_command_base_)
            {
                // Cycle to next configuration
                switchConfiguration();
            }
            else if (current_command >= switch_command_base_ + 1)
            {
                // Switch to specific configuration
                if (auto target_index = static_cast<size_t>(current_command - (switch_command_base_ + 1)); target_index < home_configs_.size())
                {
                    selectConfiguration(target_index);
                }
            }
        }

        last_command_ = current_command;

        // Get next trajectory point from unified manager
        std::vector<double> next_positions = trajectory_manager_.getNextPoint();

        if (!next_positions.empty() &&
            next_positions.size() == ctrl_interfaces_.joint_position_command_interface_.size())
        {
            // Apply interpolated position to joints
            for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size() &&
                 i < next_positions.size(); ++i)
            {
                ctrl_interfaces_.setJointPositionCommand(i, next_positions[i]);
            }
        }
        else if (next_positions.empty())
        {
            // If trajectory manager returns empty, maintain current position or use target
            static bool warned = false;
            if (!warned && !trajectory_manager_.isInitialized())
            {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *std::make_shared<rclcpp::Clock>(), 1000,
                                     "Trajectory manager not initialized, maintaining current position");
                warned = true;
            }
            // Maintain current position if trajectory manager is not initialized
            if (!trajectory_manager_.isInitialized() && !current_target_.empty())
            {
                for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size() &&
                     i < current_target_.size(); ++i)
                {
                    ctrl_interfaces_.setJointPositionCommand(i, current_target_[i]);
                }
            }
        }
        else
        {
            static bool warned = false;
            if (!warned)
            {
                RCLCPP_WARN(node_->get_logger(),
                            "Trajectory manager returned positions with size mismatch: got %zu, expected %zu",
                            next_positions.size(), ctrl_interfaces_.joint_position_command_interface_.size());
                warned = true;
            }
        }
        // In force control mode, calculate static torques
        if (ctrl_interfaces_.control_mode_ == ControlMode::MIX && gravity_compensation_)
        {
            // Get interpolated joint positions
            std::vector<double> interpolated_positions;
            for (auto i : ctrl_interfaces_.joint_position_command_interface_)
            {
                auto value = i.get().get_optional();
                interpolated_positions.push_back(value.value_or(0.0));
            }

            // Calculate static torques
            std::vector<double> static_torques =
                gravity_compensation_->calculateStaticTorques(interpolated_positions);

            // Set effort commands
            for (size_t i = 0; i < ctrl_interfaces_.joint_force_command_interface_.size() &&
                 i < static_torques.size(); ++i)
            {
                std::ignore = ctrl_interfaces_.joint_force_command_interface_[i].get().set_value(static_torques[i]);
            }
        }
    }

    void StateHome::exit()
    {
        trajectory_manager_.reset();
    }

    void StateHome::updateParam()
    {
        // Update duration from node parameter
        duration_ = node_->get_parameter("home_duration").as_double();

        // Update tanh_scale from node parameter
        double new_tanh_scale = node_->get_parameter("home_tanh_scale").as_double();
        if (new_tanh_scale > 0.0 && std::isfinite(new_tanh_scale))
        {
            tanh_scale_ = new_tanh_scale;
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Invalid home tanh scale %.3f from parameter, keeping current value %.3f",
                        new_tanh_scale, tanh_scale_);
        }

        // Update interpolation_type from node parameter
        std::string interpolation_type_str = node_->get_parameter("home_interpolation_type").as_string();
        interpolation_type_ = parseInterpolationType(interpolation_type_str, InterpolationType::LINEAR);

        // Future parameters can be added here
    }

    FSMStateName StateHome::checkChange()
    {
        switch (ctrl_interfaces_.fsm_command_)
        {
        case 2:
        case 3:
        case 4:
            return FSMStateName::HOLD;
        default:
            return FSMStateName::HOME;
        }
    }

    void StateHome::selectConfiguration(size_t config_index)
    {
        if (config_index >= home_configs_.size())
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Invalid configuration index %zu (max: %zu)",
                        config_index, home_configs_.size() - 1);
            return;
        }

        current_config_index_ = config_index;
        current_target_ = home_configs_[config_index];
        startInterpolation();
    }

    std::vector<double> StateHome::getConfiguration(size_t config_index) const
    {
        if (config_index >= home_configs_.size())
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Invalid configuration index %zu (max: %zu), returning empty vector",
                        config_index, home_configs_.size() - 1);
            return {};
        }

        return home_configs_[config_index];
    }

    void StateHome::switchConfiguration()
    {
        if (!has_multiple_configs_)
        {
            RCLCPP_WARN(node_->get_logger(), "Cannot switch: only one configuration available");
            return;
        }

        // Cycle to next configuration
        current_config_index_ = (current_config_index_ + 1) % home_configs_.size();
        current_target_ = home_configs_[current_config_index_];
        startInterpolation();
    }

    void StateHome::startInterpolation()
    {
        start_pos_.clear();
        for (size_t i = 0; i < ctrl_interfaces_.joint_position_state_interface_.size(); ++i)
        {
            start_pos_.push_back(ctrl_interfaces_.last_sent_joint_positions_[i]);
        }

        // Update parameters from node
        updateParam();

        // Initialize trajectory manager with new target
        if (!start_pos_.empty() && !current_target_.empty() && start_pos_.size() == current_target_.size())
        {
            trajectory_manager_.initSingleNode(
                start_pos_,
                current_target_,
                duration_,
                interpolation_type_,
                ctrl_interfaces_.frequency_,
                tanh_scale_
            );

            RCLCPP_INFO(node_->get_logger(),
                        "Starting interpolation to configuration %zu over %.1f seconds (type=%s)",
                        current_config_index_, duration_, toString(interpolation_type_));
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Cannot start interpolation: start_pos size (%zu) != target_pos size (%zu)",
                        start_pos_.size(), current_target_.size());
        }
    }
} // namespace arms_controller_common
