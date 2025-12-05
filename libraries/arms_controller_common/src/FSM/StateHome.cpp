//
// Common StateHome Implementation
//
#include "arms_controller_common/FSM/StateHome.h"
#include <cmath>
#include <algorithm>

namespace arms_controller_common
{
    StateHome::StateHome(CtrlInterfaces& ctrl_interfaces,
                        const rclcpp::Logger& logger,
                        double duration,
                        std::shared_ptr<GravityCompensation> gravity_compensation)
        : FSMState(FSMStateName::HOME, "home", ctrl_interfaces),
          logger_(logger),
          gravity_compensation_(gravity_compensation),
          duration_(duration)
    {
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
        rest_pos_ = rest_pos;
        has_rest_pose_ = !rest_pos.empty();
    }

    void StateHome::enter()
    {
        // Always start with first configuration when entering the state
        if (!home_configs_.empty())
        {
            current_config_index_ = 0;
            current_target_ = home_configs_[0];
        }
        is_rest_pose_ = false;

        // Get current joint positions as starting positions
        start_pos_.clear();
        for (auto i : ctrl_interfaces_.joint_position_state_interface_)
        {
            auto value = i.get().get_optional();
            if (value.has_value())
            {
                start_pos_.push_back(value.value());
            }
            else
            {
                start_pos_.push_back(0.0);
            }
        }

        // Reset interpolation progress
        percent_ = 0.0;

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

        RCLCPP_INFO(logger_,
                   "Starting linear interpolation to home configuration %zu over %.1f seconds",
                   current_config_index_, duration_);
    }

    void StateHome::run(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        int32_t current_command = ctrl_interfaces_.control_inputs_.command;
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
                size_t target_index = static_cast<size_t>(current_command - (switch_command_base_ + 1));
                if (target_index < home_configs_.size())
                {
                    selectConfiguration(target_index);
                }
            }
            // Reset command after processing
            ctrl_interfaces_.control_inputs_.command = 0;
        }

        // Handle home/rest pose switching (command == pose_switch_command_)
        if (current_command == pose_switch_command_ && command_changed && has_rest_pose_)
        {
            switchPose();
            // Reset command after processing
            ctrl_interfaces_.control_inputs_.command = 0;
        }

        last_command_ = current_command;

        // Update interpolation progress
        double controller_frequency = ctrl_interfaces_.frequency_;
        percent_ += 1.0 / (duration_ * controller_frequency);
        percent_ = std::min(percent_, 1.0);

        // Calculate interpolation phase using tanh
        double phase = std::tanh(percent_ * 3.0);

        // Apply interpolated position to joints
        for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size() &&
             i < current_target_.size() && i < start_pos_.size(); ++i)
        {
            double interpolated_value = phase * current_target_[i] + (1.0 - phase) * start_pos_[i];
            ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(interpolated_value);
        }

        // In force control mode, calculate static torques
        if (ctrl_interfaces_.control_mode_ == ControlMode::MIX && gravity_compensation_)
        {
            // Get interpolated joint positions
            std::vector<double> interpolated_positions;
            for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size(); ++i)
            {
                interpolated_positions.push_back(
                    ctrl_interfaces_.joint_position_command_interface_[i].get().get_value());
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
        percent_ = 0.0;
    }

    FSMStateName StateHome::checkChange()
    {
        switch (ctrl_interfaces_.control_inputs_.command)
        {
        case 2:
            return FSMStateName::HOLD;
        default:
            return FSMStateName::HOME;
        }
    }

    void StateHome::selectConfiguration(size_t config_index)
    {
        if (config_index >= home_configs_.size())
        {
            RCLCPP_WARN(logger_,
                       "Invalid configuration index %zu (max: %zu)", 
                       config_index, home_configs_.size() - 1);
            return;
        }

        current_config_index_ = config_index;
        current_target_ = home_configs_[config_index];
        startInterpolation();

        RCLCPP_INFO(logger_, "Switching to configuration %zu", config_index);
    }

    void StateHome::switchConfiguration()
    {
        if (!has_multiple_configs_)
        {
            RCLCPP_WARN(logger_, "Cannot switch: only one configuration available");
            return;
        }

        // Cycle to next configuration
        current_config_index_ = (current_config_index_ + 1) % home_configs_.size();
        current_target_ = home_configs_[current_config_index_];
        startInterpolation();

        RCLCPP_INFO(logger_, "Switching to configuration %zu", current_config_index_);
    }

    void StateHome::switchPose()
    {
        if (!has_rest_pose_)
        {
            RCLCPP_WARN(logger_, "Cannot switch pose: rest pose not configured");
            return;
        }

        // Toggle between home and rest pose
        is_rest_pose_ = !is_rest_pose_;
        current_target_ = is_rest_pose_ ? rest_pos_ : home_configs_[current_config_index_];
        startInterpolation();

        RCLCPP_INFO(logger_, "Switching to %s pose", is_rest_pose_ ? "rest" : "home");
    }

    void StateHome::startInterpolation()
    {
        // Get current joint positions as new starting positions
        start_pos_.clear();
        for (auto i : ctrl_interfaces_.joint_position_state_interface_)
        {
            auto value = i.get().get_optional();
            if (value.has_value())
            {
                start_pos_.push_back(value.value());
            }
            else
            {
                start_pos_.push_back(0.0);
            }
        }

        // Reset interpolation progress
        percent_ = 0.0;

        RCLCPP_INFO(logger_,
                   "Starting interpolation to configuration %zu over %.1f seconds",
                   current_config_index_, duration_);
    }
} // namespace arms_controller_common

