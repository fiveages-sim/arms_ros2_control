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
        if (rest_pos.empty())
        {
            return;
        }
        
        // Add rest pose as the second configuration (index 1) in home_configs_
        // If home_configs_ is empty, add it as the first configuration
        if (home_configs_.empty())
        {
            // If no home configs exist, add rest_pos as the first config
            home_configs_.push_back(rest_pos);
            current_target_ = rest_pos;
            current_config_index_ = 0;
            has_multiple_configs_ = false;
            RCLCPP_WARN(logger_, 
                       "Rest pose added but no home configurations exist. Using rest pose as first configuration.");
        }
        else if (home_configs_.size() == 1)
        {
            // If only one home config exists, add rest_pos as the second config
            home_configs_.push_back(rest_pos);
            has_multiple_configs_ = true;
            RCLCPP_INFO(logger_, 
                       "Rest pose added as second configuration (index 1). Total configurations: %zu", 
                       home_configs_.size());
        }
        else
        {
            // If multiple configs exist, replace or add the second one (index 1) with rest_pos
            if (home_configs_.size() > 1)
            {
                // Replace existing second config with rest_pos
                home_configs_[1] = rest_pos;
                RCLCPP_INFO(logger_, 
                           "Rest pose updated as second configuration (index 1). Total configurations: %zu", 
                           home_configs_.size());
            }
            else
            {
                // This shouldn't happen (size should be >= 1 from previous checks), but handle it anyway
                home_configs_.push_back(rest_pos);
                has_multiple_configs_ = true;
                RCLCPP_INFO(logger_, 
                           "Rest pose added as second configuration (index 1). Total configurations: %zu", 
                           home_configs_.size());
            }
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
                size_t target_index = static_cast<size_t>(current_command - (switch_command_base_ + 1));
                if (target_index < home_configs_.size())
                {
                    selectConfiguration(target_index);
                }
            }
            // Note: fsm_command_ is read-only from topic, so we don't reset it here
            // The command will be cleared by the publisher when needed
        }

        // Rest pose is now part of home_configs_ (as index 1), so it's handled by the multi-config switching above
        // No separate pose switching logic needed

        last_command_ = current_command;

        // Update interpolation progress
        double controller_frequency = ctrl_interfaces_.frequency_;
        if (duration_ <= 0.0 || controller_frequency <= 0.0)
        {
            // Invalid timing configuration: jump directly to target
            percent_ = 1.0;
        }
        else
        {
            percent_ += 1.0 / (duration_ * controller_frequency);
        }
        percent_ = std::min(percent_, 1.0);

        // Calculate interpolation phase
        double phase = 0.0;
        if (interpolation_type_ == InterpolationType::NONE)
        {
            // NONE type: directly set target position without interpolation
            phase = 1.0;
        }
        else if (percent_ >= 1.0)
        {
            phase = 1.0;
        }
        else if (interpolation_type_ == InterpolationType::LINEAR)
        {
            phase = percent_;
        }
        else
        {
            const double scale = (tanh_scale_ > 0.0) ? tanh_scale_ : 3.0;
            phase = std::tanh(percent_ * scale);
        }
        phase = std::clamp(phase, 0.0, 1.0);

        // Apply interpolated position to joints
        for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size() &&
             i < current_target_.size() && i < start_pos_.size(); ++i)
        {
            double interpolated_value = phase * current_target_[i] + (1.0 - phase) * start_pos_[i];
            std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(interpolated_value);
        }

        // In force control mode, calculate static torques
        if (ctrl_interfaces_.control_mode_ == ControlMode::MIX && gravity_compensation_)
        {
            // Get interpolated joint positions
            std::vector<double> interpolated_positions;
            for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size(); ++i)
            {
                auto value = ctrl_interfaces_.joint_position_command_interface_[i].get().get_optional();
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
        percent_ = 0.0;
    }

    void StateHome::setInterpolationType(const std::string& type)
    {
        const std::string t = toLowerCopy(type);
        if (t != "linear" && t != "tanh")
        {
            RCLCPP_WARN(logger_, "Unknown home interpolation type '%s', falling back to 'linear'", type.c_str());
        }
        interpolation_type_ = parseInterpolationType(type, InterpolationType::LINEAR);
    }

    void StateHome::setTanhScale(double scale)
    {
        if (scale <= 0.0 || !std::isfinite(scale))
        {
            RCLCPP_WARN(logger_, "Invalid home tanh scale %.3f, keeping %.3f", scale, tanh_scale_);
            return;
        }
        tanh_scale_ = scale;
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
            RCLCPP_WARN(logger_,
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
            RCLCPP_WARN(logger_,
                        "Invalid configuration index %zu (max: %zu), returning empty vector",
                        config_index, home_configs_.size() - 1);
            return std::vector<double>();
        }

        return home_configs_[config_index];
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
