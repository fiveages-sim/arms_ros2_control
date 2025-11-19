//
// Created for Basic Joint Controller - StateHome
//

#include "basic_joint_controller/FSM/StateHome.h"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <tuple>

namespace basic_joint_controller
{
    // Constructor - configurations will be loaded via init() method
    StateHome::StateHome(CtrlInterfaces& ctrl_interfaces,
                         const rclcpp::Logger& logger, double duration, int32_t switch_command_base)
        : FSMState(FSMStateName::HOME, "home"),
          ctrl_interfaces_(ctrl_interfaces),
          logger_(logger),
          duration_(duration),
          switch_command_base_(switch_command_base)
    {
        // Configurations will be loaded in init() method
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

        RCLCPP_INFO(logger_,
                    "Starting linear interpolation to home configuration %zu over %.1f seconds",
                    current_config_index_, duration_);
    }

    void StateHome::run(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        // Check for configuration switching command
        // command == switch_command_base_: cycle to next configuration
        // command == switch_command_base_+1, switch_command_base_+2, ...: switch to specific configuration
        //   command (switch_command_base_+1) -> config 0, command (switch_command_base_+2) -> config 1, etc.
        int32_t current_command = ctrl_interfaces_.control_inputs_.command;
        bool is_switch_command = current_command >= switch_command_base_;

        // Detect command change: trigger when switch command value changes
        bool command_changed = (current_command != last_command_);

        // Trigger switch if:
        // 1. Command is a switch command (>= switch_command_base_)
        // 2. Command value changed (rising edge detection)
        if (is_switch_command && command_changed)
        {
            if (has_multiple_configs_)
            {
                if (current_command == switch_command_base_)
                {
                    // Cycle to next configuration
                    switchConfiguration();
                }
                else if (current_command >= switch_command_base_ + 1)
                {
                    // Switch to specific configuration: command (switch_command_base_+1) -> config 0, etc.
                    size_t target_index = static_cast<size_t>(current_command - (switch_command_base_ + 1));
                    if (target_index < home_configs_.size())
                    {
                        selectConfiguration(target_index);
                    }
                    else
                    {
                        RCLCPP_WARN(logger_,
                                    "Invalid configuration index %zu (max: %zu), ignoring",
                                    target_index, home_configs_.size() - 1);
                    }
                }
            }
            else
            {
                RCLCPP_WARN(logger_,
                            "Only one configuration available, cannot switch");
            }
        }

        // Reset command after processing to ensure it's only processed once per cycle
        // This allows the same command to be triggered again when it's sent again
        if (is_switch_command)
        {
            ctrl_interfaces_.control_inputs_.command = 0;
        }

        // Update last command value
        last_command_ = current_command;

        // Update interpolation progress based on actual controller frequency
        double controller_frequency = ctrl_interfaces_.frequency_;
        percent_ += 1.0 / (duration_ * controller_frequency);

        // Clamp percent to [0, 1]
        if (percent_ > 1.0)
        {
            percent_ = 1.0;
        }

        // Calculate interpolation phase using tanh for smooth transition
        double phase = std::tanh(percent_ * 3.0); // Scale for faster convergence

        // Apply interpolated position to joints
        for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size() &&
             i < current_target_.size() && i < start_pos_.size(); ++i)
        {
            double interpolated_value = phase * current_target_[i] + (1.0 - phase) * start_pos_[i];
            std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(interpolated_value);
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
                        "Invalid configuration index %zu (max: %zu)", config_index, home_configs_.size() - 1);
            return;
        }

        current_config_index_ = config_index;
        current_target_ = home_configs_[config_index];
        startInterpolation();

        RCLCPP_INFO(logger_,
                    "Switching to configuration %zu", config_index);
    }

    void StateHome::switchConfiguration()
    {
        if (!has_multiple_configs_)
        {
            RCLCPP_WARN(logger_,
                        "Cannot switch: only one configuration available");
            return;
        }

        // Cycle to next configuration
        current_config_index_ = (current_config_index_ + 1) % home_configs_.size();
        current_target_ = home_configs_[current_config_index_];
        startInterpolation();

        RCLCPP_INFO(logger_,
                    "Switching to configuration %zu", current_config_index_);
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
} // namespace basic_joint_controller
