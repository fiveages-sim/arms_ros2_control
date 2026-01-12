//
// Common StateHold Implementation
//
#include "arms_controller_common/FSM/StateHold.h"
#include <cmath>

namespace arms_controller_common
{
    StateHold::StateHold(CtrlInterfaces& ctrl_interfaces,
                        const rclcpp::Logger& logger,
                        double position_threshold,
                        std::shared_ptr<GravityCompensation> gravity_compensation)
        : FSMState(FSMStateName::HOLD, "HOLD", ctrl_interfaces),
          logger_(logger),
          gravity_compensation_(gravity_compensation),
          joint_position_threshold_(position_threshold)
    {
        if (position_threshold > 0.0)
        {
            RCLCPP_INFO(logger_,
                       "StateHold initialized with position threshold: %.4f rad (%.2f degrees), threshold checking enabled",
                       joint_position_threshold_, joint_position_threshold_ * 180.0 / M_PI);
        }
        else
        {
            RCLCPP_INFO(logger_,
                       "StateHold initialized with position threshold: %.4f rad, threshold checking disabled",
                       position_threshold);
        }
    }

    void StateHold::enter()
    {
        // Always record current position when entering HOLD state
        size_t num_joints = ctrl_interfaces_.joint_position_state_interface_.size();
        hold_positions_.resize(num_joints);
        for (size_t i = 0; i < num_joints; ++i)
        {
            auto value = ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional();
            hold_positions_[i] = value.value_or(0.0);
        }

        RCLCPP_INFO(logger_,
                   "HOLD state entered, current positions recorded for %zu joints", 
                   hold_positions_.size());
    }

    void StateHold::run(const rclcpp::Time& time, const rclcpp::Duration& /* period */)
    {
        // Check position difference between current and hold positions (if threshold > 0)
        if (joint_position_threshold_ > 0.0)
        {
            double max_diff = 0.0;
            bool exceeds_threshold = false;

            for (size_t i = 0; i < ctrl_interfaces_.joint_position_state_interface_.size() && 
                 i < hold_positions_.size(); ++i)
            {
                auto value = ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional();
                double current_pos = value.value_or(0.0);
                double diff = std::abs(current_pos - hold_positions_[i]);

                if (diff > max_diff)
                {
                    max_diff = diff;
                }

                if (diff > joint_position_threshold_)
                {
                    exceeds_threshold = true;
                }
            }

            // If position difference exceeds threshold, update hold positions to current positions
            if (exceeds_threshold)
            {
                size_t num_joints = ctrl_interfaces_.joint_position_state_interface_.size();
                hold_positions_.resize(num_joints);
                for (size_t i = 0; i < num_joints; ++i)
                {
                    auto value = ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional();
                    hold_positions_[i] = value.value_or(0.0);
                }

                // Throttle warning messages using ROS2 time (log at most once per second)
                static rclcpp::Time last_warn_time(0, 0, RCL_ROS_TIME);
                static bool first_warn = true;
                
                rclcpp::Duration time_since_last_warn = time - last_warn_time;
                
                if (first_warn || time_since_last_warn.seconds() >= 1.0)
                {
                    RCLCPP_WARN(logger_,
                               "HOLD state: position difference (max: %.4f rad) exceeds threshold (%.4f rad). "
                               "Updating hold positions to current positions for safety.",
                               max_diff, joint_position_threshold_);
                    last_warn_time = time;
                    first_warn = false;
                }
            }
        }

        // HOLD state maintains the recorded positions
        // Set position commands to hold positions
        for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size() &&
             i < hold_positions_.size(); ++i)
        {
            std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(hold_positions_[i]);
        }

        // In force control mode, calculate static torques
        if (ctrl_interfaces_.control_mode_ == ControlMode::MIX && gravity_compensation_)
        {
            // Get current joint positions
            std::vector<double> current_positions;
            for (size_t i = 0; i < ctrl_interfaces_.joint_position_state_interface_.size(); ++i)
            {
                auto value = ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional();
                current_positions.push_back(value.value_or(0.0));
            }

            // Calculate static torques
            std::vector<double> static_torques = 
                gravity_compensation_->calculateStaticTorques(current_positions);

            // Set effort commands
            for (size_t i = 0; i < ctrl_interfaces_.joint_force_command_interface_.size() && 
                 i < static_torques.size(); ++i)
            {
                std::ignore = ctrl_interfaces_.joint_force_command_interface_[i].get().set_value(static_torques[i]);
            }

            // Set kp and kd gains for force control if available
            if (ctrl_interfaces_.default_gains_.size() >= 2)
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
        }
    }

    void StateHold::exit()
    {
        // Nothing to do on exit
    }

    FSMStateName StateHold::checkChange()
    {
        // Check FSM command for state transition
        switch (ctrl_interfaces_.fsm_command_)
        {
        case 1:
            return FSMStateName::HOME;
        case 3:
            // Could be MOVE or OCS2 depending on controller
            return FSMStateName::INVALID;  // Let derived classes handle this
        default:
            return FSMStateName::HOLD;
        }
    }
} // namespace arms_controller_common

