//
// Common Control Interfaces for Arm Controllers
//
#pragma once

#include <cmath>
#include <mutex>
#include <vector>
#include <string>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <std_msgs/msg/int32.hpp>

namespace arms_controller_common
{
    // Control mode enum for automatic detection
    enum class ControlMode
    {
        POSITION,    // Position control only
        MIX,         // Mixed: position + velocity + effort (+ optional kp/kd for HT)
        AUTO         // Automatic detection based on available interfaces
    };

    // Control interfaces structure for arm control
    struct CtrlInterfaces
    {
        // Command interfaces - position control (required)
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_position_command_interface_;

        // Optional command interfaces for force/mixed control
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_force_command_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_velocity_command_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_kp_command_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_kd_command_interface_;

        // State interfaces - position and velocity (required)
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> 
        joint_position_state_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> 
        joint_velocity_state_interface_;

        // Optional state interface for force
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> 
        joint_force_state_interface_;

        // FSM command (dedicated topic for state transitions)
        int32_t fsm_command_ = 0;
        
        int frequency_ = 1000;

        // Control mode - automatically detected based on available interfaces
        ControlMode control_mode_ = ControlMode::POSITION;
        bool mode_detected_ = false;

        // Force control gains [kp, kd]
        std::vector<double> default_gains_;
        
        // PD control gains [kp, kd] - used when entering OCS2 state (optional, for ocs2_arm_controller)
        std::vector<double> pd_gains_;

        std::vector<double> last_sent_joint_positions_;

        struct CommandMotionState
        {
            std::vector<double> positions;
            std::vector<double> velocities;
            std::vector<double> accelerations;
        };

        // Commit once per controller cycle, not per setter call: a joint may be
        // written several times in one cycle, or held without another write.
        void recordCommandMotion(double dt)
        {
            std::lock_guard<std::mutex> lock(command_motion_mutex_);
            const auto count = last_sent_joint_positions_.size();
            if (command_motion_.positions.size() != count ||
                !std::isfinite(dt) || dt <= 1e-6)
            {
                command_motion_.positions = last_sent_joint_positions_;
                command_motion_.velocities.assign(count, 0.0);
                command_motion_.accelerations.assign(count, 0.0);
                derivative_valid_.assign(count, false);
                command_velocity_valid_ = false;
                return;
            }
            for (size_t i = 0; i < count; ++i)
            {
                const double velocity =
                    (last_sent_joint_positions_[i] - command_motion_.positions[i]) / dt;
                // Velocity estimates refer to interval midpoints. Use their
                // separation for acceleration when the controller period varies.
                command_motion_.accelerations[i] = command_velocity_valid_
                    ? (velocity - command_motion_.velocities[i]) /
                        (0.5 * (dt + previous_command_period_)) : 0.0;
                command_motion_.velocities[i] = velocity;
                if (i < derivative_valid_.size() && derivative_valid_[i])
                {
                    command_motion_.velocities[i] = analytic_velocities_[i];
                    command_motion_.accelerations[i] = analytic_accelerations_[i];
                    derivative_valid_[i] = false;
                }
                command_motion_.positions[i] = last_sent_joint_positions_[i];
            }
            previous_command_period_ = dt;
            command_velocity_valid_ = true;
        }

        // Optional exact derivatives for this cycle (doubleS or the stop planner).
        void setCommandDerivatives(size_t i, double velocity, double acceleration)
        {
            std::lock_guard<std::mutex> lock(command_motion_mutex_);
            if (analytic_velocities_.size() != last_sent_joint_positions_.size() ||
                derivative_valid_.size() != last_sent_joint_positions_.size())
            {
                derivative_valid_.assign(last_sent_joint_positions_.size(), false);
                analytic_velocities_.resize(derivative_valid_.size());
                analytic_accelerations_.resize(derivative_valid_.size());
            }
            if (i < derivative_valid_.size())
            {
                derivative_valid_[i] = true;
                analytic_velocities_[i] = velocity;
                analytic_accelerations_[i] = acceleration;
            }
        }

        CommandMotionState getCommandMotionState() const
        {
            std::lock_guard<std::mutex> lock(command_motion_mutex_);
            return command_motion_;
        }

        void setJointPositionCommand(size_t index, double value)
        {
            std::ignore = joint_position_command_interface_[index].get().set_value(value);
            last_sent_joint_positions_[index] = value;
        }

        void initializeLastSentPositions()
        {
            last_sent_joint_positions_.clear();
            last_sent_joint_positions_.reserve(joint_position_state_interface_.size());
            for (size_t i = 0; i < joint_position_state_interface_.size(); ++i)
            {
                auto value = joint_position_state_interface_[i].get().get_optional();
                last_sent_joint_positions_.push_back(value.value_or(0.0));
            }
            std::lock_guard<std::mutex> lock(command_motion_mutex_);
            command_motion_.positions = last_sent_joint_positions_;
            command_motion_.velocities.assign(last_sent_joint_positions_.size(), 0.0);
            command_motion_.accelerations.assign(last_sent_joint_positions_.size(), 0.0);
            derivative_valid_.assign(last_sent_joint_positions_.size(), false);
            command_velocity_valid_ = false;
            previous_command_period_ = 0.0;
        }

        // Auto mode detection function - called once during initialization
        void detectAndSetControlMode()
        {
            if (mode_detected_) return;

            // MIX = pos+vel+effort on both command and state.
            // kp/kd are optional (HT claims them; ARX gains live in HI params only).
            const bool has_velocity_cmd = !joint_velocity_command_interface_.empty();
            const bool has_effort_cmd = !joint_force_command_interface_.empty();
            const bool has_position_cmd = !joint_position_command_interface_.empty();

            const bool has_velocity_state = !joint_velocity_state_interface_.empty();
            const bool has_effort_state = !joint_force_state_interface_.empty();
            const bool has_position_state = !joint_position_state_interface_.empty();

            if (has_velocity_cmd && has_effort_cmd && has_position_cmd &&
                has_velocity_state && has_effort_state && has_position_state)
            {
                control_mode_ = ControlMode::MIX;
            }
            else
            {
                control_mode_ = ControlMode::POSITION;
            }

            mode_detected_ = true;
        }

        void clear()
        {
            joint_position_command_interface_.clear();
            joint_force_command_interface_.clear();
            joint_velocity_command_interface_.clear();
            joint_kp_command_interface_.clear();
            joint_kd_command_interface_.clear();
            joint_position_state_interface_.clear();
            joint_velocity_state_interface_.clear();
            joint_force_state_interface_.clear();
            last_sent_joint_positions_.clear();
            std::lock_guard<std::mutex> lock(command_motion_mutex_);
            command_motion_ = {};
            derivative_valid_.assign(last_sent_joint_positions_.size(), false);
            command_velocity_valid_ = false;
            previous_command_period_ = 0.0;
        }
    private:
        std::vector<bool> derivative_valid_;
        std::vector<double> analytic_velocities_, analytic_accelerations_;
        mutable std::mutex command_motion_mutex_;
        CommandMotionState command_motion_;
        double previous_command_period_{0.0};
        bool command_velocity_valid_{false};
    };
} // namespace arms_controller_common

