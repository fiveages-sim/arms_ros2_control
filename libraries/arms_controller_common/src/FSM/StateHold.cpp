//
// Common StateHold Implementation
//
#include "arms_controller_common/FSM/StateHold.h"
#include <cmath>
#include <utility>

namespace arms_controller_common
{
    StateHold::StateHold(CtrlInterfaces& ctrl_interfaces,
                        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
                        const std::shared_ptr<GravityCompensation>& gravity_compensation)
        : FSMState(FSMStateName::HOLD, "HOLD", ctrl_interfaces),
          node_(std::move(node)),
          gravity_compensation_(gravity_compensation),
          first_threshold_check_passed_(false)
    {
    }

    void StateHold::updateParam()
    {
        joint_position_threshold_ = node_->get_parameter("hold_position_threshold").as_double();
    }

    void StateHold::enter()
    {
        updateParam();
        size_t num_joints = ctrl_interfaces_.joint_position_state_interface_.size();
        hold_positions_.resize(num_joints);
        first_threshold_check_passed_ = false;
        for (size_t i = 0; i < num_joints; ++i)
        {
            hold_positions_[i] = ctrl_interfaces_.last_sent_joint_positions_[i];
        }
        
        RCLCPP_INFO(node_->get_logger(),
                   "HOLD state entered, using last sent joint positions for %zu joints (avoids jumps)", 
                   hold_positions_.size());
    }

    void StateHold::run(const rclcpp::Time& time, const rclcpp::Duration& /* period */)
    {
        updateParam();
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

            if (!first_threshold_check_passed_)
            {
                if (!exceeds_threshold)
                {
                    first_threshold_check_passed_ = true;
                    RCLCPP_INFO(node_->get_logger(),
                               "HOLD state: first threshold check passed (max diff: %.4f rad <= threshold: %.4f rad). "
                               "Automatic adjustment enabled.",
                               max_diff, joint_position_threshold_);
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                       "HOLD state: first threshold check failed (max diff: %.4f rad > threshold: %.4f rad). "
                                       "Waiting for threshold to be satisfied before enabling automatic adjustment.",
                                       max_diff, joint_position_threshold_);
                }
            }
            else
            {
                if (exceeds_threshold)
                {
                    size_t num_joints = ctrl_interfaces_.joint_position_state_interface_.size();
                    hold_positions_.resize(num_joints);
                    for (size_t i = 0; i < num_joints; ++i)
                    {
                        auto value = ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional();
                        hold_positions_[i] = value.value_or(0.0);
                    }

                    static rclcpp::Time last_warn_time(0, 0, RCL_ROS_TIME);
                    static bool first_warn = true;
                    
                    rclcpp::Duration time_since_last_warn = time - last_warn_time;
                    
                    if (first_warn || time_since_last_warn.seconds() >= 1.0)
                    {
                        RCLCPP_WARN(node_->get_logger(),
                                   "HOLD state: position difference (max: %.4f rad) exceeds threshold (%.4f rad). "
                                   "Updating hold positions to current positions for safety.",
                                   max_diff, joint_position_threshold_);
                        last_warn_time = time;
                        first_warn = false;
                    }
                }
            }
        }
        else
        {
            first_threshold_check_passed_ = true;
        }

        for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size() &&
             i < hold_positions_.size(); ++i)
        {
            ctrl_interfaces_.setJointPositionCommand(i, hold_positions_[i]);
        }

        if (ctrl_interfaces_.control_mode_ == ControlMode::MIX && gravity_compensation_)
        {
            std::vector<double> current_positions;
            for (auto i : ctrl_interfaces_.joint_position_state_interface_)
            {
                auto value = i.get().get_optional();
                current_positions.push_back(value.value_or(0.0));
            }

            std::vector<double> static_torques = 
                gravity_compensation_->calculateStaticTorques(current_positions);

            for (size_t i = 0; i < ctrl_interfaces_.joint_force_command_interface_.size() && 
                 i < static_torques.size(); ++i)
            {
                std::ignore = ctrl_interfaces_.joint_force_command_interface_[i].get().set_value(static_torques[i]);
            }
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

