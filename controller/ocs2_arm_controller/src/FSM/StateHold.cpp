//
// Created for OCS2 Arm Controller - StateHold
//

#include "ocs2_arm_controller/FSM/StateHold.h"
#include <cmath>

namespace ocs2::mobile_manipulator
{
    StateHold::StateHold(CtrlInterfaces& ctrl_interfaces,
                         const std::shared_ptr<CtrlComponent>& ctrl_comp)
        : FSMState(FSMStateName::HOLD, "HOLD"),
          ctrl_interfaces_(ctrl_interfaces),
          ctrl_comp_(ctrl_comp),
          node_(ctrl_comp_->getNode())  // Save node reference from ctrl_comp
    {
        // Get hold position threshold from parameters (already declared in Ocs2ArmController::on_init)
        joint_position_threshold_ = node_->get_parameter("hold_position_threshold").as_double();
        
        RCLCPP_INFO(node_->get_logger(), "StateHold initialized with position threshold: %.4f rad (%.2f degrees)",
                   joint_position_threshold_, joint_position_threshold_ * 180.0 / M_PI);
    }

    void StateHold::enter()
    {
        // HOLD state does not send any commands, let robot maintain current position
        // Record current position on first run or if position difference exceeds threshold
        if (hold_positions_.empty())
        {
            // Get current joint positions
            hold_positions_.resize(ctrl_interfaces_.joint_position_state_interface_.size());
            for (size_t i = 0; i < ctrl_interfaces_.joint_position_state_interface_.size(); ++i)
            {
                hold_positions_[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
            }

            RCLCPP_INFO(node_->get_logger(),
                        "HOLD state entered, current positions recorded for %zu joints", hold_positions_.size());
        }
        else
        {
            // Check if current joint positions are within threshold of recorded positions
            bool within_threshold = true;
            double max_diff = 0.0;
            
            for (size_t i = 0; i < ctrl_interfaces_.joint_position_state_interface_.size() && i < hold_positions_.size(); ++i)
            {
                double current_pos = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
                double diff = std::abs(current_pos - hold_positions_[i]);
                
                if (diff > max_diff)
                {
                    max_diff = diff;
                }
                
                if (diff > joint_position_threshold_)
                {
                    within_threshold = false;
                }
            }
            
            if (within_threshold)
            {
                RCLCPP_INFO(node_->get_logger(), 
                           "HOLD state entered, using previously recorded positions (max diff: %.4f rad < %.4f rad)", 
                           max_diff, joint_position_threshold_);
            }
            else
            {
                // Update hold positions to current positions when difference exceeds threshold
                size_t num_joints = ctrl_interfaces_.joint_position_state_interface_.size();
                hold_positions_.resize(num_joints);
                for (size_t i = 0; i < num_joints; ++i)
                {
                    hold_positions_[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
                }
                
                RCLCPP_WARN(node_->get_logger(), 
                           "HOLD state entered, position difference (max: %.4f rad) exceeds threshold (%.4f rad). "
                           "Updating hold positions to current positions for safety.", 
                           max_diff, joint_position_threshold_);
            }
        }
    }

    void StateHold::run(const rclcpp::Time& /* time */, const rclcpp::Duration& /* period */)
    {
        // HOLD state does not send any commands
        // Robot will maintain its current position

        // In force control mode, calculate static torques to maintain current position
        if (ctrl_interfaces_.control_mode_ == ControlMode::MIX && ctrl_comp_)
        {
            // Get current joint positions
            vector_t current_positions(ctrl_interfaces_.joint_position_state_interface_.size());
            for (size_t i = 0; i < ctrl_interfaces_.joint_position_state_interface_.size(); ++i)
            {
                current_positions(i) = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
            }

            // Calculate static torques needed to maintain current position using CtrlComponent
            vector_t static_torques = ctrl_comp_->calculateStaticTorques();

            // Set effort commands (torques) for force control
            for (size_t i = 0; i < ctrl_interfaces_.joint_force_command_interface_.size() && i < static_torques.size();
                 ++i)
            {
                ctrl_interfaces_.joint_force_command_interface_[i].get().set_value(static_torques(i));
            }

            // Set kp and kd gains for force control if available
            if (ctrl_interfaces_.default_gains_.size() >= 2)
            {
                double kp = ctrl_interfaces_.default_gains_[0]; // Position gain
                double kd = ctrl_interfaces_.default_gains_[1]; // Velocity gain

                // Set kp gains
                for (auto& kp_interface : ctrl_interfaces_.joint_kp_command_interface_)
                {
                    kp_interface.get().set_value(kp);
                }

                // Set kd gains
                for (auto& kd_interface : ctrl_interfaces_.joint_kd_command_interface_)
                {
                    kd_interface.get().set_value(kd);
                }
            }
        }
    }

    void StateHold::exit(){}

    FSMStateName StateHold::checkChange()
    {
        // Check control inputs for state transition
        switch (ctrl_interfaces_.control_inputs_.command)
        {
        case 3: return FSMStateName::OCS2;
        case 1: return FSMStateName::HOME;
        default: return FSMStateName::HOLD;
        }
    }
} // namespace ocs2_arm_controller
