//
// Created for OCS2 Arm Controller
//

#pragma once

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include <chrono>
#include <functional>

#include <controller_interface/controller_interface.hpp>
#include <std_msgs/msg/int32.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include "ocs2_arm_controller/control/CtrlComponent.h"

// Use common FSM types from arms_controller_common
#include <arms_controller_common/FSM/FSMState.h>
#include <arms_controller_common/CtrlInterfaces.h>

namespace ocs2::mobile_manipulator
{
    // Use FSM types from arms_controller_common
    using FSMState = arms_controller_common::FSMState;
    using FSMStateName = arms_controller_common::FSMStateName;
    using FSMMode = arms_controller_common::FSMMode;
    using CtrlInterfaces = arms_controller_common::CtrlInterfaces;

    // Forward declarations
    class StateHome;
    class StateOCS2;
    class StateHold;
    class StateMoveJ;

    // Use ControlMode from arms_controller_common
    using ControlMode = arms_controller_common::ControlMode;

    struct FSMStateList
    {
        std::shared_ptr<FSMState> invalid;
        std::shared_ptr<StateHome> home;
        std::shared_ptr<StateOCS2> ocs2; // OCS2 state
        std::shared_ptr<StateHold> hold; // Hold position state
        std::shared_ptr<StateMoveJ> movej; // MoveJ state
    };

    class Ocs2ArmController final : public controller_interface::ControllerInterface
    {
    public:
        Ocs2ArmController() = default;
        ~Ocs2ArmController() override = default;

        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
        controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
        controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
        controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
        
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        std::shared_ptr<FSMState> getNextState(FSMStateName stateName) const;
        void updateControlInputs();

        // Hardware parameters
        std::string command_prefix_;
        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        // Control input parameters
        std::string control_input_name_;
        std::vector<std::string> control_input_interface_types_;

        // Control interfaces
        CtrlInterfaces ctrl_interfaces_;

        // State machine
        FSMStateList state_list_;
        std::shared_ptr<FSMState> current_state_;
        std::shared_ptr<FSMState> next_state_;
        FSMStateName next_state_name_{FSMStateName::INVALID};
        FSMMode mode_{FSMMode::NORMAL};

        // State machine parameters
        std::vector<double> home_pos_;
        std::vector<double> rest_pos_;  // Rest pose configuration

        // Interface mapping
        std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>*>
        command_interface_map_ = {
            {"position", &ctrl_interfaces_.joint_position_command_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_command_interface_},
            {"effort", &ctrl_interfaces_.joint_force_command_interface_},
            {"kp", &ctrl_interfaces_.joint_kp_command_interface_},
            {"kd", &ctrl_interfaces_.joint_kd_command_interface_}
        };

        std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>*>
        state_interface_map_ = {
            {"position", &ctrl_interfaces_.joint_position_state_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_state_interface_},
            {"effort", &ctrl_interfaces_.joint_force_state_interface_}
        };

        // ROS subscriptions
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr fsm_command_subscription_;
        
        // CtrlComponent for OCS2 interface access
        std::shared_ptr<CtrlComponent> ctrl_comp_;
    };
}
