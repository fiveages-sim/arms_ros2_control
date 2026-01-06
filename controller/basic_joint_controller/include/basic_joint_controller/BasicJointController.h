//
// Created for Basic Joint Controller
//

#pragma once

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include <limits>
#include <algorithm>

#include <controller_interface/controller_interface.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

// Use common FSM and interfaces from arms_controller_common
#include <arms_controller_common/FSM/FSMState.h>
#include <arms_controller_common/FSM/StateHome.h>
#include <arms_controller_common/FSM/StateHold.h>
#include <arms_controller_common/CtrlInterfaces.h>
#include <arms_controller_common/utils/JointLimitsManager.h>

namespace basic_joint_controller
{
    // Use FSM types from arms_controller_common
    using FSMState = arms_controller_common::FSMState;
    using FSMStateName = arms_controller_common::FSMStateName;
    using FSMMode = arms_controller_common::FSMMode;
    using CtrlInterfaces = arms_controller_common::CtrlInterfaces;

    // Forward declarations
    class StateHold;  // Extended StateHold with MOVEJ transition
    class StateMoveJ;  // Extended StateMoveJ for basic_joint_controller

    struct FSMStateList
    {
        std::shared_ptr<FSMState> invalid;
        std::shared_ptr<arms_controller_common::StateHome> home;  // Use common StateHome
        std::shared_ptr<StateHold> hold;  // Extended StateHold for basic_joint_controller
        std::shared_ptr<StateMoveJ> movej;  // Extended StateMoveJ for basic_joint_controller
    };

    class BasicJointController final : public controller_interface::ControllerInterface
    {
    public:
        BasicJointController() = default;
        ~BasicJointController() override = default;

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

        // Hardware parameters
        std::string command_prefix_;
        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        // Control interfaces
        CtrlInterfaces ctrl_interfaces_;

        // State machine
        FSMStateList state_list_;
        std::shared_ptr<FSMState> current_state_;
        std::shared_ptr<FSMState> next_state_;
        FSMStateName next_state_name_{FSMStateName::INVALID};
        FSMMode mode_{FSMMode::NORMAL};

        // State machine parameters
        double home_duration_{3.0};
        double move_duration_{3.0};
        double hold_position_threshold_{0.1};

        // Interface mapping
        std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>*>
        command_interface_map_ = {
            {"position", &ctrl_interfaces_.joint_position_command_interface_}
        };

        std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>*>
        state_interface_map_ = {
            {"position", &ctrl_interfaces_.joint_position_state_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_state_interface_}
        };

        // Controller name
        std::string controller_name_;

        // Target command parameters
        bool target_command_enabled_{false};
        int32_t target_command_close_config_{1};
        int32_t target_command_open_config_{0};

        // ROS subscriptions
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr fsm_command_subscription_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr target_command_subscription_;

        // Joint limits manager (common utility)
        std::shared_ptr<arms_controller_common::JointLimitsManager> joint_limits_manager_;
    };
}

