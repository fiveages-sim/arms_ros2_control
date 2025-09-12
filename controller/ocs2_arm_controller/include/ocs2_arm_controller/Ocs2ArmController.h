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
#include <std_msgs/msg/string.hpp>
#include <arms_ros2_control_msgs/msg/inputs.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include "ocs2_arm_controller/control/CtrlComponent.h"

namespace ocs2::mobile_manipulator
{
    // Forward declarations
    class FSMState;
    class StateHome;
    class StateOCS2;
    class StateHold;

    // FSM State names enum
    enum class FSMStateName
    {
        INVALID,
        HOME,
        OCS2, // OCS2 MPC control state
        HOLD // Hold current position state
    };

    // FSM Mode enum
    enum class FSMMode
    {
        NORMAL,
        CHANGE
    };

    // Base FSM State class
    class FSMState
    {
    public:
        virtual ~FSMState() = default;

        FSMState(FSMStateName state_name, std::string state_name_string)
            : state_name(state_name), state_name_string(std::move(state_name_string))
        {
        }

        virtual void enter() = 0;
        virtual void run(const rclcpp::Time& time, const rclcpp::Duration& period) = 0;
        virtual void exit() = 0;
        virtual FSMStateName checkChange() { return FSMStateName::INVALID; }

        FSMStateName state_name;
        std::string state_name_string;
    };

    // Control mode enum for automatic detection
    enum class ControlMode
    {
        POSITION,    // Position control only
        FORCE,       // Force control with kp, kd, velocity, effort, position
        AUTO         // Automatic detection based on available interfaces
    };

    // Control interfaces structure for arm control
    struct CtrlInterfaces
    {
        // Command interfaces - only position control for arm
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_position_command_interface_;

        // CHENHZHU: Add force, velocity, kp, kq command interface for force and mixed control
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_force_command_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_velocity_command_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_kp_command_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_kd_command_interface_;

        // State interfaces - position and velocity state for arm
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_position_state_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_velocity_state_interface_;

        // CHENHZHU: Add force state interface if available
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_force_state_interface_;

        arms_ros2_control_msgs::msg::Inputs control_inputs_;
        int frequency_ = 1000;

        // Control mode - automatically detected based on available interfaces
        ControlMode control_mode_ = ControlMode::POSITION;  // Will be set during initialization
        bool mode_detected_ = false;  // Flag to ensure detection only happens once

        // Auto mode detection function - called once during initialization
        void detectAndSetControlMode()
        {
            if (mode_detected_) return;  // Only detect once

            // Check if command interfaces include kp, kd, velocity, effort, position
            bool has_kp_cmd = !joint_kp_command_interface_.empty();
            bool has_kd_cmd = !joint_kd_command_interface_.empty();
            bool has_velocity_cmd = !joint_velocity_command_interface_.empty();
            bool has_effort_cmd = !joint_force_command_interface_.empty();
            bool has_position_cmd = !joint_position_command_interface_.empty();

            // Check if state interfaces include velocity, effort, position
            bool has_velocity_state = !joint_velocity_state_interface_.empty();
            bool has_effort_state = !joint_force_state_interface_.empty();
            bool has_position_state = !joint_position_state_interface_.empty();

            // Enable force control mode if all required interfaces are available
            bool command_has_force_control = has_kp_cmd && has_kd_cmd && has_velocity_cmd && has_effort_cmd && has_position_cmd;
            bool state_has_force_control = has_velocity_state && has_effort_state && has_position_state;

            if (command_has_force_control && state_has_force_control)
            {
                control_mode_ = ControlMode::FORCE;
            }
            else
            {
                control_mode_ = ControlMode::POSITION;
            }

            mode_detected_ = true;
        }

        // Force control gains [kp, kd]
        std::vector<double> default_gains_;

        void clear()
        {
            joint_position_command_interface_.clear();
            joint_position_state_interface_.clear();
            joint_velocity_state_interface_.clear();
            joint_force_command_interface_.clear();
            joint_velocity_command_interface_.clear();
            joint_kp_command_interface_.clear();
            joint_kd_command_interface_.clear();
            joint_force_state_interface_.clear();
        }
    };

    struct FSMStateList
    {
        std::shared_ptr<FSMState> invalid;
        std::shared_ptr<StateHome> home;
        std::shared_ptr<StateOCS2> ocs2; // OCS2 state
        std::shared_ptr<StateHold> hold; // Hold position state
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
        rclcpp::Subscription<arms_ros2_control_msgs::msg::Inputs>::SharedPtr control_input_subscription_;
        
        // CtrlComponent for OCS2 interface access
        std::shared_ptr<CtrlComponent> ctrl_comp_;
    };
}
