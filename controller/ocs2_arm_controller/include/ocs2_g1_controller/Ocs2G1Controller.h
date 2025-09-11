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

#include "ocs2_arm_controller/Ocs2ArmController.h"
#include "ocs2_arm_controller/control/CtrlComponent.h"


namespace ocs2::mobile_manipulator
{


    class Ocs2G1Controller final : public controller_interface::ControllerInterface
    {
    public:
        Ocs2G1Controller() = default;
        ~Ocs2G1Controller() override = default;

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
            {"velocity", &ctrl_interfaces_.joint_velocity_command_interface_}, // CHENHZHU: Add velocity command interface
            {"effort", &ctrl_interfaces_.joint_force_command_interface_}, // CHENHZHU: Add force command interface
            {"kp", &ctrl_interfaces_.joint_kp_command_interface_}, // CHENHZHU: Add kp command interface
            {"kd", &ctrl_interfaces_.joint_kd_command_interface_}  // CHENHZHU: Add kd command interface
        };

        std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>*>
        state_interface_map_ = {
            {"position", &ctrl_interfaces_.joint_position_state_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_state_interface_},
            {"effort", &ctrl_interfaces_.joint_force_state_interface_} // CHENHZHU: Add force state interface if available
        };

        // ROS subscriptions
        rclcpp::Subscription<arms_ros2_control_msgs::msg::Inputs>::SharedPtr control_input_subscription_;
        
        // CtrlComponent for OCS2 interface access
        std::shared_ptr<CtrlComponent> ctrl_comp_;

        // CHENHZHU: Control Type: Force control or Position control
        std::string control_output_type_ = "position"; // Default to position control

        double kp_ = 50.0; // Proportional gain for position control
        double kd_ = 1.0;  // Derivative gain for position control
    };

}