//
// Created for Basic Joint Controller
//

#pragma once

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>

#include <controller_interface/controller_interface.hpp>
#include <arms_ros2_control_msgs/msg/inputs.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

namespace basic_joint_controller
{
    // Forward declarations
    class FSMState;
    class StateHome;
    class StateHold;
    class StateMove;

    // FSM State names enum
    enum class FSMStateName
    {
        INVALID,
        HOME,
        HOLD,
        MOVE
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

    // Control interfaces structure
    struct CtrlInterfaces
    {
        // Command interfaces - position control only
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_position_command_interface_;

        // State interfaces
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> 
        joint_position_state_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> 
        joint_velocity_state_interface_;

        arms_ros2_control_msgs::msg::Inputs control_inputs_;
        int frequency_ = 1000;

        void clear()
        {
            joint_position_command_interface_.clear();
            joint_position_state_interface_.clear();
            joint_velocity_state_interface_.clear();
        }
    };

    struct FSMStateList
    {
        std::shared_ptr<FSMState> invalid;
        std::shared_ptr<StateHome> home;
        std::shared_ptr<StateHold> hold;
        std::shared_ptr<StateMove> move;
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

        // ROS subscriptions
        rclcpp::Subscription<arms_ros2_control_msgs::msg::Inputs>::SharedPtr control_input_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_position_subscription_;
    };
}

