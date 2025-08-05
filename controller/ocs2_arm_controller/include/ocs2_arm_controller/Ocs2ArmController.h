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
#include <control_input_msgs/msg/inputs.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ocs2_core/Types.h>
#include "ocs2_arm_controller/control/CtrlComponent.h"

namespace ocs2::mobile_manipulator
{
    // Forward declarations
    class FSMState;
    class StateHome;
    class StateZero;
    class StateOCS2;
    class StateHold;

    // FSM State names enum
    enum class FSMStateName
    {
        INVALID,
        HOME,
        ZERO,
        OCS2, // OCS2 MPC控制状态
        HOLD // 保持当前位置状态
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

    // Control interfaces structure for arm control
    struct CtrlInterfaces
    {
        // Command interfaces - only position control for arm
        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_position_command_interface_;

        // State interfaces - position and velocity state for arm
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_position_state_interface_;
        std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_velocity_state_interface_;

        control_input_msgs::msg::Inputs control_inputs_;
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
        std::shared_ptr<StateZero> zero;
        std::shared_ptr<StateOCS2> ocs2; // OCS2状态
        std::shared_ptr<StateHold> hold; // 保持位置状态
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

        // 硬件参数
        std::string command_prefix_;
        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        // 机器人参数
        std::string robot_name_{"cr5"}; // 默认机器人名称

        // 控制输入参数
        std::string control_input_name_;
        std::vector<std::string> control_input_interface_types_;

        // 控制接口
        CtrlInterfaces ctrl_interfaces_;

        // 状态机
        FSMStateList state_list_;
        std::shared_ptr<FSMState> current_state_;
        std::shared_ptr<FSMState> next_state_;
        FSMStateName next_state_name_{FSMStateName::INVALID};
        FSMMode mode_{FSMMode::NORMAL};

        // 状态机参数
        std::vector<double> home_pos_;
        std::vector<double> zero_pos_;

        // 接口映射
        std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>*>
        command_interface_map_ = {
            {"position", &ctrl_interfaces_.joint_position_command_interface_}
        };

        std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>*>
        state_interface_map_ = {
            {"position", &ctrl_interfaces_.joint_position_state_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_state_interface_}
        };

        // ROS订阅
        rclcpp::Subscription<control_input_msgs::msg::Inputs>::SharedPtr control_input_subscription_;
        
        // CtrlComponent for OCS2 interface access
        std::shared_ptr<CtrlComponent> ctrl_comp_;
    };
}
