//
// Created for OCS2 Arm Controller
//

#ifndef OCS2ARMCONTROLLER_H
#define OCS2ARMCONTROLLER_H

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include <chrono>

#include <controller_interface/controller_interface.hpp>
#include <std_msgs/msg/string.hpp>
#include <control_input_msgs/msg/inputs.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

namespace ocs2_arm_controller {

// Forward declarations
class FSMState;
class StateHome;
class StateZero;

// FSM State names enum
enum class FSMStateName {
    INVALID,
    HOME,
    ZERO
};

// FSM Mode enum
enum class FSMMode {
    NORMAL,
    CHANGE
};

// Base FSM State class
class FSMState {
public:
    virtual ~FSMState() = default;

    FSMState(FSMStateName state_name, std::string state_name_string)
        : state_name(state_name), state_name_string(std::move(state_name_string)) {}

    virtual void enter() = 0;
    virtual void run(const rclcpp::Time& time, const rclcpp::Duration& period) = 0;
    virtual void exit() = 0;
    virtual FSMStateName checkChange() { return FSMStateName::INVALID; }

    FSMStateName state_name;
    std::string state_name_string;
};

// Control interfaces structure for arm control
struct CtrlInterfaces {
    // Command interfaces - only position control for arm
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> joint_position_command_interface_;

    // State interfaces - only position state for arm
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_position_state_interface_;

    control_input_msgs::msg::Inputs control_inputs_;
    int frequency_ = 1000;

    void clear() {
        joint_position_command_interface_.clear();
        joint_position_state_interface_.clear();
    }
};

struct FSMStateList {
    std::shared_ptr<FSMState> invalid;
    std::shared_ptr<StateHome> home;
    std::shared_ptr<StateZero> zero;
};

class Ocs2ArmController final : public controller_interface::ControllerInterface {
public:
    Ocs2ArmController() = default;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

protected:
    std::vector<std::string> joint_names_;
    std::vector<std::string> command_interface_types_;
    std::vector<std::string> state_interface_types_;
    std::string command_prefix_;
    
    // Target positions for the arm
    std::vector<double> home_pos_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> zero_pos_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    rclcpp::Subscription<control_input_msgs::msg::Inputs>::SharedPtr control_input_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;

    // Simplified interface maps for arm control
    std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>*> command_interface_map_ = {
        {"position", &ctrl_interfaces_.joint_position_command_interface_}
    };

    FSMMode mode_ = FSMMode::NORMAL;
    std::string state_name_;
    FSMStateName next_state_name_ = FSMStateName::INVALID;
    FSMStateList state_list_;
    std::shared_ptr<FSMState> current_state_;
    std::shared_ptr<FSMState> next_state_;

    std::chrono::time_point<std::chrono::steady_clock> last_update_time_;
    double update_frequency_;

    std::shared_ptr<FSMState> getNextState(FSMStateName stateName) const;

    std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>*> state_interface_map_ = {
        {"position", &ctrl_interfaces_.joint_position_state_interface_}
    };

    CtrlInterfaces ctrl_interfaces_;
};

} // namespace ocs2_arm_controller

#endif //OCS2ARMCONTROLLER_H 