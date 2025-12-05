//
// Created for OCS2 Arm Controller
//

#include "ocs2_arm_controller/Ocs2ArmController.h"
#include "ocs2_arm_controller/FSM/StateHome.h"
#include "ocs2_arm_controller/FSM/StateOCS2.h"
#include "ocs2_arm_controller/FSM/StateHold.h"
#include "ocs2_arm_controller/FSM/StateMoveJ.h"
#include <arms_controller_common/utils/GravityCompensation.h>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <rclcpp/rclcpp.hpp>

namespace ocs2::mobile_manipulator
{
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::InterfaceConfiguration Ocs2ArmController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : command_interface_types_)
            {
                if (!command_prefix_.empty())
                {
                    conf.names.push_back(command_prefix_ + "/" + joint_name + "/" + interface_type);
                }
                else
                {
                    conf.names.push_back(joint_name + "/" + interface_type);
                }
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration Ocs2ArmController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : state_interface_types_)
            {
                conf.names.push_back(joint_name + "/" + interface_type);
            }
        }

        return conf;
    }

    controller_interface::return_type Ocs2ArmController::update(const rclcpp::Time& time,
                                                                const rclcpp::Duration& period)
    {
        // Publish end effector pose (regardless of current state)
        ctrl_comp_->updateObservation(time);

        if (mode_ == FSMMode::NORMAL)
        {
            current_state_->run(time, period);
            next_state_name_ = current_state_->checkChange();
            if (next_state_name_ != current_state_->state_name)
            {
                mode_ = FSMMode::CHANGE;
                next_state_ = getNextState(next_state_name_);
                RCLCPP_INFO(get_node()->get_logger(), "Switched from %s to %s",
                            current_state_->state_name_string.c_str(), next_state_->state_name_string.c_str());
            }
        }
        else if (mode_ == FSMMode::CHANGE)
        {
            current_state_->exit();
            current_state_ = next_state_;
            current_state_->enter();
            mode_ = FSMMode::NORMAL;
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn Ocs2ArmController::on_init()
    {
        try
        {
            // Get update frequency
            get_node()->get_parameter("update_rate", ctrl_interfaces_.frequency_);
            RCLCPP_INFO(get_node()->get_logger(), "Controller Manager Update Rate: %d Hz", ctrl_interfaces_.frequency_);

            // Hardware parameters
            command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);
            joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
            command_interface_types_ =
                auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
            state_interface_types_ =
                auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

            // Robot parameters - use robot_name to auto-generate package names
            auto_declare<std::string>("robot_name", "cr5");
            auto_declare<std::string>("robot_type", "");  // Optional robot type/variant
            auto_declare<double>("future_time_offset", 1.0);
            auto_declare<std::string>("info_file_name", "task");

            // Control input parameters
            control_input_name_ = auto_declare<std::string>("control_input_name", control_input_name_);
            control_input_interface_types_ =
                auto_declare<std::vector<std::string>>("control_input_interfaces", control_input_interface_types_);

            // State machine parameters
            home_pos_ = auto_declare<std::vector<double>>("home_pos", home_pos_);
            rest_pos_ = auto_declare<std::vector<double>>("rest_pos", rest_pos_);
            
            // Force control parameters
            ctrl_interfaces_.default_gains_ = auto_declare<std::vector<double>>("default_gains", ctrl_interfaces_.default_gains_);
            
            // PD control parameters (for OCS2 state)
            ctrl_interfaces_.pd_gains_ = auto_declare<std::vector<double>>("pd_gains", ctrl_interfaces_.pd_gains_);

            // MPC frequency parameter - default value 0 if not set
            auto_declare<int>("mpc_frequency", 0);

            // Hold state parameters
            auto_declare<double>("hold_position_threshold", 0.1);  // Default: 0.1 rad (~5.7 degrees)

            // Home state parameters
            double home_duration = auto_declare<double>("home_duration", 3.0);  // Default: 3.0 seconds

            // Create CtrlComponent (auto-initialize interface)
            ctrl_comp_ = std::make_shared<CtrlComponent>(get_node(), ctrl_interfaces_);

            // Create GravityCompensation from CtrlComponent's Pinocchio model
            std::shared_ptr<arms_controller_common::GravityCompensation> gravity_compensation = nullptr;
            if (ctrl_comp_->interface_)
            {
                const auto& pinocchio_model = ctrl_comp_->interface_->getPinocchioInterface().getModel();
                gravity_compensation = std::make_shared<arms_controller_common::GravityCompensation>(pinocchio_model);
                RCLCPP_INFO(get_node()->get_logger(), 
                            "Gravity compensation initialized from OCS2 Pinocchio model");
            }

            // Get logger for FSM states
            rclcpp::Logger logger = get_node()->get_logger();

            // Create StateHome using common implementation
            state_list_.home = std::make_shared<StateHome>(
                ctrl_interfaces_, logger, home_duration, gravity_compensation);
            
            // Set home position
            if (!home_pos_.empty())
            {
                state_list_.home->setHomePosition(home_pos_);
                RCLCPP_INFO(get_node()->get_logger(), 
                            "Home position configured with %zu joints", home_pos_.size());
            }
            
            // Configure rest pose if available
            if (!rest_pos_.empty())
            {
                state_list_.home->setRestPose(rest_pos_);
                RCLCPP_INFO(get_node()->get_logger(), 
                            "Rest pose configured with %zu joints", rest_pos_.size());
            }
            else
            {
                RCLCPP_INFO(get_node()->get_logger(), "No rest pose configured, using home pose only");
            }
            
            state_list_.ocs2 = std::make_shared<StateOCS2>(ctrl_interfaces_, get_node(), ctrl_comp_);
            
            // Hold state parameters
            double hold_position_threshold = auto_declare<double>("hold_position_threshold", 0.1);
            
            // Create StateHold using common implementation
            state_list_.hold = std::make_shared<StateHold>(
                ctrl_interfaces_, logger, hold_position_threshold, gravity_compensation);

            // MoveJ state parameters
            double move_duration = auto_declare<double>("move_duration", 3.0);
            
            // Create StateMoveJ using common implementation
            state_list_.movej = std::make_shared<StateMoveJ>(
                ctrl_interfaces_, logger, move_duration, gravity_compensation);

            return CallbackReturn::SUCCESS;
        }
        catch (const std::exception& e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    controller_interface::CallbackReturn Ocs2ArmController::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        control_input_subscription_ = get_node()->create_subscription<arms_ros2_control_msgs::msg::Inputs>(
            "/control_input", 10, [this](const arms_ros2_control_msgs::msg::Inputs::SharedPtr msg)
            {
                ctrl_interfaces_.control_inputs_ = *msg;
            });

        // Subscribe to target position for MoveJ state
        target_position_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
            get_node()->get_name() + std::string("/target_joint_position"), 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
            {
                // Convert Float64MultiArray message to vector of joint positions
                std::vector<double> target_pos;
                for (const auto& val : msg->data)
                {
                    target_pos.push_back(val);
                }
                if (state_list_.movej && !target_pos.empty())
                {
                    state_list_.movej->setTargetPosition(target_pos);
                }
            });

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2ArmController::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // clear out vectors in case of restart
        ctrl_interfaces_.clear();

        // assign command interfaces
        for (auto& interface : command_interfaces_)
        {
            std::string interface_name = interface.get_interface_name();
            if (const size_t pos = interface_name.find('/'); pos != std::string::npos)
            {
                command_interface_map_[interface_name.substr(pos + 1)]->push_back(interface);
            }
            else
            {
                command_interface_map_[interface_name]->push_back(interface);
            }
        }

        // assign state interfaces
        for (auto& interface : state_interfaces_)
        {
            state_interface_map_[interface.get_interface_name()]->push_back(interface);
        }

        // Auto-detect control mode based on available interfaces
        ctrl_interfaces_.detectAndSetControlMode();
        
        // Log detected control mode
        if (ctrl_interfaces_.control_mode_ == ControlMode::MIX)
        {
            RCLCPP_INFO(get_node()->get_logger(), "Mixed control mode enabled - detected kp, kd, velocity, effort, position interfaces");
        }
        else
        {
            RCLCPP_INFO(get_node()->get_logger(), "Position control mode enabled - standard position control interfaces detected");
        }

        // Initialize FSM
        current_state_ = state_list_.hold;
        current_state_->enter();
        next_state_ = current_state_;
        next_state_name_ = current_state_->state_name;
        mode_ = FSMMode::NORMAL;

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2ArmController::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2ArmController::on_cleanup(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2ArmController::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2ArmController::on_shutdown(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    std::shared_ptr<FSMState> Ocs2ArmController::getNextState(FSMStateName stateName) const
    {
        switch (stateName)
        {
        case FSMStateName::INVALID:
            return state_list_.invalid;
        case FSMStateName::HOME:
            return state_list_.home;
        case FSMStateName::OCS2:
            return state_list_.ocs2;
        case FSMStateName::HOLD:
            return state_list_.hold;
        case FSMStateName::MOVEJ:
            return state_list_.movej;
        default:
            return state_list_.invalid;
        }
    }
} // namespace mobile_manipulator

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ocs2::mobile_manipulator::Ocs2ArmController, controller_interface::ControllerInterface);
