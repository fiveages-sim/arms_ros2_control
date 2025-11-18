//
// Created for Basic Joint Controller
//

#include "basic_joint_controller/BasicJointController.h"
#include "basic_joint_controller/FSM/StateHome.h"
#include "basic_joint_controller/FSM/StateHold.h"
#include "basic_joint_controller/FSM/StateMove.h"

#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>

namespace basic_joint_controller
{
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::InterfaceConfiguration BasicJointController::command_interface_configuration() const
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

    controller_interface::InterfaceConfiguration BasicJointController::state_interface_configuration() const
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

    controller_interface::return_type BasicJointController::update(const rclcpp::Time& time,
                                                                   const rclcpp::Duration& period)
    {
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

    controller_interface::CallbackReturn BasicJointController::on_init()
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

            // State machine parameters
            home_duration_ = auto_declare<double>("home_duration", 3.0);
            move_duration_ = auto_declare<double>("move_duration", 3.0);
            hold_position_threshold_ = auto_declare<double>("hold_position_threshold", 0.1);
            long switch_command_base = auto_declare<long>("switch_command_base", 100);

            // Get logger for FSM states
            rclcpp::Logger logger = get_node()->get_logger();

            // Create states
            // StateHome will load configurations via init() method using auto_declare
            state_list_.home = std::make_shared<StateHome>(ctrl_interfaces_, logger, home_duration_,
                                                           switch_command_base);

            // Initialize StateHome with configurations from parameters (home_1, home_2, home_3, etc.)
            // Pass auto_declare as a lambda to allow StateHome to use it
            state_list_.home->init([this](const std::string& name, const std::vector<double>& default_value)
            {
                return this->auto_declare<std::vector<double>>(name, default_value);
            });
            state_list_.hold = std::make_shared<StateHold>(ctrl_interfaces_, logger, hold_position_threshold_);
            state_list_.move = std::make_shared<StateMove>(ctrl_interfaces_, logger, move_duration_);

            return CallbackReturn::SUCCESS;
        }
        catch (const std::exception& e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    controller_interface::CallbackReturn BasicJointController::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // Subscribe to control input
        control_input_subscription_ = get_node()->create_subscription<arms_ros2_control_msgs::msg::Inputs>(
            "/control_input", 10, [this](const arms_ros2_control_msgs::msg::Inputs::SharedPtr msg)
            {
                ctrl_interfaces_.control_inputs_ = *msg;
            });

        // Subscribe to target position for Move state
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
                if (state_list_.move && !target_pos.empty())
                {
                    state_list_.move->setTargetPosition(target_pos);
                }
            });

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BasicJointController::on_activate(
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

        // Initialize FSM
        current_state_ = state_list_.hold;
        current_state_->enter();
        next_state_ = current_state_;
        next_state_name_ = current_state_->state_name;
        mode_ = FSMMode::NORMAL;

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BasicJointController::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BasicJointController::on_cleanup(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BasicJointController::on_error(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BasicJointController::on_shutdown(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    std::shared_ptr<FSMState> BasicJointController::getNextState(FSMStateName stateName) const
    {
        switch (stateName)
        {
        case FSMStateName::INVALID:
            return state_list_.invalid;
        case FSMStateName::HOME:
            return state_list_.home;
        case FSMStateName::HOLD:
            return state_list_.hold;
        case FSMStateName::MOVE:
            return state_list_.move;
        default:
            return state_list_.invalid;
        }
    }
} // namespace basic_joint_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(basic_joint_controller::BasicJointController, controller_interface::ControllerInterface);
