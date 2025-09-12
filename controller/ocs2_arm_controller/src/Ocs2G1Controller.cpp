#include "ocs2_g1_controller/Ocs2G1Controller.h"
#include "ocs2_arm_controller/FSM/StateHome.h"
#include "ocs2_arm_controller/FSM/StateOCS2.h"
#include "ocs2_arm_controller/FSM/StateHold.h"

#include <rclcpp/rclcpp.hpp>

namespace ocs2::mobile_manipulator
{
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::InterfaceConfiguration Ocs2G1Controller::command_interface_configuration() const
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

    controller_interface::InterfaceConfiguration Ocs2G1Controller::state_interface_configuration() const
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

    controller_interface::return_type Ocs2G1Controller::update(const rclcpp::Time& time,
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

    controller_interface::CallbackReturn Ocs2G1Controller::on_init()
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
            
            // CHENHZHU: Control type parameter
            control_output_type_ = auto_declare<std::string>("control_output_type", control_output_type_);
            if (control_output_type_ != "position" && control_output_type_ != "force" && control_output_type_ != "mixed")
            {
                RCLCPP_ERROR(get_node()->get_logger(), "Invalid control_type: %s. Must be 'position', 'force', or 'mixed'.", control_output_type_.c_str());
                return controller_interface::CallbackReturn::ERROR;
            }
            RCLCPP_INFO(get_node()->get_logger(), "Control type set to: %s", control_output_type_.c_str());
            
            // CHENHZHU: Control gains for position control:
            kp_ = auto_declare<double>("default_kp", kp_);
            kd_ = auto_declare<double>("default_kd", kd_);

            // Robot parameters - use robot_name to auto-generate package names
            auto_declare<std::string>("robot_name", "unitree_g1");
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

            // Create CtrlComponent (auto-initialize interface)
            ctrl_comp_ = std::make_shared<CtrlComponent>(get_node(), ctrl_interfaces_);

            // Create states
            state_list_.home = std::make_shared<StateHome>(ctrl_interfaces_, home_pos_);
            
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
            state_list_.hold = std::make_shared<StateHold>(ctrl_interfaces_);

            return CallbackReturn::SUCCESS;
        }
        catch (const std::exception& e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    controller_interface::CallbackReturn Ocs2G1Controller::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        control_input_subscription_ = get_node()->create_subscription<arms_ros2_control_msgs::msg::Inputs>(
            "/control_input", 10, [this](const arms_ros2_control_msgs::msg::Inputs::SharedPtr msg)
            {
                ctrl_interfaces_.control_inputs_.command = msg->command;
                ctrl_interfaces_.control_inputs_.lx = msg->lx;
                ctrl_interfaces_.control_inputs_.ly = msg->ly;
                ctrl_interfaces_.control_inputs_.rx = msg->rx;
                ctrl_interfaces_.control_inputs_.ry = msg->ry;
            });

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2G1Controller::on_activate(
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

        // CHENHZHU: Set control mode in ctrl_interfaces_
        if (control_output_type_ == "position")
        {
            ctrl_interfaces_.control_output_mode_ = ControlOutputMode::POSITION;
            RCLCPP_INFO(get_node()->get_logger(), "Control mode set to POSITION");
        }
        else if (control_output_type_ == "force")
        {
            ctrl_interfaces_.control_output_mode_ = ControlOutputMode::FORCE;
            RCLCPP_INFO(get_node()->get_logger(), "Control mode set to FORCE");
        }
        else if (control_output_type_ == "mixed")
        {
            ctrl_interfaces_.control_output_mode_ = ControlOutputMode::MIXED;
            RCLCPP_INFO(get_node()->get_logger(), "Control mode set to MIXED");
        }
        else
        {
            // This should not happen due to earlier checks, but just in case
            RCLCPP_ERROR(get_node()->get_logger(), "Invalid control_output_type: %s", control_output_type_.c_str());
            return CallbackReturn::ERROR;
        }
        
        // CHENHZHU: If position or mixed control, set gains
        if (ctrl_interfaces_.control_output_mode_ == ControlOutputMode::POSITION ||
            ctrl_interfaces_.control_output_mode_ == ControlOutputMode::MIXED)
        {
            // set kp, kd gains for all joints, using set_value to those command interfaces
            for (auto& kp_interface : ctrl_interfaces_.joint_kp_command_interface_)
            {
                kp_interface.get().set_value(kp_);
            }
            for (auto& kd_interface : ctrl_interfaces_.joint_kd_command_interface_)
            {
                kd_interface.get().set_value(kd_);
            }

        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2G1Controller::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2G1Controller::on_cleanup(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2G1Controller::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2G1Controller::on_shutdown(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    std::shared_ptr<FSMState> Ocs2G1Controller::getNextState(FSMStateName stateName) const
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
        default:
            return state_list_.invalid;
        }
    }
} // namespace mobile_manipulator

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ocs2::mobile_manipulator::Ocs2G1Controller, controller_interface::ControllerInterface);
