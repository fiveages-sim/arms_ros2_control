//
// Created for OCS2 Arm Controller
//

#include "ocs2_arm_controller/Ocs2ArmController.h"
#include "ocs2_arm_controller/FSM/StateHome.h"
#include "ocs2_arm_controller/FSM/StateZero.h"
#include "ocs2_arm_controller/FSM/StateOCS2.h"
#include "ocs2_arm_controller/FSM/StateHold.h"

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
        // 发布末端执行器位置（无论当前状态如何）
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

        ctrl_comp_->publishEndEffectorPose(time);

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn Ocs2ArmController::on_init()
    {
        try
        {
            // 获取更新频率
            get_node()->get_parameter("update_rate", ctrl_interfaces_.frequency_);
            RCLCPP_INFO(get_node()->get_logger(), "Controller Manager Update Rate: %d Hz", ctrl_interfaces_.frequency_);

            // 硬件参数
            command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);
            joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
            command_interface_types_ =
                auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
            state_interface_types_ =
                auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

            // 机器人参数 - 使用robot_name自动生成包名
            robot_name_ = auto_declare<std::string>("robot_name", robot_name_);
            RCLCPP_INFO(get_node()->get_logger(), "Robot name: %s", robot_name_.c_str());

            // 控制输入参数
            control_input_name_ = auto_declare<std::string>("control_input_name", control_input_name_);
            control_input_interface_types_ =
                auto_declare<std::vector<std::string>>("control_input_interfaces", control_input_interface_types_);

            // 状态机参数
            home_pos_ = auto_declare<std::vector<double>>("home_pos", home_pos_);
            zero_pos_ = auto_declare<std::vector<double>>("zero_pos", zero_pos_);

            // 创建CtrlComponent（自动初始化接口）
            ctrl_comp_ = std::make_shared<CtrlComponent>(get_node(), ctrl_interfaces_);

            // 创建状态
            state_list_.home = std::make_shared<StateHome>(ctrl_interfaces_, home_pos_);
            state_list_.zero = std::make_shared<StateZero>(ctrl_interfaces_, zero_pos_);
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

    controller_interface::CallbackReturn Ocs2ArmController::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        control_input_subscription_ = get_node()->create_subscription<control_input_msgs::msg::Inputs>(
            "/control_input", 10, [this](const control_input_msgs::msg::Inputs::SharedPtr msg)
            {
                ctrl_interfaces_.control_inputs_.command = msg->command;
                ctrl_interfaces_.control_inputs_.lx = msg->lx;
                ctrl_interfaces_.control_inputs_.ly = msg->ly;
                ctrl_interfaces_.control_inputs_.rx = msg->rx;
                ctrl_interfaces_.control_inputs_.ry = msg->ry;
            });

        RCLCPP_INFO(get_node()->get_logger(), "End effector pose publisher created for %s_end_effector_pose topic",
                    robot_name_.c_str());

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

        // Initialize FSM
        current_state_ = state_list_.home;
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
        case FSMStateName::ZERO:
            return state_list_.zero;
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
PLUGINLIB_EXPORT_CLASS(ocs2::mobile_manipulator::Ocs2ArmController, controller_interface::ControllerInterface);
