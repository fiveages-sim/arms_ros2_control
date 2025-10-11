#include "adaptive_gripper_controller/adaptive_gripper_controller.h"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace adaptive_gripper_controller
{
    AdaptiveGripperController::AdaptiveGripperController()
    {
        gripper_interfaces_.clear();
    }

    controller_interface::CallbackReturn AdaptiveGripperController::on_init()
    {
        joint_name_ = auto_declare<std::string>("joint", "gripper_joint");
        force_threshold_ = auto_declare<double>("force_threshold", 0.1);
        force_feedback_ratio_ = auto_declare<double>("force_feedback_ratio", 0.5);
        target_position_ = 0.0;

        // 根据关节名称自动判断手臂标识
        if (joint_name_.find("left") != std::string::npos) {
            arm_id_ = 1;  // 左臂
        } else if (joint_name_.find("right") != std::string::npos) {
            arm_id_ = 2;  // 右臂
        } else {
            arm_id_ = 1;  // 默认为左臂（单臂机器人）
        }

        RCLCPP_INFO(get_node()->get_logger(),
                    "Simple gripper controller initialized for joint: %s, arm_id: %d, force threshold: %.3f, force feedback ratio: %.3f",
                    joint_name_.c_str(), arm_id_, force_threshold_, force_feedback_ratio_);

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn AdaptiveGripperController::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        gripper_subscription_ = get_node()->create_subscription<arms_ros2_control_msgs::msg::Gripper>(
            "/gripper_command", 10, [this](const arms_ros2_control_msgs::msg::Gripper::SharedPtr msg)
            {
                // 检查消息是否发给此控制器
                if (msg->arm_id != arm_id_) {
                    RCLCPP_DEBUG(get_node()->get_logger(),
                                "Ignoring gripper command: msg_arm_id=%d, my_arm_id=%d",
                                msg->arm_id, arm_id_);
                    return;  // 不是给此手臂的命令，忽略
                }

                gripper_target_ = msg->target;
                gripper_direction_ = msg->direction;

                // 收到新命令时立即计算目标位置
                process_gripper_command();

                RCLCPP_INFO(get_node()->get_logger(),
                            "Received gripper command for arm %d: target=%d, direction=%d, calculated target position: %.6f",
                            arm_id_, gripper_target_, gripper_direction_, target_position_);
            });

        robot_description_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                RCLCPP_INFO(get_node()->get_logger(), "Received robot description, parsing joint limits...");
                parse_joint_limits(msg->data);
            });

        // 移除 control_input 订阅 - gripper_command 中已包含 arm_id 信息

        RCLCPP_INFO(get_node()->get_logger(), "Simple gripper controller configured");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn AdaptiveGripperController::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // 查找位置命令接口
        auto command_interface_it = std::find_if(
            command_interfaces_.begin(), command_interfaces_.end(),
            [](const hardware_interface::LoanedCommandInterface& command_interface)
            {
                return command_interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
            });

        if (command_interface_it == command_interfaces_.end())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Expected 1 position command interface");
            return controller_interface::CallbackReturn::ERROR;
        }

        if (command_interface_it->get_prefix_name() != joint_name_)
        {
            RCLCPP_ERROR(
                get_node()->get_logger(), "Command interface is different than joint name `%s` != `%s`",
                command_interface_it->get_prefix_name().c_str(), joint_name_.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }

        // 查找位置状态接口
        auto position_state_interface_it = std::find_if(
            state_interfaces_.begin(), state_interfaces_.end(),
            [](const hardware_interface::LoanedStateInterface& state_interface)
            {
                return state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
            });

        if (position_state_interface_it == state_interfaces_.end())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Expected 1 position state interface");
            return controller_interface::CallbackReturn::ERROR;
        }

        if (position_state_interface_it->get_prefix_name() != joint_name_)
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Position state interface is different than joint name `%s` != `%s`",
                position_state_interface_it->get_prefix_name().c_str(), joint_name_.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }

        // 查找力反馈状态接口（可选）
        auto effort_state_interface_it = std::find_if(
            state_interfaces_.begin(), state_interfaces_.end(),
            [](const hardware_interface::LoanedStateInterface& state_interface)
            {
                return state_interface.get_interface_name() == hardware_interface::HW_IF_EFFORT;
            });

        if (effort_state_interface_it != state_interfaces_.end())
        {
            if (effort_state_interface_it->get_prefix_name() == joint_name_)
            {
                gripper_interfaces_.effort_state_interface_ = *effort_state_interface_it;
                has_effort_interface_ = true;
                RCLCPP_INFO(get_node()->get_logger(), "Effort feedback interface found for joint: %s",
                            joint_name_.c_str());
            }
        }
        else
        {
            has_effort_interface_ = false;
            RCLCPP_INFO(get_node()->get_logger(), "No effort feedback interface available for joint: %s",
                        joint_name_.c_str());
        }

        // 查找 reverse_flag 状态接口（可选）
        auto reverse_flag_state_interface_it = std::find_if(
            state_interfaces_.begin(), state_interfaces_.end(),
            [](const hardware_interface::LoanedStateInterface& state_interface)
            {
                return state_interface.get_interface_name() == "reverse_flag";
            });

        if (reverse_flag_state_interface_it != state_interfaces_.end())
        {
            if (reverse_flag_state_interface_it->get_prefix_name() == joint_name_)
            {
                gripper_interfaces_.reverse_flag_state_interface_ = *reverse_flag_state_interface_it;
                RCLCPP_INFO(get_node()->get_logger(), "Reverse flag interface found for joint: %s",
                            joint_name_.c_str());
            }
        }
        else
        {
            RCLCPP_INFO(get_node()->get_logger(), "No reverse flag interface available for joint: %s (using default behavior)",
                        joint_name_.c_str());
        }

        // 保存接口引用
        gripper_interfaces_.position_command_interface_ = *command_interface_it;
        gripper_interfaces_.position_state_interface_ = *position_state_interface_it;

        // 启动时自动设置为关闭状态
        // 读取 reverse_flag 来确定关闭位置
        double reverse_flag = 1.0;
        if (gripper_interfaces_.reverse_flag_state_interface_.has_value())
        {
            reverse_flag = gripper_interfaces_.reverse_flag_state_interface_->get().get_value();
        }

        // reverse_flag = 1: 关闭位置是 0 (lower)
        // reverse_flag = -1: 关闭位置是 1 (upper)
        if (reverse_flag < 0)
        {
            target_position_ = 1.0;  // 反转时，关闭位置是 upper
        }
        else
        {
            target_position_ = 0.0;  // 正常时，关闭位置是 lower
        }

        RCLCPP_INFO(get_node()->get_logger(),
                    "Initialized to closed position: %.6f (reverse_flag: %.1f)",
                    target_position_, reverse_flag);
        RCLCPP_INFO(get_node()->get_logger(), "Simple gripper controller activated");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn AdaptiveGripperController::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        gripper_interfaces_.clear();

        RCLCPP_INFO(get_node()->get_logger(), "Simple gripper controller deactivated");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type AdaptiveGripperController::update(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        // 读取当前位置
        double current_position = gripper_interfaces_.position_state_interface_->get().get_value();

        if (has_effort_interface_)
        {
            double current_effort = gripper_interfaces_.effort_state_interface_->get().get_value();
            if (gripper_target_ == 0 && !force_threshold_triggered_ && std::abs(current_effort) > force_threshold_)
            {
                // 根据比例参数计算目标位置
                // force_feedback_ratio_ = 0.0: 保持在当前位置
                // force_feedback_ratio_ = 1.0: 完全移动到目标关闭位置
                double original_target = target_position_;
                double distance_to_target = original_target - current_position;
                double target_offset = distance_to_target * force_feedback_ratio_;
                target_position_ = current_position + target_offset;
                
                force_threshold_triggered_ = true; // 设置已触发标志
                RCLCPP_INFO(get_node()->get_logger(),
                            "Force threshold triggered for closing target, moving %.1f%% toward target position: %.6f (current: %.6f, original target: %.6f)",
                            force_feedback_ratio_ * 100.0, target_position_, current_position, original_target);
            }
        }


        // 输出位置命令
        if (gripper_interfaces_.position_command_interface_.has_value())
        {
            if (!gripper_interfaces_.position_command_interface_->get().set_value(target_position_))
            {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to set position command to: %f", target_position_);
            }
        }

        // 输出调试信息
        RCLCPP_DEBUG(get_node()->get_logger(),
                     "Current position: %f, Target position: %f", current_position, target_position_);

        return controller_interface::return_type::OK;
    }


    void AdaptiveGripperController::process_gripper_command()
    {
        // 如果没有初始化夹爪范围，直接返回
        if (!limits_initialized_)
        {
            RCLCPP_WARN(get_node()->get_logger(),
                        "Gripper limits not initialized yet, skipping command processing");
            return;
        }

        // 读取 reverse_flag，默认值为 1.0（不反转）
        double reverse_flag = 1.0;
        if (gripper_interfaces_.reverse_flag_state_interface_.has_value())
        {
            reverse_flag = gripper_interfaces_.reverse_flag_state_interface_->get().get_value();
        }

        // 根据 reverse_flag 决定上下限
        double upper_limit = (reverse_flag < 0) ? joint_lower_limit_ : joint_upper_limit_;
        double lower_limit = (reverse_flag < 0) ? joint_upper_limit_ : joint_lower_limit_;

        if (gripper_target_ == 1)
        {
            force_threshold_triggered_ = false;
            target_position_ = upper_limit;
            RCLCPP_DEBUG(get_node()->get_logger(), "Opening gripper to position: %f (reverse_flag: %f)",
                        target_position_, reverse_flag);
        }
        else
        {
            target_position_ = lower_limit;
            RCLCPP_DEBUG(get_node()->get_logger(), "Closing gripper to position: %f (reverse_flag: %f)",
                        target_position_, reverse_flag);
        }
    }


    void AdaptiveGripperController::parse_joint_limits(const std::string& robot_description)
    {
        try
        {
            // 简单的XML解析，查找指定关节的限制
            size_t joint_pos = robot_description.find("<joint name=\"" + joint_name_ + "\"");
            if (joint_pos == std::string::npos)
            {
                RCLCPP_WARN(get_node()->get_logger(), "Joint %s not found in robot description", joint_name_.c_str());
                return;
            }

            // 查找limit标签
            size_t limit_pos = robot_description.find("<limit", joint_pos);
            if (limit_pos == std::string::npos)
            {
                RCLCPP_WARN(get_node()->get_logger(), "No limits found for joint %s", joint_name_.c_str());
                return;
            }

            // 解析upper和lower限制
            size_t upper_pos = robot_description.find("upper=\"", limit_pos);
            size_t lower_pos = robot_description.find("lower=\"", limit_pos);

            if (upper_pos != std::string::npos && lower_pos != std::string::npos)
            {
                upper_pos += 7; // 跳过 "upper="
                lower_pos += 7; // 跳过 "lower="

                const size_t upper_end = robot_description.find("\"", upper_pos);
                const size_t lower_end = robot_description.find("\"", lower_pos);

                if (upper_end != std::string::npos && lower_end != std::string::npos)
                {
                    std::string upper_str = robot_description.substr(upper_pos, upper_end - upper_pos);
                    std::string lower_str = robot_description.substr(lower_pos, lower_end - lower_pos);

                    joint_upper_limit_ = std::stod(upper_str);
                    joint_lower_limit_ = std::stod(lower_str);
                    limits_initialized_ = true;

                    RCLCPP_INFO(get_node()->get_logger(),
                                "Successfully parsed joint limits for %s: lower=%.6f, upper=%.6f",
                                joint_name_.c_str(), joint_lower_limit_, joint_upper_limit_);
                }
            }
            else
            {
                RCLCPP_WARN(get_node()->get_logger(), "Missing upper or lower limit attributes");
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Error parsing joint limits: %s", e.what());
        }
    }

    controller_interface::InterfaceConfiguration
    AdaptiveGripperController::command_interface_configuration() const
    {
        std::vector names = {joint_name_ + "/" + hardware_interface::HW_IF_POSITION};
        return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
    }

    controller_interface::InterfaceConfiguration
    AdaptiveGripperController::state_interface_configuration() const
    {
        std::vector names = {
            joint_name_ + "/" + hardware_interface::HW_IF_POSITION,
            joint_name_ + "/" + hardware_interface::HW_IF_EFFORT,
            joint_name_ + "/reverse_flag"
        };
        return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
    }
} // namespace adaptive_gripper_controller

PLUGINLIB_EXPORT_CLASS(
    adaptive_gripper_controller::AdaptiveGripperController,
    controller_interface::ControllerInterface)
