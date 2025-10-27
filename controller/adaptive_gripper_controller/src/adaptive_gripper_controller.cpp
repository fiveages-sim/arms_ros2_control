#include "adaptive_gripper_controller/adaptive_gripper_controller.h"

#include <algorithm>
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

        // 力反馈相关参数
        use_effort_interface_ = auto_declare<bool>("use_effort_interface", true);
        force_threshold_ = auto_declare<double>("force_threshold", 0.1);
        force_feedback_ratio_ = auto_declare<double>("force_feedback_ratio", 0.5);

        target_position_ = 0.0;

        // 根据关节名称自动判断手臂标识
        if (joint_name_.find("left") != std::string::npos)
        {
            arm_id_ = 1; // 左臂
        }
        else if (joint_name_.find("right") != std::string::npos)
        {
            arm_id_ = 2; // 右臂
        }
        else
        {
            arm_id_ = 1; // 默认为左臂（单臂机器人）
        }

        RCLCPP_INFO(get_node()->get_logger(),
                    "Adaptive Gripper Controller initialized for joint: %s, arm_id: %d, use_effort: %s, force threshold: %.3f, force feedback ratio: %.3f",
                    joint_name_.c_str(), arm_id_, use_effort_interface_ ? "true" : "false", force_threshold_,
                    force_feedback_ratio_);

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn AdaptiveGripperController::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // 构建需要的状态接口类型列表
        available_state_interface_types_.clear();
        available_state_interface_types_.emplace_back(hardware_interface::HW_IF_POSITION); // position 是必需的

        // 根据参数决定是否使用 effort 接口
        if (use_effort_interface_)
        {
            available_state_interface_types_.emplace_back(hardware_interface::HW_IF_EFFORT);
            RCLCPP_INFO(get_node()->get_logger(),
                        "Effort interface enabled (adaptive force feedback)");
        }
        else
        {
            RCLCPP_INFO(get_node()->get_logger(),
                        "Effort interface disabled (position-only mode)");
        }

        gripper_subscription_ = get_node()->create_subscription<arms_ros2_control_msgs::msg::Gripper>(
            "/gripper_command", 10, [this](const arms_ros2_control_msgs::msg::Gripper::SharedPtr msg)
            {
                // 检查消息是否发给此控制器
                if (msg->arm_id != arm_id_)
                {
                    RCLCPP_DEBUG(get_node()->get_logger(),
                                 "Ignoring gripper command: msg_arm_id=%d, my_arm_id=%d",
                                 msg->arm_id, arm_id_);
                    return; // 不是给此手臂的命令，忽略
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
                parse_joint_limits(msg->data);
                parse_initial_value(msg->data);
            });

        // 直接位置控制订阅器 - 无力反馈的精细控制
        // 话题名称格式：/<joint_name>/position_command
        std::string position_command_topic = "/" + joint_name_ + "/position_command";
        direct_position_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64>(
            position_command_topic, 10, [this](const std_msgs::msg::Float64::SharedPtr msg)
            {
                // 如果限位还未初始化，不处理位置指令
                if (!limits_initialized_)
                {
                    RCLCPP_WARN(get_node()->get_logger(),
                                "Joint limits not initialized yet, ignoring position command %.6f",
                                msg->data);
                    return;
                }

                // 切换到直接位置控制模式（无力反馈）
                direct_position_mode_ = true;
                force_threshold_triggered_ = false; // 重置力反馈触发标志

                // 限制目标位置在关节限位范围内
                double commanded_position = msg->data;
                target_position_ = std::clamp(commanded_position, joint_lower_limit_, joint_upper_limit_);

                // 只在位置被截断时输出警告
                if (target_position_ != commanded_position)
                {
                    RCLCPP_WARN(get_node()->get_logger(),
                                "Position command %.6f clamped to [%.6f, %.6f], using %.6f",
                                commanded_position, joint_lower_limit_, joint_upper_limit_, target_position_);
                }
            });

        RCLCPP_INFO(get_node()->get_logger(),
                    "Direct position control subscribed to topic: %s", position_command_topic.c_str());

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn AdaptiveGripperController::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // 查找位置命令接口（必须是指定关节的）
        auto command_interface_it = std::find_if(
            command_interfaces_.begin(), command_interfaces_.end(),
            [this](const hardware_interface::LoanedCommandInterface& command_interface)
            {
                return command_interface.get_interface_name() == hardware_interface::HW_IF_POSITION &&
                    command_interface.get_prefix_name() == joint_name_;
            });

        if (command_interface_it == command_interfaces_.end())
        {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Expected position command interface for joint: %s",
                         joint_name_.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }

        // 查找位置状态接口（必须是指定关节的）
        auto position_state_interface_it = std::find_if(
            state_interfaces_.begin(), state_interfaces_.end(),
            [this](const hardware_interface::LoanedStateInterface& state_interface)
            {
                return state_interface.get_interface_name() == hardware_interface::HW_IF_POSITION &&
                    state_interface.get_prefix_name() == joint_name_;
            });

        if (position_state_interface_it == state_interfaces_.end())
        {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Expected position state interface for joint: %s",
                         joint_name_.c_str());
            return controller_interface::CallbackReturn::ERROR;
        }

        // 如果配置中包含 effort 接口，则查找并绑定
        auto it = std::find(available_state_interface_types_.begin(),
                            available_state_interface_types_.end(),
                            hardware_interface::HW_IF_EFFORT);

        if (it != available_state_interface_types_.end())
        {
            auto effort_state_interface_it = std::find_if(
                state_interfaces_.begin(), state_interfaces_.end(),
                [this](const hardware_interface::LoanedStateInterface& state_interface)
                {
                    return state_interface.get_interface_name() == hardware_interface::HW_IF_EFFORT &&
                        state_interface.get_prefix_name() == joint_name_;
                });

            if (effort_state_interface_it == state_interfaces_.end())
            {
                RCLCPP_ERROR(get_node()->get_logger(),
                             "Effort interface was requested but not found for joint: %s",
                             joint_name_.c_str());
                return controller_interface::CallbackReturn::ERROR;
            }

            gripper_interfaces_.effort_state_interface_ = *effort_state_interface_it;
            has_effort_interface_ = true;
            RCLCPP_INFO(get_node()->get_logger(),
                        "Effort feedback interface bound for joint: %s",
                        joint_name_.c_str());
        }
        else
        {
            has_effort_interface_ = false;
        }

        // 保存接口引用
        gripper_interfaces_.position_command_interface_ = *command_interface_it;
        gripper_interfaces_.position_state_interface_ = *position_state_interface_it;

        // 注意：夹爪位置计算需要等待 robot_description 解析完成
        // 暂时使用配置的初始值作为关闭位置
        target_position_ = config_initial_position_;

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn AdaptiveGripperController::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        gripper_interfaces_.clear();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type AdaptiveGripperController::update(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        // 读取当前位置
        const double current_position = gripper_interfaces_.position_state_interface_->get().get_optional().value();

        // 只在开关控制模式（非直接位置模式）下启用力反馈
        if (!direct_position_mode_ && has_effort_interface_)
        {
            double current_effort = gripper_interfaces_.effort_state_interface_->get().get_optional().value();
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
        if (!gripper_interfaces_.position_command_interface_->get().set_value(target_position_))
        {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Failed to set position command value: %.6f", target_position_);
            return controller_interface::return_type::ERROR;
        }

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

        // 切换到开关控制模式（启用力反馈）
        direct_position_mode_ = false;

        if (gripper_target_ == 1)
        {
            // 打开夹爪
            force_threshold_triggered_ = false;
            target_position_ = open_position_;
            RCLCPP_DEBUG(get_node()->get_logger(),
                         "Opening gripper to position: %.6f (switch mode with force feedback)",
                         target_position_);
        }
        else
        {
            // 关闭夹爪
            target_position_ = closed_position_;
            RCLCPP_DEBUG(get_node()->get_logger(),
                         "Closing gripper to position: %.6f (switch mode with force feedback)",
                         target_position_);
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

                const size_t upper_end = robot_description.find('\"', upper_pos);
                const size_t lower_end = robot_description.find('\"', lower_pos);

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

    void AdaptiveGripperController::parse_initial_value(const std::string& robot_description)
    {
        try
        {
            // 遍历所有 ros2_control 标签，直到找到包含目标关节的那个
            size_t search_start = 0;
            size_t ros2_control_start = std::string::npos;
            size_t ros2_control_end = std::string::npos;
            size_t joint_pos = std::string::npos;
            
            while (true)
            {
                // 查找下一个 ros2_control 标签的起始位置
                ros2_control_start = robot_description.find("<ros2_control", search_start);
                if (ros2_control_start == std::string::npos)
                {
                    break; // 没有更多的 ros2_control 标签了
                }

                // 查找该 ros2_control 标签的结束位置
                ros2_control_end = robot_description.find("</ros2_control>", ros2_control_start);
                if (ros2_control_end == std::string::npos)
                {
                    ros2_control_end = robot_description.size();
                }

                // 在当前 ros2_control 范围内查找关节定义
                joint_pos = robot_description.find("<joint name=\"" + joint_name_ + "\"", ros2_control_start);
                if (joint_pos != std::string::npos && joint_pos < ros2_control_end)
                {
                    // 找到了目标关节，跳出循环
                    RCLCPP_DEBUG(get_node()->get_logger(),
                                "Found joint %s in ros2_control section at position %zu", 
                                joint_name_.c_str(), joint_pos);
                    break;
                }

                // 在当前 ros2_control 中没找到，继续搜索下一个
                search_start = ros2_control_end + 1;
                joint_pos = std::string::npos;
            }

            // 检查是否找到了 ros2_control 标签
            if (ros2_control_start == std::string::npos)
            {
                RCLCPP_WARN(get_node()->get_logger(), "No ros2_control tag found in robot description");
                return;
            }

            // 检查是否找到了关节定义
            if (joint_pos == std::string::npos || joint_pos > ros2_control_end)
            {
                RCLCPP_WARN(get_node()->get_logger(),
                            "Joint %s not found in any ros2_control section", joint_name_.c_str());
                return;
            }

            // 查找下一个关节标签的位置，确定当前关节的范围
            size_t next_joint_pos = robot_description.find("<joint", joint_pos + 1);
            if (next_joint_pos == std::string::npos || next_joint_pos > ros2_control_end)
            {
                next_joint_pos = ros2_control_end;
            }

            // 在当前关节范围内查找 state_interface position
            size_t state_if_pos = robot_description.find("<state_interface name=\"position\"", joint_pos);
            if (state_if_pos == std::string::npos || state_if_pos > next_joint_pos)
            {
                return;
            }

            // 查找 state_interface 的结束标签
            size_t state_if_end = robot_description.find("</state_interface>", state_if_pos);
            if (state_if_end == std::string::npos || state_if_end > next_joint_pos)
            {
                state_if_end = next_joint_pos;
            }

            // 在 state_interface 范围内查找 initial_value 参数
            const std::string param_start_tag = "<param name=\"initial_value\">";
            size_t param_pos = robot_description.find(param_start_tag, state_if_pos);

            if (param_pos != std::string::npos && param_pos < state_if_end)
            {
                // 跳过开始标签，使用动态长度
                param_pos += param_start_tag.length();
                size_t param_end = robot_description.find("</param>", param_pos);

                if (param_end != std::string::npos && param_end < state_if_end)
                {
                    std::string initial_value_str = robot_description.substr(param_pos, param_end - param_pos);

                    // 去除可能的空白字符
                    initial_value_str.erase(0, initial_value_str.find_first_not_of(" \t\n\r"));
                    initial_value_str.erase(initial_value_str.find_last_not_of(" \t\n\r") + 1);

                    if (!initial_value_str.empty())
                    {
                        config_initial_position_ = std::stod(initial_value_str);
                        RCLCPP_INFO(get_node()->get_logger(),
                                    "Parsed initial_value for %s: %.6f",
                                    joint_name_.c_str(), config_initial_position_);
                    }
                }
            }

            // 在解析完初始值后，计算夹爪的关闭和打开位置
            if (limits_initialized_)
            {
                // 找到距离 initial_value 最近的限位作为关闭位置
                double distance_to_upper = std::abs(joint_upper_limit_ - config_initial_position_);
                double distance_to_lower = std::abs(joint_lower_limit_ - config_initial_position_);

                if (distance_to_upper < distance_to_lower)
                {
                    // initial_value 更接近 upper limit，关闭位置是 upper
                    closed_position_ = joint_upper_limit_;
                    open_position_ = joint_lower_limit_;
                }
                else
                {
                    // initial_value 更接近 lower limit，关闭位置是 lower
                    closed_position_ = joint_lower_limit_;
                    open_position_ = joint_upper_limit_;
                }

                RCLCPP_INFO(get_node()->get_logger(),
                            "Gripper positions - Closed: %.6f, Open: %.6f (initial_value: %.6f, limits: [%.6f, %.6f])",
                            closed_position_, open_position_, config_initial_position_, joint_lower_limit_,
                            joint_upper_limit_);

                // 更新目标位置为关闭位置
                target_position_ = closed_position_;
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Error parsing initial_value: %s", e.what());
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
        // 只请求指定关节的指定接口，避免获取所有关节的所有接口
        controller_interface::InterfaceConfiguration conf;
        conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        for (const auto& interface_type : available_state_interface_types_)
        {
            conf.names.push_back(joint_name_ + "/" + interface_type);
        }

        return conf;
    }
} // namespace adaptive_gripper_controller

PLUGINLIB_EXPORT_CLASS(
    adaptive_gripper_controller::AdaptiveGripperController,
    controller_interface::ControllerInterface)
