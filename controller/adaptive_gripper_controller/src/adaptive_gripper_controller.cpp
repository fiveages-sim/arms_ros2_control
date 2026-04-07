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

        // 获取控制器名称（节点名称就是控制器名称）
        controller_name_ = get_node()->get_name();

        // 力反馈相关参数
        use_effort_interface_ = auto_declare<bool>("use_effort_interface", true);
        force_threshold_ = auto_declare<double>("force_threshold", 0.1);
        force_feedback_ratio_ = auto_declare<double>("force_feedback_ratio", 0.5);

        target_position_ = 0.0;

        RCLCPP_INFO(get_node()->get_logger(),
                    "Adaptive Gripper Controller initialized - controller: %s, joint: %s, use_effort: %s, force threshold: %.3f, force feedback ratio: %.3f",
                    controller_name_.c_str(), joint_name_.c_str(), use_effort_interface_ ? "true" : "false",
                    force_threshold_,
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

        robot_description_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                parse_joint_limits(msg->data);
                parse_initial_value(msg->data);
            });

        // 直接位置控制订阅器 - 无力反馈的精细控制
        // 话题名称格式：/<joint_name>/position_command
        // 回调只做校验/截断，将有效值存入原子收件箱；状态变量由 update() 独占修改
        std::string position_command_topic = "/" + joint_name_ + "/position_command";
        direct_position_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64>(
            position_command_topic, 10, [this](const std_msgs::msg::Float64::SharedPtr msg)
            {
                if (!limits_initialized_)
                {
                    RCLCPP_WARN(get_node()->get_logger(),
                                "Joint limits not initialized yet, ignoring position command %.6f",
                                msg->data);
                    return;
                }

                double commanded_position = msg->data;
                if (std::isnan(commanded_position) || std::isinf(commanded_position))
                {
                    RCLCPP_WARN(get_node()->get_logger(),
                                "Position command is NaN/Inf, ignoring");
                    return;
                }

                double clamped = std::clamp(commanded_position, joint_lower_limit_, joint_upper_limit_);
                if (clamped != commanded_position)
                {
                    RCLCPP_WARN(get_node()->get_logger(),
                                "Position command %.6f clamped to [%.6f, %.6f], using %.6f",
                                commanded_position, joint_lower_limit_, joint_upper_limit_, clamped);
                }

                pending_direct_position_.store(clamped, std::memory_order_relaxed);
            });

        RCLCPP_INFO(get_node()->get_logger(),
                    "Direct position control subscribed to topic: %s", position_command_topic.c_str());

        // 开关控制订阅器 - 通过 0/1 控制夹爪全开/全关（关闭时带力反馈）
        // 话题名称格式：/<controller_name>/target_command
        // 回调只做校验，将 0/1 存入原子收件箱；状态变量由 update() 独占修改
        std::string target_command_topic = "/" + controller_name_ + "/target_command";
        target_command_subscription_ = get_node()->create_subscription<std_msgs::msg::Int32>(
            target_command_topic, 10, [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                if (!limits_initialized_)
                {
                    RCLCPP_WARN(get_node()->get_logger(),
                                "Joint limits not initialized yet, ignoring target command %d",
                                msg->data);
                    return;
                }

                int32_t target = msg->data;
                if (target != 0 && target != 1)
                {
                    RCLCPP_WARN(get_node()->get_logger(),
                                "Invalid target command: %d (expected 0=close or 1=open), ignoring",
                                target);
                    return;
                }

                pending_target_switch_.store(target, std::memory_order_relaxed);
            });

        RCLCPP_INFO(get_node()->get_logger(),
                    "Target command control subscribed to topic: %s (0=close, 1=open, using controller name)",
                    target_command_topic.c_str());

        // 百分比控制订阅器 - 通过 0.0~1.0 比例控制夹爪开合，自动选择力反馈方向
        // 话题名称格式：/<controller_name>/target_percent
        // 0.0 = 完全关闭（closed_position_），1.0 = 完全打开（open_position_）
        // 向关闭方向运动时启用力反馈；向打开方向运动时使用纯位置控制
        std::string target_percent_topic = "/" + controller_name_ + "/target_percent";
        target_percent_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64>(
            target_percent_topic, 10, [this](const std_msgs::msg::Float64::SharedPtr msg)
            {
                if (!limits_initialized_)
                {
                    RCLCPP_WARN(get_node()->get_logger(),
                                "Joint limits not initialized yet, ignoring target_percent command %.3f",
                                msg->data);
                    return;
                }

                double percent = msg->data;
                if (percent < 0.0 || percent > 1.0)
                {
                    RCLCPP_WARN(get_node()->get_logger(),
                                "target_percent %.3f out of range [0.0, 1.0], clamping",
                                percent);
                    percent = std::clamp(percent, 0.0, 1.0);
                }

                // 将百分比存入原子变量，由 update() 线程安全地消费
                pending_percent_command_.store(percent, std::memory_order_relaxed);
            });

        RCLCPP_INFO(get_node()->get_logger(),
                    "Target percent control subscribed to topic: %s (0.0=closed, 1.0=open, auto force feedback)",
                    target_percent_topic.c_str());

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

        // 如果配置了 effort 接口参数，则查找并绑定（use_effort_interface_ 直接决定是否请求过 effort 接口）
        if (use_effort_interface_)
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

        // 重置所有运行时状态，确保重新激活时不携带上一次的脏值
        force_threshold_triggered_ = false;
        direct_position_mode_ = false;
        gripper_target_ = 0;
        pending_direct_position_.store(std::numeric_limits<double>::quiet_NaN(), std::memory_order_relaxed);
        pending_target_switch_.store(-1, std::memory_order_relaxed);
        pending_percent_command_.store(-1.0, std::memory_order_relaxed);

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
        // 读取当前位置（get_optional() 在硬件报错时可能返回 nullopt，需安全处理）
        const double current_position =
            gripper_interfaces_.position_state_interface_->get().get_optional().value_or(
                std::numeric_limits<double>::quiet_NaN());
        if (std::isnan(current_position))
        {
            RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                                  "Position state interface returned no value for joint: %s",
                                  joint_name_.c_str());
            return controller_interface::return_type::ERROR;
        }

        // ---------------------------------------------------------------
        // 命令通道消费（RT 线程独占修改所有状态变量）
        //
        // 优先级：direct_position < target_switch < target_percent
        // 若同一 update 周期内多个通道同时有待命令（极少发生），
        // 后处理的通道会覆盖前者，target_percent 最终生效。
        // ---------------------------------------------------------------

        // 通道 1：直接位置控制（NaN 哨兵表示无命令）
        const double pending_pos = pending_direct_position_.exchange(
            std::numeric_limits<double>::quiet_NaN(), std::memory_order_relaxed);
        if (!std::isnan(pending_pos))
        {
            target_position_ = pending_pos;
            direct_position_mode_ = true;
            force_threshold_triggered_ = false;
            RCLCPP_INFO(get_node()->get_logger(),
                        "Direct position command: %.6f (force feedback disabled)",
                        target_position_);
        }

        // 通道 2：开关控制 0/1（-1 哨兵表示无命令）
        const int32_t pending_switch = pending_target_switch_.exchange(-1, std::memory_order_relaxed);
        if (pending_switch >= 0)
        {
            direct_position_mode_ = false;
            gripper_target_ = pending_switch;
            force_threshold_triggered_ = false;
            target_position_ = (pending_switch == 1) ? open_position_ : closed_position_;
            RCLCPP_INFO(get_node()->get_logger(),
                        "%s gripper via target_command to position: %.6f (switch mode%s)",
                        (pending_switch == 1) ? "Opening" : "Closing",
                        target_position_,
                        has_effort_interface_ ? " with force feedback" : "");
        }

        // 通道 3：百分比控制（-1.0 哨兵表示无命令）
        const double pending_percent = pending_percent_command_.exchange(-1.0, std::memory_order_relaxed);
        if (pending_percent >= 0.0)
        {
            // 将 [0.0, 1.0] 线性映射到实际关节位置
            const double gripper_range = open_position_ - closed_position_;
            double new_target = closed_position_ + pending_percent * gripper_range;
            new_target = std::clamp(new_target, joint_lower_limit_, joint_upper_limit_);

            // 判断运动方向：closing_sign 描述"位置值增大时是否为关闭方向"
            const double delta = new_target - current_position;
            const double closing_sign = (closed_position_ > open_position_) ? 1.0 : -1.0;
            const bool is_closing = (delta * closing_sign) > 1e-6;

            target_position_ = new_target;
            direct_position_mode_ = false;
            force_threshold_triggered_ = false;
            gripper_target_ = is_closing ? 0 : 1;

            RCLCPP_INFO(get_node()->get_logger(),
                        "Target percent %.1f%% -> position: %.6f (%s, force feedback: %s)",
                        pending_percent * 100.0, new_target,
                        is_closing ? "closing" : "opening",
                        (is_closing && has_effort_interface_) ? "enabled" : "disabled");
        }

        // ---------------------------------------------------------------
        // 力反馈逻辑
        // 仅在开关/百分比模式（非直接位置模式）且有 effort 接口时生效；
        // gripper_target_ == 0 表示当前朝关闭方向运动，需检测接触力。
        // ---------------------------------------------------------------
        if (!direct_position_mode_ && has_effort_interface_)
        {
            const double current_effort =
                gripper_interfaces_.effort_state_interface_->get().get_optional().value_or(0.0);
            if (gripper_target_ == 0 && !force_threshold_triggered_ &&
                std::abs(current_effort) > force_threshold_)
            {
                // force_feedback_ratio_ = 0.0 → 停在当前位置
                // force_feedback_ratio_ = 1.0 → 继续推进到原始目标
                const double original_target = target_position_;
                const double distance_to_target = original_target - current_position;
                target_position_ = current_position + distance_to_target * force_feedback_ratio_;
                force_threshold_triggered_ = true;

                RCLCPP_INFO(get_node()->get_logger(),
                            "Force threshold triggered (closing), moving %.1f%% toward target: "
                            "%.6f (current: %.6f, original target: %.6f)",
                            force_feedback_ratio_ * 100.0, target_position_,
                            current_position, original_target);
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

            // 确定该 joint 标签的范围，防止跨关节错误匹配
            const size_t joint_end = [&]()
            {
                size_t end = robot_description.find("</joint>", joint_pos);
                return (end != std::string::npos) ? end : robot_description.size();
            }();

            // 查找 <limit> 标签，必须在当前 </joint> 范围内
            size_t limit_pos = robot_description.find("<limit", joint_pos);
            if (limit_pos == std::string::npos || limit_pos > joint_end)
            {
                RCLCPP_WARN(get_node()->get_logger(), "No limits found for joint %s", joint_name_.c_str());
                return;
            }

            // upper/lower 属性也必须在同一个 <limit .../> 标签内
            const size_t limit_tag_end = robot_description.find('>', limit_pos);
            if (limit_tag_end == std::string::npos || limit_tag_end > joint_end)
            {
                RCLCPP_WARN(get_node()->get_logger(), "Malformed <limit> tag for joint %s", joint_name_.c_str());
                return;
            }

            // 解析upper和lower限制
            size_t upper_pos = robot_description.find("upper=\"", limit_pos);
            size_t lower_pos = robot_description.find("lower=\"", limit_pos);

            // 确保 upper/lower 属性在同一个 <limit> 标签内
            if (upper_pos != std::string::npos && upper_pos > limit_tag_end) upper_pos = std::string::npos;
            if (lower_pos != std::string::npos && lower_pos > limit_tag_end) lower_pos = std::string::npos;

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
