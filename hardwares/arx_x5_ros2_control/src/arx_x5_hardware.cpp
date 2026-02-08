#include "arx_x5_ros2_control/arx_x5_hardware.h"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>

namespace arx_x5_ros2_control {

/**
 * @brief 将字符串转换为大写形式
 * @param str 输入字符串
 * @return 转换后的大写字符串
 *
 * 用于参数解析时统一字符串格式，如 "left" -> "LEFT"
 */
std::string ArxX5Hardware::normalizeString(const std::string& str)
{
    std::string result = str;
    std::transform(result.begin(), result.end(), result.begin(), ::toupper);
    return result;
}

/**
 * @brief 声明 ROS2 节点参数
 *
 * 从 URDF 的 hardware_parameters 中读取初始值，并声明为 ROS2 参数。
 * 这样可以通过 `ros2 param set` 动态修改参数值。
 *
 * 声明的参数包括：
 * - arm_config: 臂配置 ("LEFT", "RIGHT", "DUAL")
 * - robot_model / can_interface: 单臂模式配置
 * - left_robot_model / right_robot_model: 双臂模式型号配置
 * - left_can_interface / right_can_interface: 双臂模式 CAN 接口配置
 */
void ArxX5Hardware::declare_node_parameters()
{
    // 目标：优先使用 hardware_interface 的 info_.hardware_parameters 作为"初始值来源"，
    // 然后用该初始值声明为 ROS2 node 参数，便于后续 ros2 param set 动态修改。
    //
    // 规则：
    // - 如果参数已存在且类型正确：尊重现有值（可能来自 launch 覆盖/之前声明），不覆盖
    // - 如果参数已存在但类型不对：undeclare 后按正确类型重新 declare（避免类型冲突）
    // - 如果参数不存在：从 hardware_parameters 取值（若有）否则用默认值 declare

    const auto hw_find = [this](const std::string& name) -> const std::string* {
        auto it = info_.hardware_parameters.find(name);
        if (it == info_.hardware_parameters.end()) {
            return nullptr;
        }
        return &it->second;
    };

    // 辅助函数：确保字符串参数已声明
    const auto ensure_string_param = [this](const std::string& name, const std::string& default_val, const std::string* hw_val) {
        if (node_->has_parameter(name)) {
            // 检查类型是否正确
            if (node_->get_parameter(name).get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
                try { 
                    node_->undeclare_parameter(name); 
                } catch (...) {}
            } else {
                return; // 已存在且类型正确：尊重现有值
            }
        }

        // 参数不存在或类型错误，需要声明
        if (!node_->has_parameter(name)) {
            const std::string val = hw_val ? *hw_val : default_val;
            node_->declare_parameter<std::string>(name, val);
        }
    };

    // arm_config: 臂配置 ("LEFT", "RIGHT", "DUAL")
    ensure_string_param("arm_config", "LEFT", hw_find("arm_config"));
    
    // 单臂配置（向后兼容）
    ensure_string_param("robot_model", "X5", hw_find("robot_model"));
    ensure_string_param("can_interface", "can0", hw_find("can_interface"));
    
    // 双臂配置
    ensure_string_param("left_robot_model", "X5", hw_find("left_robot_model"));
    ensure_string_param("right_robot_model", "X5", hw_find("right_robot_model"));
    ensure_string_param("left_can_interface", "can0", hw_find("left_can_interface"));
    ensure_string_param("right_can_interface", "can1", hw_find("right_can_interface"));
}

/**
 * @brief 硬件接口初始化回调
 * @param params 硬件组件参数，包含 URDF 中定义的关节和参数信息
 * @return SUCCESS 表示初始化成功，ERROR 表示失败
 *
 * 生命周期：在 ros2_control 加载硬件插件时调用（unconfigured -> inactive 之前）
 *
 * 主要工作：
 * 1. 调用父类初始化
 * 2. 获取 ROS2 节点和日志器
 * 3. 声明并读取配置参数（arm_config, robot_model, can_interface 等）
 * 4. 解析 URDF 中的关节信息，区分机械臂关节和夹爪关节
 * 5. 根据单臂/双臂模式分配关节到左臂或右臂
 * 6. 初始化状态和命令数组
 */
hardware_interface::CallbackReturn ArxX5Hardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {

    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 获取节点和日志器
    node_ = get_node();
    logger_ = get_node()->get_logger();

    // 声明节点参数
    declare_node_parameters();

    // 解析arm_config参数
    std::string arm_config_raw = get_node_param("arm_config", std::string("LEFT"));
    arm_config_ = normalizeString(arm_config_raw);
    
    if (arm_config_ == "LEFT") {
        arm_index_ = ARM_LEFT;
    } else if (arm_config_ == "RIGHT") {
        arm_index_ = ARM_RIGHT;
    } else if (arm_config_ == "DUAL") {
        arm_index_ = ARM_DUAL;
    } else {
        RCLCPP_WARN(get_logger(), 
                    "Unknown arm_config '%s', using default LEFT. Valid options: LEFT, RIGHT, DUAL", 
                    arm_config_raw.c_str());
        arm_config_ = "LEFT";
        arm_index_ = ARM_LEFT;
    }

    // 读取配置参数
    if (arm_index_ == ARM_DUAL) {
        // 双臂模式：使用左右臂独立配置
        left_robot_model_ = get_node_param("left_robot_model", std::string("X5"));
        right_robot_model_ = get_node_param("right_robot_model", std::string("X5"));
        left_can_interface_ = get_node_param("left_can_interface", std::string("can0"));
        right_can_interface_ = get_node_param("right_can_interface", std::string("can1"));
    } else {
        // 单臂模式：使用向后兼容的配置
        robot_model_ = get_node_param("robot_model", std::string("X5"));
        can_interface_ = get_node_param("can_interface", std::string("can0"));
        // 同时设置对应的左右臂配置（用于统一处理）
        if (arm_index_ == ARM_LEFT) {
            left_robot_model_ = robot_model_;
            left_can_interface_ = can_interface_;
        } else {
            right_robot_model_ = robot_model_;
            right_can_interface_ = can_interface_;
        }
    }

    // 解析关节：区分左臂和右臂关节
    has_gripper_ = false;
    gripper_joint_names_.clear();
    joint_names_.clear();
    left_joint_names_.clear();
    right_joint_names_.clear();
    
    for (const auto& joint : params.hardware_info.joints) {
        std::string joint_name_lower = joint.name;
        std::transform(joint_name_lower.begin(), joint_name_lower.end(),
                       joint_name_lower.begin(), ::tolower);

        if (joint_name_lower.find("gripper") != std::string::npos) {
            has_gripper_ = true;
            gripper_joint_names_.push_back(joint.name);
            RCLCPP_INFO(get_logger(),
                        "Detected gripper joint: %s", joint.name.c_str());
        } else {
            joint_names_.push_back(joint.name);
            
            // 根据关节名称前缀判断属于左臂还是右臂
            // 假设左臂关节名称包含"left"或"l_"，右臂包含"right"或"r_"
            if (arm_index_ == ARM_DUAL) {
                if (joint_name_lower.find("left") != std::string::npos || 
                    joint_name_lower.find("l_") != std::string::npos ||
                    (joint_name_lower.length() > 0 && joint_name_lower[0] == 'l')) {
                    left_joint_names_.push_back(joint.name);
                } else if (joint_name_lower.find("right") != std::string::npos || 
                          joint_name_lower.find("r_") != std::string::npos ||
                          (joint_name_lower.length() > 0 && joint_name_lower[0] == 'r')) {
                    right_joint_names_.push_back(joint.name);
                } else {
                    // 如果没有明确的前缀，按顺序分配：前半部分给左臂，后半部分给右臂
                    // 这里假设关节数量是偶数，且按顺序排列
                    if (left_joint_names_.size() < right_joint_names_.size()) {
                        left_joint_names_.push_back(joint.name);
                    } else {
                        right_joint_names_.push_back(joint.name);
                    }
                }
            } else if (arm_index_ == ARM_LEFT) {
                left_joint_names_.push_back(joint.name);
            } else {
                right_joint_names_.push_back(joint.name);
            }
        }
    }
    
    // 计算关节数量
    joint_count_ = static_cast<int>(joint_names_.size());
    left_joint_count_ = static_cast<int>(left_joint_names_.size());
    right_joint_count_ = static_cast<int>(right_joint_names_.size());
    
    if (arm_index_ == ARM_DUAL) {
        RCLCPP_INFO(get_logger(),
                    "Dual-arm configuration: left=%d joints, right=%d joints, total=%d joints",
                    left_joint_count_, right_joint_count_, joint_count_);
    }
    
    if (has_gripper_) {
        const size_t expected_grippers = (arm_index_ == ARM_DUAL) ? 2 : 1;
        RCLCPP_INFO(get_logger(),
                    "Found %zu gripper joint(s), expected %zu gripper(s) for %s arm config",
                    gripper_joint_names_.size(), expected_grippers, arm_config_.c_str());
    }

    // 初始化状态和命令数组
    position_states_.resize(joint_count_, 0.0);
    velocity_states_.resize(joint_count_, 0.0);
    effort_states_.resize(joint_count_, 0.0);
    position_commands_.resize(joint_count_, 0.0);

    if (has_gripper_) {
        const size_t gripper_count = gripper_joint_names_.size();
        gripper_position_states_.resize(gripper_count, 0.0);
        gripper_velocity_states_.resize(gripper_count, 0.0);
        gripper_effort_states_.resize(gripper_count, 0.0);
        gripper_position_commands_.resize(gripper_count, 0.0);
    }

    // 初始化控制器指针
    controllers_[0] = nullptr;
    controllers_[1] = nullptr;

    RCLCPP_INFO(get_logger(),
                "Initialized %s arm config with %d joints",
                arm_config_.c_str(), joint_count_);

    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 导出状态接口
 * @return 状态接口列表
 *
 * 生命周期：在 on_init 之后、on_activate 之前调用
 *
 * 导出的状态接口供控制器读取硬件状态：
 * - 每个机械臂关节导出 position, velocity, effort 三个状态
 * - 每个夹爪关节导出 position, velocity, effort 三个状态
 *
 * 控制器通过这些接口获取关节的实时位置、速度和力矩反馈
 */
std::vector<hardware_interface::StateInterface> ArxX5Hardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // 导出关节状态接口
    for (int i = 0; i < joint_count_; ++i) {
        state_interfaces.emplace_back(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &position_states_[i]);
        state_interfaces.emplace_back(
            joint_names_[i], hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]);
        state_interfaces.emplace_back(
            joint_names_[i], hardware_interface::HW_IF_EFFORT, &effort_states_[i]);
    }

    // 导出夹爪状态接口（支持多个夹爪关节）
    if (has_gripper_) {
        for (size_t i = 0; i < gripper_joint_names_.size(); ++i) {
            state_interfaces.emplace_back(
                gripper_joint_names_[i], hardware_interface::HW_IF_POSITION, &gripper_position_states_[i]);
            state_interfaces.emplace_back(
                gripper_joint_names_[i], hardware_interface::HW_IF_VELOCITY, &gripper_velocity_states_[i]);
            state_interfaces.emplace_back(
                gripper_joint_names_[i], hardware_interface::HW_IF_EFFORT, &gripper_effort_states_[i]);
        }
    }

    return state_interfaces;
}

/**
 * @brief 导出命令接口
 * @return 命令接口列表
 *
 * 生命周期：在 on_init 之后、on_activate 之前调用
 *
 * 导出的命令接口供控制器写入目标命令：
 * - 每个机械臂关节导出 position 命令接口
 * - 每个夹爪关节导出 position 命令接口
 *
 * 控制器通过这些接口发送目标位置指令给硬件
 */
std::vector<hardware_interface::CommandInterface> ArxX5Hardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // 导出关节命令接口
    for (int i = 0; i < joint_count_; ++i) {
        command_interfaces.emplace_back(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &position_commands_[i]);
    }

    // 导出夹爪命令接口（支持多个夹爪关节）
    if (has_gripper_) {
        for (size_t i = 0; i < gripper_joint_names_.size(); ++i) {
            command_interfaces.emplace_back(
                gripper_joint_names_[i], hardware_interface::HW_IF_POSITION, &gripper_position_commands_[i]);
        }
    }

    return command_interfaces;
}

/**
 * @brief 硬件激活回调
 * @param previous_state 之前的生命周期状态（未使用）
 * @return SUCCESS 表示激活成功，ERROR 表示失败
 *
 * 生命周期：inactive -> active 状态转换时调用
 *
 * 主要工作：
 * 1. 根据配置创建 ARX SDK 控制器实例
 *    - 单臂模式：创建一个控制器
 *    - 双臂模式：创建左右两个控制器
 * 2. 连接到真实硬件（通过 CAN 接口）
 * 3. 读取当前关节状态作为初始值
 * 4. 将初始位置同时设为命令值（避免激活时跳变）
 *
 * 错误处理：如果连接失败，清理已创建的控制器并返回 ERROR
 */
hardware_interface::CallbackReturn ArxX5Hardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {

    try {
        if (arm_index_ == ARM_DUAL) {
            // 双臂模式：创建两个控制器
            RCLCPP_INFO(get_logger(),
                        "Activating dual-arm: Left=%s on %s, Right=%s on %s",
                        left_robot_model_.c_str(), left_can_interface_.c_str(),
                        right_robot_model_.c_str(), right_can_interface_.c_str());
            
            controllers_[ARM_LEFT] = std::make_shared<arx::Arx5JointController>(
                left_robot_model_, left_can_interface_);
            controllers_[ARM_RIGHT] = std::make_shared<arx::Arx5JointController>(
                right_robot_model_, right_can_interface_);

            // 读取左臂初始状态
            arx::JointState left_state = controllers_[ARM_LEFT]->get_joint_state();
            for (int i = 0; i < left_joint_count_ && i < static_cast<int>(left_state.pos.size()); ++i) {
                position_states_[i] = left_state.pos[i];
                velocity_states_[i] = left_state.vel[i];
                effort_states_[i] = left_state.torque[i];
                position_commands_[i] = left_state.pos[i];
            }
            
            // 读取右臂初始状态
            arx::JointState right_state = controllers_[ARM_RIGHT]->get_joint_state();
            for (int i = 0; i < right_joint_count_ && i < static_cast<int>(right_state.pos.size()); ++i) {
                const int dst_idx = left_joint_count_ + i;
                if (dst_idx < joint_count_) {
                    position_states_[dst_idx] = right_state.pos[i];
                    velocity_states_[dst_idx] = right_state.vel[i];
                    effort_states_[dst_idx] = right_state.torque[i];
                    position_commands_[dst_idx] = right_state.pos[i];
                }
            }
            
            // 初始化夹爪状态（双臂模式：左臂和右臂各一个夹爪）
            if (has_gripper_) {
                if (gripper_joint_names_.size() >= 1) {
                    gripper_position_states_[0] = left_state.gripper_pos;
                    gripper_position_commands_[0] = left_state.gripper_pos;
                }
                if (gripper_joint_names_.size() >= 2) {
                    gripper_position_states_[1] = right_state.gripper_pos;
                    gripper_position_commands_[1] = right_state.gripper_pos;
                }
            }
            
        } else {
            // 单臂模式
            const int arm_idx = arm_index_;
            const std::string& model = (arm_idx == ARM_LEFT) ? left_robot_model_ : right_robot_model_;
            const std::string& can_if = (arm_idx == ARM_LEFT) ? left_can_interface_ : right_can_interface_;
            
            RCLCPP_INFO(get_logger(),
                        "Activating %s arm: %s on %s",
                        arm_config_.c_str(), model.c_str(), can_if.c_str());
            
            controllers_[arm_idx] = std::make_shared<arx::Arx5JointController>(model, can_if);

            // 读取初始状态
            arx::JointState initial_state = controllers_[arm_idx]->get_joint_state();
            for (int i = 0; i < joint_count_ && i < static_cast<int>(initial_state.pos.size()); ++i) {
                position_states_[i] = initial_state.pos[i];
                velocity_states_[i] = initial_state.vel[i];
                effort_states_[i] = initial_state.torque[i];
                position_commands_[i] = initial_state.pos[i];
            }

            // 初始化夹爪状态
            if (has_gripper_ && gripper_joint_names_.size() >= 1) {
                gripper_position_states_[0] = initial_state.gripper_pos;
                gripper_position_commands_[0] = initial_state.gripper_pos;
            }
        }

        RCLCPP_INFO(get_logger(), "Successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(),
                     "Failed to activate: %s", e.what());
        // 清理已创建的控制器
        controllers_[0].reset();
        controllers_[1].reset();
        return hardware_interface::CallbackReturn::ERROR;
    }
}

/**
 * @brief 硬件停用回调
 * @param previous_state 之前的生命周期状态（未使用）
 * @return SUCCESS 表示停用成功
 *
 * 生命周期：active -> inactive 状态转换时调用
 *
 * 主要工作：
 * 1. 释放 ARX SDK 控制器实例
 * 2. 断开与硬件的连接
 *
 * 停用后硬件将停止接收命令，关节保持当前位置
 */
hardware_interface::CallbackReturn ArxX5Hardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {

    RCLCPP_INFO(get_logger(), "Deactivating...");

    // 释放控制器
    if (arm_index_ == ARM_DUAL) {
        controllers_[ARM_LEFT].reset();
        controllers_[ARM_RIGHT].reset();
    } else {
        controllers_[arm_index_].reset();
    }

    RCLCPP_INFO(get_logger(), "Successfully deactivated!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 从硬件读取状态
 * @param time 当前时间（未使用）
 * @param period 控制周期（未使用）
 * @return OK 表示读取成功，ERROR 表示失败
 *
 * 调用频率：由 ros2_control 的控制循环决定（通常 100-1000 Hz）
 *
 * 主要工作：
 * 1. 从 ARX SDK 获取关节状态（位置、速度、力矩）
 * 2. 更新状态数组，供控制器通过状态接口读取
 * 3. 双臂模式下分别读取左右臂状态
 * 4. 同时更新夹爪状态
 *
 * 数据流：ARX SDK -> position_states_/velocity_states_/effort_states_ -> 控制器
 */
hardware_interface::return_type ArxX5Hardware::read(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

    if (arm_index_ == ARM_DUAL) {
        // 双臂模式：从两个控制器读取状态
        if (!controllers_[ARM_LEFT] || !controllers_[ARM_RIGHT]) {
            return hardware_interface::return_type::ERROR;
        }

        // 读取左臂状态
        arx::JointState left_state = controllers_[ARM_LEFT]->get_joint_state();
        for (int i = 0; i < left_joint_count_ && i < static_cast<int>(left_state.pos.size()); ++i) {
            position_states_[i] = left_state.pos[i];
            velocity_states_[i] = left_state.vel[i];
            effort_states_[i] = left_state.torque[i];
        }
        
        // 读取右臂状态
        arx::JointState right_state = controllers_[ARM_RIGHT]->get_joint_state();
        for (int i = 0; i < right_joint_count_ && i < static_cast<int>(right_state.pos.size()); ++i) {
            const int dst_idx = left_joint_count_ + i;
            if (dst_idx < joint_count_) {
                position_states_[dst_idx] = right_state.pos[i];
                velocity_states_[dst_idx] = right_state.vel[i];
                effort_states_[dst_idx] = right_state.torque[i];
            }
        }
        
        // 更新夹爪状态（双臂模式：左臂和右臂各一个夹爪）
        if (has_gripper_) {
            if (gripper_joint_names_.size() >= 1) {
                gripper_position_states_[0] = left_state.gripper_pos;
                gripper_velocity_states_[0] = left_state.gripper_vel;
                gripper_effort_states_[0] = left_state.gripper_torque;
            }
            if (gripper_joint_names_.size() >= 2) {
                gripper_position_states_[1] = right_state.gripper_pos;
                gripper_velocity_states_[1] = right_state.gripper_vel;
                gripper_effort_states_[1] = right_state.gripper_torque;
            }
        }
        
    } else {
        // 单臂模式
        const int arm_idx = arm_index_;
        if (!controllers_[arm_idx]) {
            return hardware_interface::return_type::ERROR;
        }

        // 从SDK读取关节状态
        arx::JointState state = controllers_[arm_idx]->get_joint_state();

        // 更新关节状态
        for (int i = 0; i < joint_count_ && i < static_cast<int>(state.pos.size()); ++i) {
            position_states_[i] = state.pos[i];
            velocity_states_[i] = state.vel[i];
            effort_states_[i] = state.torque[i];
        }

        // 更新夹爪状态
        if (has_gripper_ && gripper_joint_names_.size() >= 1) {
            gripper_position_states_[0] = state.gripper_pos;
            gripper_velocity_states_[0] = state.gripper_vel;
            gripper_effort_states_[0] = state.gripper_torque;
        }
    }

    return hardware_interface::return_type::OK;
}

/**
 * @brief 向硬件写入命令
 * @param time 当前时间（未使用）
 * @param period 控制周期（未使用）
 * @return OK 表示写入成功，ERROR 表示失败
 *
 * 调用频率：由 ros2_control 的控制循环决定（通常 100-1000 Hz）
 *
 * 主要工作：
 * 1. 从命令数组读取目标位置（由控制器通过命令接口写入）
 * 2. 构建 ARX SDK 的 JointState 命令结构
 * 3. 发送命令到 ARX SDK，驱动电机运动
 * 4. 双臂模式下分别向左右臂发送命令
 * 5. 同时发送夹爪命令
 *
 * 数据流：控制器 -> position_commands_ -> ARX SDK -> 电机
 */
hardware_interface::return_type ArxX5Hardware::write(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

    if (arm_index_ == ARM_DUAL) {
        // 双臂模式：向两个控制器发送命令
        if (!controllers_[ARM_LEFT] || !controllers_[ARM_RIGHT]) {
            return hardware_interface::return_type::ERROR;
        }

        // 构建左臂命令
        arx::JointState left_cmd(left_joint_count_);
        for (int i = 0; i < left_joint_count_; ++i) {
            left_cmd.pos[i] = position_commands_[i];
        }
        if (has_gripper_ && gripper_joint_names_.size() >= 1) {
            left_cmd.gripper_pos = gripper_position_commands_[0];
        }
        controllers_[ARM_LEFT]->set_joint_cmd(left_cmd);
        
        // 构建右臂命令
        arx::JointState right_cmd(right_joint_count_);
        for (int i = 0; i < right_joint_count_; ++i) {
            const int src_idx = left_joint_count_ + i;
            if (src_idx < joint_count_) {
                right_cmd.pos[i] = position_commands_[src_idx];
            }
        }
        if (has_gripper_ && gripper_joint_names_.size() >= 2) {
            right_cmd.gripper_pos = gripper_position_commands_[1];
        } else if (has_gripper_ && gripper_joint_names_.size() == 1) {
            // 如果只有一个夹爪关节，使用第一个命令值
            right_cmd.gripper_pos = gripper_position_commands_[0];
        }
        controllers_[ARM_RIGHT]->set_joint_cmd(right_cmd);
        
    } else {
        // 单臂模式
        const int arm_idx = arm_index_;
        if (!controllers_[arm_idx]) {
            return hardware_interface::return_type::ERROR;
        }

        // 构建关节命令
        arx::JointState cmd(joint_count_);
        for (int i = 0; i < joint_count_; ++i) {
            cmd.pos[i] = position_commands_[i];
        }

        // 设置夹爪命令
        if (has_gripper_ && gripper_joint_names_.size() >= 1) {
            cmd.gripper_pos = gripper_position_commands_[0];
        }

        // 发送命令到SDK
        controllers_[arm_idx]->set_joint_cmd(cmd);
    }

    return hardware_interface::return_type::OK;
}

}  // namespace arx_x5_ros2_control

/**
 * @brief 注册硬件接口插件
 *
 * 将 ArxX5Hardware 类注册为 ros2_control 的 SystemInterface 插件。
 * ros2_control 通过 pluginlib 动态加载此硬件接口。
 *
 * 配合 arx_x5_ros2_control.xml 插件描述文件使用。
 */
PLUGINLIB_EXPORT_CLASS(arx_x5_ros2_control::ArxX5Hardware, hardware_interface::SystemInterface)
