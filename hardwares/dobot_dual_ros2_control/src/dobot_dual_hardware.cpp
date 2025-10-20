// Copyright 2024 Dobot Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "dobot_dual_ros2_control/dobot_dual_hardware.h"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>

namespace dobot_dual_ros2_control
{

hardware_interface::CallbackReturn DobotDualHardware::on_init(const hardware_interface::HardwareInfo& info)
{
    if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 初始化关节数据存储（12个关节：0-5左臂, 6-11右臂）
    joint_positions_.resize(12, 0.0);
    joint_velocities_.resize(12, 0.0);
    joint_efforts_.resize(12, 0.0);
    joint_position_commands_.resize(12, 0.0);

    // 初始化夹爪数据
    left_gripper_position_ = 0.0;
    right_gripper_position_ = 0.0;
    left_gripper_position_command_ = 0.0;
    right_gripper_position_command_ = 0.0;
    has_left_gripper_ = false;
    has_right_gripper_ = false;
    left_gripper_thread_running_ = false;
    right_gripper_thread_running_ = false;

    // 检测是否配置了夹爪关节
    for (const auto& joint : info.joints) {
        std::string joint_name_lower = joint.name;
        std::transform(joint_name_lower.begin(), joint_name_lower.end(),
                      joint_name_lower.begin(), ::tolower);

        // 检测左夹爪
        if ((joint_name_lower.find("left") != std::string::npos) &&
            (joint_name_lower.find("gripper") != std::string::npos ||
             joint_name_lower.find("hand") != std::string::npos)) {
            has_left_gripper_ = true;
            left_gripper_joint_name_ = joint.name;
            RCLCPP_INFO(get_node()->get_logger(), "Detected left gripper joint: %s", left_gripper_joint_name_.c_str());
        }

        // 检测右夹爪
        if ((joint_name_lower.find("right") != std::string::npos) &&
            (joint_name_lower.find("gripper") != std::string::npos ||
             joint_name_lower.find("hand") != std::string::npos)) {
            has_right_gripper_ = true;
            right_gripper_joint_name_ = joint.name;
            RCLCPP_INFO(get_node()->get_logger(), "Detected right gripper joint: %s", right_gripper_joint_name_.c_str());
        }
    }

    // 读取配置参数的辅助函数
    const auto get_hardware_parameter = [&info](const std::string& parameter_name, const std::string& default_value) {
        if (auto it = info.hardware_parameters.find(parameter_name); it != info.hardware_parameters.end()) {
            return it->second;
        }
        return default_value;
    };

    // 读取左臂配置参数
    left_robot_ip_ = get_hardware_parameter("left_robot_ip", "192.168.5.38");
    left_servo_time_ = std::stod(get_hardware_parameter("left_servo_time", "0.03"));
    left_aheadtime_ = std::stod(get_hardware_parameter("left_aheadtime", "20.0"));
    left_gain_ = std::stod(get_hardware_parameter("left_gain", "500.0"));
    left_speed_factor_ = std::stoi(get_hardware_parameter("left_speed_factor", "5"));

    // 读取右臂配置参数
    right_robot_ip_ = get_hardware_parameter("right_robot_ip", "192.168.5.39");
    right_servo_time_ = std::stod(get_hardware_parameter("right_servo_time", "0.03"));
    right_aheadtime_ = std::stod(get_hardware_parameter("right_aheadtime", "20.0"));
    right_gain_ = std::stod(get_hardware_parameter("right_gain", "500.0"));
    right_speed_factor_ = std::stoi(get_hardware_parameter("right_speed_factor", "5"));

    // 通用配置
    verbose_ = (get_hardware_parameter("verbose", "false") == "true");

    // 初始化统计变量
    write_count_ = 0;
    last_write_stat_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(get_node()->get_logger(), "==============================================");
    RCLCPP_INFO(get_node()->get_logger(), "DobotDualHardware initialized successfully");
    RCLCPP_INFO(get_node()->get_logger(), "LEFT ARM:");
    RCLCPP_INFO(get_node()->get_logger(), "  Robot IP: %s", left_robot_ip_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "  ServoJ Time: %.3f s", left_servo_time_);
    RCLCPP_INFO(get_node()->get_logger(), "  Aheadtime: %.1f", left_aheadtime_);
    RCLCPP_INFO(get_node()->get_logger(), "  Gain: %.1f", left_gain_);
    RCLCPP_INFO(get_node()->get_logger(), "  Speed Factor: %d%%", left_speed_factor_);
    RCLCPP_INFO(get_node()->get_logger(), "RIGHT ARM:");
    RCLCPP_INFO(get_node()->get_logger(), "  Robot IP: %s", right_robot_ip_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "  ServoJ Time: %.3f s", right_servo_time_);
    RCLCPP_INFO(get_node()->get_logger(), "  Aheadtime: %.1f", right_aheadtime_);
    RCLCPP_INFO(get_node()->get_logger(), "  Gain: %.1f", right_gain_);
    RCLCPP_INFO(get_node()->get_logger(), "  Speed Factor: %d%%", right_speed_factor_);
    RCLCPP_INFO(get_node()->get_logger(), "  Verbose: %s", verbose_ ? "true" : "false");
    RCLCPP_INFO(get_node()->get_logger(), "  Total joints: %zu", joint_positions_.size());
    RCLCPP_INFO(get_node()->get_logger(), "==============================================");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DobotDualHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
    RCLCPP_INFO(get_node()->get_logger(), "Activating DobotDualHardware...");

    try {
        // 创建左臂TCP连接
        RCLCPP_INFO(get_node()->get_logger(), "Connecting to LEFT arm at %s...", left_robot_ip_.c_str());
        left_commander_ = std::make_shared<CRCommanderRos2>(left_robot_ip_, verbose_);
        left_commander_->init();

        // 等待左臂连接
        auto start_time = std::chrono::steady_clock::now();
        const auto timeout = std::chrono::seconds(10);

        while (!left_commander_->isConnected()) {
            auto current_time = std::chrono::steady_clock::now();
            if (current_time - start_time > timeout) {
                RCLCPP_ERROR(get_node()->get_logger(), "Timeout waiting for LEFT arm connection!");
                return hardware_interface::CallbackReturn::ERROR;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        RCLCPP_INFO(get_node()->get_logger(), "✅ LEFT arm connected!");

        // 创建右臂TCP连接
        RCLCPP_INFO(get_node()->get_logger(), "Connecting to RIGHT arm at %s...", right_robot_ip_.c_str());
        right_commander_ = std::make_shared<CRCommanderRos2>(right_robot_ip_, verbose_);
        right_commander_->init();

        // 等待右臂连接
        start_time = std::chrono::steady_clock::now();
        while (!right_commander_->isConnected()) {
            auto current_time = std::chrono::steady_clock::now();
            if (current_time - start_time > timeout) {
                RCLCPP_ERROR(get_node()->get_logger(), "Timeout waiting for RIGHT arm connection!");
                return hardware_interface::CallbackReturn::ERROR;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        RCLCPP_INFO(get_node()->get_logger(), "✅ RIGHT arm connected!");

        // 设置左臂全局速度
        RCLCPP_INFO(get_node()->get_logger(), "Setting LEFT arm speed factor to %d%%...", left_speed_factor_);
        if (!left_commander_->setSpeedFactor(left_speed_factor_)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to set LEFT arm speed factor!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // 设置右臂全局速度
        RCLCPP_INFO(get_node()->get_logger(), "Setting RIGHT arm speed factor to %d%%...", right_speed_factor_);
        if (!right_commander_->setSpeedFactor(right_speed_factor_)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to set RIGHT arm speed factor!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // 初始化左夹爪
        if (has_left_gripper_) {
            if (!initializeLeftModbus()) {
                RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize LEFT gripper Modbus!");
                return hardware_interface::CallbackReturn::ERROR;
            }

            double gripper_pos;
            if (readLeftGripperState(gripper_pos)) {
                left_gripper_position_ = gripper_pos;
                left_gripper_position_command_ = gripper_pos;
                RCLCPP_INFO(get_node()->get_logger(), "Initial LEFT gripper position: %.3f", gripper_pos);
            } else {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to read LEFT gripper position, using 0.0");
                left_gripper_position_ = 0.0;
                left_gripper_position_command_ = 0.0;
            }

            left_gripper_thread_running_ = true;
            left_gripper_control_thread_ = std::thread(&DobotDualHardware::leftGripperControlLoop, this);
            RCLCPP_INFO(get_node()->get_logger(), "✅ LEFT gripper control thread started");
        }

        // 初始化右夹爪
        if (has_right_gripper_) {
            if (!initializeRightModbus()) {
                RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize RIGHT gripper Modbus!");
                return hardware_interface::CallbackReturn::ERROR;
            }

            double gripper_pos;
            if (readRightGripperState(gripper_pos)) {
                right_gripper_position_ = gripper_pos;
                right_gripper_position_command_ = gripper_pos;
                RCLCPP_INFO(get_node()->get_logger(), "Initial RIGHT gripper position: %.3f", gripper_pos);
            } else {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to read RIGHT gripper position, using 0.0");
                right_gripper_position_ = 0.0;
                right_gripper_position_command_ = 0.0;
            }

            right_gripper_thread_running_ = true;
            right_gripper_control_thread_ = std::thread(&DobotDualHardware::rightGripperControlLoop, this);
            RCLCPP_INFO(get_node()->get_logger(), "✅ RIGHT gripper control thread started");
        }

        // 读取左臂当前关节位置
        double left_current_joints[6];
        left_commander_->getCurrentJointStatus(left_current_joints);

        // 读取右臂当前关节位置
        double right_current_joints[6];
        right_commander_->getCurrentJointStatus(right_current_joints);

        // 初始化关节位置和命令
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (size_t i = 0; i < 6; ++i) {
            joint_positions_[i] = left_current_joints[i];
            joint_position_commands_[i] = left_current_joints[i];

            joint_positions_[i + 6] = right_current_joints[i];
            joint_position_commands_[i + 6] = right_current_joints[i];
        }

        RCLCPP_INFO(get_node()->get_logger(),
                   "Initial LEFT joint positions (rad): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                   joint_positions_[0], joint_positions_[1], joint_positions_[2],
                   joint_positions_[3], joint_positions_[4], joint_positions_[5]);

        RCLCPP_INFO(get_node()->get_logger(),
                   "Initial RIGHT joint positions (rad): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                   joint_positions_[6], joint_positions_[7], joint_positions_[8],
                   joint_positions_[9], joint_positions_[10], joint_positions_[11]);

        RCLCPP_INFO(get_node()->get_logger(), "✅ DobotDualHardware activated and ready!");

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to activate DobotDualHardware: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DobotDualHardware::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
{
    RCLCPP_INFO(get_node()->get_logger(), "Deactivating DobotDualHardware...");

    // 停止左夹爪控制线程
    if (has_left_gripper_ && left_gripper_thread_running_) {
        left_gripper_thread_running_ = false;
        if (left_gripper_control_thread_.joinable()) {
            left_gripper_control_thread_.join();
        }
        RCLCPP_INFO(get_node()->get_logger(), "LEFT gripper control thread stopped");
    }

    // 停止右夹爪控制线程
    if (has_right_gripper_ && right_gripper_thread_running_) {
        right_gripper_thread_running_ = false;
        if (right_gripper_control_thread_.joinable()) {
            right_gripper_control_thread_.join();
        }
        RCLCPP_INFO(get_node()->get_logger(), "RIGHT gripper control thread stopped");
    }

    // 断开TCP连接
    left_commander_.reset();
    right_commander_.reset();

    RCLCPP_INFO(get_node()->get_logger(), "✅ DobotDualHardware deactivated");

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DobotDualHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // 导出左臂6个关节的状态接口
    for (size_t i = 0; i < 6; ++i) {
        std::string joint_name = "left_joint" + std::to_string(i + 1);

        state_interfaces.emplace_back(
            joint_name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]);
        state_interfaces.emplace_back(
            joint_name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]);
        state_interfaces.emplace_back(
            joint_name, hardware_interface::HW_IF_EFFORT, &joint_efforts_[i]);
    }

    // 导出右臂6个关节的状态接口
    for (size_t i = 0; i < 6; ++i) {
        std::string joint_name = "right_joint" + std::to_string(i + 1);

        state_interfaces.emplace_back(
            joint_name, hardware_interface::HW_IF_POSITION, &joint_positions_[i + 6]);
        state_interfaces.emplace_back(
            joint_name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i + 6]);
        state_interfaces.emplace_back(
            joint_name, hardware_interface::HW_IF_EFFORT, &joint_efforts_[i + 6]);
    }

    // 导出左夹爪状态接口
    if (has_left_gripper_) {
        state_interfaces.emplace_back(
            left_gripper_joint_name_, hardware_interface::HW_IF_POSITION, &left_gripper_position_);
    }

    // 导出右夹爪状态接口
    if (has_right_gripper_) {
        state_interfaces.emplace_back(
            right_gripper_joint_name_, hardware_interface::HW_IF_POSITION, &right_gripper_position_);
    }

    RCLCPP_INFO(get_node()->get_logger(),
               "Exported %zu state interfaces (12 arm joints + %d grippers)",
               state_interfaces.size(), (has_left_gripper_ ? 1 : 0) + (has_right_gripper_ ? 1 : 0));

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DobotDualHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // 导出左臂6个关节的命令接口
    for (size_t i = 0; i < 6; ++i) {
        std::string joint_name = "left_joint" + std::to_string(i + 1);
        command_interfaces.emplace_back(
            joint_name, hardware_interface::HW_IF_POSITION, &joint_position_commands_[i]);
    }

    // 导出右臂6个关节的命令接口
    for (size_t i = 0; i < 6; ++i) {
        std::string joint_name = "right_joint" + std::to_string(i + 1);
        command_interfaces.emplace_back(
            joint_name, hardware_interface::HW_IF_POSITION, &joint_position_commands_[i + 6]);
    }

    // 导出左夹爪命令接口
    if (has_left_gripper_) {
        command_interfaces.emplace_back(
            left_gripper_joint_name_, hardware_interface::HW_IF_POSITION, &left_gripper_position_command_);
    }

    // 导出右夹爪命令接口
    if (has_right_gripper_) {
        command_interfaces.emplace_back(
            right_gripper_joint_name_, hardware_interface::HW_IF_POSITION, &right_gripper_position_command_);
    }

    RCLCPP_INFO(get_node()->get_logger(),
               "Exported %zu command interfaces (12 arm joints + %d grippers)",
               command_interfaces.size(), (has_left_gripper_ ? 1 : 0) + (has_right_gripper_ ? 1 : 0));

    return command_interfaces;
}

hardware_interface::return_type DobotDualHardware::read(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/)
{
    if (!left_commander_ || !left_commander_->isConnected() ||
        !right_commander_ || !right_commander_->isConnected()) {
        return hardware_interface::return_type::ERROR;
    }

    try {
        // 从左臂读取关节状态
        double left_current_joints[6];
        left_commander_->getCurrentJointStatus(left_current_joints);

        // 从右臂读取关节状态
        double right_current_joints[6];
        right_commander_->getCurrentJointStatus(right_current_joints);

        // 更新关节位置
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (size_t i = 0; i < 6; ++i) {
            joint_positions_[i] = left_current_joints[i];
            joint_positions_[i + 6] = right_current_joints[i];
        }

        // 获取左臂实时数据（速度和力矩）
        auto left_real_time_data = left_commander_->getRealData();
        if (left_real_time_data) {
            for (size_t i = 0; i < 6; ++i) {
                joint_velocities_[i] = left_real_time_data->qd_actual[i] * M_PI / 180.0;
                joint_efforts_[i] = left_real_time_data->m_actual[i];
            }
        }

        // 获取右臂实时数据（速度和力矩）
        auto right_real_time_data = right_commander_->getRealData();
        if (right_real_time_data) {
            for (size_t i = 0; i < 6; ++i) {
                joint_velocities_[i + 6] = right_real_time_data->qd_actual[i] * M_PI / 180.0;
                joint_efforts_[i + 6] = right_real_time_data->m_actual[i];
            }
        }

        // 夹爪状态由独立线程更新

    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(),
            *get_node()->get_clock(),
            1000,
            "Error reading joint states: %s", e.what()
        );
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DobotDualHardware::write(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/)
{
    if (!left_commander_ || !left_commander_->isConnected() ||
        !right_commander_ || !right_commander_->isConnected()) {
        return hardware_interface::return_type::ERROR;
    }

    try {
        // 获取关节命令
        double left_joint_cmd[6];
        double right_joint_cmd[6];

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (size_t i = 0; i < 6; ++i) {
                left_joint_cmd[i] = joint_position_commands_[i];
                right_joint_cmd[i] = joint_position_commands_[i + 6];
            }
        }

        // 发送左臂ServoJ命令
        bool left_success = left_commander_->servoJ(left_joint_cmd, left_servo_time_, left_aheadtime_, left_gain_);

        // 发送右臂ServoJ命令
        bool right_success = right_commander_->servoJ(right_joint_cmd, right_servo_time_, right_aheadtime_, right_gain_);

        if (!left_success || !right_success) {
            RCLCPP_ERROR_THROTTLE(
                get_node()->get_logger(),
                *get_node()->get_clock(),
                1000,
                "Failed to send ServoJ command (LEFT: %s, RIGHT: %s)",
                left_success ? "OK" : "FAIL",
                right_success ? "OK" : "FAIL"
            );
            return hardware_interface::return_type::ERROR;
        }

        // 频率统计（可选）
        if (verbose_) {
            write_count_++;
            auto now = std::chrono::steady_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_write_stat_time_);
            if (duration.count() >= 1) {
                RCLCPP_INFO(get_node()->get_logger(), "Write frequency: %d Hz", write_count_);
                write_count_ = 0;
                last_write_stat_time_ = now;
            }
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(
            get_node()->get_logger(),
            *get_node()->get_clock(),
            1000,
            "Error writing joint commands: %s", e.what()
        );
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}

// ==================== 夹爪控制辅助函数 ====================

bool DobotDualHardware::initializeLeftModbus()
{
    try {
        // 先关闭可能存在的历史 Modbus 连接，避免通道占用
        for (int i = 0; i < 5; i++) {
            left_commander_->modbusClose(i);  // 忽略失败
        }

        // 创建Modbus RTU连接 (slave_id=1, baud=115200, parity=N, data_bit=8, stop_bit=1)
        if (!left_commander_->modbusRTUCreate(1, 115200, "N", 8, 1)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to create LEFT Modbus RTU connection");
            return false;
        }

        RCLCPP_INFO(get_node()->get_logger(), "✅ LEFT gripper Modbus initialized");
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Error initializing LEFT gripper Modbus: %s", e.what());
        return false;
    }
}

bool DobotDualHardware::initializeRightModbus()
{
    try {
        // 先关闭可能存在的历史 Modbus 连接，避免通道占用
        for (int i = 0; i < 5; i++) {
            right_commander_->modbusClose(i);  // 忽略失败
        }

        // 创建Modbus RTU连接
        if (!right_commander_->modbusRTUCreate(1, 115200, "N", 8, 1)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to create RIGHT Modbus RTU connection");
            return false;
        }

        RCLCPP_INFO(get_node()->get_logger(), "✅ RIGHT gripper Modbus initialized");
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Error initializing RIGHT gripper Modbus: %s", e.what());
        return false;
    }
}

bool DobotDualHardware::controlLeftGripper(double position)
{
    try {
        // position: 0.0(闭合) - 1.0(打开)
        // 转换为 degree: 0-99
        int degree = static_cast<int>(position * 99.0);
        if (degree < 0) degree = 0;
        if (degree > 99) degree = 99;

        // 计算 Modbus 值：9000(闭合) - 90(打开)
        int modbus_value = static_cast<int>(9000 - degree * 9000.0 / 100.0);

        // 发送三个Modbus寄存器写入命令
        // 寄存器258: 控制模式 = 0
        if (!left_commander_->setHoldRegs(0, 258, 1, "{0}", "U16")) {
            return false;
        }

        // 寄存器259: 目标位置
        char val_buf[32];
        snprintf(val_buf, sizeof(val_buf), "{%d}", modbus_value);
        if (!left_commander_->setHoldRegs(0, 259, 1, val_buf, "U16")) {
            return false;
        }

        // 寄存器264: 执行命令 = 1
        if (!left_commander_->setHoldRegs(0, 264, 1, "{1}", "U16")) {
            return false;
        }

        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Error controlling LEFT gripper: %s", e.what());
        return false;
    }
}

bool DobotDualHardware::controlRightGripper(double position)
{
    try {
        // 与左夹爪相同的控制逻辑
        int degree = static_cast<int>(position * 99.0);
        if (degree < 0) degree = 0;
        if (degree > 99) degree = 99;

        int modbus_value = static_cast<int>(9000 - degree * 9000.0 / 100.0);

        if (!right_commander_->setHoldRegs(0, 258, 1, "{0}", "U16")) {
            return false;
        }

        char val_buf[32];
        snprintf(val_buf, sizeof(val_buf), "{%d}", modbus_value);
        if (!right_commander_->setHoldRegs(0, 259, 1, val_buf, "U16")) {
            return false;
        }

        if (!right_commander_->setHoldRegs(0, 264, 1, "{1}", "U16")) {
            return false;
        }

        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Error controlling RIGHT gripper: %s", e.what());
        return false;
    }
}

bool DobotDualHardware::readLeftGripperState(double &position)
{
    try {
        // 读取寄存器 0x60D (1549)，2个字节
        std::string result;
        if (!left_commander_->getHoldRegs(0, 0x60D, 2, "U16", result)) {
            return false;
        }

        // 解析结果: "{val1,val2}"
        if (result.size() < 3) {
            return false;
        }

        std::string data = result.substr(1, result.size() - 2);  // 移除 { }

        // 分割字符串
        size_t comma_pos = data.find(',');
        if (comma_pos == std::string::npos) {
            return false;
        }

        int val1 = std::stoi(data.substr(0, comma_pos));
        int val2 = std::stoi(data.substr(comma_pos + 1));

        // 计算位置：(9000 - ((val1 << 16) + val2)) / 9000 * 100
        int modbus_pos = (val1 << 16) + val2;
        double degree = (9000.0 - modbus_pos) / 9000.0 * 100.0;

        // 转换为 0.0-1.0
        position = degree / 99.0;
        if (position < 0.0) position = 0.0;
        if (position > 1.0) position = 1.0;

        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Error reading LEFT gripper state: %s", e.what());
        return false;
    }
}

bool DobotDualHardware::readRightGripperState(double &position)
{
    try {
        // 与左夹爪相同的读取逻辑
        std::string result;
        if (!right_commander_->getHoldRegs(0, 0x60D, 2, "U16", result)) {
            return false;
        }

        if (result.size() < 3) {
            return false;
        }

        std::string data = result.substr(1, result.size() - 2);

        size_t comma_pos = data.find(',');
        if (comma_pos == std::string::npos) {
            return false;
        }

        int val1 = std::stoi(data.substr(0, comma_pos));
        int val2 = std::stoi(data.substr(comma_pos + 1));

        int modbus_pos = (val1 << 16) + val2;
        double degree = (9000.0 - modbus_pos) / 9000.0 * 100.0;

        position = degree / 99.0;
        if (position < 0.0) position = 0.0;
        if (position > 1.0) position = 1.0;

        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Error reading RIGHT gripper state: %s", e.what());
        return false;
    }
}

void DobotDualHardware::leftGripperControlLoop()
{
    RCLCPP_INFO(get_node()->get_logger(), "LEFT gripper control loop started");
    
    double last_target = -1.0;
    auto last_read_time = std::chrono::steady_clock::now();
    const auto read_interval = std::chrono::milliseconds(100);  // 每100ms读取一次状态
    
    while (left_gripper_thread_running_) {
        try {
            // 直接读取 ros2_control 写入的命令值
            double target = left_gripper_position_command_;
            
            // 检查是否需要发送新命令
            if (std::abs(target - last_target) > 0.01) {
                // 目标位置变化，发送控制命令
                if (controlLeftGripper(target)) {
                    last_target = target;
                    
                    if (verbose_) {
                        RCLCPP_INFO(get_node()->get_logger(), "[LEFT Gripper] Target position: %.3f", target);
                    }
                } else {
                    static auto last_warn_time = std::chrono::steady_clock::now();
                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_warn_time).count() > 1000) {
                        RCLCPP_WARN(get_node()->get_logger(), "[LEFT Gripper] Failed to send command");
                        last_warn_time = now;
                    }
                }
            }
            
            // 定期读取夹爪状态
            auto current_time = std::chrono::steady_clock::now();
            if (current_time - last_read_time >= read_interval) {
                double gripper_pos;
                if (readLeftGripperState(gripper_pos)) {
                    std::lock_guard<std::mutex> lock(left_gripper_mutex_);
                    left_gripper_position_ = gripper_pos;
                }
                last_read_time = current_time;
            }
            
            // 休眠一小段时间，避免占用过多CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
        } catch (const std::exception& e) {
            static auto last_error_time = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_error_time).count() > 1000) {
                RCLCPP_ERROR(get_node()->get_logger(), "[LEFT Gripper] Error in control loop: %s", e.what());
                last_error_time = now;
            }
        }
    }
    
    RCLCPP_INFO(get_node()->get_logger(), "LEFT gripper control loop stopped");
}

void DobotDualHardware::rightGripperControlLoop()
{
    RCLCPP_INFO(get_node()->get_logger(), "RIGHT gripper control loop started");
    
    double last_target = -1.0;
    auto last_read_time = std::chrono::steady_clock::now();
    const auto read_interval = std::chrono::milliseconds(100);  // 每100ms读取一次状态
    
    while (right_gripper_thread_running_) {
        try {
            // 直接读取 ros2_control 写入的命令值
            double target = right_gripper_position_command_;
            
            // 检查是否需要发送新命令
            if (std::abs(target - last_target) > 0.01) {
                // 目标位置变化，发送控制命令
                if (controlRightGripper(target)) {
                    last_target = target;
                    
                    if (verbose_) {
                        RCLCPP_INFO(get_node()->get_logger(), "[RIGHT Gripper] Target position: %.3f", target);
                    }
                } else {
                    static auto last_warn_time = std::chrono::steady_clock::now();
                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_warn_time).count() > 1000) {
                        RCLCPP_WARN(get_node()->get_logger(), "[RIGHT Gripper] Failed to send command");
                        last_warn_time = now;
                    }
                }
            }
            
            // 定期读取夹爪状态
            auto current_time = std::chrono::steady_clock::now();
            if (current_time - last_read_time >= read_interval) {
                double gripper_pos;
                if (readRightGripperState(gripper_pos)) {
                    std::lock_guard<std::mutex> lock(right_gripper_mutex_);
                    right_gripper_position_ = gripper_pos;
                }
                last_read_time = current_time;
            }
            
            // 休眠一小段时间，避免占用过多CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
        } catch (const std::exception& e) {
            static auto last_error_time = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_error_time).count() > 1000) {
                RCLCPP_ERROR(get_node()->get_logger(), "[RIGHT Gripper] Error in control loop: %s", e.what());
                last_error_time = now;
            }
        }
    }
    
    RCLCPP_INFO(get_node()->get_logger(), "RIGHT gripper control loop stopped");
}

} // namespace dobot_dual_ros2_control

// 注册插件
PLUGINLIB_EXPORT_CLASS(
    dobot_dual_ros2_control::DobotDualHardware,
    hardware_interface::SystemInterface
)
