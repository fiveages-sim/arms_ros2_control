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

#include "dobot_ros2_control/dobot_hardware.h"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>

namespace dobot_ros2_control
{

hardware_interface::CallbackReturn DobotHardware::on_init(const hardware_interface::HardwareComponentInterfaceParams& params)
{
    if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    // 初始化夹爪数据
    gripper_position_ = 0.0;
    gripper_position_command_ = 0.0;
    has_gripper_ = false;
    gripper_thread_running_ = false;
    gripper_joint_index_ = -1;
    
    // 解析 URDF 配置的关节（使用基类成员变量 info_）
    int joint_index = 0;
    for (const auto& joint : info_.joints) {
        // 检查关节名称中是否包含 gripper 或 hand
        std::string joint_name_lower = joint.name;
        std::transform(joint_name_lower.begin(), joint_name_lower.end(), 
                      joint_name_lower.begin(), ::tolower);
        
        if (joint_name_lower.find("gripper") != std::string::npos || 
            joint_name_lower.find("hand") != std::string::npos) {
            // 这是夹爪关节
            has_gripper_ = true;
            gripper_joint_name_ = joint.name;
            gripper_joint_index_ = joint_index;
            RCLCPP_INFO(get_node()->get_logger(), "Detected gripper joint: %s (index %d)", 
                       gripper_joint_name_.c_str(), gripper_joint_index_);
        } else {
            // 这是机械臂关节
            joint_names_.push_back(joint.name);
        }
        joint_index++;
    }
    
    // 初始化关节数据存储（根据实际配置的关节数量）
    int arm_joint_count = static_cast<int>(joint_names_.size());
    joint_positions_.resize(arm_joint_count, 0.0);
    joint_velocities_.resize(arm_joint_count, 0.0);
    joint_efforts_.resize(arm_joint_count, 0.0);
    joint_position_commands_.resize(arm_joint_count, 0.0);
    
    RCLCPP_INFO(get_node()->get_logger(), "Found %d arm joints in configuration", arm_joint_count);
    if (has_gripper_) {
        RCLCPP_INFO(get_node()->get_logger(), "Found gripper: %s", gripper_joint_name_.c_str());
    }
    
    // 读取配置参数的辅助函数（使用基类成员变量 info_）
    const auto get_hardware_parameter = [this](const std::string& parameter_name, const std::string& default_value) {
        if (auto it = info_.hardware_parameters.find(parameter_name); it != info_.hardware_parameters.end()) {
            return it->second;
        }
        return default_value;
    };
    
    // 读取配置参数
    robot_ip_ = get_hardware_parameter("robot_ip", "192.168.5.38");
    servo_time_ = std::stod(get_hardware_parameter("servo_time", "0.03"));
    aheadtime_ = std::stod(get_hardware_parameter("aheadtime", "20.0"));
    gain_ = std::stod(get_hardware_parameter("gain", "500.0"));
    speed_factor_ = std::stoi(get_hardware_parameter("speed_factor", "5"));
    verbose_ = (get_hardware_parameter("verbose", "false") == "true");
    
    // 初始化统计变量
    write_count_ = 0;
    last_write_stat_time_ = std::chrono::steady_clock::now();
    
    RCLCPP_INFO(get_node()->get_logger(), "==============================================");
    RCLCPP_INFO(get_node()->get_logger(), "DobotHardware initialized successfully");
    RCLCPP_INFO(get_node()->get_logger(), "  Robot IP: %s", robot_ip_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "  ServoJ Time: %.3f s", servo_time_);
    RCLCPP_INFO(get_node()->get_logger(), "  Aheadtime: %.1f", aheadtime_);
    RCLCPP_INFO(get_node()->get_logger(), "  Gain: %.1f", gain_);
    RCLCPP_INFO(get_node()->get_logger(), "  Speed Factor: %d%%", speed_factor_);
    RCLCPP_INFO(get_node()->get_logger(), "  Verbose: %s", verbose_ ? "true" : "false");
    RCLCPP_INFO(get_node()->get_logger(), "  Arm Joints: %zu", joint_names_.size());
    if (has_gripper_) {
        RCLCPP_INFO(get_node()->get_logger(), "  Gripper: Yes (%s)", gripper_joint_name_.c_str());
    }
    RCLCPP_INFO(get_node()->get_logger(), "==============================================");
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DobotHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
    RCLCPP_INFO(get_node()->get_logger(), "Activating DobotHardware...");
    
    try {
        // 创建TCP连接到机器人（传递 verbose 参数）
        commander_ = std::make_shared<CRCommanderRos2>(robot_ip_, verbose_);
        commander_->init();
        
        RCLCPP_INFO(get_node()->get_logger(), "TCP connection initialized to %s", robot_ip_.c_str());
        
        // 等待TCP连接建立
        RCLCPP_INFO(get_node()->get_logger(), "Waiting for TCP connection...");
        
        auto start_time = std::chrono::steady_clock::now();
        const auto timeout = std::chrono::seconds(10);
        
        while (!commander_->isConnected()) {
            auto current_time = std::chrono::steady_clock::now();
            if (current_time - start_time > timeout) {
                RCLCPP_ERROR(get_node()->get_logger(), "Timeout waiting for TCP connection!");
                return hardware_interface::CallbackReturn::ERROR;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        RCLCPP_INFO(get_node()->get_logger(), "✅ TCP connection established!");
        
        // 设置全局速度
        RCLCPP_INFO(get_node()->get_logger(), "Setting global speed factor to %d%%...", speed_factor_);
        if (!commander_->setSpeedFactor(speed_factor_)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to set speed factor!");
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        // 如果配置了夹爪，初始化 Modbus
        if (has_gripper_) {
            if (!initializeModbus()) {
                RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize Modbus for gripper!");
                return hardware_interface::CallbackReturn::ERROR;
            }
            
            // 读取夹爪当前位置
            double gripper_pos;
            if (readGripperState(gripper_pos)) {
                gripper_position_ = gripper_pos;
                gripper_position_command_ = gripper_pos;
                RCLCPP_INFO(get_node()->get_logger(), "Initial gripper position: %.3f", gripper_pos);
            } else {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to read initial gripper position, using 0.0");
                gripper_position_ = 0.0;
                gripper_position_command_ = 0.0;
            }
            
            // 启动夹爪控制线程
            gripper_thread_running_ = true;
            gripper_control_thread_ = std::thread(&DobotHardware::gripperControlLoop, this);
            RCLCPP_INFO(get_node()->get_logger(), "✅ Gripper control thread started");
        }
        
        // 读取当前关节位置并初始化命令（机械臂关节）
        double current_joints[6];
        commander_->getCurrentJointStatus(current_joints);
        
        // 无需锁：ros2_control 保证 read/write 顺序执行，无并发访问
        size_t arm_joints = std::min(joint_names_.size(), size_t(6));  // 最多6个关节
        for (size_t i = 0; i < arm_joints; ++i) {
            joint_positions_[i] = current_joints[i];
            joint_position_commands_[i] = current_joints[i];
        }
        
        RCLCPP_INFO(get_node()->get_logger(), 
                   "Initial %zu arm joint positions read from robot", arm_joints);
        
        RCLCPP_INFO(get_node()->get_logger(), "✅ DobotHardware activated and ready!");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to activate DobotHardware: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DobotHardware::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
{
    RCLCPP_INFO(get_node()->get_logger(), "Deactivating DobotHardware...");
    
    // 停止夹爪控制线程
    if (has_gripper_ && gripper_thread_running_) {
        gripper_thread_running_ = false;
        if (gripper_control_thread_.joinable()) {
            gripper_control_thread_.join();
        }
        RCLCPP_INFO(get_node()->get_logger(), "Gripper control thread stopped");
    }
    
    // 断开TCP连接
    commander_.reset();
    
    RCLCPP_INFO(get_node()->get_logger(), "✅ DobotHardware deactivated");
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr> DobotHardware::on_export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
    
    // 为机械臂关节导出状态接口：位置、速度、力矩
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        state_interfaces.push_back(
            std::make_shared<hardware_interface::StateInterface>(
                joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
        
        state_interfaces.push_back(
            std::make_shared<hardware_interface::StateInterface>(
                joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));
        
        state_interfaces.push_back(
            std::make_shared<hardware_interface::StateInterface>(
                joint_names_[i], hardware_interface::HW_IF_EFFORT, &joint_efforts_[i]));
    }
    
    // 如果配置了夹爪，导出夹爪状态接口（只有位置）
    if (has_gripper_) {
        state_interfaces.push_back(
            std::make_shared<hardware_interface::StateInterface>(
                gripper_joint_name_, hardware_interface::HW_IF_POSITION, &gripper_position_));
        
        RCLCPP_INFO(get_node()->get_logger(), 
                   "Exported %zu state interfaces (%zu arm joints + 1 gripper)", 
                   state_interfaces.size(), joint_names_.size());
    } else {
        RCLCPP_INFO(get_node()->get_logger(), 
                   "Exported %zu state interfaces for %zu arm joints", 
                   state_interfaces.size(), joint_names_.size());
    }
    
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr> DobotHardware::on_export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
    
    // 为机械臂关节导出命令接口：位置命令
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        command_interfaces.push_back(
            std::make_shared<hardware_interface::CommandInterface>(
                joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_position_commands_[i]));
    }
    
    // 如果配置了夹爪，导出夹爪命令接口
    if (has_gripper_) {
        command_interfaces.push_back(
            std::make_shared<hardware_interface::CommandInterface>(
                gripper_joint_name_, hardware_interface::HW_IF_POSITION, &gripper_position_command_));
        
        RCLCPP_INFO(get_node()->get_logger(), 
                   "Exported %zu command interfaces (%zu arm joints + 1 gripper)", 
                   command_interfaces.size(), joint_names_.size());
    } else {
        RCLCPP_INFO(get_node()->get_logger(), 
                   "Exported %zu command interfaces for %zu arm joints", 
                   command_interfaces.size(), joint_names_.size());
    }
    
    return command_interfaces;
}

hardware_interface::return_type DobotHardware::read(
    const rclcpp::Time& /*time*/, 
    const rclcpp::Duration& /*period*/)
{
    if (!commander_ || !commander_->isConnected()) {
        return hardware_interface::return_type::ERROR;
    }
    
    try {
        // 从TCP连接读取当前关节状态
        double current_joints[6];
        commander_->getCurrentJointStatus(current_joints);
        
        // 更新关节位置（无需锁：ros2_control 框架保证顺序执行）
        size_t arm_joints = std::min(joint_names_.size(), size_t(6));  // 最多6个关节
        for (size_t i = 0; i < arm_joints; ++i) {
            joint_positions_[i] = current_joints[i];
        }
        
        // 获取实时数据（包含速度和力矩信息）
        auto real_time_data = commander_->getRealData();
        if (real_time_data) {
            for (size_t i = 0; i < arm_joints; ++i) {
                joint_velocities_[i] = real_time_data->qd_actual[i] * M_PI / 180.0;  // 转换为弧度/秒
                joint_efforts_[i] = real_time_data->m_actual[i];  // 力矩
            }
        }
        
        // 定期打印调试信息（仅 verbose 模式）
        if (verbose_) {
            // 只打印前几个关节的位置
            RCLCPP_DEBUG_THROTTLE(
                get_node()->get_logger(),
                *get_node()->get_clock(),
                1000,
                "Joint positions updated (%zu joints)", 
                joint_names_.size()
            );
        }
        
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

hardware_interface::return_type DobotHardware::write(
    const rclcpp::Time& /*time*/, 
    const rclcpp::Duration& /*period*/)
{
    if (!commander_ || !commander_->isConnected()) {
        return hardware_interface::return_type::ERROR;
    }
    
    try {
        // 获取关节命令（弧度）（无需锁：ros2_control 框架保证顺序执行）
        double joint_cmd[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        size_t arm_joints = std::min(joint_names_.size(), size_t(6));
        for (size_t i = 0; i < arm_joints; ++i) {
            joint_cmd[i] = joint_position_commands_[i];
        }
        
        // 通过ServoJ发送关节命令（包含提前量和增益参数）
        // 注意：ServoJ 始终发送6个关节值，未配置的关节发送0
        bool success = commander_->servoJ(joint_cmd, servo_time_, aheadtime_, gain_);
        
        if (!success) {
            RCLCPP_ERROR_THROTTLE(
                get_node()->get_logger(),
                *get_node()->get_clock(),
                1000,
                "Failed to send ServoJ command"
            );
            return hardware_interface::return_type::ERROR;
        }
        
        // 夹爪命令由独立线程读取 gripper_position_command_ 并处理
        
        // 统计写入频率（仅在 verbose 模式下显示）
        if (verbose_) {
            write_count_++;
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - last_write_stat_time_).count();
            
            if (elapsed >= 1000) {  // 每1秒输出一次
                double frequency = write_count_ * 1000.0 / elapsed;
                RCLCPP_INFO(get_node()->get_logger(),
                           "[Write] Frequency: %.2f Hz (%d cmds in %ld ms)",
                           frequency, write_count_, elapsed);
                
                // 重置计数器
                write_count_ = 0;
                last_write_stat_time_ = current_time;
            }
        }
        
        // 定期打印命令值（仅 verbose 模式）
        if (verbose_) {
            RCLCPP_DEBUG_THROTTLE(
                get_node()->get_logger(),
                *get_node()->get_clock(),
                1000,
                "Joint commands sent (%zu joints)", 
                joint_names_.size()
            );
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

bool DobotHardware::initializeModbus()
{
    if (!commander_ || !commander_->isConnected()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Cannot initialize Modbus: not connected");
        return false;
    }
    
    RCLCPP_INFO(get_node()->get_logger(), "Initializing Modbus for gripper control...");
    
    // 步骤1: 关闭所有已存在的Modbus连接（0-4）
    for (int i = 0; i < 5; i++) {
        commander_->modbusClose(i);
        // 不检查返回值，因为连接可能本来就不存在
    }
    
    // 步骤2: 创建新的Modbus RTU连接
    // 参数：slave_id=1, baud=115200, parity='N', data_bit=8, stop_bit=1
    if (!commander_->modbusRTUCreate(1, 115200, "N", 8, 1)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to create Modbus RTU connection");
        return false;
    }
    
    RCLCPP_INFO(get_node()->get_logger(), "✅ Modbus initialized successfully");
    return true;
}

bool DobotHardware::controlGripper(double position)
{
    // position: 0.0(闭合) - 1.0(打开)
    // 转换为 degree: 0-99
    int degree = static_cast<int>(position * 99.0);
    if (degree < 0) degree = 0;
    if (degree > 99) degree = 99;
    
    // 计算 Modbus 值：9000(闭合) - 90(打开)
    int modbus_value = static_cast<int>(9000 - degree * 9000.0 / 100.0);
    
    // 发送三个Modbus寄存器写入命令
    // 寄存器258: 控制模式 = 0
    if (!commander_->setHoldRegs(0, 258, 1, "{0}", "U16")) {
        return false;
    }
    
    // 寄存器259: 目标位置
    char val_buf[32];
    snprintf(val_buf, sizeof(val_buf), "{%d}", modbus_value);
    if (!commander_->setHoldRegs(0, 259, 1, val_buf, "U16")) {
        return false;
    }
    
    // 寄存器264: 执行命令 = 1
    if (!commander_->setHoldRegs(0, 264, 1, "{1}", "U16")) {
        return false;
    }
    
    return true;
}

bool DobotHardware::readGripperState(double &position)
{
    // 读取寄存器 0x60D (1549)，2个字节
    std::string result;
    if (!commander_->getHoldRegs(0, 0x60D, 2, "U16", result)) {
        return false;
    }
    
    // 解析结果: "{val1,val2}"
    // 去掉大括号
    if (result.size() < 3) {
        return false;
    }
    
    std::string data = result.substr(1, result.size() - 2);  // 移除 { }
    
    // 分割字符串
    size_t comma_pos = data.find(',');
    if (comma_pos == std::string::npos) {
        return false;
    }
    
    try {
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
    } catch (...) {
        return false;
    }
}

void DobotHardware::gripperControlLoop()
{
    std::cout << "[Gripper] Control loop started" << std::endl;
    
    double last_target = -1.0;
    auto last_read_time = std::chrono::steady_clock::now();
    const auto read_interval = std::chrono::milliseconds(100);  // 每100ms读取一次状态
    
    while (gripper_thread_running_) {
        try {
            // 直接读取 ros2_control 写入的命令值
            double target = gripper_position_command_;
            
            // 检查是否需要发送新命令
            if (std::abs(target - last_target) > 0.01) {
                // 目标位置变化，发送控制命令
                if (controlGripper(target)) {
                    last_target = target;
                    
                    if (verbose_) {
                        std::cout << "[Gripper] Target position: " << target << std::endl;
                    }
                } else {
                    static auto last_warn_time = std::chrono::steady_clock::now();
                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_warn_time).count() > 1000) {
                        std::cout << "[Gripper] Failed to send command" << std::endl;
                        last_warn_time = now;
                    }
                }
            }
            
            // 定期读取夹爪状态
            auto current_time = std::chrono::steady_clock::now();
            if (current_time - last_read_time >= read_interval) {
                double gripper_pos;
                if (readGripperState(gripper_pos)) {
                    // 无需锁：数据单向流动，只有此线程写入 gripper_position_
                    gripper_position_ = gripper_pos;
                }
                last_read_time = current_time;
            }
            
            // 休眠一小段时间，避免占用过多CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
        } catch (const std::exception& e) {
            static auto last_error_time = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_error_time).count() > 1000) {
                std::cout << "[Gripper] Control loop error: " << e.what() << std::endl;
                last_error_time = now;
            }
        }
    }
    
    std::cout << "[Gripper] Control loop stopped" << std::endl;
}

} // namespace dobot_ros2_control

// 导出插件
PLUGINLIB_EXPORT_CLASS(dobot_ros2_control::DobotHardware, hardware_interface::SystemInterface)
