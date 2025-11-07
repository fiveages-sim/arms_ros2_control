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
#include <map>

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
    last_gripper_command_ = 0.0;
    gripper_read_counter_ = 0;
    gripper_read_frequency_divider_ = 4;  // 默认值，将在参数读取时覆盖
    has_gripper_ = false;
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
    verbose_ = get_hardware_parameter("verbose", "false") == "true";
    wrench_frame_id_ = get_hardware_parameter("wrench_frame_id", "Link6");
    
    // 六维力传感器滤波器参数
    enable_wrench_filter_ = get_hardware_parameter("enable_wrench_filter", "true") == "true";
    wrench_filter_cutoff_freq_ = std::stod(get_hardware_parameter("wrench_filter_cutoff_freq", "10.0"));
    
    // 计算滤波器系数（假设采样频率为100Hz，一阶低通滤波器）
    // α = dt / (dt + τ)，其中 τ = 1/(2π*fc)，dt = 1/sample_rate
    // 对于100Hz采样频率：α = 1/100 / (1/100 + 1/(2π*fc))
    double sample_rate = 100.0;  // 假设控制频率为100Hz
    double dt = 1.0 / sample_rate;
    double tau = 1.0 / (2.0 * M_PI * wrench_filter_cutoff_freq_);
    wrench_filter_alpha_ = dt / (dt + tau);
    
    // 六维力传感器自动归零参数
    enable_wrench_auto_zero_ = get_hardware_parameter("enable_wrench_auto_zero", "true") == "true";
    wrench_zero_samples_ = std::stoi(get_hardware_parameter("wrench_zero_samples", "100"));  // 默认采样100次取平均
    wrench_zero_initialized_ = false;
    wrench_zero_sample_count_ = 0;
    
    // 夹爪读取频率参数
    gripper_read_frequency_divider_ = std::stoi(get_hardware_parameter("gripper_read_frequency_divider", "4"));  // 默认每4次read循环读取一次
    
    // 初始化力传感器偏移量
    ft_sensor_offset_force_x_ = 0.0;
    ft_sensor_offset_force_y_ = 0.0;
    ft_sensor_offset_force_z_ = 0.0;
    ft_sensor_offset_torque_x_ = 0.0;
    ft_sensor_offset_torque_y_ = 0.0;
    ft_sensor_offset_torque_z_ = 0.0;
    
    // 初始化滤波器状态变量
    ft_sensor_filtered_force_x_ = 0.0;
    ft_sensor_filtered_force_y_ = 0.0;
    ft_sensor_filtered_force_z_ = 0.0;
    ft_sensor_filtered_torque_x_ = 0.0;
    ft_sensor_filtered_torque_y_ = 0.0;
    ft_sensor_filtered_torque_z_ = 0.0;
    
    // 初始化力传感器数据（用于 hardware interface）
    ft_sensor_force_x_ = 0.0;
    ft_sensor_force_y_ = 0.0;
    ft_sensor_force_z_ = 0.0;
    ft_sensor_torque_x_ = 0.0;
    ft_sensor_torque_y_ = 0.0;
    ft_sensor_torque_z_ = 0.0;
    
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
    RCLCPP_INFO(get_node()->get_logger(), "  Wrench Frame ID: %s", wrench_frame_id_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "  Wrench Filter: %s (cutoff: %.1f Hz, alpha: %.4f)", 
                enable_wrench_filter_ ? "enabled" : "disabled", 
                wrench_filter_cutoff_freq_, wrench_filter_alpha_);
    RCLCPP_INFO(get_node()->get_logger(), "  Wrench Auto-Zero: %s (samples: %d)", 
                enable_wrench_auto_zero_ ? "enabled" : "disabled", wrench_zero_samples_);
    RCLCPP_INFO(get_node()->get_logger(), "  Arm Joints: %zu", joint_names_.size());
    if (has_gripper_) {
        RCLCPP_INFO(get_node()->get_logger(), "  Gripper: Yes (%s, read frequency divider: %d)", 
                   gripper_joint_name_.c_str(), gripper_read_frequency_divider_);
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
        
        // 六维力传感器自动归零：读取初始数据作为偏移量
        if (enable_wrench_auto_zero_) {
            RCLCPP_INFO(get_node()->get_logger(), "Initializing force-torque sensor auto-zero (sampling %d times)...", wrench_zero_samples_);
            
            // 等待传感器数据稳定（短暂延迟）
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // 累加器用于计算平均值
            double sum_force_x = 0.0, sum_force_y = 0.0, sum_force_z = 0.0;
            double sum_torque_x = 0.0, sum_torque_y = 0.0, sum_torque_z = 0.0;
            int valid_samples = 0;
            
            // 采样指定次数并计算平均值
            for (int i = 0; i < wrench_zero_samples_; ++i) {
                auto real_time_data = commander_->getRealData();
                if (real_time_data) {
                    sum_force_x += real_time_data->SixForceValue[0];
                    sum_force_y += real_time_data->SixForceValue[1];
                    sum_force_z += real_time_data->SixForceValue[2];
                    sum_torque_x += real_time_data->SixForceValue[3];
                    sum_torque_y += real_time_data->SixForceValue[4];
                    sum_torque_z += real_time_data->SixForceValue[5];
                    valid_samples++;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 10ms间隔，约100Hz
            }
            
            if (valid_samples > 0) {
                // 计算平均值作为偏移量
                ft_sensor_offset_force_x_ = sum_force_x / valid_samples;
                ft_sensor_offset_force_y_ = sum_force_y / valid_samples;
                ft_sensor_offset_force_z_ = sum_force_z / valid_samples;
                ft_sensor_offset_torque_x_ = sum_torque_x / valid_samples;
                ft_sensor_offset_torque_y_ = sum_torque_y / valid_samples;
                ft_sensor_offset_torque_z_ = sum_torque_z / valid_samples;
                
                wrench_zero_initialized_ = true;
                
                RCLCPP_INFO(get_node()->get_logger(), "✅ Force-torque sensor auto-zero completed:");
                RCLCPP_INFO(get_node()->get_logger(), "   Offset Force:  [%.4f, %.4f, %.4f] N", 
                           ft_sensor_offset_force_x_, ft_sensor_offset_force_y_, ft_sensor_offset_force_z_);
                RCLCPP_INFO(get_node()->get_logger(), "   Offset Torque: [%.4f, %.4f, %.4f] N·m", 
                           ft_sensor_offset_torque_x_, ft_sensor_offset_torque_y_, ft_sensor_offset_torque_z_);
            } else {
                RCLCPP_WARN(get_node()->get_logger(), "⚠️  Failed to read force-torque sensor data for auto-zero, offsets set to zero");
                wrench_zero_initialized_ = false;
            }
        } else {
            RCLCPP_INFO(get_node()->get_logger(), "Force-torque sensor auto-zero is disabled");
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
                last_gripper_command_ = gripper_pos;
                RCLCPP_INFO(get_node()->get_logger(), "Initial gripper position: %.3f", gripper_pos);
            } else {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to read initial gripper position, using 0.0");
                gripper_position_ = 0.0;
                gripper_position_command_ = 0.0;
                last_gripper_command_ = 0.0;
            }
            
            RCLCPP_INFO(get_node()->get_logger(), "✅ Gripper initialized (integrated control mode)");
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
    }
    
    // 导出六维力传感器状态接口（从配置中读取）
    hardware_interface::ComponentInfo ft_sensor_info;
    bool has_ft_sensor = findSensorByName("ft_sensor", ft_sensor_info);
    if (has_ft_sensor) {
        // 创建接口名称到值的映射
        std::map<std::string, double*> interface_map = {
            {"force_x", &ft_sensor_force_x_},
            {"force_y", &ft_sensor_force_y_},
            {"force_z", &ft_sensor_force_z_},
            {"torque_x", &ft_sensor_torque_x_},
            {"torque_y", &ft_sensor_torque_y_},
            {"torque_z", &ft_sensor_torque_z_}
        };
        
        // 按照配置中定义的顺序导出接口
        for (size_t i = 0; i < ft_sensor_info.state_interfaces.size() && i < interface_map.size(); ++i) {
            const std::string& interface_name = ft_sensor_info.state_interfaces[i].name;
            auto it = interface_map.find(interface_name);
            if (it != interface_map.end()) {
                state_interfaces.push_back(
                    std::make_shared<hardware_interface::StateInterface>(
                        ft_sensor_info.name, interface_name, it->second));
            } else {
                RCLCPP_WARN(get_node()->get_logger(), 
                           "Unknown ft_sensor interface: %s", interface_name.c_str());
            }
        }
        
        RCLCPP_INFO(get_node()->get_logger(), 
                   "Exported %zu ft_sensor state interfaces", ft_sensor_info.state_interfaces.size());
    } else {
        RCLCPP_WARN(get_node()->get_logger(), 
                   "ft_sensor not found in configuration, sensor interfaces will not be exported");
    }
    
    RCLCPP_INFO(get_node()->get_logger(), 
               "Exported %zu state interfaces (%zu arm joints%s%s)", 
               state_interfaces.size(), joint_names_.size(),
               has_gripper_ ? " + 1 gripper" : "",
               has_ft_sensor ? " + ft_sensor" : "");
    
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
            
            // 读取并处理六维力传感器数据（直接更新到 hardware interface）
            // SixForceValue通常包含：Fx, Fy, Fz, Mx, My, Mz
            // 前3个是力（Force），后3个是力矩（Torque）
            double raw_force_x = real_time_data->SixForceValue[0];
            double raw_force_y = real_time_data->SixForceValue[1];
            double raw_force_z = real_time_data->SixForceValue[2];
            double raw_torque_x = real_time_data->SixForceValue[3];
            double raw_torque_y = real_time_data->SixForceValue[4];
            double raw_torque_z = real_time_data->SixForceValue[5];
            
            // 应用低通滤波器（如果启用）
            // 使用独立的滤波器状态变量，避免归零影响滤波器状态
            if (enable_wrench_filter_) {
                ft_sensor_filtered_force_x_ = applyLowPassFilter(raw_force_x, ft_sensor_filtered_force_x_, wrench_filter_alpha_);
                ft_sensor_filtered_force_y_ = applyLowPassFilter(raw_force_y, ft_sensor_filtered_force_y_, wrench_filter_alpha_);
                ft_sensor_filtered_force_z_ = applyLowPassFilter(raw_force_z, ft_sensor_filtered_force_z_, wrench_filter_alpha_);
                ft_sensor_filtered_torque_x_ = applyLowPassFilter(raw_torque_x, ft_sensor_filtered_torque_x_, wrench_filter_alpha_);
                ft_sensor_filtered_torque_y_ = applyLowPassFilter(raw_torque_y, ft_sensor_filtered_torque_y_, wrench_filter_alpha_);
                ft_sensor_filtered_torque_z_ = applyLowPassFilter(raw_torque_z, ft_sensor_filtered_torque_z_, wrench_filter_alpha_);
            } else {
                // 不使用滤波器，直接使用原始值
                ft_sensor_filtered_force_x_ = raw_force_x;
                ft_sensor_filtered_force_y_ = raw_force_y;
                ft_sensor_filtered_force_z_ = raw_force_z;
                ft_sensor_filtered_torque_x_ = raw_torque_x;
                ft_sensor_filtered_torque_y_ = raw_torque_y;
                ft_sensor_filtered_torque_z_ = raw_torque_z;
            }
            
            // 应用自动归零偏移量（如果已初始化）
            if (enable_wrench_auto_zero_ && wrench_zero_initialized_) {
                ft_sensor_force_x_ = ft_sensor_filtered_force_x_ - ft_sensor_offset_force_x_;
                ft_sensor_force_y_ = ft_sensor_filtered_force_y_ - ft_sensor_offset_force_y_;
                ft_sensor_force_z_ = ft_sensor_filtered_force_z_ - ft_sensor_offset_force_z_;
                ft_sensor_torque_x_ = ft_sensor_filtered_torque_x_ - ft_sensor_offset_torque_x_;
                ft_sensor_torque_y_ = ft_sensor_filtered_torque_y_ - ft_sensor_offset_torque_y_;
                ft_sensor_torque_z_ = ft_sensor_filtered_torque_z_ - ft_sensor_offset_torque_z_;
            } else {
                // 不使用归零，直接使用滤波后的值（或原始值）
                ft_sensor_force_x_ = ft_sensor_filtered_force_x_;
                ft_sensor_force_y_ = ft_sensor_filtered_force_y_;
                ft_sensor_force_z_ = ft_sensor_filtered_force_z_;
                ft_sensor_torque_x_ = ft_sensor_filtered_torque_x_;
                ft_sensor_torque_y_ = ft_sensor_filtered_torque_y_;
                ft_sensor_torque_z_ = ft_sensor_filtered_torque_z_;
            }
        }
        
        // 夹爪状态读取：定期读取（降低读取频率以减少Modbus负载）
        if (has_gripper_) {
            gripper_read_counter_++;
            // 根据配置的频率除数读取夹爪状态（假设read频率为100Hz，则夹爪读取为100/N Hz）
            if (gripper_read_counter_ >= gripper_read_frequency_divider_) {
                gripper_read_counter_ = 0;
                
                double gripper_pos;
                if (readGripperState(gripper_pos)) {
                    gripper_position_ = gripper_pos;
                    
                    if (verbose_) {
                        RCLCPP_DEBUG_THROTTLE(
                            get_node()->get_logger(),
                            *get_node()->get_clock(),
                            1000,
                            "Gripper position updated: %.3f", gripper_pos
                        );
                    }
                } else {
                    RCLCPP_ERROR_THROTTLE(
                        get_node()->get_logger(),
                        *get_node()->get_clock(),
                        2000,
                        "Failed to read gripper state"
                    );
                }
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
        size_t arm_joints = std::min(joint_names_.size(), static_cast<size_t>(6));
        for (size_t i = 0; i < arm_joints; ++i) {
            joint_cmd[i] = joint_position_commands_[i];
        }
        
        // 通过ServoJ发送关节命令（异步发送，不等待响应）
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
        
        // 夹爪控制：检测命令变化并发送
        if (has_gripper_) {
            double target = gripper_position_command_;
            
            // 只在位置变化超过阈值时发送命令（避免频繁发送）
            if (std::abs(target - last_gripper_command_) > 0.01) {
                if (controlGripper(target)) {
                    last_gripper_command_ = target;
                    
                    if (verbose_) {
                        RCLCPP_DEBUG(get_node()->get_logger(), 
                                    "Gripper command sent: %.3f", target);
                    }
                } else {
                    RCLCPP_ERROR_THROTTLE(
                        get_node()->get_logger(),
                        *get_node()->get_clock(),
                        1000,
                        "Failed to send gripper command"
                    );
                }
            }
        }
        
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
    
    // 发送三个Modbus寄存器写入命令（异步发送，不等待响应）
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

bool DobotHardware::findSensorByName(const std::string& sensor_name, hardware_interface::ComponentInfo& sensor_info)
{
    for (const auto& sensor : info_.sensors) {
        if (sensor.name.find(sensor_name) != std::string::npos) {
            sensor_info = sensor;
            return true;
        }
    }
    return false;
}

double DobotHardware::applyLowPassFilter(double input, double& previous_output, double alpha)
{
    // 一阶低通滤波器：y[n] = α * x[n] + (1 - α) * y[n-1]
    previous_output = alpha * input + (1.0 - alpha) * previous_output;
    return previous_output;
}

} // namespace dobot_ros2_control

// 导出插件
PLUGINLIB_EXPORT_CLASS(dobot_ros2_control::DobotHardware, hardware_interface::SystemInterface)
