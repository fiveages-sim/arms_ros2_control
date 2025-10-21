// Copyright 2024 Rokae Team
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

#include "rokae_ros2_control/rokae_hardware.h"
#include <pluginlib/class_list_macros.hpp>
#include <sstream>
#include <iomanip>

namespace rokae_ros2_control
{

hardware_interface::CallbackReturn RokaeHardware::on_init(const hardware_interface::HardwareInfo& info)
{
    if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    // 解析 URDF 配置的关节
    for (const auto& joint : info.joints) {
        joint_names_.push_back(joint.name);
    }
    
    // 初始化关节数据存储（根据实际配置的关节数量）
    int joint_count = static_cast<int>(joint_names_.size());
    joint_positions_.resize(joint_count, 0.0);
    joint_velocities_.resize(joint_count, 0.0);
    joint_efforts_.resize(joint_count, 0.0);
    joint_position_commands_.resize(joint_count, 0.0);
    
    RCLCPP_INFO(get_node()->get_logger(), "Found %d joints in configuration", joint_count);
    
    // 读取配置参数的辅助函数
    const auto get_hardware_parameter = [&info](const std::string& parameter_name, const std::string& default_value) {
        if (auto it = info.hardware_parameters.find(parameter_name); it != info.hardware_parameters.end()) {
            return it->second;
        }
        return default_value;
    };
    
    // 读取配置参数
    arm_ip_ = get_hardware_parameter("arm_ip", "192.168.2.161");
    local_ip_ = get_hardware_parameter("local_ip", "192.168.2.233");
    
    RCLCPP_INFO(get_node()->get_logger(), "==============================================");
    RCLCPP_INFO(get_node()->get_logger(), "RokaeHardware initialized successfully");
    RCLCPP_INFO(get_node()->get_logger(), "  Arm IP: %s", arm_ip_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "  Local IP: %s", local_ip_.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "  Number of joints: %d", joint_count);
    RCLCPP_INFO(get_node()->get_logger(), "==============================================");
    
    // 初始化xCoreSDK
    try {
        initializeXCoreSDK();
        RCLCPP_INFO(get_node()->get_logger(), "✅ xCoreSDK initialized successfully");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize xCoreSDK: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RokaeHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
    RCLCPP_INFO(get_node()->get_logger(), "Activating RokaeHardware...");
    
    if (!arm_robot_) {
        RCLCPP_ERROR(get_node()->get_logger(), "Robot not initialized!");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    try {
        std::error_code ec;
        
        // 上电
        arm_robot_->setPowerState(true, ec);
        if (ec) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to power on arm: %s", ec.message().c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        RCLCPP_INFO(get_node()->get_logger(), "✅ Arm powered on");
        
        // 从当前机器人位置初始化命令位置，避免突然跳跃
        auto current_pos = arm_robot_->jointPos(ec);
        if (!ec) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (size_t i = 0; i < joint_names_.size(); ++i) {
                joint_positions_[i] = current_pos[i];
                joint_position_commands_[i] = current_pos[i];
            }
            
            // 打印初始位置（动态格式化）
            std::stringstream ss;
            ss << "Initial joint positions (rad): [";
            for (size_t i = 0; i < joint_names_.size(); ++i) {
                ss << std::fixed << std::setprecision(3) << current_pos[i];
                if (i < joint_names_.size() - 1) ss << ", ";
            }
            ss << "]";
            RCLCPP_INFO(get_node()->get_logger(), "%s", ss.str().c_str());
        } else {
            RCLCPP_WARN(get_node()->get_logger(), "Failed to read current position: %s", ec.message().c_str());
        }
        
        RCLCPP_INFO(get_node()->get_logger(), "✅ RokaeHardware activated and ready!");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to activate arm: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RokaeHardware::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
{
    RCLCPP_INFO(get_node()->get_logger(), "Deactivating RokaeHardware...");
    
    // 停止实时控制
    if (rt_control_running_) {
        stopRealtimeControl();
    }
    
    // 停止状态更新
    if (state_update_running_) {
        stopStateUpdate();
    }
    
    // 下电
    if (arm_robot_) {
        try {
            std::error_code ec;
            
            // 下电
            arm_robot_->setPowerState(false, ec);
            if (ec) {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to power off arm: %s", ec.message().c_str());
            }
            
            // 设置运动控制模式为空闲
            arm_robot_->setMotionControlMode(rokae::MotionControlMode::Idle, ec);
            if (ec) {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to set arm to idle mode: %s", ec.message().c_str());
            }
            
            RCLCPP_INFO(get_node()->get_logger(), "✅ Arm powered off and set to idle mode");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to deactivate arm: %s", e.what());
        }
    }
    
    RCLCPP_INFO(get_node()->get_logger(), "✅ RokaeHardware deactivated");
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RokaeHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    
    // 为所有关节导出状态接口：位置、速度、力矩
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        state_interfaces.emplace_back(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_positions_[i]);
        
        state_interfaces.emplace_back(
            joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]);
        
        state_interfaces.emplace_back(
            joint_names_[i], hardware_interface::HW_IF_EFFORT, &joint_efforts_[i]);
    }
    
    RCLCPP_INFO(get_node()->get_logger(), 
               "Exported %zu state interfaces for %zu joints", 
               state_interfaces.size(), joint_names_.size());
    
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RokaeHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    
    // 为所有关节导出命令接口：位置命令
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        command_interfaces.emplace_back(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_position_commands_[i]);
    }
    
    RCLCPP_INFO(get_node()->get_logger(), 
               "Exported %zu command interfaces for %zu joints", 
               command_interfaces.size(), joint_names_.size());
    
    return command_interfaces;
}

hardware_interface::return_type RokaeHardware::read(
    const rclcpp::Time& /*time*/, 
    const rclcpp::Duration& /*period*/)
{
    // 从缓存的状态数据获取关节信息
    if (arm_robot_ && state_receive_started_) {
        // 获取关节位置
        std::array<double, 7> positions{};
        int pos_result = arm_robot_->getStateData(rokae::RtSupportedFields::jointPos_m, positions);
        if (pos_result == 0) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (size_t i = 0; i < joint_names_.size(); ++i) {
                joint_positions_[i] = positions[i];
            }
            
            // 计算位置误差
            double max_error = 0.0;
            for (size_t i = 0; i < joint_names_.size(); ++i) {
                double error = std::abs(joint_position_commands_[i] - positions[i]);
                max_error = std::max(max_error, error);
            }
            RCLCPP_DEBUG_THROTTLE(
                get_node()->get_logger(),
                *get_node()->get_clock(),
                1000,
                "Max position error: %.4f rad", max_error
            );
        }
        
        // 获取关节速度
        std::array<double, 7> velocities{};
        if (arm_robot_->getStateData(rokae::RtSupportedFields::jointVel_m, velocities) == 0) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (size_t i = 0; i < joint_names_.size(); ++i) {
                joint_velocities_[i] = velocities[i];
            }
        }
        
        // 获取关节力矩
        std::array<double, 7> efforts{};
        if (arm_robot_->getStateData(rokae::RtSupportedFields::tau_m, efforts) == 0) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (size_t i = 0; i < joint_names_.size(); ++i) {
                joint_efforts_[i] = efforts[i];
            }
        }
        
        // 定期打印调试信息（动态格式化）
        std::stringstream ss;
        ss << "Joint positions (rad): [";
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            ss << std::fixed << std::setprecision(3) << joint_positions_[i];
            if (i < joint_names_.size() - 1) ss << ", ";
        }
        ss << "]";
        RCLCPP_DEBUG_THROTTLE(
            get_node()->get_logger(),
            *get_node()->get_clock(),
            1000,
            "%s", ss.str().c_str()
        );
    }
    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RokaeHardware::write(
    const rclcpp::Time& /*time*/, 
    const rclcpp::Duration& /*period*/)
{
    // 启动实时控制线程（类似Dobot的延迟启动策略）
    if (arm_robot_ && !rt_control_running_) {
        startRealtimeControl();
    }
    
    // 定期打印命令值
    std::stringstream ss;
    ss << "Joint commands (rad): [";
    for (size_t i = 0; i < joint_names_.size(); ++i) {
        ss << std::fixed << std::setprecision(3) << joint_position_commands_[i];
        if (i < joint_names_.size() - 1) ss << ", ";
    }
    ss << "]";
    RCLCPP_DEBUG_THROTTLE(
        get_node()->get_logger(),
        *get_node()->get_clock(),
        1000,
        "%s", ss.str().c_str()
    );
    
    return hardware_interface::return_type::OK;
}

void RokaeHardware::initializeXCoreSDK()
{
    try {
        // 初始化机器人
        arm_robot_ = std::make_unique<rokae::xMateErProRobot>();
        arm_robot_->connectToRobot(arm_ip_, local_ip_);
        
        RCLCPP_INFO(get_node()->get_logger(), "Connected to robot at %s", arm_ip_.c_str());
        
        // 设置机器人操作模式为自动模式
        std::error_code ec;
        arm_robot_->setOperateMode(rokae::OperateMode::automatic, ec);
        if (ec) {
            RCLCPP_WARN(get_node()->get_logger(), "Failed to set operate mode: %s", ec.message().c_str());
        }
        
        // 设置运动控制模式为实时模式
        arm_robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
        if (ec) {
            RCLCPP_WARN(get_node()->get_logger(), "Failed to set motion control mode: %s", ec.message().c_str());
        }
        
        // 获取实时控制器
        rt_controller_ = arm_robot_->getRtMotionController().lock();
        
        if (!rt_controller_) {
            throw std::runtime_error("Failed to get realtime motion controller");
        }
        
        // 初始化实时控制状态
        rt_control_running_ = false;
        state_receive_started_ = false;
        state_update_running_ = false;
        
        // 启动状态接收
        try {
            arm_robot_->startReceiveRobotState(
                std::chrono::milliseconds(1), 
                {rokae::RtSupportedFields::jointPos_m, 
                 rokae::RtSupportedFields::jointVel_m, 
                 rokae::RtSupportedFields::tau_m}
            );
            
            state_receive_started_ = true;
            RCLCPP_INFO(get_node()->get_logger(), "Robot state receiving started");
            
            // 等待状态接收稳定
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            
            // 启动状态更新线程
            startStateUpdate();
            
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_node()->get_logger(), "Failed to start robot state receiving: %s", e.what());
            state_receive_started_ = false;
        }
        
        RCLCPP_INFO(get_node()->get_logger(), "xCoreSDK initialized for arm: %s", arm_ip_.c_str());
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize xCoreSDK: %s", e.what());
        throw;
    }
}

void RokaeHardware::startRealtimeControl()
{
    if (rt_control_running_) {
        return;
    }
    
    try {
        // 启动实时控制线程
        rt_control_running_ = true;
        rt_control_thread_ = std::thread(&RokaeHardware::controlLoop, this);
        
        RCLCPP_INFO(get_node()->get_logger(), "Realtime control started");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to start realtime control: %s", e.what());
        rt_control_running_ = false;
    }
}

void RokaeHardware::stopRealtimeControl()
{
    if (!rt_control_running_) {
        return;
    }
    
    rt_control_running_ = false;
    
    // 等待线程结束
    if (rt_control_thread_.joinable()) {
        rt_control_thread_.join();
    }
    
    // 停止运动
    if (rt_controller_) {
        rt_controller_->stopMove();
    }
    
    RCLCPP_INFO(get_node()->get_logger(), "Realtime control stopped");
}

void RokaeHardware::controlLoop()
{
    try {
        // 设置控制回调
        std::function<rokae::JointPosition()> callback = [this]() {
            std::vector<double> joint_positions(joint_names_.size());
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                for (size_t i = 0; i < joint_names_.size(); ++i) {
                    joint_positions[i] = joint_position_commands_[i];
                }
            }
            return rokae::JointPosition(joint_positions);
        };
        
        rt_controller_->setControlLoop(callback);
        rt_controller_->startMove(rokae::RtControllerMode::jointPosition);
        rt_controller_->startLoop(true);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Control loop error: %s", e.what());
    }
}

void RokaeHardware::startStateUpdate()
{
    if (state_update_running_) {
        return;
    }
    
    try {
        state_update_running_ = true;
        state_update_thread_ = std::thread(&RokaeHardware::stateUpdateLoop, this);
        RCLCPP_INFO(get_node()->get_logger(), "State update thread started");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to start state update thread: %s", e.what());
        state_update_running_ = false;
    }
}

void RokaeHardware::stopStateUpdate()
{
    if (!state_update_running_) {
        return;
    }
    
    state_update_running_ = false;
    
    if (state_update_thread_.joinable()) {
        state_update_thread_.join();
    }
    
    RCLCPP_INFO(get_node()->get_logger(), "State update thread stopped");
}

void RokaeHardware::stateUpdateLoop()
{
    while (state_update_running_ && arm_robot_) {
        try {
            // 更新机器人状态
            arm_robot_->updateRobotState(std::chrono::microseconds(100));
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(
                get_node()->get_logger(),
                *get_node()->get_clock(),
                1000,
                "State update error: %s", e.what()
            );
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

} // namespace rokae_ros2_control

// 导出插件
PLUGINLIB_EXPORT_CLASS(rokae_ros2_control::RokaeHardware, hardware_interface::SystemInterface)

