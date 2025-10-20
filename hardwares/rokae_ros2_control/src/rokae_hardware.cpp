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

namespace rokae_ros2_control
{

hardware_interface::CallbackReturn RokaeHardware::on_init(const hardware_interface::HardwareInfo& info)
{
    if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    // 初始化关节数据存储 - 支持所有关节的所有接口
    // 底盘关节 (2个关节)
    chassis_positions_.resize(2, 0.0);
    chassis_velocities_.resize(2, 0.0);
    chassis_efforts_.resize(2, 0.0);
    chassis_velocity_commands_.resize(2, 0.0);
    
    // 身体关节 (4个关节)
    body_positions_.resize(4, 0.0);
    body_velocities_.resize(4, 0.0);
    body_efforts_.resize(4, 0.0);
    body_position_commands_.resize(4, 0.0);
    
    // 头部关节 (2个关节)
    head_positions_.resize(2, 0.0);
    head_velocities_.resize(2, 0.0);
    head_efforts_.resize(2, 0.0);
    head_position_commands_.resize(2, 0.0);
    
    // 左臂关节 (7个关节) - 真实数据
    left_arm_positions_.resize(7, 0.0);
    left_arm_velocities_.resize(7, 0.0);
    left_arm_efforts_.resize(7, 0.0);
    left_arm_position_commands_.resize(7, 0.0);
    
    // 右臂关节 (7个关节) - 真实数据
    right_arm_positions_.resize(7, 0.0);
    right_arm_velocities_.resize(7, 0.0);
    right_arm_efforts_.resize(7, 0.0);
    right_arm_position_commands_.resize(7, 0.0);
    
    // 读取配置参数
    const auto get_hardware_parameter = [&info](const std::string& parameter_name, const std::string& default_value) {
        if (auto it = info.hardware_parameters.find(parameter_name); it != info.hardware_parameters.end()) {
            return it->second;
        }
        return default_value;
    };
    
    update_rate_ = std::stod(get_hardware_parameter("update_rate", "100.0"));
    
    // 读取xCoreSDK配置参数
    left_arm_ip_ = get_hardware_parameter("left_arm_ip", "192.168.2.161");
    right_arm_ip_ = get_hardware_parameter("right_arm_ip", "192.168.2.162");
    local_ip_ = get_hardware_parameter("local_ip", "192.168.2.233");
    
    
    RCLCPP_INFO(get_node()->get_logger(), "RokaeHardware initialized successfully");
    RCLCPP_INFO(get_node()->get_logger(), "Update rate: %.1f Hz", update_rate_);
    
    // 初始化xCoreSDK
    try {
        initializeXCoreSDK();
        RCLCPP_INFO(get_node()->get_logger(), "xCoreSDK initialized successfully");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize xCoreSDK: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RokaeHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */)
{
    // 在激活时上电
    if (left_arm_robot_ && right_arm_robot_) {
        try {
            std::error_code ec;
            
            // 上电（操作模式和运动控制模式已在初始化时设置）
            left_arm_robot_->setPowerState(true, ec);
            if (ec) {
                RCLCPP_ERROR(get_node()->get_logger(), "Failed to power on left arm: %s", ec.message().c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
            
            right_arm_robot_->setPowerState(true, ec);
            if (ec) {
                RCLCPP_ERROR(get_node()->get_logger(), "Failed to power on right arm: %s", ec.message().c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
            
            // 从当前机器人位置初始化命令位置，避免突然跳跃
            auto left_current_pos = left_arm_robot_->jointPos(ec);
            if (!ec) {
                for (size_t i = 0; i < 7; ++i) {
                    left_arm_position_commands_[i] = left_current_pos[i];
                }
                RCLCPP_INFO(get_node()->get_logger(), "Left arm commands initialized from current position: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                           left_current_pos[0], left_current_pos[1], left_current_pos[2], left_current_pos[3],
                           left_current_pos[4], left_current_pos[5], left_current_pos[6]);
            } else {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to read left arm current position: %s", ec.message().c_str());
            }
            
            auto right_current_pos = right_arm_robot_->jointPos(ec);
            if (!ec) {
                for (size_t i = 0; i < 7; ++i) {
                    right_arm_position_commands_[i] = right_current_pos[i];
                }
                RCLCPP_INFO(get_node()->get_logger(), "Right arm commands initialized from current position: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                           right_current_pos[0], right_current_pos[1], right_current_pos[2], right_current_pos[3],
                           right_current_pos[4], right_current_pos[5], right_current_pos[6]);
            } else {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to read right arm current position: %s", ec.message().c_str());
            }
            
            
            RCLCPP_INFO(get_node()->get_logger(), "Robots powered on and ready for realtime control");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to activate robots: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RokaeHardware::on_deactivate(const rclcpp_lifecycle::State & /* previous_state */)
{
    // 停止实时控制
    if (rt_control_running_) {
        stopRealtimeControl();
    }
    
    // 停止状态更新
    if (state_update_running_) {
        stopStateUpdate();
    }
    
    // 在停用时下电
    if (left_arm_robot_ && right_arm_robot_) {
        try {
            std::error_code ec;
            
            // 下电
            left_arm_robot_->setPowerState(false, ec);
            if (ec) {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to power off left arm: %s", ec.message().c_str());
            }
            
            right_arm_robot_->setPowerState(false, ec);
            if (ec) {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to power off right arm: %s", ec.message().c_str());
            }
            
            // 设置运动控制模式为空闲
            left_arm_robot_->setMotionControlMode(rokae::MotionControlMode::Idle, ec);
            if (ec) {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to set left arm to idle mode: %s", ec.message().c_str());
            }
            
            right_arm_robot_->setMotionControlMode(rokae::MotionControlMode::Idle, ec);
            if (ec) {
                RCLCPP_WARN(get_node()->get_logger(), "Failed to set right arm to idle mode: %s", ec.message().c_str());
            }
            
            RCLCPP_INFO(get_node()->get_logger(), "Robots powered off and set to idle mode");
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to deactivate robots: %s", e.what());
        }
    }
    
    return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> RokaeHardware::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    
    // 底盘关节状态接口 (2个关节)
    for (size_t i = 0; i < 2; ++i) {
        std::string joint_name = (i == 0) ? "chassis_left_joint" : "chassis_right_joint";
        state_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, &chassis_positions_[i]);
        state_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY, &chassis_velocities_[i]);
        state_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_EFFORT, &chassis_efforts_[i]);
    }
    
    // 身体关节状态接口 (4个关节)
    for (size_t i = 0; i < 4; ++i) {
        std::string joint_name = "body_joint" + std::to_string(i + 1);
        state_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, &body_positions_[i]);
        state_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY, &body_velocities_[i]);
        state_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_EFFORT, &body_efforts_[i]);
    }
    
    // 头部关节状态接口 (2个关节)
    for (size_t i = 0; i < 2; ++i) {
        std::string joint_name = "head_joint" + std::to_string(i + 1);
        state_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, &head_positions_[i]);
        state_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY, &head_velocities_[i]);
        state_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_EFFORT, &head_efforts_[i]);
    }
    
    // 左臂关节状态接口 (7个关节) - 真实数据
    for (size_t i = 0; i < 7; ++i) {
        std::string joint_name = "left_joint" + std::to_string(i + 1);
        state_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, &left_arm_positions_[i]);
        state_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY, &left_arm_velocities_[i]);
        state_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_EFFORT, &left_arm_efforts_[i]);
    }
    
    // 右臂关节状态接口 (7个关节) - 真实数据
    for (size_t i = 0; i < 7; ++i) {
        std::string joint_name = "right_joint" + std::to_string(i + 1);
        state_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, &right_arm_positions_[i]);
        state_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY, &right_arm_velocities_[i]);
        state_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_EFFORT, &right_arm_efforts_[i]);
    }
    
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RokaeHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    
    // 底盘关节命令接口 (2个关节) - 只支持速度命令
    for (size_t i = 0; i < 2; ++i) {
        std::string joint_name = (i == 0) ? "chassis_left_joint" : "chassis_right_joint";
        command_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY, &chassis_velocity_commands_[i]);
    }
    
    // 身体关节命令接口 (4个关节) - 只支持位置命令
    for (size_t i = 0; i < 4; ++i) {
        std::string joint_name = "body_joint" + std::to_string(i + 1);
        command_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, &body_position_commands_[i]);
    }
    
    // 头部关节命令接口 (2个关节) - 只支持位置命令
    for (size_t i = 0; i < 2; ++i) {
        std::string joint_name = "head_joint" + std::to_string(i + 1);
        command_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, &head_position_commands_[i]);
    }
    
    // 左臂关节命令接口 (7个关节) - 只支持位置命令
    for (size_t i = 0; i < 7; ++i) {
        std::string joint_name = "left_joint" + std::to_string(i + 1);
        command_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, &left_arm_position_commands_[i]);
    }
    
    // 右臂关节命令接口 (7个关节) - 只支持位置命令
    for (size_t i = 0; i < 7; ++i) {
        std::string joint_name = "right_joint" + std::to_string(i + 1);
        command_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, &right_arm_position_commands_[i]);
    }
    
    return command_interfaces;
}

hardware_interface::return_type RokaeHardware::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // 从缓存的状态数据获取关节位置
    if (left_arm_robot_ && right_arm_robot_ && state_receive_started_) {
        // 获取左臂关节位置
        std::array<double, 7> left_positions{};
        int left_pos_result = left_arm_robot_->getStateData(rokae::RtSupportedFields::jointPos_m, left_positions);
        if (left_pos_result == 0) {
            for (size_t i = 0; i < 7; ++i) {
                left_arm_positions_[i] = left_positions[i];
            }
            
            // 计算位置指令和实际位置的差异
            double max_error = 0.0;
            for (size_t i = 0; i < 7; ++i) {
                double error = std::abs(left_arm_position_commands_[i] - left_positions[i]);
                max_error = std::max(max_error, error);
            }
            RCLCPP_INFO(get_node()->get_logger(), "Left arm max position error: %.4f rad", max_error);
        }
        
        // 获取右臂关节位置
        std::array<double, 7> right_positions{};
        int right_pos_result = right_arm_robot_->getStateData(rokae::RtSupportedFields::jointPos_m, right_positions);
        if (right_pos_result == 0) {
            for (size_t i = 0; i < 7; ++i) {
                right_arm_positions_[i] = right_positions[i];
            }
            
            // 计算位置指令和实际位置的差异
            double max_error = 0.0;
            for (size_t i = 0; i < 7; ++i) {
                double error = std::abs(right_arm_position_commands_[i] - right_positions[i]);
                max_error = std::max(max_error, error);
            }
            RCLCPP_INFO(get_node()->get_logger(), "Right arm max position error: %.4f rad", max_error);
        }
        
        // 获取左臂关节速度
        std::array<double, 7> left_velocities{};
        if (left_arm_robot_->getStateData(rokae::RtSupportedFields::jointVel_m, left_velocities) == 0) {
            for (size_t i = 0; i < 7; ++i) {
                left_arm_velocities_[i] = left_velocities[i];
            }
        }
        
        // 获取右臂关节速度
        std::array<double, 7> right_velocities{};
        if (right_arm_robot_->getStateData(rokae::RtSupportedFields::jointVel_m, right_velocities) == 0) {
            for (size_t i = 0; i < 7; ++i) {
                right_arm_velocities_[i] = right_velocities[i];
            }
        }
        
        // 获取左臂关节力矩
        std::array<double, 7> left_efforts{};
        if (left_arm_robot_->getStateData(rokae::RtSupportedFields::tau_m, left_efforts) == 0) {
            for (size_t i = 0; i < 7; ++i) {
                left_arm_efforts_[i] = left_efforts[i];
            }
        }
        
        // 获取右臂关节力矩
        std::array<double, 7> right_efforts{};
        if (right_arm_robot_->getStateData(rokae::RtSupportedFields::tau_m, right_efforts) == 0) {
            for (size_t i = 0; i < 7; ++i) {
                right_arm_efforts_[i] = right_efforts[i];
            }
        }
    } else {
        // 如果状态接收未启动，使用备用方法
        updateJointStates();
    }
    
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type RokaeHardware::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // 启动实时控制线程
    if (left_arm_robot_ && right_arm_robot_) {
        if (!rt_control_running_) {
            startRealtimeControl();
        }
        return hardware_interface::return_type::OK;
    }
    
    return hardware_interface::return_type::OK;
}

void RokaeHardware::updateJointStates()
{
    // 更新所有关节状态
    // 只有手臂关节有真实数据，其他关节保持零值
    
    // 底盘关节 - 保持零值
    for (size_t i = 0; i < 2; ++i) {
        chassis_positions_[i] = 0.0;
        chassis_velocities_[i] = 0.0;
        chassis_efforts_[i] = 0.0;
    }
    
    // 身体关节 - 保持零值
    for (size_t i = 0; i < 4; ++i) {
        body_positions_[i] = 0.0;
        body_velocities_[i] = 0.0;
        body_efforts_[i] = 0.0;
    }
    
    // 头部关节 - 保持零值
    for (size_t i = 0; i < 2; ++i) {
        head_positions_[i] = 0.0;
        head_velocities_[i] = 0.0;
        head_efforts_[i] = 0.0;
    }
    
    // 左臂和右臂关节 - 使用真实数据（已在arm_state_callback中更新）
    // 这里可以添加速度估算逻辑，目前保持零值
    for (size_t i = 0; i < 7; ++i) {
        left_arm_velocities_[i] = 0.0;
        left_arm_efforts_[i] = 0.0;
        right_arm_velocities_[i] = 0.0;
        right_arm_efforts_[i] = 0.0;
    }
}

void RokaeHardware::initializeXCoreSDK()
{
    try {
        // 初始化左臂机器人
        left_arm_robot_ = std::make_unique<rokae::xMateErProRobot>();
        left_arm_robot_->connectToRobot(left_arm_ip_, local_ip_);
        
        // 初始化右臂机器人
        right_arm_robot_ = std::make_unique<rokae::xMateErProRobot>();
        right_arm_robot_->connectToRobot(right_arm_ip_, local_ip_);
        
        // 设置机器人操作模式为自动模式
        std::error_code ec;
        left_arm_robot_->setOperateMode(rokae::OperateMode::automatic, ec);
        if (ec) {
            RCLCPP_WARN(get_node()->get_logger(), "Failed to set left arm operate mode: %s", ec.message().c_str());
        }
        
        right_arm_robot_->setOperateMode(rokae::OperateMode::automatic, ec);
        if (ec) {
            RCLCPP_WARN(get_node()->get_logger(), "Failed to set right arm operate mode: %s", ec.message().c_str());
        }
        
        // 设置运动控制模式为实时模式
        left_arm_robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
        if (ec) {
            RCLCPP_WARN(get_node()->get_logger(), "Failed to set left arm motion control mode: %s", ec.message().c_str());
        }
        
        right_arm_robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
        if (ec) {
            RCLCPP_WARN(get_node()->get_logger(), "Failed to set right arm motion control mode: %s", ec.message().c_str());
        }
        
        // 获取实时控制器
        left_rt_controller_ = left_arm_robot_->getRtMotionController().lock();
        right_rt_controller_ = right_arm_robot_->getRtMotionController().lock();
        
        if (!left_rt_controller_ || !right_rt_controller_) {
            throw std::runtime_error("Failed to get realtime motion controllers");
        }
        
        // 初始化实时控制状态
        rt_control_running_ = false;
        state_receive_started_ = false;
        state_update_running_ = false;
        
        // 先启动状态接收
        try {
            // 启动左臂状态接收
            left_arm_robot_->startReceiveRobotState(std::chrono::milliseconds(1), 
                {rokae::RtSupportedFields::jointPos_m, rokae::RtSupportedFields::jointVel_m, rokae::RtSupportedFields::tau_m});
            
            // 启动右臂状态接收
            right_arm_robot_->startReceiveRobotState(std::chrono::milliseconds(1), 
                {rokae::RtSupportedFields::jointPos_m, rokae::RtSupportedFields::jointVel_m, rokae::RtSupportedFields::tau_m});
            
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
        
        RCLCPP_INFO(get_node()->get_logger(), "xCoreSDK initialized for left arm: %s, right arm: %s", 
                    left_arm_ip_.c_str(), right_arm_ip_.c_str());
        
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
        left_arm_rt_thread_ = std::thread(&RokaeHardware::leftArmControlLoop, this);
        right_arm_rt_thread_ = std::thread(&RokaeHardware::rightArmControlLoop, this);
        
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
    if (left_arm_rt_thread_.joinable()) {
        left_arm_rt_thread_.join();
    }
    if (right_arm_rt_thread_.joinable()) {
        right_arm_rt_thread_.join();
    }
    
    // 停止运动
    if (left_rt_controller_) {
        left_rt_controller_->stopMove();
    }
    if (right_rt_controller_) {
        right_rt_controller_->stopMove();
    }
    
    RCLCPP_INFO(get_node()->get_logger(), "Realtime control stopped");
}

void RokaeHardware::leftArmControlLoop()
{
    try {
        // 设置控制回调
        std::function<rokae::JointPosition()> callback = [this]() {
            std::vector<double> joint_positions(7);
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                for (size_t i = 0; i < 7; ++i) {
                    joint_positions[i] = left_arm_position_commands_[i];
                }
            }
            return rokae::JointPosition(joint_positions);
        };
        
        left_rt_controller_->setControlLoop(callback);
        left_rt_controller_->startMove(rokae::RtControllerMode::jointPosition);
        left_rt_controller_->startLoop(true);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Left arm control loop error: %s", e.what());
    }
}

void RokaeHardware::rightArmControlLoop()
{
    try {
        // 设置控制回调
        std::function<rokae::JointPosition()> callback = [this]() {
            std::vector<double> joint_positions(7);
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                for (size_t i = 0; i < 7; ++i) {
                    joint_positions[i] = right_arm_position_commands_[i];
                }
            }
            return rokae::JointPosition(joint_positions);
        };
        
        right_rt_controller_->setControlLoop(callback);
        right_rt_controller_->startMove(rokae::RtControllerMode::jointPosition);
        right_rt_controller_->startLoop(true);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Right arm control loop error: %s", e.what());
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
    while (state_update_running_ && left_arm_robot_ && right_arm_robot_) {
        try {
            // 更新左臂状态
            left_arm_robot_->updateRobotState(std::chrono::microseconds(100));
            
            // 更新右臂状态
            right_arm_robot_->updateRobotState(std::chrono::microseconds(100));
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_node()->get_logger(), "State update error: %s", e.what());
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

} // namespace rokae_ros2_control

PLUGINLIB_EXPORT_CLASS(rokae_ros2_control::RokaeHardware, hardware_interface::SystemInterface)
