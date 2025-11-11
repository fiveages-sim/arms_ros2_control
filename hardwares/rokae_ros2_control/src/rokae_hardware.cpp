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
#include "rokae_ros2_control/gripper_contro.h"
#include <pluginlib/class_list_macros.hpp>
#include <sstream>
#include <iomanip>
#include <cmath>


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
    
    // 初始化夹爪参数
    gripper_position_ = -1.0;
    gripper_position_command_ = -1.0;
    last_gripper_command_ = -1.0;
    // gripper_read_counter_ = 0;
    has_gripper_ = false;
    gripper_joint_index_ = -1;
    gripper_initilized_ = false;
    gripper_stopped_ = true;
    contains_gripper();

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
        if(has_gripper_)
        {
            while (true) {
            //②获取初始化状态
                uint8_t init_status = 0;
                bool init_finished = DHGripGetInitStatus(*arm_robot_, init_status, get_node());
                RCLCPP_ERROR(get_node()->get_logger(), " initialize arm : %d", init_status);
                if (init_finished)
                {
                    RCLCPP_ERROR(get_node()->get_logger(), "gripper initialize success");
                    break;
                }
                // 读取一次数据初始化
                // std::this_thread::sleep_for(std::chrono::milliseconds(500));//5s用于检查初始化，dh电爪初始化需要7s左右
                std::vector<int> robot_stats;
                DHGripGetStatus(*arm_robot_, robot_stats, get_node());
                int pos_now_get = robot_stats[1] >> 8;
                gripper_position_ = 1.0 -  round(pos_now_get / 255.0);
                gripper_stopped_ = ((robot_stats[0] >> 3) & 1) == 0 ? true: false;
            } 
        }
        gripper_initilized_ = true;
        
        RCLCPP_INFO(get_node()->get_logger(), "✅ Arm powered on");
        RCLCPP_INFO(get_node()->get_logger(), "✅ Gripper powered on");
        
        // 从当前机器人位置初始化命令位置，避免突然跳跃
        auto current_pos = arm_robot_->jointPos(ec);
        if (!ec) {
            // std::lock_guard<std::mutex> lock(data_mutex_);
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
        // 初始化夹爪
        if(has_gripper_)
        {
            DHGripInit(*arm_robot_, get_node());
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));//5s用于检查初始化，dh电爪初始化需要7s左右
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
    int joint_name_sz = joint_names_.size();
    if (has_gripper_) 
    {
        joint_name_sz = joint_names_.size() - 1;
    }
    RCLCPP_INFO(get_node()->get_logger(), 
                   "Exported %d \n", 
                   joint_name_sz);
    for (size_t i = 0; i < joint_name_sz; ++i) {
        state_interfaces.emplace_back(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_positions_[i]);
        
        state_interfaces.emplace_back(
            joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]);
        
        state_interfaces.emplace_back(
            joint_names_[i], hardware_interface::HW_IF_EFFORT, &joint_efforts_[i]);
    }
    
    // 如果配置了夹爪，导出夹爪状态接口（只有位置）
    if (has_gripper_) {
        state_interfaces.emplace_back(
            gripper_joint_name_, hardware_interface::HW_IF_POSITION, &gripper_position_);
        
        state_interfaces.emplace_back(
            gripper_joint_name_, hardware_interface::HW_IF_VELOCITY, &gripper_velocity_);
        
        state_interfaces.emplace_back(
            gripper_joint_name_, hardware_interface::HW_IF_EFFORT, &gripper_effort_);
        
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

std::vector<hardware_interface::CommandInterface> RokaeHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    int joint_name_sz = joint_names_.size();
    if (has_gripper_) 
    {
        joint_name_sz = joint_names_.size() - 1;
    }
    // 为所有关节导出命令接口：位置命令
    for (size_t i = 0; i < joint_name_sz; ++i) {
        command_interfaces.emplace_back(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_position_commands_[i]);
        // command_interfaces.emplace_back(
        //     joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joint_velocities_commands_[i]);
        // command_interfaces.emplace_back(
        //     joint_names_[i], hardware_interface::HW_IF_EFFORT, &joint_efforts_commands_[i]);
    }
    
    // 如果配置了夹爪，导出夹爪命令接口
    if (has_gripper_) {
        command_interfaces.emplace_back(
                gripper_joint_name_, hardware_interface::HW_IF_POSITION, &gripper_position_command_);
        
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

hardware_interface::return_type RokaeHardware::read(
    const rclcpp::Time& /*time*/, 
    const rclcpp::Duration& /*period*/)
{
    // 从缓存的状态数据获取关节信息
    if (arm_robot_ && state_receive_started_) {
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            joint_positions_[i] = joint_positions_array_[i];
            // joint_velocities_[i] = joint_velocities_array_[i];
            // joint_efforts_[i] = joint_efforts_array_[i];
        }
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
                // std::lock_guard<std::mutex> lock(data_mutex_);
                // arm_robot_->updateRobotState(std::chrono::milliseconds(1));
                arm_robot_->getStateData(rokae::RtSupportedFields::jointPos_m, joint_positions_array_);
                // arm_robot_->getStateData(rokae::RtSupportedFields::jointVel_m, joint_velocities_array_);
                // arm_robot_->getStateData(rokae::RtSupportedFields::tau_m, joint_efforts_array_);
                for (size_t i = 0; i < joint_names_.size(); ++i) {
                    joint_positions[i] = joint_position_commands_[i];
                }
            }
            return rokae::JointPosition(joint_positions);
        };
        
        rt_controller_->setControlLoop(callback, 0, true);
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
    while (true) {
        if(state_update_running_ && arm_robot_ && gripper_initilized_)
        {
            try {
                // 更新机器人状态
                // 只有夹爪动的时候读取数据
                if(!gripper_stopped_)
                {
                    std::vector<int> robot_stats;
                    DHGripGetStatus(*arm_robot_, robot_stats, get_node());
                    int pos_now_get = robot_stats[1] >> 8;
                    gripper_position_ = 1.0 -  round(pos_now_get / 255.0);
                    gripper_stopped_ = ((robot_stats[0] >> 3) & 1) == 0 ? true: false;
                    RCLCPP_INFO(get_node()->get_logger(), "Gripper position read %d, %d, %f", (robot_stats[0] >> 3) & 1, pos_now_get, gripper_position_);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 2HZ
                }
                // 只有新指令来了 且新指令会导致夹爪位置变化才会写
                if(last_gripper_command_ != gripper_position_command_)
                {
                    // 写指令
                    {
                    
                        RCLCPP_INFO(get_node()->get_logger(), "Gripper position %f, %f, %f", last_gripper_command_, gripper_position_command_, gripper_position_);
                        std::vector<int> robot_stats;
                        int pos_set = 255 - static_cast<int>(gripper_position_command_ * 255);    
                        int trq_set = 255;
                        int vel_set = 255;
                        RCLCPP_INFO(get_node()->get_logger(), "Send new gripper action command %d", pos_set);
                        DHGripMove(*arm_robot_, trq_set, vel_set, pos_set, get_node());
                        DHGripMove(*arm_robot_, trq_set, vel_set, pos_set, get_node());
                        gripper_stopped_ = false;
                        last_gripper_command_ = gripper_position_command_;
                    }

                }
                
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
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // 2HZ
    }
}

void RokaeHardware::contains_gripper()
{
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
            // joint_names_.push_back(joint.name);
        }
        joint_index++;
    }
}

} // namespace rokae_ros2_control

// 导出插件
PLUGINLIB_EXPORT_CLASS(rokae_ros2_control::RokaeHardware, hardware_interface::SystemInterface)

