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

#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

// xCoreSDK includes
#include "rokae/robot.h"
#include "rokae/motion_control_rt.h"
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <mutex>

namespace rokae_ros2_control {

/**
 * @brief Rokae机器人的ROS2 Control硬件接口
 * 
 * 该类实现了hardware_interface::SystemInterface，通过xCoreSDK与Rokae机械臂通信
 * 重构后的设计：一个hardware实例只控制一个机械臂（7自由度）
 * 参考Dobot的设计思路，简化架构，提高模块化程度
 */
class RokaeHardware : public hardware_interface::SystemInterface {
public:
    /**
     * @brief 初始化硬件接口
     * @param info 从URDF/配置文件中读取的硬件信息
     * @return 初始化结果
     */
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    
    /**
     * @brief 激活硬件接口（连接机器人、上电、初始化实时控制）
     * @param previous_state 前一个生命周期状态
     * @return 激活结果
     */
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    
    /**
     * @brief 停用硬件接口（停止实时控制、下电、断开连接）
     * @param previous_state 前一个生命周期状态
     * @return 停用结果
     */
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    /**
     * @brief 导出状态接口（位置、速度、力矩）
     * @return 状态接口列表
     */
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    /**
     * @brief 导出命令接口（位置命令）
     * @return 命令接口列表
     */
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    /**
     * @brief 读取机器人状态
     * @param time 当前时间
     * @param period 距离上次读取的时间间隔
     * @return 读取结果
     */
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    /**
     * @brief 写入控制命令到机器人
     * @param time 当前时间
     * @param period 距离上次写入的时间间隔
     * @return 写入结果
     */
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    // 关节数据存储（动态大小，根据配置）
    std::vector<double> joint_positions_;          // 关节位置（弧度）
    std::vector<double> joint_velocities_;         // 关节速度（弧度/秒）
    std::vector<double> joint_efforts_;            // 关节力矩（N·m）
    std::vector<double> joint_position_commands_;  // 关节位置命令（弧度）
    std::vector<std::string> joint_names_;         // 关节名称列表

    // 配置参数
    std::string arm_ip_;           // 机械臂IP地址
    std::string local_ip_;         // 本地IP地址

    // xCoreSDK相关
    std::unique_ptr<rokae::xMateErProRobot> arm_robot_;
    std::shared_ptr<rokae::RtMotionControlCobot<7>> rt_controller_;
    
    // 实时控制线程
    std::thread rt_control_thread_;
    std::atomic<bool> rt_control_running_;
    std::mutex data_mutex_;
    
    // 状态接收相关
    std::atomic<bool> state_receive_started_;
    std::thread state_update_thread_;
    std::atomic<bool> state_update_running_;

    // xCoreSDK相关方法
    void initializeXCoreSDK();
    void startRealtimeControl();
    void stopRealtimeControl();
    void controlLoop();
    
    // 状态更新相关方法
    void startStateUpdate();
    void stopStateUpdate();
    void stateUpdateLoop();
};

} // namespace rokae_ros2_control

