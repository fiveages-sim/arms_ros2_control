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

#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "dobot_ros2_control/command.h"

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>

namespace dobot_ros2_control {

/**
 * @brief Dobot CR系列机器人的ROS2 Control硬件接口
 * 
 * 该类实现了hardware_interface::SystemInterface，直接通过TCP与Dobot CR系列机器人通信
 * 当前版本实现了关节状态读取功能
 */
class DobotHardware : public hardware_interface::SystemInterface {
public:
    /**
     * @brief 初始化硬件接口
     * @param params 硬件组件接口参数（包含硬件信息、日志记录器等）
     * @return 初始化结果
     */
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;
    
    /**
     * @brief 激活硬件接口（建立TCP连接、设置全局速度、初始化关节位置）
     * @param previous_state 前一个生命周期状态
     * @return 激活结果
     */
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    
    /**
     * @brief 停用硬件接口（断开TCP连接）
     * @param previous_state 前一个生命周期状态
     * @return 停用结果
     */
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    /**
     * @brief 导出状态接口（位置、速度、力矩）
     * @return 状态接口列表（智能指针）
     */
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;

    /**
     * @brief 导出命令接口（位置命令）
     * @return 命令接口列表（智能指针）
     */
    std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;

    /**
     * @brief 读取机器人状态（从TCP实时数据流）
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
    
    // 夹爪数据存储
    double gripper_position_;                      // 夹爪位置（0.0=闭合, 1.0=打开）
    double gripper_position_command_;              // 夹爪位置命令（0.0-1.0）
    bool has_gripper_;                             // 是否配置了夹爪
    std::string gripper_joint_name_;               // 夹爪关节名称
    int gripper_joint_index_;                      // 夹爪在关节列表中的索引（-1表示无夹爪）

    // 配置参数
    std::string robot_ip_;    // 机器人IP地址
    double servo_time_;       // ServoJ执行时间（秒）
    double aheadtime_;        // 提前量（aheadtime）
    double gain_;             // 比例增益（gain）
    int speed_factor_;        // 全局速度比例（1-100）
    bool verbose_;            // 是否显示详细日志（包括频率统计）

    // Dobot底层通信接口
    std::shared_ptr<CRCommanderRos2> commander_;
    
    // 控制频率统计
    int write_count_;
    std::chrono::steady_clock::time_point last_write_stat_time_;
    
    // 夹爪控制线程
    std::thread gripper_control_thread_;
    std::atomic<bool> gripper_thread_running_;
    
    // 夹爪控制辅助函数
    bool initializeModbus();
    bool controlGripper(double position);  // position: 0.0(闭合) - 1.0(打开)
    bool readGripperState(double &position);
    void gripperControlLoop();  // 夹爪控制线程函数
};

} // namespace dobot_ros2_control

