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
#include <thread>

namespace dobot_dual_ros2_control {

/**
 * @brief Dobot CR系列双机械臂的ROS2 Control硬件接口
 *
 * 该类实现了hardware_interface::SystemInterface，同时管理两台独立的Dobot CR机器人
 * 通过两个独立的TCP连接分别控制左臂和右臂
 */
class DobotDualHardware : public hardware_interface::SystemInterface {
public:
    /**
     * @brief 初始化硬件接口
     * @param info 从URDF/配置文件中读取的硬件信息
     * @return 初始化结果
     */
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    /**
     * @brief 激活硬件接口（建立两个TCP连接、设置全局速度、初始化关节位置）
     * @param previous_state 前一个生命周期状态
     * @return 激活结果
     */
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    /**
     * @brief 停用硬件接口（断开两个TCP连接）
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
     * @brief 读取两台机器人的状态（从TCP实时数据流）
     * @param time 当前时间
     * @param period 距离上次读取的时间间隔
     * @return 读取结果
     */
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    /**
     * @brief 写入控制命令到两台机器人
     * @param time 当前时间
     * @param period 距离上次写入的时间间隔
     * @return 写入结果
     */
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    // 关节数据存储（12个关节: 0-5左臂, 6-11右臂）
    std::vector<double> joint_positions_;          // 关节位置（弧度）
    std::vector<double> joint_velocities_;         // 关节速度（弧度/秒）
    std::vector<double> joint_efforts_;            // 关节力矩（N·m）
    std::vector<double> joint_position_commands_;  // 关节位置命令（弧度）

    // 夹爪数据存储
    double left_gripper_position_;                 // 左夹爪位置（0.0=闭合, 1.0=打开）
    double right_gripper_position_;                // 右夹爪位置
    double left_gripper_position_command_;         // 左夹爪位置命令
    double right_gripper_position_command_;        // 右夹爪位置命令
    bool has_left_gripper_;                        // 是否配置了左夹爪
    bool has_right_gripper_;                       // 是否配置了右夹爪
    std::string left_gripper_joint_name_;          // 左夹爪关节名称
    std::string right_gripper_joint_name_;         // 右夹爪关节名称

    // 配置参数 - 左臂
    std::string left_robot_ip_;
    double left_servo_time_;
    double left_aheadtime_;
    double left_gain_;
    int left_speed_factor_;

    // 配置参数 - 右臂
    std::string right_robot_ip_;
    double right_servo_time_;
    double right_aheadtime_;
    double right_gain_;
    int right_speed_factor_;

    // 通用配置
    bool verbose_;

    // Dobot底层通信接口（两个独立的Commander）
    std::shared_ptr<CRCommanderRos2> left_commander_;
    std::shared_ptr<CRCommanderRos2> right_commander_;

    // 数据同步
    std::mutex data_mutex_;
    std::mutex left_gripper_mutex_;
    std::mutex right_gripper_mutex_;

    // 控制频率统计
    int write_count_;
    std::chrono::steady_clock::time_point last_write_stat_time_;

    // 夹爪控制线程
    std::thread left_gripper_control_thread_;
    std::thread right_gripper_control_thread_;
    std::atomic<bool> left_gripper_thread_running_;
    std::atomic<bool> right_gripper_thread_running_;

    // 夹爪控制辅助函数
    bool initializeLeftModbus();
    bool initializeRightModbus();
    bool controlLeftGripper(double position);
    bool controlRightGripper(double position);
    bool readLeftGripperState(double &position);
    bool readRightGripperState(double &position);
    void leftGripperControlLoop();
    void rightGripperControlLoop();
};

} // namespace dobot_dual_ros2_control
