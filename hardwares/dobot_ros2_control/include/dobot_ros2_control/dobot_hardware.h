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
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include "dobot_ros2_control/command.h"

#include <memory>
#include <string>
#include <vector>
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
    double last_gripper_command_;                  // 上次发送的夹爪命令（用于检测变化）
    int gripper_read_counter_;                     // 夹爪读取计数器（用于降低读取频率）
    int gripper_read_frequency_divider_;           // 夹爪读取频率除数（每N次read循环读取一次，默认4）
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
    std::string wrench_frame_id_;  // 六维力传感器数据的frame_id

    // Dobot底层通信接口
    std::shared_ptr<CRCommanderRos2> commander_;
    
    // 控制频率统计
    int write_count_;
    std::chrono::steady_clock::time_point last_write_stat_time_;
    
    // 六维力传感器自动归零
    bool enable_wrench_auto_zero_;          // 是否启用自动归零
    bool wrench_zero_initialized_;           // 是否已初始化偏移量
    double ft_sensor_offset_force_x_;        // 力 X 偏移量 (N)
    double ft_sensor_offset_force_y_;        // 力 Y 偏移量 (N)
    double ft_sensor_offset_force_z_;        // 力 Z 偏移量 (N)
    double ft_sensor_offset_torque_x_;       // 力矩 X 偏移量 (N·m)
    double ft_sensor_offset_torque_y_;       // 力矩 Y 偏移量 (N·m)
    double ft_sensor_offset_torque_z_;       // 力矩 Z 偏移量 (N·m)
    int wrench_zero_samples_;                // 归零采样次数（用于平均）
    int wrench_zero_sample_count_;           // 当前采样计数
    
    // 六维力传感器数据（用于 hardware interface 导出）
    double ft_sensor_force_x_;   // 力 X (N)
    double ft_sensor_force_y_;   // 力 Y (N)
    double ft_sensor_force_z_;   // 力 Z (N)
    double ft_sensor_torque_x_;  // 力矩 X (N·m)
    double ft_sensor_torque_y_;  // 力矩 Y (N·m)
    double ft_sensor_torque_z_;  // 力矩 Z (N·m)
    
    // 夹爪控制辅助函数
    bool initializeModbus();
    bool controlGripper(double position);  // position: 0.0(闭合) - 1.0(打开)
    bool readGripperState(double &position);
    
    // 传感器辅助函数
    bool findSensorByName(const std::string& sensor_name, hardware_interface::ComponentInfo& sensor_info);
};

} // namespace dobot_ros2_control

