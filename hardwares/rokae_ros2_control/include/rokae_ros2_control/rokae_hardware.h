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
#include <rclcpp/subscription.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp_lifecycle/state.hpp>

// xCoreSDK includes
#include "rokae/robot.h"
#include "rokae/motion_control_rt.h"
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>

namespace rokae_ros2_control {
    class RokaeHardware : public hardware_interface::SystemInterface {
    public:
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:

        // 关节数据存储 - 支持所有关节的所有接口
        // 底盘关节 (2个关节)
        std::vector<double> chassis_positions_;
        std::vector<double> chassis_velocities_;
        std::vector<double> chassis_efforts_;
        std::vector<double> chassis_velocity_commands_;

        // 身体关节 (4个关节)
        std::vector<double> body_positions_;
        std::vector<double> body_velocities_;
        std::vector<double> body_efforts_;
        std::vector<double> body_position_commands_;

        // 头部关节 (2个关节)
        std::vector<double> head_positions_;
        std::vector<double> head_velocities_;
        std::vector<double> head_efforts_;
        std::vector<double> head_position_commands_;

        // 左臂关节 (7个关节) - 真实数据
        std::vector<double> left_arm_positions_;
        std::vector<double> left_arm_velocities_;
        std::vector<double> left_arm_efforts_;
        std::vector<double> left_arm_position_commands_;

        // 右臂关节 (7个关节) - 真实数据
        std::vector<double> right_arm_positions_;
        std::vector<double> right_arm_velocities_;
        std::vector<double> right_arm_efforts_;
        std::vector<double> right_arm_position_commands_;

        // 配置参数
        double update_rate_;

        // xCoreSDK相关
        std::unique_ptr<rokae::xMateErProRobot> left_arm_robot_;
        std::unique_ptr<rokae::xMateErProRobot> right_arm_robot_;
        std::shared_ptr<rokae::RtMotionControlCobot<7>> left_rt_controller_;
        std::shared_ptr<rokae::RtMotionControlCobot<7>> right_rt_controller_;
        
        // 实时控制线程
        std::thread left_arm_rt_thread_;
        std::thread right_arm_rt_thread_;
        std::atomic<bool> rt_control_running_;
        std::mutex data_mutex_;
        
        // 状态接收相关
        std::atomic<bool> state_receive_started_;
        std::thread state_update_thread_;
        std::atomic<bool> state_update_running_;
        
        // xCoreSDK配置参数
        std::string left_arm_ip_;
        std::string right_arm_ip_;
        std::string local_ip_;


        // 更新关节状态
        void updateJointStates();
        
        // xCoreSDK相关方法
        void initializeXCoreSDK();
        void startRealtimeControl();
        void stopRealtimeControl();
        void leftArmControlLoop();
        void rightArmControlLoop();
        
        // 状态更新相关方法
        void startStateUpdate();
        void stopStateUpdate();
        void stateUpdateLoop();
    };
} // namespace rokae_ros2_control
