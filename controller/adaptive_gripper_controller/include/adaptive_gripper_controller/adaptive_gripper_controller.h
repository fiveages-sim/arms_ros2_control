// Copyright 2024, Your Organization
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


// C++ standard
#pragma once
#include <memory>
#include <string>

#include "arms_ros2_control_msgs/msg/gripper.hpp"
#include "std_msgs/msg/string.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

namespace adaptive_gripper_controller
{
    /**
     * \brief 简单的夹爪控制器，实现基本的位置读取和输出功能
     */
    class AdaptiveGripperController : public controller_interface::ControllerInterface
    {
    public:
        AdaptiveGripperController();

        /**
         * @brief 命令接口配置
         */
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        /**
         * @brief 状态接口配置
         */
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        /**
         * @brief 控制器更新函数
         */
        controller_interface::return_type update(
            const rclcpp::Time& time, const rclcpp::Duration& period) override;

        /**
         * @brief 控制器初始化
         */
        controller_interface::CallbackReturn on_init() override;

        /**
         * @brief 控制器配置
         */
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State& previous_state) override;

        /**
         * @brief 控制器激活
         */
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& previous_state) override;

        /**
         * @brief 控制器停用
         */
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State& previous_state) override;

    private:
        // 夹爪关节名称
        std::string joint_name_ = "gripper_joint";

        // 目标位置
        double target_position_ = 0.0;

        // 夹爪状态
        int32_t gripper_target_ = 0;
        int32_t gripper_direction_ = 0;

        // 关节限制
        double joint_upper_limit_ = 0.1;
        double joint_lower_limit_ = 0.0;
        bool limits_initialized_ = false;

        // 配置的初始值（从 robot_description 读取，默认为 0.0）
        double config_initial_position_ = 0.0;
        
        // 夹爪的关闭和打开位置（初始化时计算）
        double closed_position_ = 0.0;
        double open_position_ = 0.0;

        // 是否使用力反馈接口
        bool use_effort_interface_ = true;
        
        // 力反馈阈值
        double force_threshold_ = 0.1;

        // 力反馈触发后的移动距离比例 (0.0-1.0)
        double force_feedback_ratio_ = 0.5;

        // 夹取任务状态跟踪
        bool force_threshold_triggered_ = false;
        
        // 是否有 effort 接口（运行时自动检测）
        bool has_effort_interface_ = false;
        
        // 需要的状态接口类型列表（从 hardware 查询）
        std::vector<std::string> available_state_interface_types_;

        // 硬件接口结构体
        struct GripperInterfaces
        {
            // 命令接口
            std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
            position_command_interface_;

            // 状态接口
            std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
            position_state_interface_;
            std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
            effort_state_interface_;

            void clear()
            {
                position_command_interface_ = std::nullopt;
                position_state_interface_ = std::nullopt;
                effort_state_interface_ = std::nullopt;
            }
        };

        GripperInterfaces gripper_interfaces_;

        // 手臂标识 (1=左臂, 2=右臂)
        int32_t arm_id_ = 1;  // 默认为左臂

        // ROS订阅器
        rclcpp::Subscription<arms_ros2_control_msgs::msg::Gripper>::SharedPtr gripper_subscription_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;

                 // 处理夹爪命令
         void process_gripper_command();

        // 解析robot description获取关节限制
        void parse_joint_limits(const std::string& robot_description);

        // 解析robot description获取初始值，并计算夹爪位置
        void parse_initial_value(const std::string& robot_description);
    };
} // namespace adaptive_gripper_controller
