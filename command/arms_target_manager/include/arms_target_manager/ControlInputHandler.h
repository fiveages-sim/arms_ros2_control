//
// Created for Arms ROS2 Control - ControlInputHandler
//
#pragma once


#include <memory>
#include <vector>
#include <string>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <arms_ros2_control_msgs/msg/inputs.hpp>
#include <std_msgs/msg/int32.hpp>

namespace arms_ros2_control::command
{
    // 前向声明
    class ArmsTargetManager;

    /**
     * ControlInputHandler - 控制输入处理器Wrapper
     * 
     * 接收control input数据，累积位置和旋转变化，通过ArmsTargetManager更新marker位置
     * 支持线性缩放和角度缩放，死区处理在输入源层面完成
     * 处理hand_command并映射到对应的hand/gripper控制器
     */
    class ControlInputHandler
    {
    public:
        /**
         * 构造函数
         * @param node ROS节点指针
         * @param targetManager ArmsTargetManager指针
         * @param linearScale 线性移动缩放因子，默认为0.005
         * @param angularScale 角度移动缩放因子，默认为0.05
         * @param handControllers 手部/夹爪控制器名称列表（用于映射hand_command）
         */
        ControlInputHandler(
            rclcpp::Node::SharedPtr node,
            ArmsTargetManager* targetManager,
            double linearScale = 0.005,
            double angularScale = 0.05,
            const std::vector<std::string>& handControllers = {});

        ~ControlInputHandler() = default;

        /**
         * 处理控制输入并更新ArmsTargetManager
         * @param msg 控制输入消息
         */
        void processControlInput(arms_ros2_control_msgs::msg::Inputs::ConstSharedPtr msg);

    private:
        /**
         * 处理hand_command并发布到对应的控制器
         * @param target 目标手臂 (1=left, 2=right)
         * @param handCommand 手部命令 (0=close, 1=open, -1=no command)
         */
        void processHandCommand(int32_t target, int32_t handCommand);

        // ROS节点和TargetManager
        rclcpp::Node::SharedPtr node_;
        ArmsTargetManager* target_manager_;

        // 控制参数
        double linear_scale_;
        double angular_scale_;

        // Hand controllers mapping
        std::vector<std::string> hand_controllers_;
        // Publishers for hand controllers (created on demand)
        std::map<std::string, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> hand_command_publishers_;
    };
} // namespace arms_ros2_control::command
