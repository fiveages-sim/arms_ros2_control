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
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>

namespace arms_ros2_control::command
{
    // 前向声明
    class ArmsTargetManager;

    /**
     * ControlInputHandler - 控制输入处理器
     *
     * 接收 control_input (Inputs)，适配为 left/right_target/twist 速度命令；
     * hand_command 仍转发到夹爪控制器。Marker 仅做可视化，不再发绝对 Pose。
     *
     * linear_scale / angular_scale 单位为最大速度：m/s、rad/s（满杆）。
     * 与旧「每消息位移步进」对齐：scale_vel ≈ scale_old * control_input_rate。
     */
    class ControlInputHandler
    {
    public:
        ControlInputHandler(
            rclcpp::Node::SharedPtr node,
            ArmsTargetManager* targetManager,
            double linearScale = 0.25,
            double angularScale = 2.5,
            double controlInputRate = 50.0,
            const std::vector<std::string>& handControllers = {});

        ~ControlInputHandler() = default;

        void processControlInput(arms_ros2_control_msgs::msg::Inputs::ConstSharedPtr msg);

    private:
        void processHandCommand(int32_t target, float hand_command);
        std::string resolveHandControllerName(int32_t target) const;

        rclcpp::Node::SharedPtr node_;
        ArmsTargetManager* target_manager_;

        double linear_scale_;
        double angular_scale_;
        double control_input_rate_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr left_twist_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr right_twist_publisher_;

        std::vector<std::string> hand_controllers_;
        std::map<std::string, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> hand_switch_publishers_;
        std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> hand_percent_publishers_;
    };
} // namespace arms_ros2_control::command
