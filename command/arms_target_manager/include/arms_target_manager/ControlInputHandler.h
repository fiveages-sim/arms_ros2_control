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
#include <std_msgs/msg/float64.hpp>

namespace arms_ros2_control::command
{
    // 前向声明
    class ArmsTargetManager;

    /**
     * ControlInputHandler - 控制输入处理器Wrapper
     *
     * 接收 control_input，累积位姿增量更新 marker，并将 hand_command 转发到夹爪控制器。
     */
    class ControlInputHandler
    {
    public:
        ControlInputHandler(
            rclcpp::Node::SharedPtr node,
            ArmsTargetManager* targetManager,
            double linearScale = 0.005,
            double angularScale = 0.05,
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

        std::vector<std::string> hand_controllers_;
        std::map<std::string, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> hand_switch_publishers_;
        std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> hand_percent_publishers_;
    };
} // namespace arms_ros2_control::command
