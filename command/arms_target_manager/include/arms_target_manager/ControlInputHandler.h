//
// ControlInputHandler - 摇杆 / 键盘输入适配。
//
// 输入：control_input（arms_ros2_control_msgs/Inputs，速度与手部命令）
// 输出：left_target/twist、right_target/twist；手部命令交给 HandCommandBus
//
// 速度流不参与目标所有权，但会被 TargetRouter 的屏蔽表拦住：
// VR 或手动拖动正在控制某条手臂时，这条速度命令直接丢弃（手部命令照常转发）。
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
#include "arms_target_manager/core/HandCommandBus.h"

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
     */
    class ControlInputHandler
    {
    public:
        ControlInputHandler(
            rclcpp::Node::SharedPtr node,
            ArmsTargetManager* targetManager,
            double linearScale = 0.1,
            double angularScale = 0.25,
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
        HandCommandBus hand_bus_;
    };
} // namespace arms_ros2_control::command
