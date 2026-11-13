//
// ControlInputHandler 实现。
//
// 速度命令先问 ArmsTargetManager::canAcceptSpeedCommand()，
// 被占用时整条丢弃并打调试日志；手部命令不参与这个判断。
//
#include "arms_target_manager/ControlInputHandler.h"

#include <algorithm>
#include <cmath>
#include "arms_target_manager/ArmsTargetManager.h"
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>

namespace arms_ros2_control::command
{
    ControlInputHandler::ControlInputHandler(
        rclcpp::Node::SharedPtr node,
        ArmsTargetManager* targetManager,
        const double linearScale,
        const double angularScale,
        const double controlInputRate,
        const std::vector<std::string>& handControllers)
        : node_(std::move(node))
          , target_manager_(targetManager)
          , linear_scale_(linearScale)
          , angular_scale_(angularScale)
          , control_input_rate_(std::max(1.0, controlInputRate))
          , hand_controllers_(handControllers)
          , hand_bus_(node_, handControllers)
    {
        left_twist_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(
            "left_target/twist", 10);
        right_twist_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(
            "right_target/twist", 10);

        RCLCPP_INFO(node_->get_logger(),
                    "🎮 ControlInputHandler: twist velocity scales linear=%.3f m/s, angular=%.3f rad/s "
                    "(control_input_rate=%.1f Hz)",
                    linear_scale_, angular_scale_, control_input_rate_);
        if (!hand_controllers_.empty()) {
            RCLCPP_INFO(node_->get_logger(), "🎮 Hand controllers configured: %zu controller(s)", hand_controllers_.size());
            for (size_t i = 0; i < hand_controllers_.size(); ++i) {
                RCLCPP_INFO(node_->get_logger(), "   [%zu] %s", i, hand_controllers_[i].c_str());
            }
        } else {
            RCLCPP_INFO(node_->get_logger(), "🎮 No hand controllers configured - hand_command will be ignored");
        }
    }

    void ControlInputHandler::processControlInput(const arms_ros2_control_msgs::msg::Inputs::ConstSharedPtr msg)
    {
        double x_input = msg->x;
        double y_input = msg->y;
        double z_input = msg->z;
        double roll_input = msg->roll;
        double pitch_input = msg->pitch;
        double yaw_input = msg->yaw;

        bool hasValidInput = std::abs(x_input) > 0.001 || std::abs(y_input) > 0.001 || std::abs(z_input) > 0.001 ||
            std::abs(roll_input) > 0.001 || std::abs(pitch_input) > 0.001 || std::abs(yaw_input) > 0.001;

        geometry_msgs::msg::Twist twist;
        twist.linear.x = x_input * linear_scale_;
        twist.linear.y = y_input * linear_scale_;
        twist.linear.z = z_input * linear_scale_;
        twist.angular.x = roll_input * angular_scale_;
        twist.angular.y = pitch_input * angular_scale_;
        twist.angular.z = yaw_input * angular_scale_;

        const bool is_left = msg->target != 2;

        // 目标通道同时只允许一个绝对控制来源。VR 或 RViz 拖动正占用该臂时，
        // 速度流会被拒绝，避免“位姿目标 + 速度积分”互相覆盖。
        if (target_manager_ && !target_manager_->canAcceptSpeedCommand(is_left))
        {
            RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
                                  "🎮 %s臂当前由其它方式控制，忽略 control_input 速度流",
                                  is_left ? "左" : "右");
            processHandCommand(msg->target, msg->hand_command);
            return;
        }

        if (is_left)
        {
            left_twist_publisher_->publish(twist);
        }
        else
        {
            right_twist_publisher_->publish(twist);
        }

        if (hasValidInput && target_manager_ &&
            target_manager_->getCurrentMode() != MarkerState::CONTINUOUS)
        {
            target_manager_->togglePublishMode();
            RCLCPP_INFO(node_->get_logger(), "🎮 ArmsTargetManager switched to CONTINUOUS mode for control input");
        }

        // marker 的显示由控制器回传的 current_target 驱动，这里不再自行积分一份。
        processHandCommand(msg->target, msg->hand_command);
    }

    std::string ControlInputHandler::resolveHandControllerName(int32_t target) const
    {
        return hand_bus_.controllerFor(target);
    }

    void ControlInputHandler::processHandCommand(int32_t target, float hand_command)
    {
        if (!std::isfinite(hand_command)) {
            return;
        }

        const std::string controller_name = resolveHandControllerName(target);
        if (controller_name.empty()) {
            RCLCPP_DEBUG(node_->get_logger(),
                         "🎮 Hand command ignored (target=%d, value=%.3f): no controller configured",
                         target, hand_command);
            return;
        }

        const double value = static_cast<double>(hand_command);
        // 仅精确 0/1 走开关；中间比例走 target_percent（扳机未按时 teleop 不发 hand_command）
        const bool use_switch = (hand_command == 0.0f || hand_command == 1.0f);
        const std::string arm_label = (hand_controllers_.size() == 1)
            ? controller_name
            : (target == 1 ? "left" : "right");

        if (use_switch)
        {
            hand_bus_.publishSwitch(controller_name, hand_command >= 0.5f);
            RCLCPP_DEBUG(node_->get_logger(), "🎮 Hand switch %s -> %d",
                         arm_label.c_str(), hand_command >= 0.5f ? 1 : 0);
        }
        else
        {
            const double percent = std::clamp(value, 0.0, 1.0);
            hand_bus_.publishPercent(controller_name, percent);
            RCLCPP_DEBUG(node_->get_logger(), "🎮 Hand percent %s -> %.3f",
                         arm_label.c_str(), percent);
        }
    }
} // namespace arms_ros2_control::command
