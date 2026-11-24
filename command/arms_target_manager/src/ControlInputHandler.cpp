//
// Created for Arms ROS2 Control - ControlInputHandler
//

#include "arms_target_manager/ControlInputHandler.h"

#include <algorithm>
#include <cmath>
#include "arms_target_manager/ArmsTargetManager.h"

namespace arms_ros2_control::command
{
    ControlInputHandler::ControlInputHandler(
        rclcpp::Node::SharedPtr node,
        ArmsTargetManager* targetManager,
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr left_twist_publisher,
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr right_twist_publisher,
        const double linearScale,
        const double angularScale,
        const double controlInputRate,
        const std::vector<std::string>& handControllers)
        : node_(std::move(node))
          , target_manager_(targetManager)
          , linear_scale_(linearScale)
          , angular_scale_(angularScale)
          , control_input_rate_(std::max(1.0, controlInputRate))
          , left_twist_publisher_(std::move(left_twist_publisher))
          , right_twist_publisher_(std::move(right_twist_publisher))
          , hand_controllers_(handControllers)
    {
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
        if (is_left)
        {
            left_twist_publisher_->publish(twist);
        }
        else
        {
            right_twist_publisher_->publish(twist);
        }

        if (hasValidInput && target_manager_)
        {
            if (target_manager_->getCurrentMode() != MarkerState::CONTINUOUS)
            {
                target_manager_->togglePublishMode();
                RCLCPP_INFO(node_->get_logger(), "🎮 ArmsTargetManager switched to CONTINUOUS mode for control input");
            }

            const double inv_rate = 1.0 / control_input_rate_;
            std::array<double, 3> positionDelta = {
                twist.linear.x * inv_rate,
                twist.linear.y * inv_rate,
                twist.linear.z * inv_rate
            };
            std::array<double, 3> rpyDelta = {
                twist.angular.x * inv_rate,
                twist.angular.y * inv_rate,
                twist.angular.z * inv_rate
            };

            std::string armType = is_left ? "left" : "right";
            target_manager_->updateMarkerPoseIncremental(armType, positionDelta, rpyDelta, false);

            RCLCPP_DEBUG(node_->get_logger(),
                         "🎮 Published %s twist vel pos[%.3f, %.3f, %.3f] ang[%.3f, %.3f, %.3f]",
                         armType.c_str(),
                         twist.linear.x, twist.linear.y, twist.linear.z,
                         twist.angular.x, twist.angular.y, twist.angular.z);
        }

        processHandCommand(msg->target, msg->hand_command);
    }

    std::string ControlInputHandler::resolveHandControllerName(int32_t target) const
    {
        if (hand_controllers_.empty()) {
            return "";
        }

        size_t controller_index = 0;
        if (hand_controllers_.size() > 1) {
            controller_index = static_cast<size_t>(target - 1);
            if (controller_index >= hand_controllers_.size()) {
                return "";
            }
        }

        return hand_controllers_[controller_index];
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
        const bool use_switch = (hand_command == 0.0f || hand_command == 1.0f);
        const std::string arm_label = (hand_controllers_.size() == 1)
            ? controller_name
            : (target == 1 ? "left" : "right");

        if (use_switch)
        {
            auto& pub = hand_switch_publishers_[controller_name];
            if (!pub)
            {
                pub = node_->create_publisher<std_msgs::msg::Int32>(
                    controller_name + "/target_command", 10);
            }
            std_msgs::msg::Int32 cmd;
            cmd.data = (hand_command >= 0.5f) ? 1 : 0;
            pub->publish(cmd);
            RCLCPP_DEBUG(node_->get_logger(), "🎮 Hand switch %s -> %d", arm_label.c_str(), cmd.data);
        }
        else
        {
            auto& pub = hand_percent_publishers_[controller_name];
            if (!pub)
            {
                pub = node_->create_publisher<std_msgs::msg::Float64>(
                    controller_name + "/target_percent", 10);
            }
            std_msgs::msg::Float64 pct;
            pct.data = std::clamp(value, 0.0, 1.0);
            pub->publish(pct);
            RCLCPP_DEBUG(node_->get_logger(), "🎮 Hand percent %s -> %.3f", arm_label.c_str(), pct.data);
        }
    }
} // namespace arms_ros2_control::command
