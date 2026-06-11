//
// Created for Arms ROS2 Control - ControlInputHandler
//

#include "arms_target_manager/ControlInputHandler.h"

#include <algorithm>
#include <cmath>
#include "arms_target_manager/ArmsTargetManager.h"
#include "arms_target_manager/MarkerFactory.h"
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>

namespace arms_ros2_control::command
{
    ControlInputHandler::ControlInputHandler(
        rclcpp::Node::SharedPtr node,
        ArmsTargetManager* targetManager,
        const double linearScale,
        const double angularScale,
        const std::vector<std::string>& handControllers)
        : node_(std::move(node))
          , target_manager_(targetManager)
          , linear_scale_(linearScale)
          , angular_scale_(angularScale)
          , hand_controllers_(handControllers)
    {
        RCLCPP_INFO(node_->get_logger(), "🎮 ControlInputHandler created with scales: linear=%.3f, angular=%.3f",
                    linear_scale_, angular_scale_);
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

        if (hasValidInput && target_manager_)
        {
            if (target_manager_->getCurrentMode() != MarkerState::CONTINUOUS)
            {
                target_manager_->togglePublishMode();
                RCLCPP_INFO(node_->get_logger(), "🎮 ArmsTargetManager switched to CONTINUOUS mode for control input");
            }

            std::array<double, 3> positionDelta = {
                x_input * linear_scale_,
                y_input * linear_scale_,
                z_input * linear_scale_
            };

            std::array<double, 3> rpyDelta = {
                roll_input * angular_scale_,
                pitch_input * angular_scale_,
                yaw_input * angular_scale_
            };

            std::string armType = msg->target == 1 ? "left" : "right";
            target_manager_->updateMarkerPoseIncremental(armType, positionDelta, rpyDelta);

            RCLCPP_DEBUG(node_->get_logger(),
                         "🎮 Updated %s arm pose incrementally: pos[%.3f, %.3f, %.3f], rpy[%.3f, %.3f, %.3f]",
                         armType.c_str(),
                         positionDelta[0], positionDelta[1], positionDelta[2],
                         rpyDelta[0], rpyDelta[1], rpyDelta[2]);
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
        // 仅精确 0/1 走开关；中间比例走 target_percent（扳机未按时 teleop 不发 hand_command）
        const bool use_switch = (hand_command == 0.0f || hand_command == 1.0f);
        const std::string arm_label = (hand_controllers_.size() == 1)
            ? controller_name
            : ((target == 1) ? "LEFT" : "RIGHT") + std::string(" (") + controller_name + ")";

        if (use_switch) {
            if (hand_switch_publishers_.find(controller_name) == hand_switch_publishers_.end()) {
                const std::string topic_name = "/" + controller_name + "/target_command";
                hand_switch_publishers_[controller_name] =
                    node_->create_publisher<std_msgs::msg::Int32>(topic_name, 10);
                RCLCPP_INFO(node_->get_logger(), "🎮 Created hand switch publisher: %s", topic_name.c_str());
            }

            auto switch_msg = std_msgs::msg::Int32();
            switch_msg.data = static_cast<int32_t>(value);
            hand_switch_publishers_[controller_name]->publish(switch_msg);

            RCLCPP_INFO(node_->get_logger(), "🎮 Hand switch sent to %s: %s",
                        arm_label.c_str(), (switch_msg.data == 1) ? "OPEN" : "CLOSE");
            return;
        }

        const double ratio = std::clamp(value, 0.0, 1.0);
        if (hand_percent_publishers_.find(controller_name) == hand_percent_publishers_.end()) {
            const std::string topic_name = "/" + controller_name + "/target_percent";
            hand_percent_publishers_[controller_name] =
                node_->create_publisher<std_msgs::msg::Float64>(topic_name, 10);
            RCLCPP_INFO(node_->get_logger(), "🎮 Created hand percent publisher: %s", topic_name.c_str());
        }

        auto percent_msg = std_msgs::msg::Float64();
        percent_msg.data = ratio;
        hand_percent_publishers_[controller_name]->publish(percent_msg);

        RCLCPP_DEBUG(node_->get_logger(), "🎮 Hand percent sent to %s: %.3f",
                     arm_label.c_str(), ratio);
    }
} // namespace arms_ros2_control::command
