//
// Created for Arms ROS2 Control - ControlInputHandler
//

#include "arms_target_manager/ControlInputHandler.h"
#include "arms_target_manager/ArmsTargetManager.h"
#include "arms_target_manager/MarkerFactory.h"
#include <std_msgs/msg/int32.hpp>

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
        // 直接使用输入值，死区处理已在输入源层面完成
        double x_input = msg->x;
        double y_input = msg->y;
        double z_input = msg->z;
        double roll_input = msg->roll;
        double pitch_input = msg->pitch;
        double yaw_input = msg->yaw;

        // 检查是否有有效输入
        bool hasValidInput = std::abs(x_input) > 0.001 || std::abs(y_input) > 0.001 || std::abs(z_input) > 0.001 ||
            std::abs(roll_input) > 0.001 || std::abs(pitch_input) > 0.001 || std::abs(yaw_input) > 0.001;

        if (hasValidInput && target_manager_)
        {
            // 确保切换到连续发布模式以获得更好的响应性
            if (target_manager_->getCurrentMode() != MarkerState::CONTINUOUS)
            {
                target_manager_->togglePublishMode();
                RCLCPP_INFO(node_->get_logger(), "🎮 ArmsTargetManager switched to CONTINUOUS mode for control input");
            }

            // 应用缩放因子
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

            // 根据target选择更新左臂或右臂
            std::string armType = msg->target == 1 ? "left" : "right";
            target_manager_->updateMarkerPoseIncremental(armType, positionDelta, rpyDelta);

            RCLCPP_DEBUG(node_->get_logger(),
                         "🎮 Updated %s arm pose incrementally: pos[%.3f, %.3f, %.3f], rpy[%.3f, %.3f, %.3f]",
                         armType.c_str(),
                         positionDelta[0], positionDelta[1], positionDelta[2],
                         rpyDelta[0], rpyDelta[1], rpyDelta[2]);
        }

        // 处理hand_command（如果有）
        if (msg->hand_command != -1) {
            processHandCommand(msg->target, msg->hand_command);
        }
    }

    void ControlInputHandler::processHandCommand(int32_t target, int32_t handCommand)
    {
        // 检查是否有配置的hand controllers
        if (hand_controllers_.empty()) {
            RCLCPP_DEBUG(node_->get_logger(), "🎮 Hand command received but no hand controllers configured (target=%d, command=%d)",
                        target, handCommand);
            return;
        }

        // 选择控制器索引
        size_t controller_index;
        if (hand_controllers_.size() == 1) {
            // 单臂模式：无论target是1还是2，都使用唯一的控制器（index 0）
            controller_index = 0;
            RCLCPP_DEBUG(node_->get_logger(), "🎮 Single arm mode: using controller[0] regardless of target=%d", target);
        } else {
            // 双臂模式：根据target选择控制器
            // target=1 (left) -> index 0, target=2 (right) -> index 1
            controller_index = static_cast<size_t>(target - 1);
            
            // 检查索引是否有效
            if (controller_index >= hand_controllers_.size()) {
                RCLCPP_WARN(node_->get_logger(), "🎮 Hand command for target=%d but only %zu controller(s) configured",
                           target, hand_controllers_.size());
                return;
            }
        }

        std::string controller_name = hand_controllers_[controller_index];
        
        // 创建发布器（如果还没有）
        if (hand_command_publishers_.find(controller_name) == hand_command_publishers_.end()) {
            std::string topic_name = "/" + controller_name + "/target_command";
            hand_command_publishers_[controller_name] = 
                node_->create_publisher<std_msgs::msg::Int32>(topic_name, 10);
            RCLCPP_INFO(node_->get_logger(), "🎮 Created hand command publisher: %s", topic_name.c_str());
        }

        // 发布命令
        auto target_msg = std_msgs::msg::Int32();
        target_msg.data = handCommand;
        hand_command_publishers_[controller_name]->publish(target_msg);

        // 日志输出
        if (hand_controllers_.size() == 1) {
            // 单臂模式：不区分左右
            RCLCPP_INFO(node_->get_logger(), "🎮 Hand command sent to controller (%s): %s",
                       controller_name.c_str(),
                       (handCommand == 1) ? "OPEN" : "CLOSE");
        } else {
            // 双臂模式：显示左右
            std::string arm_name = (target == 1) ? "LEFT" : "RIGHT";
            RCLCPP_INFO(node_->get_logger(), "🎮 Hand command sent to %s controller (%s): %s",
                       arm_name.c_str(), controller_name.c_str(),
                       (handCommand == 1) ? "OPEN" : "CLOSE");
        }
    }
} // namespace arms_ros2_control::command
