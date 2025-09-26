//
// Created for Arms ROS2 Control - ControlInputHandler
//

#include "arms_target_manager/ControlInputHandler.h"
#include "arms_target_manager/ArmsTargetManager.h"

namespace arms_ros2_control::command
{

    ControlInputHandler::ControlInputHandler(
        rclcpp::Node::SharedPtr node,
        ArmsTargetManager* targetManager,
        const double linearScale,
        const double angularScale)
        : node_(std::move(node))
        , target_manager_(targetManager)
        , linear_scale_(linearScale)
        , angular_scale_(angularScale)
    {
        RCLCPP_INFO(node_->get_logger(), "🎮 ControlInputHandler created with scales: linear=%.3f, angular=%.3f", 
                    linear_scale_, angular_scale_);
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
        bool hasValidInput = (std::abs(x_input) > 0.001 || std::abs(y_input) > 0.001 || std::abs(z_input) > 0.001 ||
                             std::abs(roll_input) > 0.001 || std::abs(pitch_input) > 0.001 || std::abs(yaw_input) > 0.001);

        if (hasValidInput && target_manager_)
        {
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

            RCLCPP_DEBUG(node_->get_logger(), "🎮 Updated %s arm pose incrementally: pos[%.3f, %.3f, %.3f], rpy[%.3f, %.3f, %.3f]",
                         armType.c_str(),
                         positionDelta[0], positionDelta[1], positionDelta[2],
                         rpyDelta[0], rpyDelta[1], rpyDelta[2]);
        }
    }





} // namespace arms_ros2_control::command
