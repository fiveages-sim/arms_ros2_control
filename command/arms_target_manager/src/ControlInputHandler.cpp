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
        double linearScale,
        double angularScale,
        double deadzone)
        : node_(std::move(node))
        , target_manager_(targetManager)
        , linear_scale_(linearScale)
        , angular_scale_(angularScale)
        , deadzone_(deadzone)
    {
        RCLCPP_INFO(node_->get_logger(), "🎮 ControlInputHandler created with scales: linear=%.3f, angular=%.3f", 
                    linear_scale_, angular_scale_);
    }

    void ControlInputHandler::processControlInput(const arms_ros2_control_msgs::msg::Inputs::ConstSharedPtr msg)
    {
        // 处理位置输入
        double x_input = applyDeadzone(msg->x, deadzone_);
        double y_input = applyDeadzone(msg->y, deadzone_);
        double z_input = applyDeadzone(msg->z, deadzone_);

        // 处理旋转输入
        double roll_input = applyDeadzone(msg->roll, deadzone_);
        double pitch_input = applyDeadzone(msg->pitch, deadzone_);
        double yaw_input = applyDeadzone(msg->yaw, deadzone_);

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
            std::string armType = (msg->target == 1) ? "left" : "right";
            target_manager_->updateMarkerPoseIncremental(armType, positionDelta, rpyDelta);

            RCLCPP_DEBUG(node_->get_logger(), "🎮 Updated %s arm pose incrementally: pos[%.3f, %.3f, %.3f], rpy[%.3f, %.3f, %.3f]",
                         armType.c_str(),
                         positionDelta[0], positionDelta[1], positionDelta[2],
                         rpyDelta[0], rpyDelta[1], rpyDelta[2]);
        }
    }


    void ControlInputHandler::setLinearScale(double scale)
    {
        linear_scale_ = scale;
        RCLCPP_INFO(node_->get_logger(), "🎮 Linear scale set to: %.3f", linear_scale_);
    }

    void ControlInputHandler::setAngularScale(double scale)
    {
        angular_scale_ = scale;
        RCLCPP_INFO(node_->get_logger(), "🎮 Angular scale set to: %.3f", angular_scale_);
    }

    void ControlInputHandler::setDeadzone(double deadzone)
    {
        deadzone_ = deadzone;
        RCLCPP_INFO(node_->get_logger(), "🎮 Deadzone set to: %.3f", deadzone_);
    }

    double ControlInputHandler::applyDeadzone(double value, double deadzone) const
    {
        if (std::abs(value) < deadzone)
        {
            return 0.0;
        }
        return value;
    }


} // namespace arms_ros2_control::command
