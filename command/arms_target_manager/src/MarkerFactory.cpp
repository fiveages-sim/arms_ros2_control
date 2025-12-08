//
// Created for Arms ROS2 Control - MarkerFactory
// Marker 创建工厂类实现
//

#include "arms_target_manager/MarkerFactory.h"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>
#include <utility>

namespace arms_ros2_control::command
{
    MarkerFactory::MarkerFactory(
        rclcpp::Node::SharedPtr node,
        std::string  frame_id,
        const std::vector<int32_t>& disable_states)
        : node_(std::move(node))
          , frame_id_(std::move(frame_id))
          , disable_states_(disable_states)
    {
    }

    visualization_msgs::msg::InteractiveMarker MarkerFactory::createArmMarker(
        const std::string& name,
        const std::string& description,
        const geometry_msgs::msg::Pose& pose,
        const std::string& color,
        MarkerState mode,
        bool enable_interaction) const
    {
        visualization_msgs::msg::InteractiveMarker interactiveMarker;
        interactiveMarker.header.frame_id = frame_id_;
        interactiveMarker.header.stamp = node_->now();
        interactiveMarker.name = name;
        interactiveMarker.scale = 0.2;
        interactiveMarker.description = description;

        // 设置位姿
        interactiveMarker.pose = pose;

        // 根据模式选择形状：连续模式 = 球体，单次模式 = 盒子
        visualization_msgs::msg::Marker marker;
        if (mode == MarkerState::CONTINUOUS)
        {
            marker = createSphereMarker(color);
        }
        else
        {
            marker = createBoxMarker(color);
        }

        // 创建控制
        visualization_msgs::msg::InteractiveMarkerControl boxControl;
        boxControl.always_visible = true;
        boxControl.markers.push_back(marker);

        // 根据是否启用交互功能设置交互模式
        if (enable_interaction)
        {
            boxControl.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;
        }
        else
        {
            boxControl.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;
        }

        interactiveMarker.controls.push_back(boxControl);

        // 只有在启用交互功能时才添加移动控制
        if (enable_interaction)
        {
            addMovementControls(interactiveMarker);
        }

        return interactiveMarker;
    }

    visualization_msgs::msg::InteractiveMarker MarkerFactory::createHeadMarker(
        const std::string& name,
        const geometry_msgs::msg::Pose& pose,
        bool enable_interaction,
        const std::set<std::string>& available_joints) const
    {
        visualization_msgs::msg::InteractiveMarker interactiveMarker;
        interactiveMarker.header.frame_id = frame_id_;
        interactiveMarker.header.stamp = node_->now();
        interactiveMarker.name = name;
        interactiveMarker.scale = 0.25;
        interactiveMarker.description = "Head Target";

        // 设置位姿
        interactiveMarker.pose = pose;

        // 创建箭头marker表示头部朝向
        visualization_msgs::msg::Marker arrowMarker = createArrowMarker("red");

        // 箭头marker只用于显示，不用于交互
        visualization_msgs::msg::InteractiveMarkerControl arrowControl;
        arrowControl.always_visible = true;
        arrowControl.markers.push_back(arrowMarker);
        arrowControl.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;
        interactiveMarker.controls.push_back(arrowControl);

        // 根据是否启用交互功能和可用的关节添加旋转控制
        if (enable_interaction)
        {
            visualization_msgs::msg::InteractiveMarkerControl control;

            // 根据可用的关节添加对应的旋转控制
            if (available_joints.find("head_roll") != available_joints.end())
            {
                // 绕X轴旋转（roll）
                control.orientation.w = 1;
                control.orientation.x = 1;
                control.orientation.y = 0;
                control.orientation.z = 0;
                control.name = "rotate_x";
                control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
                interactiveMarker.controls.push_back(control);
            }

            if (available_joints.find("head_pitch") != available_joints.end())
            {
                // 绕Y轴旋转（pitch）
                control.orientation.w = 1;
                control.orientation.x = 0;
                control.orientation.y = 1;
                control.orientation.z = 0;
                control.name = "rotate_y";
                control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
                interactiveMarker.controls.push_back(control);
            }

            if (available_joints.find("head_yaw") != available_joints.end())
            {
                // 绕Z轴旋转（yaw）
                control.orientation.w = 1;
                control.orientation.x = 0;
                control.orientation.y = 0;
                control.orientation.z = 1;
                control.name = "rotate_z";
                control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
                interactiveMarker.controls.push_back(control);
            }
        }

        return interactiveMarker;
    }

    visualization_msgs::msg::Marker MarkerFactory::createBoxMarker(const std::string& color) const
    {
        visualization_msgs::msg::Marker marker;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        setMarkerColor(marker, color);
        return marker;
    }

    visualization_msgs::msg::Marker MarkerFactory::createSphereMarker(const std::string& color) const
    {
        visualization_msgs::msg::Marker marker;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        setMarkerColor(marker, color);
        return marker;
    }

    visualization_msgs::msg::Marker MarkerFactory::createArrowMarker(const std::string& color) const
    {
        visualization_msgs::msg::Marker marker;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.scale.x = 0.15;  // 箭头长度
        marker.scale.y = 0.03;  // 箭头宽度
        marker.scale.z = 0.03;  // 箭头高度
        setMarkerColor(marker, color);
        return marker;
    }

    void MarkerFactory::addMovementControls(
        visualization_msgs::msg::InteractiveMarker& interactiveMarker) const
    {
        visualization_msgs::msg::InteractiveMarkerControl control;

        // X轴旋转和移动
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        interactiveMarker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        interactiveMarker.controls.push_back(control);

        // Y轴旋转和移动
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_y";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        interactiveMarker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        interactiveMarker.controls.push_back(control);

        // Z轴旋转和移动
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_z";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
        interactiveMarker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        interactiveMarker.controls.push_back(control);
    }

    void MarkerFactory::setMarkerColor(
        visualization_msgs::msg::Marker& marker,
        const std::string& color) const
    {
        if (color == "blue")
        {
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }
        else if (color == "red")
        {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
        }
        else if (color == "green")
        {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        else
        {
            // 默认灰色
            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.5;
        }
        marker.color.a = 0.7;
    }

    bool MarkerFactory::isStateDisabled(int32_t state) const
    {
        return std::find(disable_states_.begin(), disable_states_.end(), state) != disable_states_.end();
    }

} // namespace arms_ros2_control::command

