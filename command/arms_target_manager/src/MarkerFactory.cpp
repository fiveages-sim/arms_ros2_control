//
// Created for Arms ROS2 Control - MarkerFactory
// Marker 创建工厂类实现
//

#include "arms_target_manager/MarkerFactory.h"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cmath>

namespace arms_ros2_control::command
{
    MarkerFactory::MarkerFactory(
        rclcpp::Node::SharedPtr node,
        const std::string& frame_id,
        const std::vector<int32_t>& disable_states)
        : node_(std::move(node))
          , frame_id_(frame_id)
          , disable_states_(disable_states)
    {
    }

    visualization_msgs::msg::InteractiveMarker MarkerFactory::createMarker(
        const std::string& name,
        const std::string& markerType,
        const geometry_msgs::msg::Pose& pose,
        MarkerState mode,
        int32_t controller_state,
        bool auto_update_enabled) const
    {
        // 根据 marker 类型分发到对应的创建函数
        if (markerType == "left_arm")
        {
            // 判断是否启用自动更新
            bool is_auto_update_enabled = auto_update_enabled && !isStateDisabled(controller_state);
            return createArmMarker(name, "Left Arm Target", pose, "blue", mode, is_auto_update_enabled);
        }
        else if (markerType == "right_arm")
        {
            // 判断是否启用自动更新
            bool is_auto_update_enabled = auto_update_enabled && !isStateDisabled(controller_state);
            return createArmMarker(name, "Right Arm Target", pose, "red", mode, is_auto_update_enabled);
        }
        else if (markerType == "head")
        {
            return createHeadMarker(name, pose, controller_state);
        }
        else
        {
            // 未知的 marker 类型，返回空的 InteractiveMarker 并记录警告
            RCLCPP_WARN(node_->get_logger(),
                       "Unknown marker type: '%s'. Returning empty InteractiveMarker.",
                       markerType.c_str());
            visualization_msgs::msg::InteractiveMarker interactiveMarker;
            interactiveMarker.header.frame_id = frame_id_;
            interactiveMarker.header.stamp = node_->now();
            interactiveMarker.name = name;
            interactiveMarker.scale = 0.1;
            interactiveMarker.description = "Unknown Marker Type: " + markerType;
            return interactiveMarker;
        }
    }

    visualization_msgs::msg::InteractiveMarker MarkerFactory::createArmMarker(
        const std::string& name,
        const std::string& description,
        const geometry_msgs::msg::Pose& pose,
        const std::string& color,
        MarkerState mode,
        bool is_auto_update_enabled) const
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

        // 如果当前state启用自动更新，则禁用交互功能
        if (is_auto_update_enabled)
        {
            boxControl.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;
        }
        else
        {
            boxControl.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;
        }

        interactiveMarker.controls.push_back(boxControl);

        // 只有在非自动更新模式下才添加移动控制
        if (!is_auto_update_enabled)
        {
            addMovementControls(interactiveMarker);
        }

        return interactiveMarker;
    }

    visualization_msgs::msg::InteractiveMarker MarkerFactory::createHeadMarker(
        const std::string& name,
        const geometry_msgs::msg::Pose& pose,
        int32_t controller_state) const
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

        // 只有在 MOVE 状态（command = 3）时才启用交互功能
        // HOME (1) 和 HOLD (2) 状态时禁用交互
        bool is_head_control_enabled = (controller_state == 3);

        if (is_head_control_enabled)
        {
            // 只添加左右旋转（yaw）和上下旋转（pitch）控制，不添加roll旋转
            // 左右旋转（绕Z轴 - yaw）
            visualization_msgs::msg::InteractiveMarkerControl control;
            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 0;
            control.orientation.z = 1;
            control.name = "rotate_z";
            control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
            interactiveMarker.controls.push_back(control);

            // 上下旋转（绕Y轴 - pitch）
            control.orientation.w = 1;
            control.orientation.x = 0;
            control.orientation.y = 1;
            control.orientation.z = 0;
            control.name = "rotate_y";
            control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
            interactiveMarker.controls.push_back(control);
        }
        // 如果不在 MOVE 状态，不添加交互控制，marker 将不可交互

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

