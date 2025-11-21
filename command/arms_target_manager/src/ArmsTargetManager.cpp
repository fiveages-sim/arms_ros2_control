//
// Created for Arms ROS2 Control - ArmsTargetManager
//

#include "arms_target_manager/ArmsTargetManager.h"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cmath>

namespace arms_ros2_control::command
{
    ArmsTargetManager::ArmsTargetManager(
        rclcpp::Node::SharedPtr node,
        const std::string& topicPrefix,
        bool dualArmMode,
        const std::string& frameId,
        const std::string& markerFixedFrame,
        double publishRate,
        const std::vector<int32_t>& disableAutoUpdateStates,
        double markerUpdateInterval,
        bool enableHeadControl,
        const std::string& headControllerName,
        const std::array<double, 3>& headMarkerPosition)
        : node_(std::move(node))
          , topic_prefix_(topicPrefix)
          , dual_arm_mode_(dualArmMode)
          , control_base_frame_(frameId)
          , marker_fixed_frame_(markerFixedFrame)
          , publish_rate_(publishRate)
          , current_mode_(MarkerState::SINGLE_SHOT)
          , current_controller_state_(2)
          , auto_update_enabled_(true)
          , disable_auto_update_states_(disableAutoUpdateStates)
          , last_marker_update_time_(node_->now())
          , marker_update_interval_(markerUpdateInterval)
          , enable_head_control_(enableHeadControl)
          , head_controller_name_(headControllerName)
          , head_marker_position_(headMarkerPosition)
    {
    }

    void ArmsTargetManager::initialize()
    {
        // 创建 InteractiveMarkerServer
        server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
            "arms_target_manager", node_);

        // 初始化TF2 buffer和listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        setupMenu();

        // 定义所有 lambda 函数
        // 初始化 pose 的辅助函数
        auto initPose = [](geometry_msgs::msg::Pose& pose, double x, double y, double z)
        {
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = z;
            pose.orientation.w = 1.0;
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;
        };

        // 初始化 marker 的辅助函数
        auto initMarker = [this](const std::string& markerName, const std::string& markerType,
                                   std::shared_ptr<interactive_markers::MenuHandler>& menuHandler)
        {
            auto marker = createMarker(markerName, markerType);
            server_->insert(marker);
            server_->setCallback(marker.name, [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
            {
                handleMarkerFeedback(feedback);
            });
            menuHandler->apply(*server_, marker.name);
        };

        // 左臂：初始化 pose -> 创建 marker
        initPose(left_pose_, 0.0, 0.5, 1.0);
        initMarker("left_arm_target", "left_arm", left_menu_handler_);

        // 右臂：初始化 pose -> 创建 marker（如果是双臂模式）
        if (dual_arm_mode_)
        {
            initPose(right_pose_, 0.0, -0.5, 1.0);
            initMarker("right_arm_target", "right_arm", right_menu_handler_);
        }

        // 头部：初始化 pose -> 创建 marker（如果启用头部控制）
        if (enable_head_control_)
        {
            initPose(head_pose_, head_marker_position_[0], head_marker_position_[1], head_marker_position_[2]);
            initMarker("head_target", "head", head_menu_handler_);
        }

        // 创建所有发布器和订阅器
        createPublishersAndSubscribers();

        updateMenuVisibility();

        server_->applyChanges();

        // 输出初始化信息
        RCLCPP_INFO(node_->get_logger(),
                    "ArmsTargetManager initialized. Mode: %s, Control Base Frame: %s, Marker Fixed Frame: %s, Publish Rate: %.1f Hz",
                    dual_arm_mode_ ? "dual_arm" : "single_arm",
                    control_base_frame_.c_str(),
                    marker_fixed_frame_.c_str(),
                    publish_rate_);
        RCLCPP_INFO(node_->get_logger(),
                    "📍 Markers will be created in frame: %s | "
                    "🔄 Received current_pose will be transformed to marker frame: %s | "
                    "📤 Published target poses will be transformed to control base frame: %s",
                    marker_fixed_frame_.c_str(),
                    marker_fixed_frame_.c_str(),
                    control_base_frame_.c_str());
    }

    void ArmsTargetManager::setMarkerPose(
        const std::string& armType,
        const geometry_msgs::msg::Point& position,
        const geometry_msgs::msg::Quaternion& orientation)
    {
        geometry_msgs::msg::Pose* current_pose = nullptr;
        std::string marker_name;


        if (armType == "left")
        {
            current_pose = &left_pose_;
            marker_name = "left_arm_target";
        }
        else if (armType == "right" && dual_arm_mode_)
        {
            current_pose = &right_pose_;
            marker_name = "right_arm_target";
        }
        else
        {
            return; // 无效的手臂类型
        }

        current_pose->position = position;
        current_pose->orientation = orientation;

        // 更新marker
        if (server_)
        {
            server_->setPose(marker_name, *current_pose);
            if (shouldUpdateMarker())
            {
                server_->applyChanges();
            }
        }

        // 在连续发布模式下，发送target pose（需要转换到control_base_frame_）
        if (current_mode_ == MarkerState::CONTINUOUS)
        {
            geometry_msgs::msg::Pose transformed_pose = transformPose(*current_pose, marker_fixed_frame_, control_base_frame_);
            if (armType == "left" && left_pose_publisher_)
            {
                left_pose_publisher_->publish(transformed_pose);
            }
            else if (armType == "right" && dual_arm_mode_ && right_pose_publisher_)
            {
                right_pose_publisher_->publish(transformed_pose);
            }
        }
    }

    void ArmsTargetManager::updateMarkerPoseIncremental(
        const std::string& armType,
        const std::array<double, 3>& positionDelta,
        const std::array<double, 3>& rpyDelta)
    {
        // 检查是否在禁用状态，只有在禁用状态下才允许增量更新
        if (!isStateDisabled(current_controller_state_))
        {
            RCLCPP_DEBUG(node_->get_logger(), "🎮 Incremental update blocked - controller state %d is not disabled",
                         current_controller_state_);
            return;
        }

        geometry_msgs::msg::Pose* current_pose = nullptr;
        std::string marker_name;

        if (armType == "left")
        {
            current_pose = &left_pose_;
            marker_name = "left_arm_target";
        }
        else if (armType == "right" && dual_arm_mode_)
        {
            current_pose = &right_pose_;
            marker_name = "right_arm_target";
        }
        else
        {
            return; // 无效的手臂类型
        }

        // 更新位置
        current_pose->position.x += positionDelta[0];
        current_pose->position.y += positionDelta[1];
        current_pose->position.z += positionDelta[2];

        // 更新旋转（使用RPY增量）
        if (std::abs(rpyDelta[0]) > 0.001 || std::abs(rpyDelta[1]) > 0.001 || std::abs(rpyDelta[2]) > 0.001)
        {
            // 将当前四元数转换为Eigen
            Eigen::Quaterniond current_quat(
                current_pose->orientation.w,
                current_pose->orientation.x,
                current_pose->orientation.y,
                current_pose->orientation.z
            );

            // 创建旋转增量
            Eigen::AngleAxisd rollAngle(rpyDelta[0], Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(rpyDelta[1], Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(rpyDelta[2], Eigen::Vector3d::UnitZ());

            // 组合旋转（ZYX顺序）
            Eigen::Quaterniond rotationIncrement = yawAngle * pitchAngle * rollAngle;

            // 方案 A：右乘 - 旋转相对于 marker 自身的局部坐标系
            // 特点：旋转绕 marker 当前的红/绿/蓝箭头方向
            // 适用场景：末端执行器视角控制，类似第一人称
            // current_quat = current_quat * rotationIncrement;

            // 方案 B：左乘 - 旋转相对于全局坐标系（世界坐标系）
            // 特点：旋转始终绕世界的 X/Y/Z 轴，与 marker 朝向无关
            // 适用场景：固定视角控制，与位置控制一致
            current_quat = rotationIncrement * current_quat;

            current_quat.normalize();

            // 转换回geometry_msgs
            current_pose->orientation.w = current_quat.w();
            current_pose->orientation.x = current_quat.x();
            current_pose->orientation.y = current_quat.y();
            current_pose->orientation.z = current_quat.z();
        }

        // 更新marker
        if (server_)
        {
            server_->setPose(marker_name, *current_pose);
            if (shouldUpdateMarker())
            {
                server_->applyChanges();
            }
        }

        // 在连续发布模式下，发送target pose（需要转换到control_base_frame_）
        if (current_mode_ == MarkerState::CONTINUOUS)
        {
            geometry_msgs::msg::Pose transformed_pose = transformPose(*current_pose, marker_fixed_frame_, control_base_frame_);
            if (armType == "left" && left_pose_publisher_)
            {
                left_pose_publisher_->publish(transformed_pose);
            }
            else if (armType == "right" && dual_arm_mode_ && right_pose_publisher_)
            {
                right_pose_publisher_->publish(transformed_pose);
            }
        }
    }


    geometry_msgs::msg::Pose ArmsTargetManager::getMarkerPose(const std::string& armType) const
    {
        if (armType == "left")
        {
            return left_pose_;
        }
        if (armType == "right" && dual_arm_mode_)
        {
            return right_pose_;
        }

        geometry_msgs::msg::Pose zero_pose;
        zero_pose.position.x = 0.0;
        zero_pose.position.y = 0.0;
        zero_pose.position.z = 0.0;
        zero_pose.orientation.w = 1.0;
        zero_pose.orientation.x = 0.0;
        zero_pose.orientation.y = 0.0;
        zero_pose.orientation.z = 0.0;
        return zero_pose;
    }

    visualization_msgs::msg::InteractiveMarker ArmsTargetManager::createMarker(
        const std::string& name,
        const std::string& markerType) const
    {
        // 头部 marker 逻辑
        if (markerType == "head")
        {
            visualization_msgs::msg::InteractiveMarker interactiveMarker;
            interactiveMarker.header.frame_id = marker_fixed_frame_;  // 统一使用 marker_fixed_frame_
            interactiveMarker.header.stamp = node_->now();
            interactiveMarker.name = name;
            interactiveMarker.scale = 0.25;
            interactiveMarker.description = "Head Target";

            // 使用配置的固定位置
            interactiveMarker.pose.position.x = head_pose_.position.x;
            interactiveMarker.pose.position.y = head_pose_.position.y;
            interactiveMarker.pose.position.z = head_pose_.position.z;
            interactiveMarker.pose.orientation = head_pose_.orientation;

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
            bool is_head_control_enabled = (current_controller_state_ == 3);

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
        // 左臂 marker 逻辑
        else if (markerType == "left_arm")
        {
            visualization_msgs::msg::InteractiveMarker interactiveMarker;
            interactiveMarker.header.frame_id = marker_fixed_frame_;
            interactiveMarker.header.stamp = node_->now();
            interactiveMarker.name = name;
            interactiveMarker.scale = 0.2;
            interactiveMarker.description = "Left Arm Target";

            // pose统一存储在marker_fixed_frame_下，直接使用
            interactiveMarker.pose = left_pose_;

            visualization_msgs::msg::Marker marker;

            MarkerState current_mode = current_mode_;

            if (current_mode == MarkerState::CONTINUOUS)
            {
                marker = createSphereMarker("blue");
            }
            else
            {
                marker = createBoxMarker("blue");
            }

            visualization_msgs::msg::InteractiveMarkerControl boxControl;
            boxControl.always_visible = true;
            boxControl.markers.push_back(marker);
            
            // 如果当前state启用自动更新（不在禁用列表中），则禁用交互功能
            bool is_auto_update_enabled = auto_update_enabled_ && !isStateDisabled(current_controller_state_);
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
        // 右臂 marker 逻辑
        else if (markerType == "right_arm")
        {
            visualization_msgs::msg::InteractiveMarker interactiveMarker;
            interactiveMarker.header.frame_id = marker_fixed_frame_;
            interactiveMarker.header.stamp = node_->now();
            interactiveMarker.name = name;
            interactiveMarker.scale = 0.2;
            interactiveMarker.description = "Right Arm Target";

            // pose统一存储在marker_fixed_frame_下，直接使用
            interactiveMarker.pose = right_pose_;

            visualization_msgs::msg::Marker marker;

            MarkerState current_mode = current_mode_;

            if (current_mode == MarkerState::CONTINUOUS)
            {
                marker = createSphereMarker("red");
            }
            else
            {
                marker = createBoxMarker("red");
            }

            visualization_msgs::msg::InteractiveMarkerControl boxControl;
            boxControl.always_visible = true;
            boxControl.markers.push_back(marker);
            
            // 如果当前state启用自动更新（不在禁用列表中），则禁用交互功能
            bool is_auto_update_enabled = auto_update_enabled_ && !isStateDisabled(current_controller_state_);
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
        // 未知的 marker 类型，返回空的 InteractiveMarker 并记录警告
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                       "Unknown marker type: '%s'. Returning empty InteractiveMarker.",
                       markerType.c_str());
            visualization_msgs::msg::InteractiveMarker interactiveMarker;
            interactiveMarker.header.frame_id = marker_fixed_frame_;
            interactiveMarker.header.stamp = node_->now();
            interactiveMarker.name = name;
            interactiveMarker.scale = 0.1;
            interactiveMarker.description = "Unknown Marker Type: " + markerType;
            return interactiveMarker;
        }
    }

    visualization_msgs::msg::Marker ArmsTargetManager::createBoxMarker(const std::string& color) const
    {
        visualization_msgs::msg::Marker marker;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

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
        else
        {
            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.5;
        }
        marker.color.a = 0.7;

        return marker;
    }

    void ArmsTargetManager::addMovementControls(
        visualization_msgs::msg::InteractiveMarker& interactiveMarker) const
    {
        visualization_msgs::msg::InteractiveMarkerControl control;
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

    void ArmsTargetManager::handleMarkerFeedback(
        const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
    {
        std::string source_frame_id = feedback->header.frame_id;
        std::string marker_name = feedback->marker_name;

        // 转换pose到目标frame（配置的marker_fixed_frame_）
        geometry_msgs::msg::Pose transformed_pose = transformPose(
            feedback->pose, source_frame_id, marker_fixed_frame_);

        // 根据 marker 名称分发处理
        if (marker_name == "left_arm_target")
        {
            // 左臂 marker 处理
            left_pose_ = transformed_pose;

            if (current_mode_ == MarkerState::CONTINUOUS)
            {
                // 在连续发布模式下，只发送左臂目标位姿
                sendTargetPose("left_arm");
            }
        }
        else if (marker_name == "right_arm_target")
        {
            // 右臂 marker 处理
            right_pose_ = transformed_pose;

            if (current_mode_ == MarkerState::CONTINUOUS)
            {
                // 在连续发布模式下，只发送右臂目标位姿
                sendTargetPose("right_arm");
            }
        }
        else if (marker_name == "head_target")
        {
            // 头部 marker 处理
            head_pose_ = transformed_pose;

            if (current_mode_ == MarkerState::CONTINUOUS)
            {
                // 在连续发布模式下，发送头部目标关节位置
                sendTargetPose("head");
            }
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                       "Unknown marker name in feedback: '%s'",
                       marker_name.c_str());
        }
    }



    void ArmsTargetManager::togglePublishMode()
    {
        if (current_mode_ == MarkerState::SINGLE_SHOT)
        {
            current_mode_ = MarkerState::CONTINUOUS;
        }
        else
        {
            current_mode_ = MarkerState::SINGLE_SHOT;
        }

        updateMarkerShape();
        updateMenuVisibility();

        server_->applyChanges();
    }

    MarkerState ArmsTargetManager::getCurrentMode() const
    {
        return current_mode_;
    }

    void ArmsTargetManager::sendTargetPose(const std::string& marker_type)
    {
        // 根据marker类型执行不同的发送操作
        if (marker_type == "head")
        {
            // 发送头部目标关节位置
            if (!head_joint_publisher_)
            {
                RCLCPP_WARN(node_->get_logger(), "Head joint publisher not initialized");
                return;
            }

            // 从头部pose的orientation提取关节角度
            std::vector<double> joint_angles = quaternionToHeadJointAngles(head_pose_.orientation);

            // 创建并发布消息
            std_msgs::msg::Float64MultiArray msg;
            msg.data = joint_angles;

            head_joint_publisher_->publish(msg);

            // RCLCPP_INFO(node_->get_logger(), 
            //            "Published head target joint angles: [%.3f, %.3f] (head_joint1, head_joint2)",
            //            joint_angles[0], joint_angles[1]);
        }
        else if (marker_type == "left_arm")
        {
            // 只发送左臂目标位姿
            geometry_msgs::msg::Pose transformed_left_pose = transformPose(left_pose_, marker_fixed_frame_, control_base_frame_);
            left_pose_publisher_->publish(transformed_left_pose);
        }
        else if (marker_type == "right_arm")
        {
            // 只发送右臂目标位姿
            geometry_msgs::msg::Pose transformed_right_pose = transformPose(right_pose_, marker_fixed_frame_, control_base_frame_);
            right_pose_publisher_->publish(transformed_right_pose);
        }
        else
        {
            // 未知的marker类型
            RCLCPP_WARN(node_->get_logger(),
                       "Unknown marker type: '%s'. Supported types: 'left_arm', 'right_arm', 'head'",
                       marker_type.c_str());
        }
    }


    void ArmsTargetManager::setupMarkerMenu(
        std::shared_ptr<interactive_markers::MenuHandler>& menu_handler,
        interactive_markers::MenuHandler::EntryHandle& send_handle,
        interactive_markers::MenuHandler::EntryHandle& toggle_handle,
        std::function<void()> sendCallback)
    {
        menu_handler = std::make_shared<interactive_markers::MenuHandler>();

        auto menuSendCallback = [sendCallback](
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& /*feedback*/)
        {
            sendCallback();
        };

        auto menuToggleCallback = [this](
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& /*feedback*/)
        {
            togglePublishMode();
        };

        send_handle = menu_handler->insert("发送目标", menuSendCallback);

        std::string toggleText = (current_mode_ == MarkerState::CONTINUOUS) 
            ? "切换到单次发布" 
            : "切换到连续发布";
        toggle_handle = menu_handler->insert(toggleText, menuToggleCallback);
    }

    void ArmsTargetManager::setupMenu()
    {
        // 为左臂设置菜单
            setupMarkerMenu(
                left_menu_handler_,
                left_send_handle_,
                left_toggle_handle_,
                [this]() { sendTargetPose("left_arm"); });

        // 为右臂设置菜单（如果是双臂模式）
        if (dual_arm_mode_)
        {
            setupMarkerMenu(
                right_menu_handler_,
                right_send_handle_,
                right_toggle_handle_,
                [this]() { sendTargetPose("right_arm"); });
        }

        // 为头部设置菜单（如果启用头部控制）
        if (enable_head_control_)
        {
            setupMarkerMenu(
                head_menu_handler_,
                head_send_handle_,
                head_toggle_handle_,
                [this]() { sendTargetPose("head"); });
        }
    }

    void ArmsTargetManager::updateMarkerShape()
    {
        // 统一的 marker 回调函数
        auto markerCallback = [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
        {
            handleMarkerFeedback(feedback);
        };

        // 更新左臂 marker
        auto leftMarker = createMarker("left_arm_target", "left_arm");
        server_->insert(leftMarker);
        server_->setCallback(leftMarker.name, markerCallback);
        left_menu_handler_->apply(*server_, leftMarker.name);

        if (dual_arm_mode_)
        {
            auto rightMarker = createMarker("right_arm_target", "right_arm");
            server_->insert(rightMarker);
            server_->setCallback(rightMarker.name, markerCallback);
            right_menu_handler_->apply(*server_, rightMarker.name);
        }

        // 更新头部 marker（如果启用头部控制）
        if (enable_head_control_)
        {
            auto headMarker = createMarker("head_target", "head");
            server_->insert(headMarker);
            server_->setCallback(headMarker.name, markerCallback);
            head_menu_handler_->apply(*server_, headMarker.name);
        }
    }

    void ArmsTargetManager::updateMenuVisibility()
    {
        setupMenu();

        left_menu_handler_->apply(*server_, "left_arm_target");
        if (dual_arm_mode_)
        {
            right_menu_handler_->apply(*server_, "right_arm_target");
        }

        if (current_mode_ == MarkerState::CONTINUOUS)
        {
            left_menu_handler_->setVisible(left_send_handle_, false);
            if (dual_arm_mode_)
            {
                right_menu_handler_->setVisible(right_send_handle_, false);
            }
            // 更新头部菜单可见性
            if (enable_head_control_)
            {
                head_menu_handler_->setVisible(head_send_handle_, false);
            }
        }
        else
        {
            left_menu_handler_->setVisible(left_send_handle_, true);
            if (dual_arm_mode_)
            {
                right_menu_handler_->setVisible(right_send_handle_, true);
            }
            // 更新头部菜单可见性
            if (enable_head_control_)
            {
                head_menu_handler_->setVisible(head_send_handle_, true);
            }
        }

        left_menu_handler_->reApply(*server_);
        if (dual_arm_mode_)
        {
            right_menu_handler_->reApply(*server_);
        }
        // 更新头部菜单
        if (enable_head_control_)
        {
            head_menu_handler_->reApply(*server_);
        }
    }

    void ArmsTargetManager::createPublishersAndSubscribers()
    {
        // 创建左臂发布器和订阅器
        left_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::Pose>("left_target", 1);
        left_end_effector_pose_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "left_current_pose", 10, [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
            {
                leftEndEffectorPoseCallback(msg);
            });

        // 创建右臂发布器和订阅器（如果是双臂模式）
        if (dual_arm_mode_)
        {
            right_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::Pose>("right_target", 1);
            right_end_effector_pose_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
                "right_current_pose", 10, [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
                {
                    rightEndEffectorPoseCallback(msg);
                });
        }

        // 创建头部发布器和订阅器（如果启用头部控制）
        if (enable_head_control_)
        {
            std::string head_topic = "/" + head_controller_name_ + "/target_joint_position";
            head_joint_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(head_topic, 1);
            head_joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, [this](const sensor_msgs::msg::JointState::ConstSharedPtr msg)
                {
                    headJointStateCallback(msg);
                });
        }
    }

    visualization_msgs::msg::Marker ArmsTargetManager::createSphereMarker(const std::string& color) const
    {
        visualization_msgs::msg::Marker marker;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

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
        else
        {
            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.5;
        }
        marker.color.a = 0.7;

        return marker;
    }


    void ArmsTargetManager::setAutoUpdateEnabled(bool enable)
    {
        auto_update_enabled_ = enable;
    }

    bool ArmsTargetManager::isAutoUpdateEnabled() const
    {
        return auto_update_enabled_;
    }

    void ArmsTargetManager::controlInputCallback(const arms_ros2_control_msgs::msg::Inputs::ConstSharedPtr msg)
    {
        int32_t new_state = msg->command;

        if (new_state == 0)
        {
            return;
        }

        if (new_state != current_controller_state_)
        {
            current_controller_state_ = new_state;
            
            // 处理状态切换时的特殊逻辑（根据不同 marker 类型执行相应操作）
            handleStateTransition(new_state);
            
            // 状态变化时重新创建marker
            updateMarkerShape();
            server_->applyChanges();
        }
    }

    void ArmsTargetManager::handleStateTransition(int32_t new_state)
    {
        // 头部状态切换处理：如果切换到 MOVE 状态（command = 3）且启用了头部控制
        // 将当前头部位置作为目标位置发布，确保头部保持当前位置
        if (new_state == 3 && enable_head_control_ && head_joint_publisher_)
        {
            // 如果有缓存的关节角度，使用缓存的；否则从 head_pose_ 提取
            if (last_head_joint_angles_.size() == 2)
            {
                std_msgs::msg::Float64MultiArray msg;
                msg.data = last_head_joint_angles_;
                head_joint_publisher_->publish(msg);
                RCLCPP_INFO(node_->get_logger(),
                           "Entered MOVE state, published current head position as target: [%.3f, %.3f]",
                           last_head_joint_angles_[0], last_head_joint_angles_[1]);
            }
            else
            {
                // 如果没有缓存，从 head_pose_ 提取（可能不是最新的，但总比没有好）
                sendTargetPose("head");
                RCLCPP_INFO(node_->get_logger(),
                           "Entered MOVE state, published head position from marker as target");
            }
        }

        // 未来可以在这里添加其他 marker 类型的状态切换处理
        // 例如：
        // if (new_state == 3 && enable_arms_control_)
        // {
        //     // 双臂的状态切换处理
        // }
    }


    void ArmsTargetManager::leftEndEffectorPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
        if (auto_update_enabled_ && !isStateDisabled(current_controller_state_))
        {
            // 将接收到的pose转换到marker_fixed_frame_下，使用最新的可用变换
            std::string source_frame_id = msg->header.frame_id;
            geometry_msgs::msg::Pose transformed_pose = transformPose(
                msg->pose, source_frame_id, marker_fixed_frame_);
            left_pose_ = transformed_pose;
            server_->setPose("left_arm_target", left_pose_);

            if (shouldUpdateMarker())
            {
                server_->applyChanges();
            }
        }
    }

    void ArmsTargetManager::rightEndEffectorPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
        if (auto_update_enabled_ && !isStateDisabled(current_controller_state_))
        {
            // 将接收到的pose转换到marker_fixed_frame_下，使用最新的可用变换
            std::string source_frame_id = msg->header.frame_id;
            geometry_msgs::msg::Pose transformed_pose = transformPose(
                msg->pose, source_frame_id, marker_fixed_frame_);
            right_pose_ = transformed_pose;
            server_->setPose("right_arm_target", right_pose_);

            if (shouldUpdateMarker())
            {
                server_->applyChanges();
            }
        }
    }


    bool ArmsTargetManager::isStateDisabled(int32_t state) const
    {
        return std::find(disable_auto_update_states_.begin(), disable_auto_update_states_.end(), state) !=
            disable_auto_update_states_.end();
    }

    bool ArmsTargetManager::shouldUpdateMarker()
    {
        auto now = node_->now();
        auto time_since_last_update = (now - last_marker_update_time_).seconds();

        if (time_since_last_update >= marker_update_interval_)
        {
            last_marker_update_time_ = now;
            return true;
        }
        return false;
    }

    geometry_msgs::msg::Pose ArmsTargetManager::transformPose(
        const geometry_msgs::msg::Pose& pose,
        const std::string& sourceFrameId,
        const std::string& targetFrameId) const
    {
        // 如果源frame和目标frame相同，不需要转换
        if (sourceFrameId == targetFrameId)
        {
            return pose;
        }

        try
        {
            // 创建PoseStamped用于转换
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = sourceFrameId;
            pose_stamped.header.stamp = rclcpp::Time(0);  // 使用Time(0)表示使用最新变换
            pose_stamped.pose = pose;

            // 获取最新的变换并使用doTransform进行转换
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                targetFrameId, sourceFrameId, tf2::TimePointZero);
            
            // 使用doTransform进行转换
            geometry_msgs::msg::PoseStamped result_stamped;
            tf2::doTransform(pose_stamped, result_stamped, transform);
            return result_stamped.pose;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "无法将pose从 %s 转换到 %s: %s，使用原始pose",
                        sourceFrameId.c_str(), targetFrameId.c_str(), ex.what());
            return pose;
        }
    }

    std::vector<double> ArmsTargetManager::quaternionToHeadJointAngles(
        const geometry_msgs::msg::Quaternion& quaternion) const
    {
        // 使用 tf2 的 getRPY 从 quaternion 提取欧拉角
        tf2::Quaternion tf_quat;
        tf2::fromMsg(quaternion, tf_quat);
        
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
        
        // yaw (Z轴旋转) -> head_joint1
        // pitch (Y轴旋转) -> head_joint2
        // 注意：pitch 取反，使得向上转动 marker 时头部向上看
        // 忽略 roll (X轴旋转)
        return {yaw, -pitch};
    }

    geometry_msgs::msg::Quaternion ArmsTargetManager::headJointAnglesToQuaternion(
        const std::vector<double>& joint_angles) const
    {
        if (joint_angles.size() < 2)
        {
            RCLCPP_WARN(node_->get_logger(), "Invalid joint angles size, expected 2, got %zu", joint_angles.size());
            geometry_msgs::msg::Quaternion quat;
            quat.w = 1.0;
            quat.x = 0.0;
            quat.y = 0.0;
            quat.z = 0.0;
            return quat;
        }

        // head_joint1 -> yaw (Z轴旋转)
        // head_joint2 -> pitch (Y轴旋转，需要取反)
        double yaw = joint_angles[0];
        double pitch = -joint_angles[1];  // 取反，与 quaternionToHeadJointAngles 对应
        double roll = 0.0;  // 忽略 roll

        // 使用 tf2 从 RPY 创建四元数
        tf2::Quaternion tf_quat;
        tf_quat.setRPY(roll, pitch, yaw);
        tf_quat.normalize();

        geometry_msgs::msg::Quaternion quat;
        quat.w = tf_quat.w();
        quat.x = tf_quat.x();
        quat.y = tf_quat.y();
        quat.z = tf_quat.z();
        return quat;
    }

    void ArmsTargetManager::headJointStateCallback(sensor_msgs::msg::JointState::ConstSharedPtr msg)
    {
        if (!enable_head_control_)
        {
            return;
        }

        // 头部 marker 在所有状态下都自动跟踪 head_link2 的 xyz 位置
        // 但在 MOVE 状态下不更新 orientation（orientation 由用户拖拽控制）
        
        // 检查是否应该更新 orientation
        // HOME (1) 和 HOLD (2) 状态：更新 position 和 orientation（自动跟踪）
        // MOVE (3) 状态：只更新 position，不更新 orientation（position 自动跟踪，orientation 用户控制）
        bool should_update_orientation = true;
        
        // 如果当前状态在禁用列表中（通常是 MOVE 状态），则不更新 orientation
        if (isStateDisabled(current_controller_state_))
        {
            should_update_orientation = false;
        }
        
        // 如果自动更新被禁用，也不更新 orientation
        if (!auto_update_enabled_)
        {
            should_update_orientation = false;
        }

        // 从 TF 获取 head_link2 的实际位置并更新（所有状态下都更新位置）
        try
        {
            // 获取 head_link2 在 marker_fixed_frame_ 中的位置（统一使用 marker_fixed_frame_）
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                marker_fixed_frame_, HEAD_LINK_NAME, tf2::TimePointZero);
            
            // 更新 marker 位置为 head_link2 的实际位置（所有状态下都更新）
            head_pose_.position.x = transform.transform.translation.x;
            head_pose_.position.y = transform.transform.translation.y;
            head_pose_.position.z = transform.transform.translation.z;
        }
        catch (const tf2::TransformException& ex)
        {
            // 如果 TF 转换失败，保持当前位置不变（使用固定位置或上次的位置）
            RCLCPP_DEBUG(node_->get_logger(),
                        "无法从 TF 获取头部 link %s 的位置: %s，保持当前位置",
                        HEAD_LINK_NAME, ex.what());
        }

        // 只有在需要更新 orientation 时才从 joint_states 获取并更新
        if (should_update_orientation)
        {
            // 从 joint_states 中查找 head_joint1 和 head_joint2
            double head_joint1_angle = 0.0;
            double head_joint2_angle = 0.0;
            bool found_joint1 = false;
            bool found_joint2 = false;

            for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
            {
                if (msg->name[i] == "head_joint1")
                {
                    head_joint1_angle = msg->position[i];
                    found_joint1 = true;
                }
                else if (msg->name[i] == "head_joint2")
                {
                    head_joint2_angle = msg->position[i];
                    found_joint2 = true;
                }

                if (found_joint1 && found_joint2)
                {
                    break;
                }
            }

            // 如果找到了两个关节，更新头部 marker 的 orientation
            if (found_joint1 && found_joint2)
            {
                // 将关节角度转换为四元数（确保顺序：head_joint1, head_joint2）
                std::vector<double> head_joint_angles = {head_joint1_angle, head_joint2_angle};
                
                // 缓存最新的关节角度，用于状态切换时发布
                last_head_joint_angles_ = head_joint_angles;
                
                geometry_msgs::msg::Quaternion quat = headJointAnglesToQuaternion(head_joint_angles);

                // 更新头部 pose 的 orientation（只在非 MOVE 状态下更新）
                head_pose_.orientation = quat;
            }
        }

        // 更新 marker（position 已更新，orientation 根据状态决定是否更新）
        server_->setPose("head_target", head_pose_);

        if (shouldUpdateMarker())
        {
            server_->applyChanges();
        }
    }

    visualization_msgs::msg::Marker ArmsTargetManager::createArrowMarker(const std::string& color) const
    {
        visualization_msgs::msg::Marker marker;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.scale.x = 0.15;  // 箭头长度
        marker.scale.y = 0.03;  // 箭头宽度
        marker.scale.z = 0.03;  // 箭头高度

        if (color == "green")
        {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
        }
        else if (color == "blue")
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
        else
        {
            marker.color.r = 0.5;
            marker.color.g = 0.5;
            marker.color.b = 0.5;
        }
        marker.color.a = 0.7;

        return marker;
    }




} // namespace arms_ros2_control::command
