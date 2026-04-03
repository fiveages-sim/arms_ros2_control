//
// Created for Arms ROS2 Control - ArmsTargetManager
//

#include "arms_target_manager/ArmsTargetManager.h"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>
#include <cmath>
#include <algorithm>
#include <utility>
#include <chrono>
#include <arms_controller_common/utils/FSMStateTransitionValidator.h>

namespace arms_ros2_control::command
{
    ArmsTargetManager::ArmsTargetManager(
        rclcpp::Node::SharedPtr node,
        const bool dualArmMode,
        std::string frameId,
        std::string markerFixedFrame,
        const double publishRate,
        const std::vector<int32_t>& disableAutoUpdateStates,
        const double markerUpdateInterval)
        : node_(std::move(node))
          , dual_arm_mode_(dualArmMode)
          , control_base_frame_(std::move(frameId))
          , marker_fixed_frame_(std::move(markerFixedFrame))
          , publish_rate_(publishRate)
          , disable_auto_update_states_(disableAutoUpdateStates)
          , last_marker_update_time_(node_->now())
          , marker_update_interval_(markerUpdateInterval)
          , last_publish_time_(node_->now())
    {
    }

    void ArmsTargetManager::initialize(
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_left_target,
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_left_target_stamped,
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_right_target,
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_right_target_stamped)
    {
        // 创建 MarkerFactory（必须在 Marker 类之前创建）
        marker_factory_ = std::make_unique<MarkerFactory>(
            node_, marker_fixed_frame_, disable_auto_update_states_);

        // 初始化TF2 buffer和listener（必须在 Marker 类之前创建）
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 创建左臂 Marker（带更新回调和状态检查回调，使用外部发布器）
        left_arm_marker_ = std::make_shared<ArmMarker>(
            node_, marker_factory_, tf_buffer_, marker_fixed_frame_, control_base_frame_,
            ArmType::LEFT, std::array<double, 3>{0.0, 0.5, 1.0},
            pub_left_target, pub_left_target_stamped, "left_current_pose",
            publish_rate_,
            [this](const std::string& marker_name, const geometry_msgs::msg::Pose& pose)
            {
                // 如果 marker 不显示（enable_interaction = false），只更新数值，不更新可视化
                if (!isStateDisabled(current_controller_state_))
                {
                    return;  // marker 不显示时，不更新可视化位置
                }

                server_->setPose(marker_name, pose);
                // 标记有待应用的更改，由定时器统一处理
                markPendingChanges();

            });

        // 设置状态检查回调（用于控制是否允许 current_pose 更新 marker 的 pose_）
        left_arm_marker_->setStateCheckCallback(
            [this]() { return !isStateDisabled(current_controller_state_); });

        // 创建右臂 Marker（如果是双臂模式，带更新回调和状态检查回调，使用外部发布器）
        if (dual_arm_mode_)
        {
            right_arm_marker_ = std::make_shared<ArmMarker>(
                node_, marker_factory_, tf_buffer_, marker_fixed_frame_, control_base_frame_,
                ArmType::RIGHT, std::array<double, 3>{0.0, -0.5, 1.0},
                pub_right_target, pub_right_target_stamped, "right_current_pose",
                publish_rate_,
                [this](const std::string& marker_name, const geometry_msgs::msg::Pose& pose)
                {
                    // 如果 marker 不显示（enable_interaction = false），只更新数值，不更新可视化
                    if (!isStateDisabled(current_controller_state_))
                    {
                        return;  // marker 不显示时，不更新可视化位置
                    }

                    server_->setPose(marker_name, pose);
                    // 标记有待应用的更改，由定时器统一处理
                    markPendingChanges();
                });

            // 设置状态检查回调（用于控制是否允许 current_pose 更新 marker 的 pose_）
            right_arm_marker_->setStateCheckCallback(
                [this]() { return !isStateDisabled(current_controller_state_); });
        }

        // 创建 HeadMarker 实例并初始化
        head_marker_ = std::make_shared<HeadMarker>(
            node_, marker_factory_, tf_buffer_, marker_fixed_frame_, publish_rate_);
        head_marker_->initialize();

        // 创建 InteractiveMarkerServer
        server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
            "arms_target_manager", node_);

        setupMenu();

        // 初始化 marker 的辅助函数
        auto initMarker = [this](const std::string& markerName, const std::string& markerType,
                                 std::shared_ptr<interactive_markers::MenuHandler>& menuHandler)
        {
            auto marker = buildMarker(markerName, markerType);
            server_->insert(marker);
            server_->setCallback(marker.name,
                                 [this](
                                 const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
                                 {
                                     handleMarkerFeedback(feedback);
                                 });
            menuHandler->apply(*server_, marker.name);
        };

        // 左臂：创建 marker
        if (left_arm_marker_)
        {
            initMarker("left_arm_target", "left_arm", left_menu_handler_);
        }

        // 右臂：创建 marker（如果是双臂模式）
        if (dual_arm_mode_ && right_arm_marker_)
        {
            initMarker("right_arm_target", "right_arm", right_menu_handler_);
        }

        // 头部：初始化 pose -> 创建 marker（如果启用头部控制）
        if (head_marker_ && head_marker_->isEnabled())
        {
            initMarker("head_target", "head", head_menu_handler_);
        }

        // 创建所有发布器和订阅器
        createPublishersAndSubscribers();

        // 创建定时器，定期检查并应用 marker 更新
        // 使用 marker_update_interval_ 作为定时器周期，确保更新频率一致
        marker_update_timer_ = node_->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(marker_update_interval_)),
            [this]() { markerUpdateTimerCallback(); });

        updateMenuVisibility();

        // 标记有待应用的更改，由定时器统一处理
        markPendingChanges();

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

    visualization_msgs::msg::InteractiveMarker ArmsTargetManager::buildMarker(
        const std::string& name,
        const std::string& markerType) const
    {
        // 计算是否启用交互功能（如果状态不在禁用列表中，则启用交互）
        bool enable_interaction = isStateDisabled(current_controller_state_);

        // 根据类型直接调用对应的创建方法
        if (markerType == "left_arm" && left_arm_marker_)
        {
            return left_arm_marker_->createMarker(name, current_mode_, enable_interaction);
        }
        if (markerType == "right_arm" && right_arm_marker_)
        {
            return right_arm_marker_->createMarker(name, current_mode_, enable_interaction);
        }
        if (markerType == "head")
        {
            if (head_marker_ && head_marker_->isEnabled())
            {
                return head_marker_->createMarker(name, head_marker_->getPose(), enable_interaction);
            }
            // 如果头部未启用，返回一个空的 marker
            visualization_msgs::msg::InteractiveMarker empty_marker;
            empty_marker.name = name;
            return empty_marker;
        }

        // 默认使用 left_arm（包括未知类型）
        if (left_arm_marker_)
        {
            return left_arm_marker_->createMarker(name, current_mode_, enable_interaction);
        }
        
        // 如果左臂也未创建，返回空 marker
        visualization_msgs::msg::InteractiveMarker empty_marker;
        empty_marker.name = name;
        return empty_marker;
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
        if (marker_name == "left_arm_target" && left_arm_marker_)
        {
            // 左臂 marker 处理
            geometry_msgs::msg::Pose new_pose = left_arm_marker_->handleFeedback(feedback, source_frame_id);
            left_arm_marker_->setPose(new_pose);

            if (current_mode_ == MarkerState::CONTINUOUS)
            {
                // 在连续发布模式下，只发送左臂目标位姿（使用内部节流）
                left_arm_marker_->publishTargetPose();
            }
        }
        else if (marker_name == "right_arm_target" && right_arm_marker_)
        {
            // 右臂 marker 处理
            geometry_msgs::msg::Pose new_pose = right_arm_marker_->handleFeedback(feedback, source_frame_id);
            right_arm_marker_->setPose(new_pose);

            if (current_mode_ == MarkerState::CONTINUOUS)
            {
                // 在连续发布模式下，只发送右臂目标位姿（使用内部节流）
                right_arm_marker_->publishTargetPose();
            }
        }
        else if (marker_name == "head_target")
        {
            // 头部 marker 处理
            if (head_marker_ && head_marker_->isEnabled())
            {
                geometry_msgs::msg::Pose clamped_pose = transformed_pose;
                bool was_clamped = head_marker_->clampPoseRotation(clamped_pose);
                head_marker_->setPose(clamped_pose);

                // 如果被限制，更新 marker 位置，让用户看到限制效果
                // 只有在marker显示时才更新可视化
                if (was_clamped && server_ && isStateDisabled(current_controller_state_))
                {
                    server_->setPose(marker_name, clamped_pose);
                    // 标记有待应用的更改，由定时器统一处理
                    markPendingChanges();
                }

                if (current_mode_ == MarkerState::CONTINUOUS)
                {
                    // 在连续发布模式下，发送头部目标关节位置（使用内部节流）
                    head_marker_->publishTargetJointAngles();
                }
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

        // 标记有待应用的更改，由定时器统一处理
        markPendingChanges();
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
            if (head_marker_ && head_marker_->isEnabled())
            {
                // 单次模式下，强制发送，忽略节流
                head_marker_->publishTargetJointAngles(true);
            }
            return;
        }
        // 单次模式下，只发送对应的手臂（强制发送，忽略节流）
        // 单次发布时使用 stamped 话题，转换到 left_current_target 的 frame_id
        // 连续模式下，如果是双臂模式，拖动时会同时发送两个手臂（在 handleMarkerFeedback 中处理）
        if (marker_type == "left_arm" && left_arm_marker_)
        {
            left_arm_marker_->publishTargetPose(true, true);  // 强制发送，使用 stamped 话题
            return;
        }
        if (marker_type == "right_arm" && right_arm_marker_)
        {
            right_arm_marker_->publishTargetPose(true, true);  // 强制发送，使用 stamped 话题
        }
    }

    void ArmsTargetManager::sendDualArmTargetPose()
    {
        // 使用新的双臂接口：发布到 dual_target/stamped topic
        // Path 包含2个 pose：第一个是左臂，第二个是右臂
        if (!dual_arm_mode_ || !dual_target_stamped_publisher_ || !left_arm_marker_ || !right_arm_marker_)
        {
            RCLCPP_WARN(node_->get_logger(), "Cannot send dual arm target pose: dual arm mode not enabled or publishers not initialized");
            return;
        }

        // 获取左臂和右臂的 pose（在 marker_fixed_frame_ 坐标系下）
        geometry_msgs::msg::Pose left_pose = left_arm_marker_->getPose();
        geometry_msgs::msg::Pose right_pose = right_arm_marker_->getPose();

        // 转换到 control_base_frame_ 坐标系
        geometry_msgs::msg::Pose left_transformed = transformPose(
            left_pose, marker_fixed_frame_, control_base_frame_);
        geometry_msgs::msg::Pose right_transformed = transformPose(
            right_pose, marker_fixed_frame_, control_base_frame_);

        // 创建 Path 消息
        nav_msgs::msg::Path dual_path;
        dual_path.header.stamp = node_->get_clock()->now();
        dual_path.header.frame_id = control_base_frame_;
        dual_path.poses.resize(2);

        // 第一个 pose：左臂
        dual_path.poses[0].header.stamp = dual_path.header.stamp;
        dual_path.poses[0].header.frame_id = control_base_frame_;
        dual_path.poses[0].pose = left_transformed;

        // 第二个 pose：右臂
        dual_path.poses[1].header.stamp = dual_path.header.stamp;
        dual_path.poses[1].header.frame_id = control_base_frame_;
        dual_path.poses[1].pose = right_transformed;

        // 发布
        dual_target_stamped_publisher_->publish(dual_path);
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

    void ArmsTargetManager::setupDualArmMenu(
        std::shared_ptr<interactive_markers::MenuHandler>& menu_handler,
        interactive_markers::MenuHandler::EntryHandle& both_handle)
    {
        if (!dual_arm_mode_ || !menu_handler)
        {
            return;
        }

        auto menuBothCallback = [this](
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& /*feedback*/)
        {
            sendDualArmTargetPose();
        };

        both_handle = menu_handler->insert("发送双臂", menuBothCallback);
    }

    void ArmsTargetManager::setupMenu()
    {
        // 为左臂设置菜单
        setupMarkerMenu(
            left_menu_handler_,
            left_send_handle_,
            left_toggle_handle_,
            [this]() { sendTargetPose("left_arm"); });

        // 为左臂添加"发送双臂"按钮（如果是双臂模式）
        if (dual_arm_mode_)
        {
            setupDualArmMenu(left_menu_handler_, left_both_handle_);
        }

        // 为右臂设置菜单（如果是双臂模式）
        if (dual_arm_mode_)
        {
            setupMarkerMenu(
                right_menu_handler_,
                right_send_handle_,
                right_toggle_handle_,
                [this]() { sendTargetPose("right_arm"); });
            
            // 为右臂添加"发送双臂"按钮
            setupDualArmMenu(right_menu_handler_, right_both_handle_);
        }

        // 为头部设置菜单（如果启用头部控制）
        if (head_marker_ && head_marker_->isEnabled())
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
        auto leftMarker = buildMarker("left_arm_target", "left_arm");
        server_->insert(leftMarker);
        server_->setCallback(leftMarker.name, markerCallback);
        left_menu_handler_->apply(*server_, leftMarker.name);

        if (dual_arm_mode_)
        {
            auto rightMarker = buildMarker("right_arm_target", "right_arm");
            server_->insert(rightMarker);
            server_->setCallback(rightMarker.name, markerCallback);
            right_menu_handler_->apply(*server_, rightMarker.name);
        }

        // 更新头部 marker（如果启用头部控制）
        if (head_marker_ && head_marker_->isEnabled())
        {
            auto headMarker = buildMarker("head_target", "head");
            server_->insert(headMarker);
            server_->setCallback(headMarker.name, markerCallback);
            head_menu_handler_->apply(*server_, headMarker.name);
        }
    }

    void ArmsTargetManager::updateMenuVisibility()
    {
        setupMenu();

        // 应用菜单到 marker（重新创建菜单后需要先应用）
        left_menu_handler_->apply(*server_, "left_arm_target");
        if (dual_arm_mode_)
        {
            right_menu_handler_->apply(*server_, "right_arm_target");
        }
        // 应用头部菜单（如果启用头部控制）
        if (head_marker_ && head_marker_->isEnabled())
        {
            head_menu_handler_->apply(*server_, "head_target");
        }

        // 根据当前模式设置菜单项可见性
        if (current_mode_ == MarkerState::CONTINUOUS)
        {
            left_menu_handler_->setVisible(left_send_handle_, false);
            if (dual_arm_mode_)
            {
                right_menu_handler_->setVisible(right_send_handle_, false);
                // 在连续模式下，"发送双臂"按钮也隐藏
                left_menu_handler_->setVisible(left_both_handle_, false);
                right_menu_handler_->setVisible(right_both_handle_, false);
            }
            // 更新头部菜单可见性
            if (head_marker_ && head_marker_->isEnabled())
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
                // 在单次模式下，"发送双臂"按钮显示
                left_menu_handler_->setVisible(left_both_handle_, true);
                right_menu_handler_->setVisible(right_both_handle_, true);
            }
            // 更新头部菜单可见性
            if (head_marker_ && head_marker_->isEnabled())
            {
                head_menu_handler_->setVisible(head_send_handle_, true);
            }
        }

        // 重新应用菜单（更新可见性和菜单项文本）
        left_menu_handler_->reApply(*server_);
        if (dual_arm_mode_)
        {
            right_menu_handler_->reApply(*server_);
        }
        // 更新头部菜单
        if (head_marker_ && head_marker_->isEnabled())
        {
            head_menu_handler_->reApply(*server_);
        }
    }

    void ArmsTargetManager::createPublishersAndSubscribers()
    {
        // 订阅器现在都在各自的 marker 类内部管理
        // 只需要创建头部订阅器（如果启用头部控制）
        if (head_marker_ && head_marker_->isEnabled())
        {
            head_joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, [this](const sensor_msgs::msg::JointState::ConstSharedPtr msg)
                {
                    updateHeadMarkerFromTopic(msg);
                });
        }
        
        // 创建双臂目标发布器（仅双臂模式）
        if (dual_arm_mode_)
        {
            dual_target_stamped_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(
                "dual_target/stamped", 1);
        }
    }


    void ArmsTargetManager::fsmCommandCallback(std_msgs::msg::Int32::ConstSharedPtr msg)
    {
        int32_t command = msg->data;

        if (command == 0)
        {
            return;
        }

        // 使用公共的状态转换验证工具类
        std::string new_state;
        bool valid_transition = arms_controller_common::FSMStateTransitionValidator::validateTransition(
            current_fsm_state_, command, new_state);

        // 只有在有效转换时才更新状态
        if (valid_transition)
        {
            current_fsm_state_ = new_state;
            current_controller_state_ = command;

            // 状态变化时重新创建marker
            updateMarkerShape();
            // 标记有待应用的更改，由定时器统一处理
            markPendingChanges();
        }
    }

    void ArmsTargetManager::setCurrentPoseCallback(
        const std::string& armType,
        std::function<void(const geometry_msgs::msg::PoseStamped::ConstSharedPtr&)> callback)
    {
        if (armType == "left" && left_arm_marker_)
        {
            left_arm_marker_->setCurrentPoseCallback(callback);
        }
        else if (armType == "right" && dual_arm_mode_ && right_arm_marker_)
        {
            right_arm_marker_->setCurrentPoseCallback(callback);
        }
    }

    void ArmsTargetManager::updateHeadMarkerFromTopic(
        const sensor_msgs::msg::JointState::ConstSharedPtr& joint_msg)
    {
        if (!head_marker_ || !head_marker_->isEnabled())
        {
            return;
        }

        // 如果 marker 不显示（状态不在禁用列表中），不需要更新
        if (!isStateDisabled(current_controller_state_))
        {
            return;  // marker 不显示时，不进行任何更新操作
        }

        // 使用 HeadMarker 更新
        geometry_msgs::msg::Pose updated_pose = head_marker_->updateFromJointState(
            joint_msg, isStateDisabled(current_controller_state_));

        // 更新 marker 可视化
        server_->setPose("head_target", updated_pose);
        // 标记有待应用的更改，由定时器统一处理
        markPendingChanges();
    }


    bool ArmsTargetManager::isStateDisabled(int32_t state) const
    {
        return std::find(disable_auto_update_states_.begin(), disable_auto_update_states_.end(), state) !=
            disable_auto_update_states_.end();
    }

    bool ArmsTargetManager::shouldThrottle(rclcpp::Time& last_time, double interval)
    {
        auto now = node_->now();
        auto time_since_last = (now - last_time).seconds();

        if (time_since_last >= interval)
        {
            last_time = now;
            return true;
        }
        return false;
    }

    void ArmsTargetManager::markPendingChanges()
    {
        // 标记有待应用的更改
        bool was_pending = pending_changes_.exchange(true, std::memory_order_acq_rel);
        
        // 如果之前没有待处理的更改，且距离上次应用已经超过最小间隔的一半，
        // 立即应用一次，减少延迟并避免序列号乱序
        if (!was_pending && server_)
        {
            auto now = node_->now();
            auto time_since_last = (now - last_marker_update_time_).seconds();
            
            // 如果距离上次更新已经超过最小间隔的一半，立即应用
            // 这样可以减少延迟，同时避免过于频繁的更新
            if (time_since_last >= marker_update_interval_ * 0.5)
            {
                server_->applyChanges();
                last_marker_update_time_ = now;
                pending_changes_.store(false, std::memory_order_release);
            }
        }
    }

    void ArmsTargetManager::markerUpdateTimerCallback()
    {
        // 检查是否有待应用的更改
        if (pending_changes_.exchange(false, std::memory_order_acq_rel))
        {
            // 有更改，应用更新
            // 定时器回调本身已经是串行的，可以直接调用 applyChanges()
            if (server_)
            {
                server_->applyChanges();
                last_marker_update_time_ = node_->now();
            }
        }
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
            pose_stamped.header.stamp = rclcpp::Time(0); // 使用Time(0)表示使用最新变换
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
            // 转换失败时直接使用原始pose，不输出警告
            return pose;
        }
    }



    // ============================================================================
    // 外部 API 接口函数（供 VRInputHandler、ControlInputHandler 等外部类调用）
    // ============================================================================
    // 这些函数与 marker 的创建和管理逻辑分离，专门用于外部控制接口
    // 主要包括：VR 输入、手柄控制输入等外部控制方式

    /**
     * 设置 marker 的绝对位姿（供外部调用，如 VR 输入）
     *
     * 这是一个外部 API 接口，主要用于：
     * - VRInputHandler: VR 控制器输入时调用，设置 marker 的绝对位置和方向
     * - 其他外部控制接口：需要直接设置 marker 目标位姿的场景
     *
     * 功能：
     * 1. 更新内部存储的 pose（通过 ArmMarker）
     * 2. 更新 interactive marker 的可视化位置
     * 3. 在连续发布模式下，自动发布目标位姿到控制器
     *
     * @param armType 手臂类型："left" 或 "right"
     * @param position 目标位置（在 marker_fixed_frame_ 坐标系下）
     * @param orientation 目标方向（四元数）
     *
     * @note 如果 armType 无效或不在双臂模式下请求右臂，函数将静默返回
     * @note 在连续发布模式下，会自动转换坐标系并发布目标位姿
     *
     * @see VRInputHandler::updateMarkerPose() - VR 输入调用此函数
     */
    void ArmsTargetManager::setMarkerPose(
        const std::string& armType,
        const geometry_msgs::msg::Point& position,
        const geometry_msgs::msg::Quaternion& orientation)
    {
        std::shared_ptr<ArmMarker> arm_marker = nullptr;

        if (armType == "left" && left_arm_marker_)
        {
            arm_marker = left_arm_marker_;
        }
        else if (armType == "right" && dual_arm_mode_ && right_arm_marker_)
        {
            arm_marker = right_arm_marker_;
        }
        else
        {
            return; // 无效的手臂类型
        }

        // 更新内部 pose 存储
        geometry_msgs::msg::Pose new_pose;
        new_pose.position = position;
        new_pose.orientation = orientation;
        arm_marker->setPose(new_pose);

        // 更新 interactive marker 的可视化位置
        // 只有在marker显示时才更新可视化（避免在不显示的状态下不必要的更新）
        if (server_ && isStateDisabled(current_controller_state_))
        {
            server_->setPose(arm_marker->getMarkerName(), new_pose);
            // 标记有待应用的更改，由定时器统一处理
            markPendingChanges();
        }

        // 在连续发布模式下，自动发送目标位姿到控制器（使用内部节流）
        if (current_mode_ == MarkerState::CONTINUOUS)
        {
            arm_marker->publishTargetPose();
        }
    }

    /**
     * 增量更新 marker 的位姿（供外部调用，如手柄控制输入）
     *
     * 这是一个外部 API 接口，主要用于：
     * - ControlInputHandler: 手柄/游戏手柄输入时调用，基于增量值更新 marker 位置
     * - 其他需要相对控制的外部接口：基于增量输入调整 marker 位置的场景
     *
     * 功能：
     * 1. 检查控制器状态：只有在禁用状态（如 HOME、HOLD）下才允许增量更新
     * 2. 基于增量值更新位置（相对当前位置的偏移）
     * 3. 基于 RPY 增量更新旋转（使用全局坐标系）
     * 4. 更新 interactive marker 的可视化位置
     * 5. 在连续发布模式下，自动发布目标位姿到控制器
     *
     * @param armType 手臂类型："left" 或 "right"
     * @param positionDelta 位置增量 [x, y, z]（在 marker_fixed_frame_ 坐标系下，已缩放）
     * @param rpyDelta 旋转增量 [roll, pitch, yaw]（弧度，已缩放）
     *
     * @note 只有在禁用状态（disable_auto_update_states_）下才允许增量更新
     * @note 如果在启用状态（如 MOVE）下调用，函数会静默返回
     * @note 旋转使用全局坐标系（世界坐标系），与 marker 当前朝向无关
     * @note 在连续发布模式下，会自动转换坐标系并发布目标位姿
     *
     * @see ControlInputHandler::processControlInput() - 手柄控制调用此函数
     */
    void ArmsTargetManager::updateMarkerPoseIncremental(
        const std::string& armType,
        const std::array<double, 3>& positionDelta,
        const std::array<double, 3>& rpyDelta)
    {
        // 检查是否在禁用状态，只有在禁用状态下才允许增量更新
        // 这是为了避免在 MOVE 状态下与自动跟踪冲突
        if (!isStateDisabled(current_controller_state_))
        {
            RCLCPP_DEBUG(node_->get_logger(), "🎮 Incremental update blocked - controller state %d is not disabled",
                         current_controller_state_);
            return;
        }

        std::shared_ptr<ArmMarker> arm_marker = nullptr;

        if (armType == "left" && left_arm_marker_)
        {
            arm_marker = left_arm_marker_;
        }
        else if (armType == "right" && dual_arm_mode_ && right_arm_marker_)
        {
            arm_marker = right_arm_marker_;
        }
        else
        {
            return; // 无效的手臂类型
        }

        // 获取当前 pose
        geometry_msgs::msg::Pose current_pose = arm_marker->getPose();

        // 更新位置：基于当前位置添加增量
        current_pose.position.x += positionDelta[0];
        current_pose.position.y += positionDelta[1];
        current_pose.position.z += positionDelta[2];

        // 更新旋转：使用 RPY 增量（仅在增量足够大时）
        if (std::abs(rpyDelta[0]) > 0.001 || std::abs(rpyDelta[1]) > 0.001 || std::abs(rpyDelta[2]) > 0.001)
        {
            // 将当前四元数转换为 Eigen 格式
            Eigen::Quaterniond current_quat(
                current_pose.orientation.w,
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z
            );

            // 创建旋转增量（RPY 欧拉角）
            Eigen::AngleAxisd rollAngle(rpyDelta[0], Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(rpyDelta[1], Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(rpyDelta[2], Eigen::Vector3d::UnitZ());

            // 组合旋转（ZYX 顺序：先 roll，再 pitch，最后 yaw）
            Eigen::Quaterniond rotationIncrement = yawAngle * pitchAngle * rollAngle;

            // 使用全局坐标系旋转（左乘）
            // 特点：旋转始终绕世界的 X/Y/Z 轴，与 marker 当前朝向无关
            // 适用场景：固定视角控制，与位置控制一致（符合用户直觉）
            current_quat = rotationIncrement * current_quat;
            current_quat.normalize();

            // 转换回 geometry_msgs 格式
            current_pose.orientation.w = current_quat.w();
            current_pose.orientation.x = current_quat.x();
            current_pose.orientation.y = current_quat.y();
            current_pose.orientation.z = current_quat.z();
        }

        // 更新内部 pose
        arm_marker->setPose(current_pose);

        // 更新 interactive marker 的可视化位置
        // 只有在marker显示时才更新可视化（避免在不显示的状态下不必要的更新）
        // 注意：updateMarkerPoseIncremental 已经在函数开头检查了 isStateDisabled，
        // 所以这里一定是在禁用状态下，marker是显示的
        if (server_)
        {
            server_->setPose(arm_marker->getMarkerName(), current_pose);
            // 标记有待应用的更改，由定时器统一处理
            markPendingChanges();
        }

        // 在连续发布模式下，自动发送目标位姿到控制器（使用内部节流）
        if (current_mode_ == MarkerState::CONTINUOUS)
        {
            arm_marker->publishTargetPose();
        }
    }

    /**
     * 获取 marker 的当前位姿（供外部查询调用）
     *
     * 这是一个外部 API 接口，用于：
     * - 外部系统查询当前 marker 的目标位姿
     * - 状态监控和日志记录
     * - 其他需要读取 marker 位置的场景
     *
     * 功能：
     * - 返回指定手臂的当前目标位姿（存储在 marker_fixed_frame_ 坐标系下）
     * - 如果手臂类型无效，返回零位姿（默认值）
     *
     * @param armType 手臂类型："left" 或 "right"
     * @return 当前 marker 的位姿（geometry_msgs::msg::Pose）
     *         如果 armType 无效，返回零位姿（位置为原点，方向为单位四元数）
     *
     * @note 返回的 pose 在 marker_fixed_frame_ 坐标系下
     * @note 如果请求右臂但不在双臂模式下，返回零位姿
     */
    geometry_msgs::msg::Pose ArmsTargetManager::getMarkerPose(const std::string& armType) const
    {
        if (armType == "left" && left_arm_marker_)
        {
            return left_arm_marker_->getPose();
        }
        if (armType == "right" && dual_arm_mode_ && right_arm_marker_)
        {
            return right_arm_marker_->getPose();
        }

        // 无效的手臂类型，返回零位姿（默认值）
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
} // namespace arms_ros2_control::command
