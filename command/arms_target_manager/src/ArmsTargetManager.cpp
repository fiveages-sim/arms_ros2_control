//
// Created for Arms ROS2 Control - ArmsTargetManager
//

#include "arms_target_manager/ArmsTargetManager.h"
#include "arms_target_manager/marker/BodyMarker.h"
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
    namespace
    {
        Eigen::Isometry3d poseToIsometry(const geometry_msgs::msg::Pose& pose)
        {
            Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
            tf.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
            Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
            if (q.norm() < 1e-9)
            {
                q = Eigen::Quaterniond::Identity();
            }
            else
            {
                q.normalize();
            }
            tf.linear() = q.toRotationMatrix();
            return tf;
        }

        geometry_msgs::msg::Pose isometryToPose(const Eigen::Isometry3d& tf)
        {
            geometry_msgs::msg::Pose pose;
            pose.position.x = tf.translation().x();
            pose.position.y = tf.translation().y();
            pose.position.z = tf.translation().z();
            const Eigen::Quaterniond q(tf.linear());
            pose.orientation.w = q.w();
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            return pose;
        }

        geometry_msgs::msg::Pose applyRelativePose(
            const geometry_msgs::msg::Pose& old_anchor_pose,
            const geometry_msgs::msg::Pose& old_follower_pose,
            const geometry_msgs::msg::Pose& new_anchor_pose)
        {
            const Eigen::Isometry3d old_anchor = poseToIsometry(old_anchor_pose);
            const Eigen::Isometry3d old_follower = poseToIsometry(old_follower_pose);
            const Eigen::Isometry3d new_anchor = poseToIsometry(new_anchor_pose);
            const Eigen::Isometry3d relative = old_anchor.inverse() * old_follower;
            return isometryToPose(new_anchor * relative);
        }
    } // namespace

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

    bool ArmsTargetManager::shouldShowLeftArmMarker() const
    {
        return left_arm_state_ != 0;
    }

    bool ArmsTargetManager::shouldShowRightArmMarker() const
    {
        return right_arm_state_ != 0;
    }

    bool ArmsTargetManager::shouldShowBodyMarker() const
    {
        return (current_controller_state_ == 3) && (body_state_ == 2);
    }

    void ArmsTargetManager::updateBodyMarkerVisibility()
    {
        if (!server_ || !body_marker_)
        {
            return;
        }

        const bool show = shouldShowBodyMarker();

        if (show)
        {
            auto marker = buildMarker("body_target", "body");
            server_->insert(marker);
            server_->setCallback(
                marker.name,
                [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
                {
                    handleMarkerFeedback(feedback);
                });

            if (body_menu_handler_)
            {
                body_menu_handler_->apply(*server_, marker.name);
            }
        }
        else
        {
            server_->erase("body_target");
        }

        markPendingChanges();
    }

    void ArmsTargetManager::wbcStateCallback(
        const arms_ros2_control_msgs::msg::WbcCurrentState::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        const int prev_left_arm_state = left_arm_state_;
        const int prev_right_arm_state = right_arm_state_;
        const int prev_bimanual_state = bimanual_state_;
        const int prev_body_state = body_state_;

        left_arm_state_ = msg->left_arm_state;
        right_arm_state_ = msg->right_arm_state;
        bimanual_state_ = msg->bimanual_state;
        body_state_ = msg->body_state;

        const bool bimanual_state_changed = (prev_bimanual_state != bimanual_state_);

        if (bimanual_state_changed)
        {
            refreshArmMarkersFromLatestCurrentTargets();
        }

        if (prev_left_arm_state != left_arm_state_ ||
            prev_right_arm_state != right_arm_state_ ||
            prev_body_state != body_state_)
        {
            updateMarkerShape();
            updateMenuVisibility();
            updateBodyMarkerVisibility();
            markPendingChanges();
        }
    }

    void ArmsTargetManager::initialize(
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_left_target,
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_left_target_stamped,
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_right_target,
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_right_target_stamped)
    {
        marker_factory_ = std::make_unique<MarkerFactory>(
            node_, marker_fixed_frame_, disable_auto_update_states_);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        left_arm_marker_ = std::make_shared<ArmMarker>(
            node_, marker_factory_, tf_buffer_, marker_fixed_frame_, control_base_frame_,
            ArmType::LEFT, std::array<double, 3>{0.0, 0.5, 1.0},
            pub_left_target, pub_left_target_stamped, "left_current_pose",
            publish_rate_,
            [this](const std::string& marker_name, const geometry_msgs::msg::Pose& pose)
            {
                if (!isStateDisabled(current_controller_state_))
                {
                    return;
                }

                if (current_mode_ == MarkerState::SINGLE_SHOT)
                {
                    return;
                }

                if (!shouldShowLeftArmMarker() && marker_name == "left_arm_target")
                {
                    return;
                }
                if (!shouldShowRightArmMarker() && marker_name == "right_arm_target")
                {
                    return;
                }

                server_->setPose(marker_name, pose);
                markPendingChanges();
            });

        left_arm_marker_->setStateCheckCallback(
            [this]() { return !isStateDisabled(current_controller_state_); });

        if (dual_arm_mode_)
        {
            right_arm_marker_ = std::make_shared<ArmMarker>(
                node_, marker_factory_, tf_buffer_, marker_fixed_frame_, control_base_frame_,
                ArmType::RIGHT, std::array<double, 3>{0.0, -0.5, 1.0},
                pub_right_target, pub_right_target_stamped, "right_current_pose",
                publish_rate_,
                [this](const std::string& marker_name, const geometry_msgs::msg::Pose& pose)
                {
                    if (!isStateDisabled(current_controller_state_))
                    {
                        return;
                    }

                    if (current_mode_ == MarkerState::SINGLE_SHOT)
                    {
                        return;
                    }

                    if (!shouldShowLeftArmMarker() && marker_name == "left_arm_target")
                    {
                        return;
                    }
                    if (!shouldShowRightArmMarker() && marker_name == "right_arm_target")
                    {
                        return;
                    }

                    server_->setPose(marker_name, pose);
                    markPendingChanges();
                });

            right_arm_marker_->setStateCheckCallback(
                [this]() { return !isStateDisabled(current_controller_state_); });
        }

        head_marker_ = std::make_shared<HeadMarker>(
            node_, marker_factory_, tf_buffer_, marker_fixed_frame_, publish_rate_);
        head_marker_->initialize();

        body_target_stamped_publisher_ =
            node_->create_publisher<geometry_msgs::msg::PoseStamped>(
                "body_target/stamped", 10);

        body_marker_ = std::make_shared<BodyMarker>(
            node_,
            marker_factory_,
            tf_buffer_,
            marker_fixed_frame_,
            control_base_frame_,
            body_target_stamped_publisher_,
            "body_current_pose",
            publish_rate_,
            [this](const std::string& marker_name, const geometry_msgs::msg::Pose& pose)
            {
                if (shouldShowBodyMarker())
                {
                    return;
                }

                (void)marker_name;
                (void)pose;
            });

        body_marker_->setStateCheckCallback(
            [this]() { return !shouldShowBodyMarker(); });

        server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
            "arms_target_manager", node_);

        setupMenu();

        auto initMarker = [this](const std::string& markerName, const std::string& markerType,
                                 std::shared_ptr<interactive_markers::MenuHandler>& menuHandler)
        {
            auto marker = buildMarker(markerName, markerType);
            if (marker.name.empty())
            {
                return;
            }
            server_->insert(marker);
            server_->setCallback(marker.name,
                                 [this](
                                 const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
                                 {
                                     handleMarkerFeedback(feedback);
                                 });
            menuHandler->apply(*server_, marker.name);
        };

        if (left_arm_marker_ && shouldShowLeftArmMarker())
        {
            initMarker("left_arm_target", "left_arm", left_menu_handler_);
        }

        if (dual_arm_mode_ && right_arm_marker_ && shouldShowRightArmMarker())
        {
            initMarker("right_arm_target", "right_arm", right_menu_handler_);
        }

        if (head_marker_ && head_marker_->isEnabled())
        {
            initMarker("head_target", "head", head_menu_handler_);
        }

        createPublishersAndSubscribers();

        marker_update_timer_ = node_->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(marker_update_interval_)),
            [this]() { markerUpdateTimerCallback(); });

        updateMenuVisibility();
        updateBodyMarkerVisibility();
        markPendingChanges();

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
        bool enable_interaction = isStateDisabled(current_controller_state_);

        if (markerType == "left_arm" && left_arm_marker_)
        {
            if (!shouldShowLeftArmMarker())
            {
                visualization_msgs::msg::InteractiveMarker empty_marker;
                empty_marker.name = name;
                return empty_marker;
            }
            return left_arm_marker_->createMarker(name, current_mode_, enable_interaction);
        }

        if (markerType == "right_arm" && right_arm_marker_)
        {
            if (!shouldShowRightArmMarker())
            {
                visualization_msgs::msg::InteractiveMarker empty_marker;
                empty_marker.name = name;
                return empty_marker;
            }
            return right_arm_marker_->createMarker(name, current_mode_, enable_interaction);
        }

        if (markerType == "head")
        {
            if (head_marker_ && head_marker_->isEnabled())
            {
                return head_marker_->createMarker(name, head_marker_->getPose(), enable_interaction);
            }
            visualization_msgs::msg::InteractiveMarker empty_marker;
            empty_marker.name = name;
            return empty_marker;
        }

        if (markerType == "body" && body_marker_)
        {
            if (!shouldShowBodyMarker())
            {
                visualization_msgs::msg::InteractiveMarker empty_marker;
                empty_marker.name = name;
                return empty_marker;
            }
            return body_marker_->createMarker(name, current_mode_, true);
        }

        if (left_arm_marker_)
        {
            return left_arm_marker_->createMarker(name, current_mode_, enable_interaction);
        }

        visualization_msgs::msg::InteractiveMarker empty_marker;
        empty_marker.name = name;
        return empty_marker;
    }

    void ArmsTargetManager::handleMarkerFeedback(
        const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
    {
        std::string source_frame_id = feedback->header.frame_id;
        std::string marker_name = feedback->marker_name;

        geometry_msgs::msg::Pose transformed_pose = transformPose(
            feedback->pose, source_frame_id, marker_fixed_frame_);

        if (marker_name == "left_arm_target" && left_arm_marker_)
        {
            if (!shouldShowLeftArmMarker())
            {
                return;
            }

            const geometry_msgs::msg::Pose old_left_pose = left_arm_marker_->getPose();
            geometry_msgs::msg::Pose old_right_pose;
            const bool can_follow_right =
                (dual_arm_mode_ &&
                 right_arm_marker_ &&
                 shouldShowRightArmMarker() &&
                 bimanual_state_ == arms_ros2_control_msgs::msg::WbcCurrentState::BIMANUAL_COUPLED);
            if (can_follow_right)
            {
                old_right_pose = right_arm_marker_->getPose();
            }

            geometry_msgs::msg::Pose new_pose = left_arm_marker_->handleFeedback(feedback, source_frame_id);
            left_arm_marker_->setPose(new_pose);

            // 单次发布 + 双臂耦合：仅做本地marker联动，不自动下发控制命令
            if (current_mode_ == MarkerState::SINGLE_SHOT && can_follow_right)
            {
                const geometry_msgs::msg::Pose new_right_pose =
                    applyRelativePose(old_left_pose, old_right_pose, new_pose);
                right_arm_marker_->setPose(new_right_pose);
                if (server_ && isStateDisabled(current_controller_state_))
                {
                    server_->setPose(right_arm_marker_->getMarkerName(), new_right_pose);
                    markPendingChanges();
                }
            }

            if (current_mode_ == MarkerState::CONTINUOUS)
            {
                left_arm_marker_->publishTargetPose();
                if (body_marker_ && shouldShowBodyMarker())
                {
                    body_marker_->publishTargetPose(true);
                }
            }
        }
        else if (marker_name == "right_arm_target" && right_arm_marker_)
        {
            if (!shouldShowRightArmMarker())
            {
                return;
            }

            const geometry_msgs::msg::Pose old_right_pose = right_arm_marker_->getPose();
            geometry_msgs::msg::Pose old_left_pose;
            const bool can_follow_left =
                (left_arm_marker_ &&
                 shouldShowLeftArmMarker() &&
                 bimanual_state_ == arms_ros2_control_msgs::msg::WbcCurrentState::BIMANUAL_COUPLED);
            if (can_follow_left)
            {
                old_left_pose = left_arm_marker_->getPose();
            }

            geometry_msgs::msg::Pose new_pose = right_arm_marker_->handleFeedback(feedback, source_frame_id);
            right_arm_marker_->setPose(new_pose);

            // 单次发布 + 双臂耦合：仅做本地marker联动，不自动下发控制命令
            if (current_mode_ == MarkerState::SINGLE_SHOT && can_follow_left)
            {
                const geometry_msgs::msg::Pose new_left_pose =
                    applyRelativePose(old_right_pose, old_left_pose, new_pose);
                left_arm_marker_->setPose(new_left_pose);
                if (server_ && isStateDisabled(current_controller_state_))
                {
                    server_->setPose(left_arm_marker_->getMarkerName(), new_left_pose);
                    markPendingChanges();
                }
            }

            if (current_mode_ == MarkerState::CONTINUOUS)
            {
                right_arm_marker_->publishTargetPose();
                if (body_marker_ && shouldShowBodyMarker())
                {
                    body_marker_->publishTargetPose(true);
                }
            }
        }
        else if (marker_name == "head_target")
        {
            if (head_marker_ && head_marker_->isEnabled())
            {
                geometry_msgs::msg::Pose clamped_pose = transformed_pose;
                bool was_clamped = head_marker_->clampPoseRotation(clamped_pose);
                head_marker_->setPose(clamped_pose);

                if (was_clamped && server_ && isStateDisabled(current_controller_state_))
                {
                    server_->setPose(marker_name, clamped_pose);
                    markPendingChanges();
                }

                if (current_mode_ == MarkerState::CONTINUOUS)
                {
                    head_marker_->publishTargetJointAngles();
                }
            }
        }
        else if (marker_name == "body_target" && body_marker_)
        {
            if (!shouldShowBodyMarker())
            {
                return;
            }

            geometry_msgs::msg::Pose new_pose = body_marker_->handleFeedback(feedback, source_frame_id);
            body_marker_->setPose(new_pose);

            if (current_mode_ == MarkerState::CONTINUOUS)
            {
                body_marker_->publishTargetPose();
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
        const MarkerState previous_mode = current_mode_;

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
        updateBodyMarkerVisibility();

        if (previous_mode == MarkerState::SINGLE_SHOT &&
            current_mode_ == MarkerState::CONTINUOUS)
        {
            refreshArmMarkersFromLatestCurrentTargets();
        }

        markPendingChanges();
    }

    MarkerState ArmsTargetManager::getCurrentMode() const
    {
        return current_mode_;
    }

    void ArmsTargetManager::sendTargetPose(const std::string& marker_type)
    {
        const bool is_bimanual_coupled =
            dual_arm_mode_ &&
            (bimanual_state_ == arms_ros2_control_msgs::msg::WbcCurrentState::BIMANUAL_COUPLED) &&
            shouldShowLeftArmMarker() &&
            shouldShowRightArmMarker();

        if (marker_type == "head")
        {
            if (head_marker_ && head_marker_->isEnabled())
            {
                head_marker_->publishTargetJointAngles(true);
            }
            return;
        }

        if (marker_type == "body")
        {
            sendBodyTargetPose();
            return;
        }

        // 双臂耦合下，任一侧“发送目标”都应统一发送双臂，
        // 避免只更新单臂current_target导致后续模式切换时另一侧marker回跳。
        if (is_bimanual_coupled &&
            (marker_type == "left_arm" || marker_type == "right_arm"))
        {
            sendDualArmTargetPose();
            return;
        }

        if (marker_type == "left_arm" && left_arm_marker_ && shouldShowLeftArmMarker())
        {
            left_arm_marker_->publishTargetPose(true, true);
            return;
        }

        if (marker_type == "right_arm" && right_arm_marker_ && shouldShowRightArmMarker())
        {
            right_arm_marker_->publishTargetPose(true, true);
            return;
        }
    }

    void ArmsTargetManager::sendDualArmTargetPose()
    {
        if (!dual_arm_mode_ || !dual_target_stamped_publisher_ || !left_arm_marker_ || !right_arm_marker_)
        {
            RCLCPP_WARN(node_->get_logger(), "Cannot send dual arm target pose: dual arm mode not enabled or publishers not initialized");
            return;
        }

        if (!shouldShowLeftArmMarker() || !shouldShowRightArmMarker())
        {
            RCLCPP_WARN(node_->get_logger(), "Cannot send dual arm target pose: one or both arm markers are hidden");
            return;
        }

        geometry_msgs::msg::Pose left_pose = left_arm_marker_->getPose();
        geometry_msgs::msg::Pose right_pose = right_arm_marker_->getPose();

        geometry_msgs::msg::Pose left_transformed = transformPose(
            left_pose, marker_fixed_frame_, control_base_frame_);
        geometry_msgs::msg::Pose right_transformed = transformPose(
            right_pose, marker_fixed_frame_, control_base_frame_);

        nav_msgs::msg::Path dual_path;
        dual_path.header.stamp = node_->get_clock()->now();
        dual_path.header.frame_id = control_base_frame_;
        const bool include_body = (body_marker_ && shouldShowBodyMarker());
        dual_path.poses.resize(include_body ? 3 : 2);

        dual_path.poses[0].header.stamp = dual_path.header.stamp;
        dual_path.poses[0].header.frame_id = control_base_frame_;
        dual_path.poses[0].pose = left_transformed;

        dual_path.poses[1].header.stamp = dual_path.header.stamp;
        dual_path.poses[1].header.frame_id = control_base_frame_;
        dual_path.poses[1].pose = right_transformed;

        // 若 body marker 可用，三条轨迹（左/右/腰）合并成一次 dual_target/stamped 下发，
        // 由接收端统一插值规划，避免多topic覆盖。
        if (include_body)
        {
            geometry_msgs::msg::Pose body_pose = body_marker_->getPose();
            geometry_msgs::msg::Pose body_transformed = transformPose(
                body_pose, marker_fixed_frame_, control_base_frame_);
            dual_path.poses[2].header.stamp = dual_path.header.stamp;
            dual_path.poses[2].header.frame_id = control_base_frame_;
            dual_path.poses[2].pose = body_transformed;
        }

        dual_target_stamped_publisher_->publish(dual_path);
    }

    void ArmsTargetManager::sendBodyTargetPose()
    {
        if (!body_marker_ || !shouldShowBodyMarker())
        {
            RCLCPP_WARN(node_->get_logger(), "Cannot send body target pose: body marker not available");
            return;
        }

        body_marker_->publishTargetPose(true);
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
        setupMarkerMenu(
            left_menu_handler_,
            left_send_handle_,
            left_toggle_handle_,
            [this]() { sendTargetPose("left_arm"); });

        if (dual_arm_mode_)
        {
            setupDualArmMenu(left_menu_handler_, left_both_handle_);
        }

        if (dual_arm_mode_)
        {
            setupMarkerMenu(
                right_menu_handler_,
                right_send_handle_,
                right_toggle_handle_,
                [this]() { sendTargetPose("right_arm"); });

            setupDualArmMenu(right_menu_handler_, right_both_handle_);
        }

        if (head_marker_ && head_marker_->isEnabled())
        {
            setupMarkerMenu(
                head_menu_handler_,
                head_send_handle_,
                head_toggle_handle_,
                [this]() { sendTargetPose("head"); });
        }

        setupMarkerMenu(
            body_menu_handler_,
            body_send_handle_,
            body_toggle_handle_,
            [this]() { sendTargetPose("body"); });
    }

    void ArmsTargetManager::updateMarkerShape()
    {
        auto markerCallback = [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
        {
            handleMarkerFeedback(feedback);
        };

        if (left_arm_marker_ && shouldShowLeftArmMarker())
        {
            auto leftMarker = buildMarker("left_arm_target", "left_arm");
            server_->insert(leftMarker);
            server_->setCallback(leftMarker.name, markerCallback);
            left_menu_handler_->apply(*server_, leftMarker.name);
        }
        else if (left_arm_marker_)
        {
            server_->erase("left_arm_target");
        }

        if (dual_arm_mode_ && right_arm_marker_ && shouldShowRightArmMarker())
        {
            auto rightMarker = buildMarker("right_arm_target", "right_arm");
            server_->insert(rightMarker);
            server_->setCallback(rightMarker.name, markerCallback);
            right_menu_handler_->apply(*server_, rightMarker.name);
        }
        else if (dual_arm_mode_ && right_arm_marker_)
        {
            server_->erase("right_arm_target");
        }

        if (head_marker_ && head_marker_->isEnabled())
        {
            auto headMarker = buildMarker("head_target", "head");
            server_->insert(headMarker);
            server_->setCallback(headMarker.name, markerCallback);
            head_menu_handler_->apply(*server_, headMarker.name);
        }

        if (body_marker_ && shouldShowBodyMarker())
        {
            auto bodyMarker = buildMarker("body_target", "body");
            server_->insert(bodyMarker);
            server_->setCallback(bodyMarker.name, markerCallback);
            body_menu_handler_->apply(*server_, bodyMarker.name);
        }
        else if (body_marker_)
        {
            server_->erase("body_target");
        }
    }

    void ArmsTargetManager::updateMenuVisibility()
    {
        setupMenu();

        if (left_arm_marker_ && shouldShowLeftArmMarker())
        {
            left_menu_handler_->apply(*server_, "left_arm_target");
        }
        if (dual_arm_mode_ && right_arm_marker_ && shouldShowRightArmMarker())
        {
            right_menu_handler_->apply(*server_, "right_arm_target");
        }

        if (head_marker_ && head_marker_->isEnabled())
        {
            head_menu_handler_->apply(*server_, "head_target");
        }

        if (body_marker_ && shouldShowBodyMarker())
        {
            body_menu_handler_->apply(*server_, "body_target");
        }

        if (current_mode_ == MarkerState::CONTINUOUS)
        {
            if (left_arm_marker_ && shouldShowLeftArmMarker())
            {
                left_menu_handler_->setVisible(left_send_handle_, false);
            }

            if (dual_arm_mode_ && right_arm_marker_ && shouldShowRightArmMarker())
            {
                right_menu_handler_->setVisible(right_send_handle_, false);
            }

            if (dual_arm_mode_)
            {
                if (shouldShowLeftArmMarker() && shouldShowRightArmMarker())
                {
                    left_menu_handler_->setVisible(left_both_handle_, false);
                    right_menu_handler_->setVisible(right_both_handle_, false);
                }
                else
                {
                    left_menu_handler_->setVisible(left_both_handle_, false);
                    right_menu_handler_->setVisible(right_both_handle_, false);
                }
            }

            if (head_marker_ && head_marker_->isEnabled())
            {
                head_menu_handler_->setVisible(head_send_handle_, false);
            }

            if (body_marker_ && shouldShowBodyMarker())
            {
                body_menu_handler_->setVisible(body_send_handle_, false);
            }
        }
        else
        {
            if (left_arm_marker_ && shouldShowLeftArmMarker())
            {
                left_menu_handler_->setVisible(left_send_handle_, true);
            }

            if (dual_arm_mode_ && right_arm_marker_ && shouldShowRightArmMarker())
            {
                right_menu_handler_->setVisible(right_send_handle_, true);
            }

            if (dual_arm_mode_)
            {
                const bool show_both = shouldShowLeftArmMarker() && shouldShowRightArmMarker();
                left_menu_handler_->setVisible(left_both_handle_, show_both);
                right_menu_handler_->setVisible(right_both_handle_, show_both);
            }

            if (head_marker_ && head_marker_->isEnabled())
            {
                head_menu_handler_->setVisible(head_send_handle_, true);
            }

            if (body_marker_ && shouldShowBodyMarker())
            {
                body_menu_handler_->setVisible(body_send_handle_, true);
            }
        }

        if (left_arm_marker_ && shouldShowLeftArmMarker())
        {
            left_menu_handler_->reApply(*server_);
        }
        if (dual_arm_mode_ && right_arm_marker_ && shouldShowRightArmMarker())
        {
            right_menu_handler_->reApply(*server_);
        }

        if (head_marker_ && head_marker_->isEnabled())
        {
            head_menu_handler_->reApply(*server_);
        }

        if (body_marker_ && shouldShowBodyMarker())
        {
            body_menu_handler_->reApply(*server_);
        }
    }

    void ArmsTargetManager::createPublishersAndSubscribers()
    {
        if (head_marker_ && head_marker_->isEnabled())
        {
            head_joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, [this](const sensor_msgs::msg::JointState::ConstSharedPtr msg)
                {
                    updateHeadMarkerFromTopic(msg);
                });
        }

        if (dual_arm_mode_)
        {
            dual_target_stamped_publisher_ = node_->create_publisher<nav_msgs::msg::Path>(
                "dual_target/stamped", 1);
        }

        wbc_state_subscriber_ =
            node_->create_subscription<arms_ros2_control_msgs::msg::WbcCurrentState>(
                "/ocs2_wbc_controller/current_state",
                10,
                std::bind(&ArmsTargetManager::wbcStateCallback, this, std::placeholders::_1));
    }

    void ArmsTargetManager::refreshArmMarkersFromLatestCurrentTargets()
    {
        if (!server_)
        {
            return;
        }

        bool refreshed_any = false;

        if (left_arm_marker_ && shouldShowLeftArmMarker())
        {
            if (left_arm_marker_->refreshFromLatestCurrentTarget())
            {
                server_->setPose(left_arm_marker_->getMarkerName(), left_arm_marker_->getPose());
                refreshed_any = true;
            }
        }

        if (dual_arm_mode_ && right_arm_marker_ && shouldShowRightArmMarker())
        {
            if (right_arm_marker_->refreshFromLatestCurrentTarget())
            {
                server_->setPose(right_arm_marker_->getMarkerName(), right_arm_marker_->getPose());
                refreshed_any = true;
            }
        }

        if (refreshed_any)
        {
            markPendingChanges();
        }
    }

    void ArmsTargetManager::fsmCommandCallback(std_msgs::msg::Int32::ConstSharedPtr msg)
    {
        int32_t command = msg->data;

        if (command == 0)
        {
            return;
        }

        std::string new_state;
        bool valid_transition = arms_controller_common::FSMStateTransitionValidator::validateTransition(
            current_fsm_state_, command, new_state);

        if (valid_transition)
        {
            current_fsm_state_ = new_state;
            current_controller_state_ = command;

            updateMarkerShape();
            updateMenuVisibility();
            updateBodyMarkerVisibility();
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

        if (!isStateDisabled(current_controller_state_))
        {
            return;
        }

        geometry_msgs::msg::Pose updated_pose = head_marker_->updateFromJointState(
            joint_msg, isStateDisabled(current_controller_state_));

        server_->setPose("head_target", updated_pose);
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
        bool was_pending = pending_changes_.exchange(true, std::memory_order_acq_rel);

        if (!was_pending && server_)
        {
            auto now = node_->now();
            auto time_since_last = (now - last_marker_update_time_).seconds();

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
        if (pending_changes_.exchange(false, std::memory_order_acq_rel))
        {
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
        if (sourceFrameId == targetFrameId)
        {
            return pose;
        }

        try
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = sourceFrameId;
            pose_stamped.header.stamp = rclcpp::Time(0);
            pose_stamped.pose = pose;

            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                targetFrameId, sourceFrameId, tf2::TimePointZero);

            geometry_msgs::msg::PoseStamped result_stamped;
            tf2::doTransform(pose_stamped, result_stamped, transform);
            return result_stamped.pose;
        }
        catch (const tf2::TransformException& ex)
        {
            return pose;
        }
    }

    void ArmsTargetManager::setMarkerPose(
        const std::string& armType,
        const geometry_msgs::msg::Point& position,
        const geometry_msgs::msg::Quaternion& orientation)
    {
        std::shared_ptr<ArmMarker> arm_marker = nullptr;
        bool should_show = false;

        if (armType == "left" && left_arm_marker_)
        {
            arm_marker = left_arm_marker_;
            should_show = shouldShowLeftArmMarker();
        }
        else if (armType == "right" && dual_arm_mode_ && right_arm_marker_)
        {
            arm_marker = right_arm_marker_;
            should_show = shouldShowRightArmMarker();
        }
        else
        {
            return;
        }

        if (!should_show)
        {
            return;
        }

        geometry_msgs::msg::Pose new_pose;
        new_pose.position = position;
        new_pose.orientation = orientation;
        arm_marker->setPose(new_pose);

        if (server_ && isStateDisabled(current_controller_state_))
        {
            server_->setPose(arm_marker->getMarkerName(), new_pose);
            markPendingChanges();
        }

        if (current_mode_ == MarkerState::CONTINUOUS)
        {
            arm_marker->publishTargetPose();
        }
    }

    void ArmsTargetManager::updateMarkerPoseIncremental(
        const std::string& armType,
        const std::array<double, 3>& positionDelta,
        const std::array<double, 3>& rpyDelta)
    {
        if (!isStateDisabled(current_controller_state_))
        {
            RCLCPP_DEBUG(node_->get_logger(), "🎮 Incremental update blocked - controller state %d is not disabled",
                         current_controller_state_);
            return;
        }

        std::shared_ptr<ArmMarker> arm_marker = nullptr;
        bool should_show = false;

        if (armType == "left" && left_arm_marker_)
        {
            arm_marker = left_arm_marker_;
            should_show = shouldShowLeftArmMarker();
        }
        else if (armType == "right" && dual_arm_mode_ && right_arm_marker_)
        {
            arm_marker = right_arm_marker_;
            should_show = shouldShowRightArmMarker();
        }
        else
        {
            return;
        }

        if (!should_show)
        {
            return;
        }

        geometry_msgs::msg::Pose current_pose = arm_marker->getPose();

        current_pose.position.x += positionDelta[0];
        current_pose.position.y += positionDelta[1];
        current_pose.position.z += positionDelta[2];

        if (std::abs(rpyDelta[0]) > 0.001 || std::abs(rpyDelta[1]) > 0.001 || std::abs(rpyDelta[2]) > 0.001)
        {
            Eigen::Quaterniond current_quat(
                current_pose.orientation.w,
                current_pose.orientation.x,
                current_pose.orientation.y,
                current_pose.orientation.z
            );

            Eigen::AngleAxisd rollAngle(rpyDelta[0], Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(rpyDelta[1], Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(rpyDelta[2], Eigen::Vector3d::UnitZ());

            Eigen::Quaterniond rotationIncrement = yawAngle * pitchAngle * rollAngle;

            current_quat = rotationIncrement * current_quat;
            current_quat.normalize();

            current_pose.orientation.w = current_quat.w();
            current_pose.orientation.x = current_quat.x();
            current_pose.orientation.y = current_quat.y();
            current_pose.orientation.z = current_quat.z();
        }

        arm_marker->setPose(current_pose);

        if (server_)
        {
            server_->setPose(arm_marker->getMarkerName(), current_pose);
            markPendingChanges();
        }

        if (current_mode_ == MarkerState::CONTINUOUS)
        {
            arm_marker->publishTargetPose();
        }
    }

    geometry_msgs::msg::Pose ArmsTargetManager::getMarkerPose(const std::string& armType) const
    {
        if (armType == "left" && left_arm_marker_ && shouldShowLeftArmMarker())
        {
            return left_arm_marker_->getPose();
        }
        if (armType == "right" && dual_arm_mode_ && right_arm_marker_ && shouldShowRightArmMarker())
        {
            return right_arm_marker_->getPose();
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
} // namespace arms_ros2_control::command
