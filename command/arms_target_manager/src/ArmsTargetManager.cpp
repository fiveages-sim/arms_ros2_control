//
// ArmsTargetManager 实现。
//
// 记忆点：
//   * marker 只显示，目标一律走 router_；
//   * 拖动 → Intent（SourceId::Rviz, 高优先级），松手后由租约自动交还；
//   * 单次发送模式下拖动只更新预览，菜单“发送目标”才真正下发。
//
#include "arms_target_manager/ArmsTargetManager.h"
#include "arms_target_manager/marker/BodyMarker.h"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.hpp>
#include <std_msgs/msg/header.hpp>
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

    bool ArmsTargetManager::isBimanualCoupled() const
    {
        return dual_arm_mode_ &&
            left_arm_marker_ &&
            right_arm_marker_ &&
            shouldShowLeftArmMarker() &&
            shouldShowRightArmMarker() &&
            (bimanual_state_ == arms_ros2_control_msgs::msg::WbcCurrentState::BIMANUAL_COUPLED);
    }

    bool ArmsTargetManager::shouldStreamPoseCommands() const
    {
        return current_mode_ == MarkerState::CONTINUOUS && current_controller_state_ != 4;
    }

    void ArmsTargetManager::followCoupledRightMarker(
        const geometry_msgs::msg::Pose& old_left_pose,
        const geometry_msgs::msg::Pose& old_right_pose,
        const geometry_msgs::msg::Pose& new_left_pose)
    {
        if (!right_arm_marker_)
        {
            return;
        }

        const geometry_msgs::msg::Pose new_right_pose =
            applyRelativePose(old_left_pose, old_right_pose, new_left_pose);
        right_arm_marker_->setPose(new_right_pose);
        if (server_ && isStateDisabled(current_controller_state_))
        {
            setServerPose(right_arm_marker_->getMarkerName(), new_right_pose);
            markPendingChanges();
        }
    }

    void ArmsTargetManager::followCoupledLeftMarker(
        const geometry_msgs::msg::Pose& old_right_pose,
        const geometry_msgs::msg::Pose& old_left_pose,
        const geometry_msgs::msg::Pose& new_right_pose)
    {
        if (!left_arm_marker_)
        {
            return;
        }

        const geometry_msgs::msg::Pose new_left_pose =
            applyRelativePose(old_right_pose, old_left_pose, new_right_pose);
        left_arm_marker_->setPose(new_left_pose);
        if (server_ && isStateDisabled(current_controller_state_))
        {
            setServerPose(left_arm_marker_->getMarkerName(), new_left_pose);
            markPendingChanges();
        }
    }

    bool ArmsTargetManager::shouldShowBodyMarker() const
    {
        return (current_controller_state_ == 3) && (body_state_ == 2) &&
            head_state_ != arms_ros2_control_msgs::msg::WbcCurrentState::HEAD_FORWARD;
    }

    bool ArmsTargetManager::shouldShowWbcHeadMarker() const
    {
        return head_marker_ && head_marker_->isWbcEnabled() &&
            current_controller_state_ == 3 &&
            head_state_ == arms_ros2_control_msgs::msg::WbcCurrentState::HEAD_TRACKING;
    }

    bool ArmsTargetManager::shouldShowHeadMarker() const
    {
        return shouldShowWbcHeadMarker();
    }

    void ArmsTargetManager::setHeadVrActive(bool active)
    {
        if (head_vr_active_ == active)
        {
            return;
        }
        head_vr_active_ = active;
        if (router_)
        {
            // VR 接管后不再受上一次手动拖动留下的回显抑制影响。
            router_->clearEchoHold(Channel::Head);
        }
        updateHeadMarkerVisibility();
    }

    void ArmsTargetManager::setArmVrActive(bool is_left, bool active)
    {
        bool& flag = is_left ? left_arm_vr_active_ : right_arm_vr_active_;
        if (flag == active)
        {
            return;
        }
        flag = active;
        if (router_)
        {
            // 只挡摇杆/键盘的速度流；RViz 拖动仍然可用（它是人工覆盖）。
            router_->setBlocked(
                is_left ? Channel::ArmLeft : Channel::ArmRight,
                SourceId::ControlInput, active);
        }
    }

    void ArmsTargetManager::updateHeadMarkerVisibility()
    {
        if (!server_ || !head_marker_)
        {
            return;
        }

        const bool show_wbc = shouldShowWbcHeadMarker();
        if (!show_wbc)
        {
            server_->erase("head_target");
            wbc_head_marker_visible_ = false;
            markPendingChanges();
            return;
        }

        if (show_wbc && !wbc_head_marker_visible_)
        {
            if (!head_marker_->syncPoseFromTf())
            {
                server_->erase("head_target");
                markPendingChanges();
                return;
            }
            head_marker_->clearCommandCooldown();
        }

        auto marker = buildMarker("head_target", "head");
        server_->insert(marker);
        server_->setCallback(
            marker.name,
            [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
            {
                handleMarkerFeedback(feedback);
            });
        if (head_menu_handler_)
        {
            head_menu_handler_->apply(*server_, marker.name);
        }
        wbc_head_marker_visible_ = show_wbc;
        markPendingChanges();
    }

    int ArmsTargetManager::getCurrentBodyState() const
    {
        return body_state_;
    }

    int ArmsTargetManager::getCurrentBaseState() const
    {
        return base_state_;
    }

    int ArmsTargetManager::getCurrentBimanualState() const
    {
        return bimanual_state_;
    }

    int ArmsTargetManager::getCurrentLeftArmState() const
    {
        return left_arm_state_;
    }

    int ArmsTargetManager::getCurrentRightArmState() const
    {
        return right_arm_state_;
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
        const int prev_head_state = head_state_;

        left_arm_state_ = msg->left_arm_state;
        right_arm_state_ = msg->right_arm_state;
        bimanual_state_ = msg->bimanual_state;
        body_state_ = msg->body_state;
        head_state_ = msg->head_state;
        base_state_ = msg->base_state;

        if (wbc_state_callback_)
        {
            wbc_state_callback_(msg);
        }

        const bool bimanual_state_changed = (prev_bimanual_state != bimanual_state_);

        if (bimanual_state_changed)
        {
            if (left_arm_marker_)
            {
                left_arm_marker_->clearCommandCooldown();
            }
            if (right_arm_marker_)
            {
                right_arm_marker_->clearCommandCooldown();
            }
            // Prefer measured EE: during coupling the right arm tracks the constraint,
            // not a possibly stale right_current_target / marker pose.
            refreshArmMarkersFromLatestCurrentPoses();
        }

        if (prev_left_arm_state != left_arm_state_ ||
            prev_right_arm_state != right_arm_state_ ||
            prev_body_state != body_state_ || prev_head_state != head_state_)
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
        // Feed Buffer from relative topics so __ns alone reaches /<ns>/tf (Isaac dual).
        // TransformListener hard-binds absolute /tf and misses namespaced trees.
        std::string tf_topic = "tf";
        std::string tf_static_topic = "tf_static";
        if (!node_->has_parameter("tf_topic")) {
            node_->declare_parameter("tf_topic", tf_topic);
        }
        if (!node_->has_parameter("tf_static_topic")) {
            node_->declare_parameter("tf_static_topic", tf_static_topic);
        }
        tf_topic = node_->get_parameter("tf_topic").as_string();
        tf_static_topic = node_->get_parameter("tf_static_topic").as_string();
        tf_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
            tf_topic,
            rclcpp::QoS(rclcpp::KeepLast(100)).best_effort().durability_volatile(),
            [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
                for (const auto& t : msg->transforms) {
                    tf_buffer_->setTransform(t, "isaac_tf_authority", false);
                }
            });
        tf_static_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
            tf_static_topic,
            rclcpp::QoS(rclcpp::KeepLast(100)).transient_local().reliable(),
            [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
                for (const auto& t : msg->transforms) {
                    tf_buffer_->setTransform(t, "isaac_tf_authority", true);
                }
            });
        RCLCPP_INFO(node_->get_logger(),
                    "ATM TF buffer on relative '%s'+'%s' (ns=%s)",
                    tf_topic.c_str(), tf_static_topic.c_str(),
                    node_->get_namespace());

        // 目标汇聚器：只有它会发布 left_target / right_target / head_target / body_target。
        TargetRouter::Config router_config;
        router_config.control_frame = control_base_frame_;
        router_config.marker_frame = marker_fixed_frame_;
        router_config.dual_arm = dual_arm_mode_;
        router_config.publish_rate = publish_rate_;
        router_ = std::make_unique<TargetRouter>(node_, tf_buffer_, router_config);
        router_->start(pub_left_target, pub_left_target_stamped,
                       pub_right_target, pub_right_target_stamped);

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

                setServerPose(marker_name, pose);
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

                    setServerPose(marker_name, pose);
                    markPendingChanges();
                });

            right_arm_marker_->setStateCheckCallback(
                [this]() { return !isStateDisabled(current_controller_state_); });
        }

        head_marker_ = std::make_shared<HeadMarker>(
            node_, marker_factory_, tf_buffer_, marker_fixed_frame_, control_base_frame_, publish_rate_,
            [this](const std::string& marker_name, const geometry_msgs::msg::Pose& pose)
            {
                if (!server_ || !shouldShowWbcHeadMarker())
                {
                    return;
                }
                setServerPose(marker_name, pose);
                markPendingChanges();
            });
        head_marker_->initialize();
        head_marker_->setWbcTargetStateCheckCallback(
            [this]() { return shouldShowWbcHeadMarker(); });

        body_marker_ = std::make_shared<BodyMarker>(
            node_,
            marker_factory_,
            tf_buffer_,
            marker_fixed_frame_,
            control_base_frame_,
            nullptr,
            nullptr,
            "body_current_pose",
            "body_current_target",
            publish_rate_,
            [this](const std::string& marker_name,
                   const geometry_msgs::msg::Pose& pose)
            {
                if (!server_ || !shouldShowBodyMarker())
                {
                    return;
                }

                setServerPose(marker_name, pose);
                markPendingChanges();
            });

        body_marker_->setStateCheckCallback(
            [this]() { return !shouldShowBodyMarker(); });

        body_marker_->setTargetStateCheckCallback(
            [this]() { return shouldShowBodyMarker(); });

        // 显示层只负责“把目标画出来”，发布与回显规则都在 router_ 里。
        router_->setView(
            Channel::ArmLeft,
            [this](const geometry_msgs::msg::Pose& pose)
            {
                if (!left_arm_marker_)
                {
                    return;
                }
                left_arm_marker_->setPose(pose);
                if (server_ && shouldShowLeftArmMarker())
                {
                    setServerPose(left_arm_marker_->getMarkerName(), pose);
                    markPendingChanges();
                }
            });
        router_->setView(
            Channel::ArmRight,
            [this](const geometry_msgs::msg::Pose& pose)
            {
                if (!right_arm_marker_)
                {
                    return;
                }
                right_arm_marker_->setPose(pose);
                if (server_ && shouldShowRightArmMarker())
                {
                    setServerPose(right_arm_marker_->getMarkerName(), pose);
                    markPendingChanges();
                }
            });
        router_->setView(
            Channel::Head,
            [this](const geometry_msgs::msg::Pose& pose)
            {
                if (!head_marker_)
                {
                    return;
                }
                head_marker_->setPose(pose);
                if (server_ && shouldShowWbcHeadMarker())
                {
                    setServerPose("head_target", pose);
                    markPendingChanges();
                }
            });
        router_->setView(
            Channel::Body,
            [this](const geometry_msgs::msg::Pose& pose)
            {
                if (!body_marker_)
                {
                    return;
                }
                body_marker_->setPose(pose);
                if (server_ && shouldShowBodyMarker())
                {
                    setServerPose(body_marker_->getMarkerName(), pose);
                    markPendingChanges();
                }
            });

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
            if (shouldShowHeadMarker())
            {
                auto marker = head_marker_->createMarker(
                    name, head_marker_->getPose(), enable_interaction, current_mode_);
                if (head_vr_active_)
                {
                    // Keep the visible arrow; the factory's disabled mode hides it too.
                    marker.controls.erase(std::remove_if(marker.controls.begin(), marker.controls.end(),
                        [](const auto& control) {
                            return control.interaction_mode !=
                                visualization_msgs::msg::InteractiveMarkerControl::NONE;
                        }), marker.controls.end());
                }
                return marker;
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
        if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::KEEP_ALIVE ||
            feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT ||
            feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::BUTTON_CLICK)
        {
            return;
        }

        const std::string source_frame_id = feedback->header.frame_id;
        const std::string marker_name = feedback->marker_name;

        // 拖动一次提交一次目标；控制权在 500ms 内保持，松手后自动交还给原来的控制来源。
        auto submit_drag = [this](Channel channel, const geometry_msgs::msg::Pose& pose,
                                  bool publish)
        {
            Intent intent;
            intent.channel = channel;
            intent.source = SourceId::Rviz;
            intent.priority = Priority::High;
            intent.position = Eigen::Vector3d(
                pose.position.x, pose.position.y, pose.position.z);
            intent.orientation = Eigen::Quaterniond(
                pose.orientation.w, pose.orientation.x,
                pose.orientation.y, pose.orientation.z);
            intent.frame = marker_fixed_frame_;
            intent.lease = std::chrono::milliseconds(500);
            intent.publish = publish;
            return router_->submit(intent);
        };

        if (marker_name == "left_arm_target" && left_arm_marker_)
        {
            if (!shouldShowLeftArmMarker())
            {
                return;
            }

            const geometry_msgs::msg::Pose new_pose =
                left_arm_marker_->handleFeedback(feedback, source_frame_id);

            if (!shouldStreamPoseCommands())
            {
                // 非连续模式：只更新预览目标，菜单“发送目标”时再真正下发。
                submit_drag(Channel::ArmLeft, new_pose, false);
                return;
            }

            const geometry_msgs::msg::Pose old_left_pose = left_arm_marker_->getPose();
            geometry_msgs::msg::Pose old_right_pose;
            const bool can_follow_right = isBimanualCoupled();
            if (can_follow_right && right_arm_marker_)
            {
                old_right_pose = right_arm_marker_->getPose();
            }

            const Verdict verdict = submit_drag(Channel::ArmLeft, new_pose, true);
            if (!verdict.accepted)
            {
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 2000,
                    "左臂拖动未生效：%s", verdict.reason);
                return;
            }

            if (can_follow_right)
            {
                followCoupledRightMarker(old_left_pose, old_right_pose, new_pose);
            }
        }
        else if (marker_name == "right_arm_target" && right_arm_marker_)
        {
            if (!shouldShowRightArmMarker())
            {
                return;
            }

            const geometry_msgs::msg::Pose new_pose =
                right_arm_marker_->handleFeedback(feedback, source_frame_id);

            if (!shouldStreamPoseCommands())
            {
                submit_drag(Channel::ArmRight, new_pose, false);
                return;
            }

            const geometry_msgs::msg::Pose old_right_pose = right_arm_marker_->getPose();
            geometry_msgs::msg::Pose old_left_pose;
            const bool can_follow_left = isBimanualCoupled();
            if (can_follow_left && left_arm_marker_)
            {
                old_left_pose = left_arm_marker_->getPose();
            }

            const Verdict verdict = submit_drag(Channel::ArmRight, new_pose, true);
            if (!verdict.accepted)
            {
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 2000,
                    "右臂拖动未生效：%s", verdict.reason);
                return;
            }

            if (can_follow_left)
            {
                followCoupledLeftMarker(old_right_pose, old_left_pose, new_pose);
            }
        }
        else if (marker_name == "head_target")
        {
            if (!shouldShowWbcHeadMarker())
            {
                return;
            }

            // 头部与手臂一致：六自由度，位置与姿态都可以拖动。
            const geometry_msgs::msg::Pose new_pose =
                transformPose(feedback->pose, source_frame_id, marker_fixed_frame_);
            if (!shouldStreamPoseCommands())
            {
                submit_drag(Channel::Head, new_pose, false);
                return;
            }

            const Verdict verdict = submit_drag(Channel::Head, new_pose, true);
            if (!verdict.accepted)
            {
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 2000,
                    "头部拖动未生效：%s", verdict.reason);
            }
        }
        else if (marker_name == "body_target" && body_marker_)
        {
            if (!shouldShowBodyMarker())
            {
                return;
            }

            const geometry_msgs::msg::Pose new_pose =
                body_marker_->handleFeedback(feedback, source_frame_id);

            if (!shouldStreamPoseCommands())
            {
                submit_drag(Channel::Body, new_pose, false);
                return;
            }

            const Verdict verdict = submit_drag(Channel::Body, new_pose, true);
            if (!verdict.accepted)
            {
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 2000,
                    "胸部拖动未生效：%s", verdict.reason);
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
        if (current_controller_state_ == 4)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Continuous publish is not available in MOVEJ; use 发送目标 for a one-shot IK MoveL");
            return;
        }

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
            if (shouldShowWbcHeadMarker())
            {
                router_->publishNow(Channel::Head, true);
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
            router_->publishNow(Channel::ArmLeft, true);
            return;
        }

        if (marker_type == "right_arm" && right_arm_marker_ && shouldShowRightArmMarker())
        {
            router_->publishNow(Channel::ArmRight, true);
            return;
        }
    }

    void ArmsTargetManager::sendDualArmTargetPose()
    {
        if (!dual_arm_mode_)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Cannot send dual arm target pose: dual arm mode not enabled");
            return;
        }

        if (!shouldShowLeftArmMarker() || !shouldShowRightArmMarker())
        {
            RCLCPP_WARN(node_->get_logger(), "Cannot send dual arm target pose: one or both arm markers are hidden");
            return;
        }

        // 左/右臂目标已由 router_ 缓存并换算到控制坐标系，这里只负责合成一条 Path。
        const bool include_body = (body_marker_ && shouldShowBodyMarker());
        if (!router_->publishDual(include_body))
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Cannot send dual arm target pose: target not ready");
        }
    }

    void ArmsTargetManager::sendBodyTargetPose()
    {
        if (!body_marker_ || !shouldShowBodyMarker())
        {
            RCLCPP_WARN(node_->get_logger(), "Cannot send body target pose: body marker not available");
            return;
        }

        router_->publishNow(Channel::Body, true);
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

        updateHeadMarkerVisibility();

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

        if (shouldShowHeadMarker())
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

            if (shouldShowHeadMarker())
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

            if (shouldShowHeadMarker())
            {
                head_menu_handler_->setVisible(head_send_handle_, true);
            }

            if (body_marker_ && shouldShowBodyMarker())
            {
                body_menu_handler_->setVisible(body_send_handle_, true);
            }
        }

        const bool show_continuous_toggle = current_controller_state_ != 4;
        if (left_arm_marker_ && shouldShowLeftArmMarker())
        {
            left_menu_handler_->setVisible(left_toggle_handle_, show_continuous_toggle);
        }
        if (dual_arm_mode_ && right_arm_marker_ && shouldShowRightArmMarker())
        {
            right_menu_handler_->setVisible(right_toggle_handle_, show_continuous_toggle);
        }
        if (shouldShowHeadMarker())
        {
            head_menu_handler_->setVisible(head_toggle_handle_, show_continuous_toggle);
        }
        if (body_marker_ && shouldShowBodyMarker())
        {
            body_menu_handler_->setVisible(body_toggle_handle_, show_continuous_toggle);
        }

        if (left_arm_marker_ && shouldShowLeftArmMarker())
        {
            left_menu_handler_->reApply(*server_);
        }
        if (dual_arm_mode_ && right_arm_marker_ && shouldShowRightArmMarker())
        {
            right_menu_handler_->reApply(*server_);
        }

        if (shouldShowHeadMarker())
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
        wbc_state_subscriber_ =
            node_->create_subscription<arms_ros2_control_msgs::msg::WbcCurrentState>(
                "ocs2_wbc_controller/current_state",
                10,
                std::bind(&ArmsTargetManager::wbcStateCallback, this, std::placeholders::_1));

        if (!node_->has_parameter("body_controller_name"))
        {
            node_->declare_parameter("body_controller_name", body_controller_name_);
        }
        body_controller_name_ = node_->get_parameter("body_controller_name").as_string();

        const std::string body_joint_target_topic =
            body_controller_name_ + "/current_target_joint";
        body_joint_target_subscriber_ =
            node_->create_subscription<std_msgs::msg::Float64MultiArray>(
                body_joint_target_topic, 10,
                std::bind(&ArmsTargetManager::currentTargetJointCallback, this, std::placeholders::_1));

        RCLCPP_INFO(node_->get_logger(),
                    "Subscribed to body joint target topic: %s (waist teleop marker sync trigger)",
                    body_joint_target_topic.c_str());
    }

    bool ArmsTargetManager::bodyJointTargetsChanged(
        const std::vector<double>& previous,
        const std::vector<double>& current,
        const double threshold)
    {
        if (previous.empty())
        {
            return false;
        }

        if (previous.size() != current.size())
        {
            return !current.empty();
        }

        for (size_t i = 0; i < current.size(); ++i)
        {
            if (std::abs(current[i] - previous[i]) > threshold)
            {
                return true;
            }
        }
        return false;
    }

    void ArmsTargetManager::currentTargetJointCallback(
        const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg)
    {
        if (!msg || !isStateDisabled(current_controller_state_))
        {
            return;
        }

        constexpr double kJointChangeThreshold = 1e-4;

        if (!bodyJointTargetsChanged(last_body_joint_targets_, msg->data, kJointChangeThreshold))
        {
            if (last_body_joint_targets_.empty() && !msg->data.empty())
            {
                last_body_joint_targets_ = msg->data;
            }
            return;
        }

        last_body_joint_targets_ = msg->data;
        refreshArmMarkersFromLatestCurrentPoses();
    }

    void ArmsTargetManager::refreshArmMarkersFromLatestCurrentPoses()
    {
        if (!server_)
        {
            return;
        }

        bool refreshed_any = false;

        if (left_arm_marker_ && shouldShowLeftArmMarker())
        {
            if (left_arm_marker_->refreshFromLatestCurrentPose())
            {
                setServerPose(left_arm_marker_->getMarkerName(), left_arm_marker_->getPose());
                refreshed_any = true;
            }
        }

        if (dual_arm_mode_ && right_arm_marker_ && shouldShowRightArmMarker())
        {
            if (right_arm_marker_->refreshFromLatestCurrentPose())
            {
                setServerPose(right_arm_marker_->getMarkerName(), right_arm_marker_->getPose());
                refreshed_any = true;
            }
        }

        if (refreshed_any)
        {
            markPendingChanges();
        }
    }

    void ArmsTargetManager::refreshArmMarkersFromLatestCurrentTargets()
    {
        if (!server_ || !router_)
        {
            return;
        }

        bool refreshed_any = false;
        if (shouldShowLeftArmMarker())
        {
            refreshed_any |= router_->refreshViewFromLastCommand(Channel::ArmLeft);
        }
        if (dual_arm_mode_ && shouldShowRightArmMarker())
        {
            refreshed_any |= router_->refreshViewFromLastCommand(Channel::ArmRight);
        }

        if (refreshed_any)
        {
            markPendingChanges();
        }
    }

    bool ArmsTargetManager::canAcceptSpeedCommand(bool is_left) const
    {
        if (!router_)
        {
            return true;
        }
        return router_->canWrite(
            is_left ? Channel::ArmLeft : Channel::ArmRight,
            SourceId::ControlInput);
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

            if (command == 4 && current_mode_ == MarkerState::CONTINUOUS)
            {
                current_mode_ = MarkerState::SINGLE_SHOT;
            }

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

    void ArmsTargetManager::setWbcStateCallback(
        std::function<void(
            const arms_ros2_control_msgs::msg::WbcCurrentState::ConstSharedPtr&)>
            callback)
    {
        wbc_state_callback_ = std::move(callback);
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
        if (shouldShowWbcHeadMarker() && !wbc_head_marker_visible_)
        {
            updateHeadMarkerVisibility();
        }
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
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "Failed to transform pose from '%s' to '%s': %s (using untransformed pose)",
                sourceFrameId.c_str(), targetFrameId.c_str(), ex.what());
            return pose;
        }
    }

    void ArmsTargetManager::setServerPose(
        const std::string& name,
        const geometry_msgs::msg::Pose& pose)
    {
        if (!server_)
        {
            return;
        }

        std_msgs::msg::Header header;
        header.frame_id = marker_fixed_frame_;
        header.stamp.sec = 0;
        header.stamp.nanosec = 0;
        server_->setPose(name, pose, header);
    }

    void ArmsTargetManager::setMarkerPose(
        const std::string& armType,
        const geometry_msgs::msg::Point& position,
        const geometry_msgs::msg::Quaternion& orientation)
    {
        const bool is_left = armType == "left";
        if (!is_left && armType != "right")
        {
            return;
        }

        std::shared_ptr<ArmMarker> arm_marker = is_left ? left_arm_marker_ : right_arm_marker_;
        const bool should_show = is_left ? shouldShowLeftArmMarker() : shouldShowRightArmMarker();
        if (!arm_marker || !should_show)
        {
            return;
        }

        Intent intent;
        intent.channel = is_left ? Channel::ArmLeft : Channel::ArmRight;
        intent.source = SourceId::External;
        intent.priority = Priority::Normal;
        intent.position = Eigen::Vector3d(position.x, position.y, position.z);
        intent.orientation = Eigen::Quaterniond(
            orientation.w, orientation.x, orientation.y, orientation.z);
        intent.frame = marker_fixed_frame_;
        intent.lease = std::chrono::milliseconds(500);
        intent.publish = shouldStreamPoseCommands();
        router_->submit(intent);
    }

    void ArmsTargetManager::updateMarkerPoseIncremental(
        const std::string& armType,
        const std::array<double, 3>& positionDelta,
        const std::array<double, 3>& rpyDelta,
        bool publish)
    {
        if (!isStateDisabled(current_controller_state_))
        {
            RCLCPP_DEBUG(node_->get_logger(), "🎮 Incremental update blocked - controller state %d is not disabled",
                         current_controller_state_);
            return;
        }

        const bool is_left = armType == "left";
        if (!is_left && armType != "right")
        {
            return;
        }

        std::shared_ptr<ArmMarker> arm_marker = is_left ? left_arm_marker_ : right_arm_marker_;
        const bool should_show = is_left ? shouldShowLeftArmMarker() : shouldShowRightArmMarker();
        if (!arm_marker || !should_show)
        {
            return;
        }

        const geometry_msgs::msg::Pose old_pose = arm_marker->getPose();
        geometry_msgs::msg::Pose old_opposite_pose;
        const bool coupled = isBimanualCoupled();
        if (coupled)
        {
            old_opposite_pose = is_left ? right_arm_marker_->getPose()
                                        : left_arm_marker_->getPose();
        }

        geometry_msgs::msg::Pose current_pose = old_pose;
        current_pose.position.x += positionDelta[0];
        current_pose.position.y += positionDelta[1];
        current_pose.position.z += positionDelta[2];

        if (std::abs(rpyDelta[0]) > 0.001 || std::abs(rpyDelta[1]) > 0.001 ||
            std::abs(rpyDelta[2]) > 0.001)
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

        Intent intent;
        intent.channel = is_left ? Channel::ArmLeft : Channel::ArmRight;
        intent.source = SourceId::External;
        intent.priority = Priority::Normal;
        intent.position = Eigen::Vector3d(
            current_pose.position.x, current_pose.position.y, current_pose.position.z);
        intent.orientation = Eigen::Quaterniond(
            current_pose.orientation.w, current_pose.orientation.x,
            current_pose.orientation.y, current_pose.orientation.z);
        intent.frame = marker_fixed_frame_;
        intent.lease = std::chrono::milliseconds(500);
        intent.publish = publish && shouldStreamPoseCommands();
        if (!router_->submit(intent).accepted)
        {
            return;
        }

        if (coupled && is_left)
        {
            followCoupledRightMarker(old_pose, old_opposite_pose, current_pose);
        }
        else if (coupled && !is_left)
        {
            followCoupledLeftMarker(old_pose, old_opposite_pose, current_pose);
        }
    }

    bool ArmsTargetManager::updateBodyMarkerPoseIncremental(
        const std::array<double, 3>& positionDelta,
        const std::array<double, 3>& rpyDelta)
    {
        if (!body_marker_ || !shouldShowBodyMarker())
        {
            return false;
        }

        geometry_msgs::msg::Pose current_pose = body_marker_->getPose();

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

        Intent intent;
        intent.channel = Channel::Body;
        intent.source = SourceId::Vr;
        intent.priority = Priority::Normal;
        intent.position = Eigen::Vector3d(
            current_pose.position.x, current_pose.position.y, current_pose.position.z);
        intent.orientation = Eigen::Quaterniond(
            current_pose.orientation.w, current_pose.orientation.x,
            current_pose.orientation.y, current_pose.orientation.z);
        intent.frame = marker_fixed_frame_;
        intent.lease = std::chrono::milliseconds(500);
        intent.publish = shouldStreamPoseCommands();
        return router_->submit(intent).accepted;
    }

    bool ArmsTargetManager::setBodyMarkerPose(const geometry_msgs::msg::Pose& pose)
    {
        if (!body_marker_ || !shouldShowBodyMarker())
        {
            return false;
        }

        // pose 由调用方给出，表达在 marker 固定系（marker_fixed_frame_，通常是 world）。
        // 发布 /body_target 时由 router_ 统一换算到 control_base_frame_
        // （下游 OCS2 模型的 base_frame_），两者不能混用。
        Intent intent;
        intent.channel = Channel::Body;
        intent.source = SourceId::Vr;
        intent.priority = Priority::Normal;
        intent.position = Eigen::Vector3d(
            pose.position.x, pose.position.y, pose.position.z);
        intent.orientation = Eigen::Quaterniond(
            pose.orientation.w, pose.orientation.x,
            pose.orientation.y, pose.orientation.z);
        intent.frame = marker_fixed_frame_;
        intent.lease = std::chrono::milliseconds(500);
        intent.publish = shouldStreamPoseCommands();
        return router_->submit(intent).accepted;
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
