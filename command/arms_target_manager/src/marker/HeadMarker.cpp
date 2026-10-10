//
// HeadMarker - 头部 Marker 管理类实现
//

#include "arms_target_manager/marker/HeadMarker.h"
#include "arms_target_manager/MarkerFactory.h"
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.hpp>

namespace arms_ros2_control::command
{
    HeadMarker::HeadMarker(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<MarkerFactory> marker_factory,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        const std::string& frame_id,
        const std::string& control_base_frame,
        double publish_rate,
        UpdateCallback update_callback)
        : node_(std::move(node))
          , marker_factory_(std::move(marker_factory))
          , tf_buffer_(std::move(tf_buffer))
          , frame_id_(frame_id)
          , control_base_frame_(control_base_frame)
          , publish_rate_(publish_rate)
          , update_callback_(std::move(update_callback))
          , last_publish_time_(node_->now())
          , last_marker_command_time_(rclcpp::Time(0, 0, node_->get_clock()->get_clock_type()))
    {
        head_pose_.position.x = 1.0;
        head_pose_.position.y = 0.0;
        head_pose_.position.z = 1.5;
        head_pose_.orientation.w = 1.0;
        head_pose_.orientation.x = 0.0;
        head_pose_.orientation.y = 0.0;
        head_pose_.orientation.z = 0.0;
    }

    void HeadMarker::initialize()
    {
        node_->declare_parameter<bool>("enable_wbc_head_tracking_marker", false);
        enable_wbc_head_tracking_marker_ =
            node_->get_parameter("enable_wbc_head_tracking_marker").as_bool();

        if (!isEnabled())
        {
            return;
        }

        node_->declare_parameter<std::string>("head_link_name", "head_link2");
        head_link_name_ = node_->get_parameter("head_link_name").as_string();
        RCLCPP_INFO(node_->get_logger(),
                    "头部marker所在的link名称: %s", head_link_name_.c_str());

        pose_publisher_ = node_->create_publisher<geometry_msgs::msg::Pose>("head_target", 10);
        pose_stamped_publisher_ =
            node_->create_publisher<geometry_msgs::msg::PoseStamped>("head_target/stamped", 10);
        current_target_subscription_ =
            node_->create_subscription<geometry_msgs::msg::PoseStamped>(
                "head_current_target", 10,
                [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
                {
                    updateWbcTargetFromTopic(msg);
                });
    }

    visualization_msgs::msg::InteractiveMarker HeadMarker::createMarker(
        const std::string& name,
        const geometry_msgs::msg::Pose& pose,
        bool enable_interaction,
        MarkerState mode) const
    {
        return marker_factory_->createHeadMarker(name, pose, enable_interaction, mode);
    }

    bool HeadMarker::shouldThrottle(double interval) const
    {
        auto now = node_->now();
        auto time_since_last = (now - last_publish_time_).seconds();

        if (time_since_last >= interval)
        {
            last_publish_time_ = now;
            return true;
        }
        return false;
    }

    bool HeadMarker::transformPose(
        const geometry_msgs::msg::Pose& pose,
        const std::string& source_frame,
        const std::string& target_frame,
        geometry_msgs::msg::Pose& result) const
    {
        if (source_frame == target_frame)
        {
            result = pose;
            return true;
        }
        try
        {
            geometry_msgs::msg::PoseStamped source;
            source.header.frame_id = source_frame;
            source.header.stamp = rclcpp::Time(0);
            source.pose = pose;
            const auto transform = tf_buffer_->lookupTransform(
                target_frame, source_frame, tf2::TimePointZero);
            geometry_msgs::msg::PoseStamped transformed;
            tf2::doTransform(source, transformed, transform);
            result = transformed.pose;
            return true;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "Failed to transform WBC head target from '%s' to '%s': %s",
                source_frame.c_str(), target_frame.c_str(), ex.what());
            return false;
        }
    }

    bool HeadMarker::syncPoseFromTf()
    {
        if (!enable_wbc_head_tracking_marker_ || head_link_name_.empty())
        {
            return false;
        }
        try
        {
            const auto transform = tf_buffer_->lookupTransform(
                frame_id_, head_link_name_, tf2::TimePointZero);
            head_pose_.position.x = transform.transform.translation.x;
            head_pose_.position.y = transform.transform.translation.y;
            head_pose_.position.z = transform.transform.translation.z;
            head_pose_.orientation = transform.transform.rotation;
            return true;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "Cannot initialize WBC head marker from %s -> %s TF: %s",
                frame_id_.c_str(), head_link_name_.c_str(), ex.what());
            return false;
        }
    }

    bool HeadMarker::publishTargetPose(bool force, bool use_stamped)
    {
        if (!enable_wbc_head_tracking_marker_)
        {
            return false;
        }
        geometry_msgs::msg::Pose transformed;
        if (!transformPose(head_pose_, frame_id_, control_base_frame_, transformed))
        {
            return false;
        }
        if (force && use_stamped)
        {
            if (!pose_stamped_publisher_)
            {
                return false;
            }
            geometry_msgs::msg::PoseStamped msg;
            msg.header.stamp = node_->now();
            msg.header.frame_id = control_base_frame_;
            msg.pose = transformed;
            pose_stamped_publisher_->publish(msg);
        }
        else
        {
            if (!pose_publisher_ || (!force && !shouldThrottle(1.0 / publish_rate_)))
            {
                return false;
            }
            pose_publisher_->publish(transformed);
        }
        last_marker_command_time_ = node_->now();
        if (force)
        {
            last_publish_time_ = last_marker_command_time_;
        }
        return true;
    }

    void HeadMarker::updateWbcTargetFromTopic(
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
    {
        if (!msg)
        {
            return;
        }
        latest_current_target_ = *msg;
        has_latest_current_target_ = true;
        if (msg->header.frame_id.empty() ||
            (wbc_target_state_check_callback_ && !wbc_target_state_check_callback_()))
        {
            return;
        }
        const auto now = node_->now();
        if (last_marker_command_time_.nanoseconds() > 0 &&
            (now - last_marker_command_time_).seconds() < 1.0)
        {
            return;
        }
        geometry_msgs::msg::Pose transformed;
        if (!transformPose(msg->pose, msg->header.frame_id, frame_id_, transformed))
        {
            return;
        }
        head_pose_ = transformed;
        if (update_callback_)
        {
            update_callback_("head_target", head_pose_);
        }
    }

    void HeadMarker::clearCommandCooldown()
    {
        last_marker_command_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
    }
} // namespace arms_ros2_control::command
