#pragma once

#include <memory>
#include <string>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>

#include "arms_target_manager/MarkerFactory.h"

namespace arms_ros2_control::command
{

class BodyMarker
{
public:
    using UpdateCallback =
        std::function<void(const std::string& marker_name, const geometry_msgs::msg::Pose& pose)>;

    using StateCheckCallback =
        std::function<bool()>;

    using CurrentPoseCallback =
        std::function<void(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg)>;

    BodyMarker(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<MarkerFactory> marker_factory,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        const std::string& frame_id,
        const std::string& control_base_frame,
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_publisher,
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_stamped_publisher,
        const std::string& current_pose_topic,
        double publish_rate = 20.0,
        UpdateCallback update_callback = nullptr);

    visualization_msgs::msg::InteractiveMarker createMarker(
        const std::string& name,
        MarkerState mode,
        bool enable_interaction) const;

    geometry_msgs::msg::Pose handleFeedback(
        const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback,
        const std::string& source_frame_id) const;

    void updateFromTopic(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg);

    void setUpdateCallback(UpdateCallback callback) { update_callback_ = std::move(callback); }
    void setStateCheckCallback(StateCheckCallback callback) { state_check_callback_ = std::move(callback); }
    void setCurrentPoseCallback(CurrentPoseCallback callback) { current_pose_callback_ = std::move(callback); }

    void setPose(const geometry_msgs::msg::Pose& pose) { pose_ = pose; }
    geometry_msgs::msg::Pose getPose() const { return pose_; }

    bool publishTargetPose(bool force = false);

    std::string getMarkerName() const { return "body_target"; }
    std::string getDescription() const { return "Body Target"; }

private:
    geometry_msgs::msg::Pose transformPose(
        const geometry_msgs::msg::Pose& pose,
        const std::string& source_frame_id,
        const std::string& target_frame_id) const;

    bool shouldThrottle(double interval) const;

    static void add6DofControls(
        visualization_msgs::msg::InteractiveMarker& int_marker,
        bool enable_interaction);

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<MarkerFactory> marker_factory_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    std::string frame_id_;
    std::string control_base_frame_;

    geometry_msgs::msg::Pose pose_;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_stamped_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_subscription_;

    double publish_rate_;
    mutable rclcpp::Time last_publish_time_;
    mutable rclcpp::Time last_subscription_update_time_;

    UpdateCallback update_callback_;
    StateCheckCallback state_check_callback_;
    CurrentPoseCallback current_pose_callback_;
};

} // namespace arms_ros2_control::command
