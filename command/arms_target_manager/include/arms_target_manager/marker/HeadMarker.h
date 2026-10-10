//
// HeadMarker - 头部 Marker 管理类
//
#pragma once

#include <memory>
#include <string>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace arms_ros2_control::command
{
    class MarkerFactory;
    enum class MarkerState;

    /**
     * @brief WBC Head 6D Marker：发布 head_target / head_target/stamped，
     *        并从 head_current_target 回写最终目标。
     */
    class HeadMarker
    {
    public:
        using UpdateCallback =
            std::function<void(const std::string&, const geometry_msgs::msg::Pose&)>;
        using StateCheckCallback = std::function<bool()>;

        HeadMarker(
            rclcpp::Node::SharedPtr node,
            std::shared_ptr<MarkerFactory> marker_factory,
            std::shared_ptr<tf2_ros::Buffer> tf_buffer,
            const std::string& frame_id,
            const std::string& control_base_frame,
            double publish_rate = 20.0,
            UpdateCallback update_callback = nullptr);

        void initialize();

        bool isEnabled() const { return enable_wbc_head_tracking_marker_; }
        bool isWbcEnabled() const { return enable_wbc_head_tracking_marker_; }

        visualization_msgs::msg::InteractiveMarker createMarker(
            const std::string& name,
            const geometry_msgs::msg::Pose& pose,
            bool enable_interaction,
            MarkerState mode) const;

        geometry_msgs::msg::Pose getPose() const { return head_pose_; }
        void setPose(const geometry_msgs::msg::Pose& pose) { head_pose_ = pose; }

        bool publishTargetPose(bool force = false, bool use_stamped = false);

        /** Initialize the full marker pose from marker_fixed_frame -> head link TF. */
        bool syncPoseFromTf();
        void setWbcTargetStateCheckCallback(StateCheckCallback callback)
        {
            wbc_target_state_check_callback_ = std::move(callback);
        }
        void clearCommandCooldown();

        std::string getLinkName() const { return head_link_name_; }

    private:
        bool shouldThrottle(double interval) const;
        bool transformPose(const geometry_msgs::msg::Pose& pose,
                           const std::string& source_frame,
                           const std::string& target_frame,
                           geometry_msgs::msg::Pose& result) const;
        void updateWbcTargetFromTopic(
            const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);

        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<MarkerFactory> marker_factory_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::string frame_id_;
        std::string control_base_frame_;

        bool enable_wbc_head_tracking_marker_ = false;
        std::string head_link_name_;

        geometry_msgs::msg::Pose head_pose_;

        double publish_rate_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_publisher_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_target_subscription_;
        geometry_msgs::msg::PoseStamped latest_current_target_;
        bool has_latest_current_target_{false};
        UpdateCallback update_callback_;
        StateCheckCallback wbc_target_state_check_callback_;

        mutable rclcpp::Time last_publish_time_;
        mutable rclcpp::Time last_marker_command_time_;
    };
} // namespace arms_ros2_control::command
