//
// Created for Arms ROS2 Control - ArmsTargetManager
//
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <array>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <arms_ros2_control_msgs/msg/inputs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <arms_ros2_control_msgs/msg/wbc_current_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/path.hpp>
#include "arms_target_manager/MarkerFactory.h"
#include "arms_target_manager/marker/HeadMarker.h"
#include "arms_target_manager/marker/ArmMarker.h"
#include "arms_target_manager/marker/BodyMarker.h"

namespace arms_ros2_control::command
{
    class ArmsTargetManager
    {
    public:
        ArmsTargetManager(
            rclcpp::Node::SharedPtr node,
            bool dualArmMode = false,
            std::string frameId = "world",
            std::string markerFixedFrame = "base_footprint",
            double publishRate = 20.0,
            const std::vector<int32_t>& disableAutoUpdateStates = {3},
            double markerUpdateInterval = 0.05);

        ~ArmsTargetManager() = default;

        void initialize(
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_left_target,
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_left_target_stamped,
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_right_target = nullptr,
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_right_target_stamped = nullptr);

        void setMarkerPose(
            const std::string& armType,
            const geometry_msgs::msg::Point& position,
            const geometry_msgs::msg::Quaternion& orientation);

        void updateMarkerPoseIncremental(
            const std::string& armType,
            const std::array<double, 3>& positionDelta,
            const std::array<double, 3>& rpyDelta);

        geometry_msgs::msg::Pose getMarkerPose(const std::string& armType) const;

        void togglePublishMode();

        MarkerState getCurrentMode() const;

        void sendTargetPose(const std::string& marker_type = "arm");

        void sendDualArmTargetPose();

        void sendBodyTargetPose();

        bool isStateDisabled(int32_t state) const;

        bool shouldThrottle(rclcpp::Time& last_time, double interval);

        bool shouldShowLeftArmMarker() const;
        bool shouldShowRightArmMarker() const;
        bool shouldShowBodyMarker() const;

        void markPendingChanges();

        void markerUpdateTimerCallback();

        void fsmCommandCallback(std_msgs::msg::Int32::ConstSharedPtr msg);

        void wbcStateCallback(const arms_ros2_control_msgs::msg::WbcCurrentState::SharedPtr msg);

        void setCurrentPoseCallback(
            const std::string& armType,
            std::function<void(const geometry_msgs::msg::PoseStamped::ConstSharedPtr&)> callback);

        void currentTargetJointCallback(const std_msgs::msg::Float64MultiArray::ConstSharedPtr& msg);

    private:
        visualization_msgs::msg::InteractiveMarker buildMarker(
            const std::string& name,
            const std::string& markerType) const;

        void handleMarkerFeedback(
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);

        void setupMenu();

        void setupMarkerMenu(
            std::shared_ptr<interactive_markers::MenuHandler>& menu_handler,
            interactive_markers::MenuHandler::EntryHandle& send_handle,
            interactive_markers::MenuHandler::EntryHandle& toggle_handle,
            std::function<void()> sendCallback);

        void setupDualArmMenu(
            std::shared_ptr<interactive_markers::MenuHandler>& menu_handler,
            interactive_markers::MenuHandler::EntryHandle& both_handle);

        void updateMarkerShape();

        void updateMenuVisibility();

        void createPublishersAndSubscribers();

        void updateHeadMarkerFromTopic(
            const sensor_msgs::msg::JointState::ConstSharedPtr& joint_msg);

        void updateBodyMarkerVisibility();
        void refreshArmMarkersFromLatestCurrentTargets();

        geometry_msgs::msg::Pose transformPose(
            const geometry_msgs::msg::Pose& pose,
            const std::string& sourceFrameId,
            const std::string& targetFrameId) const;

        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

        std::shared_ptr<MarkerFactory> marker_factory_;

        rclcpp::Subscription<arms_ros2_control_msgs::msg::Inputs>::SharedPtr control_input_subscription_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr head_joint_state_subscription_;
        rclcpp::Subscription<arms_ros2_control_msgs::msg::WbcCurrentState>::SharedPtr wbc_state_subscriber_;

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr dual_target_stamped_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr body_target_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr body_target_stamped_publisher_;

        std::shared_ptr<interactive_markers::MenuHandler> left_menu_handler_;
        std::shared_ptr<interactive_markers::MenuHandler> right_menu_handler_;
        std::shared_ptr<interactive_markers::MenuHandler> head_menu_handler_;
        std::shared_ptr<interactive_markers::MenuHandler> body_menu_handler_;

        interactive_markers::MenuHandler::EntryHandle left_send_handle_{};
        interactive_markers::MenuHandler::EntryHandle left_toggle_handle_{};
        interactive_markers::MenuHandler::EntryHandle left_both_handle_{};
        interactive_markers::MenuHandler::EntryHandle right_send_handle_{};
        interactive_markers::MenuHandler::EntryHandle right_toggle_handle_{};
        interactive_markers::MenuHandler::EntryHandle right_both_handle_{};
        interactive_markers::MenuHandler::EntryHandle head_send_handle_{};
        interactive_markers::MenuHandler::EntryHandle head_toggle_handle_{};
        interactive_markers::MenuHandler::EntryHandle body_send_handle_{};
        interactive_markers::MenuHandler::EntryHandle body_toggle_handle_{};

        bool dual_arm_mode_;
        std::string control_base_frame_;
        std::string marker_fixed_frame_;
        double publish_rate_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        MarkerState current_mode_ = MarkerState::SINGLE_SHOT;

        mutable std::mutex state_update_mutex_;
        int32_t current_controller_state_ = 2;
        std::string current_fsm_state_ = "HOLD";
        std::vector<int32_t> disable_auto_update_states_;

        int left_arm_state_{1};
        int right_arm_state_{1};
        int bimanual_state_{0};
        int body_state_{0};

        rclcpp::Time last_marker_update_time_;
        double marker_update_interval_;
        rclcpp::Time last_publish_time_;

        rclcpp::TimerBase::SharedPtr marker_update_timer_;
        std::atomic<bool> pending_changes_{false};

        std::shared_ptr<ArmMarker> left_arm_marker_;
        std::shared_ptr<ArmMarker> right_arm_marker_;
        std::shared_ptr<HeadMarker> head_marker_;
        std::shared_ptr<BodyMarker> body_marker_;
    };
} // namespace arms_ros2_control::command
