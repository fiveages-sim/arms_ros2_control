//
// ArmMarker - 机械臂 Marker 管理类实现
//

#include "arms_target_manager/marker/ArmMarker.h"
#include "arms_target_manager/MarkerFactory.h"
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>

namespace arms_ros2_control::command
{
    ArmMarker::ArmMarker(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<MarkerFactory> marker_factory,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        const std::string& frame_id,
        const std::string& control_base_frame,
        ArmType arm_type,
        const std::array<double, 3>& initial_position)
        : node_(std::move(node))
          , marker_factory_(std::move(marker_factory))
          , tf_buffer_(std::move(tf_buffer))
          , frame_id_(frame_id)
          , control_base_frame_(control_base_frame)
          , arm_type_(arm_type)
          , initial_position_(initial_position)
    {
        // 初始化默认 pose
        pose_.position.x = initial_position_[0];
        pose_.position.y = initial_position_[1];
        pose_.position.z = initial_position_[2];
        pose_.orientation.w = 1.0;
        pose_.orientation.x = 0.0;
        pose_.orientation.y = 0.0;
        pose_.orientation.z = 0.0;
    }

    visualization_msgs::msg::InteractiveMarker ArmMarker::createMarker(
        const std::string& name,
        MarkerState mode,
        bool enable_interaction) const
    {
        return marker_factory_->createArmMarker(
            name, getDescription(), pose_, getColor(), mode, enable_interaction);
    }

    geometry_msgs::msg::Pose ArmMarker::handleFeedback(
        const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback,
        const std::string& source_frame_id) const
    {
        // 转换 pose 到目标 frame
        geometry_msgs::msg::Pose transformed_pose = transformPose(
            feedback->pose, source_frame_id, frame_id_);
        
        return transformed_pose;
    }

    void ArmMarker::updateFromTopic(
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg,
        const std::string& source_frame_id)
    {
        // 转换 pose 到目标 frame
        pose_ = transformPose(pose_msg->pose, source_frame_id, frame_id_);
    }

    bool ArmMarker::publishTargetPose(
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher,
        double publish_rate,
        rclcpp::Time& last_publish_time) const
    {
        if (!publisher)
        {
            return false;
        }

        // 检查是否需要节流
        if (!shouldThrottle(last_publish_time, 1.0 / publish_rate))
        {
            return false;
        }

        // 转换坐标系：从 marker_fixed_frame_ 转换到 control_base_frame_
        geometry_msgs::msg::Pose transformed_pose = transformPose(
            pose_, frame_id_, control_base_frame_);

        publisher->publish(transformed_pose);
        return true;
    }

    std::string ArmMarker::getMarkerName() const
    {
        return (arm_type_ == ArmType::LEFT) ? "left_arm_target" : "right_arm_target";
    }

    std::string ArmMarker::getDescription() const
    {
        return (arm_type_ == ArmType::LEFT) ? "Left Arm Target" : "Right Arm Target";
    }

    std::string ArmMarker::getColor() const
    {
        return (arm_type_ == ArmType::LEFT) ? "blue" : "red";
    }

    geometry_msgs::msg::Pose ArmMarker::transformPose(
        const geometry_msgs::msg::Pose& pose,
        const std::string& source_frame_id,
        const std::string& target_frame_id) const
    {
        // 如果源 frame 和目标 frame 相同，不需要转换
        if (source_frame_id == target_frame_id)
        {
            return pose;
        }

        try
        {
            // 创建 PoseStamped 用于转换
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = source_frame_id;
            pose_stamped.header.stamp = rclcpp::Time(0); // 使用 Time(0) 表示使用最新变换
            pose_stamped.pose = pose;

            // 获取最新的变换并使用 doTransform 进行转换
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                target_frame_id, source_frame_id, tf2::TimePointZero);

            // 使用 doTransform 进行转换
            geometry_msgs::msg::PoseStamped result_stamped;
            tf2::doTransform(pose_stamped, result_stamped, transform);
            return result_stamped.pose;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "无法将pose从 %s 转换到 %s: %s，使用原始pose",
                        source_frame_id.c_str(), target_frame_id.c_str(), ex.what());
            return pose;
        }
    }

    bool ArmMarker::shouldThrottle(rclcpp::Time& last_time, double interval) const
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
} // namespace arms_ros2_control::command

