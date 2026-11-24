//
//

#include "arms_target_manager/marker/ArmMarker.h"
#include "arms_target_manager/MarkerFactory.h"
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.hpp>

namespace arms_ros2_control::command
{
    ArmMarker::ArmMarker(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<MarkerFactory> marker_factory,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        const std::string& frame_id,
        const std::string& control_base_frame,
        ArmType arm_type,
        const std::array<double, 3>& initial_position,
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_publisher,
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_stamped_publisher,
        const std::string& current_pose_topic,
        double publish_rate,
        UpdateCallback update_callback)
        : node_(std::move(node))
          , marker_factory_(std::move(marker_factory))
          , tf_buffer_(std::move(tf_buffer))
          , frame_id_(frame_id)
          , control_base_frame_(control_base_frame)
          , arm_type_(arm_type)
          , initial_position_(initial_position)
          , publish_rate_(publish_rate)
          , target_publisher_(std::move(target_publisher))
          , target_stamped_publisher_(std::move(target_stamped_publisher))
          , current_target_frame_id_(control_base_frame)
          , update_callback_(std::move(update_callback))
          , last_publish_time_(node_->now())
          , last_subscription_update_time_(node_->now())
          , last_marker_command_time_(rclcpp::Time(0, 0, node_->get_clock()->get_clock_type()))
    {
        pose_.position.x = initial_position_[0];
        pose_.position.y = initial_position_[1];
        pose_.position.z = initial_position_[2];
        pose_.orientation.w = 1.0;
        pose_.orientation.x = 0.0;
        pose_.orientation.y = 0.0;
        pose_.orientation.z = 0.0;

        current_pose_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            current_pose_topic, 10,
            [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
            {
                latest_current_pose_ = *msg;
                has_latest_current_pose_ = true;

                if (current_pose_callback_)
                {
                    current_pose_callback_(msg);
                }
                updateFromTopic(msg);
            });

        std::string current_target_topic = arm_type == ArmType::LEFT ? "left_current_target" : "right_current_target";
        current_target_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            current_target_topic, 10,
            [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
            {
                current_target_frame_id_ = msg->header.frame_id;
                latest_current_target_pose_ = *msg;
                has_latest_current_target_pose_ = true;

                std::string source_frame_id = msg->header.frame_id;
                
                if (source_frame_id.empty())
                {
                    return;
                }

                auto now = node_->now();
                if (last_marker_command_time_.seconds() > 0)
                {
                    auto time_since_last_command = (now - last_marker_command_time_).seconds();
                    if (time_since_last_command < 1.0)
                    {
                        return;
                    }
                }

                geometry_msgs::msg::Pose transformed_pose = transformPose(
                    msg->pose, source_frame_id, frame_id_);

                pose_ = transformed_pose;

                if (update_callback_)
                {
                    update_callback_(getMarkerName(), pose_);
                }
            });
    }

    bool ArmMarker::getLatestCurrentTarget(geometry_msgs::msg::PoseStamped& out) const
    {
        if (!has_latest_current_target_pose_)
        {
            return false;
        }
        out = latest_current_target_pose_;
        return true;
    }

    bool ArmMarker::refreshFromLatestCurrentTarget()
    {
        if (!has_latest_current_target_pose_)
        {
            return false;
        }

        const std::string& source_frame_id = latest_current_target_pose_.header.frame_id;
        if (source_frame_id.empty())
        {
            return false;
        }

        geometry_msgs::msg::Pose transformed_pose = transformPose(
            latest_current_target_pose_.pose, source_frame_id, frame_id_);
        pose_ = transformed_pose;

        if (update_callback_)
        {
            update_callback_(getMarkerName(), pose_);
        }
        return true;
    }

    bool ArmMarker::refreshFromLatestCurrentPose()
    {
        if (!has_latest_current_pose_)
        {
            return false;
        }

        const std::string& source_frame_id = latest_current_pose_.header.frame_id;
        if (source_frame_id.empty())
        {
            return false;
        }

        geometry_msgs::msg::Pose transformed_pose = transformPose(
            latest_current_pose_.pose, source_frame_id, frame_id_);
        pose_ = transformed_pose;

        if (update_callback_)
        {
            update_callback_(getMarkerName(), pose_);
        }
        return true;
    }

    void ArmMarker::clearCommandCooldown()
    {
        last_marker_command_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
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
        geometry_msgs::msg::Pose transformed_pose = transformPose(
            feedback->pose, source_frame_id, frame_id_);
        
        return transformed_pose;
    }

    void ArmMarker::updateFromTopic(
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg)
    {
        if (state_check_callback_ && !state_check_callback_())
        {
            return;
        }

        auto now = node_->now();
        auto time_since_last = (now - last_subscription_update_time_).seconds();
        if (time_since_last < 1.0 / 30.0)
        {
            return;
        }
        last_subscription_update_time_ = now;

        std::string source_frame_id = pose_msg->header.frame_id;
        pose_ = transformPose(pose_msg->pose, source_frame_id, frame_id_);

        if (update_callback_)
        {
            update_callback_(getMarkerName(), pose_);
        }
    }

    void ArmMarker::setUpdateCallback(UpdateCallback callback)
    {
        update_callback_ = std::move(callback);
    }

    void ArmMarker::setStateCheckCallback(StateCheckCallback callback)
    {
        state_check_callback_ = std::move(callback);
    }

    void ArmMarker::setCurrentPoseCallback(CurrentPoseCallback callback)
    {
        current_pose_callback_ = std::move(callback);
    }

    bool ArmMarker::publishTargetPose(bool force, bool use_stamped)
    {
        if (force && use_stamped)
        {
            if (!target_stamped_publisher_)
            {
                return false;
            }

            std::string target_frame_id = current_target_frame_id_;

            if (target_frame_id.empty())
            {
                target_frame_id = control_base_frame_;
            }

            geometry_msgs::msg::Pose transformed_pose = transformPose(
                pose_, frame_id_, target_frame_id);

            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = target_frame_id;
            pose_stamped.header.stamp = node_->get_clock()->now();
            pose_stamped.pose = transformed_pose;

            target_stamped_publisher_->publish(pose_stamped);
            
            last_marker_command_time_ = node_->now();
            
            last_publish_time_ = node_->now();
            
            return true;
        }

        if (!target_publisher_)
        {
            return false;
        }

        if (!force)
        {
            if (!shouldThrottle(1.0 / publish_rate_))
            {
                return false;
            }
        }

        geometry_msgs::msg::Pose transformed_pose = transformPose(
            pose_, frame_id_, control_base_frame_);

        target_publisher_->publish(transformed_pose);
        
        last_marker_command_time_ = node_->now();
        
        if (force)
        {
            last_publish_time_ = node_->now();
        }
        
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
        if (source_frame_id == target_frame_id)
        {
            return pose;
        }

        try
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = source_frame_id;
            pose_stamped.header.stamp = rclcpp::Time(0);
            pose_stamped.pose = pose;

            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                target_frame_id, source_frame_id, tf2::TimePointZero);

            geometry_msgs::msg::PoseStamped result_stamped;
            tf2::doTransform(pose_stamped, result_stamped, transform);
            return result_stamped.pose;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "Failed to transform pose from '%s' to '%s': %s (using untransformed pose)",
                source_frame_id.c_str(), target_frame_id.c_str(), ex.what());
            return pose;
        }
    }

    bool ArmMarker::shouldThrottle(double interval)
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
} // namespace arms_ros2_control::command

