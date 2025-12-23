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
        const std::array<double, 3>& initial_position,
        const std::string& target_topic,
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
          , target_publisher_(node_->create_publisher<geometry_msgs::msg::Pose>(target_topic, 1))
          , target_stamped_publisher_(node_->create_publisher<geometry_msgs::msg::PoseStamped>(target_topic + "/stamped", 1))
          , current_target_frame_id_(control_base_frame)  // 默认使用 control_base_frame
          , update_callback_(std::move(update_callback))
          , last_publish_time_(node_->now())
          , last_subscription_update_time_(node_->now())
    {
        // 初始化默认 pose
        pose_.position.x = initial_position_[0];
        pose_.position.y = initial_position_[1];
        pose_.position.z = initial_position_[2];
        pose_.orientation.w = 1.0;
        pose_.orientation.x = 0.0;
        pose_.orientation.y = 0.0;
        pose_.orientation.z = 0.0;

        // 创建当前 pose 订阅器
        current_pose_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            current_pose_topic, 10,
            [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
            {
                updateFromTopic(msg);
            });

        // 创建当前目标订阅器（用于获取 frame_id 和更新 marker 位置）
        std::string current_target_topic = (arm_type == ArmType::LEFT) ? "left_current_target" : "right_current_target";
        current_target_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            current_target_topic, 10,
            [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
            {
                // 更新 frame_id
                {
                    std::lock_guard<std::mutex> lock(frame_id_mutex_);
                    current_target_frame_id_ = msg->header.frame_id;
                }

                // 获取消息的 frame_id
                std::string source_frame_id = msg->header.frame_id;
                
                // 如果 frame_id 为空，跳过更新
                if (source_frame_id.empty())
                {
                    return;
                }

                // 转换 pose 到 marker 的坐标系（frame_id_）
                geometry_msgs::msg::Pose transformed_pose = transformPose(
                    msg->pose, source_frame_id, frame_id_);

                // 更新内部 pose 存储
                pose_ = transformed_pose;

                // 调用更新回调，通知外部更新可视化
                if (update_callback_)
                {
                    update_callback_(getMarkerName(), pose_);
                }
            });
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
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg)
    {
        // 先检查是否允许自动更新（只有在非禁用状态下才更新 pose_）
        if (state_check_callback_ && !state_check_callback_())
        {
            // 状态不允许自动更新，直接返回，不覆盖用户拖动的位置
            return;
        }

        // 节流检查：限制更新频率为最多30Hz（1/30秒间隔）
        auto now = node_->now();
        auto time_since_last = (now - last_subscription_update_time_).seconds();
        if (time_since_last < 1.0 / 30.0)  // 30Hz = 1/30秒
        {
            return;  // 跳过此次更新
        }
        last_subscription_update_time_ = now;

        // 转换 pose 到目标 frame
        std::string source_frame_id = pose_msg->header.frame_id;
        pose_ = transformPose(pose_msg->pose, source_frame_id, frame_id_);

        // 调用更新回调，通知外部更新可视化
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

    bool ArmMarker::publishTargetPose(bool force, bool use_stamped)
    {
        // 如果是单次发布且使用 stamped，发布到 left_target/stamped
        if (force && use_stamped)
        {
            if (!target_stamped_publisher_)
            {
                return false;
            }

            // 获取当前目标的 frame_id（从 left_current_target 或 right_current_target）
            std::string target_frame_id;
            {
                std::lock_guard<std::mutex> lock(frame_id_mutex_);
                target_frame_id = current_target_frame_id_;
            }

            // 如果 frame_id 为空，使用默认的 control_base_frame
            if (target_frame_id.empty())
            {
                target_frame_id = control_base_frame_;
            }

            // 转换坐标系：从 marker_fixed_frame_ 转换到 target_frame_id
            geometry_msgs::msg::Pose transformed_pose = transformPose(
                pose_, frame_id_, target_frame_id);

            // 创建 PoseStamped 消息
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = target_frame_id;
            pose_stamped.header.stamp = node_->get_clock()->now();
            pose_stamped.pose = transformed_pose;

            target_stamped_publisher_->publish(pose_stamped);
            
            // 更新节流时间
            last_publish_time_ = node_->now();
            
            return true;
        }

        // 连续发布模式：发布到 left_target（原逻辑）
        if (!target_publisher_)
        {
            return false;
        }

        // 如果不是强制发送，检查是否需要节流（用于连续发布模式）
        if (!force)
        {
            if (!shouldThrottle(1.0 / publish_rate_))
            {
                return false;
            }
        }

        // 转换坐标系：从 marker_fixed_frame_ 转换到 control_base_frame_
        geometry_msgs::msg::Pose transformed_pose = transformPose(
            pose_, frame_id_, control_base_frame_);

        target_publisher_->publish(transformed_pose);
        
        // 即使强制发送，也更新节流时间，避免连续强制发送过于频繁
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
            // 转换失败时直接使用原始pose，不输出警告
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

