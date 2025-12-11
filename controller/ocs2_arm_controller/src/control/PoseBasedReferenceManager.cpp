//
// Created for OCS2 Arm Controller - PoseBasedReferenceManager
//

#include "ocs2_arm_controller/control/PoseBasedReferenceManager.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/exceptions.h>

namespace ocs2::mobile_manipulator
{
    PoseBasedReferenceManager::PoseBasedReferenceManager(
        std::string topicPrefix,
        std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr,
        std::shared_ptr<MobileManipulatorInterface> interfacePtr)
        : ReferenceManagerDecorator(std::move(referenceManagerPtr)),
          topic_prefix_(std::move(topicPrefix)),
          interface_(std::move(interfacePtr)),
          logger_(rclcpp::get_logger("PoseBasedReferenceManager"))
    {
        dual_arm_mode_ = interface_->dual_arm_;

        // 获取base frame
        base_frame_ = interface_->getManipulatorModelInfo().baseFrame;

        // 初始化target state缓存
        left_target_state_ = vector_t::Zero(7);
        right_target_state_ = vector_t::Zero(7);
    }

    void PoseBasedReferenceManager::subscribe(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
    {
        // 保存node的logger用于后续日志输出
        logger_ = node->get_logger();
        
        // 保存clock用于时间戳
        clock_ = node->get_clock();
        
        // 初始化TF2 buffer和listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 左臂pose订阅者（单臂机器人也使用这个）
        auto leftCallback = [this](const geometry_msgs::msg::Pose::SharedPtr msg)
        {
            leftPoseCallback(msg);
        };
        left_pose_subscriber_ = node->create_subscription<geometry_msgs::msg::Pose>(
            "left_target", 1, leftCallback);

        // 右臂pose订阅者（仅双臂机器人）
        if (dual_arm_mode_)
        {
            auto rightCallback = [this](const geometry_msgs::msg::Pose::SharedPtr msg)
            {
                rightPoseCallback(msg);
            };
            right_pose_subscriber_ = node->create_subscription<geometry_msgs::msg::Pose>(
                "right_target", 1, rightCallback);
        }

        // 左臂PoseStamped订阅者（单臂机器人也使用这个）
        auto leftStampedCallback = [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            leftPoseStampedCallback(msg);
        };
        left_pose_stamped_subscriber_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
            "left_target/stamped", 1, leftStampedCallback);

        // 右臂PoseStamped订阅者（仅双臂机器人）
        if (dual_arm_mode_)
        {
            auto rightStampedCallback = [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                rightPoseStampedCallback(msg);
            };
            right_pose_stamped_subscriber_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
                "right_target/stamped", 1, rightStampedCallback);
        }
        
        // 初始化发布器：发布当前目标
        left_target_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
            "left_current_target", 1);
        
        if (dual_arm_mode_)
        {
            right_target_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
                "right_current_target", 1);
        }
    }

    void PoseBasedReferenceManager::setCurrentObservation(const SystemObservation& observation)
    {
        current_observation_ = observation;
    }

    void PoseBasedReferenceManager::updateTargetTrajectory()
    {
        // 创建合并的target state
        vector_t combined_target_state;

        if (dual_arm_mode_)
        {
            combined_target_state = vector_t::Zero(14);
            combined_target_state.segment(0, 7) = left_target_state_;
            combined_target_state.segment(7, 7) = right_target_state_;
        }
        else
        {
            combined_target_state = left_target_state_;
        }

        // 使用当前系统时间作为目标时间
        double target_time = current_observation_.time;

        // 创建TargetTrajectories - 只使用一个时间点
        scalar_array_t time_trajectory = {target_time};
        vector_array_t state_trajectory = {combined_target_state};
        vector_array_t input_trajectory(1, vector_t::Zero(interface_->getManipulatorModelInfo().inputDim));

        TargetTrajectories target_trajectories(time_trajectory, state_trajectory, input_trajectory);

        // 设置到ReferenceManager
        referenceManagerPtr_->setTargetTrajectories(std::move(target_trajectories));
    }

    void PoseBasedReferenceManager::leftPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        // 转换pose到状态向量（单臂和双臂模式都使用前7维）
        vector_t target_state = vector_t::Zero(7);
        target_state(0) = msg->position.x;
        target_state(1) = msg->position.y;
        target_state(2) = msg->position.z;
        target_state(3) = msg->orientation.x;
        target_state(4) = msg->orientation.y;
        target_state(5) = msg->orientation.z;
        target_state(6) = msg->orientation.w;

        // 更新左臂target state缓存
        left_target_state_ = target_state;

        // 更新target trajectory
        updateTargetTrajectory();
        
        // 发布当前目标
        publishCurrentTargets();
    }

    void PoseBasedReferenceManager::rightPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        // 转换pose到状态向量（右臂状态为7维）
        vector_t target_state = vector_t::Zero(7);
        target_state(0) = msg->position.x;
        target_state(1) = msg->position.y;
        target_state(2) = msg->position.z;
        target_state(3) = msg->orientation.x;
        target_state(4) = msg->orientation.y;
        target_state(5) = msg->orientation.z;
        target_state(6) = msg->orientation.w;

        // 更新右臂target state缓存
        right_target_state_ = target_state;

        // 更新target trajectory
        updateTargetTrajectory();
        
        // 发布当前目标
        publishCurrentTargets();
    }

    void PoseBasedReferenceManager::processPoseStamped(
        const geometry_msgs::msg::PoseStamped::SharedPtr& msg,
        std::function<void(geometry_msgs::msg::Pose::SharedPtr)> callback)
    {
        // 如果源frame和目标frame相同，直接使用
        if (msg->header.frame_id == base_frame_)
        {
            auto pose_msg = std::make_shared<geometry_msgs::msg::Pose>();
            pose_msg->position = msg->pose.position;
            pose_msg->orientation = msg->pose.orientation;
            callback(pose_msg);
            return;
        }

        // 使用TF转换到base frame（使用最新变换）
        try
        {
            geometry_msgs::msg::PoseStamped transformed_pose;
            
            // 使用最新可用变换
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                base_frame_, msg->header.frame_id, tf2::TimePointZero);

            // 使用doTransform进行转换
            tf2::doTransform(*msg, transformed_pose, transform);

            // 调用指定的回调方法
            auto pose_msg = std::make_shared<geometry_msgs::msg::Pose>();
            pose_msg->position = transformed_pose.pose.position;
            pose_msg->orientation = transformed_pose.pose.orientation;
            callback(pose_msg);
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN(logger_,
                        "无法将pose从 %s 转换到 %s: %s",
                        msg->header.frame_id.c_str(), base_frame_.c_str(), ex.what());
        }
    }

    void PoseBasedReferenceManager::leftPoseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        processPoseStamped(msg, [this](geometry_msgs::msg::Pose::SharedPtr pose_msg) {
            leftPoseCallback(pose_msg);
        });
    }

    void PoseBasedReferenceManager::rightPoseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        processPoseStamped(msg, [this](geometry_msgs::msg::Pose::SharedPtr pose_msg) {
            rightPoseCallback(pose_msg);
        });
    }

    void PoseBasedReferenceManager::resetTargetStateCache()
    {
        // 重置target state缓存
        left_target_state_ = vector_t::Zero(7);
        right_target_state_ = vector_t::Zero(7);

        RCLCPP_INFO(logger_,
                    "Target state cache reset - cleared all cached target states");
    }

    void PoseBasedReferenceManager::setCurrentEndEffectorPoses(const vector_t& left_ee_pose, const vector_t& right_ee_pose)
    {
        // 设置左臂pose到缓存
        left_target_state_ = left_ee_pose;
        
        // 如果是双臂模式，设置右臂pose到缓存
        if (dual_arm_mode_)
        {
            right_target_state_ = right_ee_pose;
        }
        
        // 发布当前目标
        publishCurrentTargets();
    }

    void PoseBasedReferenceManager::publishCurrentTargets()
    {
        // 发布左臂当前目标
        geometry_msgs::msg::PoseStamped left_target_msg;
        left_target_msg.header.stamp = clock_->now();
        left_target_msg.header.frame_id = base_frame_;
        left_target_msg.pose.position.x = left_target_state_(0);
        left_target_msg.pose.position.y = left_target_state_(1);
        left_target_msg.pose.position.z = left_target_state_(2);
        left_target_msg.pose.orientation.x = left_target_state_(3);
        left_target_msg.pose.orientation.y = left_target_state_(4);
        left_target_msg.pose.orientation.z = left_target_state_(5);
        left_target_msg.pose.orientation.w = left_target_state_(6);
        left_target_publisher_->publish(left_target_msg);
        
        // 发布右臂当前目标（仅双臂模式）
        if (dual_arm_mode_)
        {
            geometry_msgs::msg::PoseStamped right_target_msg;
            right_target_msg.header.stamp = clock_->now();
            right_target_msg.header.frame_id = base_frame_;
            right_target_msg.pose.position.x = right_target_state_(0);
            right_target_msg.pose.position.y = right_target_state_(1);
            right_target_msg.pose.position.z = right_target_state_(2);
            right_target_msg.pose.orientation.x = right_target_state_(3);
            right_target_msg.pose.orientation.y = right_target_state_(4);
            right_target_msg.pose.orientation.z = right_target_state_(5);
            right_target_msg.pose.orientation.w = right_target_state_(6);
            right_target_publisher_->publish(right_target_msg);
        }
    }
} // namespace ocs2::mobile_manipulator
