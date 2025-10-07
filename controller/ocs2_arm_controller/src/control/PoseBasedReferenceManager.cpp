//
// Created for OCS2 Arm Controller - PoseBasedReferenceManager
//

#include "ocs2_arm_controller/control/PoseBasedReferenceManager.h"
#include <rclcpp/rclcpp.hpp>

namespace ocs2::mobile_manipulator
{
    PoseBasedReferenceManager::PoseBasedReferenceManager(
        std::string topicPrefix,
        std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr,
        std::shared_ptr<MobileManipulatorInterface> interfacePtr)
        : ReferenceManagerDecorator(std::move(referenceManagerPtr)),
          topic_prefix_(std::move(topicPrefix)),
          interface_(std::move(interfacePtr))
    {
        dual_arm_mode_ = interface_->dual_arm_;

        // 初始化target state缓存
        left_target_state_ = vector_t::Zero(7);
        right_target_state_ = vector_t::Zero(7);
        left_target_valid_ = false;
        right_target_valid_ = false;
    }

    void PoseBasedReferenceManager::subscribe(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
    {
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
    }

    void PoseBasedReferenceManager::setCurrentObservation(const SystemObservation& observation)
    {
        current_observation_ = observation;
    }

    void PoseBasedReferenceManager::updateTargetTrajectory()
    {
        // 创建合并的target state
        vector_t combined_target_state;
        bool has_valid_target = false;

        if (dual_arm_mode_)
        {
            if (left_target_valid_ && right_target_valid_)
            {
                combined_target_state = vector_t::Zero(14);
                combined_target_state.segment(0, 7) = left_target_state_;
                combined_target_state.segment(7, 7) = right_target_state_;
                has_valid_target = true;
            }
        }
        else
        {
            if (left_target_valid_)
            {
                combined_target_state = left_target_state_;
                has_valid_target = true;
            }
        }

        if (has_valid_target)
        {
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
        left_target_valid_ = true;

        // 更新target trajectory
        updateTargetTrajectory();
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
        right_target_valid_ = true;

        // 更新target trajectory
        updateTargetTrajectory();
    }

    void PoseBasedReferenceManager::resetTargetStateCache()
    {
        // 重置target state缓存
        left_target_state_ = vector_t::Zero(7);
        right_target_state_ = vector_t::Zero(7);
        left_target_valid_ = false;
        right_target_valid_ = false;

        RCLCPP_INFO(rclcpp::get_logger("PoseBasedReferenceManager"),
                    "Target state cache reset - cleared all cached target states");
    }
} // namespace ocs2::mobile_manipulator
