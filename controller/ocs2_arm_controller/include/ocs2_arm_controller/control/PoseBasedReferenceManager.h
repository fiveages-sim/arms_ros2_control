//
// Created for OCS2 Arm Controller - PoseBasedReferenceManager
//

#ifndef POSE_BASED_REFERENCE_MANAGER_H
#define POSE_BASED_REFERENCE_MANAGER_H

#include <memory>
#include <string>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ocs2_oc/synchronized_module/ReferenceManagerDecorator.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mpc/SystemObservation.h>

namespace ocs2::mobile_manipulator
{
    /**
     * PoseBasedReferenceManager - 基于末端执行器pose的参考管理器
     * 
     * 订阅主题: 
     * - {topicPrefix}_left_pose_target: 左臂目标pose
     * - {topicPrefix}_right_pose_target: 右臂目标pose
     * 
     * 单臂机器人默认使用左臂主题
     */
    class PoseBasedReferenceManager : public ReferenceManagerDecorator
    {
    public:
        /**
         * 构造函数
         */
        PoseBasedReferenceManager(
            std::string topicPrefix,
            std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr,
            std::shared_ptr<MobileManipulatorInterface> interfacePtr);

        ~PoseBasedReferenceManager() override = default;

        /**
         * 订阅ROS主题（仅支持LifecycleNode）
         */
        void subscribe(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node);

    public:
        /**
         * 设置当前的SystemObservation（由CtrlComponent调用）
         */
        void setCurrentObservation(const SystemObservation& observation);

        /**
         * 重置target state缓存（在离开OCS2状态时调用）
         */
        void resetTargetStateCache();

    private:
        void leftPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
        void rightPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
        void updateTargetTrajectory();

        const std::string topic_prefix_;
        std::shared_ptr<MobileManipulatorInterface> interface_;
        bool dual_arm_mode_;
        
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr left_pose_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr right_pose_subscriber_;
        
        // 当前系统观测（线程安全）
        mutable std::mutex observation_mutex_;
        SystemObservation current_observation_;
        
        // 双臂target state缓存（线程安全）
        mutable std::mutex target_state_mutex_;
        vector_t left_target_state_;
        vector_t right_target_state_;
        bool left_target_valid_;
        bool right_target_valid_;
    };

} // namespace ocs2::mobile_manipulator

#endif // POSE_BASED_REFERENCE_MANAGER_H
