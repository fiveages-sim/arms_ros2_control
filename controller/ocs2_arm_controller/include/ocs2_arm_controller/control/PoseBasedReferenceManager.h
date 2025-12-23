//
// Created for OCS2 Arm Controller - PoseBasedReferenceManager
//
#pragma once

#include <memory>
#include <string>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
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
         * @param topicPrefix Topic前缀
         * @param referenceManagerPtr 参考管理器指针
         * @param interfacePtr 移动操作器接口指针
         * @param trajectoryDuration 插值轨迹持续时间（秒），默认2.0秒
         * @param moveLDuration moveL 插值轨迹持续时间（秒），默认2.0秒
         */
        PoseBasedReferenceManager(
            std::string topicPrefix,
            std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr,
            std::shared_ptr<MobileManipulatorInterface> interfacePtr,
            double trajectoryDuration = 2.0,
            double moveLDuration = 2.0);

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

        /**
         * 设置当前状态的末端执行器pose到缓存（在enter状态时调用）
         * @param left_ee_pose 左臂末端执行器pose（7维：x, y, z, qx, qy, qz, qw）
         * @param right_ee_pose 右臂末端执行器pose（7维，仅双臂模式需要）
         */
        void setCurrentEndEffectorPoses(const vector_t& left_ee_pose, const vector_t& right_ee_pose);

    private:
        void leftPoseCallback(geometry_msgs::msg::Pose::SharedPtr msg);
        void rightPoseCallback(geometry_msgs::msg::Pose::SharedPtr msg);
        void leftPoseStampedCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void rightPoseStampedCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void dualTargetStampedCallback(nav_msgs::msg::Path::SharedPtr msg);
        void pathCallback(nav_msgs::msg::Path::SharedPtr msg);
        void updateTargetTrajectory();
        /**
         * 使用"上一帧缓存目标 -> 当前新目标缓存"生成插值轨迹并写入 ReferenceManager。
         * 轨迹时长由 moveL_duration_ 参数决定。
         * 仅用于 PoseStamped 相关回调（支持 TF 转换后的目标）。
         * 
         * 在双臂模式下，总是同时更新两个臂的轨迹。
         * 在单臂模式下，previous_right_target_state 参数不会被使用（可以传递零向量）。
         * 
         * @param previous_left_target_state 左臂上一帧目标状态
         * @param previous_right_target_state 右臂上一帧目标状态（单臂模式下不使用）
         */
        void updateTrajectory(const vector_t& previous_left_target_state,
                              const vector_t& previous_right_target_state);

        // PoseStamped 经过 TF 转换后的 Pose 处理（与 Pose 回调分开，便于区分旧/新接口）
        void leftPoseStampedPoseCallback(geometry_msgs::msg::Pose::SharedPtr msg);
        void rightPoseStampedPoseCallback(geometry_msgs::msg::Pose::SharedPtr msg);
        
        // 通用的PoseStamped处理函数：转换到base frame并调用指定的回调
        void processPoseStamped(
            const geometry_msgs::msg::PoseStamped::SharedPtr& msg,
            std::function<void(geometry_msgs::msg::Pose::SharedPtr)> callback);
        
        // 发布当前目标
        void publishCurrentTargets();

        const std::string topic_prefix_;
        std::shared_ptr<MobileManipulatorInterface> interface_;
        bool dual_arm_mode_;
        
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr left_pose_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr right_pose_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_pose_stamped_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_pose_stamped_subscriber_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr dual_target_stamped_subscriber_;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
        
        // 发布器：发布当前目标
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_target_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_target_publisher_;
        
        // TF2相关
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::shared_ptr<rclcpp::Clock> clock_;
        std::string base_frame_;
        
        // Logger（从node获取，用于日志输出）
        rclcpp::Logger logger_;
        
        // 当前系统观测
        SystemObservation current_observation_;
        
        // 双臂target state缓存
        vector_t left_target_state_;
        vector_t right_target_state_;
        
        // 插值轨迹持续时间（秒）
        double trajectory_duration_;
        
        // moveL 插值轨迹持续时间（秒）
        double moveL_duration_;
    };

} // namespace ocs2::mobile_manipulator
