//
// ArmMarker - 机械臂 Marker 管理类
// 封装所有机械臂 marker 相关的逻辑，包括创建、更新、发布等
//
#pragma once

#include <memory>
#include <string>
#include <array>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <tf2_ros/buffer.h>
#include "arms_target_manager/MarkerFactory.h"

namespace arms_ros2_control::command
{
    class MarkerFactory; // 前向声明

    /**
     * @brief 手臂类型枚举
     */
    enum class ArmType
    {
        LEFT,   // 左臂
        RIGHT   // 右臂
    };

    /**
     * @brief ArmMarker - 机械臂 Marker 管理类
     * 
     * 负责管理机械臂 marker 的所有功能，包括：
     * - Marker 创建和更新
     * - 处理 marker feedback
     * - 发布目标 pose
     * - 从 topic 更新 marker
     * - 坐标转换
     */
    class ArmMarker
    {
    public:
        /**
         * @brief 构造函数
         * @param node ROS 节点指针
         * @param marker_factory Marker 工厂（用于创建 marker）
         * @param tf_buffer TF 缓冲区（用于坐标转换）
         * @param frame_id Marker 所在的坐标系ID
         * @param control_base_frame 控制基坐标系（目标 pose 会转换到此坐标系发布）
         * @param arm_type 手臂类型（LEFT 或 RIGHT）
         * @param initial_position 初始位置 [x, y, z]
         */
        ArmMarker(
            rclcpp::Node::SharedPtr node,
            std::shared_ptr<MarkerFactory> marker_factory,
            std::shared_ptr<tf2_ros::Buffer> tf_buffer,
            const std::string& frame_id,
            const std::string& control_base_frame,
            ArmType arm_type,
            const std::array<double, 3>& initial_position);

        /**
         * @brief 创建 marker
         * @param name Marker 名称
         * @param mode 发布模式（SINGLE_SHOT 或 CONTINUOUS）
         * @param enable_interaction 是否启用交互功能
         * @return 创建好的 InteractiveMarker
         */
        visualization_msgs::msg::InteractiveMarker createMarker(
            const std::string& name,
            MarkerState mode,
            bool enable_interaction) const;

        /**
         * @brief 处理 marker feedback
         * @param feedback Marker 反馈消息
         * @param source_frame_id 源坐标系ID
         * @return 转换后的 pose（在 marker_fixed_frame_ 坐标系下）
         */
        geometry_msgs::msg::Pose handleFeedback(
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback,
            const std::string& source_frame_id) const;

        /**
         * @brief 从 topic 更新 marker pose
         * @param pose_msg PoseStamped 消息
         * @param source_frame_id 源坐标系ID
         */
        void updateFromTopic(
            const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg,
            const std::string& source_frame_id);

        /**
         * @brief 发布目标 pose
         * @param publisher 发布器
         * @param publish_rate 发布频率（用于节流）
         * @param last_publish_time 上次发布时间（会被更新）
         * @return 是否成功发布
         */
        bool publishTargetPose(
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher,
            double publish_rate,
            rclcpp::Time& last_publish_time) const;

        /**
         * @brief 获取当前 pose
         * @return 当前 pose（在 marker_fixed_frame_ 坐标系下）
         */
        geometry_msgs::msg::Pose getPose() const { return pose_; }

        /**
         * @brief 设置 pose
         * @param pose 新的 pose
         */
        void setPose(const geometry_msgs::msg::Pose& pose) { pose_ = pose; }

        /**
         * @brief 获取手臂类型
         * @return 手臂类型（LEFT 或 RIGHT）
         */
        ArmType getArmType() const { return arm_type_; }

        /**
         * @brief 获取 marker 名称（用于识别）
         * @return Marker 名称（"left_arm_target" 或 "right_arm_target"）
         */
        std::string getMarkerName() const;

        /**
         * @brief 获取描述文本
         * @return 描述文本（"Left Arm Target" 或 "Right Arm Target"）
         */
        std::string getDescription() const;

        /**
         * @brief 获取颜色
         * @return 颜色字符串（"blue" 或 "red"）
         */
        std::string getColor() const;

    private:
        /**
         * @brief 转换 pose 到目标坐标系
         * @param pose 源 pose
         * @param source_frame_id 源坐标系ID
         * @param target_frame_id 目标坐标系ID
         * @return 转换后的 pose
         */
        geometry_msgs::msg::Pose transformPose(
            const geometry_msgs::msg::Pose& pose,
            const std::string& source_frame_id,
            const std::string& target_frame_id) const;

        /**
         * @brief 检查是否应该节流发布
         * @param last_time 上次执行时间（会被更新）
         * @param interval 最小间隔（秒）
         * @return 如果应该执行返回 true
         */
        bool shouldThrottle(rclcpp::Time& last_time, double interval) const;

        // ROS 节点和工具
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<MarkerFactory> marker_factory_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::string frame_id_;              // Marker 所在的坐标系
        std::string control_base_frame_;    // 控制基坐标系

        // 手臂类型和配置
        ArmType arm_type_;
        std::array<double, 3> initial_position_;

        // 状态
        geometry_msgs::msg::Pose pose_;
    };
} // namespace arms_ros2_control::command

