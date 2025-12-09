//
// HeadMarker - 头部 Marker 管理类
// 封装所有头部 marker 相关的逻辑，包括配置、更新、限制等
//
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <array>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <arms_controller_common/utils/JointLimitsManager.h>
#include <arms_controller_common/utils/AngleUtils.h>

namespace arms_ros2_control::command
{
    class MarkerFactory; // 前向声明

    /**
     * @brief HeadMarker - 头部 Marker 管理类
     * 
     * 负责管理头部 marker 的所有功能，包括：
     * - 配置管理（关节映射、限位等）
     * - Marker 创建和更新
     * - 从四元数提取关节角度
     * - 限制头部旋转范围
     * - 从关节状态更新 marker
     * - 发布目标关节角度
     */
    class HeadMarker
    {
    public:
        /**
         * @brief 构造函数
         * @param node ROS 节点指针
         * @param marker_factory Marker 工厂（用于创建 marker）
         * @param tf_buffer TF 缓冲区（用于坐标转换）
         * @param frame_id Marker 所在的坐标系ID
         */
        HeadMarker(
            rclcpp::Node::SharedPtr node,
            std::shared_ptr<MarkerFactory> marker_factory,
            std::shared_ptr<tf2_ros::Buffer> tf_buffer,
            const std::string& frame_id);

        /**
         * @brief 初始化头部配置
         * 从 ROS 参数读取头部控制相关配置
         */
        void initialize();

        /**
         * @brief 检查是否启用头部控制
         * @return 如果启用头部控制返回 true
         */
        bool isEnabled() const { return enable_head_control_; }

        /**
         * @brief 创建头部 marker
         * @param name Marker 名称
         * @param pose Marker 的位姿
         * @param enable_interaction 是否启用交互功能
         * @return 创建好的 InteractiveMarker
         */
        visualization_msgs::msg::InteractiveMarker createMarker(
            const std::string& name,
            const geometry_msgs::msg::Pose& pose,
            bool enable_interaction) const;

        /**
         * @brief 从四元数提取头部关节角度
         * @param quaternion 输入四元数
         * @return 关节角度数组（按照控制器期望的顺序）
         */
        std::vector<double> quaternionToJointAngles(
            const geometry_msgs::msg::Quaternion& quaternion) const;

        /**
         * @brief 限制头部 pose 的旋转范围（根据关节限位）
         * @param pose 要限制的 pose（输入输出参数）
         * @return 如果 pose 被限制返回 true，否则返回 false
         */
        bool clampPoseRotation(geometry_msgs::msg::Pose& pose) const;

        /**
         * @brief 从关节状态更新 marker
         * @param joint_msg 关节状态消息
         * @param current_controller_state 当前控制器状态
         * @param is_state_disabled 状态是否禁用自动更新
         * @return 更新后的 pose（用于更新 marker 位置）
         */
        geometry_msgs::msg::Pose updateFromJointState(
            const sensor_msgs::msg::JointState::ConstSharedPtr& joint_msg,
            int32_t current_controller_state,
            bool is_state_disabled);

        /**
         * @brief 获取当前头部 pose
         * @return 当前头部 pose
         */
        geometry_msgs::msg::Pose getPose() const { return head_pose_; }

        /**
         * @brief 设置头部 pose
         * @param pose 新的 pose
         */
        void setPose(const geometry_msgs::msg::Pose& pose) { head_pose_ = pose; }

        /**
         * @brief 创建关节角度发布器
         * @param topic_name Topic 名称
         * @return 发布器指针
         */
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
        createJointPublisher(const std::string& topic_name);

        /**
         * @brief 发布目标关节角度
         * @param publisher 发布器
         * @param pose 目标 pose
         */
        void publishTargetJointAngles(
            rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher,
            const geometry_msgs::msg::Pose& pose) const;

        /**
         * @brief 获取头部 link 名称
         * @return 头部 link 名称
         */
        std::string getLinkName() const { return head_link_name_; }

        /**
         * @brief 获取关节发送顺序
         * @return 关节名称数组（按照控制器期望的顺序）
         */
        std::vector<std::string> getJointSendOrder() const { return head_joint_send_order_; }

    private:
        /**
         * @brief 初始化关节索引（从 joint_states 中查找）
         * @param joint_msg 关节状态消息
         */
        void initializeJointIndices(const sensor_msgs::msg::JointState::ConstSharedPtr& joint_msg);

        /**
         * @brief 从关节状态提取 RPY 角度
         * @param joint_msg 关节状态消息
         * @param head_roll 输出的 roll 角度
         * @param head_pitch 输出的 pitch 角度
         * @param head_yaw 输出的 yaw 角度
         * @return 是否成功提取到角度
         */
        bool extractRPYFromJointState(
            const sensor_msgs::msg::JointState::ConstSharedPtr& joint_msg,
            double& head_roll,
            double& head_pitch,
            double& head_yaw) const;

        // ROS 节点和工具
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<MarkerFactory> marker_factory_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::string frame_id_;

        // 配置
        bool enable_head_control_ = false;
        std::string head_link_name_;
        std::array<double, 3> head_marker_position_ = {1.0, 0.0, 1.5};

        // 关节映射和配置
        std::map<std::string, size_t> head_joint_indices_; // 关节名称到索引的映射
        std::map<std::string, std::string> head_joint_to_rpy_mapping_; // 关节名称到RPY的映射
        std::vector<std::string> head_joint_send_order_; // 发送时的关节顺序
        std::map<std::string, double> head_rpy_axis_direction_; // RPY旋转轴方向

        // 限位管理
        std::shared_ptr<arms_controller_common::JointLimitsManager> head_limits_manager_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;

        // 状态
        geometry_msgs::msg::Pose head_pose_;

        // 上一次的 RPY 角度（用于避免角度跳变）
        mutable std::array<double, 3> last_head_rpy_ = {0.0, 0.0, 0.0};
        mutable bool last_head_rpy_initialized_ = false;
    };
} // namespace arms_ros2_control::command

