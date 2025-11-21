//
// Created for Arms ROS2 Control - MarkerFactory
// Marker 创建工厂类，统一管理所有 marker 的创建逻辑
//
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace arms_ros2_control::command
{
    /**
     * Marker状态枚举
     */
    enum class MarkerState {
        SINGLE_SHOT,  // 单次发布模式
        CONTINUOUS    // 连续发布模式
    };

    /**
     * MarkerFactory - Marker 创建工厂类
     * 
     * 负责创建各种类型的 interactive marker，统一管理 marker 的创建逻辑
     * 支持：
     * - left_arm: 左臂 marker（蓝色）
     * - right_arm: 右臂 marker（红色）
     * - head: 头部 marker（红色箭头）
     */
    class MarkerFactory
    {
    public:
        /**
         * 构造函数
         * @param node ROS节点指针（用于时间戳和日志）
         * @param frame_id marker 所在的坐标系ID
         * @param disable_states 禁用自动更新的状态值数组（用于判断是否启用交互）
         */
        MarkerFactory(
            rclcpp::Node::SharedPtr node,
            const std::string& frame_id,
            const std::vector<int32_t>& disable_states);

        ~MarkerFactory() = default;

        /**
         * 创建 interactive marker（统一接口，支持所有类型）
         * @param name marker 名称（如 "left_arm_target"）
         * @param markerType marker 类型："left_arm", "right_arm", "head"
         * @param pose marker 的位姿
         * @param mode 发布模式（SINGLE_SHOT / CONTINUOUS），影响可视化形状
         * @param controller_state 当前控制器状态（1=HOME, 2=HOLD, 3=MOVE）
         * @param auto_update_enabled 是否启用自动更新（影响交互功能）
         * @return 创建好的 InteractiveMarker
         */
        visualization_msgs::msg::InteractiveMarker createMarker(
            const std::string& name,
            const std::string& markerType,
            const geometry_msgs::msg::Pose& pose,
            MarkerState mode,
            int32_t controller_state,
            bool auto_update_enabled) const;

    private:
        /**
         * 创建手臂 marker（统一左臂和右臂的逻辑）
         * @param name marker 名称
         * @param description 描述文本（"Left Arm Target" 或 "Right Arm Target"）
         * @param pose marker 的位姿
         * @param color 颜色（"blue" 或 "red"）
         * @param mode 发布模式
         * @param is_auto_update_enabled 是否启用自动更新
         * @return 创建好的 InteractiveMarker
         */
        visualization_msgs::msg::InteractiveMarker createArmMarker(
            const std::string& name,
            const std::string& description,
            const geometry_msgs::msg::Pose& pose,
            const std::string& color,
            MarkerState mode,
            bool is_auto_update_enabled) const;

        /**
         * 创建头部 marker
         * @param name marker 名称
         * @param pose marker 的位姿
         * @param controller_state 控制器状态（只在 MOVE 状态启用交互）
         * @return 创建好的 InteractiveMarker
         */
        visualization_msgs::msg::InteractiveMarker createHeadMarker(
            const std::string& name,
            const geometry_msgs::msg::Pose& pose,
            int32_t controller_state) const;

        /**
         * 创建盒子 marker（用于单次模式）
         * @param color 颜色 ("blue", "red", "grey")
         * @return Marker 消息
         */
        visualization_msgs::msg::Marker createBoxMarker(const std::string& color) const;

        /**
         * 创建球体 marker（用于连续模式）
         * @param color 颜色 ("blue", "red", "grey")
         * @return Marker 消息
         */
        visualization_msgs::msg::Marker createSphereMarker(const std::string& color) const;

        /**
         * 创建箭头 marker（用于头部）
         * @param color 颜色
         * @return Marker 消息
         */
        visualization_msgs::msg::Marker createArrowMarker(const std::string& color) const;

        /**
         * 添加移动控制（6DOF：X/Y/Z 平移和旋转）
         * @param interactiveMarker 要添加控制的 InteractiveMarker
         */
        void addMovementControls(
            visualization_msgs::msg::InteractiveMarker& interactiveMarker) const;

        /**
         * 设置 marker 颜色
         * @param marker 要设置颜色的 Marker
         * @param color 颜色字符串 ("blue", "red", "green", "grey")
         */
        void setMarkerColor(visualization_msgs::msg::Marker& marker, const std::string& color) const;

        /**
         * 判断指定状态是否在禁用列表中
         * @param state 要检查的状态值
         * @return 如果状态在禁用列表中返回 true
         */
        bool isStateDisabled(int32_t state) const;

        // 配置成员
        rclcpp::Node::SharedPtr node_;                      // ROS 节点（用于时间戳和日志）
        std::string frame_id_;                              // marker 坐标系ID
        std::vector<int32_t> disable_states_;               // 禁用自动更新的状态列表
    };

} // namespace arms_ros2_control::command

