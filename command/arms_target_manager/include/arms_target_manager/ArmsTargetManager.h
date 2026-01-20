//
// Created for Arms ROS2 Control - ArmsTargetManager
//
#pragma once


#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <array>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <arms_ros2_control_msgs/msg/inputs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/path.hpp>
#include "arms_target_manager/MarkerFactory.h"
#include "arms_target_manager/marker/HeadMarker.h"
#include "arms_target_manager/marker/ArmMarker.h"

namespace arms_ros2_control::command
{
    /**
     * ArmsTargetManager - 机械臂目标管理器
     * 
     * 提供3D交互式marker，用于设置机械臂末端执行器的目标pose
     * 支持单臂和双臂模式，包含右键菜单和连续发布功能
     * 
     * 发布主题: 
     * - left_target: 左臂目标pose
     * - right_target: 右臂目标pose
     * 
     * 单臂机器人默认使用左臂主题
     */
    class ArmsTargetManager
    {
    public:
        /**
         * 构造函数
         * @param node ROS节点指针
         * @param dualArmMode 是否为双臂模式
         * @param frameId 坐标系ID，默认为"world"（目标frame，marker会转换到这个frame下发布）
         * @param markerFixedFrame marker实际创建的frame，默认为"world"（接收到的current_pose会转换到这个frame下）
         * @param publishRate 连续发布频率，默认为20Hz
         * @param disableAutoUpdateStates 禁用自动更新的状态值数组，默认为{3}（OCS2状态）
         * @param markerUpdateInterval 最小marker更新间隔（秒），默认为0.05秒（20Hz）
         * 注意：头部控制相关参数（enable_head_control、head_link_name、head_joint_to_rpy_mapping等）在 initialize() 中从配置文件读取
         */
        ArmsTargetManager(
            rclcpp::Node::SharedPtr node,
            bool dualArmMode = false,
            std::string  frameId = "world",
            std::string  markerFixedFrame = "base_footprint",
            double publishRate = 20.0,
            const std::vector<int32_t>& disableAutoUpdateStates = {3},
            double markerUpdateInterval = 0.05);

        ~ArmsTargetManager() = default;

        /**
         * 初始化interactive marker
         */
        void initialize();

        /**
         * 设置marker位置
         * @param armType 手臂类型 ("left" 或 "right")
         * @param position 位置
         * @param orientation 方向
         */
        void setMarkerPose(
            const std::string& armType,
            const geometry_msgs::msg::Point& position,
            const geometry_msgs::msg::Quaternion& orientation);

        /**
         * 基于增量更新marker位置
         * @param armType 手臂类型 ("left" 或 "right")
         * @param positionDelta 已缩放的位置增量 [x, y, z]
         * @param rpyDelta 已缩放的旋转增量 [roll, pitch, yaw]
         */
        void updateMarkerPoseIncremental(
            const std::string& armType,
            const std::array<double, 3>& positionDelta,
            const std::array<double, 3>& rpyDelta);

        /**
         * 获取marker位置
         * @param armType 手臂类型 ("left" 或 "right")
         * @return pose消息
         */
        geometry_msgs::msg::Pose getMarkerPose(const std::string& armType) const;

        /**
         * 切换发布模式
         */
        void togglePublishMode();

        /**
         * 获取当前发布模式
         * @return 当前模式
         */
        MarkerState getCurrentMode() const;

        /**
         * 发送目标位置（统一函数，根据marker类型区分手臂和头部）
         * @param marker_type marker类型：
         *                    "left_arm" - 只发送左臂目标位姿
         *                    "right_arm" - 只发送右臂目标位姿
         *                    "head" - 发送头部关节角度
         *                    空字符串或默认 - 发送手臂目标位姿
         */
        void sendTargetPose(const std::string& marker_type = "arm");

        /**
         * 同时发送双臂目标位姿（仅双臂模式）
         */
        void sendDualArmTargetPose();

        /**
         * 检查指定状态是否在禁用列表中
         * @param state 要检查的状态值
         * @return 如果状态在禁用列表中返回true
         */
        bool isStateDisabled(int32_t state) const;

        /**
         * 通用的节流检查函数
         * @param last_time 最后执行时间（会被更新）
         * @param interval 最小间隔（秒）
         * @return 如果应该执行返回true
         */
        bool shouldThrottle(rclcpp::Time& last_time, double interval);

        /**
         * 标记有待应用的更改（由定时器定期检查并应用）
         */
        void markPendingChanges();

        /**
         * 定时器回调：定期检查并应用 marker 更新
         */
        void markerUpdateTimerCallback();

        /**
         * FSM 命令回调函数（用于状态切换和 marker 形态更新）
         * @param msg FSM 命令消息
         */
        void fsmCommandCallback(std_msgs::msg::Int32::ConstSharedPtr msg);

        /**
         * 设置 current pose 回调函数（用于通知外部收到原始 current_pose 消息，如 VRInputHandler）
         * @param armType 手臂类型 ("left" 或 "right")
         * @param callback current pose 回调函数
         */
        void setCurrentPoseCallback(
            const std::string& armType,
            std::function<void(const geometry_msgs::msg::PoseStamped::ConstSharedPtr&)> callback);

    private:
        /**
         * 从当前状态构建 interactive marker（适配器函数）
         * 
         * 这是一个适配器函数，内部会自动：
         * 1. 根据 markerType 获取对应的 marker（left_arm_marker_, right_arm_marker_, head_marker_）
         * 2. 收集当前状态（current_mode_, current_controller_state_）
         * 3. 调用对应的 marker 类的 createMarker 方法
         * 
         * @param name marker名称
         * @param markerType marker类型 ("left_arm", "right_arm", "head", 或其他自定义类型)
         * @return interactive marker消息
         * @note 实际创建逻辑在 MarkerFactory 中实现
         */
        visualization_msgs::msg::InteractiveMarker buildMarker(
            const std::string& name,
            const std::string& markerType) const;

        /**
         * 统一的marker反馈回调处理函数
         * 根据marker名称自动分发到对应的处理逻辑
         * @param feedback 反馈消息
         */
        void handleMarkerFeedback(
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);




        /**
         * 设置菜单系统
         */
        void setupMenu();

        /**
         * 通用的菜单设置辅助函数
         * @param menu_handler 菜单处理器（输出参数）
         * @param send_handle 发送菜单项句柄（输出参数）
         * @param toggle_handle 切换菜单项句柄（输出参数）
         * @param sendCallback 发送目标的回调函数
         */
        void setupMarkerMenu(
            std::shared_ptr<interactive_markers::MenuHandler>& menu_handler,
            interactive_markers::MenuHandler::EntryHandle& send_handle,
            interactive_markers::MenuHandler::EntryHandle& toggle_handle,
            std::function<void()> sendCallback);

        /**
         * 为双臂模式添加"发送双臂"菜单项
         * @param menu_handler 菜单处理器（必须已初始化）
         * @param both_handle 发送双臂菜单项句柄（输出参数）
         */
        void setupDualArmMenu(
            std::shared_ptr<interactive_markers::MenuHandler>& menu_handler,
            interactive_markers::MenuHandler::EntryHandle& both_handle);

        /**
         * 更新marker形状（统一管理左臂、右臂和头部marker）
         */
        void updateMarkerShape();

        /**
         * 更新菜单可见性
         */
        void updateMenuVisibility();

        /**
         * 创建所有发布器和订阅器（根据配置统一创建）
         * 注意：左臂和右臂的订阅器现在在 ArmMarker 内部管理
         */
        void createPublishersAndSubscribers();

        /**
         * 头部的 marker 自动更新回调函数（头部专用）
         * @param msg JointState 消息（用于头部）
         */
        void updateHeadMarkerFromTopic(
            const sensor_msgs::msg::JointState::ConstSharedPtr& joint_msg);

        /**
         * 将pose从源frame_id转换到指定的目标frame_id
         * 默认使用最新的可用变换（tf2::TimePointZero）
         * @param pose 要转换的pose（在源frame_id下）
         * @param sourceFrameId 源frame_id
         * @param targetFrameId 目标frame_id
         * @return 转换后的pose（在targetFrameId下），如果转换失败或frame相同则返回原始pose
         */
        geometry_msgs::msg::Pose transformPose(
            const geometry_msgs::msg::Pose& pose,
            const std::string& sourceFrameId,
            const std::string& targetFrameId) const;


        // 核心成员
        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

        // Marker 工厂（用于创建 marker）
        std::shared_ptr<MarkerFactory> marker_factory_;

        // 订阅器（注意：左臂和右臂的订阅器现在在 ArmMarker 内部管理）
        rclcpp::Subscription<arms_ros2_control_msgs::msg::Inputs>::SharedPtr control_input_subscription_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr head_joint_state_subscription_;
        
        // 发布器：双臂目标（仅双臂模式）
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr dual_target_stamped_publisher_;

        // 菜单系统
        std::shared_ptr<interactive_markers::MenuHandler> left_menu_handler_;
        std::shared_ptr<interactive_markers::MenuHandler> right_menu_handler_;
        std::shared_ptr<interactive_markers::MenuHandler> head_menu_handler_;

        // 菜单句柄
        interactive_markers::MenuHandler::EntryHandle left_send_handle_{};
        interactive_markers::MenuHandler::EntryHandle left_toggle_handle_{};
        interactive_markers::MenuHandler::EntryHandle left_both_handle_{};  // 发送双臂按钮（仅双臂模式）
        interactive_markers::MenuHandler::EntryHandle right_send_handle_{};
        interactive_markers::MenuHandler::EntryHandle right_toggle_handle_{};
        interactive_markers::MenuHandler::EntryHandle right_both_handle_{};  // 发送双臂按钮（仅双臂模式）
        interactive_markers::MenuHandler::EntryHandle head_send_handle_{};
        interactive_markers::MenuHandler::EntryHandle head_toggle_handle_{};

        // 配置
        bool dual_arm_mode_;
        std::string control_base_frame_; // 目标frame，marker会转换到这个frame下发布
        std::string marker_fixed_frame_; // marker实际创建的frame，接收到的current_pose会转换到这个frame下
        double publish_rate_;

        // TF2相关
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // 状态管理
        MarkerState current_mode_ = MarkerState::SINGLE_SHOT;

        // 状态管理
        mutable std::mutex state_update_mutex_;
        int32_t current_controller_state_ = 2; // 当前控制器状态（命令值）
        std::string current_fsm_state_ = "HOLD"; // 当前FSM状态（字符串，用于状态转换验证）
        std::vector<int32_t> disable_auto_update_states_; // 禁用自动更新的状态值数组

        // 更新节流
        rclcpp::Time last_marker_update_time_;
        double marker_update_interval_; // 最小更新间隔（秒）
        rclcpp::Time last_publish_time_; // 连续发布模式的最后发布时间
        
        // 定时器机制：定期检查并应用 marker 更新
        rclcpp::TimerBase::SharedPtr marker_update_timer_;
        std::atomic<bool> pending_changes_{false}; // 标记是否有待应用的更改

        // Marker 管理（封装所有 marker 相关逻辑）
        std::shared_ptr<ArmMarker> left_arm_marker_;
        std::shared_ptr<ArmMarker> right_arm_marker_;
        std::shared_ptr<HeadMarker> head_marker_;
    };
} // namespace arms_ros2_control::command
