//
// Created for Arms ROS2 Control - ArmsTargetManager
//
#pragma once


#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <array>
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
     * ArmsTargetManager - 机械臂目标管理器
     * 
     * 提供3D交互式marker，用于设置机械臂末端执行器的目标pose
     * 支持单臂和双臂模式，包含右键菜单和连续发布功能
     * 
     * 发布主题: 
     * - {topicPrefix}_left_target: 左臂目标pose
     * - {topicPrefix}_right_target: 右臂目标pose
     * 
     * 单臂机器人默认使用左臂主题
     */
    class ArmsTargetManager
    {
    public:
        /**
         * 构造函数
         * @param node ROS节点指针
         * @param topicPrefix 主题前缀
         * @param dualArmMode 是否为双臂模式
         * @param frameId 坐标系ID，默认为"world"（目标frame，marker会转换到这个frame下发布）
         * @param markerFixedFrame marker实际创建的frame，默认为"world"（接收到的current_pose会转换到这个frame下）
         * @param publishRate 连续发布频率，默认为20Hz
         * @param disableAutoUpdateStates 禁用自动更新的状态值数组，默认为{3}（OCS2状态）
         * @param markerUpdateInterval 最小marker更新间隔（秒），默认为0.05秒（20Hz）
         * @param enableHeadControl 是否启用头部控制，默认为false
         * @param headMarkerFrame 头部marker的坐标系，默认为"base_footprint"
         * @param headControllerName 头部控制器名称，默认为"head_joint_controller"
         * @param headMarkerPosition 头部marker在base_footprint中的固定位置，默认为[1.0, 0.0, 1.5]
         */
        ArmsTargetManager(
            rclcpp::Node::SharedPtr node,
            const std::string& topicPrefix,
            bool dualArmMode = false,
            const std::string& frameId = "world",
            const std::string& markerFixedFrame = "base_link",
            double publishRate = 20.0,
            const std::vector<int32_t>& disableAutoUpdateStates = {3},
            double markerUpdateInterval = 0.05,
            bool enableHeadControl = true,
            const std::string& headMarkerFrame = "base_footprint",
            const std::string& headControllerName = "head_joint_controller",
            const std::array<double, 3>& headMarkerPosition = {1.0, 0.0, 1.5});

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
         * 发送目标pose（单次发布模式）
         */
        void sendTargetPose();



        /**
         * 启用/禁用基于状态的自动marker更新
         * @param enable 是否启用
         */
        void setAutoUpdateEnabled(bool enable);

        /**
         * 获取当前自动更新状态
         * @return 是否启用自动更新
         */
        bool isAutoUpdateEnabled() const;

        /**
         * 检查指定状态是否在禁用列表中
         * @param state 要检查的状态值
         * @return 如果状态在禁用列表中返回true
         */
        bool isStateDisabled(int32_t state) const;

        /**
         * 节流更新marker（避免频繁更新导致RViz警告）
         * @return 如果应该更新marker返回true
         */
        bool shouldUpdateMarker();

        /**
         * 控制输入回调函数
         * @param msg 控制输入消息
         */
        void controlInputCallback(arms_ros2_control_msgs::msg::Inputs::ConstSharedPtr msg);

        /**
         * 发送头部目标关节位置（单次发布模式）
         */
        void sendHeadTargetJointPosition();

    private:
        /**
         * 创建interactive marker
         * @param name marker名称
         * @param armType 手臂类型
         * @return interactive marker消息
         */
        visualization_msgs::msg::InteractiveMarker createMarker(
            const std::string& name,
            const std::string& armType) const;

        /**
         * 创建box marker
         * @param color 颜色 ("blue", "red", "grey")
         * @return marker消息
         */
        visualization_msgs::msg::Marker createBoxMarker(const std::string& color = "grey") const;

        /**
         * 添加移动控制
         * @param interactiveMarker 交互marker
         */
        void addMovementControls(visualization_msgs::msg::InteractiveMarker& interactiveMarker) const;

        /**
         * 左臂marker反馈回调
         * @param feedback 反馈消息
         */
        void leftMarkerCallback(
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);

        /**
         * 右臂marker反馈回调
         * @param feedback 反馈消息
         */
        void rightMarkerCallback(
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);

        /**
         * 头部marker反馈回调
         * @param feedback 反馈消息
         */
        void headMarkerCallback(
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);

        /**
         * 创建头部interactive marker
         * @return interactive marker消息
         */
        visualization_msgs::msg::InteractiveMarker createHeadMarker() const;

        /**
         * 创建箭头marker（用于头部）
         * @param color 颜色
         * @return marker消息
         */
        visualization_msgs::msg::Marker createArrowMarker(const std::string& color = "green") const;

        /**
         * 设置头部菜单系统
         */
        void setupHeadMenu();

        /**
         * 从quaternion提取头部关节角度（yaw和pitch）
         * @param quaternion 四元数
         * @return [head_joint1_angle, head_joint2_angle] 关节角度（弧度）
         */
        std::vector<double> quaternionToHeadJointAngles(
            const geometry_msgs::msg::Quaternion& quaternion) const;


        /**
         * 设置菜单系统
         */
        void setupMenu();

        /**
         * 更新marker形状
         */
        void updateMarkerShape();

        /**
         * 更新菜单可见性
         */
        void updateMenuVisibility();


        /**
         * 创建球体marker
         * @param color 颜色
         * @return marker消息
         */
        visualization_msgs::msg::Marker createSphereMarker(const std::string& color = "grey") const;


        /**
         * 左臂末端执行器位置回调函数
         * @param msg 位置消息
         */
        void leftEndEffectorPoseCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

        /**
         * 右臂末端执行器位置回调函数
         * @param msg 位置消息
         */
        void rightEndEffectorPoseCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

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
        
        // 发布器
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr left_pose_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr right_pose_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr head_joint_publisher_;
        
        // 订阅器
        rclcpp::Subscription<arms_ros2_control_msgs::msg::Inputs>::SharedPtr control_input_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_end_effector_pose_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_end_effector_pose_subscription_;
        
        // 菜单系统
        std::shared_ptr<interactive_markers::MenuHandler> left_menu_handler_;
        std::shared_ptr<interactive_markers::MenuHandler> right_menu_handler_;
        std::shared_ptr<interactive_markers::MenuHandler> head_menu_handler_;
        
        // 菜单句柄
        interactive_markers::MenuHandler::EntryHandle left_send_handle_;
        interactive_markers::MenuHandler::EntryHandle left_toggle_handle_;
        interactive_markers::MenuHandler::EntryHandle right_send_handle_;
        interactive_markers::MenuHandler::EntryHandle right_toggle_handle_;
        interactive_markers::MenuHandler::EntryHandle head_send_handle_;
        
        // 配置
        std::string topic_prefix_;
        bool dual_arm_mode_;
        std::string control_base_frame_;  // 目标frame，marker会转换到这个frame下发布
        std::string marker_fixed_frame_;  // marker实际创建的frame，接收到的current_pose会转换到这个frame下
        double publish_rate_;
        
        // TF2相关
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        
        // 状态管理
        MarkerState current_mode_;
        
        // 当前pose状态
        geometry_msgs::msg::Pose left_pose_;
        geometry_msgs::msg::Pose right_pose_;
        geometry_msgs::msg::Pose head_pose_;
        
        // 状态管理
        mutable std::mutex state_update_mutex_;
        int32_t current_controller_state_;  // 当前控制器状态
        bool auto_update_enabled_;         // 是否启用自动更新
        std::vector<int32_t> disable_auto_update_states_; // 禁用自动更新的状态值数组
        
        // 更新节流
        rclcpp::Time last_marker_update_time_;
        double marker_update_interval_;  // 最小更新间隔（秒）
        
        // 头部控制相关
        bool enable_head_control_;  // 是否启用头部控制
        std::string head_marker_frame_;  // 头部marker的坐标系
        std::string head_controller_name_;  // 头部控制器名称
        std::array<double, 3> head_marker_position_;  // marker在base_footprint中的固定位置
        
    };

} // namespace arms_ros2_control::command
