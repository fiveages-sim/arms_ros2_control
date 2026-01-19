//
// Created for Arms ROS2 Control - VRInputHandler
//
#pragma once

#include <memory>
#include <string>
#include <mutex>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <std_msgs/msg/int32.hpp>
#include "arms_target_manager/ArmsTargetManager.h"

namespace arms_ros2_control::command
{
    /**
     * VRInputHandler - VR输入处理器Wrapper
     * 
     * 基于VRMarkerWrapper功能，适配arms_target_manager
     * 实现VR pose订阅、状态切换机制和marker更新功能
     * 
     * 功能特性:
     * - 支持存储模式和更新模式切换
     * - 基于left_thumbstick的状态切换
     * - VR pose变化检测和频闪优化
     * - 与ArmsTargetManager的集成
     */
    class VRInputHandler
    {
    public:
        /**
         * 构造函数
         * @param node ROS节点指针
         * @param targetManager ArmsTargetManager指针
         * @param pub_left_target 外部传入的左臂目标位姿发布器（统一管理）
         * @param pub_right_target 外部传入的右臂目标位姿发布器（统一管理）
         * @param updateRate 更新频率，默认为500Hz
         */
        VRInputHandler(
            rclcpp::Node::SharedPtr node,
            ArmsTargetManager* targetManager,
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_left_target,
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_right_target,
            double updateRate = 500.0);

        ~VRInputHandler() = default;

        /**
         * 启用VR控制
         */
        void enable();

        /**
         * 禁用VR控制
         */
        void disable();

        /**
         * 检查VR控制是否启用
         * @return true如果启用，false否则
         */
        bool isEnabled() const { return enabled_.load(); }

        /**
         * 检查是否处于镜像模式
         * @return true如果启用镜像模式，false否则
         */
        bool isMirrorMode() const { return mirror_mode_.load(); }

        /**
         * 检查节点是否存在
         * @param node ROS节点指针
         * @param targetNodeName 目标节点名称
         * @return true如果节点存在，false否则
         */
        bool checkNodeExists(const std::shared_ptr<rclcpp::Node>& node, const std::string& targetNodeName);

        /**
         * FSM命令回调函数（用于跟踪FSM状态）
         * 由外部统一订阅后调用，避免重复订阅
         * @param msg FSM命令消息
         */
        void fsmCommandCallback(std_msgs::msg::Int32::SharedPtr msg);

        /**
         * 机器人左臂当前pose回调函数
         * 由外部统一订阅后调用，避免重复订阅
         * @param msg 机器人pose消息
         */
        void robotLeftPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

        /**
         * 机器人右臂当前pose回调函数
         * 由外部统一订阅后调用，避免重复订阅
         * @param msg 机器人pose消息
         */
        void robotRightPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

    private:
        /**
         * VR左臂pose回调函数
         * @param msg VR pose消息
         */
        void vrLeftCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

        /**
         * VR右臂pose回调函数
         * @param msg VR pose消息
         */
        void vrRightCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

        /**
         * 右摇杆回调函数
         * @param msg 布尔消息，表示摇杆按下状态
         */
        void rightThumbstickCallback(std_msgs::msg::Bool::SharedPtr msg);

        /**
         * 左摇杆回调函数
         * @param msg 布尔消息，表示摇杆按下状态
         */
        void leftThumbstickCallback(std_msgs::msg::Bool::SharedPtr msg);

        /**
         * 左摇杆轴值回调函数
         * @param msg 摇杆轴值消息 (x, y, z)
         */
        void leftThumbstickAxesCallback(geometry_msgs::msg::Point::SharedPtr msg);

        /**
         * 右摇杆轴值回调函数
         * @param msg 摇杆轴值消息 (x, y, z)
         */
        void rightThumbstickAxesCallback(geometry_msgs::msg::Point::SharedPtr msg);

        /**
         * 左握把按钮回调函数
         * @param msg 握把按钮状态消息
         */
        void leftGripCallback(std_msgs::msg::Bool::SharedPtr msg);

        /**
         * 右握把按钮回调函数
         * @param msg 握把按钮状态消息
         */
        void rightGripCallback(std_msgs::msg::Bool::SharedPtr msg);

        /**
         * 左Y按键回调函数（用于设置左臂基准位姿）
         * @param msg Y按键状态消息
         */
        void leftYButtonCallback(std_msgs::msg::Bool::SharedPtr msg);

        /**
         * 右B按键回调函数（用于设置右臂基准位姿）
         * @param msg B按键状态消息
         */
        void rightBButtonCallback(std_msgs::msg::Bool::SharedPtr msg);

        /**
         * 更新marker位置（已废弃，保留用于兼容）
         * @param armType 手臂类型 ("left" 或 "right")
         * @param position 位置
         * @param orientation 方向
         */
        void updateMarkerPose(const std::string& armType,
                              const Eigen::Vector3d& position,
                              const Eigen::Quaterniond& orientation);

        /**
         * 直接发布目标位姿到left_target/right_target话题（解耦模式，无坐标转换）
         * @param armType 手臂类型 ("left" 或 "right")
         * @param position 位置（已在目标坐标系下）
         * @param orientation 方向
         */
        void publishTargetPoseDirect(const std::string& armType,
                                     const Eigen::Vector3d& position,
                                     const Eigen::Quaterniond& orientation);

        /**
         * 将PoseStamped消息转换为Eigen::Matrix4d
         * @param msg PoseStamped消息
         * @return 4x4变换矩阵
         */
        Eigen::Matrix4d poseMsgToMatrix(geometry_msgs::msg::PoseStamped::SharedPtr msg);

        /**
         * 从4x4矩阵提取位置和方向
         * @param matrix 4x4变换矩阵
         * @param position 输出位置
         * @param orientation 输出方向
         */
        void matrixToPosOri(const Eigen::Matrix4d& matrix,
                            Eigen::Vector3d& position,
                            Eigen::Quaterniond& orientation);

        /**
         * 检查pose是否发生显著变化
         * @param currentPos 当前位置
         * @param currentOri 当前方向
         * @param prevPos 之前位置
         * @param prevOri 之前方向
         * @return true如果pose发生显著变化
         */
        bool hasPoseChanged(const Eigen::Vector3d& currentPos,
                            const Eigen::Quaterniond& currentOri,
                            const Eigen::Vector3d& prevPos,
                            const Eigen::Quaterniond& prevOri);

        /**
         * 基于差值计算pose并应用到机器人base pose
         * @param vrCurrentPos 当前VR位置
         * @param vrCurrentOri 当前VR方向
         * @param vrBasePos VR base位置
         * @param vrBaseOri VR base方向
         * @param robotBasePos 机器人base位置
         * @param robotBaseOri 机器人base方向
         * @param resultPos 输出计算的位置
         * @param resultOri 输出计算的方向
         */
        void calculatePoseFromDifference(const Eigen::Vector3d& vrCurrentPos,
                                         const Eigen::Quaterniond& vrCurrentOri,
                                         const Eigen::Vector3d& vrBasePos,
                                         const Eigen::Quaterniond& vrBaseOri,
                                         const Eigen::Vector3d& robotBasePos,
                                         const Eigen::Quaterniond& robotBaseOri,
                                         Eigen::Vector3d& resultPos,
                                         Eigen::Quaterniond& resultOri);

        // ROS组件
        rclcpp::Node::SharedPtr node_;
        ArmsTargetManager* target_manager_;

        // 发布器（用于直接发布到left_target/right_target）
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_left_target_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_right_target_;

        // 订阅器
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_left_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_right_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_right_thumbstick_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_left_thumbstick_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_left_thumbstick_axes_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_right_thumbstick_axes_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_left_grip_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_right_grip_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_left_y_button_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_right_b_button_;
        // 机器人 current_pose 订阅已移除，改为在 arms_target_manager_node 中统一处理
        // FSM命令订阅已移除，改为在 arms_target_manager_node 中统一处理

        // VR pose参数
        Eigen::Matrix4d left_ee_pose_ = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d right_ee_pose_ = Eigen::Matrix4d::Identity();

        // VR位置和方向参数
        Eigen::Vector3d left_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond left_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d right_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond right_orientation_ = Eigen::Quaterniond::Identity();

        // 用于变化检测的之前pose（更新模式）
        Eigen::Vector3d prev_calculated_left_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond prev_calculated_left_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d prev_calculated_right_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond prev_calculated_right_orientation_ = Eigen::Quaterniond::Identity();

        // 用于变化检测的之前VR pose（存储模式）
        Eigen::Vector3d prev_vr_left_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond prev_vr_left_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d prev_vr_right_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond prev_vr_right_orientation_ = Eigen::Quaterniond::Identity();

        // 状态管理
        std::atomic<bool> enabled_;
        std::atomic<bool> is_update_mode_; // true = 更新模式, false = 存储模式
        std::atomic<bool> last_thumbstick_state_;
        std::atomic<bool> mirror_mode_; // true = 镜像模式, false = 正常模式
        std::atomic<bool> last_left_thumbstick_state_;
        std::atomic<bool> last_left_grip_state_; // 左握把按钮上次状态
        std::atomic<bool> last_right_grip_state_; // 右握把按钮上次状态
        std::atomic<bool> last_left_y_button_state_; // 左Y按键上次状态
        std::atomic<bool> last_right_b_button_state_; // 右B按键上次状态
        std::atomic<bool> left_arm_paused_; // 左臂是否暂停更新（Y按键控制）
        std::atomic<bool> right_arm_paused_; // 右臂是否暂停更新（B按键控制）
        std::atomic<bool> left_grip_mode_; // 左摇杆控制模式：false=XY平移, true=Z轴+Yaw
        std::atomic<bool> right_grip_mode_; // 右摇杆控制模式：false=XY平移, true=Z轴+Yaw
        std::mutex state_mutex_;
        std::atomic<int32_t> current_fsm_state_; // 当前FSM状态：1=HOME, 2=HOLD, 3=OCS2, 100=REST

        // 时间控制
        rclcpp::Time last_update_time_;
        double update_rate_;

        // 当前VR位置和方向（用于兼容性）
        Eigen::Vector3d current_position_;
        Eigen::Quaterniond current_orientation_;

        // VR base poses（摇杆按下时存储）
        Eigen::Vector3d vr_base_left_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond vr_base_left_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d vr_base_right_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond vr_base_right_orientation_ = Eigen::Quaterniond::Identity();

        // 机器人base poses（摇杆按下时存储）
        Eigen::Vector3d robot_base_left_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond robot_base_left_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d robot_base_right_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond robot_base_right_orientation_ = Eigen::Quaterniond::Identity();

        // 当前机器人poses
        Eigen::Vector3d robot_current_left_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond robot_current_left_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d robot_current_right_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond robot_current_right_orientation_ = Eigen::Quaterniond::Identity();

        // 摇杆轴值（归一化 -1.0 ~ 1.0）
        Eigen::Vector2d left_thumbstick_axes_ = Eigen::Vector2d::Zero();
        Eigen::Vector2d right_thumbstick_axes_ = Eigen::Vector2d::Zero();

        // 摇杆累积偏移量（米）
        Eigen::Vector3d left_thumbstick_offset_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d right_thumbstick_offset_ = Eigen::Vector3d::Zero();

        // 摇杆累积Yaw旋转（弧度）
        double left_thumbstick_yaw_offset_ = 0.0;
        double right_thumbstick_yaw_offset_ = 0.0;

        // 常量
        static const std::string XR_NODE_NAME;
        static const double POSITION_THRESHOLD;
        static const double ORIENTATION_THRESHOLD;
        static const double LINEAR_SCALE; // 摇杆位置缩放因子
        static const double ANGULAR_SCALE; // 摇杆旋转缩放因子
    };
} // namespace arms_ros2_control::command
