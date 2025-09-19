//
// Created for Arms ROS2 Control - VRInputHandler
//

#ifndef VR_INPUT_HANDLER_H
#define VR_INPUT_HANDLER_H

#include <memory>
#include <string>
#include <mutex>
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
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
         * @param updateRate 更新频率，默认为500Hz
         */
        VRInputHandler(
            rclcpp::Node::SharedPtr node,
            ArmsTargetManager* targetManager,
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
         * 检查节点是否存在
         * @param node ROS节点指针
         * @param targetNodeName 目标节点名称
         * @return true如果节点存在，false否则
         */
        bool checkNodeExists(const std::shared_ptr<rclcpp::Node>& node, const std::string& targetNodeName);

    private:
        /**
         * VR左臂pose回调函数
         * @param msg VR pose消息
         */
        void vrLeftCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        /**
         * VR右臂pose回调函数
         * @param msg VR pose消息
         */
        void vrRightCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        /**
         * 右摇杆回调函数
         * @param msg 布尔消息，表示摇杆按下状态
         */
        void rightThumbstickCallback(const std_msgs::msg::Bool::SharedPtr msg);

        /**
         * 机器人左臂当前pose回调函数
         * @param msg 机器人pose消息
         */
        void robotLeftPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        /**
         * 机器人右臂当前pose回调函数
         * @param msg 机器人pose消息
         */
        void robotRightPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        /**
         * 更新marker位置
         * @param armType 手臂类型 ("left" 或 "right")
         * @param position 位置
         * @param orientation 方向
         */
        void updateMarkerPose(const std::string& armType, 
                             const Eigen::Vector3d& position, 
                             const Eigen::Quaterniond& orientation);

        /**
         * 将PoseStamped消息转换为Eigen::Matrix4d
         * @param msg PoseStamped消息
         * @return 4x4变换矩阵
         */
        Eigen::Matrix4d poseMsgToMatrix(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

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

        // 订阅器
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_left_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_right_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_right_thumbstick_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_robot_left_pose_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_robot_right_pose_;

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
        std::atomic<bool> is_update_mode_;  // true = 更新模式, false = 存储模式
        std::atomic<bool> last_thumbstick_state_;
        std::mutex state_mutex_;

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

        // 常量
        static const std::string XR_NODE_NAME;
        static const double POSITION_THRESHOLD;
        static const double ORIENTATION_THRESHOLD;
    };

} // namespace arms_ros2_control::command

#endif // VR_INPUT_HANDLER_H
