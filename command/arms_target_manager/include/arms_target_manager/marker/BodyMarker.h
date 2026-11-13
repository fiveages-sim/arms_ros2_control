//
// BodyMarker - 胸部（腰部）marker 的显示。
//
// 作用：
//   保存胸部 marker 的当前显示位姿，把 RViz 拖动事件换算到显示坐标系，
//   并在实测位姿 / 控制器回传目标到来时刷新显示。
//
// 输入：<controller>/current_pose（实测位姿，约 30 Hz 节流后用于显示）。
// 输出：只改显示与缓存，不直接发目标——目标由 core/TargetRouter 统一发。
//
// 注意：
//   刚下发过命令的 1 秒内不采用回显位姿，否则拖动时 marker 会被拉回去。
//
#pragma once

#include <memory>
#include <string>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.hpp>

#include "arms_target_manager/MarkerFactory.h"

namespace arms_ros2_control::command
{

class BodyMarker
{
public:
    /** 拖动/回显后通知上层刷新 marker 显示：参数为 marker 名与 marker 固定系下的位姿。 */
    using UpdateCallback =
        std::function<void(const std::string& marker_name, const geometry_msgs::msg::Pose& pose)>;

    /** 是否允许用实测位姿自动刷新 marker 显示（由上层按 FSM/模式决定）。 */
    using StateCheckCallback =
        std::function<bool()>;

    /** 是否允许用控制器目标位姿刷新 marker 显示（拖动后 1 秒内会被抑制）。 */
    using TargetStateCheckCallback =
        std::function<bool()>;

    /** 实测位姿回调，转发给上层使用（VR 需要它作为控制基准）。 */
    using CurrentPoseCallback =
        std::function<void(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg)>;

    /**
     * @brief 构造胸部 marker
     * @param node ROS 节点
     * @param marker_factory marker 外形工厂
     * @param tf_buffer TF 缓存，用于坐标系换算
     * @param frame_id marker 显示所在坐标系（通常是 world）
     * @param control_base_frame 下发目标时的坐标系（下游控制器的基准系）
     * @param target_publisher 连续发送时的目标发布器
     * @param target_stamped_publisher 单次发送时的目标发布器（带时间戳，会走插值）
     * @param current_pose_topic 实测位姿话题
     * @param current_target_topic 保留参数；目标回显已统一由 TargetRouter 处理
     * @param publish_rate 连续发送频率 [Hz]
     * @param update_callback 显示刷新回调，可为空
     */
    BodyMarker(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<MarkerFactory> marker_factory,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        const std::string& frame_id,
        const std::string& control_base_frame,
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_publisher,
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_stamped_publisher,
        const std::string& current_pose_topic,
        const std::string& current_target_topic,
        double publish_rate = 20.0,
        UpdateCallback update_callback = nullptr);

    /**
     * @brief 生成 RViz 交互 marker 描述
     * @param name marker 名，必须与反馈回调里判断的名字一致
     * @param mode 单次/连续发布模式，决定外观
     * @param enable_interaction false 时生成不可拖动的静态 marker
     * @return 可直接插入 InteractiveMarkerServer 的描述
     */
    visualization_msgs::msg::InteractiveMarker createMarker(
        const std::string& name,
        MarkerState mode,
        bool enable_interaction) const;

    /**
     * @brief 把 RViz 的拖动事件换算到 marker 显示坐标系
     * @param feedback RViz 交互反馈
     * @param source_frame_id 反馈位姿所在坐标系
     * @return marker 固定系下的新位姿；TF 失败时返回未换算的原值
     */
    geometry_msgs::msg::Pose handleFeedback(
        const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback,
        const std::string& source_frame_id) const;

    /**
     * @brief 用实测位姿刷新 marker 显示
     *
     * 内部节流到约 30 Hz，并受 state_check_callback_ 门控。
     *
     * @param pose_msg 实测位姿（带 frame_id）
     */
    void updateFromTopic(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg);

    /**
     * @brief 用控制器回传的目标位姿刷新 marker 显示
     *
     * 刚下发过命令的 1 秒内不覆盖，避免拖动时 marker 被回显拉回。
     *
     * @param pose_msg 控制器回传的目标位姿（带 frame_id）
     */
    void updateTargetFromTopic(
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg);

    void setUpdateCallback(UpdateCallback callback) { update_callback_ = std::move(callback); }
    void setStateCheckCallback(StateCheckCallback callback) { state_check_callback_ = std::move(callback); }
    void setTargetStateCheckCallback(TargetStateCheckCallback callback)
    {
        target_state_check_callback_ = std::move(callback);
    }
    void setCurrentPoseCallback(CurrentPoseCallback callback) { current_pose_callback_ = std::move(callback); }

    void setPose(const geometry_msgs::msg::Pose& pose) { pose_ = pose; }
    geometry_msgs::msg::Pose getPose() const { return pose_; }

    /**
     * @brief 把当前 marker 位姿下发到目标话题
     * @param force true = 单次发送（走 stamped，控制器会插值）；
     *              false = 连续发送（按 publish_rate 节流，不插值）
     * @return true 表示本次真的发出了
     */
    bool publishTargetPose(bool force = false);

    std::string getMarkerName() const { return "body_target"; }
    std::string getDescription() const { return "Body Target"; }

private:
    geometry_msgs::msg::Pose transformPose(
        const geometry_msgs::msg::Pose& pose,
        const std::string& source_frame_id,
        const std::string& target_frame_id) const;

    bool shouldThrottle(double interval) const;

    static void add6DofControls(
        visualization_msgs::msg::InteractiveMarker& int_marker,
        bool enable_interaction);

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<MarkerFactory> marker_factory_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    std::string frame_id_;
    std::string control_base_frame_;

    geometry_msgs::msg::Pose pose_;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr target_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_stamped_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        current_target_subscription_;

    geometry_msgs::msg::PoseStamped latest_current_target_pose_;
    bool has_latest_current_target_pose_{false};

    double publish_rate_;
    mutable rclcpp::Time last_publish_time_;
    mutable rclcpp::Time last_subscription_update_time_;
    mutable rclcpp::Time last_marker_command_time_;

    UpdateCallback update_callback_;
    StateCheckCallback state_check_callback_;
    TargetStateCheckCallback target_state_check_callback_;
    CurrentPoseCallback current_pose_callback_;
};

} // namespace arms_ros2_control::command
