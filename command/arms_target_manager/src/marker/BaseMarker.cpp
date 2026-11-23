//
// BaseMarker 实现：显示、TF 初始位姿同步与拖动换算。
//
// 注意：
//   目标发布由 core/TargetRouter 负责，本文件只维护显示。
//
#include "arms_target_manager/marker/BaseMarker.h"

#include "arms_target_manager/MarkerFactory.h"

#include <cmath>

#include <tf2/exceptions.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace arms_ros2_control::command
{

BaseMarker::BaseMarker(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<MarkerFactory> marker_factory,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::string& frame_id)
    : node_(std::move(node))
    , marker_factory_(std::move(marker_factory))
    , tf_buffer_(std::move(tf_buffer))
    , frame_id_(frame_id)
{
    pose_.position.x = 0.0;
    pose_.position.y = 0.0;
    pose_.position.z = 0.0;
    pose_.orientation.w = 1.0;
}

/**
 * @brief 读取底盘 link 名称
 *
 * 进入跟随模式时用它把 marker 放到当前底盘位置，避免 marker 先跳到原点。
 */
void BaseMarker::initialize()
{
    node_->declare_parameter<std::string>("base_link_name", base_link_name_);
    base_link_name_ = node_->get_parameter("base_link_name").as_string();
    RCLCPP_INFO(node_->get_logger(),
                "底盘 marker 使用的 link 名称: %s", base_link_name_.c_str());
}

/**
 * @brief 生成底盘 marker（黄色，平面三自由度）
 */
visualization_msgs::msg::InteractiveMarker BaseMarker::createMarker(
    const std::string& name,
    MarkerState mode,
    bool enable_interaction) const
{
    return marker_factory_->createBaseMarker(
        name,
        "Base Target",
        pose_,
        "yellow",
        mode,
        enable_interaction);
}

/**
 * @brief 把 RViz 拖动事件换算到 marker 显示坐标系
 */
geometry_msgs::msg::Pose BaseMarker::handleFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback,
    const std::string& source_frame_id) const
{
    return transformPose(feedback->pose, source_frame_id, frame_id_);
}

/**
 * @brief 用 TF 把 marker 放到当前底盘位置
 */
bool BaseMarker::syncPoseFromTf()
{
    if (base_link_name_.empty())
    {
        return false;
    }
    try
    {
        const auto transform = tf_buffer_->lookupTransform(
            frame_id_, base_link_name_, tf2::TimePointZero);
        pose_.position.x = transform.transform.translation.x;
        pose_.position.y = transform.transform.translation.y;
        pose_.position.z = transform.transform.translation.z;
        // 底盘只有平面位姿：只保留偏航，去掉横滚/俯仰，
        // 否则 marker 的 x/y 控制轴会随本体倾斜，看起来像在拖 z 轴或绕 y 轴旋转。
        const double yaw = tf2::getYaw(transform.transform.rotation);
        pose_.orientation.x = 0.0;
        pose_.orientation.y = 0.0;
        pose_.orientation.z = std::sin(yaw * 0.5);
        pose_.orientation.w = std::cos(yaw * 0.5);
        return true;
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "底盘 marker 无法用 TF %s -> %s 初始化: %s",
            frame_id_.c_str(), base_link_name_.c_str(), ex.what());
        return false;
    }
}

/**
 * @brief 坐标系换算；失败时打限频 WARN 并返回未换算的原值
 */
geometry_msgs::msg::Pose BaseMarker::transformPose(
    const geometry_msgs::msg::Pose& pose,
    const std::string& source_frame_id,
    const std::string& target_frame_id) const
{
    if (source_frame_id == target_frame_id)
    {
        return pose;
    }

    try
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = source_frame_id;
        pose_stamped.header.stamp = rclcpp::Time(0);
        pose_stamped.pose = pose;

        const geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
            target_frame_id, source_frame_id, tf2::TimePointZero);

        geometry_msgs::msg::PoseStamped result_stamped;
        tf2::doTransform(pose_stamped, result_stamped, transform);
        return result_stamped.pose;
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000,
            "底盘 marker 位姿无法从 '%s' 换算到 '%s': %s",
            source_frame_id.c_str(), target_frame_id.c_str(), ex.what());
        return pose;
    }
}

} // namespace arms_ros2_control::command
