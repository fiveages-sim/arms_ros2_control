//
// BaseMarker - 底盘跟随目标 marker 的显示。
//
// 作用：
//   保存底盘 marker 的显示位姿，把 RViz 拖动事件换算到 marker 显示坐标系，
//   并在进入跟随模式时用 TF 把 marker 放到当前底盘位置。
//
// 输入：TF（marker 显示系 → 底盘 link），用于初始化显示位置。
// 输出：只改显示与缓存，不直接发目标——目标由 core/TargetRouter 统一发。
//
// 注意：
//   底盘模型只有平面位姿，marker 只提供 x/y 平移与偏航旋转，
//   不提供 z 平移和横滚/俯仰手柄。
//
#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <tf2_ros/buffer.hpp>

namespace arms_ros2_control::command
{
    class MarkerFactory;
    enum class MarkerState;

    /**
     * @brief 底盘平面目标 marker：显示、TF 初始化与拖动换算。
     */
    class BaseMarker
    {
    public:
        /**
         * @brief 构造底盘 marker
         * @param node ROS 节点
         * @param marker_factory marker 外形工厂
         * @param tf_buffer TF 缓存，用于初始化显示位姿
         * @param frame_id marker 显示坐标系（移动底盘通常是 world）
         */
        BaseMarker(
            rclcpp::Node::SharedPtr node,
            std::shared_ptr<MarkerFactory> marker_factory,
            std::shared_ptr<tf2_ros::Buffer> tf_buffer,
            const std::string& frame_id);

        /** 读取参数（base_link_name），建立 TF 初始化所需的名称。 */
        void initialize();

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
         * @brief 用 TF 把 marker 放到当前底盘位置
         * @return true 表示 TF 可用且已写入位姿
         */
        bool syncPoseFromTf();

        void setPose(const geometry_msgs::msg::Pose& pose) { pose_ = pose; }
        [[nodiscard]] geometry_msgs::msg::Pose getPose() const { return pose_; }

        [[nodiscard]] std::string getMarkerName() const { return "base_target"; }
        [[nodiscard]] std::string getDescription() const { return "Base Target"; }

    private:
        geometry_msgs::msg::Pose transformPose(
            const geometry_msgs::msg::Pose& pose,
            const std::string& source_frame_id,
            const std::string& target_frame_id) const;

        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<MarkerFactory> marker_factory_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

        std::string frame_id_;
        std::string base_link_name_{"base_footprint"};

        geometry_msgs::msg::Pose pose_;
    };
} // namespace arms_ros2_control::command
