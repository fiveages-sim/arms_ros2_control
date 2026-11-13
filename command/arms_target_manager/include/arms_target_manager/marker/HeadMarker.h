//
// HeadMarker - 头部 marker 的显示。
//
// 作用：
//   维护头部 6 自由度目标的 marker：位置与姿态都可以通过拖拽下发，
//   并在实测/回传位姿到来时刷新显示。
//
// 特殊点：
//   syncPoseFromTf() 用 TF 把 marker 初始化到头部位置，
//   避免 marker 一出现先跳到默认位姿再被拉过去。
//
// 注意：
//   marker 只在 enable_wbc_head_tracking_marker 打开时才建话题，
//   目标发布同样由 core/PoseTargetSink 负责。
//
#pragma once

#include <memory>
#include <string>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace arms_ros2_control::command
{
    class MarkerFactory;
    enum class MarkerState;

    /**
     * @brief WBC Head 6D Marker：发布 head_target / head_target/stamped，
     *        并从 head_current_target 回写最终目标。
     */
    class HeadMarker
    {
    public:
        /** 位姿变化后通知上层刷新显示：参数为 marker 名与显示系下的位姿。 */
        using UpdateCallback =
            std::function<void(const std::string&, const geometry_msgs::msg::Pose&)>;
        /** 是否允许用控制器回传的目标刷新显示。 */
        using StateCheckCallback = std::function<bool()>;

        /**
         * @brief 构造头部 marker
         * @param node ROS 节点
         * @param marker_factory marker 外形工厂
         * @param tf_buffer TF 缓存
         * @param frame_id marker 显示坐标系
         * @param control_base_frame 下发目标时的坐标系
         * @param publish_rate 连续发送频率 [Hz]
         * @param update_callback 显示刷新回调，可为空
         */
        HeadMarker(
            rclcpp::Node::SharedPtr node,
            std::shared_ptr<MarkerFactory> marker_factory,
            std::shared_ptr<tf2_ros::Buffer> tf_buffer,
            const std::string& frame_id,
            const std::string& control_base_frame,
            double publish_rate = 20.0,
            UpdateCallback update_callback = nullptr);

        /**
         * @brief 读取参数、按需创建发布器与回显订阅。
         *
         * 只有在 enable_wbc_head_tracking_marker 打开时才真正建立话题，
         * 因此关闭时不会在 RViz 里出现多余 marker。
         */
        void initialize();

        [[nodiscard]] bool isEnabled() const { return enable_wbc_head_tracking_marker_; }
        [[nodiscard]] bool isWbcEnabled() const { return enable_wbc_head_tracking_marker_; }

        /**
         * @brief 生成 RViz 交互 marker 描述
         * @param name marker 名
         * @param pose 初始位姿（显示系）
         * @param enable_interaction false 时生成不可拖动的静态 marker
         * @param mode 单次/连续发布模式，决定外观
         * @return 可直接插入 InteractiveMarkerServer 的描述
         */
        visualization_msgs::msg::InteractiveMarker createMarker(
            const std::string& name,
            const geometry_msgs::msg::Pose& pose,
            bool enable_interaction,
            MarkerState mode) const;

        [[nodiscard]] geometry_msgs::msg::Pose getPose() const { return head_pose_; }
        void setPose(const geometry_msgs::msg::Pose& pose) { head_pose_ = pose; }

        /**
         * @brief 把当前 marker 位姿下发到头部目标话题
         * @param force true = 忽略节流立即发送
         * @param use_stamped true = 发带时间戳的版本（控制器走插值）
         * @return true 表示本次真的发出了
         */
        bool publishTargetPose(bool force = false, bool use_stamped = false);

        /**
         * @brief 用 TF 初始化 marker 位姿（marker 固定系 → 头部 link）
         *
         * 让 marker 一出现就在头部位置，而不是先跳到默认原点。
         *
         * @return true 表示 TF 可用且已写入位姿
         */
        bool syncPoseFromTf();
        void setWbcTargetStateCheckCallback(StateCheckCallback callback)
        {
            wbc_target_state_check_callback_ = std::move(callback);
        }

        /** 清除“刚下发过命令”的冷却，使下一个回显位姿可以立即接管显示。 */
        void clearCommandCooldown();

        [[nodiscard]] std::string getLinkName() const { return head_link_name_; }

    private:
        bool shouldThrottle(double interval) const;
        bool transformPose(const geometry_msgs::msg::Pose& pose,
                           const std::string& source_frame,
                           const std::string& target_frame,
                           geometry_msgs::msg::Pose& result) const;
        void updateWbcTargetFromTopic(
            const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);

        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<MarkerFactory> marker_factory_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::string frame_id_;
        std::string control_base_frame_;

        bool enable_wbc_head_tracking_marker_ = false;
        std::string head_link_name_;

        geometry_msgs::msg::Pose head_pose_;

        double publish_rate_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_publisher_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_target_subscription_;
        geometry_msgs::msg::PoseStamped latest_current_target_;
        bool has_latest_current_target_{false};
        UpdateCallback update_callback_;
        StateCheckCallback wbc_target_state_check_callback_;

        mutable rclcpp::Time last_publish_time_;
        mutable rclcpp::Time last_marker_command_time_;
    };
} // namespace arms_ros2_control::command
