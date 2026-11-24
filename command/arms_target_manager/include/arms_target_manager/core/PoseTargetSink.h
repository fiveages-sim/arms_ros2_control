//
// 输出面：一个目标通道的发送实现。
//
// 作用：
//   把“控制坐标系下的一个目标位姿”变成真正发出去的消息，并维护
//   与之相关的显示与缓存。左臂 / 右臂 / 头 / 胸共用同一份代码，
//   以后新增被控目标也只是再绑一个 sink。
//
// 一个 sink 内部维护四件事：
//   最后一帧命令、最后一帧发布、控制器回传值、以及回显抑制与交接过渡状态。
//
// 注意：
//   坐标系换算由外部注入（TransformFn），本文件不依赖 TF，便于单独测试；
//   发布出去的目标始终是控制坐标系下的值，不做二次换算。
//
#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "arms_target_manager/detail/PoseFilters.h"

namespace arms_ros2_control::command
{
    class PoseTargetSink
    {
    public:
        /** 坐标系换算：从 from 到 to。返回 false 表示当前不可用。 */
        using TransformFn = std::function<bool(
            const geometry_msgs::msg::Pose&, const std::string& from,
            const std::string& to, geometry_msgs::msg::Pose& out)>;
        /** 显示回调：参数为 marker 坐标系下的位姿。 */
        using ViewFn = std::function<void(const geometry_msgs::msg::Pose&)>;

        struct Options
        {
            bool stamped{false};
            bool filter_stale{false};
            bool manual{false};              // 手动输入：占住回显
            std::chrono::milliseconds handover{0};
            std::chrono::milliseconds echo_hold{1000};
        };

        PoseTargetSink(std::string control_frame, std::string marker_frame);

        void bind(rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub,
                  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr stamped_pub,
                  TransformFn transform, rclcpp::Node* node);
        void setView(ViewFn view) { view_ = std::move(view); }
        void clearView() { view_ = nullptr; }

        [[nodiscard]] bool hasPublisher() const
        {
            return static_cast<bool>(pose_pub_) || static_cast<bool>(stamped_pub_);
        }

        /** 发送一个控制坐标系下的目标。 */
        bool publish(const geometry_msgs::msg::Pose& control_pose, const Options& options);

        /** 重发最近一次目标（菜单“发送目标”）。 */
        bool republish(const Options& options);

        /** 只记录目标，不发送（预览模式）。 */
        void recordCommand(const geometry_msgs::msg::Pose& control_pose);

        /** 从当前位置开始一段平滑过渡（交还控制权时使用）。 */
        void startHandover(std::chrono::milliseconds duration);

        /** 手动输入占住回显：期间控制器回传不刷新显示。 */
        void holdEcho(std::chrono::milliseconds duration);

        /** 控制器回传的目标；在回显抑制期内只记录不显示。 */
        void onReported(const geometry_msgs::msg::PoseStamped& msg);

        void clearEchoHold() { echo_hold_until_ = std::chrono::steady_clock::time_point{}; }

        [[nodiscard]] const std::optional<geometry_msgs::msg::Pose>& lastCommand() const
        {
            return last_command_;
        }
        [[nodiscard]] const std::optional<geometry_msgs::msg::Pose>& lastPublished() const
        {
            return last_published_;
        }

        /** 用最近一次目标刷新显示（切到连续发布时使用）。 */
        bool refreshViewFromLastCommand();
        /** 用最近一次回传目标刷新显示。 */
        bool refreshViewFromReported();

        /** 坐标系/拓扑变化时清缓存。 */
        void invalidate();

    private:
        void notifyView(const geometry_msgs::msg::Pose& control_pose);

        std::string control_frame_;
        std::string marker_frame_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr stamped_pub_;
        TransformFn transform_;
        rclcpp::Node* node_{nullptr};
        ViewFn view_;

        std::optional<geometry_msgs::msg::Pose> last_command_;
        std::optional<geometry_msgs::msg::Pose> last_published_;
        std::optional<geometry_msgs::msg::PoseStamped> last_reported_;

        bool has_handover_{false};
        geometry_msgs::msg::Pose handover_from_;
        std::chrono::steady_clock::time_point handover_start_{};
        std::chrono::milliseconds handover_duration_{0};

        std::chrono::steady_clock::time_point echo_hold_until_{};

        detail::StaleCatchUpFilter stale_{2};
    };

} // namespace arms_ros2_control::command
