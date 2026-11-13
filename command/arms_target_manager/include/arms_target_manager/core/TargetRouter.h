//
// TargetRouter - 目标汇聚与调度。
//
// 所有输入源（VR、RViz 拖拽、摇杆/键盘、程序调用）都只提交 Intent，
// 由这里决定“谁可以写”“写到哪里”“什么时候显示”。
//
// 内部由三块拼成，各自只有一件事：
//   ChannelOwnership  谁在写、还能写多久
//   PoseTargetSink    怎么发、怎么显示、怎么平滑
//   本类              把两者串起来，并处理双臂合成与状态报告
//
#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>

#include "arms_target_manager/core/ChannelOwnership.h"
#include "arms_target_manager/core/PoseTargetSink.h"
#include "arms_target_manager/core/TeleopTypes.h"

namespace arms_ros2_control::command
{
    class TargetRouter
    {
    public:
        struct Config
        {
            std::string control_frame{"world"};
            std::string marker_frame{"base_link"};
            bool dual_arm{false};
            double publish_rate{20.0};
        };

        TargetRouter(rclcpp::Node::SharedPtr node,
                     std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                     Config config);

        /** 创建发布器、回显订阅与到期检查，并启动 1 Hz 状态日志。 */
        void start(rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr left_target,
                   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_stamped,
                   rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr right_target,
                   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_stamped);

        // ---- 提交与交还 ----

        Verdict submit(const Intent& intent);

        void release(Channel channel, SourceId source);
        void releaseAll(SourceId source);

        void setBlocked(Channel channel, SourceId source, bool blocked);
        [[nodiscard]] bool isBlocked(Channel channel, SourceId source) const;
        [[nodiscard]] bool canWrite(Channel channel, SourceId source) const;

        // ---- 发送与显示 ----

        bool publishNow(Channel channel, bool stamped);
        bool publishDual(bool include_body);

        void setView(Channel channel, PoseTargetSink::ViewFn view);
        void clearView(Channel channel);

        bool refreshViewFromLastCommand(Channel channel);
        bool refreshViewFromReported(Channel channel);
        void clearEchoHold(Channel channel);

        /** 坐标系/拓扑变了：清掉所有通道的缓存，避免用旧值当锚点。 */
        void invalidate();

        // ---- 查询 ----

        std::optional<geometry_msgs::msg::Pose> lastCommand(Channel channel) const;
        std::optional<geometry_msgs::msg::Pose> lastPublished(Channel channel) const;
        [[nodiscard]] Ownership ownership(Channel channel) const;
        [[nodiscard]] bool hasPublisher(Channel channel) const;
        [[nodiscard]] TeleopStatus status() const;

        [[nodiscard]] const std::string& controlFrame() const { return config_.control_frame; }
        [[nodiscard]] const std::string& markerFrame() const { return config_.marker_frame; }

        static const char* channelName(Channel channel);
        static const char* sourceName(SourceId source);

    private:
        struct ChannelState
        {
            ChannelPolicy policy;
            PoseTargetSink sink;
            std::uint64_t accepted{0};
            std::uint64_t rejected{0};
            const char* last_reason{"ok"};

            ChannelState(const Config& config, Channel channel)
                : policy{}
                  , sink(config.control_frame, config.marker_frame)
            {
                (void)channel;
            }
        };

        ChannelState& state(Channel channel) { return channels_[static_cast<size_t>(channel)]; }
        const ChannelState& state(Channel channel) const
        {
            return channels_[static_cast<size_t>(channel)];
        }

        bool transformBetween(const geometry_msgs::msg::Pose& pose,
                              const std::string& from,
                              const std::string& to,
                              geometry_msgs::msg::Pose& out) const;
        void onReportedTarget(Channel channel, const geometry_msgs::msg::PoseStamped& msg);
        void logStatusIfChanged();

        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        Config config_;

        std::array<ChannelState, static_cast<size_t>(Channel::Count)> channels_;
        ChannelOwnership ownership_;

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr dual_publisher_;
        rclcpp::TimerBase::SharedPtr expiry_timer_;
        rclcpp::TimerBase::SharedPtr status_timer_;
        TeleopStatus last_logged_{};
    };

} // namespace arms_ros2_control::command
