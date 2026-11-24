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
        /** 路由配置：目标坐标系、marker 坐标系、是否双臂、发布频率。 */
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

        /**
         * @brief 建立发布器、回显订阅与控制权到期检查
         *
         * 双臂的两条发布器由外部传入（与其他模块共用），头/胸通道在内部创建。
         *
         * @param left_target 左臂连续目标发布器
         * @param left_stamped 左臂单次目标发布器
         * @param right_target 右臂连续目标发布器（单臂模式可为空）
         * @param right_stamped 右臂单次目标发布器（单臂模式可为空）
         */
        void start(rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr left_target,
                   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_stamped,
                   rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr right_target,
                   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_stamped);

        // ---- 提交与交还 ----

        /**
         * @brief 提交一个目标意图
         *
         * 顺序固定：坐标系换算 → 合法性检查 → 申请控制权 → 发布/预览。
         *
         * @param intent 来源、通道、优先级、租约、坐标系与位姿
         * @return 是否接受；不接受时 reason 说明原因
         */
        Verdict submit(const Intent& intent);

        /**
         * @brief 主动交还某通道的控制权
         * @param channel 目标通道
         * @param source 交还方；不是当前持有者时不做任何事
         */
        void release(Channel channel, SourceId source);

        /**
         * @brief 交还该来源持有的所有通道
         * @param source 交还方（例如 VR 失能时一次性清掉）
         */
        void releaseAll(SourceId source);

        /**
         * @brief 屏蔽或解除屏蔽某个来源对某通道的写入
         *
         * 用于“这条通道现在不该被它写”的场景（例如全身跟踪期间屏蔽速度流）。
         */
        void setBlocked(Channel channel, SourceId source, bool blocked);
        [[nodiscard]] bool isBlocked(Channel channel, SourceId source) const;
        [[nodiscard]] bool canWrite(Channel channel, SourceId source) const;

        // ---- 发送与显示 ----

        /**
         * @brief 把最近缓存的目标立即发一次
         * @param channel 目标通道
         * @param stamped true = 发带时间戳的版本（控制器走插值）
         * @return false 表示没有可发目标或发布器缺失
         */
        bool publishNow(Channel channel, bool stamped);

        /**
         * @brief 双臂合成一条 Path 下发
         * @param include_body true 时把胸部目标一并写进 Path
         * @return false 表示左右臂目标尚未就绪
         */
        bool publishDual(bool include_body);

        /** 注册显示回调：目标更新时把位姿交回 marker（marker 自己不发布）。 */
        void setView(Channel channel, PoseTargetSink::ViewFn view);
        void clearView(Channel channel);

        /** 用最近一次命令刷新显示，返回是否真的刷新了。 */
        bool refreshViewFromLastCommand(Channel channel);
        /** 用控制器回传的实际目标刷新显示，返回是否真的刷新了。 */
        bool refreshViewFromReported(Channel channel);
        /** 结束回显抑制，允许控制器回传值立即接管显示。 */
        void clearEchoHold(Channel channel);

        /** 坐标系/拓扑变了：清掉所有通道的缓存，避免用旧值当锚点。 */
        void invalidate();

        // ---- 查询 ----

        /** 最近一次收到的命令位姿（控制坐标系），没有则为空。 */
        std::optional<geometry_msgs::msg::Pose> lastCommand(Channel channel) const;
        /** 最近一次真正发出去的位姿（控制坐标系），没有则为空。 */
        std::optional<geometry_msgs::msg::Pose> lastPublished(Channel channel) const;
        /** 当前控制权快照（谁在写、租约是否有效）。 */
        [[nodiscard]] Ownership ownership(Channel channel) const;
        /** 该通道是否已绑定发布器（单臂模式下的右臂为 false）。 */
        [[nodiscard]] bool hasPublisher(Channel channel) const;
        /** 全部通道的汇总状态，供诊断与调试使用。 */
        [[nodiscard]] TeleopStatus status() const;

        [[nodiscard]] const std::string& controlFrame() const { return config_.control_frame; }
        [[nodiscard]] const std::string& markerFrame() const { return config_.marker_frame; }

        /** 通道名（left_arm / right_arm / head / body），用于日志与界面。 */
        static const char* channelName(Channel channel);
        /** 来源名（vr / rviz / control_input / ...），用于日志与界面。 */
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

        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        Config config_;

        std::array<ChannelState, static_cast<size_t>(Channel::Count)> channels_;
        ChannelOwnership ownership_;

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr dual_publisher_;
        rclcpp::TimerBase::SharedPtr expiry_timer_;
    };

} // namespace arms_ros2_control::command
