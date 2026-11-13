//
// TargetRouter 实现：换算 → 仲裁 → 发送 → 显示 → 状态日志。
//
// 这里只应出现“通用规则”，任何与 VR / 全身模式相关的判断都不该写进来。
//
#include "arms_target_manager/core/TargetRouter.h"

#include <algorithm>
#include <utility>

#include <tf2/exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace arms_ros2_control::command
{
    namespace
    {
        constexpr std::array<const char*, static_cast<size_t>(Channel::Count)> kTopics{
            "left_current_target", "right_current_target",
            "head_current_target", "body_current_target"};
    } // namespace

    TargetRouter::TargetRouter(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        Config config)
        : node_(std::move(node))
          , tf_buffer_(std::move(tf_buffer))
          , config_(std::move(config))
          , channels_{
                ChannelState{config_, Channel::ArmLeft},
                ChannelState{config_, Channel::ArmRight},
                ChannelState{config_, Channel::Head},
                ChannelState{config_, Channel::Body}}
    {
    }

    void TargetRouter::start(
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr left_target,
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_stamped,
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr right_target,
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_stamped)
    {
        auto transform = [this](const geometry_msgs::msg::Pose& pose,
                                const std::string& from,
                                const std::string& to,
                                geometry_msgs::msg::Pose& out)
        {
            return transformBetween(pose, from, to, out);
        };

        state(Channel::ArmLeft).sink.bind(
            std::move(left_target), std::move(left_stamped), transform, node_.get());
        state(Channel::ArmRight).sink.bind(
            std::move(right_target), std::move(right_stamped), transform, node_.get());

        auto& head = state(Channel::Head);
        head.sink.bind(
            node_->create_publisher<geometry_msgs::msg::Pose>("head_target", 10),
            node_->create_publisher<geometry_msgs::msg::PoseStamped>("head_target/stamped", 10),
            transform, node_.get());

        auto& body = state(Channel::Body);
        body.sink.bind(
            node_->create_publisher<geometry_msgs::msg::Pose>("body_target", 10),
            node_->create_publisher<geometry_msgs::msg::PoseStamped>("body_target/stamped", 10),
            transform, node_.get());

        if (config_.dual_arm)
        {
            dual_publisher_ =
                node_->create_publisher<nav_msgs::msg::Path>("dual_target/stamped", 1);
        }

        for (size_t i = 0; i < kTopics.size(); ++i)
        {
            const auto channel = static_cast<Channel>(i);
            node_->create_subscription<geometry_msgs::msg::PoseStamped>(
                kTopics[i], 10,
                [this, channel](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
                {
                    if (msg)
                    {
                        onReportedTarget(channel, *msg);
                    }
                });
        }

        expiry_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]()
            {
                if (ownership_.expire(std::chrono::steady_clock::now()))
                {
                    logStatusIfChanged();
                }
            });

        status_timer_ = node_->create_wall_timer(
            std::chrono::seconds(1), [this]() { logStatusIfChanged(); });

        RCLCPP_INFO(
            node_->get_logger(),
            "🎯 TargetRouter 已启动：control=%s, marker=%s, dual_arm=%s",
            config_.control_frame.c_str(), config_.marker_frame.c_str(),
            config_.dual_arm ? "true" : "false");
    }

    Verdict TargetRouter::submit(const Intent& intent)
    {
        auto& st = state(intent.channel);
        if (!st.sink.hasPublisher())
        {
            st.rejected++;
            st.last_reason = "no_publisher";
            return {false, st.last_reason};
        }

        geometry_msgs::msg::Pose requested;
        requested.position.x = intent.position.x();
        requested.position.y = intent.position.y();
        requested.position.z = intent.position.z();
        requested.orientation.x = intent.orientation.x();
        requested.orientation.y = intent.orientation.y();
        requested.orientation.z = intent.orientation.z();
        requested.orientation.w = intent.orientation.w();

        geometry_msgs::msg::Pose pose;
        if (!transformBetween(requested, intent.frame, config_.control_frame, pose))
        {
            st.rejected++;
            st.last_reason = "no_tf";
            return {false, st.last_reason};
        }
        if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
            !std::isfinite(pose.position.z) || !std::isfinite(pose.orientation.w))
        {
            st.rejected++;
            st.last_reason = "invalid_pose";
            return {false, st.last_reason};
        }

        const auto now = std::chrono::steady_clock::now();
        const auto lease = intent.lease.count() > 0 ? intent.lease : st.policy.lease;
        const SourceId previous_owner = ownership_.currentOwner(intent.channel);

        const Verdict claimed =
            ownership_.claim(intent.channel, intent.source, intent.priority, lease, now);
        if (!claimed.accepted)
        {
            st.rejected++;
            st.last_reason = claimed.reason;
            logStatusIfChanged();
            return claimed;
        }

        if (previous_owner != intent.source && intent.source != SourceId::Rviz)
        {
            // 自动控制重新接管：从最后一次实际发出的位置平滑过渡。
            st.sink.startHandover(st.policy.handover_ramp);
        }

        st.sink.recordCommand(pose);

        if (intent.source == SourceId::Rviz)
        {
            st.sink.holdEcho(st.policy.echo_hold_after_manual);
        }

        if (!intent.publish)
        {
            st.accepted++;
            st.last_reason = "preview";
            return {true, st.last_reason};
        }

        PoseTargetSink::Options options;
        options.stamped = intent.one_shot;
        options.filter_stale = intent.filter_stale;
        options.manual = intent.source == SourceId::Rviz;
        options.handover = st.policy.handover_ramp;
        options.echo_hold = st.policy.echo_hold_after_manual;

        if (!st.sink.publish(pose, options))
        {
            st.rejected++;
            st.last_reason = "publish_failed";
            return {false, st.last_reason};
        }

        st.accepted++;
        st.last_reason = "ok";
        logStatusIfChanged();
        return {true, st.last_reason};
    }

    void TargetRouter::release(Channel channel, SourceId source)
    {
        const bool was_owner = ownership_.currentOwner(channel) == source;
        ownership_.release(channel, source);
        if (was_owner)
        {
            // 显式交还后不再保留这条来源的旧目标，避免菜单发到过期值。
            state(channel).sink.invalidate();
            logStatusIfChanged();
        }
    }

    void TargetRouter::releaseAll(SourceId source)
    {
        for (size_t i = 0; i < static_cast<size_t>(Channel::Count); ++i)
        {
            release(static_cast<Channel>(i), source);
        }
    }

    void TargetRouter::setBlocked(Channel channel, SourceId source, bool blocked)
    {
        ownership_.setBlocked(channel, source, blocked);
    }

    bool TargetRouter::isBlocked(Channel channel, SourceId source) const
    {
        return ownership_.isBlocked(channel, source);
    }

    bool TargetRouter::canWrite(Channel channel, SourceId source) const
    {
        return ownership_.canWrite(channel, source, std::chrono::steady_clock::now());
    }

    bool TargetRouter::publishNow(Channel channel, bool stamped)
    {
        auto& st = state(channel);
        PoseTargetSink::Options options;
        options.stamped = stamped;
        options.echo_hold = st.policy.echo_hold_after_manual;
        if (!st.sink.republish(options))
        {
            return false;
        }
        if (ownership_.currentOwner(channel) == SourceId::Rviz)
        {
            st.sink.holdEcho(st.policy.echo_hold_after_manual);
        }
        return true;
    }

    bool TargetRouter::publishDual(bool include_body)
    {
        if (!dual_publisher_)
        {
            return false;
        }
        const auto& left = state(Channel::ArmLeft).sink.lastPublished();
        const auto& right = state(Channel::ArmRight).sink.lastPublished();
        if (!left || !right)
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "双臂目标尚未生成，忽略 dual_target 发送");
            return false;
        }

        nav_msgs::msg::Path path;
        path.header.stamp = node_->now();
        path.header.frame_id = config_.control_frame;

        geometry_msgs::msg::PoseStamped item;
        item.header = path.header;
        item.pose = *left;
        path.poses.push_back(item);
        item.pose = *right;
        path.poses.push_back(item);

        if (include_body)
        {
            const auto& body = state(Channel::Body).sink.lastPublished();
            if (body)
            {
                item.pose = *body;
                path.poses.push_back(item);
            }
        }

        dual_publisher_->publish(path);
        return true;
    }

    void TargetRouter::setView(Channel channel, PoseTargetSink::ViewFn view)
    {
        state(channel).sink.setView(std::move(view));
    }

    void TargetRouter::clearView(Channel channel)
    {
        state(channel).sink.clearView();
    }

    bool TargetRouter::refreshViewFromLastCommand(Channel channel)
    {
        return state(channel).sink.refreshViewFromLastCommand();
    }

    bool TargetRouter::refreshViewFromReported(Channel channel)
    {
        return state(channel).sink.refreshViewFromReported();
    }

    void TargetRouter::clearEchoHold(Channel channel)
    {
        state(channel).sink.clearEchoHold();
    }

    void TargetRouter::invalidate()
    {
        for (auto& st : channels_)
        {
            st.sink.invalidate();
        }
    }

    std::optional<geometry_msgs::msg::Pose> TargetRouter::lastCommand(Channel channel) const
    {
        return state(channel).sink.lastCommand();
    }

    std::optional<geometry_msgs::msg::Pose> TargetRouter::lastPublished(Channel channel) const
    {
        return state(channel).sink.lastPublished();
    }

    Ownership TargetRouter::ownership(Channel channel) const
    {
        return ownership_.ownership(channel, std::chrono::steady_clock::now());
    }

    bool TargetRouter::hasPublisher(Channel channel) const
    {
        return state(channel).sink.hasPublisher();
    }

    TeleopStatus TargetRouter::status() const
    {
        TeleopStatus out;
        for (size_t i = 0; i < static_cast<size_t>(Channel::Count); ++i)
        {
            const auto channel = static_cast<Channel>(i);
            const auto& st = state(channel);
            const Ownership own =
                ownership_.ownership(channel, std::chrono::steady_clock::now());
            auto& entry = out.channels[i];
            entry.channel = channel;
            entry.owner = own.owner;
            entry.priority = own.priority;
            entry.active = own.active;
            entry.blocked_for_control_input =
                ownership_.isBlocked(channel, SourceId::ControlInput);
            entry.has_target = st.sink.lastCommand().has_value();
            entry.accepted = st.accepted;
            entry.rejected = st.rejected;
            entry.last_reason = st.last_reason;
        }
        return out;
    }

    const char* TargetRouter::channelName(Channel channel)
    {
        switch (channel)
        {
            case Channel::ArmLeft:
                return "left_arm";
            case Channel::ArmRight:
                return "right_arm";
            case Channel::Head:
                return "head";
            case Channel::Body:
                return "body";
            case Channel::Count:
                break;
        }
        return "unknown";
    }

    const char* TargetRouter::sourceName(SourceId source)
    {
        switch (source)
        {
            case SourceId::None:
                return "none";
            case SourceId::Vr:
                return "vr";
            case SourceId::Rviz:
                return "rviz";
            case SourceId::ControlInput:
                return "control_input";
            case SourceId::External:
                return "external";
        }
        return "unknown";
    }

    bool TargetRouter::transformBetween(
        const geometry_msgs::msg::Pose& pose,
        const std::string& from,
        const std::string& to,
        geometry_msgs::msg::Pose& out) const
    {
        if (from.empty() || to.empty() || from == to)
        {
            out = pose;
            return !from.empty() || !to.empty();
        }
        try
        {
            geometry_msgs::msg::PoseStamped input;
            input.header.frame_id = from;
            input.header.stamp = rclcpp::Time(0);
            input.pose = pose;
            const auto transform =
                tf_buffer_->lookupTransform(to, from, tf2::TimePointZero);
            geometry_msgs::msg::PoseStamped transformed;
            tf2::doTransform(input, transformed, transform);
            out = transformed.pose;
            return true;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "目标坐标变换失败 %s -> %s：%s", from.c_str(), to.c_str(), ex.what());
            return false;
        }
    }

    void TargetRouter::onReportedTarget(
        Channel channel, const geometry_msgs::msg::PoseStamped& msg)
    {
        state(channel).sink.onReported(msg);
    }

    void TargetRouter::logStatusIfChanged()
    {
        const TeleopStatus current = status();
        for (size_t i = 0; i < static_cast<size_t>(Channel::Count); ++i)
        {
            const auto& now = current.channels[i];
            const auto& was = last_logged_.channels[i];
            if (now.owner == was.owner && now.active == was.active &&
                now.last_reason == was.last_reason)
            {
                continue;
            }
            RCLCPP_INFO(
                node_->get_logger(),
                "🎯 %s：控制来源=%s，有效=%s，原因=%s（接受 %llu / 拒绝 %llu）",
                channelName(now.channel), sourceName(now.owner),
                now.active ? "是" : "否", now.last_reason,
                static_cast<unsigned long long>(now.accepted),
                static_cast<unsigned long long>(now.rejected));
        }
        last_logged_ = current;
    }

} // namespace arms_ros2_control::command
