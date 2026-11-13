//
// ChannelOwnership 实现：控制权的申请、续期、到期回收与屏蔽表。
//
// 关键点：
//   * 每条通道独立仲裁，互不影响；
//   * 同级冲突返回 busy，低优先级返回 lower_priority，
//     对方是手动拖动时额外区分为 manual_override，便于上层提示；
//   * 到期回收由 TargetRouter 的 100 ms 定时器驱动，不靠调用方显式释放。
//
// 注意：
//   本文件不做任何日志输出；需要现场排查请调 ownership() 取快照。
//
#include "arms_target_manager/core/ChannelOwnership.h"

namespace arms_ros2_control::command
{
    namespace
    {
        uint8_t priorityValue(Priority priority)
        {
            return static_cast<uint8_t>(priority);
        }
    } // namespace

    ChannelOwnership::Entry& ChannelOwnership::entry(Channel channel)
    {
        return entries_[static_cast<size_t>(channel)];
    }

    const ChannelOwnership::Entry& ChannelOwnership::entry(Channel channel) const
    {
        return entries_[static_cast<size_t>(channel)];
    }

    /**
     * @brief 申请（或续期）某通道的控制权
     *
     * 同级已被别人占着时返回 busy；对方优先级更高时返回 lower_priority
     * （对方是手动拖动时额外区分为 manual_override，便于上层提示）。
     *
     * @param channel 目标通道
     * @param source 申请方
     * @param priority 优先级，数值越大越优先
     * @param lease 本次控制权保留时长，过期自动释放
     * @param now 当前时间
     * @return 是否得到控制权；被拒时 reason 说明原因
     */
    Verdict ChannelOwnership::claim(
        Channel channel, SourceId source, Priority priority,
        std::chrono::milliseconds lease,
        std::chrono::steady_clock::time_point now)
    {
        if (source == SourceId::None)
        {
            return {false, "no_source"};
        }

        auto& e = entry(channel);
        if (isBlocked(channel, source))
        {
            return {false, "blocked"};
        }

        if (e.own.owner != SourceId::None && now >= e.own.until)
        {
            e.own = Ownership{};
        }

        if (e.own.owner != SourceId::None && e.own.owner != source)
        {
            const uint8_t incoming = priorityValue(priority);
            const uint8_t current = priorityValue(e.own.priority);
            if (incoming < current)
            {
                const bool manual = current == priorityValue(Priority::High);
                return {false, manual ? "manual_override" : "lower_priority"};
            }
            if (incoming == current)
            {
                return {false, "busy"};
            }
        }

        e.own.owner = source;
        e.own.priority = priority;
        e.own.until = now + lease;
        e.own.active = true;
        return {true, "ok"};
    }

    /**
     * @brief 主动释放某通道的控制权
     * @param channel 目标通道
     * @param source 释放方；不是当前持有者时不做任何事
     */
    void ChannelOwnership::release(Channel channel, SourceId source)
    {
        auto& e = entry(channel);
        if (e.own.owner == source)
        {
            e.own = Ownership{};
        }
    }

    /**
     * @brief 释放该来源持有的全部通道（来源失能时一次性清理）
     * @param source 释放方
     */
    void ChannelOwnership::releaseAll(SourceId source)
    {
        for (size_t i = 0; i < static_cast<size_t>(Channel::Count); ++i)
        {
            release(static_cast<Channel>(i), source);
        }
    }

    /**
     * @brief 屏蔽/解除屏蔽某来源对某通道的写入
     *
     * 屏蔽时如果正是它在写，会同时收回控制权，避免屏蔽后仍然发得出去。
     */
    void ChannelOwnership::setBlocked(Channel channel, SourceId source, bool blocked)
    {
        auto& e = entry(channel);
        e.blocked[source] = blocked;
        if (blocked && e.own.owner == source)
        {
            e.own = Ownership{};
        }
    }

    bool ChannelOwnership::isBlocked(Channel channel, SourceId source) const
    {
        const auto& e = entry(channel);
        const auto it = e.blocked.find(source);
        return it != e.blocked.end() && it->second;
    }

    /**
     * @brief 某来源现在能不能写这条通道
     *
     * 被屏蔽或正被别人持有（且未过期）时为 false；空闲或自己持有时为 true。
     */
    bool ChannelOwnership::canWrite(Channel channel, SourceId source,
                                    std::chrono::steady_clock::time_point now) const
    {
        if (isBlocked(channel, source))
        {
            return false;
        }
        const auto& e = entry(channel);
        if (e.own.owner == SourceId::None || now >= e.own.until)
        {
            return true;
        }
        return e.own.owner == source;
    }

    /**
     * @brief 取控制权快照，并按当前时间重算“是否有效”
     */
    Ownership ChannelOwnership::ownership(Channel channel,
                                         std::chrono::steady_clock::time_point now) const
    {
        const auto& e = entry(channel);
        Ownership out = e.own;
        out.active = out.owner != SourceId::None && now < out.until;
        return out;
    }

    /**
     * @brief 回收所有已过期的控制权
     * @return true 表示至少回收了一条（调用方可以据此做一次现场清理）
     */
    bool ChannelOwnership::expire(std::chrono::steady_clock::time_point now)
    {
        bool changed = false;
        for (auto& e : entries_)
        {
            if (e.own.owner != SourceId::None && now >= e.own.until)
            {
                e.own = Ownership{};
                changed = true;
            }
        }
        return changed;
    }

    SourceId ChannelOwnership::currentOwner(Channel channel) const
    {
        return entry(channel).own.owner;
    }

} // namespace arms_ros2_control::command
