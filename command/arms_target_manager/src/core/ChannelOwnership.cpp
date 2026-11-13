//
// ChannelOwnership 实现：控制权的申请、续期、到期回收与屏蔽表。
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

    void ChannelOwnership::release(Channel channel, SourceId source)
    {
        auto& e = entry(channel);
        if (e.own.owner == source)
        {
            e.own = Ownership{};
        }
    }

    void ChannelOwnership::releaseAll(SourceId source)
    {
        for (size_t i = 0; i < static_cast<size_t>(Channel::Count); ++i)
        {
            release(static_cast<Channel>(i), source);
        }
    }

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

    Ownership ChannelOwnership::ownership(Channel channel,
                                         std::chrono::steady_clock::time_point now) const
    {
        const auto& e = entry(channel);
        Ownership out = e.own;
        out.active = out.owner != SourceId::None && now < out.until;
        return out;
    }

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
