//
// 控制权：谁在写某个目标、还能写多久、新来的请求能不能插队。
//
// 作用：
//   每条目标通道一份仲裁记录，回答四件事：
//     claim    —— 我能不能现在开始写；
//     canWrite —— 我现在还写不写得动；
//     expire   —— 谁的超时了，该收回；
//     blocked  —— 谁被显式禁止写这条通道。
//
// 为什么单独成文件：
//   它是一条规则而不是一个功能：输入源、模式、UI 都要问同一个
//   “现在谁说了算”，所以只能有一份实现，不能各处各写一个布尔量。
//
// 注意：
//   控制权是带租约的：每提交一次续一次，停手超过租约自动交还，
//   调用方通常不需要显式 release。
//
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <map>

#include "arms_target_manager/core/TeleopTypes.h"

namespace arms_ros2_control::command
{
    class ChannelOwnership
    {
    public:
        struct Entry
        {
            Ownership own;
            std::map<SourceId, bool> blocked;
        };

        /**
         * 申请写入。
         * @return 接受时 accepted=true；拒绝时 reason 说明原因。
         */
        Verdict claim(Channel channel, SourceId source, Priority priority,
                      std::chrono::milliseconds lease,
                      std::chrono::steady_clock::time_point now);

        /** 显式交还（失能、退出模式）。不改变被屏蔽表。 */
        void release(Channel channel, SourceId source);
        void releaseAll(SourceId source);

        /** 屏蔽/解除屏蔽某个来源写某个目标（保留旧的 setArmVrActive 语义）。 */
        void setBlocked(Channel channel, SourceId source, bool blocked);
        [[nodiscard]] bool isBlocked(Channel channel, SourceId source) const;

        /** 只判断能否写，不改变控制权（速度类输入使用）。 */
        [[nodiscard]] bool canWrite(Channel channel, SourceId source,
                                    std::chrono::steady_clock::time_point now) const;

        [[nodiscard]] Ownership ownership(Channel channel,
                                          std::chrono::steady_clock::time_point now) const;

        /** 到期控制权回收，返回是否有变化。 */
        bool expire(std::chrono::steady_clock::time_point now);

        /** 当前持有者，用于“手动拖动抢占”这类判断。 */
        [[nodiscard]] SourceId currentOwner(Channel channel) const;

    private:
        Entry& entry(Channel channel);
        const Entry& entry(Channel channel) const;

        std::array<Entry, static_cast<size_t>(Channel::Count)> entries_;
    };

} // namespace arms_ros2_control::command
