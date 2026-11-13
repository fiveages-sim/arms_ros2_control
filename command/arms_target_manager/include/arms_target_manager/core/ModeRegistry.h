//
// 会话面：模式声明“我允许哪些动作、我独占哪些目标”。
//
// 作用：
//   把“哪个模式允许哪些动作”从散落的 if 里抽出来，变成一张声明表：
//     ModeId      —— 有哪几种工作模式；
//     OverlayId   —— 与模式正交的叠加层（目前是底盘遥控）；
//     ActionMask / ChannelMask —— 编译期可组合的集合；
//     VrSession   —— 运行时回答“这个动作现在能不能用”。
//
// 用法：
//   动作入口只需 `if (!session_.allows(action)) return;`，
//   新增模式时只在 kModes 里加一条，不必改任何动作处理代码。
//
// 注意：
//   这里只回答“允不允许”，不关心动作具体怎么做。
//
#pragma once

#include <cstdint>
#include <string_view>

#include "arms_target_manager/core/Action.h"
#include "arms_target_manager/core/TeleopTypes.h"

namespace arms_ros2_control::command
{
    /** 动作集合：够用 64 个动作，编译期可组合。 */
    class ActionMask
    {
    public:
        constexpr ActionMask() = default;
        constexpr explicit ActionMask(Action action)
            : bits_(uint64_t{1} << static_cast<uint64_t>(action))
        {
        }

        constexpr ActionMask operator|(ActionMask other) const
        {
            return ActionMask(bits_ | other.bits_);
        }
        constexpr ActionMask operator&(ActionMask other) const
        {
            return ActionMask(bits_ & other.bits_);
        }
        constexpr ActionMask operator~() const
        {
            return ActionMask(~bits_);
        }
        constexpr ActionMask& operator|=(ActionMask other)
        {
            bits_ |= other.bits_;
            return *this;
        }
        [[nodiscard]] constexpr bool has(Action action) const
        {
            return (bits_ & (uint64_t{1} << static_cast<uint64_t>(action))) != 0;
        }

    private:
        constexpr explicit ActionMask(uint64_t bits)
            : bits_(bits)
        {
        }
        uint64_t bits_{0};
    };

    inline constexpr ActionMask operator|(Action a, Action b) { return ActionMask(a) | ActionMask(b); }
    inline constexpr ActionMask operator|(ActionMask m, Action a) { return m | ActionMask(a); }

    /** 目标集合。 */
    class ChannelMask
    {
    public:
        constexpr ChannelMask() = default;
        constexpr explicit ChannelMask(Channel channel)
            : bits_(uint16_t{1} << static_cast<uint16_t>(channel))
        {
        }
        constexpr ChannelMask operator|(ChannelMask other) const
        {
            return ChannelMask(uint16_t(bits_ | other.bits_));
        }
        [[nodiscard]] constexpr bool has(Channel channel) const
        {
            return (bits_ & (uint16_t{1} << static_cast<uint16_t>(channel))) != 0;
        }
        [[nodiscard]] constexpr uint16_t bits() const { return bits_; }

    private:
        constexpr explicit ChannelMask(uint16_t bits)
            : bits_(bits)
        {
        }
        uint16_t bits_{0};
    };

    constexpr ChannelMask operator|(Channel a, Channel b) { return ChannelMask(a) | ChannelMask(b); }
    constexpr ChannelMask operator|(ChannelMask m, Channel c) { return m | ChannelMask(c); }

    enum class ModeId : uint8_t
    {
        Idle = 0,        // VR 未使能
        NormalStorage,   // 普通 VR，存储
        NormalUpdate,    // 普通 VR，更新
        WholeBodyIdle,   // 全身模式，未开跟踪
        WholeBodyTrack,  // 全身模式，全部跟踪
        Count,
    };

    /** 与模式正交的叠加层：底盘遥控、手动接管等。 */
    enum class OverlayId : uint8_t
    {
        Chassis = 0,
        Count,
    };

    struct ModeSpec
    {
        ModeId id;
        std::string_view name;
        ActionMask allows;
        ChannelMask owns;
    };

    struct OverlaySpec
    {
        OverlayId id;
        std::string_view name;
        ActionMask allows;
        ChannelMask takes;
    };

    // ---- 各层允许的动作 ----

    // 两种模式共用的“系统级”动作
    inline constexpr ActionMask kCommonActions =
        Action::FsmNext | Action::FsmBack | Action::FsmToggleRest |
        Action::HandSwitchLeft | Action::HandSwitchRight |
        Action::HandModeToggleLeft | Action::HandModeToggleRight |
        Action::WholeBodyToggle | Action::ChassisToggle;

    // 只有普通模式才有：手臂暂停、更新/存储、镜像、身体模式、WBC 组合键
    inline constexpr ActionMask kNormalActions =
        kCommonActions |
        Action::UpdateToggle | Action::ArmToggleLeft | Action::ArmToggleRight |
        Action::MirrorToggle | Action::MirrorProfileToggle | Action::YbSemanticsToggle |
        Action::BodyVertical | Action::BodyLocked | Action::BodyTracking | Action::BodyCustom |
        Action::BimanualToggle | Action::BaseToggle | Action::HomeJointToggle |
        Action::ScaleCalibrate;

    // 全身模式：只保留右摇杆短按（全部跟踪）与底盘相关
    inline constexpr ActionMask kWholeBodyActions =
        kCommonActions | Action::AbsoluteTrackingToggle | Action::BaseToggle;

    inline constexpr ModeSpec kModes[] = {
        {ModeId::Idle, "未使能", ActionMask{}, ChannelMask{}},
        {ModeId::NormalStorage, "普通/存储", kNormalActions,
         Channel::ArmLeft | Channel::ArmRight | Channel::Head},
        {ModeId::NormalUpdate, "普通/更新", kNormalActions,
         Channel::ArmLeft | Channel::ArmRight | Channel::Head},
        {ModeId::WholeBodyIdle, "全身/未开跟踪", kWholeBodyActions, ChannelMask{}},
        {ModeId::WholeBodyTrack, "全身/全部跟踪", kWholeBodyActions,
         Channel::ArmLeft | Channel::ArmRight | Channel::Head | Channel::Body},
    };

    inline constexpr OverlaySpec kOverlays[] = {
        {OverlayId::Chassis, "底盘遥控",
         Action::ChassisToggle | Action::BaseToggle,
         ChannelMask{}},
    };

    inline const ModeSpec& modeSpec(ModeId id)
    {
        for (const auto& spec : kModes)
        {
            if (spec.id == id)
            {
                return spec;
            }
        }
        return kModes[0];
    }

    inline const char* modeName(ModeId id)
    {
        return modeSpec(id).name.data();
    }

    /**
     * 当前会话：由它回答“这个动作现在能不能用”。
     *
     * 注意“编号 → 动作”已经由 BindingProfile 决定，这里只做允许性判断，
     * 因此新增模式只需在 kModes 里加一条，不必改任何动作处理代码。
     */
    class VrSession
    {
    public:
        void setEnabled(bool enabled)
        {
            if (!enabled)
            {
                mode_ = ModeId::Idle;
                whole_body_active_ = false;
                absolute_tracking_ = false;
            }
            enabled_ = enabled;
            refresh();
        }

        void setWholeBodyActive(bool active)
        {
            whole_body_active_ = active;
            if (!active)
            {
                absolute_tracking_ = false;
            }
            refresh();
        }

        void setAbsoluteTracking(bool active)
        {
            absolute_tracking_ = active && whole_body_active_;
            refresh();
        }

        void setRelativeUpdate(bool active)
        {
            relative_update_ = active;
            refresh();
        }

        void setOverlay(OverlayId id, bool active)
        {
            if (id == OverlayId::Chassis)
            {
                chassis_ = active;
            }
        }

        [[nodiscard]] bool enabled() const { return enabled_; }
        [[nodiscard]] bool wholeBodyActive() const { return whole_body_active_; }
        [[nodiscard]] bool absoluteTracking() const { return absolute_tracking_; }
        [[nodiscard]] bool relativeUpdate() const { return relative_update_; }
        [[nodiscard]] bool overlay(OverlayId id) const
        {
            return id == OverlayId::Chassis && chassis_;
        }

        [[nodiscard]] ModeId mode() const { return mode_; }

        [[nodiscard]] bool allows(Action action) const
        {
            if (action == Action::None)
            {
                return false;
            }
            return modeSpec(mode_).allows.has(action);
        }

        [[nodiscard]] ChannelMask ownedChannels() const
        {
            return modeSpec(mode_).owns;
        }

        [[nodiscard]] bool owns(Channel channel) const
        {
            return ownedChannels().has(channel);
        }

    private:
        void refresh()
        {
            if (!enabled_)
            {
                mode_ = ModeId::Idle;
                return;
            }
            if (whole_body_active_)
            {
                mode_ = absolute_tracking_ ? ModeId::WholeBodyTrack : ModeId::WholeBodyIdle;
                return;
            }
            mode_ = relative_update_ ? ModeId::NormalUpdate : ModeId::NormalStorage;
        }

        bool enabled_{false};
        bool whole_body_active_{false};
        bool absolute_tracking_{false};
        bool relative_update_{false};
        bool chassis_{false};
        ModeId mode_{ModeId::Idle};
    };

} // namespace arms_ros2_control::command
