//
// 语义面：把设备上报的原始按键编号翻译成“用户想做什么”。
//
// 作用：
//   定义全部动作（Action）、修饰键（Modifier）与默认的“编号 → 动作”绑定表。
//   设备（手柄 / 手套 / 键盘）只负责上报编号，编号代表什么由这张表统一回答。
//
// 用法：
//   decodeAction(code) 得到一个 Action，再交给会话层（ModeRegistry）判断
//   当前模式是否允许它，最后才进入具体处理函数。
//
// 注意：
//   这里不描述“哪个模式允许哪个动作”（那是 ModeRegistry 的事），
//   也不做任何动作处理。新增组合键只改本文件的 kDefaultBindings。
//
#pragma once

#include <cstdint>
#include <optional>
#include <string_view>

namespace arms_ros2_control::command
{
    /** 可被触发的动作。新增功能时在这里加一项。 */
    enum class Action : uint16_t
    {
        None = 0,
        // 状态机
        FsmNext,
        FsmBack,
        FsmToggleRest,
        // 手臂更新
        UpdateToggle,
        ArmToggleLeft,
        ArmToggleRight,
        MirrorToggle,
        MirrorProfileToggle,
        YbSemanticsToggle,
        // 全身
        WholeBodyToggle,
        AbsoluteTrackingToggle,
        // 握持与夹爪
        HandSwitchLeft,
        HandSwitchRight,
        HandModeToggleLeft,
        HandModeToggleRight,
        // 身体模式
        BodyVertical,
        BodyLocked,
        BodyTracking,
        BodyCustom,
        // WBC 开关
        BimanualToggle,
        BaseToggle,
        HomeJointToggle,
        // 底盘
        ChassisToggle,
        // 校准
        ScaleCalibrate,
        Count,
    };

    /** 同时按住的修饰键位。用于组合键判定。 */
    enum class Modifier : uint16_t
    {
        None = 0,
        LeftGrip = 1u << 0,
        RightGrip = 1u << 1,
        LeftTrigger = 1u << 2,
        RightTrigger = 1u << 3,
    };

    using ModifierMask = uint16_t;

    constexpr ModifierMask operator|(Modifier a, Modifier b)
    {
        return static_cast<ModifierMask>(a) | static_cast<ModifierMask>(b);
    }

    /** 一条绑定：某个编号（可能需要某些修饰键、且不能有另外一些）→ 一个动作。 */
    struct BindingRule
    {
        int32_t code;
        ModifierMask require;
        ModifierMask forbid;
        Action action;
    };

    /**
     * 默认绑定表：与头显侧（xr_target_node）现有的编号一一对应。
     *
     * 注意这里只描述“编号→动作”，不描述“哪个模式允许这个动作”，
     * 后者在 ModeRegistry 里声明。两件事分开，加模式时不用改这张表。
     */
    inline constexpr BindingRule kDefaultBindings[] = {
        {3, 0, 0, Action::ArmToggleLeft},
        {6, 0, 0, Action::ArmToggleRight},
        {4, 0, 0, Action::UpdateToggle},          // 普通模式；全身模式另解（见 ModeSpec）
        {7, 0, 0, Action::MirrorToggle},
        {9, 0, 0, Action::HandSwitchLeft},
        {10, 0, 0, Action::HandSwitchRight},
        {11, 0, 0, Action::FsmNext},
        {12, 0, 0, Action::FsmBack},
        {13, 0, 0, Action::HandModeToggleLeft},
        {14, 0, 0, Action::HandModeToggleRight},
        {15, 0, 0, Action::ScaleCalibrate},
        {16, 0, 0, Action::YbSemanticsToggle},
        {20, 0, 0, Action::ChassisToggle},
        {21, 0, 0, Action::BodyVertical},
        {22, 0, 0, Action::BodyLocked},
        {23, 0, 0, Action::BodyTracking},
        {24, 0, 0, Action::BodyCustom},
        {25, 0, 0, Action::BimanualToggle},
        {26, 0, 0, Action::BaseToggle},
        {28, 0, 0, Action::HomeJointToggle},
        {29, 0, 0, Action::MirrorProfileToggle},
        {30, 0, 0, Action::WholeBodyToggle},
    };

    /** 编号 → 动作。找不到返回 None。 */
    inline Action decodeAction(int32_t code, ModifierMask held = 0)
    {
        for (const auto& rule : kDefaultBindings)
        {
            if (rule.code != code)
            {
                continue;
            }
            if ((held & rule.require) != rule.require)
            {
                continue;
            }
            if ((held & rule.forbid) != 0)
            {
                continue;
            }
            return rule.action;
        }
        return Action::None;
    }

    /** 动作的中文名（日志与界面用），与枚举一一对应。 */
    inline const char* actionName(Action action)
    {
        switch (action)
        {
            case Action::None:
                return "none";
            case Action::FsmNext:
                return "FSM 前进";
            case Action::FsmBack:
                return "FSM 后退";
            case Action::FsmToggleRest:
                return "FSM 姿态切换";
            case Action::UpdateToggle:
                return "更新/存储切换";
            case Action::ArmToggleLeft:
                return "左臂暂停/恢复";
            case Action::ArmToggleRight:
                return "右臂暂停/恢复";
            case Action::MirrorToggle:
                return "镜像开关";
            case Action::MirrorProfileToggle:
                return "镜像方案切换";
            case Action::YbSemanticsToggle:
                return "Y/B 语义切换";
            case Action::WholeBodyToggle:
                return "全身模式开关";
            case Action::AbsoluteTrackingToggle:
                return "全部跟踪开关";
            case Action::HandSwitchLeft:
                return "左手开关";
            case Action::HandSwitchRight:
                return "右手开关";
            case Action::HandModeToggleLeft:
                return "左手模式切换";
            case Action::HandModeToggleRight:
                return "右手模式切换";
            case Action::BodyVertical:
                return "身体竖直";
            case Action::BodyLocked:
                return "身体锁定";
            case Action::BodyTracking:
                return "身体跟随";
            case Action::BodyCustom:
                return "身体自定义";
            case Action::BimanualToggle:
                return "双臂耦合开关";
            case Action::BaseToggle:
                return "底盘解锁/锁定";
            case Action::HomeJointToggle:
                return "参考关节开关";
            case Action::ChassisToggle:
                return "底盘遥控开关";
            case Action::ScaleCalibrate:
                return "尺度校准";
            case Action::Count:
                break;
        }
        return "unknown";
    }

} // namespace arms_ros2_control::command
