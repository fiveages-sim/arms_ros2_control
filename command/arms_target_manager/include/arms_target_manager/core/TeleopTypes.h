//
// 遥操作目标链路的公共类型。
//
// 这里只放“输入源要表达什么”和“目标通道是谁”的词汇，
// 不包含任何 ROS 调用，也不包含业务逻辑。
//
#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

namespace arms_ros2_control::command
{
    /** 左右两侧。用于替代代码里到处出现的 "left"/"right" 字符串。 */
    enum class Side : uint8_t
    {
        Left = 0,
        Right = 1,
    };

    inline Side otherSide(Side side)
    {
        return side == Side::Left ? Side::Right : Side::Left;
    }

    /** 可以被下发目标的位置。每个位置同一时刻只允许一个输入源在写。 */
    enum class Channel : uint8_t
    {
        ArmLeft = 0,
        ArmRight = 1,
        Head = 2,
        Body = 3,
        Count = 4,
    };

    /** 输入源。新增控制方式时在这里加一项。 */
    enum class SourceId : uint8_t
    {
        None = 0,
        Vr = 1,            // VR 手柄 / 全身骨架
        Rviz = 2,          // RViz 里拖动 marker 或菜单发送
        ControlInput = 3,  // 摇杆 / 键盘的 control_input 速度流
        External = 4,      // 其它程序化调用（兼容旧接口）
    };

    /**
     * 优先级只用于“绝对目标”之间的抢占，不用于速度叠加。
     * 高优先级可以打断低优先级；同级之间不允许互相打断。
     */
    enum class Priority : uint8_t
    {
        Low = 10,
        Normal = 20,
        High = 30,
    };

    /**
     * 一次目标提交。
     *
     * position/orientation 是绝对位姿；frame 为表达坐标系，留空表示控制坐标系。
     * lease 是这次控制权的有效期：期间其它低优先级来源会被拒绝；
     * 调用方每次下发都会续期，因此“停止下发”等于“自动交还”。
     */
    struct Intent
    {
        Channel channel{Channel::ArmLeft};
        SourceId source{SourceId::None};
        Priority priority{Priority::Normal};

        Eigen::Vector3d position{Eigen::Vector3d::Zero()};
        Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
        std::string frame;

        std::chrono::milliseconds lease{0};
        bool one_shot{false};
        /** false 表示只更新显示与内部记录，不真的发话题（兼容旧的“仅可视化”调用）。 */
        bool publish{true};
        /**
         * 开启冻结追赶平滑。
         *
         * VR 手柄数据在链路未更新时会重复发同一帧，恢复后一帧补齐整段位移；
         * 打开后由输出层把这个缺口摊到随后几帧，正常运动不受影响。
         * 手动拖动与程序调用不需要，保持关闭。
         */
        bool filter_stale{false};
    };

    /** 提交结果。reason 直接写进日志，便于排查“为什么不动”。 */
    struct Verdict
    {
        bool accepted{false};
        const char* reason{"ok"};

        explicit operator bool() const { return accepted; }
    };

    /** 每个目标通道的策略。默认值与旧实现保持一致。 */
    struct ChannelPolicy
    {
        /** 一次提交默认获得多长的控制权。 */
        std::chrono::milliseconds lease{500};
        /** 手动拖动以后，多久之内不接受控制器回传的目标刷新显示。 */
        std::chrono::milliseconds echo_hold_after_manual{1000};
        /** 从手动拖动交还给自动控制时的过渡时间。 */
        std::chrono::milliseconds handover_ramp{400};
    };

    /** 当前谁在控制某个通道。 */
    struct Ownership
    {
        SourceId owner{SourceId::None};
        Priority priority{Priority::Low};
        std::chrono::steady_clock::time_point until{};
        bool active{false};
    };

    /** 单个目标的运行状态，用于“为什么不动”这类排查。 */
    struct ChannelStatus
    {
        Channel channel{Channel::ArmLeft};
        SourceId owner{SourceId::None};
        Priority priority{Priority::Low};
        bool active{false};
        bool blocked_for_control_input{false};
        bool has_target{false};
        std::uint64_t accepted{0};
        std::uint64_t rejected{0};
        const char* last_reason{"ok"};
    };

    struct TeleopStatus
    {
        ChannelStatus channels[static_cast<size_t>(Channel::Count)];
    };

} // namespace arms_ros2_control::command
