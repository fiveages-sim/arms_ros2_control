//
// FsmClient - 机器状态机（FSM）的唯一访问点。
//
// 为什么要它：
//   过去 fsm 状态有三个副本（ArmsTargetManager 一份字符串、VRInputHandler 一份
//   整数、各 RViz 面板又各自用 FSMStateTransitionValidator 推一份），而且推断用
//   的规则表与控制器实际行为并不一致。控制器其实一直在发 fsm_state，只是没人订。
//
// 现在：
//   * 读：只订阅 fsm_state（transient_local，后加入也能拿到最后一帧）；
//   * 写：只通过 request(意图) 发 fsm_command；
//   * 任何设备（VR / 手柄 / 面板 / 教具）共享同一个实例，不再各自维护状态。
//
// 两条约束（改动前请先确认）：
//   1. 不在本地推断状态。控制器的 checkChange() 才是唯一真相，
//      它还可能因为碰撞检测主动把机器拉回 HOLD。
//   2. 不做乐观更新。request() 只发命令，状态要等 fsm_state 回来才变，
//      否则控制器拒绝命令时界面会抢跑。
//
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include <arms_controller_common/utils/FSMCommandPublisher.h>

namespace arms_ros2_control::command
{
    /**
     * FSM 状态。数值与控制器 publishCurrentFsmState() 一一对应，
     * 也与历史 fsm_command 的 command 号保持一致（100 除外，那是特殊命令）。
     */
    enum class FsmState : int32_t
    {
        Reset = 0,
        Home = 1,
        Hold = 2,
        Ocs2 = 3,
        MoveJ = 4,
    };

    /** 用户意图。"前进/后退"比"发数字 3"好读，也让设备层与协议解耦。 */
    enum class FsmIntent : uint8_t
    {
        Next, ///< HOME → HOLD → OCS2
        Back, ///< OCS2/MOVEJ → HOLD → HOME（HOME 下再按 = 切换预设姿态）
    };
    /** 状态枚举 ↔ 英文名。日志与界面统一用这一份，避免各处各写一套。 */
    [[nodiscard]] const char* fsmStateName(FsmState state) noexcept;
    /**
     * FSM 的唯一访问点：一个状态缓存 + 一个命令出口。
     *
     * 生命周期内只有一个订阅和一个发布器；多个设备共享同一个实例。
     */
    class FsmClient
    {
    public:
        using Observer = std::function<void(FsmState from, FsmState to)>;

        /**
         * @param node ROS 节点（用于建订阅/发布器）
         * @param state_topic 状态话题，默认相对名 fsm_state（按命名空间隔离多机器人）
         * @param command_topic 命令话题，默认相对名 fsm_command
         */
        FsmClient(
            rclcpp::Node::SharedPtr node,
            const std::string& state_topic = "fsm_state",
            const std::string& command_topic = "fsm_command");

        ~FsmClient();

        FsmClient(const FsmClient&) = delete;
        FsmClient& operator=(const FsmClient&) = delete;

        // ---- 读：任意线程、任意频率 ----

        /**
         * 当前状态。构造即 HOLD，与控制器 on_activate() 的初始状态一致，
         * 因此在收到第一帧之前也不会出现"未知"。
         */
        [[nodiscard]] FsmState state() const noexcept;
        [[nodiscard]] bool is(FsmState s) const noexcept;
        [[nodiscard]] bool isOcs2() const noexcept { return is(FsmState::Ocs2); }
        [[nodiscard]] bool isHold() const noexcept { return is(FsmState::Hold); }

        /** 状态的英文名（HOME / HOLD / OCS2 / MOVEJ / RESET），用于日志。 */
        [[nodiscard]] const char* stateName() const noexcept;

        /** 是否已经真的收到过控制器的状态帧（调试/诊断用）。 */
        [[nodiscard]] bool hasReceivedState() const noexcept;

        // ---- 写：只表达意图 ----

        /**
         * 计算这个意图在当前状态下对应的命令号，不发送也不改状态。
         * @return 命令号；当前状态下不合法时返回 nullopt
         */
        [[nodiscard]] std::optional<int32_t> plan(FsmIntent intent) const;

        /**
         * 请求一次状态切换。不合法时打 WARN 并返回 false。
         * 状态变化要等 fsm_state 回来，不在本地乐观更新。
         */
        bool request(FsmIntent intent);

        /**
         * 兼容入口：RViz 面板 / 外部脚本直接给命令号。
         * 0（无命令）与 100（HOME 下切预设姿态）原样透传。
         */
        bool requestCommand(int32_t command);

        // ---- 观测 ----

        /** 注册状态变化回调，返回句柄。回调在订阅线程执行。 */
        uint64_t observe(Observer observer);
        void unobserve(uint64_t token);

    private:
        void onStateMessage(const std_msgs::msg::Int32::SharedPtr msg);
        void notify(FsmState from, FsmState to);

        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr state_sub_;
        // 命令出口。复用通用工具类，command=100 会自动补一个 0 复位去抖标记。
        std::unique_ptr<arms_controller_common::FSMCommandPublisher> command_pub_;

        std::atomic<FsmState> state_{FsmState::Hold};
        std::atomic<bool> received_state_{false};

        mutable std::mutex observer_mutex_;
        std::map<uint64_t, Observer> observers_;
        uint64_t next_token_{1};
    };
} // namespace arms_ros2_control::command
