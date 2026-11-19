//
// HumanoidModeClient - WBC（全身控制器）内部模式的唯一访问点。
//
// 和 FsmClient 是两层不同的东西，不要合并：
//   * FsmClient          管"整机现在归谁控制"（HOME / HOLD / MOVEJ / OCS2），4 个状态；
//   * HumanoidModeClient 管"OCS2 内部怎么控制"（身体约束 / 双臂 / 耦合 / 底盘锁定 / 头部），
//                        是控制器里一个 9 位整数，在 HOLD 状态下同样可以修改。
//
// 输入：ocs2_wbc_controller/current_state   控制器算好的模式视图
//       ocs2_wbc_controller/wbc_capabilities 这台机器支持哪些模式（静态）
// 输出：mode_command                       局部修改语义的字符串命令
//
// 三条约束（改动前请先确认）：
//   1. 不要用模式位自己推导手臂状态：控制器要等内部等待相位结束才报 ARM_ENABLED，
//      自己推会和它打架。一律读 state()。
//   2. 命令是"局部修改"语义：BODY_TRACKING 只改身体那一项，其余保持。
//      所以意图级接口不需要自己拼完整模式。
//   3. 控制器可能静默拒绝命令（不支持的模式、没有外部 TF 时强制锁底盘）。
//      需要确认的操作用 expect() 登记期望，超时会打 WARN。
//
#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <arms_ros2_control_msgs/msg/wbc_capability.hpp>
#include <arms_ros2_control_msgs/msg/wbc_current_state.hpp>

namespace arms_ros2_control::command
{
    /** 身体约束方式。对应控制器 BodyConstraintMode。 */
    enum class BodyMode : uint8_t
    {
        Free,
        Relative,
        Tracking,
        Locked,
        CustomLocked,
    };

    /** 头部模式。对应控制器 HeadMode。 */
    enum class HeadMode : uint8_t
    {
        Disabled,
        Tracking,
        Gaze,
        Forward,
    };

    /**
     * WBC 内部模式的唯一访问点：一个状态缓存 + 一个命令出口。
     *
     * 生命周期内只有一个 current_state 订阅、一个 wbc_capabilities 订阅和一个
     * mode_command 发布器；VR / 面板 / 手柄共享同一个实例。
     */
    class HumanoidModeClient
    {
    public:
        using StateObserver = std::function<void(const arms_ros2_control_msgs::msg::WbcCurrentState&)>;
        using CapabilityObserver = std::function<void(const arms_ros2_control_msgs::msg::WbcCapability&)>;

        HumanoidModeClient(
            rclcpp::Node::SharedPtr node,
            const std::string& controller_name = "ocs2_wbc_controller",
            const std::string& command_topic = "mode_command");

        ~HumanoidModeClient();

        HumanoidModeClient(const HumanoidModeClient&) = delete;
        HumanoidModeClient& operator=(const HumanoidModeClient&) = delete;

        // ---- 读：直接用控制器报的值，不做二次推导 ----

        /** 最近一次收到的模式状态。未收到过时为默认值（底盘锁定 / 身体自由）。 */
        [[nodiscard]] arms_ros2_control_msgs::msg::WbcCurrentState state() const;

        /** 是否已收到过控制器状态（诊断用）。 */
        [[nodiscard]] bool hasState() const;

        [[nodiscard]] bool baseUnlocked() const;
        [[nodiscard]] bool bodyTracking() const;
        /** 手臂是否真的可用（已过控制器的使能等待相位）。 */
        [[nodiscard]] bool armEnabled(bool left) const;
        [[nodiscard]] uint8_t bodyState() const;
        [[nodiscard]] uint8_t headState() const;

        /** 机器能力声明（静态配置，不是运行状态）。 */
        [[nodiscard]] arms_ros2_control_msgs::msg::WbcCapability capability() const;
        [[nodiscard]] bool hasCapability() const;

        // ---- 写：意图级 ----

        /**
         * 最底层出口：直接发一条命令字符串（局部修改语义）。
         * 只应在没有对应意图方法时使用。
         */
        bool publishCommand(const std::string& command);

        bool setBodyMode(BodyMode mode);
        bool setHeadMode(HeadMode mode);
        bool setBaseLocked(bool locked);
        bool setBimanualCoupled(bool coupled);
        bool setArmEnabled(bool left, bool enabled);
        bool setHomeJointReference(bool enabled);
        /** 全身遥操进入用：身体跟随 + 头部跟踪一起打开。 */
        bool setBodyAndHeadTracking();

        // ---- 期望确认 ----

        /** 一条待确认的请求。predicate 返回 true 表示控制器已生效。 */
        struct Expectation
        {
            std::string what;
            std::function<bool(const arms_ros2_control_msgs::msg::WbcCurrentState&)> satisfied;
        };

        /**
         * 登记一条期望。控制器可能静默拒绝（不支持的模式、无外部 TF 时强制锁底盘），
         * 超时未满足会打 WARN 并丢弃，避免"按了没反应"。
         */
        void expect(Expectation expectation,
                    std::chrono::milliseconds timeout = std::chrono::milliseconds(800));

        // ---- 观测 ----

        uint64_t observeState(StateObserver observer);
        void unobserveState(uint64_t token);

        uint64_t observeCapability(CapabilityObserver observer);
        void unobserveCapability(uint64_t token);

    private:
        struct PendingExpectation
        {
            Expectation expectation;
            std::chrono::steady_clock::time_point deadline;
        };

        void onStateMessage(const arms_ros2_control_msgs::msg::WbcCurrentState::SharedPtr msg);
        void onCapabilityMessage(const arms_ros2_control_msgs::msg::WbcCapability::SharedPtr msg);
        void checkExpirations();

        rclcpp::Node::SharedPtr node_;
        std::string controller_name_;

        rclcpp::Subscription<arms_ros2_control_msgs::msg::WbcCurrentState>::SharedPtr state_sub_;
        rclcpp::Subscription<arms_ros2_control_msgs::msg::WbcCapability>::SharedPtr capability_sub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
        rclcpp::TimerBase::SharedPtr expiry_timer_;

        mutable std::mutex state_mutex_;
        arms_ros2_control_msgs::msg::WbcCurrentState state_{};
        bool has_state_{false};

        mutable std::mutex capability_mutex_;
        arms_ros2_control_msgs::msg::WbcCapability capability_{};
        bool has_capability_{false};

        mutable std::mutex state_observer_mutex_;
        std::map<uint64_t, StateObserver> state_observers_;

        mutable std::mutex capability_observer_mutex_;
        std::map<uint64_t, CapabilityObserver> capability_observers_;

        mutable std::mutex expectation_mutex_;
        std::vector<PendingExpectation> expectations_;

        uint64_t next_token_{1};
    };
} // namespace arms_ros2_control::command
