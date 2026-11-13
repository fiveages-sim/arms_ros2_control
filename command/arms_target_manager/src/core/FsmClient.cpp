//
// FsmClient 实现。
//
// 关键点：
//   * 只订阅真状态（transient_local），晚启动也能立刻拿到最后一帧；
//   * request() 只发命令、不做乐观更新，状态一律等 fsm_state 回来；
//   * 观察者先拷贝再回调，避免持锁回调造成死锁。
//
// 注意：
//   任何“控制器会怎么反应”的推断都不在这里，也不该在调用方。
//
#include "arms_target_manager/core/FsmClient.h"

#include <utility>

namespace arms_ros2_control::command
{
    namespace
    {
        // 与控制器 publishCurrentFsmState() 的状态码一一对应。
        constexpr const char* kStateNames[] = {"RESET", "HOME", "HOLD", "OCS2", "MOVEJ"};
        constexpr int32_t kMaxStateCode = 4;
    } // namespace

    const char* fsmStateName(FsmState state) noexcept
    {
        const auto code = static_cast<int32_t>(state);
        return (code >= 0 && code <= kMaxStateCode) ? kStateNames[code] : "UNKNOWN";
    }

    /**
     * @brief 构造 FSM 客户端：一个状态订阅 + 一个命令发布器
     * @param node ROS 节点
     * @param state_topic 状态话题（相对名，由命名空间隔离多机器人）
     * @param command_topic 命令话题
     */
    FsmClient::FsmClient(
        rclcpp::Node::SharedPtr node,
        const std::string& state_topic,
        const std::string& command_topic)
        : node_(std::move(node))
    {
        if (!node_)
        {
            return;
        }

        // transient_local：控制器只在状态变化时发一帧（on_activate 会发初始的
        // HOLD），后加入的订阅者必须能立刻拿到最后一帧，否则会一直用默认值。
        state_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
            state_topic,
            rclcpp::QoS(1).reliable().transient_local(),
            [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                onStateMessage(msg);
            });

        auto publisher = node_->create_publisher<std_msgs::msg::Int32>(command_topic, 10);
        command_pub_ = std::make_unique<arms_controller_common::FSMCommandPublisher>(
            node_, publisher);

        RCLCPP_INFO(node_->get_logger(),
                    "🕹️ FsmClient: state=%s command=%s (默认 %s)",
                    state_topic.c_str(), command_topic.c_str(), fsmStateName(state_.load()));
    }

    FsmClient::~FsmClient()
    {
        // 订阅/发布器由 shared_ptr 管理；析构时不再回调观察者。
        observer_mutex_.lock();
        observers_.clear();
        observer_mutex_.unlock();
    }

    FsmState FsmClient::state() const noexcept
    {
        return state_.load();
    }

    bool FsmClient::is(FsmState s) const noexcept
    {
        return state_.load() == s;
    }

    const char* FsmClient::stateName() const noexcept
    {
        return fsmStateName(state_.load());
    }

    bool FsmClient::hasReceivedState() const noexcept
    {
        return received_state_.load();
    }

    /**
     * @brief 计算某个意图对应的命令号（纯函数，不改状态也不发送）
     *
     * 规则表只此一份；当前状态下不合法时返回 nullopt。
     */
    std::optional<int32_t> FsmClient::plan(FsmIntent intent) const
    {
        const FsmState current = state_.load();
        switch (intent)
        {
            case FsmIntent::Next:
                // HOME → HOLD → OCS2。已在 OCS2 / MOVEJ 时没有"更前"的状态，
                // MOVEJ 必须先回 HOLD 才能进 OCS2。
                if (current == FsmState::Home)
                {
                    return 2;
                }
                if (current == FsmState::Hold)
                {
                    return 3;
                }
                return std::nullopt;

            case FsmIntent::Back:
                if (current == FsmState::Ocs2 || current == FsmState::MoveJ)
                {
                    return 2;
                }
                if (current == FsmState::Hold)
                {
                    return 1;
                }
                // HOME 下"后退"改为切换预设姿态（状态不变）。
                return 100;

            default:
                return std::nullopt;
        }
    }

    /**
     * @brief 请求一次状态切换
     *
     * 不合法时打 WARN 并返回 false；合法时只发命令，不乐观更新本地状态，
     * 状态要等 fsm_state 回来才变。
     */
    bool FsmClient::request(FsmIntent intent)
    {
        const auto command = plan(intent);
        if (!command)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "🕹️ FSM 请求被忽略：当前状态 %s 无法%s",
                        stateName(),
                        intent == FsmIntent::Next ? "前进" : "后退");
            return false;
        }

        if (!command_pub_)
        {
            return false;
        }

        const FsmState before = state_.load();
        command_pub_->publishCommand(*command);
        RCLCPP_INFO(node_->get_logger(),
                    "🕹️ FSM 请求：%s %s → command=%d（等待控制器确认）",
                    stateName(),
                    intent == FsmIntent::Next ? "前进" : "后退",
                    *command);
        // 不做乐观更新：状态只认 fsm_state。
        (void)before;
        return true;
    }

    /**
     * @brief 兼容入口：直接发命令号（0 与 100 原样透传）
     */
    bool FsmClient::requestCommand(int32_t command)
    {
        if (!command_pub_)
        {
            return false;
        }
        // 0（无命令 / 去抖复位）与 100（HOME 下切预设姿态）原样透传；
        // 1..4 由控制器 checkChange() 决定是否接受。
        command_pub_->publishCommand(command);
        RCLCPP_INFO(node_->get_logger(),
                    "🕹️ FSM 命令：command=%d（当前 %s）",
                    command, stateName());
        return true;
    }

    uint64_t FsmClient::observe(Observer observer)
    {
        if (!observer)
        {
            return 0;
        }
        std::lock_guard<std::mutex> lock(observer_mutex_);
        const uint64_t token = next_token_++;
        observers_[token] = std::move(observer);
        return token;
    }

    void FsmClient::unobserve(uint64_t token)
    {
        std::lock_guard<std::mutex> lock(observer_mutex_);
        observers_.erase(token);
    }

    /**
     * @brief 收到控制器报的真实状态
     *
     * 非法值直接丢弃；与当前值相同时不触发回调，避免重复通知。
     */
    void FsmClient::onStateMessage(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        const int32_t code = msg->data;
        if (code < 0 || code > kMaxStateCode)
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 5000,
                "🕹️ 忽略非法的 fsm_state 值 %d", code);
            return;
        }

        received_state_.store(true);

        const FsmState next = static_cast<FsmState>(code);
        const FsmState previous = state_.exchange(next);
        if (previous == next)
        {
            return;
        }

        RCLCPP_INFO(node_->get_logger(),
                    "🕹️ FSM：%s → %s", fsmStateName(previous), fsmStateName(next));
        notify(previous, next);
    }

    /**
     * @brief 通知所有观察者
     *
     * 先拷贝再回调：观察者里可能 unobserve 或关节点，持锁回调会死锁。
     */
    void FsmClient::notify(FsmState from, FsmState to)
    {
        // 先拷贝再回调：观察者里可能有 unobserve / 关闭节点等操作，
        // 持锁回调会死锁。
        std::map<uint64_t, Observer> snapshot;
        {
            std::lock_guard<std::mutex> lock(observer_mutex_);
            snapshot = observers_;
        }
        for (auto& [token, observer] : snapshot)
        {
            (void)token;
            if (observer)
            {
                observer(from, to);
            }
        }
    }
} // namespace arms_ros2_control::command
