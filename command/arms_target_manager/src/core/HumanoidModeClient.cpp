//
// HumanoidModeClient 实现。
//
// 关键点：
//   * 命令是“局部修改”语义，因此每个意图方法只发一个关键字；
//   * 手臂可用性直接用报文里的 *_arm_state，不用模式位自己推；
//   * expect() 登记期望，超时未满足会 WARN，用来暴露“命令被静默拒绝”。
//
// 注意：
//   任何“控制器会怎么反应”的推导都不在这里，也不该在调用方。
//
#include "arms_target_manager/core/HumanoidModeClient.h"

#include <utility>

namespace arms_ros2_control::command
{
    namespace
    {
        const char *bodyModeCommand(uint8_t mode)
        {
            using State = arms_ros2_control_msgs::msg::WbcCurrentState;
            switch (mode)
            {
            case State::BODY_FREE:
                return "BODY_FREE";
            case State::BODY_VERTICAL:
                return "BODY_RELATIVE";
            case State::BODY_TRACKING:
                return "BODY_TRACKING";
            case State::BODY_LOCKED:
                return "BODY_LOCK";
            case State::BODY_CUSTOM_LOCKED:
                return "BODY_CUSTOM_LOCK";
            }
            return nullptr;
        }

        const char *headModeCommand(uint8_t mode)
        {
            using State = arms_ros2_control_msgs::msg::WbcCurrentState;
            switch (mode)
            {
            case State::HEAD_DISABLED:
                return "HEAD_DISABLE";
            case State::HEAD_TRACKING:
                return "HEAD_TRACKING";
            case State::HEAD_GAZE:
                return "HEAD_GAZE";
            case State::HEAD_FORWARD:
                return "HEAD_FORWARD";
            }
            return nullptr;
        }

        const char *baseModeCommand(uint8_t mode)
        {
            using State = arms_ros2_control_msgs::msg::WbcCurrentState;
            switch (mode)
            {
            case State::BASE_LOCKED:
                return "BASE_LOCK";
            case State::BASE_UNLOCKED:
                return "BASE_UNLOCK";
            case State::BASE_TRACKING:
                return "BASE_TRACK";
            }
            return nullptr;
        }
    } // namespace

    /**
     * @brief 构造：订阅模式状态与能力声明，建命令发布器与超时检查定时器
     */
    HumanoidModeClient::HumanoidModeClient(
        rclcpp::Node::SharedPtr node,
        const std::string &controller_name,
        const std::string &command_topic)
        : node_(std::move(node)), controller_name_(controller_name)
    {
        if (!node_)
        {
            return;
        }

        const std::string state_topic = controller_name_ + "/current_state";
        const std::string capability_topic = controller_name_ + "/wbc_capabilities";

        // transient_local：控制器只在模式变化时发一帧，后加入的节点必须能立刻
        // 拿到最后一帧，否则会一直用默认值。
        state_sub_ = node_->create_subscription<arms_ros2_control_msgs::msg::WbcCurrentState>(
            state_topic,
            rclcpp::QoS(1).reliable().transient_local(),
            [this](const arms_ros2_control_msgs::msg::WbcCurrentState::SharedPtr msg)
            {
                onStateMessage(msg);
            });

        capability_sub_ = node_->create_subscription<arms_ros2_control_msgs::msg::WbcCapability>(
            capability_topic,
            rclcpp::QoS(1).reliable().transient_local(),
            [this](const arms_ros2_control_msgs::msg::WbcCapability::SharedPtr msg)
            {
                onCapabilityMessage(msg);
            });

        command_pub_ = node_->create_publisher<std_msgs::msg::String>(command_topic, 10);

        // 低频检查未满足的期望，避免"按了没反应"却没有任何提示。
        expiry_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(200),
            [this]()
            {
                checkExpirations();
            });

        RCLCPP_INFO(node_->get_logger(),
                    "🕹️ HumanoidModeClient: state=%s capability=%s command=%s",
                    state_topic.c_str(), capability_topic.c_str(), command_topic.c_str());
    }

    HumanoidModeClient::~HumanoidModeClient()
    {
        if (expiry_timer_)
        {
            expiry_timer_->cancel();
        }
        {
            std::lock_guard<std::mutex> lock(state_observer_mutex_);
            state_observers_.clear();
        }
        {
            std::lock_guard<std::mutex> lock(capability_observer_mutex_);
            capability_observers_.clear();
        }
    }

    arms_ros2_control_msgs::msg::WbcCurrentState HumanoidModeClient::state() const
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return state_;
    }

    bool HumanoidModeClient::hasState() const
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return has_state_;
    }

    bool HumanoidModeClient::baseUnlocked() const
    {
        // 随动与跟随都属于“底盘可动”；只有锁定不算。
        std::lock_guard<std::mutex> lock(state_mutex_);
        return state_.base_state != arms_ros2_control_msgs::msg::WbcCurrentState::BASE_LOCKED;
    }

    bool HumanoidModeClient::baseTracking() const
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return state_.base_state == arms_ros2_control_msgs::msg::WbcCurrentState::BASE_TRACKING;
    }

    bool HumanoidModeClient::bodyTracking() const
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return state_.body_state == arms_ros2_control_msgs::msg::WbcCurrentState::BODY_TRACKING;
    }

    bool HumanoidModeClient::armEnabled(bool left) const
    {
        // 必须用报文里的值：控制器要等内部使能等待相位结束才报 ARM_ENABLED，
        // 用模式位自己推会得到"已使能"的假象。
        std::lock_guard<std::mutex> lock(state_mutex_);
        const uint8_t value = left ? state_.left_arm_state : state_.right_arm_state;
        return value == arms_ros2_control_msgs::msg::WbcCurrentState::ARM_ENABLED;
    }

    uint8_t HumanoidModeClient::bodyState() const
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return state_.body_state;
    }

    uint8_t HumanoidModeClient::headState() const
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return state_.head_state;
    }

    arms_ros2_control_msgs::msg::WbcCapability HumanoidModeClient::capability() const
    {
        std::lock_guard<std::mutex> lock(capability_mutex_);
        return capability_;
    }

    bool HumanoidModeClient::hasCapability() const
    {
        std::lock_guard<std::mutex> lock(capability_mutex_);
        return has_capability_;
    }

    /**
     * @brief 最底层出口：直接发一条命令字符串
     *
     * 服务端是“局部修改”语义，所以拼字符串前必须清楚哪些字段会被改。
     */
    bool HumanoidModeClient::publishCommand(const std::string &command)
    {
        if (command.empty())
        {
            return false;
        }
        if (!command_pub_)
        {
            return false;
        }
        std_msgs::msg::String msg;
        msg.data = command;
        command_pub_->publish(msg);
        RCLCPP_INFO(node_->get_logger(), "🕹️ WBC 模式命令：%s", command.c_str());
        return true;
    }

    bool HumanoidModeClient::setBodyMode(uint8_t mode)
    {
        const char *command = bodyModeCommand(mode);
        return command != nullptr && publishCommand(command);
    }

    bool HumanoidModeClient::setHeadMode(uint8_t mode)
    {
        const char *command = headModeCommand(mode);
        return command != nullptr && publishCommand(command);
    }

    bool HumanoidModeClient::setBaseMode(uint8_t mode)
    {
        const char *command = baseModeCommand(mode);
        return command != nullptr && publishCommand(command);
    }

    bool HumanoidModeClient::setBimanualCoupled(bool coupled)
    {
        return publishCommand(coupled ? "ARMS_COUPLED" : "ARMS_INDEPENDENT");
    }

    bool HumanoidModeClient::setArmEnabled(bool left, bool enabled)
    {
        if (left)
        {
            return publishCommand(enabled ? "LEFT_ARM_ENABLE" : "LEFT_ARM_DISABLE");
        }
        return publishCommand(enabled ? "RIGHT_ARM_ENABLE" : "RIGHT_ARM_DISABLE");
    }

    bool HumanoidModeClient::setHomeJointReference(bool enabled)
    {
        return publishCommand(enabled ? "HOME_JOINT_ON" : "HOME_JOINT_OFF");
    }

    bool HumanoidModeClient::setBodyAndHeadTracking()
    {
        // 两条独立的局部修改命令，顺序无关。
        const bool body_ok = publishCommand("BODY_TRACKING");
        const bool head_ok = publishCommand("HEAD_TRACKING");
        return body_ok && head_ok;
    }

    /**
     * @brief 登记一条待确认的请求
     *
     * 控制器可能静默拒绝（不支持的模式、无外部 TF 时强制锁底盘），
     * 超时未满足会打 WARN 并丢弃，避免“按了没反应”没有任何线索。
     */
    void HumanoidModeClient::expect(Expectation expectation,
                                    std::chrono::milliseconds timeout)
    {
        if (!expectation.satisfied)
        {
            return;
        }
        std::lock_guard<std::mutex> lock(expectation_mutex_);
        expectations_.push_back(
            PendingExpectation{std::move(expectation), std::chrono::steady_clock::now() + timeout});
    }

    uint64_t HumanoidModeClient::observeState(StateObserver observer)
    {
        if (!observer)
        {
            return 0;
        }
        std::lock_guard<std::mutex> lock(state_observer_mutex_);
        const uint64_t token = next_token_++;
        state_observers_[token] = std::move(observer);
        return token;
    }

    void HumanoidModeClient::unobserveState(uint64_t token)
    {
        std::lock_guard<std::mutex> lock(state_observer_mutex_);
        state_observers_.erase(token);
    }

    uint64_t HumanoidModeClient::observeCapability(CapabilityObserver observer)
    {
        if (!observer)
        {
            return 0;
        }
        std::lock_guard<std::mutex> lock(capability_observer_mutex_);
        const uint64_t token = next_token_++;
        capability_observers_[token] = std::move(observer);
        return token;
    }

    void HumanoidModeClient::unobserveCapability(uint64_t token)
    {
        std::lock_guard<std::mutex> lock(capability_observer_mutex_);
        capability_observers_.erase(token);
    }

    /**
     * @brief 收到模式状态
     *
     * 先更新缓存与期望匹配，再通知观察者；观察者在订阅线程里执行。
     */
    void HumanoidModeClient::onStateMessage(
        const arms_ros2_control_msgs::msg::WbcCurrentState::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state_ = *msg;
            has_state_ = true;
        }

        // 先处理期望：命中就移除，避免"已生效还报超时"。
        {
            std::lock_guard<std::mutex> lock(expectation_mutex_);
            for (auto it = expectations_.begin(); it != expectations_.end();)
            {
                if (it->expectation.satisfied && it->expectation.satisfied(*msg))
                {
                    RCLCPP_INFO(node_->get_logger(),
                                "🕹️ WBC 模式确认：%s", it->expectation.what.c_str());
                    it = expectations_.erase(it);
                }
                else
                {
                    ++it;
                }
            }
        }

        std::map<uint64_t, StateObserver> snapshot;
        {
            std::lock_guard<std::mutex> lock(state_observer_mutex_);
            snapshot = state_observers_;
        }
        for (auto &[token, observer] : snapshot)
        {
            (void)token;
            if (observer)
            {
                observer(*msg);
            }
        }
    }

    void HumanoidModeClient::onCapabilityMessage(
        const arms_ros2_control_msgs::msg::WbcCapability::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(capability_mutex_);
            capability_ = *msg;
            has_capability_ = true;
        }

        std::map<uint64_t, CapabilityObserver> snapshot;
        {
            std::lock_guard<std::mutex> lock(capability_observer_mutex_);
            snapshot = capability_observers_;
        }
        for (auto &[token, observer] : snapshot)
        {
            (void)token;
            if (observer)
            {
                observer(*msg);
            }
        }
    }

    /**
     * @brief 清理超时未满足的期望并逐条告警
     */
    void HumanoidModeClient::checkExpirations()
    {
        const auto now = std::chrono::steady_clock::now();
        std::vector<PendingExpectation> expired;
        {
            std::lock_guard<std::mutex> lock(expectation_mutex_);
            for (auto it = expectations_.begin(); it != expectations_.end();)
            {
                if (it->deadline <= now)
                {
                    expired.push_back(*it);
                    it = expectations_.erase(it);
                }
                else
                {
                    ++it;
                }
            }
        }

        for (const auto &pending : expired)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "🕹️ WBC 模式未生效：%s（控制器可能拒绝该命令，"
                        "例如未支持的模式或缺少外部世界坐标）",
                        pending.expectation.what.c_str());
        }
    }
} // namespace arms_ros2_control::command
