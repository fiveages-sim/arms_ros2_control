#pragma once

#include "UnitreeCommunicator.h"
#include <unitree/dds_wrapper/robots/go2/go2.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <mutex>

/**
 * @brief 四足机器人通信器实现（支持Go2等四足机器人）
 */
class QuadrupedCommunicator : public UnitreeCommunicator {
public:
    QuadrupedCommunicator();
    ~QuadrupedCommunicator() override = default;

    // 基类接口实现
    bool initialize(int domain, const std::string& network_interface) override;
    
    // 四足机器人特有的重载方法，支持高状态控制
    bool initialize(int domain, const std::string& network_interface, bool enable_high_state);
    bool readState(RobotState& state) override;
    bool writeCommand(const RobotCommand& command) override;
    void setStateCallback(std::function<void(const RobotState&)> callback) override;
    std::string getRobotType() const override { return "quadruped"; }
    int getJointCount() const override { return joint_count_; }
    void setJointCount(int joint_count) override { joint_count_ = joint_count; }
    bool supportsFootForce() const override { return true; }
    bool supportsHighState() const override { return true; }

private:
    void lowStateMessageHandle(const void* messages);
    void highStateMessageHandle(const void* messages);
    void initLowCmd();
    void resizeStateData();

    // Go2特定的消息类型
    unitree_go::msg::dds_::LowCmd_ low_cmd_;
    unitree_go::msg::dds_::LowState_ low_state_;
    unitree_go::msg::dds_::SportModeState_ high_state_;

    // 通信对象
    std::shared_ptr<unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>> low_cmd_publisher_;
    std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>> low_state_subscriber_;
    std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>> high_state_subscriber_;

    // 线程安全
    std::mutex state_mutex_;
    std::function<void(const RobotState&)> state_callback_;
    
    // 状态缓存
    RobotState cached_state_;
    bool state_updated_ = false;
    
    // 关节数量（从配置中设置）
    int joint_count_ = 12; // 默认值，会被配置覆盖
};
