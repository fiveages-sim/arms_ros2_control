#pragma once

#include "UnitreeCommunicator.h"
#include <unitree/dds_wrapper/robots/g1/g1.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <mutex>

/**
 * @brief Humanoid robot communicator implementation (supports G1 and other humanoid robots)
 */
class HumanoidCommunicator : public UnitreeCommunicator {
public:
    HumanoidCommunicator();
    ~HumanoidCommunicator() override = default;

    bool initialize(int domain, const std::string& network_interface) override;
    bool readState(RobotState& state) override;
    bool writeCommand(const RobotCommand& command) override;
    void setStateCallback(std::function<void(const RobotState&)> callback) override;
    std::string getRobotType() const override { return "humanoid"; }
    int getJointCount() const override { return joint_count_; }
    void setJointCount(int joint_count) override;
    bool supportsFootForce() const override { return false; }
    bool supportsHighState() const override { return false; }
    
    // Set control mode
    void setControlMode(uint8_t mode_pr, uint8_t mode_machine = 0);
    
    // Set robot type for different behavior
    void setRobotType(const std::string& robot_type);
    
    // Lifecycle management
    void activate() override;
    void deactivate() override;

private:
    void lowStateMessageHandle(const void* messages);
    void initLowCmd();
    void resizeStateData();

    // G1 specific message types
    unitree_hg::msg::dds_::LowCmd_ low_cmd_;
    unitree_hg::msg::dds_::LowState_ low_state_;

    // Communication objects
    std::shared_ptr<unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>> low_cmd_publisher_;
    std::shared_ptr<unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>> low_state_subscriber_;

    // Thread safety
    std::mutex state_mutex_;
    std::function<void(const RobotState&)> state_callback_;
    
    // State cache
    RobotState cached_state_;
    bool state_updated_ = false;
    
    // Joint count (set from configuration)
    int joint_count_ = 20;
    
    // Control mode settings
    uint8_t mode_pr_ = 0;        // PR = 0, AB = 1
    uint8_t mode_machine_ = 0;   // Machine mode from robot
    
    // Robot type for different behavior
    std::string robot_type_ = "humanoid";  // Default type
    
    // Weight control for arm_sdk mode
    double current_weight_ = 0.0;
};
