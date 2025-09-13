#include "unitree_ros2_control/HumanoidCommunicator.h"
#include "crc32.h"
#include <rclcpp/rclcpp.hpp>

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::robot;

HumanoidCommunicator::HumanoidCommunicator() {
    resizeStateData();
}

bool HumanoidCommunicator::initialize(int domain, const std::string& network_interface) {
    try {
        ChannelFactory::Instance()->Init(domain, network_interface);

        low_cmd_publisher_ = std::make_shared<ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>(TOPIC_LOWCMD);
        low_cmd_publisher_->InitChannel();

        low_state_subscriber_ = std::make_shared<ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(TOPIC_LOWSTATE);
        low_state_subscriber_->InitChannel(
            [this](auto&& PH1) {
                lowStateMessageHandle(std::forward<decltype(PH1)>(PH1));
            },
            1);

        initLowCmd();
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("HumanoidCommunicator"), "Failed to initialize humanoid communicator: %s", e.what());
        return false;
    }
}

bool HumanoidCommunicator::readState(RobotState& state) {
    std::lock_guard lock(state_mutex_);
    if (state_updated_) {
        state = cached_state_;
        state_updated_ = false;
        return true;
    }
    return false;
}

bool HumanoidCommunicator::writeCommand(const RobotCommand& command) {
    try {
        // Set control mode
        low_cmd_.mode_pr() = mode_pr_;
        low_cmd_.mode_machine() = mode_machine_;
        
        // Set motor commands only for valid joints
        int valid_joints = std::min(joint_count_, static_cast<int>(command.joint_position.size()));
        for (int i = 0; i < valid_joints; ++i) {
            low_cmd_.motor_cmd()[i].mode() = 1;  // 1:Enable, 0:Disable
            low_cmd_.motor_cmd()[i].q() = static_cast<float>(command.joint_position[i]);
            low_cmd_.motor_cmd()[i].dq() = static_cast<float>(command.joint_velocity[i]);
            low_cmd_.motor_cmd()[i].kp() = static_cast<float>(command.joint_kp[i]);
            low_cmd_.motor_cmd()[i].kd() = static_cast<float>(command.joint_kd[i]);
            low_cmd_.motor_cmd()[i].tau() = static_cast<float>(command.joint_effort[i]);
        }

        // Calculate and set CRC
        low_cmd_.crc() = crc32_core(reinterpret_cast<uint32_t*>(&low_cmd_),
                                   (sizeof(unitree_hg::msg::dds_::LowCmd_) >> 2) - 1);
        low_cmd_publisher_->Write(low_cmd_);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("HumanoidCommunicator"), "Failed to write command: %s", e.what());
        return false;
    }
}

void HumanoidCommunicator::setStateCallback(std::function<void(const RobotState&)> callback) {
    state_callback_ = callback;
}

void HumanoidCommunicator::setControlMode(uint8_t mode_pr, uint8_t mode_machine) {
    mode_pr_ = mode_pr;
    mode_machine_ = mode_machine;
    RCLCPP_INFO(rclcpp::get_logger("HumanoidCommunicator"), 
               "Set control mode: PR=%d, Machine=%d", mode_pr_, mode_machine_);
}

void HumanoidCommunicator::lowStateMessageHandle(const void* messages) {
    low_state_ = *static_cast<const unitree_hg::msg::dds_::LowState_*>(messages);
    
    // CRC validation
    if (low_state_.crc() != crc32_core(reinterpret_cast<uint32_t*>(&low_state_),
                                      (sizeof(unitree_hg::msg::dds_::LowState_) >> 2) - 1)) {
        RCLCPP_WARN(rclcpp::get_logger("HumanoidCommunicator"), "Low state CRC error");
        return;
    }
    
    std::lock_guard lock(state_mutex_);
    
    // Update machine mode
    if (mode_machine_ != low_state_.mode_machine()) {
        if (mode_machine_ == 0) {
            RCLCPP_INFO(rclcpp::get_logger("HumanoidCommunicator"), 
                       "G1 type: %d", static_cast<int>(low_state_.mode_machine()));
        }
        mode_machine_ = low_state_.mode_machine();
    }
    
    for (int i = 0; i < joint_count_ && i < static_cast<int>(cached_state_.joint_position.size()); ++i) {
        cached_state_.joint_position[i] = low_state_.motor_state()[i].q();
        cached_state_.joint_velocity[i] = low_state_.motor_state()[i].dq();
        cached_state_.joint_effort[i] = low_state_.motor_state()[i].tau_est();
    }

    cached_state_.imu_quaternion[0] = low_state_.imu_state().quaternion()[0];
    cached_state_.imu_quaternion[1] = low_state_.imu_state().quaternion()[1];
    cached_state_.imu_quaternion[2] = low_state_.imu_state().quaternion()[2];
    cached_state_.imu_quaternion[3] = low_state_.imu_state().quaternion()[3];

    cached_state_.imu_gyroscope[0] = low_state_.imu_state().gyroscope()[0];
    cached_state_.imu_gyroscope[1] = low_state_.imu_state().gyroscope()[1];
    cached_state_.imu_gyroscope[2] = low_state_.imu_state().gyroscope()[2];

    cached_state_.imu_accelerometer[0] = low_state_.imu_state().accelerometer()[0];
    cached_state_.imu_accelerometer[1] = low_state_.imu_state().accelerometer()[1];
    cached_state_.imu_accelerometer[2] = low_state_.imu_state().accelerometer()[2];

    cached_state_.foot_force[0] = 0.0;
    cached_state_.foot_force[1] = 0.0;
    cached_state_.foot_force[2] = 0.0;
    cached_state_.foot_force[3] = 0.0;

    state_updated_ = true;
    
    if (state_callback_) {
        state_callback_(cached_state_);
    }
}

void HumanoidCommunicator::setJointCount(int joint_count) {
    joint_count_ = joint_count;
    resizeStateData();
}

void HumanoidCommunicator::resizeStateData() {
    cached_state_.joint_position.resize(joint_count_, 0.0);
    cached_state_.joint_velocity.resize(joint_count_, 0.0);
    cached_state_.joint_effort.resize(joint_count_, 0.0);
    cached_state_.imu_quaternion.resize(4, 0.0);
    cached_state_.imu_gyroscope.resize(3, 0.0);
    cached_state_.imu_accelerometer.resize(3, 0.0);
    cached_state_.foot_force.resize(4, 0.0);
    cached_state_.high_position.resize(3, 0.0);
    cached_state_.high_velocity.resize(3, 0.0);
}

void HumanoidCommunicator::initLowCmd() {
    // Initialize control mode
    low_cmd_.mode_pr() = mode_pr_;
    low_cmd_.mode_machine() = mode_machine_;
    
    // Initialize motor commands for all joints
    for (int i = 0; i < joint_count_; i++) {
        low_cmd_.motor_cmd()[i].mode() = 1;  // 1:Enable, 0:Disable
        low_cmd_.motor_cmd()[i].q() = 0;
        low_cmd_.motor_cmd()[i].kp() = 60;   // 设置kp=60
        low_cmd_.motor_cmd()[i].dq() = 0;
        low_cmd_.motor_cmd()[i].kd() = 2;    // 设置kd=2
        low_cmd_.motor_cmd()[i].tau() = 0;
    }
}
