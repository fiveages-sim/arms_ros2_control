#include "unitree_ros2_control/QuadrupedCommunicator.h"
#include "crc32.h"
#include <rclcpp/rclcpp.hpp>

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::robot;

QuadrupedCommunicator::QuadrupedCommunicator() {
    // 初始化状态数据结构（使用默认关节数，后续会被配置覆盖）
    resizeStateData();
}

// 基类接口实现 - 默认启用高状态
bool QuadrupedCommunicator::initialize(int domain, const std::string& network_interface) {
    return initialize(domain, network_interface, true);
}

// 四足机器人特有的重载方法
bool QuadrupedCommunicator::initialize(int domain, const std::string& network_interface, bool enable_high_state) {
    try {
        ChannelFactory::Instance()->Init(domain, network_interface);

        // 初始化发布器
        low_cmd_publisher_ = std::make_shared<ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>(TOPIC_LOWCMD);
        low_cmd_publisher_->InitChannel();

        // 初始化订阅器
        low_state_subscriber_ = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(TOPIC_LOWSTATE);
        low_state_subscriber_->InitChannel(
            [this](auto&& PH1) {
                lowStateMessageHandle(std::forward<decltype(PH1)>(PH1));
            },
            1);

        // 只有在启用高状态时才创建订阅器
        if (enable_high_state) {
            high_state_subscriber_ = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(TOPIC_HIGHSTATE);
            high_state_subscriber_->InitChannel(
                [this](auto&& PH1) {
                    highStateMessageHandle(std::forward<decltype(PH1)>(PH1));
                },
                1);
        }

        initLowCmd();
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("QuadrupedCommunicator"), "Failed to initialize quadruped communicator: %s", e.what());
        return false;
    }
}

bool QuadrupedCommunicator::readState(RobotState& state) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_updated_) {
        state = cached_state_;
        state_updated_ = false;
        return true;
    }
    return false;
}

bool QuadrupedCommunicator::writeCommand(const RobotCommand& command) {
    try {
        // 更新命令（使用动态关节数）
        int cmd_joints = std::min(joint_count_, 20); // Unitree消息最多支持20个关节
        for (int i = 0; i < cmd_joints; ++i) {
            low_cmd_.motor_cmd()[i].mode() = 0x01;
            if (i < static_cast<int>(command.joint_position.size())) {
                low_cmd_.motor_cmd()[i].q() = static_cast<float>(command.joint_position[i]);
                low_cmd_.motor_cmd()[i].dq() = static_cast<float>(command.joint_velocity[i]);
                low_cmd_.motor_cmd()[i].kp() = static_cast<float>(command.joint_kp[i]);
                low_cmd_.motor_cmd()[i].kd() = static_cast<float>(command.joint_kd[i]);
                low_cmd_.motor_cmd()[i].tau() = static_cast<float>(command.joint_effort[i]);
            }
        }

        // 计算CRC并发送
        low_cmd_.crc() = crc32_core(reinterpret_cast<uint32_t*>(&low_cmd_),
                                   (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
        low_cmd_publisher_->Write(low_cmd_);
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("Go2Communicator"), "Failed to write command: %s", e.what());
        return false;
    }
}

void QuadrupedCommunicator::setStateCallback(std::function<void(const RobotState&)> callback) {
    state_callback_ = callback;
}

void QuadrupedCommunicator::lowStateMessageHandle(const void* messages) {
    low_state_ = *static_cast<const unitree_go::msg::dds_::LowState_*>(messages);
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // 更新关节状态（使用动态关节数）
    int state_joints = std::min(joint_count_, 12); // Go2最多12个关节
    for (int i = 0; i < state_joints && i < static_cast<int>(cached_state_.joint_position.size()); ++i) {
        cached_state_.joint_position[i] = low_state_.motor_state()[i].q();
        cached_state_.joint_velocity[i] = low_state_.motor_state()[i].dq();
        cached_state_.joint_effort[i] = low_state_.motor_state()[i].tau_est();
    }

    // 更新IMU状态
    cached_state_.imu_quaternion[0] = low_state_.imu_state().quaternion()[0]; // w
    cached_state_.imu_quaternion[1] = low_state_.imu_state().quaternion()[1]; // x
    cached_state_.imu_quaternion[2] = low_state_.imu_state().quaternion()[2]; // y
    cached_state_.imu_quaternion[3] = low_state_.imu_state().quaternion()[3]; // z

    cached_state_.imu_gyroscope[0] = low_state_.imu_state().gyroscope()[0];
    cached_state_.imu_gyroscope[1] = low_state_.imu_state().gyroscope()[1];
    cached_state_.imu_gyroscope[2] = low_state_.imu_state().gyroscope()[2];

    cached_state_.imu_accelerometer[0] = low_state_.imu_state().accelerometer()[0];
    cached_state_.imu_accelerometer[1] = low_state_.imu_state().accelerometer()[1];
    cached_state_.imu_accelerometer[2] = low_state_.imu_state().accelerometer()[2];

    // 更新足力传感器
    cached_state_.foot_force[0] = low_state_.foot_force()[0];
    cached_state_.foot_force[1] = low_state_.foot_force()[1];
    cached_state_.foot_force[2] = low_state_.foot_force()[2];
    cached_state_.foot_force[3] = low_state_.foot_force()[3];

    state_updated_ = true;
    
    if (state_callback_) {
        state_callback_(cached_state_);
    }
}

void QuadrupedCommunicator::highStateMessageHandle(const void* messages) {
    high_state_ = *static_cast<const unitree_go::msg::dds_::SportModeState_*>(messages);
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    // 更新高状态
    cached_state_.high_position[0] = high_state_.position()[0];
    cached_state_.high_position[1] = high_state_.position()[1];
    cached_state_.high_position[2] = high_state_.position()[2];
    
    cached_state_.high_velocity[0] = high_state_.velocity()[0];
    cached_state_.high_velocity[1] = high_state_.velocity()[1];
    cached_state_.high_velocity[2] = high_state_.velocity()[2];
}

void QuadrupedCommunicator::resizeStateData() {
    // 根据关节数量调整状态数据结构大小
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

void QuadrupedCommunicator::initLowCmd() {
    low_cmd_.head()[0] = 0xFE;
    low_cmd_.head()[1] = 0xEF;
    low_cmd_.level_flag() = 0xFF;
    low_cmd_.gpio() = 0;

    for (int i = 0; i < 20; i++) {
        low_cmd_.motor_cmd()[i].mode() = 0x01; // motor switch to servo (PMSM) mode
        low_cmd_.motor_cmd()[i].q() = 0;
        low_cmd_.motor_cmd()[i].kp() = 0;
        low_cmd_.motor_cmd()[i].dq() = 0;
        low_cmd_.motor_cmd()[i].kd() = 0;
        low_cmd_.motor_cmd()[i].tau() = 0;
    }
}
