#include "unitree_ros2_control/HumanoidCommunicator.h"
#include "unitree_ros2_control/RobotJointConfig.h"
#include "crc32.h"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>
#include <array>

#define ARM_SDK "rt/arm_sdk"
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
        
        // 根据robot_type选择command topic
        std::string command_topic = (robot_type_ == "humanoid_arm_sdk" || robot_type_ == "g1_arm_sdk") ? 
                                   ARM_SDK : TOPIC_LOWCMD;
        low_cmd_publisher_ = std::make_shared<ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>>(command_topic.c_str());
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
        // 根据robot_type决定是否设置control mode
        if (robot_type_ != "humanoid_arm_sdk" && robot_type_ != "g1_arm_sdk") {
            low_cmd_.mode_pr() = mode_pr_;
            low_cmd_.mode_machine() = mode_machine_;
        }

        // 对于arm_sdk模式，设置weight控制（使用kNotUsedJoint）
        if (robot_type_ == "humanoid_arm_sdk" || robot_type_ == "g1_arm_sdk") {
            low_cmd_.motor_cmd()[static_cast<int>(kNotUsedJoint)].q(current_weight_);
        }

        // Set motor commands only for valid joints
        int valid_joints = std::min(joint_count_, static_cast<int>(command.joint_position.size()));
        for (int i = 0; i < valid_joints; ++i) {
            // 根据robot_type决定是否设置motor mode
            if (robot_type_ != "humanoid_arm_sdk" && robot_type_ != "g1_arm_sdk") {
                low_cmd_.motor_cmd()[i].mode() = 1;  // 1:Enable, 0:Disable
            }
            low_cmd_.motor_cmd()[i].q() = static_cast<float>(command.joint_position[i]);
            low_cmd_.motor_cmd()[i].dq() = static_cast<float>(command.joint_velocity[i]);
            low_cmd_.motor_cmd()[i].kp() = static_cast<float>(command.joint_kp[i]);
            low_cmd_.motor_cmd()[i].kd() = static_cast<float>(command.joint_kd[i]);
            low_cmd_.motor_cmd()[i].tau() = static_cast<float>(command.joint_effort[i]);
        }

        // 根据robot_type决定是否计算CRC
        if (robot_type_ != "humanoid_arm_sdk" && robot_type_ != "g1_arm_sdk") {
            low_cmd_.crc() = crc32_core(reinterpret_cast<uint32_t*>(&low_cmd_),
                                       (sizeof(unitree_hg::msg::dds_::LowCmd_) >> 2) - 1);
        }
        
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
    
    // // CRC validation
    // if (low_state_.crc() != crc32_core(reinterpret_cast<uint32_t*>(&low_state_),
    //                                   (sizeof(unitree_hg::msg::dds_::LowState_) >> 2) - 1)) {
    //     RCLCPP_WARN(rclcpp::get_logger("HumanoidCommunicator"), "Low state CRC error");
    //     return;
    // }
    
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

void HumanoidCommunicator::setRobotType(const std::string& robot_type) {
    robot_type_ = robot_type;
    RCLCPP_INFO(rclcpp::get_logger("HumanoidCommunicator"), 
               "Set robot type to: %s", robot_type_.c_str());
}

void HumanoidCommunicator::activate() {
    if (robot_type_ == "g1_arm_sdk") {
        RCLCPP_INFO(rclcpp::get_logger("HumanoidCommunicator"), 
                   "Starting arm activation...");
        
        // 获取当前关节位置
        std::array<float, 17> current_jpos{};
        for (size_t i = 0; i < arm_joints.size(); ++i) {
            current_jpos[i] = cached_state_.joint_position[i];
        }
        
        // 打印当前关节位置用于调试
        RCLCPP_INFO(rclcpp::get_logger("HumanoidCommunicator"), 
                   "Current joint positions:");
        for (size_t i = 0; i < arm_joints.size(); ++i) {
            RCLCPP_INFO(rclcpp::get_logger("HumanoidCommunicator"), 
                       "Joint %zu: %f", i, current_jpos[i]);
        }
        
        // 目标位置（参考G1示例）
        std::array<float, 17> init_pos{0, 0, 0, 0, 0, 0, 0,
                                       0, 0, 0, 0, 0, 0, 0,
                                       0, 0, 0};
        
        // 初始化参数
        float init_time = 2.0f;
        float control_dt = 0.02f;
        int init_time_steps = static_cast<int>(init_time / control_dt);
        auto sleep_time = std::chrono::milliseconds(static_cast<int>(control_dt * 1000));
        
        RCLCPP_INFO(rclcpp::get_logger("HumanoidCommunicator"), 
                   "Initializing arms for %d steps...", init_time_steps);
        

        
        // 执行初始化循环（逐步增加权重）
        for (int i = 0; i < init_time_steps; ++i) {
            // 计算相位（用于权重和关节位置插值）
            float phase = 1.0f * i / init_time_steps;
            
            // 逐步增加权重（从0到1）
            current_weight_ = phase;  // 权重从0逐渐增加到1
            low_cmd_.motor_cmd()[static_cast<int>(kNotUsedJoint)].q(current_weight_);
            
            // 打印权重设置用于调试
            if (i % 50 == 0) {  // 每1秒打印一次
                RCLCPP_INFO(rclcpp::get_logger("HumanoidCommunicator"), 
                           "Step %d: Setting weight to %f (phase %f)", i, current_weight_, phase);
            }
            
            // 设置手臂关节（使用RobotJointConfig.h中的arm_joints）
            for (size_t j = 0; j < arm_joints.size(); ++j) {
                int joint_idx = static_cast<int>(arm_joints[j]);
                float target_pos = init_pos[j] * phase + current_jpos[j] * (1 - phase);
                low_cmd_.motor_cmd()[joint_idx].q(target_pos);
                low_cmd_.motor_cmd()[joint_idx].dq(0.0f);
                low_cmd_.motor_cmd()[joint_idx].kp(60.0f);
                low_cmd_.motor_cmd()[joint_idx].kd(1.5f);
                low_cmd_.motor_cmd()[joint_idx].tau(0.0f);
                
                RCLCPP_INFO(rclcpp::get_logger("HumanoidCommunicator"),
                     "Step %d, Joint %zu (idx %d): target=%f, phase=%f",
                     i, j, joint_idx, target_pos, phase);
            }
            
            // 发送命令
            low_cmd_publisher_->Write(low_cmd_);
            
            // 等待控制周期
            std::this_thread::sleep_for(sleep_time);
            
            // 打印进度
            if (i % 50 == 0) {  // 每1秒打印一次
                RCLCPP_INFO(rclcpp::get_logger("HumanoidCommunicator"), 
                           "Initialization progress: %d/%d (%.1f%%)", 
                           i, init_time_steps, phase * 100);
            }
        }
        
        RCLCPP_INFO(rclcpp::get_logger("HumanoidCommunicator"), 
                   "Arm activation completed");
    }
}

void HumanoidCommunicator::deactivate() {
    if (robot_type_ == "g1_arm_sdk") {
        RCLCPP_INFO(rclcpp::get_logger("HumanoidCommunicator"), 
                   "Stopping arm control...");
        
        // 权重逐渐减少（参考G1示例的停止部分）
        float stop_time = 5.0f;
        float control_dt = 0.02f;
        int stop_time_steps = static_cast<int>(stop_time / control_dt);
        float weight_rate = 0.2f;
        float delta_weight = weight_rate * control_dt;
        auto sleep_time = std::chrono::milliseconds(static_cast<int>(control_dt * 1000));
        
        for (int i = 0; i < stop_time_steps; ++i) {
            current_weight_ -= delta_weight;
            current_weight_ = std::clamp(current_weight_, 0.0, 1.0);
            
            low_cmd_.motor_cmd()[static_cast<int>(kNotUsedJoint)].q(current_weight_);
            low_cmd_publisher_->Write(low_cmd_);
            
            std::this_thread::sleep_for(sleep_time);
        }
        
        // 最终设置为0
        current_weight_ = 0.0f;
        low_cmd_.motor_cmd()[static_cast<int>(kNotUsedJoint)].q(current_weight_);
        low_cmd_publisher_->Write(low_cmd_);
        
        RCLCPP_INFO(rclcpp::get_logger("HumanoidCommunicator"), 
                   "Arm control stopped");
    }
}

