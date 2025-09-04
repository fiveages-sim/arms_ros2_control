#pragma once

#include <string>
#include <vector>
#include <memory>
#include <functional>

// 前向声明，避免包含具体的Unitree消息头文件
namespace unitree {
namespace robot {
class ChannelPublisherBase;
class ChannelSubscriberBase;
}
}

/**
 * @brief Unitree机器人通信抽象接口
 * 
 * 这个接口定义了与Unitree机器人通信的标准协议，
 * 具体的消息类型由实现类决定（Go2或G1）
 */
class UnitreeCommunicator {
public:
    // 机器人状态数据结构
    struct RobotState {
        std::vector<double> joint_position;
        std::vector<double> joint_velocity;
        std::vector<double> joint_effort;
        std::vector<double> imu_quaternion;  // w, x, y, z
        std::vector<double> imu_gyroscope;   // x, y, z
        std::vector<double> imu_accelerometer; // x, y, z
        std::vector<double> foot_force;      // 4个足端力
        std::vector<double> high_position;   // x, y, z
        std::vector<double> high_velocity;   // vx, vy, vz
    };

    // 机器人命令数据结构
    struct RobotCommand {
        std::vector<double> joint_position;
        std::vector<double> joint_velocity;
        std::vector<double> joint_effort;
        std::vector<double> joint_kp;
        std::vector<double> joint_kd;
    };

    virtual ~UnitreeCommunicator() = default;

    /**
     * @brief 初始化通信器
     * @param domain DDS域ID
     * @param network_interface 网络接口
     * @return 是否初始化成功
     */
    virtual bool initialize(int domain, const std::string& network_interface) = 0;

    /**
     * @brief 读取机器人状态
     * @param state 输出的状态数据
     * @return 是否读取成功
     */
    virtual bool readState(RobotState& state) = 0;

    /**
     * @brief 发送机器人命令
     * @param command 要发送的命令数据
     * @return 是否发送成功
     */
    virtual bool writeCommand(const RobotCommand& command) = 0;

    /**
     * @brief 设置状态更新回调
     * @param callback 状态更新时的回调函数
     */
    virtual void setStateCallback(std::function<void(const RobotState&)> callback) = 0;

    /**
     * @brief 获取机器人类型名称
     * @return 机器人类型字符串（如"go2", "g1"）
     */
    virtual std::string getRobotType() const = 0;

    /**
     * @brief 获取关节数量
     * @return 关节数量
     */
    virtual int getJointCount() const = 0;

    /**
     * @brief 设置关节数量（从配置中读取）
     * @param joint_count 关节数量
     */
    virtual void setJointCount(int joint_count) = 0;

    /**
     * @brief 是否支持足力传感器
     * @return 是否支持
     */
    virtual bool supportsFootForce() const = 0;

    /**
     * @brief 是否支持高状态（位置/速度）
     * @return 是否支持
     */
    virtual bool supportsHighState() const = 0;
};
