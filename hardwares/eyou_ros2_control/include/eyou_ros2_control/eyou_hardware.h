#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "eu_harmonic.h"

#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <map>

namespace eyou_ros2_control {

/**
 * @brief Eyou Harmonic电机ROS2 Control硬件接口
 * 
 * 该类实现了hardware_interface::SystemInterface，通过Eyou Harmonic SDK控制CANopen电机
 * 支持同步位置模式（CSP），通过独立控制线程实现4ms周期的实时控制
 */
class EyouHardware : public hardware_interface::SystemInterface {
public:
    /**
     * @brief 初始化硬件接口
     * @param params 硬件组件接口参数
     * @return 初始化结果
     */
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams &params) override;
    
    /**
     * @brief 激活硬件接口（初始化CAN设备、配置PDO、启动控制线程）
     * @param previous_state 前一个生命周期状态
     * @return 激活结果
     */
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;
    
    /**
     * @brief 停用硬件接口（停止控制线程、释放CAN设备）
     * @param previous_state 前一个生命周期状态
     * @return 停用结果
     */
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    /**
     * @brief 导出状态接口（位置、速度、力矩）
     * @return 状态接口列表
     */
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> 
        on_export_state_interfaces() override;

    /**
     * @brief 导出命令接口（位置命令）
     * @return 命令接口列表
     */
    std::vector<hardware_interface::CommandInterface::SharedPtr> 
        on_export_command_interfaces() override;

    /**
     * @brief 读取电机状态（从本地字典读取）
     * @param time 当前时间
     * @param period 距离上次读取的时间间隔
     * @return 读取结果
     */
    hardware_interface::return_type read(
        const rclcpp::Time &time, 
        const rclcpp::Duration &period) override;

    /**
     * @brief 写入控制命令（更新命令缓存）
     * @param time 当前时间
     * @param period 距离上次写入的时间间隔
     * @return 写入结果
     */
    hardware_interface::return_type write(
        const rclcpp::Time &time, 
        const rclcpp::Duration &period) override;

private:
    /**
     * @brief 电机配置结构
     */
    struct MotorConfig {
        huint8 motor_id;                    // 电机节点ID
        double reduction_ratio;             // 减速比
        
        // 预计算的转换系数
        double position_scale_to_encoder;   // 弧度 → 编码器计数
        double position_scale_to_radian;    // 编码器计数 → 弧度
    };
    
    // === 固定参数 ===
    static constexpr int ENCODER_RESOLUTION = 65536;  // 编码器分辨率（每圈脉冲数，默认65536）
    static constexpr double EFFORT_SCALE = 0.001;    // 力矩转换系数
    
    // === 配置参数 ===
    huint8 device_index_;                   // CAN设备索引
    harmonic_DeviceType device_type_;       // CAN设备类型
    harmonic_Baudrate baudrate_;            // CAN波特率
    std::vector<huint8> motor_ids_;        // 电机节点ID列表
    int control_period_ms_;                // 控制周期（ms）
    int report_period_ms_;                 // 状态上报周期（ms）
    bool use_sync_;                        // 是否使用同步模式
    
    // === 电机配置 ===
    std::vector<MotorConfig> motor_configs_;
    
    // === 状态数据（ROS2单位：弧度、弧度/秒、N·m）===
    std::vector<double> joint_positions_;      // 实际位置（弧度）
    std::vector<double> joint_velocities_;      // 实际速度（弧度/秒）
    std::vector<double> joint_efforts_;        // 实际力矩（N·m）
    
    // === 命令数据（ROS2单位：弧度）===
    std::vector<double> joint_position_commands_;  // 位置命令（弧度）
    std::vector<hint32> raw_position_commands_;     // 位置命令（编码器计数）
    
    // === 原始数据（编码器单位）===
    std::vector<hint32> raw_positions_;        // 实际位置（编码器计数）
    std::vector<hint16> raw_efforts_;          // 实际力矩（驱动器原始值）
    
    // === 速度计算辅助 ===
    std::vector<double> last_positions_;       // 上次位置（用于速度计算）
    std::vector<hint32> last_raw_positions_;   // 上次原始位置
    rclcpp::Time last_read_time_;             // 上次读取时间
    
    // === 控制线程 ===
    std::thread csp_control_thread_;
    std::atomic<bool> running_;               // 控制线程运行标志
    
    // === 辅助函数 ===
    void calculateConversionScales(MotorConfig& config);
    hint32 radianToEncoder(size_t motor_index, double radian);
    double encoderToRadian(size_t motor_index, hint32 encoder);
    double encoderPerSecToRadianPerSec(size_t motor_index, double encoder_per_sec);
    
    bool parseMotorConfigs(const std::vector<double>& reduction_ratios = {});
    MotorConfig parseMotorConfig(huint8 motor_id, double reduction_ratio);
    
    bool initializeCAN();
    bool configurePDO();
    bool enableMotors();
    void cspControlLoop();                    // CSP控制线程函数
    void readMotorStates();                   // 从本地字典读取状态
    void setPos(huint8 dev_index, unsigned id, hint32 pos);  // 发送位置指令
};

} // namespace eyou_ros2_control

