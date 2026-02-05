#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "app/joint_controller.h"

#include <memory>
#include <string>
#include <vector>
#include <optional>

namespace arx_x5_ros2_control {

    // 臂配置常量
    constexpr int ARM_LEFT = 0;
    constexpr int ARM_RIGHT = 1;
    constexpr int ARM_DUAL = 2;

class ArxX5Hardware : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ArxX5Hardware)

    // 初始化
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params) override;

    // 导出状态接口 (position, velocity, effort)
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    // 导出命令接口 (position)
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // 激活 (连接机械臂)
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;

    // 停用 (断开连接)
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

    // 读取状态 (从SDK获取关节状态)
    hardware_interface::return_type read(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override;

    // 写入命令 (发送关节命令到SDK)
    hardware_interface::return_type write(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override;

private:
    // ROS2 节点和日志器
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Logger get_logger() const 
    { 
        return logger_.value();
    }
    std::optional<rclcpp::Logger> logger_;

    // ARX5 SDK控制器（支持单臂和双臂）
    std::shared_ptr<arx::Arx5JointController> controllers_[2];  // [0]=左臂, [1]=右臂

    // 配置参数
    std::string arm_config_;       // 臂配置: "LEFT", "RIGHT", "DUAL"
    int arm_index_;                // 0=LEFT, 1=RIGHT, 2=DUAL
    
    // 单臂配置（向后兼容）
    std::string robot_model_;      // 机器人型号 (X5, L5等)
    std::string can_interface_;    // CAN接口名 (can0, can1等)
    
    // 双臂配置
    std::string left_robot_model_;   // 左臂机器人型号
    std::string right_robot_model_;  // 右臂机器人型号
    std::string left_can_interface_; // 左臂CAN接口
    std::string right_can_interface_; // 右臂CAN接口
    
    int joint_count_;              // 总关节数量
    int left_joint_count_;         // 左臂关节数量
    int right_joint_count_;        // 右臂关节数量

    // 关节名称
    std::vector<std::string> joint_names_;           // 所有关节名称（按顺序：左臂+右臂）
    std::vector<std::string> left_joint_names_;      // 左臂关节名称
    std::vector<std::string> right_joint_names_;     // 右臂关节名称

    // 状态数据 (从SDK读取)
    std::vector<double> position_states_;
    std::vector<double> velocity_states_;
    std::vector<double> effort_states_;

    // 命令数据 (发送到SDK)
    std::vector<double> position_commands_;

    // 夹爪
    bool has_gripper_;
    std::vector<std::string> gripper_joint_names_;  // 夹爪关节名称列表（支持多个夹爪关节）
    std::vector<double> gripper_position_states_;    // 夹爪位置状态（每个夹爪关节独立的值）
    std::vector<double> gripper_velocity_states_;   // 夹爪速度状态
    std::vector<double> gripper_effort_states_;      // 夹爪力矩状态
    std::vector<double> gripper_position_commands_;  // 夹爪位置命令
    
    // 辅助函数：字符串标准化（用于参数解析）
    std::string normalizeString(const std::string& str);

    // 参数管理辅助函数
    template<typename T>
    T get_node_param(const std::string& name, const T& default_val)
    {
        if (!node_->has_parameter(name)) {
            node_->declare_parameter<T>(name, default_val);
        }
        return node_->get_parameter(name).get_value<T>();
    }

    // 声明节点参数
    void declare_node_parameters();
};

}  // namespace arx_x5_ros2_control
