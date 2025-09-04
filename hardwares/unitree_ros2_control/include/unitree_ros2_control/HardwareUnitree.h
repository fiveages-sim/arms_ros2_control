#pragma once

#include "hardware_interface/system_interface.hpp"
#include "unitree_ros2_control/UnitreeCommunicator.h"
#include "unitree_ros2_control/UnitreeCommunicatorFactory.h"

class HardwareUnitree final : public hardware_interface::SystemInterface {
public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:
    std::vector<double> joint_torque_command_;
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocities_command_;
    std::vector<double> joint_kp_command_;
    std::vector<double> joint_kd_command_;

    std::vector<double> joint_position_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_effort_;

    std::vector<double> imu_states_;
    std::vector<double> foot_force_;
    std::vector<double> high_states_;

    std::unordered_map<std::string, std::vector<std::string> > joint_interfaces = {
        {"position", {}},
        {"velocity", {}},
        {"effort", {}}
    };


    void initializeCommunicator();
    void exportSensorStateInterfaces(std::vector<hardware_interface::StateInterface>& state_interfaces);
    bool findSensorByName(const std::string& sensor_name, hardware_interface::ComponentInfo& sensor_info);

    std::unique_ptr<UnitreeCommunicator> communicator_;
    std::string robot_type_ = "quadruped"; // 默认机器人类型
    std::string network_interface_ = "lo";
    int domain_ = 1;
    bool show_foot_force_ = false;
    bool enable_high_state_ = true; // 是否启用高状态读取（仿真环境）
};
