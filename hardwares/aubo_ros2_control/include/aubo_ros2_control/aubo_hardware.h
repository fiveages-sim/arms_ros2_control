#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "aubo_ros2_control/aubo_commander.h"

namespace aubo_ros2_control
{
class AuboHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  static constexpr size_t kArmJointCount = 6;

  bool findSensorByName(
    const std::string & sensor_name,
    hardware_interface::ComponentInfo & sensor_info) const;

  void wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_efforts_;
  std::vector<double> joint_position_commands_;
  std::vector<double> last_sent_commands_;

  bool has_ft_sensor_{false};
  double ft_sensor_force_x_{0.0};
  double ft_sensor_force_y_{0.0};
  double ft_sensor_force_z_{0.0};
  double ft_sensor_torque_x_{0.0};
  double ft_sensor_torque_y_{0.0};
  double ft_sensor_torque_z_{0.0};
  std::mutex ft_mutex_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;

  std::unique_ptr<AuboCommander> commander_;

  std::string robot_ip_{"192.168.1.107"};
  int robot_port_{8899};
  std::string username_{"aubo"};
  std::string password_{"123456"};
  int collision_class_{6};
  double max_joint_acc_deg_{50.0};
  double max_joint_vel_deg_{50.0};
  std::string force_topic_{"/ft_sensor_wrench"};
  std::string command_mode_{"follow_mode"};
  bool move_blocking_{false};
  bool shutdown_on_disconnect_{false};
  int command_period_ms_{20};
  double position_command_threshold_{2e-4};

  std::chrono::steady_clock::time_point last_command_send_time_{};
  bool has_last_sent_command_{false};
};
}  // namespace aubo_ros2_control
