#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace can_ros2_control
{

class O6CanHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface::ConstSharedPtr>
  on_export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

private:
  static constexpr std::size_t kJointCount = 6;
  static constexpr uint32_t kRightHandCanId = 0x27;
  static constexpr uint32_t kLeftHandCanId = 0x28;
  static constexpr uint8_t kAngleCommand = 0x01;

  void load_parameters();
  bool validate_joint_interfaces() const;
  bool open_socket();
  void close_socket();
  bool receive_feedback(const rclcpp::Duration& period);
  bool send_command(const std::array<uint8_t, kJointCount>& raw_command);

  uint8_t radians_to_raw(double radians, std::size_t joint_index) const;
  double raw_to_radians(uint8_t raw, std::size_t joint_index) const;
  bool raw_command_changed(const std::array<uint8_t, kJointCount>& raw_command) const;

  static bool parse_bool(const std::string& value, bool default_value);
  static int parse_int(const std::string& value, int default_value);
  static double initial_value_for_joint(const hardware_interface::ComponentInfo& joint);
  static std::array<double, kJointCount> default_upper_limits(const std::vector<std::string>& joint_names);

  std::string can_interface_ = "can0";
  std::string hand_side_ = "right";
  std::string hand_type_;
  uint32_t can_id_ = kRightHandCanId;
  bool read_feedback_ = true;
  bool send_initial_command_ = false;
  int feedback_timeout_ms_ = 1;
  int command_deadband_raw_ = 0;

  int socket_fd_ = -1;
  bool command_sent_ = false;

  std::vector<std::string> joint_names_;
  std::array<double, kJointCount> lower_limits_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, kJointCount> upper_limits_{0.58, 1.30, 1.60, 1.60, 1.60, 1.60};
  std::array<double, kJointCount> hw_positions_{};
  std::array<double, kJointCount> hw_velocities_{};
  std::array<double, kJointCount> hw_efforts_{};
  std::array<double, kJointCount> hw_commands_{};
  std::array<double, kJointCount> previous_positions_{};
  std::array<uint8_t, kJointCount> last_raw_command_{255, 255, 255, 255, 255, 255};
};

}  // namespace can_ros2_control
