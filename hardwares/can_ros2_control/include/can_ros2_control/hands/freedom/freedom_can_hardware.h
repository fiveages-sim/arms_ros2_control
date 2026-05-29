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

class FreedomCanHardware : public hardware_interface::SystemInterface
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
  static constexpr uint8_t kMoveCommand = 0x10;
  static constexpr uint8_t kAngleQueryCommand = 0xF1;
  static constexpr uint8_t kMaxProtocolAngle = 90;

  void load_parameters();
  bool validate_joint_interfaces() const;
  bool open_socket();
  void close_socket();
  bool receive_feedback(const rclcpp::Duration& period);
  bool send_query();
  bool send_command(const std::array<uint8_t, kJointCount>& angles);

  uint8_t radians_to_protocol_angle(double radians, std::size_t joint_index) const;
  double protocol_angle_to_radians(uint8_t angle, std::size_t joint_index) const;
  bool command_changed(const std::array<uint8_t, kJointCount>& angles) const;

  static uint32_t make_extended_id(uint8_t device_id, uint8_t command, uint8_t total_frames, uint8_t frame_seq);
  static uint8_t frame_device_id(uint32_t can_id);
  static uint8_t frame_command(uint32_t can_id);
  static bool parse_bool(const std::string& value, bool default_value);
  static int parse_int(const std::string& value, int default_value);
  static double initial_value_for_joint(const hardware_interface::ComponentInfo& joint);
  static std::array<double, kJointCount> default_upper_limits(const std::vector<std::string>& joint_names);

  std::string can_interface_ = "can0";
  uint8_t device_id_ = 0;
  bool read_feedback_ = true;
  int feedback_timeout_ms_ = 1;
  int command_deadband_deg_ = 0;

  int socket_fd_ = -1;
  bool command_sent_ = false;

  std::vector<std::string> joint_names_;
  std::array<double, kJointCount> lower_limits_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, kJointCount> upper_limits_{0.785, 0.29, 1.24, 1.24, 1.24, 1.24};
  std::array<double, kJointCount> hw_positions_{};
  std::array<double, kJointCount> hw_velocities_{};
  std::array<double, kJointCount> hw_efforts_{};
  std::array<double, kJointCount> hw_commands_{};
  std::array<double, kJointCount> previous_positions_{};
  std::array<uint8_t, kJointCount> last_command_angles_{0, 0, 0, 0, 0, 0};
};

}  // namespace can_ros2_control
