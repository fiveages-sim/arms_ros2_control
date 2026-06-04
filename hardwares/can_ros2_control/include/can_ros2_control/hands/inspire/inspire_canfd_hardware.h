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

class InspireCanfdHardware : public hardware_interface::SystemInterface
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
  static constexpr std::size_t kActuatorCount = 6;

  void load_parameters();
  bool validate_joint_interfaces() const;
  bool open_socket();
  void close_socket();
  bool initialize_hand();
  bool read_feedback(
    std::array<double, kJointCount>& positions,
    std::array<double, kJointCount>& efforts);
  bool write_register(uint16_t address, uint16_t value);
  bool read_register(uint16_t address, uint16_t& value);
  bool send_read_request(uint16_t address, uint8_t byte_count);
  bool send_write_request(uint16_t address, uint16_t value);
  bool receive_register_response(
    bool write_response,
    uint16_t address,
    uint8_t expected_length,
    std::array<uint8_t, 8>& data);

  std::array<uint16_t, kActuatorCount> joints_to_actuators(
    const std::array<double, kJointCount>& joints) const;
  std::array<double, kJointCount> actuators_to_joints(
    const std::array<uint16_t, kActuatorCount>& actuators) const;
  uint16_t joint_to_actuator_value(double radians, std::size_t joint_index) const;
  double actuator_value_to_joint(uint16_t value, std::size_t joint_index) const;
  bool command_changed(const std::array<uint16_t, kActuatorCount>& actuator_values) const;

  static bool parse_bool(const std::string& value, bool default_value);
  static int parse_int(const std::string& value, int default_value);
  static double initial_value_for_joint(const hardware_interface::ComponentInfo& joint);
  static std::array<double, kJointCount> default_upper_limits(
    const std::vector<std::string>& joint_names);

  std::string can_interface_ = "can0";
  std::string hand_side_ = "left";
  uint8_t hand_id_ = 2;
  bool read_feedback_enabled_ = true;
  bool wait_write_ack_ = false;
  int feedback_timeout_ms_ = 2;
  int frame_tx_retries_ = 20;
  int inter_frame_delay_us_ = 1000;
  int command_deadband_raw_ = 1;
  uint16_t default_speed_ = 4000;
  uint16_t default_force_ = 6000;

  int socket_fd_ = -1;
  bool command_sent_ = false;

  std::vector<std::string> joint_names_;
  std::array<double, kJointCount> lower_limits_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::array<double, kJointCount> upper_limits_{1.1641, 0.5864, 1.4381, 1.4381, 1.4381, 1.4381};
  std::array<double, kJointCount> hw_positions_{};
  std::array<double, kJointCount> previous_positions_{};
  std::array<double, kJointCount> hw_velocities_{};
  std::array<double, kJointCount> hw_efforts_{};
  std::array<double, kJointCount> hw_commands_{};
  std::array<uint16_t, kActuatorCount> last_command_values_{};
};

}  // namespace can_ros2_control
