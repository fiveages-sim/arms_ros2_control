#ifndef TJ2_HARDWARE__TJ2_HARDWARE_HPP_
#define TJ2_HARDWARE__TJ2_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "MarvinSDK.h"

namespace tj2_hardware
{

class TJ2Hardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TJ2Hardware)

  // Hardware interface lifecycle methods
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  // Hardware interface methods
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Hardware parameters
  std::string device_ip_;
  int device_port_;
  double simulation_mode_;
  double read_timeout_;
  double write_timeout_;
  int  robot_arm_left_right_;
  int  previous_message_frame_;
  DCSS frame_data_;
  // Joint data storage
  std::array<double, 7> hw_position_commands_;
  std::array<double, 7> hw_velocity_commands_;
  std::array<double, 7> hw_position_states_;
  std::array<double, 7> hw_velocity_states_;
  std::array<double, 7> hw_effort_states_;

  // Joint limits from URDF
  std::array<double, 7> position_lower_limits_;
  std::array<double, 7> position_upper_limits_;
  std::array<double, 7> velocity_limits_;
  std::array<double, 7> effort_limits_;

  // Connection status
  bool hardware_connected_;
  bool simulation_active_;

  // Dobot communication handle (placeholder)
  // void* dobot_handle_;

  // Helper methods
  bool connectToHardware();
  void disconnectFromHardware();
  bool readFromHardware(int robot_arm_left_right_, bool initial_frame);
  bool writeToHardware(int robot_arm_left_right_);
  void simulateHardware(const rclcpp::Duration & period);
  void enforceJointLimits();
  bool initializeJointLimits();
  void logJointStates();

  void splitIPToCharArrays(const char* ipStr, char octet1[4], char octet2[4], char octet3[4], char octet4[4]) {
        sscanf(ipStr, "%3[^.].%3[^.].%3[^.].%3[^.]", octet1, octet2, octet3, octet4);
    }
};

}  // namespace tj2_hardware

#endif  // TJ2_HARDWARE__TJ2_HARDWARE_HPP_