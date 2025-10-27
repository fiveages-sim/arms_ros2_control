#include "tj2_hardware.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include <random>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace tj2_hardware
{

hardware_interface::CallbackReturn TJ2Hardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "Initializing TJ2 Hardware Interface...");

  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize joint data vectors
  // hw_position_commands_.resize(info_.joints.size(), 0.0);
  // hw_velocity_commands_.resize(info_.joints.size(), 0.0);
  // hw_position_states_.resize(info_.joints.size(), 0.0);
  // hw_velocity_states_.resize(info_.joints.size(), 0.0);
  // hw_effort_states_.resize(info_.joints.size(), 0.0);

  // Initialize hardware connection status
  hardware_connected_ = false;
  simulation_active_ = false;

  // read ip address and connect robot 
    // 读取配置参数的辅助函数
const auto get_hardware_parameter = [&info](const std::string& parameter_name, const std::string& default_value) {
    if (auto it = info.hardware_parameters.find(parameter_name); it != info.hardware_parameters.end()) {
        return it->second;
    }
    return default_value;
    };

    std::string arm_ip = get_hardware_parameter("arm_ip", "192.168.2.160");
    robot_arm_left_right_ = std::stoi(get_hardware_parameter("arm_arm_left_right", "0")); // 0 is left, 1 is right
    RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "TJ2 Hardware Interface initialized with %zu joints", info_.joints.size());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TJ2Hardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "Configuring TJ2 Hardware Interface...");


  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TJ2Hardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "Cleaning up TJ2 Hardware Interface...");
  
  if (hardware_connected_) {
    disconnectFromHardware();
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TJ2Hardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "Activating TJ2 Hardware Interface...");

  if (simulation_mode_) {
    RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "Running in simulation mode");
    simulation_active_ = true;
    
    // Initialize simulation states
    for (size_t i = 0; i < hw_position_states_.size(); i++) {
      hw_position_states_[i] = 0.0;
      hw_velocity_states_[i] = 0.0;
      hw_effort_states_[i] = 0.0;
      hw_position_commands_[i] = 0.0;
      hw_velocity_commands_[i] = 0.0;
    }
    
  } else {
    // Connect to real hardware
    if (!connectToHardware()) {
      RCLCPP_ERROR(rclcpp::get_logger("TJ2Hardware"), "Failed to connect to Dobot CR5 hardware");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Read initial joint states from hardware
    if (!readFromHardware(robot_arm_left_right_, true)) {
      RCLCPP_ERROR(rclcpp::get_logger("TJ2Hardware"), "Failed to read initial joint states from hardware");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize commands with current positions
    for (size_t i = 0; i < hw_position_commands_.size(); i++) {
      hw_position_commands_[i] = hw_position_states_[i];
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "TJ2 Hardware Interface activated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TJ2Hardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "Deactivating TJ2 Hardware Interface...");

  if (hardware_connected_) {
    disconnectFromHardware();
  }
  simulation_active_ = false;
  OnClearSet();
  if (robot_arm_left_right_ == 0)
  {
    OnSetTargetState_A(1);
  }
  else
  {
    OnSetTargetState_B(1);
  }
  
  OnSetSend();
  usleep(100000);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TJ2Hardware::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "Shutting down TJ2 Hardware Interface...");
  
  if (hardware_connected_) {
    disconnectFromHardware();
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TJ2Hardware::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(rclcpp::get_logger("TJ2Hardware"), "Error in TJ2 Hardware Interface");
  
  // Attempt to safely disconnect from hardware
  if (hardware_connected_) {
    disconnectFromHardware();
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TJ2Hardware::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger("TJ2Hardware"), "Exporting state interfaces for %zu joints", info_.joints.size());

  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_position_states_[i]));
    
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &hw_velocity_states_[i]));
    
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_EFFORT,
        &hw_effort_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> TJ2Hardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger("TJ2Hardware"), "Exporting command interfaces for %zu joints", info_.joints.size());

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_position_commands_[i]));
    
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &hw_velocity_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type TJ2Hardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (simulation_active_) {
    simulateHardware(period);
    return hardware_interface::return_type::OK;
  }

  if (!hardware_connected_) {
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger("TJ2Hardware"), 
      *std::make_shared<rclcpp::Clock>(), 5000, 
      "Not connected to hardware");
    return hardware_interface::return_type::ERROR;
  }

  if (!readFromHardware(robot_arm_left_right_, false)) {
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger("TJ2Hardware"), 
      *std::make_shared<rclcpp::Clock>(), 5000, 
      "Failed to read from hardware");
    return hardware_interface::return_type::ERROR;
  }

  // Log joint states periodically for debugging
  OnGetBuf(&frame_data_); //订阅数据
  

  static auto last_log_time = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 5) {
    logJointStates();
    last_log_time = now;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TJ2Hardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (simulation_active_) {
    // In simulation, commands are handled in the read method
    return hardware_interface::return_type::OK;
  }

  if (!hardware_connected_) {
    return hardware_interface::return_type::ERROR;
  }

  // Enforce joint limits before sending commands
  enforceJointLimits();
  if (!writeToHardware(robot_arm_left_right_)) {
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger("TJ2Hardware"), 
      *std::make_shared<rclcpp::Clock>(), 5000, 
      "Failed to write to hardware");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

bool TJ2Hardware::connectToHardware()
{
  RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "Connecting to Dobot CR5 at %s:%d", device_ip_.c_str(), device_port_);
  
  // TODO: Implement actual Dobot connection using Dobot SDK
  // Example:
  unsigned char octet1;
  unsigned char octet2;
  unsigned char octet3;
  unsigned char octet4;
  
  hardware_connected_ = OnLinkTo(octet1,octet2,octet3,octet4) == false;
    
  
  // Simulate connection for demonstration
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  hardware_connected_ = true;
  
  RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "Successfully connected to Dobot CR5 hardware");
  return hardware_connected_;
}

void TJ2Hardware::disconnectFromHardware()
{
  RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "Disconnecting from Dobot CR5 hardware");
  
  // TODO: Implement actual Dobot disconnection
  OnRelease();
  
  hardware_connected_ = false;
  RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "Disconnected from Dobot CR5 hardware");
}

bool TJ2Hardware::readFromHardware(int robot_arm_left_right, bool initial_frame)
{
  OnGetBuf(&frame_data_);
  if (initial_frame)
  {
    previous_message_frame_ = frame_data_.m_Out[robot_arm_left_right].m_OutFrameSerial;
  }

  if (previous_message_frame_ - frame_data_.m_Out[robot_arm_left_right].m_OutFrameSerial < 2)
  {
    for (size_t i = 0; i < 7; i++) {
        hw_position_states_[i] = frame_data_.m_Out[robot_arm_left_right].m_FB_Joint_Pos[i];
        hw_velocity_states_[i] = frame_data_.m_Out[robot_arm_left_right].m_FB_Joint_Vel[i];
        hw_effort_states_[i] = frame_data_.m_Out[robot_arm_left_right].m_FB_Joint_SToq[i];
    }
    return true;
  }
   else
   {
        return false;
   }
}

bool TJ2Hardware::writeToHardware(int robot_arm_left_right)
{
  // TODO: Implement actual joint command sending to Dobot
  if (robot_arm_left_right ==0)
  {
    OnSetJointCmdPos_A(hw_position_states_.data());

  }
  else
  {
    OnSetJointCmdPos_B(hw_position_states_.data());
  }
  
  return true;
}

void TJ2Hardware::simulateHardware(const rclcpp::Duration & period)
{
  // Simple simulation: move toward commanded position with velocity limits
  for (size_t i = 0; i < hw_position_states_.size(); i++) {
    double position_error = hw_position_commands_[i] - hw_position_states_[i];
    double max_velocity_change = velocity_limits_[i] * period.seconds();
    
    // Calculate desired velocity
    double desired_velocity = std::copysign(
      std::min(std::abs(position_error) / period.seconds(), velocity_limits_[i]),
      position_error
    );
    
    // Apply velocity limits
    double velocity_change = desired_velocity - hw_velocity_states_[i];
    if (std::abs(velocity_change) > max_velocity_change) {
      velocity_change = std::copysign(max_velocity_change, velocity_change);
    }
    
    hw_velocity_states_[i] += velocity_change;
    hw_position_states_[i] += hw_velocity_states_[i] * period.seconds();
    
    // Simulate effort based on acceleration
    hw_effort_states_[i] = velocity_change / period.seconds() * 0.1;
  }
}

void TJ2Hardware::enforceJointLimits()
{
  for (size_t i = 0; i < hw_position_commands_.size(); i++) {
    // Enforce position limits
    if (hw_position_commands_[i] < position_lower_limits_[i]) {
      hw_position_commands_[i] = position_lower_limits_[i];
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("TJ2Hardware"), 
        *std::make_shared<rclcpp::Clock>(), 10000, 
        "Joint %s position command (%.3f) below lower limit (%.3f)", 
        info_.joints[i].name.c_str(), hw_position_commands_[i], position_lower_limits_[i]);
    } else if (hw_position_commands_[i] > position_upper_limits_[i]) {
      hw_position_commands_[i] = position_upper_limits_[i];
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("TJ2Hardware"), 
        *std::make_shared<rclcpp::Clock>(), 10000, 
        "Joint %s position command (%.3f) above upper limit (%.3f)", 
        info_.joints[i].name.c_str(), hw_position_commands_[i], position_upper_limits_[i]);
    }
    
    // Enforce velocity limits
    if (std::abs(hw_velocity_commands_[i]) > velocity_limits_[i]) {
      hw_velocity_commands_[i] = std::copysign(velocity_limits_[i], hw_velocity_commands_[i]);
    }
  }
}

bool TJ2Hardware::initializeJointLimits()
{
  // position_lower_limits_.resize(info_.joints.size());
  // position_upper_limits_.resize(info_.joints.size());
  // velocity_limits_.resize(info_.joints.size());
  // effort_limits_.resize(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); i++) {
    const auto & joint = info_.joints[i];
    
    // Parse position limits
    for (const auto & command_interface : joint.command_interfaces) {
      if (command_interface.name == hardware_interface::HW_IF_POSITION) {
        if (!command_interface.min.empty()) {
          position_lower_limits_[i] = std::stod(command_interface.min);
        }
        if (!command_interface.max.empty()) {
          position_upper_limits_[i] = std::stod(command_interface.max);
        }
      } else if (command_interface.name == hardware_interface::HW_IF_VELOCITY) {
        if (!command_interface.max.empty()) {
          velocity_limits_[i] = std::stod(command_interface.max);
        }
      }
    }
    
    // Parse effort limits from joint limits in URDF
    effort_limits_[i] = std::stod(joint.command_interfaces[0].max);
    
    RCLCPP_DEBUG(
      rclcpp::get_logger("TJ2Hardware"),
      "Joint %s: pos=[%.3f, %.3f], vel_limit=%.3f, effort_limit=%.3f",
      joint.name.c_str(), position_lower_limits_[i], position_upper_limits_[i],
      velocity_limits_[i], effort_limits_[i]);
  }

  return true;
}

void TJ2Hardware::logJointStates()
{
  std::stringstream ss;
  ss << "Joint States - ";
  for (size_t i = 0; i < hw_position_states_.size(); i++) {
    ss << info_.joints[i].name << ": pos=" << std::fixed << std::setprecision(3) << hw_position_states_[i]
       << ", vel=" << hw_velocity_states_[i] << ", cmd=" << hw_position_commands_[i];
    if (i < hw_position_states_.size() - 1) ss << " | ";
  }
  RCLCPP_DEBUG(rclcpp::get_logger("TJ2Hardware"), "%s", ss.str().c_str());
}

}  // namespace tj2_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  tj2_hardware::TJ2Hardware,
  hardware_interface::SystemInterface)

