#include "tj2_hardware.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include <random>
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>

namespace tj2_ros2_control
{

hardware_interface::CallbackReturn TJ2Hardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "Initializing TJ2 Hardware Interface...");

  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("TJ2Hardware"));
  clock_ = std::make_shared<rclcpp::Clock>();
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
    
    // Get the number of joints from the hardware info
    size_t num_joints = info.joints.size();
    robot_arm_left_right_ = static_cast<int>(RobotArmConfig::LEFT_ARM);
    if (info_.hardware_parameters.find("left_right_arm") != info_.hardware_parameters.end()) {
      robot_arm_left_right_ = std::stoi(info_.hardware_parameters.at("left_right_arm"));
      RCLCPP_INFO(get_logger(), "Found left_right_arm parameter: %d", robot_arm_left_right_);
    } else {
      RCLCPP_WARN(get_logger(), "No left_right_arm parameter found, using default: %d", robot_arm_left_right_);
    }
   
    RCLCPP_INFO(get_logger(), "Initializing %zu joints", num_joints);

    // Resize all arrays based on the number of joints
    hw_position_commands_.resize(num_joints, 0.0);
    hw_velocity_commands_.resize(num_joints, 0.0);
    hw_position_states_.resize(num_joints, 0.0);
    hw_velocity_states_.resize(num_joints, 0.0);
    hw_effort_states_.resize(num_joints, 0.0);

    // Initialize joint limits arrays
    // position_lower_limits_.resize(num_joints);
    // position_upper_limits_.resize(num_joints);
    // velocity_limits_.resize(num_joints);
    // effort_limits_.resize(num_joints);

    // Populate joint limits from URDF
    // for (size_t i = 0; i < num_joints; i++) {
    //   const auto& joint = info.joints[i];
      
    //   // Set position limits (if available)
    //   position_lower_limits_[i] = joint.limits.min_position;
    //   position_upper_limits_[i] = joint.limits.max_position;
      
    //   // Set velocity limits (if available)
    //   velocity_limits_[i] = joint.limits.max_velocity;
      
    //   // Set effort limits (if available)
    //   effort_limits_[i] = joint.limits.max_effort;

    //   RCLCPP_DEBUG(get_logger(), "Joint %s: pos=[%.3f, %.3f], vel=%.3f, effort=%.3f",
    //               joint.name.c_str(),
    //               position_lower_limits_[i], position_upper_limits_[i],
    //               velocity_limits_[i], effort_limits_[i]);
    // }
  
  

  // Initialize hardware connection status
  hardware_connected_ = false;
  simulation_active_ = false;
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
  simulation_mode_ = false;
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
    RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "Running in real hardware mode");
    // Connect to real hardware
    if (!connectToHardware()) {
      RCLCPP_ERROR(rclcpp::get_logger("TJ2Hardware"), "Failed to connect to TJ2 hardware");
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
      hw_velocity_commands_[i] = hw_velocity_states_[i];
      // RCLCPP_ERROR(rclcpp::get_logger("TJ2Hardware"), "initia position ...  %f", hw_velocity_commands_[i]);
    }
  }
  OnClearSet();
  if (robot_arm_left_right_ == static_cast<int>(RobotArmConfig::LEFT_ARM))
  {
    // OnSetTargetState_A(1);
    // OnSetJointLmt_A(50, 50);
    double K[7] = {500,500,500,10,10,10,0}; //预设为参数最大上限，供参考。
    double D[7] = {0.1,0.1,0.1,0.3,0.3,1};//预设为参数最大上限，供参考。
    int type = 2; //type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控

    OnClearSet();
    OnSetCartKD_A(K, D,type) ;
    OnSetSend();
    usleep(100000);

    OnClearSet();
    OnSetJointLmt_A(10, 10) ;
    OnSetSend();
    usleep(100000);


    OnClearSet();
    OnSetTargetState_A(3) ; //3:torque mode; 1:position mode; 
    OnSetImpType_A(2) ;//type = 1 关节阻抗;type = 2 坐标阻抗;type = 3 力控
    OnSetSend();
    usleep(100000);
    RCLCPP_ERROR(rclcpp::get_logger("TJ2Hardware"), "cartesian impedance control ..........");
  }
  else if (robot_arm_left_right_ == static_cast<int>(RobotArmConfig::RIGHT_ARM))
  {
    
    OnSetTargetState_B(1);
    OnSetJointLmt_B(10, 10);
  }
  else if(robot_arm_left_right_ == static_cast<int>(RobotArmConfig::DUAL_ARM))
  {
    
    OnSetTargetState_A(1);
    OnSetJointLmt_A(50, 50);
    OnSetTargetState_B(1);
    OnSetJointLmt_B(50, 50);
  }
  

  OnSetSend();
  usleep(100000);

  OnGetBuf(&frame_data_);
  RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "current state of A arm:%d\n",frame_data_.m_State[0].m_CurState);
  RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "cmd state of A arm:%d\n",frame_data_.m_State[0].m_CmdState);
  RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "error code of A arms:%d\n",frame_data_.m_State[0].m_ERRCode);
  RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "cmd of vel and acc:%d %d\n",frame_data_.m_In[0].m_Joint_Vel_Ratio,frame_data_.m_In[0].m_Joint_Acc_Ratio);
  RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "TJ2 Hardware Interface activated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn TJ2Hardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "Deactivating TJ2 Hardware Interface...");

  simulation_active_ = false;
  OnClearSet();
  if (robot_arm_left_right_ == static_cast<int>(RobotArmConfig::LEFT_ARM))
  {
    OnSetTargetState_A(0);
  }
  else if (robot_arm_left_right_ == static_cast<int>(RobotArmConfig::RIGHT_ARM))
  {
    OnSetTargetState_B(0);
  }
  else if(robot_arm_left_right_ == static_cast<int>(RobotArmConfig::DUAL_ARM))
  {
    OnSetTargetState_A(0);
    OnSetTargetState_B(0);
  }
  
  OnSetSend();
  usleep(100000);

  if (hardware_connected_) {
    disconnectFromHardware();
  }
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
  // if (simulation_active_) {
  //   simulateHardware(period);
  //   return hardware_interface::return_type::OK;
  // }

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

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TJ2Hardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // if (simulation_active_) {
  //   // In simulation, commands are handled in the read method
  //   return hardware_interface::return_type::OK;
  // }

  if (!hardware_connected_) {
    return hardware_interface::return_type::ERROR;
  }
  /// convert rad to degree
  // RCLCPP_ERROR(rclcpp::get_logger("TJ2Hardware"), "write command ...  %f", hw_position_commands_[2]);
  std::vector<double> hw_commands;
  for(int i =0; i < hw_position_commands_.size(); i++)
  {
    hw_commands.push_back(radToDegree(hw_position_commands_[i]));
  }
  // RCLCPP_ERROR(rclcpp::get_logger("TJ2Hardware"), "write command deg ...  %f", hw_position_commands_[2]);
  // Enforce joint limits before sending commands
  // enforceJointLimits();    
  if (!writeToHardware(robot_arm_left_right_, hw_commands)) {
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger("TJ2Hardware"), 
      *clock_, 5000, 
      "Failed to write to hardware");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

bool TJ2Hardware::connectToHardware()
{
  device_ip_ = "192.168.1.190";
  RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "Connecting to TJ2 at %s:%d", device_ip_.c_str(), device_port_);
  
  // TODO: Implement actual Dobot connection using Dobot SDK
  // Example:
  unsigned char octet1;
  unsigned char octet2;
  unsigned char octet3;
  unsigned char octet4;
  
  // hardware_connected_ = OnLinkTo(octet1,octet2,octet3,octet4) == true;
  hardware_connected_ = OnLinkTo(192,168,1,190);
      
  
  // Simulate connection for demonstration
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // hardware_connected_ = true;
  if (hardware_connected_)
  {
    RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "Successfully connected to TJ2 hardware");
    usleep(100000);
    OnClearSet();
    OnClearErr_A();
    OnClearErr_B();
    OnSetSend();
    usleep(100000);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "Failed to connect to TJ2 hardware");
  }
  
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

bool TJ2Hardware::readFromHardware(int robot_arm_left_right_, bool initial_frame)
{
  
  OnGetBuf(&frame_data_);
  // RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "Reading from hardware interface ... %d", frame_data_.m_Out[robot_arm_left_right_].m_OutFrameSerial);
  if (initial_frame)
  {
    previous_message_frame_ = frame_data_.m_Out[robot_arm_left_right_].m_OutFrameSerial;
  }

  if (previous_message_frame_ - frame_data_.m_Out[robot_arm_left_right_].m_OutFrameSerial < 2)
  {
    if (robot_arm_left_right_ == static_cast<int>(RobotArmConfig::LEFT_ARM)|| robot_arm_left_right_ == static_cast<int>(RobotArmConfig::RIGHT_ARM))
    {
      for (size_t i = 0; i < 7; i++) {
        hw_position_states_[i] = degreeToRad(frame_data_.m_Out[robot_arm_left_right_].m_FB_Joint_Pos[i]);
        hw_velocity_states_[i] = frame_data_.m_Out[robot_arm_left_right_].m_FB_Joint_Vel[i];
        hw_effort_states_[i] = frame_data_.m_Out[robot_arm_left_right_].m_FB_Joint_SToq[i];
      }
    }
    else if(robot_arm_left_right_ == static_cast<int>(RobotArmConfig::DUAL_ARM))
    {
      for (size_t i = 0; i < 7; i++) {
        hw_position_states_[i] = degreeToRad(frame_data_.m_Out[static_cast<int>(RobotArmConfig::LEFT_ARM)].m_FB_Joint_Pos[i]);
        hw_velocity_states_[i] = frame_data_.m_Out[static_cast<int>(RobotArmConfig::LEFT_ARM)].m_FB_Joint_Vel[i];
        hw_effort_states_[i] = frame_data_.m_Out[static_cast<int>(RobotArmConfig::LEFT_ARM)].m_FB_Joint_SToq[i];
      }

      for (size_t i = 0; i < 7; i++) {
        hw_position_states_[i + 7] = degreeToRad(frame_data_.m_Out[static_cast<int>(RobotArmConfig::RIGHT_ARM)].m_FB_Joint_Pos[i]);
        hw_velocity_states_[i + 7] = frame_data_.m_Out[static_cast<int>(RobotArmConfig::RIGHT_ARM)].m_FB_Joint_Vel[i];
        hw_effort_states_[i + 7] = frame_data_.m_Out[static_cast<int>(RobotArmConfig::RIGHT_ARM)].m_FB_Joint_SToq[i];
      }
    }
    
    return true;
  }
   else
   {
        RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "Missing more than 2 frames");
        return false;
   }
}

bool TJ2Hardware::writeToHardware(int robot_arm_left_right, std::vector<double> & hw_commands)
{
  // TODO: Implement actual joint command sending to Dobot
  bool result = true;
  OnClearSet(); 
  if (robot_arm_left_right == static_cast<int>(RobotArmConfig::LEFT_ARM))
  {
    result = OnSetJointCmdPos_A(hw_commands.data());
  }
  else if (robot_arm_left_right == static_cast<int>(RobotArmConfig::RIGHT_ARM))
  {
    result = OnSetJointCmdPos_B(hw_commands.data());
  }
  else if (robot_arm_left_right == static_cast<int>(RobotArmConfig::DUAL_ARM))
  {
    result = OnSetJointCmdPos_A(hw_commands.data());
    // RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "write to left arm %d", result);
    result = result && OnSetJointCmdPos_B(hw_commands.data() + 7);
    // RCLCPP_INFO(rclcpp::get_logger("TJ2Hardware"), "write to right arm %d", result);
  }
  OnSetSend();
  return result;
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
  if (hw_position_commands_.size() != position_lower_limits_.size() || 
      hw_position_commands_.size() != position_upper_limits_.size()) {
    RCLCPP_ERROR(*logger_, "Array size mismatch in enforceJointLimits!");
    return;
  }

  for (size_t i = 0; i < hw_position_commands_.size(); i++) {
    if (hw_position_commands_[i] < position_lower_limits_[i]) {
      hw_position_commands_[i] = position_lower_limits_[i];
      RCLCPP_WARN_THROTTLE(
        *logger_, 
        *clock_, 10000,  // Use the class member clock
        "Joint %s position command (%.3f) below lower limit (%.3f)", 
        info_.joints[i].name.c_str(), hw_position_commands_[i], position_lower_limits_[i]);
    } else if (hw_position_commands_[i] > position_upper_limits_[i]) {
      hw_position_commands_[i] = position_upper_limits_[i];
      RCLCPP_WARN_THROTTLE(
        *logger_, 
        *clock_, 10000,
        "Joint %s position command (%.3f) above upper limit (%.3f)", 
        info_.joints[i].name.c_str(), hw_position_commands_[i], position_upper_limits_[i]);
    }
    
    // Add safety check for velocity arrays too
    if (i < hw_velocity_commands_.size() && i < velocity_limits_.size() && 
        std::abs(hw_velocity_commands_[i]) > velocity_limits_[i]) {
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


PLUGINLIB_EXPORT_CLASS(tj2_ros2_control::TJ2Hardware, hardware_interface::SystemInterface)

