#include "aubo_ros2_control/aubo_hardware.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <utility>

#include <pluginlib/class_list_macros.hpp>

namespace aubo_ros2_control
{
hardware_interface::CallbackReturn AuboHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.clear();
  for (const auto & joint : info_.joints) {
    joint_names_.push_back(joint.name);
  }

  const auto joint_count = joint_names_.size();
  joint_positions_.assign(joint_count, 0.0);
  joint_velocities_.assign(joint_count, 0.0);
  joint_efforts_.assign(joint_count, 0.0);
  joint_position_commands_.assign(joint_count, 0.0);
  last_sent_commands_.assign(joint_count, 0.0);

  hardware_interface::ComponentInfo ft_sensor_info;
  has_ft_sensor_ = findSensorByName("ft_sensor", ft_sensor_info);

  const auto get_hardware_parameter =
    [this](const std::string & parameter_name, const std::string & default_value) -> std::string
    {
      if (auto it = info_.hardware_parameters.find(parameter_name);
        it != info_.hardware_parameters.end())
      {
        return it->second;
      }
      return default_value;
    };

  robot_ip_ = get_hardware_parameter("robot_ip", "192.168.1.107");
  robot_port_ = std::stoi(get_hardware_parameter("robot_port", "8899"));
  username_ = get_hardware_parameter("username", "aubo");
  password_ = get_hardware_parameter("password", "123456");
  collision_class_ = std::stoi(get_hardware_parameter("collision_class", "6"));
  max_joint_acc_deg_ = std::stod(get_hardware_parameter("max_joint_acc_deg", "50.0"));
  max_joint_vel_deg_ = std::stod(get_hardware_parameter("max_joint_vel_deg", "50.0"));
  force_topic_ = get_hardware_parameter("ft_topic", "/ft_sensor_wrench");
  command_mode_ = get_hardware_parameter("command_mode", "movej");
  move_blocking_ = get_hardware_parameter("move_blocking", "false") == "true";
  shutdown_on_disconnect_ = get_hardware_parameter("shutdown_on_disconnect", "false") == "true";
  command_period_ms_ = std::max(1, std::stoi(get_hardware_parameter("command_period_ms", "100")));
  position_command_threshold_ = std::stod(
    get_hardware_parameter("position_command_threshold", "0.001"));

  if (has_ft_sensor_) {
    wrench_sub_ = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
      force_topic_,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&AuboHardware::wrenchCallback, this, std::placeholders::_1));
  }

  RCLCPP_INFO(get_node()->get_logger(), "Initialized AuboHardware with %zu joints", joint_count);
  if (joint_count > kArmJointCount) {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "  Treating %zu extra joints as virtual passthrough joints",
      joint_count - kArmJointCount);
  }
  RCLCPP_INFO(get_node()->get_logger(), "  AUBO target: %s:%d", robot_ip_.c_str(), robot_port_);
  RCLCPP_INFO(get_node()->get_logger(), "  Command mode: %s", command_mode_.c_str());
  RCLCPP_INFO(
    get_node()->get_logger(), "  Shutdown on disconnect: %s",
    shutdown_on_disconnect_ ? "true" : "false");
  RCLCPP_INFO(get_node()->get_logger(), "  Command period: %d ms", command_period_ms_);
  RCLCPP_INFO(
    get_node()->get_logger(), "  Position command threshold: %.6f rad", position_command_threshold_);
  if (has_ft_sensor_) {
    RCLCPP_INFO(
      get_node()->get_logger(), "  FT sensor topic enabled: %s", force_topic_.c_str());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuboHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Activating AuboHardware...");

  AuboConnectionOptions options;
  options.ip = robot_ip_;
  options.port = robot_port_;
  options.username = username_;
  options.password = password_;
  options.collision_class = static_cast<uint8_t>(collision_class_);
  options.shutdown_on_disconnect = shutdown_on_disconnect_;
  options.max_joint_acc_rad = max_joint_acc_deg_ * M_PI / 180.0;
  options.max_joint_vel_rad = max_joint_vel_deg_ * M_PI / 180.0;

  commander_ = std::make_unique<AuboCommander>(options);

  std::string error_message;
  if (!commander_->connect(&error_message)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to activate AUBO hardware: %s", error_message.c_str());
    commander_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::array<double, 6> joints{};
  if (!commander_->readJointPositions(joints)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to read initial AUBO joint positions");
    commander_->disconnect();
    commander_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }

  const size_t arm_joint_count = std::min({joint_names_.size(), joints.size(), kArmJointCount});
  for (size_t i = 0; i < arm_joint_count; ++i) {
    joint_positions_[i] = joints[i];
    joint_position_commands_[i] = joints[i];
    last_sent_commands_[i] = joints[i];
  }
  for (size_t i = arm_joint_count; i < joint_names_.size(); ++i) {
    joint_positions_[i] = joint_position_commands_[i];
    last_sent_commands_[i] = joint_position_commands_[i];
  }

  has_last_sent_command_ = true;
  last_command_send_time_ =
    std::chrono::steady_clock::now() - std::chrono::milliseconds(command_period_ms_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AuboHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Deactivating AuboHardware...");

  if (commander_) {
    commander_->disconnect();
    commander_.reset();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
AuboHardware::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
    state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));
    state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
      joint_names_[i], hardware_interface::HW_IF_EFFORT, &joint_efforts_[i]));
  }

  if (has_ft_sensor_) {
    hardware_interface::ComponentInfo ft_sensor_info;
    if (findSensorByName("ft_sensor", ft_sensor_info)) {
      std::map<std::string, double *> interface_map = {
        {"force.x", &ft_sensor_force_x_},
        {"force.y", &ft_sensor_force_y_},
        {"force.z", &ft_sensor_force_z_},
        {"torque.x", &ft_sensor_torque_x_},
        {"torque.y", &ft_sensor_torque_y_},
        {"torque.z", &ft_sensor_torque_z_},
      };

      for (const auto & state_interface : ft_sensor_info.state_interfaces) {
        const auto it = interface_map.find(state_interface.name);
        if (it == interface_map.end()) {
          RCLCPP_WARN(
            get_node()->get_logger(),
            "Unknown AUBO ft_sensor interface requested: %s",
            state_interface.name.c_str());
          continue;
        }
        state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
          ft_sensor_info.name, state_interface.name, it->second));
      }
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
AuboHardware::on_export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &joint_position_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type AuboHardware::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  if (!commander_ || !commander_->isConnected()) {
    return hardware_interface::return_type::ERROR;
  }

  std::array<double, 6> joints{};
  if (!commander_->readJointPositions(joints)) {
    return hardware_interface::return_type::ERROR;
  }

  const double dt = period.seconds();
  const size_t arm_joint_count = std::min({joint_names_.size(), joints.size(), kArmJointCount});
  for (size_t i = 0; i < arm_joint_count; ++i) {
    const double previous_position = joint_positions_[i];
    joint_positions_[i] = joints[i];
    joint_velocities_[i] = (dt > 1e-9) ? ((joint_positions_[i] - previous_position) / dt) : 0.0;
    joint_efforts_[i] = 0.0;
  }
  for (size_t i = arm_joint_count; i < joint_names_.size(); ++i) {
    const double previous_position = joint_positions_[i];
    joint_positions_[i] = joint_position_commands_[i];
    joint_velocities_[i] = (dt > 1e-9) ? ((joint_positions_[i] - previous_position) / dt) : 0.0;
    joint_efforts_[i] = 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AuboHardware::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (!commander_ || !commander_->isConnected()) {
    return hardware_interface::return_type::ERROR;
  }

  const auto now = std::chrono::steady_clock::now();
  if (has_last_sent_command_) {
    const auto elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_command_send_time_).count();
    if (elapsed_ms < command_period_ms_) {
      return hardware_interface::return_type::OK;
    }
  }

  std::array<double, 6> joints{};
  const size_t arm_joint_count = std::min({joint_names_.size(), joints.size(), kArmJointCount});
  double max_delta = 0.0;
  for (size_t i = 0; i < arm_joint_count; ++i) {
    joints[i] = joint_position_commands_[i];
    if (has_last_sent_command_) {
      max_delta = std::max(max_delta, std::abs(joints[i] - last_sent_commands_[i]));
    }
  }

  if (has_last_sent_command_ && max_delta < position_command_threshold_) {
    return hardware_interface::return_type::OK;
  }

  const bool use_follow_mode = (command_mode_ == "follow_mode");

  bool ok = false;
  if (use_follow_mode) {
    ok = commander_->sendFollowModeJointMove(joints);
  } else {
    ok = commander_->sendJointMove(joints, move_blocking_);
  }

  if (!ok) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Failed to send AUBO joint command in mode '%s'",
      command_mode_.c_str());
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < arm_joint_count; ++i) {
    last_sent_commands_[i] = joints[i];
  }
  has_last_sent_command_ = true;
  last_command_send_time_ = now;

  RCLCPP_INFO_THROTTLE(
    get_node()->get_logger(),
    *get_node()->get_clock(),
    1000,
    "Sent AUBO %s command, max joint delta %.5f rad",
    use_follow_mode ? "follow_mode" : "movej",
    max_delta);

  return hardware_interface::return_type::OK;
}

bool AuboHardware::findSensorByName(
  const std::string & sensor_name,
  hardware_interface::ComponentInfo & sensor_info) const
{
  for (const auto & sensor : info_.sensors) {
    if (sensor.name.find(sensor_name) != std::string::npos) {
      sensor_info = sensor;
      return true;
    }
  }
  return false;
}

void AuboHardware::wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  std::lock_guard<std::mutex> lock(ft_mutex_);
  ft_sensor_force_x_ = msg->wrench.force.x;
  ft_sensor_force_y_ = msg->wrench.force.y;
  ft_sensor_force_z_ = msg->wrench.force.z;
  ft_sensor_torque_x_ = msg->wrench.torque.x;
  ft_sensor_torque_y_ = msg->wrench.torque.y;
  ft_sensor_torque_z_ = msg->wrench.torque.z;
}
}  // namespace aubo_ros2_control

PLUGINLIB_EXPORT_CLASS(aubo_ros2_control::AuboHardware, hardware_interface::SystemInterface)
