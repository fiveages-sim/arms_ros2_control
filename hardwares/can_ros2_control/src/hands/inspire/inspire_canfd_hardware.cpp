#include "can_ros2_control/hands/inspire/inspire_canfd_hardware.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <thread>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <pluginlib/class_list_macros.hpp>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "can_ros2_control/hands/inspire/inspire_can_protocol.h"

namespace can_ros2_control
{
namespace
{
constexpr auto kLoggerName = "InspireCanfdHardware";

bool has_suffix(const std::string& value, const std::string& suffix)
{
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

int16_t to_signed(uint16_t value)
{
  return static_cast<int16_t>(value);
}

bool write_can_frame_with_retry(
  int socket_fd,
  const struct can_frame& frame,
  int retry_count,
  int inter_frame_delay_us,
  const char* operation,
  uint16_t address)
{
  for (int attempt = 0; attempt <= retry_count; ++attempt)
  {
    const auto bytes_written = ::write(socket_fd, &frame, sizeof(frame));
    if (bytes_written == static_cast<ssize_t>(sizeof(frame)))
    {
      if (inter_frame_delay_us > 0)
      {
        std::this_thread::sleep_for(std::chrono::microseconds(inter_frame_delay_us));
      }
      return true;
    }

    if (bytes_written < 0 &&
        (errno == ENOBUFS || errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to write Inspire CAN2.0 %s 0x%X: %s",
      operation,
      address,
      std::strerror(errno));
    return false;
  }

  RCLCPP_ERROR(
    rclcpp::get_logger(kLoggerName),
    "Failed to write Inspire CAN2.0 %s 0x%X after %d retries: %s",
    operation,
    address,
    retry_count,
    std::strerror(errno));
  return false;
}
}  // namespace

hardware_interface::CallbackReturn InspireCanfdHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams& params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kJointCount)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Inspire CAN2.0 hardware expects exactly %zu joints, got %zu",
      kJointCount,
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.reserve(kJointCount);
  for (const auto& joint : info_.joints)
  {
    joint_names_.push_back(joint.name);
  }

  if (!validate_joint_interfaces())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  load_parameters();
  upper_limits_ = default_upper_limits(joint_names_);

  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    hw_positions_[i] = std::clamp(
      initial_value_for_joint(info_.joints[i]), lower_limits_[i], upper_limits_[i]);
    previous_positions_[i] = hw_positions_[i];
    hw_commands_[i] = hw_positions_[i];
    hw_velocities_[i] = 0.0;
    hw_efforts_[i] = 0.0;
  }

  last_command_values_ = joints_to_actuators(hw_commands_);

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Configured Inspire CAN2.0 hardware: interface=%s, hand_id=%u, feedback=%s",
    can_interface_.c_str(),
    hand_id_,
    read_feedback_enabled_ ? "true" : "false");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn InspireCanfdHardware::on_activate(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  if (!open_socket())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!initialize_hand())
  {
    close_socket();
    return hardware_interface::CallbackReturn::ERROR;
  }

  command_sent_ = false;

  std::array<double, kJointCount> initial_positions{};
  std::array<double, kJointCount> initial_efforts{};
  if (read_feedback_enabled_ && read_feedback(initial_positions, initial_efforts))
  {
    for (std::size_t i = 0; i < kJointCount; ++i)
    {
      hw_positions_[i] = initial_positions[i];
      previous_positions_[i] = hw_positions_[i];
      hw_commands_[i] = hw_positions_[i];
      hw_efforts_[i] = initial_efforts[i];
    }
  }

  last_command_values_ = joints_to_actuators(hw_commands_);

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Inspire CAN2.0 hardware activated on %s",
    can_interface_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn InspireCanfdHardware::on_deactivate(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  close_socket();
  RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "Inspire CAN2.0 hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
InspireCanfdHardware::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
  state_interfaces.reserve(kJointCount * 3);

  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    state_interfaces.push_back(
      std::make_shared<hardware_interface::StateInterface>(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.push_back(
      std::make_shared<hardware_interface::StateInterface>(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.push_back(
      std::make_shared<hardware_interface::StateInterface>(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
InspireCanfdHardware::on_export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
  command_interfaces.reserve(kJointCount);

  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    command_interfaces.push_back(
      std::make_shared<hardware_interface::CommandInterface>(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type InspireCanfdHardware::read(
  const rclcpp::Time& /* time */,
  const rclcpp::Duration& period)
{
  if (socket_fd_ < 0)
  {
    return hardware_interface::return_type::ERROR;
  }

  previous_positions_ = hw_positions_;

  if (read_feedback_enabled_)
  {
    std::array<double, kJointCount> positions{};
    std::array<double, kJointCount> efforts{};
    if (read_feedback(positions, efforts))
    {
      hw_positions_ = positions;
      hw_efforts_ = efforts;
    }
    else
    {
      RCLCPP_DEBUG_THROTTLE(
        rclcpp::get_logger(kLoggerName),
        *get_node()->get_clock(),
        2000,
        "No valid Inspire CAN2.0 feedback received from hand ID %u",
        hand_id_);
    }
  }
  else
  {
    hw_positions_ = hw_commands_;
    hw_efforts_.fill(0.0);
  }

  const double dt = period.seconds();
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    hw_velocities_[i] = dt > std::numeric_limits<double>::epsilon()
                          ? (hw_positions_[i] - previous_positions_[i]) / dt
                          : 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type InspireCanfdHardware::write(
  const rclcpp::Time& /* time */,
  const rclcpp::Duration& /* period */)
{
  if (socket_fd_ < 0)
  {
    return hardware_interface::return_type::ERROR;
  }

  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    hw_commands_[i] = std::clamp(hw_commands_[i], lower_limits_[i], upper_limits_[i]);
  }

  const auto actuator_values = joints_to_actuators(hw_commands_);
  if (command_sent_ && !command_changed(actuator_values))
  {
    return hardware_interface::return_type::OK;
  }

  for (std::size_t actuator_index = 0; actuator_index < kActuatorCount; ++actuator_index)
  {
    const auto address = static_cast<uint16_t>(
      InspireCanProtocol::kAngleSetRegister + actuator_index);
    if (!write_register(address, actuator_values[actuator_index]))
    {
      return hardware_interface::return_type::ERROR;
    }
  }

  last_command_values_ = actuator_values;
  command_sent_ = true;

  if (!read_feedback_enabled_)
  {
    hw_positions_ = hw_commands_;
  }

  return hardware_interface::return_type::OK;
}

void InspireCanfdHardware::load_parameters()
{
  const auto get_parameter = [this](const std::string& name, const std::string& fallback) {
    const auto it = info_.hardware_parameters.find(name);
    return it == info_.hardware_parameters.end() ? fallback : it->second;
  };

  can_interface_ = get_parameter("can_interface", can_interface_);
  hand_side_ = get_parameter("hand_side", hand_side_);
  read_feedback_enabled_ = parse_bool(
    get_parameter("read_feedback", read_feedback_enabled_ ? "true" : "false"),
    read_feedback_enabled_);
  wait_write_ack_ = parse_bool(
    get_parameter("wait_write_ack", wait_write_ack_ ? "true" : "false"),
    wait_write_ack_);
  feedback_timeout_ms_ = std::max(
    0,
    parse_int(get_parameter("feedback_timeout_ms", std::to_string(feedback_timeout_ms_)),
              feedback_timeout_ms_));
  frame_tx_retries_ = std::max(
    0,
    parse_int(get_parameter("frame_tx_retries", std::to_string(frame_tx_retries_)),
              frame_tx_retries_));
  inter_frame_delay_us_ = std::max(
    0,
    parse_int(get_parameter("inter_frame_delay_us", std::to_string(inter_frame_delay_us_)),
              inter_frame_delay_us_));
  command_deadband_raw_ = std::clamp(
    parse_int(get_parameter("command_deadband_raw", std::to_string(command_deadband_raw_)),
              command_deadband_raw_),
    0,
    2000);
  default_speed_ = static_cast<uint16_t>(std::clamp(
    parse_int(get_parameter("default_speed", std::to_string(default_speed_)), default_speed_),
    0,
    4000));
  default_force_ = static_cast<uint16_t>(std::clamp(
    parse_int(get_parameter("default_force", std::to_string(default_force_)), default_force_),
    0,
    12000));

  auto side_lower = hand_side_;
  std::transform(side_lower.begin(), side_lower.end(), side_lower.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  const auto side_default_id = side_lower == "right" || side_lower == "right_hand" ? 1 : 2;
  const auto id_value = get_parameter(
    "hand_id",
    get_parameter("slave_id", get_parameter("device_id", "auto")));
  if (id_value == "auto" || id_value == "AUTO" || id_value.empty())
  {
    hand_id_ = static_cast<uint8_t>(side_default_id);
  }
  else
  {
    hand_id_ = static_cast<uint8_t>(
      std::clamp(parse_int(id_value, side_default_id), 1, 254));
  }
}

bool InspireCanfdHardware::validate_joint_interfaces() const
{
  for (const auto& joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Joint '%s' must expose exactly one position command interface",
        joint.name.c_str());
      return false;
    }

    const auto has_state_interface = [&joint](const std::string& interface_name) {
      return std::any_of(
        joint.state_interfaces.begin(),
        joint.state_interfaces.end(),
        [&interface_name](const auto& interface) { return interface.name == interface_name; });
    };

    if (!has_state_interface(hardware_interface::HW_IF_POSITION) ||
        !has_state_interface(hardware_interface::HW_IF_VELOCITY) ||
        !has_state_interface(hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Joint '%s' must expose position, velocity, and effort state interfaces",
        joint.name.c_str());
      return false;
    }
  }

  return true;
}

bool InspireCanfdHardware::open_socket()
{
  close_socket();

  socket_fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  if (socket_fd_ < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to create CAN socket: %s",
      std::strerror(errno));
    return false;
  }

  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);

  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to find CAN interface '%s': %s",
      can_interface_.c_str(),
      std::strerror(errno));
    close_socket();
    return false;
  }

  struct sockaddr_can address;
  std::memset(&address, 0, sizeof(address));
  address.can_family = AF_CAN;
  address.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&address), sizeof(address)) < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to bind CAN interface '%s': %s",
      can_interface_.c_str(),
      std::strerror(errno));
    close_socket();
    return false;
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Opened SocketCAN interface %s for Inspire hand ID %u",
    can_interface_.c_str(),
    hand_id_);

  return true;
}

void InspireCanfdHardware::close_socket()
{
  if (socket_fd_ >= 0)
  {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool InspireCanfdHardware::initialize_hand()
{
  for (std::size_t i = 0; i < kActuatorCount; ++i)
  {
    const auto mode_address = static_cast<uint16_t>(InspireCanProtocol::kModeRegister + i);
    const auto speed_address = static_cast<uint16_t>(InspireCanProtocol::kSpeedSetRegister + i);
    const auto force_address = static_cast<uint16_t>(InspireCanProtocol::kForceSetRegister + i);

    if (!write_register(mode_address, 0))
    {
      RCLCPP_WARN(
        rclcpp::get_logger(kLoggerName),
        "Failed to set Inspire CAN2.0 mode register %u; continuing",
        mode_address);
    }
    if (!write_register(speed_address, default_speed_))
    {
      RCLCPP_WARN(
        rclcpp::get_logger(kLoggerName),
        "Failed to set Inspire CAN2.0 speed register %u; continuing",
        speed_address);
    }
    if (!write_register(force_address, default_force_))
    {
      RCLCPP_WARN(
        rclcpp::get_logger(kLoggerName),
        "Failed to set Inspire CAN2.0 force register %u; continuing",
        force_address);
    }
  }

  return true;
}

bool InspireCanfdHardware::read_feedback(
  std::array<double, kJointCount>& positions,
  std::array<double, kJointCount>& efforts)
{
  std::array<uint16_t, kActuatorCount> angle_registers{};
  for (std::size_t actuator_index = 0; actuator_index < kActuatorCount; ++actuator_index)
  {
    const auto address = static_cast<uint16_t>(
      InspireCanProtocol::kAngleActualRegister + actuator_index);
    if (!read_register(address, angle_registers[actuator_index]))
    {
      return false;
    }
  }

  positions = actuators_to_joints(angle_registers);

  std::array<uint16_t, kActuatorCount> force_registers{};
  bool force_feedback_valid = true;
  for (std::size_t actuator_index = 0; actuator_index < kActuatorCount; ++actuator_index)
  {
    const auto address = static_cast<uint16_t>(
      InspireCanProtocol::kForceActualRegister + actuator_index);
    if (!read_register(address, force_registers[actuator_index]))
    {
      force_feedback_valid = false;
      break;
    }
  }

  if (force_feedback_valid)
  {
    for (std::size_t joint_index = 0; joint_index < kJointCount; ++joint_index)
    {
      const auto actuator_index = kJointCount - 1 - joint_index;
      efforts[joint_index] = static_cast<double>(
        to_signed(force_registers[actuator_index]));
    }
  }
  else
  {
    efforts.fill(0.0);
  }

  return true;
}

bool InspireCanfdHardware::write_register(uint16_t address, uint16_t value)
{
  if (!send_write_request(address, value))
  {
    return false;
  }

  if (!wait_write_ack_)
  {
    return true;
  }

  std::array<uint8_t, 8> data{};
  if (!receive_register_response(
      true, address, InspireCanProtocol::kWriteAckBytes, data))
  {
    return false;
  }

  return data[0] == InspireCanProtocol::kWriteAckOk;
}

bool InspireCanfdHardware::read_register(uint16_t address, uint16_t& value)
{
  if (!send_read_request(address, InspireCanProtocol::kRegisterBytes))
  {
    return false;
  }

  std::array<uint8_t, 8> data{};
  if (!receive_register_response(
      false, address, InspireCanProtocol::kRegisterBytes, data))
  {
    return false;
  }

  value = InspireCanProtocol::decode_u16_le(data.data());
  return true;
}

bool InspireCanfdHardware::send_read_request(uint16_t address, uint8_t byte_count)
{
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = CAN_EFF_FLAG | InspireCanProtocol::make_id(
    false, hand_id_, address, InspireCanProtocol::kReadRequestBytes);
  frame.can_dlc = InspireCanProtocol::kReadRequestBytes;
  frame.data[0] = byte_count;

  return write_can_frame_with_retry(
    socket_fd_,
    frame,
    frame_tx_retries_,
    inter_frame_delay_us_,
    "read request",
    address);
}

bool InspireCanfdHardware::send_write_request(uint16_t address, uint16_t value)
{
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = CAN_EFF_FLAG | InspireCanProtocol::make_id(
    true, hand_id_, address, InspireCanProtocol::kRegisterBytes);
  frame.can_dlc = InspireCanProtocol::kRegisterBytes;
  InspireCanProtocol::encode_u16_le(value, frame.data);

  return write_can_frame_with_retry(
    socket_fd_,
    frame,
    frame_tx_retries_,
    inter_frame_delay_us_,
    "register",
    address);
}

bool InspireCanfdHardware::receive_register_response(
  bool write_response,
  uint16_t address,
  uint8_t expected_length,
  std::array<uint8_t, 8>& data)
{
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(feedback_timeout_ms_);

  while (true)
  {
    struct can_frame frame;
    const auto bytes_read = ::read(socket_fd_, &frame, sizeof(frame));

    if (bytes_read == static_cast<ssize_t>(sizeof(frame)))
    {
      if ((frame.can_id & CAN_EFF_FLAG) == 0)
      {
        continue;
      }

      const auto frame_id = frame.can_id & CAN_EFF_MASK;
      if (InspireCanProtocol::hand_id(frame_id) == hand_id_ &&
          InspireCanProtocol::address(frame_id) == address &&
          InspireCanProtocol::is_write_id(frame_id) == write_response &&
          InspireCanProtocol::data_length(frame_id) == expected_length &&
          frame.can_dlc >= expected_length)
      {
        std::copy(frame.data, frame.data + expected_length, data.begin());
        return true;
      }
      continue;
    }

    if (bytes_read < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
    {
      if (feedback_timeout_ms_ == 0 || std::chrono::steady_clock::now() >= deadline)
      {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    if (bytes_read < 0 && errno == EINTR)
    {
      continue;
    }

    if (bytes_read < 0)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Failed to read Inspire CAN2.0 frame: %s",
        std::strerror(errno));
      return false;
    }

    break;
  }

  return false;
}

std::array<uint16_t, InspireCanfdHardware::kActuatorCount>
InspireCanfdHardware::joints_to_actuators(
  const std::array<double, kJointCount>& joints) const
{
  return {
    joint_to_actuator_value(joints[5], 5),
    joint_to_actuator_value(joints[4], 4),
    joint_to_actuator_value(joints[3], 3),
    joint_to_actuator_value(joints[2], 2),
    joint_to_actuator_value(joints[1], 1),
    joint_to_actuator_value(joints[0], 0)};
}

std::array<double, InspireCanfdHardware::kJointCount>
InspireCanfdHardware::actuators_to_joints(
  const std::array<uint16_t, kActuatorCount>& actuators) const
{
  return {
    actuator_value_to_joint(actuators[5], 0),
    actuator_value_to_joint(actuators[4], 1),
    actuator_value_to_joint(actuators[3], 2),
    actuator_value_to_joint(actuators[2], 3),
    actuator_value_to_joint(actuators[1], 4),
    actuator_value_to_joint(actuators[0], 5)};
}

uint16_t InspireCanfdHardware::joint_to_actuator_value(
  double radians,
  std::size_t joint_index) const
{
  constexpr std::array<double, kJointCount> raw_open{
    1750.0, 1450.0, 1740.0, 1740.0, 1740.0, 1740.0};
  constexpr std::array<double, kJointCount> raw_closed{
    500.0, 1100.0, 900.0, 900.0, 900.0, 900.0};

  const auto lower = lower_limits_[joint_index];
  const auto upper = upper_limits_[joint_index];
  if (upper <= lower)
  {
    return static_cast<uint16_t>(std::lround(raw_open[joint_index]));
  }

  const auto ratio = std::clamp((radians - lower) / (upper - lower), 0.0, 1.0);
  const auto value = raw_open[joint_index] +
                     ratio * (raw_closed[joint_index] - raw_open[joint_index]);
  return static_cast<uint16_t>(std::clamp<int>(
    static_cast<int>(std::lround(value)), 0, 2000));
}

double InspireCanfdHardware::actuator_value_to_joint(
  uint16_t value,
  std::size_t joint_index) const
{
  constexpr std::array<double, kJointCount> raw_open{
    1750.0, 1450.0, 1740.0, 1740.0, 1740.0, 1740.0};
  constexpr std::array<double, kJointCount> raw_closed{
    500.0, 1100.0, 900.0, 900.0, 900.0, 900.0};

  const auto lower = lower_limits_[joint_index];
  const auto upper = upper_limits_[joint_index];
  if (upper <= lower || std::abs(raw_closed[joint_index] - raw_open[joint_index]) <
                          std::numeric_limits<double>::epsilon())
  {
    return lower;
  }

  const auto ratio = std::clamp(
    (static_cast<double>(value) - raw_open[joint_index]) /
      (raw_closed[joint_index] - raw_open[joint_index]),
    0.0,
    1.0);
  return lower + ratio * (upper - lower);
}

bool InspireCanfdHardware::command_changed(
  const std::array<uint16_t, kActuatorCount>& actuator_values) const
{
  for (std::size_t i = 0; i < kActuatorCount; ++i)
  {
    if (std::abs(static_cast<int>(actuator_values[i]) -
                 static_cast<int>(last_command_values_[i])) > command_deadband_raw_)
    {
      return true;
    }
  }
  return false;
}

bool InspireCanfdHardware::parse_bool(const std::string& value, bool default_value)
{
  if (value == "true" || value == "1" || value == "True" || value == "TRUE")
  {
    return true;
  }
  if (value == "false" || value == "0" || value == "False" || value == "FALSE")
  {
    return false;
  }
  return default_value;
}

int InspireCanfdHardware::parse_int(const std::string& value, int default_value)
{
  try
  {
    return std::stoi(value, nullptr, 0);
  }
  catch (const std::exception&)
  {
    return default_value;
  }
}

double InspireCanfdHardware::initial_value_for_joint(
  const hardware_interface::ComponentInfo& joint)
{
  for (const auto& interface : joint.state_interfaces)
  {
    if (interface.name == hardware_interface::HW_IF_POSITION && !interface.initial_value.empty())
    {
      try
      {
        return std::stod(interface.initial_value);
      }
      catch (const std::exception&)
      {
        return 0.0;
      }
    }
  }
  return 0.0;
}

std::array<double, InspireCanfdHardware::kJointCount>
InspireCanfdHardware::default_upper_limits(const std::vector<std::string>& joint_names)
{
  std::array<double, kJointCount> limits{1.1641, 0.5864, 1.4381, 1.4381, 1.4381, 1.4381};

  for (std::size_t i = 0; i < std::min(kJointCount, joint_names.size()); ++i)
  {
    const auto& name = joint_names[i];
    if (has_suffix(name, "thumb_joint1"))
    {
      limits[i] = 1.1641;
    }
    else if (has_suffix(name, "thumb_joint2"))
    {
      limits[i] = 0.5864;
    }
    else
    {
      limits[i] = 1.4381;
    }
  }

  return limits;
}

}  // namespace can_ros2_control

PLUGINLIB_EXPORT_CLASS(
  can_ros2_control::InspireCanfdHardware,
  hardware_interface::SystemInterface)
