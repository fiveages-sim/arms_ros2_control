#include "can_ros2_control/hands/freedom/freedom_can_hardware.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
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

namespace can_ros2_control
{
namespace
{
constexpr auto kLoggerName = "FreedomCanHardware";

bool has_suffix(const std::string& value, const std::string& suffix)
{
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}
}  // namespace

hardware_interface::CallbackReturn FreedomCanHardware::on_init(
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
      "Freedom CAN hardware expects exactly %zu joints, got %zu",
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
    last_command_angles_[i] = radians_to_protocol_angle(hw_commands_[i], i);
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Configured Freedom CAN hardware: interface=%s, device_id=%u, feedback=%s",
    can_interface_.c_str(),
    device_id_,
    read_feedback_ ? "true" : "false");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FreedomCanHardware::on_activate(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  if (!open_socket())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  command_sent_ = false;
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    hw_commands_[i] = hw_positions_[i];
    last_command_angles_[i] = radians_to_protocol_angle(hw_commands_[i], i);
  }

  if (read_feedback_ && send_query() && receive_feedback(rclcpp::Duration::from_seconds(0.0)))
  {
    for (std::size_t i = 0; i < kJointCount; ++i)
    {
      hw_commands_[i] = hw_positions_[i];
      previous_positions_[i] = hw_positions_[i];
      last_command_angles_[i] = radians_to_protocol_angle(hw_commands_[i], i);
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Freedom CAN hardware activated on %s",
    can_interface_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FreedomCanHardware::on_deactivate(
  const rclcpp_lifecycle::State& /* previous_state */)
{
  close_socket();
  RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "Freedom CAN hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
FreedomCanHardware::on_export_state_interfaces()
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
FreedomCanHardware::on_export_command_interfaces()
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

hardware_interface::return_type FreedomCanHardware::read(
  const rclcpp::Time& /* time */,
  const rclcpp::Duration& period)
{
  if (socket_fd_ < 0)
  {
    return hardware_interface::return_type::ERROR;
  }

  previous_positions_ = hw_positions_;

  if (read_feedback_)
  {
    if (!send_query() || !receive_feedback(period))
    {
      return hardware_interface::return_type::ERROR;
    }
  }
  else
  {
    hw_positions_ = hw_commands_;
  }

  const double dt = period.seconds();
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    hw_efforts_[i] = 0.0;
    hw_velocities_[i] = dt > std::numeric_limits<double>::epsilon()
                          ? (hw_positions_[i] - previous_positions_[i]) / dt
                          : 0.0;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FreedomCanHardware::write(
  const rclcpp::Time& /* time */,
  const rclcpp::Duration& /* period */)
{
  if (socket_fd_ < 0)
  {
    return hardware_interface::return_type::ERROR;
  }

  std::array<uint8_t, kJointCount> angles{};
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    hw_commands_[i] = std::clamp(hw_commands_[i], lower_limits_[i], upper_limits_[i]);
    angles[i] = radians_to_protocol_angle(hw_commands_[i], i);
  }

  if (command_sent_ && !command_changed(angles))
  {
    return hardware_interface::return_type::OK;
  }

  if (!send_command(angles))
  {
    return hardware_interface::return_type::ERROR;
  }

  last_command_angles_ = angles;
  command_sent_ = true;

  if (!read_feedback_)
  {
    hw_positions_ = hw_commands_;
  }

  return hardware_interface::return_type::OK;
}

void FreedomCanHardware::load_parameters()
{
  const auto get_parameter = [this](const std::string& name, const std::string& fallback) {
    const auto it = info_.hardware_parameters.find(name);
    return it == info_.hardware_parameters.end() ? fallback : it->second;
  };

  can_interface_ = get_parameter("can_interface", can_interface_);
  read_feedback_ = parse_bool(get_parameter("read_feedback", read_feedback_ ? "true" : "false"), read_feedback_);
  feedback_timeout_ms_ = std::max(0, parse_int(get_parameter("feedback_timeout_ms", "1"), feedback_timeout_ms_));
  command_deadband_deg_ = std::clamp(parse_int(get_parameter("command_deadband_deg", "0"), 0), 0, 90);

  const auto device_id_param = get_parameter("device_id", get_parameter("slave_id", std::to_string(device_id_)));
  device_id_ = static_cast<uint8_t>(std::clamp(parse_int(device_id_param, device_id_), 0, 31));
}

bool FreedomCanHardware::validate_joint_interfaces() const
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

bool FreedomCanHardware::open_socket()
{
  close_socket();

  socket_fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
  if (socket_fd_ < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "Failed to create CAN socket: %s", std::strerror(errno));
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

  return true;
}

void FreedomCanHardware::close_socket()
{
  if (socket_fd_ >= 0)
  {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

bool FreedomCanHardware::send_query()
{
  struct can_frame frame;
  std::memset(&frame, 0, sizeof(frame));
  frame.can_id = CAN_EFF_FLAG | make_extended_id(device_id_, kAngleQueryCommand, 1, 1);
  frame.can_dlc = 1;
  frame.data[0] = 0x00;

  const auto bytes_written = ::write(socket_fd_, &frame, sizeof(frame));
  return bytes_written == static_cast<ssize_t>(sizeof(frame));
}

bool FreedomCanHardware::receive_feedback(const rclcpp::Duration& period)
{
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(feedback_timeout_ms_);
  bool processed_frame = false;

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
      if (frame_device_id(frame_id) == device_id_ &&
          frame_command(frame_id) == kAngleQueryCommand &&
          frame.can_dlc >= kJointCount)
      {
        for (std::size_t i = 0; i < kJointCount; ++i)
        {
          hw_positions_[i] = protocol_angle_to_radians(frame.data[i], i);
        }
        processed_frame = true;
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
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "Failed to read CAN frame: %s", std::strerror(errno));
      return false;
    }

    break;
  }

  if (!processed_frame && !command_sent_)
  {
    hw_positions_ = hw_commands_;
  }

  if (!processed_frame && period.seconds() > 0.0)
  {
    RCLCPP_DEBUG_THROTTLE(
      rclcpp::get_logger(kLoggerName),
      *get_node()->get_clock(),
      2000,
      "No Freedom CAN angle feedback frame received for device ID %u",
      device_id_);
  }

  return true;
}

bool FreedomCanHardware::send_command(const std::array<uint8_t, kJointCount>& angles)
{
  struct can_frame frame1;
  std::memset(&frame1, 0, sizeof(frame1));
  frame1.can_id = CAN_EFF_FLAG | make_extended_id(device_id_, kMoveCommand, 2, 1);
  frame1.can_dlc = 8;
  frame1.data[0] = 0x01;
  frame1.data[1] = angles[0];
  frame1.data[2] = 0x01;
  frame1.data[3] = angles[1];
  frame1.data[4] = 0x01;
  frame1.data[5] = angles[2];
  frame1.data[6] = 0x01;
  frame1.data[7] = angles[3];

  struct can_frame frame2;
  std::memset(&frame2, 0, sizeof(frame2));
  frame2.can_id = CAN_EFF_FLAG | make_extended_id(device_id_, kMoveCommand, 2, 2);
  frame2.can_dlc = 4;
  frame2.data[0] = 0x01;
  frame2.data[1] = angles[4];
  frame2.data[2] = 0x01;
  frame2.data[3] = angles[5];

  const auto written1 = ::write(socket_fd_, &frame1, sizeof(frame1));
  const auto written2 = ::write(socket_fd_, &frame2, sizeof(frame2));
  if (written1 != static_cast<ssize_t>(sizeof(frame1)) ||
      written2 != static_cast<ssize_t>(sizeof(frame2)))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Failed to write Freedom CAN command to device ID %u: %s",
      device_id_,
      std::strerror(errno));
    return false;
  }

  return true;
}

uint8_t FreedomCanHardware::radians_to_protocol_angle(double radians, std::size_t joint_index) const
{
  const auto lower = lower_limits_[joint_index];
  const auto upper = upper_limits_[joint_index];
  if (upper <= lower)
  {
    return 0;
  }

  const auto normalized = std::clamp((radians - lower) / (upper - lower), 0.0, 1.0);
  const auto angle = std::lround(normalized * static_cast<double>(kMaxProtocolAngle));
  return static_cast<uint8_t>(std::clamp<long>(angle, 0, kMaxProtocolAngle));
}

double FreedomCanHardware::protocol_angle_to_radians(uint8_t angle, std::size_t joint_index) const
{
  const auto lower = lower_limits_[joint_index];
  const auto upper = upper_limits_[joint_index];
  if (upper <= lower)
  {
    return lower;
  }

  const auto normalized = std::clamp(
    static_cast<double>(angle) / static_cast<double>(kMaxProtocolAngle), 0.0, 1.0);
  return lower + normalized * (upper - lower);
}

bool FreedomCanHardware::command_changed(const std::array<uint8_t, kJointCount>& angles) const
{
  for (std::size_t i = 0; i < kJointCount; ++i)
  {
    if (std::abs(static_cast<int>(angles[i]) - static_cast<int>(last_command_angles_[i])) >
        command_deadband_deg_)
    {
      return true;
    }
  }
  return false;
}

uint32_t FreedomCanHardware::make_extended_id(
  uint8_t device_id,
  uint8_t command,
  uint8_t total_frames,
  uint8_t frame_seq)
{
  return (static_cast<uint32_t>(device_id & 0x1F) << 24) |
         (static_cast<uint32_t>(command) << 16) |
         (static_cast<uint32_t>(total_frames) << 8) |
         static_cast<uint32_t>(frame_seq);
}

uint8_t FreedomCanHardware::frame_device_id(uint32_t can_id)
{
  return static_cast<uint8_t>((can_id >> 24) & 0x1F);
}

uint8_t FreedomCanHardware::frame_command(uint32_t can_id)
{
  return static_cast<uint8_t>((can_id >> 16) & 0xFF);
}

bool FreedomCanHardware::parse_bool(const std::string& value, bool default_value)
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

int FreedomCanHardware::parse_int(const std::string& value, int default_value)
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

double FreedomCanHardware::initial_value_for_joint(const hardware_interface::ComponentInfo& joint)
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

std::array<double, FreedomCanHardware::kJointCount> FreedomCanHardware::default_upper_limits(
  const std::vector<std::string>& joint_names)
{
  std::array<double, kJointCount> limits{0.785, 0.29, 1.24, 1.24, 1.24, 1.24};

  for (std::size_t i = 0; i < std::min(kJointCount, joint_names.size()); ++i)
  {
    const auto& name = joint_names[i];
    if (has_suffix(name, "thumb_joint1"))
    {
      limits[i] = 0.785;
    }
    else if (has_suffix(name, "thumb_joint2"))
    {
      limits[i] = 0.29;
    }
    else
    {
      limits[i] = 1.24;
    }
  }

  return limits;
}

}  // namespace can_ros2_control

PLUGINLIB_EXPORT_CLASS(can_ros2_control::FreedomCanHardware, hardware_interface::SystemInterface)
