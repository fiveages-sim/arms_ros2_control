#include "aubo_ros2_control/bluedot_force_sensor.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace aubo_ros2_control
{
namespace
{
constexpr size_t kRequestSize = 8;
constexpr size_t kResponseSize = 36;
}  // namespace

BlueDotForceSensor::BlueDotForceSensor(
  const std::string & sensor_ip,
  uint16_t sensor_port,
  uint16_t command,
  uint32_t num_samples,
  int zero_sample_count,
  double scale)
: sensor_ip_(sensor_ip),
  sensor_port_(sensor_port),
  command_(command),
  num_samples_(num_samples),
  zero_sample_count_(zero_sample_count),
  scale_(scale)
{
}

BlueDotForceSensor::~BlueDotForceSensor()
{
  disconnect();
}

bool BlueDotForceSensor::connect(std::string * error_message)
{
  if (connected_) {
    return true;
  }

  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    if (error_message) {
      *error_message = "Failed to create UDP socket: " + std::string(std::strerror(errno));
    }
    return false;
  }

  timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = 200000;
  if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
    if (error_message) {
      *error_message = "Failed to set UDP receive timeout: " + std::string(std::strerror(errno));
    }
    disconnect();
    return false;
  }

  connected_ = true;
  return true;
}

void BlueDotForceSensor::disconnect()
{
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
  connected_ = false;
}

bool BlueDotForceSensor::isConnected() const
{
  return connected_;
}

bool BlueDotForceSensor::read(
  BlueDotForceData & data,
  bool & zero_ready,
  std::string * error_message)
{
  if (!connected_) {
    if (!connect(error_message)) {
      return false;
    }
  }

  if (!sendRequest(error_message)) {
    return false;
  }

  ResponsePacket packet;
  if (!receiveResponse(packet, error_message)) {
    return false;
  }

  std::array<double, 6> current{};
  for (size_t i = 0; i < current.size(); ++i) {
    current[i] = static_cast<double>(packet.ft_data[i]) / scale_;
  }

  if (!zero_ready_) {
    for (size_t i = 0; i < current.size(); ++i) {
      zero_sum_[i] += current[i];
    }
    ++zero_count_;

    if (zero_count_ >= zero_sample_count_) {
      for (size_t i = 0; i < current.size(); ++i) {
        zero_offset_[i] = zero_sum_[i] / static_cast<double>(zero_count_);
      }
      zero_ready_ = true;
    }

    zero_ready = zero_ready_;
    data = {};
    return true;
  }

  data.fx = current[0] - zero_offset_[0];
  data.fy = current[1] - zero_offset_[1];
  data.fz = current[2] - zero_offset_[2];
  data.tx = current[3] - zero_offset_[3];
  data.ty = current[4] - zero_offset_[4];
  data.tz = current[5] - zero_offset_[5];
  zero_ready = true;
  return true;
}

void BlueDotForceSensor::resetZero()
{
  zero_sum_.fill(0.0);
  zero_offset_.fill(0.0);
  zero_count_ = 0;
  zero_ready_ = false;
}

bool BlueDotForceSensor::sendRequest(std::string * error_message)
{
  std::array<uint8_t, kRequestSize> request{};
  const uint16_t header = htons(0x1234);
  const uint16_t command = htons(command_);
  const uint32_t num_samples = htonl(num_samples_);

  std::memcpy(request.data(), &header, sizeof(header));
  std::memcpy(request.data() + 2, &command, sizeof(command));
  std::memcpy(request.data() + 4, &num_samples, sizeof(num_samples));

  sockaddr_in sensor_addr{};
  sensor_addr.sin_family = AF_INET;
  sensor_addr.sin_port = htons(sensor_port_);
  sensor_addr.sin_addr.s_addr = inet_addr(sensor_ip_.c_str());

  const ssize_t sent = sendto(
    socket_fd_,
    request.data(),
    request.size(),
    0,
    reinterpret_cast<const sockaddr *>(&sensor_addr),
    sizeof(sensor_addr));
  if (sent != static_cast<ssize_t>(request.size())) {
    if (error_message) {
      *error_message = "Failed to send UDP request to BlueDot sensor: " +
        std::string(std::strerror(errno));
    }
    return false;
  }

  return true;
}

bool BlueDotForceSensor::receiveResponse(ResponsePacket & packet, std::string * error_message)
{
  std::array<uint8_t, kResponseSize> buffer{};
  sockaddr_in source_addr{};
  socklen_t source_len = sizeof(source_addr);

  const ssize_t received = recvfrom(
    socket_fd_,
    buffer.data(),
    buffer.size(),
    0,
    reinterpret_cast<sockaddr *>(&source_addr),
    &source_len);

  if (received < static_cast<ssize_t>(kResponseSize)) {
    if (error_message) {
      if (received < 0) {
        *error_message = "Failed to receive UDP response from BlueDot sensor: " +
          std::string(std::strerror(errno));
      } else {
        *error_message = "Received truncated UDP response from BlueDot sensor";
      }
    }
    return false;
  }

  packet.rdt_sequence = parseUint32(buffer.data());
  packet.ft_sequence = parseUint32(buffer.data() + 4);
  packet.status = parseUint32(buffer.data() + 8);
  for (size_t i = 0; i < packet.ft_data.size(); ++i) {
    packet.ft_data[i] = parseInt32(buffer.data() + 12 + i * 4);
  }

  return true;
}

uint32_t BlueDotForceSensor::parseUint32(const uint8_t * data)
{
  uint32_t value = 0;
  std::memcpy(&value, data, sizeof(value));
  return ntohl(value);
}

int32_t BlueDotForceSensor::parseInt32(const uint8_t * data)
{
  uint32_t value = 0;
  std::memcpy(&value, data, sizeof(value));
  return static_cast<int32_t>(ntohl(value));
}
}  // namespace aubo_ros2_control
