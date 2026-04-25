#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace aubo_ros2_control
{
struct BlueDotForceData
{
  double fx{0.0};
  double fy{0.0};
  double fz{0.0};
  double tx{0.0};
  double ty{0.0};
  double tz{0.0};
};

class BlueDotForceSensor
{
public:
  BlueDotForceSensor(
    const std::string & sensor_ip = "192.168.0.20",
    uint16_t sensor_port = 49152,
    uint16_t command = 2,
    uint32_t num_samples = 1,
    int zero_sample_count = 10,
    double scale = 1000000.0);

  ~BlueDotForceSensor();

  bool connect(std::string * error_message = nullptr);
  void disconnect();
  bool isConnected() const;

  bool read(BlueDotForceData & data, bool & zero_ready, std::string * error_message = nullptr);
  void resetZero();

private:
  struct ResponsePacket
  {
    uint32_t rdt_sequence{0};
    uint32_t ft_sequence{0};
    uint32_t status{0};
    std::array<int32_t, 6> ft_data{};
  };

  bool sendRequest(std::string * error_message);
  bool receiveResponse(ResponsePacket & packet, std::string * error_message);
  static uint32_t parseUint32(const uint8_t * data);
  static int32_t parseInt32(const uint8_t * data);

  std::string sensor_ip_;
  uint16_t sensor_port_;
  uint16_t command_;
  uint32_t num_samples_;
  int zero_sample_count_;
  double scale_;

  int socket_fd_{-1};
  bool connected_{false};

  std::array<double, 6> zero_sum_{};
  std::array<double, 6> zero_offset_{};
  int zero_count_{0};
  bool zero_ready_{false};
};
}  // namespace aubo_ros2_control
