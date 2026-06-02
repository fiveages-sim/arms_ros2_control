#pragma once

#include <array>
#include <cstdint>

namespace can_ros2_control
{

class InspireCanProtocol
{
public:
  static constexpr uint16_t kIdRegister = 1000;
  static constexpr uint16_t kBaudrateRegister = 1001;
  static constexpr uint16_t kClearErrorRegister = 1003;
  static constexpr uint16_t kAngleSetRegister = 1040;
  static constexpr uint16_t kForceSetRegister = 1046;
  static constexpr uint16_t kSpeedSetRegister = 1052;
  static constexpr uint16_t kAngleActualRegister = 1064;
  static constexpr uint16_t kForceActualRegister = 1070;
  static constexpr uint16_t kModeRegister = 1100;

  static constexpr uint8_t kRegisterBytes = 2;
  static constexpr uint8_t kReadRequestBytes = 1;
  static constexpr uint8_t kWriteAckBytes = 1;
  static constexpr uint8_t kWriteAckOk = 1;

  static uint32_t make_id(bool write, uint8_t hand_id, uint16_t address, uint8_t data_length)
  {
    return (static_cast<uint32_t>(write ? 1 : 0) << 28) |
           (static_cast<uint32_t>(data_length) << 20) |
           (static_cast<uint32_t>(address & 0x0FFF) << 8) |
           static_cast<uint32_t>(hand_id);
  }

  static bool is_write_id(uint32_t can_id)
  {
    return ((can_id >> 28) & 0x01) != 0;
  }

  static uint8_t data_length(uint32_t can_id)
  {
    return static_cast<uint8_t>((can_id >> 20) & 0xFF);
  }

  static uint16_t address(uint32_t can_id)
  {
    return static_cast<uint16_t>((can_id >> 8) & 0x0FFF);
  }

  static uint8_t hand_id(uint32_t can_id)
  {
    return static_cast<uint8_t>(can_id & 0xFF);
  }

  static void encode_u16_le(uint16_t value, uint8_t* data)
  {
    data[0] = static_cast<uint8_t>(value & 0xFF);
    data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  }

  static uint16_t decode_u16_le(const uint8_t* data)
  {
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8);
  }
};

}  // namespace can_ros2_control
