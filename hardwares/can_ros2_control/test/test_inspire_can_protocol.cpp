#include <gtest/gtest.h>

#include "can_ros2_control/hands/inspire/inspire_can_protocol.h"

namespace
{

TEST(InspireCanProtocol, BuildsReadIdentifierFromRegisterAddressLengthAndHandId)
{
  EXPECT_EQ(
    can_ros2_control::InspireCanProtocol::make_id(false, 1, 1066, 1),
    0x00142A01u);
}

TEST(InspireCanProtocol, BuildsWriteIdentifierFromRegisterAddressLengthAndHandId)
{
  EXPECT_EQ(
    can_ros2_control::InspireCanProtocol::make_id(true, 1, 1042, 2),
    0x10241201u);
}

TEST(InspireCanProtocol, EncodesAndDecodesRegisterValuesLittleEndian)
{
  std::array<uint8_t, 2> bytes{};
  can_ros2_control::InspireCanProtocol::encode_u16_le(600, bytes.data());

  EXPECT_EQ(bytes[0], 0x58);
  EXPECT_EQ(bytes[1], 0x02);
  EXPECT_EQ(can_ros2_control::InspireCanProtocol::decode_u16_le(bytes.data()), 600);
}

}  // namespace
