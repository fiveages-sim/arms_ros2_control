//
// Modbus Configuration - Common Modbus register addresses and function codes for grippers
//
#pragma once

#include <cstdint>
#include <array>

namespace gripper_hardware_common
{
    namespace ModbusConfig
    {
        namespace detail
        {
            struct SerialDefaults
            {
                static constexpr const char* DEFAULT_SERIAL_PORT = "/dev/ttyUSB0";
                static constexpr int DEFAULT_BAUDRATE = 115200;
                static constexpr char DEFAULT_PARITY = 'N';
                static constexpr int DEFAULT_DATA_BITS = 8;
                static constexpr int DEFAULT_STOP_BITS = 1;
            };

            /** Shared Changingtek AG2F register map (90C/90D/120S variants). */
            struct ChangingtekBase : SerialDefaults
            {
                static constexpr uint16_t POS_REG_ADDR = 0x0102;
                static constexpr uint16_t VELOCITY_REG = 0x0104;
                static constexpr uint16_t TORQUE_REG = 0x0105;
                static constexpr uint16_t ACCELERATION_REG = 0x0106;
                static constexpr uint16_t DECELERATION_REG = 0x0107;
                static constexpr uint16_t TRIGGER_REG_ADDR = 0x0108;
                static constexpr uint16_t INIT_REG_ADDR = 0x0100;

                static constexpr uint16_t MAX_POSITION_MM = 9000;
                static constexpr double JOINT_LOWER_RAD = 0.0;
                static constexpr double JOINT_UPPER_RAD = 1.0;

                static constexpr uint8_t READ_FUNCTION = 0x03;
                static constexpr uint8_t WRITE_FUNCTION = 0x10;
                static constexpr uint8_t WRITE_SINGLE_FUNCTION = 0x06;

                static constexpr uint8_t SLAVE_ID = 0x01;
                static constexpr uint8_t DEFAULT_SLAVE_ID = SLAVE_ID;

                static constexpr uint16_t DEFAULT_ACCELERATION = 2000;
                static constexpr uint16_t DEFAULT_DECELERATION = 2000;
                static constexpr uint16_t TRIGGER_VALUE = 0x0001;
                static constexpr uint16_t POWER_ON = 0x0001;
                static constexpr uint16_t POWER_OFF = 0x0000;

                static constexpr int MAX_TORQUE_VALUE = 255;
                static constexpr int MAX_VELOCITY_VALUE = 255;
            };

            constexpr std::array<uint16_t, 7> linkerHandRegBlock(uint16_t base)
            {
                return {base, static_cast<uint16_t>(base + 1), static_cast<uint16_t>(base + 2),
                        static_cast<uint16_t>(base + 3), static_cast<uint16_t>(base + 4),
                        static_cast<uint16_t>(base + 5), static_cast<uint16_t>(base + 6)};
            }

            /** Backward-compat placeholder limits (real limits live in PositionConverter). */
            struct DexterousHandJointLimits
            {
                static constexpr double THUMB_JOINT1_LOWER = 0.0, THUMB_JOINT1_UPPER = 1.0;
                static constexpr double THUMB_JOINT2_LOWER = 0.0, THUMB_JOINT2_UPPER = 1.0;
                static constexpr double INDEX_JOINT_LOWER = 0.0, INDEX_JOINT_UPPER = 1.0;
                static constexpr double MIDDLE_JOINT_LOWER = 0.0, MIDDLE_JOINT_UPPER = 1.0;
                static constexpr double RING_JOINT_LOWER = 0.0, RING_JOINT_UPPER = 1.0;
                static constexpr double PINKY_JOINT_LOWER = 0.0, PINKY_JOINT_UPPER = 1.0;
            };
        }

        struct Changingtek90C : detail::ChangingtekBase
        {
            static constexpr uint16_t FEEDBACK_REG_ADDR = 0x060D;
        };

        struct Changingtek90D : detail::ChangingtekBase
        {
            static constexpr uint16_t FEEDBACK_REG_ADDR = 0x0418;
        };

        /** AG2F120S — Modbus 0=开、12000=关，反馈 0x060D。 */
        struct Changingtek120S : Changingtek90D
        {
            static constexpr uint16_t MAX_POSITION_MM = 12000;
            static constexpr uint16_t FEEDBACK_REG_ADDR = 0x060D;
            static constexpr int MAX_TORQUE_VALUE = 100;
            static constexpr int MAX_VELOCITY_VALUE = 100;
        };

        /** AG2F120S_D — Modbus 0=开、1000=关；力矩/速度 0–100，反馈 0x0418。 */
        struct Changingtek120S_D : Changingtek120S
        {
            static constexpr uint16_t MAX_POSITION_MM = 1000;
            static constexpr uint16_t FEEDBACK_REG_ADDR = 0x0418;
        };

        namespace Jodell
        {
            static constexpr uint16_t INIT_REG_ADDR = 0x03E8;
            static constexpr uint16_t POSITION_REG_ADDR = INIT_REG_ADDR;
            static constexpr uint16_t STATUS_REG_ADDR = 0x07D0;

            static constexpr int MAX_POSITION = 255;
            static constexpr double JOINT_LOWER_M = 0.0;
            static constexpr double MAX_OPENING_DISTANCE = 0.038372;

            static constexpr uint8_t READ_FUNCTION = 0x04;
            static constexpr uint8_t WRITE_FUNCTION = 0x10;
            static constexpr uint8_t WRITE_SINGLE_FUNCTION = 0x06;

            static constexpr uint8_t SLAVE_ADDRESS = 0x09;
            static constexpr uint16_t TRIGGER_VALUE = 0x09;
            static constexpr int STATUS_REG_COUNT = 3;

            static constexpr uint16_t INIT_VALUE = 0x0000;
        }

        /** Jodell ERG32-150 (rotate + grip). Manual V1.6: FC03 @0x07D0, FC16 @0x03E8+. */
        namespace ERG32
        {
            static constexpr uint8_t SLAVE_ADDRESS = 0x09;
            static constexpr uint8_t SLAVE_ID = SLAVE_ADDRESS;
            static constexpr uint8_t READ_FUNCTION = 0x03;
            static constexpr uint8_t WRITE_FUNCTION = 0x10;

            static constexpr uint16_t GRIP_CTRL_REG = 0x03E8;
            static constexpr uint16_t ROTATE_CTRL_REG = 0x03E9;
            static constexpr uint16_t CTRL_REG_BASE = GRIP_CTRL_REG;
            static constexpr int CTRL_REG_COUNT = 2;
            static constexpr uint16_t GRIP_PARAM_REG = 0x03EA;
            static constexpr uint16_t ROTATE_ABS_POS_REG = 0x03EC;
            static constexpr uint16_t ROTATE_PARAM_REG = 0x03ED;
            static constexpr uint16_t ROTATE_TRIGGER_REG = 0x03EF;

            static constexpr uint16_t STATUS_REG_ADDR = 0x07D0;
            static constexpr int STATUS_REG_COUNT = 8;

            static constexpr uint16_t ACTIVATE_VALUE = 0x0001;
            static constexpr uint16_t DEACTIVATE_VALUE = 0x0000;
            static constexpr uint8_t GRIP_TRIGGER = 0x01;
            static constexpr uint8_t ROTATE_ABS_TRIGGER = 0x01;
        }

        /**
         * LinkerHand O7 layout (position 0–6, torque 7–13, speed 14–20).
         * O6/L6 per-model offsets: modbus_ros2_control DexterousHandProductTraits.
         */
        namespace LinkerHand
        {
            static constexpr uint8_t RIGHT_HAND_SLAVE_ID = 0x27;
            static constexpr uint8_t LEFT_HAND_SLAVE_ID = 0x28;
            static constexpr uint8_t DEFAULT_SLAVE_ID = 0x01;

            static constexpr const char* DEFAULT_SERIAL_PORT = detail::SerialDefaults::DEFAULT_SERIAL_PORT;
            static constexpr int DEFAULT_BAUDRATE = detail::SerialDefaults::DEFAULT_BAUDRATE;
            static constexpr char DEFAULT_PARITY = detail::SerialDefaults::DEFAULT_PARITY;
            static constexpr int DEFAULT_DATA_BITS = detail::SerialDefaults::DEFAULT_DATA_BITS;
            static constexpr int DEFAULT_STOP_BITS = detail::SerialDefaults::DEFAULT_STOP_BITS;

            static constexpr uint8_t READ_FUNCTION = 0x04;
            static constexpr uint8_t WRITE_FUNCTION = 0x10;
            static constexpr uint8_t WRITE_SINGLE_FUNCTION = 0x06;

            static constexpr uint16_t POSITION_REG_BASE = 0x0000;
            static constexpr uint16_t TORQUE_REG_BASE = 0x0007;
            static constexpr uint16_t SPEED_REG_BASE = 0x000E;
            static constexpr uint16_t LOCK_THRESHOLD_REG_BASE = 0x0015;
            static constexpr uint16_t LOCK_TIME_REG_BASE = 0x001C;
            static constexpr uint16_t LOCK_TORQUE_REG_BASE = 0x0023;
            static constexpr uint16_t PRESSURE_SENSING_DATA_SELECTION_REG = 0x002A;

            static constexpr uint16_t THUMB_PITCH_REG = POSITION_REG_BASE;

            static constexpr std::array<uint16_t, 7> POSITION_REGS = detail::linkerHandRegBlock(POSITION_REG_BASE);
            static constexpr std::array<uint16_t, 7> TORQUE_REGS = detail::linkerHandRegBlock(TORQUE_REG_BASE);
            static constexpr std::array<uint16_t, 7> SPEED_REGS = detail::linkerHandRegBlock(SPEED_REG_BASE);
        }

        namespace DexterousHand
        {
            using namespace LinkerHand;

            static constexpr int JOINT_COUNT_O7 = 7;
            static constexpr int JOINT_COUNT_O6 = 6;
            static constexpr int JOINT_COUNT = JOINT_COUNT_O7;

            struct O7 : detail::DexterousHandJointLimits
            {
                static constexpr double THUMB_JOINT3_LOWER = 0.0, THUMB_JOINT3_UPPER = 1.0;
            };
            struct O6 : detail::DexterousHandJointLimits {};
            struct L6 : detail::DexterousHandJointLimits {};
        }
    }
} // namespace gripper_hardware_common
