//
// Modbus Configuration - Common Modbus register addresses and function codes for grippers
//
#pragma once

#include <cstdint>
#include <array>

namespace gripper_hardware_common
{
    /**
     * @brief Modbus configuration constants for different gripper types
     * 
     * This namespace provides register addresses, function codes, and other
     * Modbus-related constants for various gripper implementations.
     */
    namespace ModbusConfig
    {
        /**
         * @brief Changingtek 90 gripper Modbus configuration
         */
        struct Changingtek90C
        {
            // Register addresses
            static constexpr uint16_t POS_REG_ADDR = 0x0102;        // Position register address (2 registers, 32-bit)
            static constexpr uint16_t VELOCITY_REG = 0x0104;
            static constexpr uint16_t TORQUE_REG = 0x0105;
            static constexpr uint16_t ACCELERATION_REG = 0x0106;
            static constexpr uint16_t DECELERATION_REG = 0x0107;
            static constexpr uint16_t TRIGGER_REG_ADDR = 0x0108;    // Trigger register address
            static constexpr uint16_t FEEDBACK_REG_ADDR = 0x060D;   // Feedback register address (2 registers, 32-bit)
            static constexpr uint16_t INIT_REG_ADDR = 0x0100;

            // Position range
            static constexpr uint16_t MAX_POSITION_MM = 9000;       // Maximum position value (mm)

            // Function codes
            static constexpr uint8_t READ_FUNCTION = 0x03;          // Read Holding Registers
            static constexpr uint8_t WRITE_FUNCTION = 0x10;         // Write Multiple Registers
            static constexpr uint8_t WRITE_SINGLE_FUNCTION = 0x06;  // Write Single Register

            // Default Modbus parameters
            static constexpr const char* DEFAULT_SERIAL_PORT = "/dev/ttyUSB0";
            static constexpr int DEFAULT_BAUDRATE = 115200;
            static constexpr uint8_t SLAVE_ID = 0x01;               // Slave address 3 for 90C
            static constexpr uint8_t DEFAULT_SLAVE_ID = SLAVE_ID;
            static constexpr char DEFAULT_PARITY = 'N';
            static constexpr int DEFAULT_DATA_BITS = 8;
            static constexpr int DEFAULT_STOP_BITS = 1;
            static constexpr uint16_t DEFAULT_ACCELERATION = 2000;
            static constexpr uint16_t DEFAULT_DECELERATION = 2000;
            static constexpr uint16_t TRIGGER_VALUE = 0x0001;
            static constexpr uint16_t POWER_ON = 0x0001;
            static constexpr uint16_t POWER_OFF = 0x0000;
        };

        /**
         * @brief Changingtek 90D gripper Modbus configuration
         * 
         * Similar to Changingtek90 but uses a different slave address (2 instead of 1).
         * All register addresses and function codes are the same.
         */
        struct Changingtek90D
        {
            // Register addresses
            static constexpr uint16_t POS_REG_ADDR = 0x0102;        // Position register address (2 registers, 32-bit)
            static constexpr uint16_t VELOCITY_REG = 0x0104;
            static constexpr uint16_t TORQUE_REG = 0x0105;
            static constexpr uint16_t ACCELERATION_REG = 0x0106;
            static constexpr uint16_t DECELERATION_REG = 0x0107;
            static constexpr uint16_t TRIGGER_REG_ADDR = 0x0108;    // Trigger register address
            static constexpr uint16_t FEEDBACK_REG_ADDR = 0x0418;   // Feedback register address (2 registers, 32-bit)
            static constexpr uint16_t INIT_REG_ADDR = 0x0100;
            // Position range
            static constexpr uint16_t MAX_POSITION_MM = 9000;       // Maximum position value (mm)

            // Function codes
            static constexpr uint8_t READ_FUNCTION = 0x03;          // Read Holding Registers
            static constexpr uint8_t WRITE_FUNCTION = 0x10;         // Write Multiple Registers
            static constexpr uint8_t WRITE_SINGLE_FUNCTION = 0x06;  // Write Single Register

            // Default Modbus parameters
            static constexpr const char* DEFAULT_SERIAL_PORT = "/dev/ttyUSB0";
            static constexpr int DEFAULT_BAUDRATE = 115200;
            static constexpr uint8_t SLAVE_ID = 0x01;               // Slave address 4 for 90D
            static constexpr uint8_t DEFAULT_SLAVE_ID = SLAVE_ID;
            static constexpr char DEFAULT_PARITY = 'N';
            static constexpr int DEFAULT_DATA_BITS = 8;
            static constexpr int DEFAULT_STOP_BITS = 1;
            static constexpr uint16_t DEFAULT_ACCELERATION = 2000;
            static constexpr uint16_t DEFAULT_DECELERATION = 2000;
            static constexpr uint16_t TRIGGER_VALUE = 0x0001;
            static constexpr uint16_t POWER_ON = 0x0001;
            static constexpr uint16_t POWER_OFF = 0x0000;
        };

        /**
         * @brief Jodell gripper Modbus configuration
         */
        namespace Jodell
        {
            // Register addresses
            static constexpr uint16_t INIT_REG_ADDR = 0x03E8;       // Initialization register address (1000)
            static constexpr uint16_t POSITION_REG_ADDR = 0x03E8;   // Position control register (same as init)
            static constexpr uint16_t STATUS_REG_ADDR = 0x07D0;     // Status register address (2000)

            // Position range
            static constexpr int MAX_POSITION = 255;                // Maximum position value
            static constexpr double MAX_OPENING_DISTANCE = 0.038372;  // Maximum opening distance (meters) for RG75

            // Function codes
            static constexpr uint8_t READ_FUNCTION = 0x04;         // Read Input Registers
            static constexpr uint8_t WRITE_FUNCTION = 0x10;        // Write Multiple Registers
            static constexpr uint8_t WRITE_SINGLE_FUNCTION = 0x06; // Write Single Register

            // Slave address
            static constexpr uint8_t SLAVE_ADDRESS = 0x09;         // Slave address (9)
            static constexpr uint8_t SLAVE_ID = SLAVE_ADDRESS;    // Alias for consistency

            // Control data format
            static constexpr uint16_t TRIGGER_VALUE = 0x09;        // Trigger value for movement command

            // Status register format
            // Register 0: Status bits (bit 3 = stopped flag)
            // Register 1: Position (high byte) and error code (low byte)
            // Register 2: Force (high byte) and velocity (low byte)
            static constexpr int STATUS_REG_COUNT = 3;              // Number of status registers to read
            static constexpr uint16_t READ_REG_NUM = STATUS_REG_COUNT;  // Alias for consistency

            /// initialize gripper
            static constexpr uint16_t INIT_REGISTER = 0x03E8;
            static constexpr uint16_t INIT_VALUE = 0x0000;
            static constexpr uint16_t INIT_REGISTER_NUM = 0x0002;
        }

        /**
         * @brief LinkerHand dexterous hand Modbus configuration namespace
         * 
         * Contains Modbus protocol configurations for different LinkerHand models:
         * - O7 (7-DOF)
         * - O6 (6-DOF)
         * - L6 (6-DOF)
         * 
         * All models use the same Modbus register layout and function codes.
         * Only Modbus protocol-related addresses and function codes are defined here.
         * Joint limits should be defined in URDF/Xacro or separate configuration files.
         */
        namespace LinkerHand
        {
            // Common Modbus configuration for all LinkerHand models
            // Modbus slave addresses
            static constexpr uint8_t RIGHT_HAND_SLAVE_ID = 0x27;  // Right hand Modbus ID
            static constexpr uint8_t LEFT_HAND_SLAVE_ID = 0x28;   // Left hand Modbus ID
            static constexpr uint8_t DEFAULT_SLAVE_ID = 0x01;     // Default slave address (can be configured)

            // Default Modbus parameters
            static constexpr const char* DEFAULT_SERIAL_PORT = "/dev/ttyUSB0";
            static constexpr int DEFAULT_BAUDRATE = 115200;
            static constexpr char DEFAULT_PARITY = 'N';
            static constexpr int DEFAULT_DATA_BITS = 8;
            static constexpr int DEFAULT_STOP_BITS = 1;

            // Function codes
            static constexpr uint8_t READ_FUNCTION = 0x03;   // Read Holding Registers
            static constexpr uint8_t WRITE_FUNCTION = 0x10;  // Write Multiple Registers
            static constexpr uint8_t WRITE_SINGLE_FUNCTION = 0x06;  // Write Single Register

            // Register addresses (common for all LinkerHand models)
            // Position registers (0-6) - 7 joints (O7) or 6 joints (O6/L6)
            static constexpr uint16_t THUMB_PITCH_REG = 0x0000;
            static constexpr uint16_t THUMB_YAW_REG = 0x0001;
            static constexpr uint16_t INDEX_PITCH_REG = 0x0002;
            static constexpr uint16_t MIDDLE_PITCH_REG = 0x0003;
            static constexpr uint16_t RING_PITCH_REG = 0x0004;
            static constexpr uint16_t LITTLE_PITCH_REG = 0x0005;
            static constexpr uint16_t THUMB_ROLL_REG = 0x0006;

            // Torque registers (7-13)
            static constexpr uint16_t THUMB_PITCH_TORQUE_REG = 0x0007;
            static constexpr uint16_t THUMB_YAW_TORQUE_REG = 0x0008;
            static constexpr uint16_t INDEX_PITCH_TORQUE_REG = 0x0009;
            static constexpr uint16_t MIDDLE_PITCH_TORQUE_REG = 0x000A;
            static constexpr uint16_t RING_PITCH_TORQUE_REG = 0x000B;
            static constexpr uint16_t LITTLE_PITCH_TORQUE_REG = 0x000C;
            static constexpr uint16_t THUMB_ROLL_TORQUE_REG = 0x000D;

            // Speed registers (14-20)
            static constexpr uint16_t THUMB_PITCH_SPEED_REG = 0x000E;
            static constexpr uint16_t THUMB_YAW_SPEED_REG = 0x000F;
            static constexpr uint16_t INDEX_PITCH_SPEED_REG = 0x0010;
            static constexpr uint16_t MIDDLE_PITCH_SPEED_REG = 0x0011;
            static constexpr uint16_t RING_PITCH_SPEED_REG = 0x0012;
            static constexpr uint16_t LITTLE_PITCH_SPEED_REG = 0x0013;
            static constexpr uint16_t THUMB_ROLL_SPEED_REG = 0x0014;

            // Lock rotor threshold registers (21-27)
            static constexpr uint16_t THUMB_PITCH_LOCK_THRESHOLD_REG = 0x0015;
            static constexpr uint16_t THUMB_YAW_LOCK_THRESHOLD_REG = 0x0016;
            static constexpr uint16_t INDEX_PITCH_LOCK_THRESHOLD_REG = 0x0017;
            static constexpr uint16_t MIDDLE_PITCH_LOCK_THRESHOLD_REG = 0x0018;
            static constexpr uint16_t RING_PITCH_LOCK_THRESHOLD_REG = 0x0019;
            static constexpr uint16_t LITTLE_PITCH_LOCK_THRESHOLD_REG = 0x001A;
            static constexpr uint16_t THUMB_ROLL_LOCK_THRESHOLD_REG = 0x001B;

            // Lock rotor time registers (28-34)
            static constexpr uint16_t THUMB_PITCH_LOCK_TIME_REG = 0x001C;
            static constexpr uint16_t THUMB_YAW_LOCK_TIME_REG = 0x001D;
            static constexpr uint16_t INDEX_PITCH_LOCK_TIME_REG = 0x001E;
            static constexpr uint16_t MIDDLE_PITCH_LOCK_TIME_REG = 0x001F;
            static constexpr uint16_t RING_PITCH_LOCK_TIME_REG = 0x0020;
            static constexpr uint16_t LITTLE_PITCH_LOCK_TIME_REG = 0x0021;
            static constexpr uint16_t THUMB_ROLL_LOCK_TIME_REG = 0x0022;

            // Lock rotor torque registers (35-41)
            static constexpr uint16_t THUMB_PITCH_LOCK_TORQUE_REG = 0x0023;
            static constexpr uint16_t THUMB_YAW_LOCK_TORQUE_REG = 0x0024;
            static constexpr uint16_t INDEX_PITCH_LOCK_TORQUE_REG = 0x0025;
            static constexpr uint16_t MIDDLE_PITCH_LOCK_TORQUE_REG = 0x0026;
            static constexpr uint16_t RING_PITCH_LOCK_TORQUE_REG = 0x0027;
            static constexpr uint16_t LITTLE_PITCH_LOCK_TORQUE_REG = 0x0028;
            static constexpr uint16_t THUMB_ROLL_LOCK_TORQUE_REG = 0x0029;

            // Pressure sensing data selection (42)
            static constexpr uint16_t PRESSURE_SENSING_DATA_SELECTION_REG = 0x002A;

            // Register arrays for convenience (joint order: Thumb_Pitch, Thumb_Yaw, Index_Pitch, Middle_Pitch, Ring_Pitch, Little_Pitch, Thumb_Roll)
            static constexpr std::array<uint16_t, 7> POSITION_REGS = {
                THUMB_PITCH_REG,
                THUMB_YAW_REG,
                INDEX_PITCH_REG,
                MIDDLE_PITCH_REG,
                RING_PITCH_REG,
                LITTLE_PITCH_REG,
                THUMB_ROLL_REG
            };

            static constexpr std::array<uint16_t, 7> TORQUE_REGS = {
                THUMB_PITCH_TORQUE_REG,
                THUMB_YAW_TORQUE_REG,
                INDEX_PITCH_TORQUE_REG,
                MIDDLE_PITCH_TORQUE_REG,
                RING_PITCH_TORQUE_REG,
                LITTLE_PITCH_TORQUE_REG,
                THUMB_ROLL_TORQUE_REG
            };

            static constexpr std::array<uint16_t, 7> SPEED_REGS = {
                THUMB_PITCH_SPEED_REG,
                THUMB_YAW_SPEED_REG,
                INDEX_PITCH_SPEED_REG,
                MIDDLE_PITCH_SPEED_REG,
                RING_PITCH_SPEED_REG,
                LITTLE_PITCH_SPEED_REG,
                THUMB_ROLL_SPEED_REG
            };
        }

        // Backward compatibility namespace for modbus_ros2_control
        namespace DexterousHand
        {
            using namespace LinkerHand;
            
            static constexpr int JOINT_COUNT_O7 = 7;
            static constexpr int JOINT_COUNT_O6 = 6;
            static constexpr int JOINT_COUNT = JOINT_COUNT_O7;
            
            // Joint limits (for backward compatibility only - should be in URDF/Xacro)
            struct O7 {
                static constexpr double THUMB_JOINT1_LOWER = 0.0, THUMB_JOINT1_UPPER = 1.0;
                static constexpr double THUMB_JOINT2_LOWER = 0.0, THUMB_JOINT2_UPPER = 1.0;
                static constexpr double THUMB_JOINT3_LOWER = 0.0, THUMB_JOINT3_UPPER = 1.0;
                static constexpr double INDEX_JOINT_LOWER = 0.0, INDEX_JOINT_UPPER = 1.0;
                static constexpr double MIDDLE_JOINT_LOWER = 0.0, MIDDLE_JOINT_UPPER = 1.0;
                static constexpr double RING_JOINT_LOWER = 0.0, RING_JOINT_UPPER = 1.0;
                static constexpr double PINKY_JOINT_LOWER = 0.0, PINKY_JOINT_UPPER = 1.0;
            };
            struct O6 {
                static constexpr double THUMB_JOINT1_LOWER = 0.0, THUMB_JOINT1_UPPER = 1.0;
                static constexpr double THUMB_JOINT2_LOWER = 0.0, THUMB_JOINT2_UPPER = 1.0;
                static constexpr double INDEX_JOINT_LOWER = 0.0, INDEX_JOINT_UPPER = 1.0;
                static constexpr double MIDDLE_JOINT_LOWER = 0.0, MIDDLE_JOINT_UPPER = 1.0;
                static constexpr double RING_JOINT_LOWER = 0.0, RING_JOINT_UPPER = 1.0;
                static constexpr double PINKY_JOINT_LOWER = 0.0, PINKY_JOINT_UPPER = 1.0;
            };
            struct L6 {
                static constexpr double THUMB_JOINT1_LOWER = 0.0, THUMB_JOINT1_UPPER = 1.0;
                static constexpr double THUMB_JOINT2_LOWER = 0.0, THUMB_JOINT2_UPPER = 1.0;
                static constexpr double INDEX_JOINT_LOWER = 0.0, INDEX_JOINT_UPPER = 1.0;
                static constexpr double MIDDLE_JOINT_LOWER = 0.0, MIDDLE_JOINT_UPPER = 1.0;
                static constexpr double RING_JOINT_LOWER = 0.0, RING_JOINT_UPPER = 1.0;
                static constexpr double PINKY_JOINT_LOWER = 0.0, PINKY_JOINT_UPPER = 1.0;
            };
        }
    }
} // namespace gripper_hardware_common
