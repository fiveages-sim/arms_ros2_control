//
// Modbus Configuration - Common Modbus register addresses and function codes for grippers
//
#pragma once

#include <cstdint>

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

            // Control data format
            static constexpr uint16_t TRIGGER_VALUE = 0x09;        // Trigger value for movement command

            // Status register format
            // Register 0: Status bits (bit 3 = stopped flag)
            // Register 1: Position (high byte) and error code (low byte)
            // Register 2: Force (high byte) and velocity (low byte)
            static constexpr int STATUS_REG_COUNT = 3;              // Number of status registers to read

            /// initialize gripper
            static constexpr uint16_t INIT_REGISTER = 0x03E8;
            static constexpr uint16_t INIT_VALUE = 0x0000;
            static constexpr uint16_t INIT_REGISTER_NUM = 0x0002;
        }

        /**
         * @brief Dexterous Hand Modbus configuration
         * 
         * Configuration for O7 (7-DOF) and O6 (6-DOF) dexterous hands.
         * Both use the same Modbus slave addresses and register layout.
         */
        struct DexterousHand
        {
            // Register addresses
            static constexpr uint16_t JOINT_POSITION_REG_START = 0;  // Starting register for joint positions

            // Joint counts
            static constexpr int JOINT_COUNT_O7 = 7;  // O7 hand has 7 joints
            static constexpr int JOINT_COUNT_O6 = 6;  // O6 hand has 6 joints
            static constexpr int JOINT_COUNT = JOINT_COUNT_O7;  // Default to O7 for backward compatibility

            // Modbus slave addresses
            static constexpr uint8_t RIGHT_HAND_SLAVE_ID = 0x27;  // Right hand Modbus ID
            static constexpr uint8_t LEFT_HAND_SLAVE_ID = 0x28;   // Left hand Modbus ID

            // Default Modbus parameters
            static constexpr const char* DEFAULT_SERIAL_PORT = "/dev/ttyUSB0";
            static constexpr int DEFAULT_BAUDRATE = 115200;
            static constexpr char DEFAULT_PARITY = 'N';
            static constexpr int DEFAULT_DATA_BITS = 8;
            static constexpr int DEFAULT_STOP_BITS = 1;

            // Function codes
            static constexpr uint8_t READ_FUNCTION = 0x04;   // Read Input Registers
            static constexpr uint8_t WRITE_FUNCTION = 0x10;  // Write Multiple Registers
        };
    }
} // namespace gripper_hardware_common
