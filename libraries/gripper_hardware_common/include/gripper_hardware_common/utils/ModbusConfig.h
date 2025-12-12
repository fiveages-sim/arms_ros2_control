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
        namespace Changingtek90
        {
            // Register addresses
            static constexpr uint16_t POS_REG_ADDR = 0x0102;        // Position register address (2 registers, 32-bit)
            static constexpr uint16_t TRIGGER_REG_ADDR = 0x0108;    // Trigger register address
            static constexpr uint16_t FEEDBACK_REG_ADDR = 0x060D;   // Feedback register address (2 registers, 32-bit)

            // Position range
            static constexpr uint16_t MAX_POSITION_MM = 9000;       // Maximum position value (mm)

            // Function codes
            static constexpr uint8_t READ_FUNCTION = 0x03;          // Read Holding Registers
            static constexpr uint8_t WRITE_FUNCTION = 0x10;         // Write Multiple Registers
            static constexpr uint8_t WRITE_SINGLE_FUNCTION = 0x06;  // Write Single Register

            // Default Modbus parameters
            static constexpr const char* DEFAULT_SERIAL_PORT = "/dev/ttyUSB0";
            static constexpr int DEFAULT_BAUDRATE = 115200;
            static constexpr uint8_t DEFAULT_SLAVE_ID = 1;
            static constexpr char DEFAULT_PARITY = 'N';
            static constexpr int DEFAULT_DATA_BITS = 8;
            static constexpr int DEFAULT_STOP_BITS = 1;
        }

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
        }
    }
} // namespace gripper_hardware_common
