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
         * @brief O7 Dexterous Hand Modbus configuration
         * 
         * O7机械手采用ModbusRTU协议通信
         * ModbusID: 右手0x27(十进制39), 左手0x28(十进制40)
         * 波特率: 115200 (固定)
         * 功能码: 04 (读输入寄存器), 16 (写保持寄存器)
         */
        struct DexterousHand
        {
            // 关节位置寄存器（功能码16: 写保持寄存器）
            static constexpr uint16_t JOINT_POSITION_REG_START = 0x0000;  // 关节位置寄存器起始地址
            // 寄存器映射：
            static constexpr uint16_t THUMB_PITCH_REG = 0x0000;          // 大拇指弯曲 (0-255, 小值弯曲，大值伸直)
            static constexpr uint16_t THUMB_YAW_REG = 0x0001;             // 大拇指横摆 (0-255, 小值向掌心靠拢，大值远离掌心)
            static constexpr uint16_t INDEX_PITCH_REG = 0x0002;          // 食指弯曲 (0-255, 小值弯曲，大值伸直)
            static constexpr uint16_t MIDDLE_PITCH_REG = 0x0003;          // 中指弯曲 (0-255, 小值弯曲，大值伸直)
            static constexpr uint16_t RING_PITCH_REG = 0x0004;            // 无名指弯曲 (0-255, 小值弯曲，大值伸直)
            static constexpr uint16_t LITTLE_PITCH_REG = 0x0005;          // 小拇指弯曲 (0-255, 小值弯曲，大值伸直)
            static constexpr uint16_t THUMB_ROLL_REG = 0x0006;            // 大拇指横滚 (0-255, 小值向掌心靠拢，大值远离掌心)
            static constexpr int JOINT_COUNT = 7;                          // 关节数量

            // 关节转矩寄存器（可选，用于未来扩展）
            static constexpr uint16_t JOINT_TORQUE_REG_START = 0x0007;     // 关节转矩寄存器起始地址
            static constexpr uint16_t THUMB_PITCH_TORQUE_REG = 0x0007;    // 大拇指弯曲转矩
            static constexpr uint16_t THUMB_YAW_TORQUE_REG = 0x0008;      // 大拇指横摆转矩
            static constexpr uint16_t INDEX_PITCH_TORQUE_REG = 0x0009;     // 食指弯曲转矩
            static constexpr uint16_t MIDDLE_PITCH_TORQUE_REG = 0x000A;   // 中指弯曲转矩
            static constexpr uint16_t RING_PITCH_TORQUE_REG = 0x000B;     // 无名指弯曲转矩
            static constexpr uint16_t LITTLE_PITCH_TORQUE_REG = 0x000C;   // 小拇指弯曲转矩
            static constexpr uint16_t THUMB_ROLL_TORQUE_REG = 0x000D;      // 大拇指横滚转矩

            // 关节速度寄存器（可选，用于未来扩展）
            static constexpr uint16_t JOINT_SPEED_REG_START = 0x000E;     // 关节速度寄存器起始地址
            static constexpr uint16_t THUMB_PITCH_SPEED_REG = 0x000E;      // 大拇指弯曲速度
            static constexpr uint16_t THUMB_YAW_SPEED_REG = 0x000F;       // 大拇指横摆速度
            static constexpr uint16_t INDEX_PITCH_SPEED_REG = 0x0010;      // 食指弯曲速度
            static constexpr uint16_t MIDDLE_PITCH_SPEED_REG = 0x0011;     // 中指弯曲速度
            static constexpr uint16_t RING_PITCH_SPEED_REG = 0x0012;     // 无名指弯曲速度
            static constexpr uint16_t LITTLE_PITCH_SPEED_REG = 0x0013;    // 小拇指弯曲速度
            static constexpr uint16_t THUMB_ROLL_SPEED_REG = 0x0014;      // 大拇指横滚速度

            // 堵转保护寄存器（可选，用于未来扩展）
            static constexpr uint16_t LOCK_ROTOR_THRESHOLD_REG_START = 0x0015;  // 堵转阈值寄存器起始地址
            static constexpr uint16_t LOCK_ROTOR_TIME_REG_START = 0x001C;        // 堵转时间寄存器起始地址
            static constexpr uint16_t LOCK_ROTOR_TORQUE_REG_START = 0x0023;      // 堵转扭矩寄存器起始地址

            // 压力传感器数据选择寄存器
            static constexpr uint16_t PRESSURE_SENSING_DATA_SELECTION_REG = 0x002A;  // 压力传感器数据选择 (0-5)

            // 位置范围
            static constexpr uint8_t MIN_POSITION = 0;                    // 最小位置值
            static constexpr uint8_t MAX_POSITION = 255;                  // 最大位置值
            static constexpr uint8_t MID_POSITION = 128;                  // 中间位置值

            // 功能码
            static constexpr uint8_t READ_FUNCTION = 0x04;                // 读输入寄存器
            static constexpr uint8_t WRITE_FUNCTION = 0x10;               // 写多个保持寄存器 (16)
            static constexpr uint8_t WRITE_SINGLE_FUNCTION = 0x06;       // 写单个保持寄存器

            // Modbus 从站地址
            static constexpr uint8_t RIGHT_HAND_SLAVE_ID = 0x27;          // 右手 Modbus ID (十进制39)
            static constexpr uint8_t LEFT_HAND_SLAVE_ID = 0x28;           // 左手 Modbus ID (十进制40)
            static constexpr uint8_t DEFAULT_SLAVE_ID = RIGHT_HAND_SLAVE_ID;

            // 默认 Modbus 参数（固定，不支持修改）
            static constexpr const char* DEFAULT_SERIAL_PORT = "/dev/ttyUSB0";
            static constexpr int DEFAULT_BAUDRATE = 115200;               // 固定波特率，不支持修改
            static constexpr char DEFAULT_PARITY = 'N';                    // 无校验（固定）
            static constexpr int DEFAULT_DATA_BITS = 8;                    // 数据位（固定）
            static constexpr int DEFAULT_STOP_BITS = 1;                   // 停止位（固定）

            // 协议类型
            static constexpr const char* PROTOCOL_TYPE = "RTU";           // Modbus RTU 协议
        };
    }
} // namespace gripper_hardware_common
