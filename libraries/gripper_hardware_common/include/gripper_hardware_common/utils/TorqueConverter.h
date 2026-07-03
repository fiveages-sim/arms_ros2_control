//
// Torque Converter - Common utility functions for gripper torque conversion
//
#pragma once

#include <algorithm>
#include <cmath>

namespace gripper_hardware_common
{
    /**
     * @brief Torque conversion utilities for different gripper types
     * 
     * This utility class provides torque conversion functions for:
     * - Changingtek 90 gripper
     * - Jodell gripper
     * 
     * All conversions use normalized torque (0.0-1.0) where:
     * - 0.0 = no torque
     * - 1.0 = maximum torque
     */
    class TorqueConverter
    {
    public:
        /** @brief Changingtek 系列共用；默认量程 0–255，120S 等由调用方传入 max。 */
        class Changingtek
        {
        public:
            static constexpr int DEFAULT_MAX_TORQUE = 0xFF;

            static int normalizedToModbus(double normalized, int max_torque)
            {
                normalized = std::clamp(normalized, 0.0, 1.0);
                return static_cast<int>(normalized * max_torque);
            }

            static int normalizedToModbus(double normalized)
            {
                return normalizedToModbus(normalized, DEFAULT_MAX_TORQUE);
            }

            static double modbusToNormalized(int modbus_torque, int max_torque)
            {
                modbus_torque = std::max(0, std::min(max_torque, modbus_torque));
                return static_cast<double>(modbus_torque) / max_torque;
            }

            static double modbusToNormalized(int modbus_torque)
            {
                return modbusToNormalized(modbus_torque, DEFAULT_MAX_TORQUE);
            }
        };

        using Changingtek90 = Changingtek;

        /** @brief EincinX — grip/release current (registers 0x4307 / 0x4308, manual defaults 35/50). */
        class EincinX
        {
        public:
            static int normalizedToGripCurrent(double normalized, int max_current = MAX_GRIP_CURRENT)
            {
                normalized = std::clamp(normalized, 0.0, 1.0);
                return static_cast<int>(normalized * max_current);
            }

            static int normalizedToReleaseCurrent(double normalized, int max_current = MAX_RELEASE_CURRENT)
            {
                normalized = std::clamp(normalized, 0.0, 1.0);
                return static_cast<int>(normalized * max_current);
            }

            static constexpr int MAX_GRIP_CURRENT = 100;   // 1.0A, unit 0.01A (tutorial 0x64)
            static constexpr int MAX_RELEASE_CURRENT = 90; // 0.9A (tutorial 0x5A)
        };

        /**
         * @brief Jodell gripper torque conversion
         * 
         * Torque range on wire: Modbus high byte 0-255 (see JodellCommandBuilder).
         * Normalized: 0.0 = no torque, 1.0 = maximum torque
         */
        class Jodell
        {
        public:
            static constexpr int MAX_TORQUE = 255;  // Maximum torque value (typical max is 255, 0xFF)

            /**
             * @brief Convert normalized torque (0.0-1.0) to Jodell torque byte (Modbus high byte)
             * @param normalized Normalized torque (0.0=no torque, 1.0=max torque)
             * @return Torque value 0-255 (written as high byte of the speed/torque register)
             */
            static int normalizedToJodell(double normalized)
            {
                // Limit to valid range
                normalized = std::clamp(normalized, 0.0, 1.0);
                
                // If normalized is exactly 0.0, return 0 (completely off)
                if (normalized == 0.0)
                {
                    return 0;
                }
                
                // Linear mapping: 0.0 -> 0, 1.0 -> 255
                int torque = static_cast<int>(normalized * MAX_TORQUE);
                return std::min(MAX_TORQUE, torque);
            }

            /**
             * @brief Convert Jodell torque byte to normalized torque (0.0-1.0)
             * @param jodell_torque Torque byte 0-255 (same range as Modbus high byte)
             * @return Normalized torque (0.0=no torque, 1.0=max torque)
             */
            static double jodellToNormalized(int jodell_torque)
            {
                // Limit to valid range
                jodell_torque = std::max(0, std::min(MAX_TORQUE, jodell_torque));
                return static_cast<double>(jodell_torque) / MAX_TORQUE;
            }
        };
    };
} // namespace gripper_hardware_common

