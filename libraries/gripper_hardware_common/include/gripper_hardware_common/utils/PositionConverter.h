//
// Position Converter - Common utility functions for gripper position conversion
//
#pragma once

#include <cmath>
#include <algorithm>

namespace gripper_hardware_common
{
    /**
     * @brief Position conversion utilities for different gripper types
     * 
     * This utility class provides position conversion functions for:
     * - Changingtek 90 gripper (0-9000 range)
     * - Jodell gripper (0-255 range)
     * 
     * All conversions use normalized position (0.0 = closed, 1.0 = open)
     */
    class PositionConverter
    {
    public:
        /**
         * @brief Changingtek 90 gripper position conversion
         * 
         * Position range: 0-9000 (0 = open, 9000 = closed)
         * Normalized: 0.0 = closed, 1.0 = open
         */
        class Changingtek90
        {
        public:
            static constexpr uint16_t MAX_POSITION = 9000;  // Maximum position value

            /**
             * @brief Convert normalized position (0.0-1.0) to Modbus position (0-9000)
             * @param normalized Normalized position (0.0=closed, 1.0=open)
             * @return Modbus position value (0=open, 9000=closed)
             */
            static uint16_t normalizedToModbus(double normalized)
            {
                // Limit to valid range
                normalized = std::max(0.0, std::min(1.0, normalized));
                
                // Changingtek: 0.0(closed) -> 9000, 1.0(open) -> 0
                return static_cast<uint16_t>((1.0 - normalized) * MAX_POSITION);
            }

            /**
             * @brief Convert Modbus position (0-9000) to normalized position (0.0-1.0)
             * @param modbus_pos Modbus position value (0=open, 9000=closed)
             * @return Normalized position (0.0=closed, 1.0=open)
             */
            static double modbusToNormalized(uint32_t modbus_pos)
            {
                // Changingtek: 0(open) -> 1.0, 9000(closed) -> 0.0
                if (modbus_pos > MAX_POSITION)
                {
                    modbus_pos = MAX_POSITION;
                }

                double normalized = 1.0 - (static_cast<double>(modbus_pos) / MAX_POSITION);
                return std::max(0.0, std::min(1.0, normalized));
            }
        };

        /**
         * @brief Jodell gripper position conversion
         * 
         * Position range: 0-255 (0 = closed, 255 = open)
         * Normalized: 0.0 = closed, 1.0 = open
         */
        class Jodell
        {
        public:
            static constexpr int MAX_POSITION = 255;  // Maximum position value
            static constexpr double MAX_OPENING_DISTANCE = 0.038372;  // Maximum opening distance (meters) for RG75

            /**
             * @brief Convert normalized position (0.0-1.0) to Jodell position (0-255)
             * @param normalized Normalized position (0.0=closed, 1.0=open)
             * @return Jodell position value (0=closed, 255=open)
             */
            static int normalizedToJodell(double normalized)
            {
                // Limit to valid range
                normalized = physicalToNormalized(normalized);
                
                // Jodell: 0.0(closed) -> 255, 1.0(open) -> 0
                // Formula: pos_set = 255 * (1.0 - normalized)
                return 255 - static_cast<int>(normalized * MAX_POSITION);
            }

            /**
             * @brief Convert Jodell position (0-255) to normalized position (0.0-1.0)
             * @param pos_set Jodell position value (0=closed, 255=open)
             * @return Normalized position (0.0=closed, 1.0=open)
             */
            static double jodellToNormalized(int pos_set)
            {
                // Limit to valid range
                pos_set = physicalToNormalized(pos_set);
                
                // Jodell: 0(closed) -> 0.0, 255(open) -> 1.0
                // Formula: normalized = 1.0 - (pos_set / 255.0)
                return 1.0 - (static_cast<double>(pos_set) / MAX_POSITION);
            }

            /**
             * @brief ExtractnormalizedToPhysical position from Jodell status register
             * 
             * Jodell status register format: position is in the high byte of the second register
             * @param status_reg_high High byte of status register (robot_stats[1] >> 8)
             * @return Normalized position (0.0=closed, 1.0=open)
             */
            static double extractPositionFromStatus(int status_reg_high)
            {
                int pos_now_get = status_reg_high;
                return jodellToNormalized(pos_now_get);
            }

            /**
             * @brief Convert physical position (meters) to normalized position (0.0-1.0)
             * @param physical_pos Physical position in meters (0.0=closed, MAX_OPENING_DISTANCE=open)
             * @return Normalized position (0.0=closed, 1.0=open)
             */
            static double physicalToNormalized(double physical_pos)
            {
                // Limit to valid range
                physical_pos = std::max(0.0, std::min(MAX_OPENING_DISTANCE, physical_pos));
                return physical_pos / MAX_OPENING_DISTANCE;
            }

            /**
             * @brief Convert normalized position (0.0-1.0) to physical position (meters)
             * @param normalized Normalized position (0.0=closed, 1.0=open)
             * @return Physical position in meters (0.0=closed, MAX_OPENING_DISTANCE=open)
             */
            static double normalizedToPhysical(double normalized)
            {
                // Limit to valid range
                normalized = std::max(0.0, std::min(1.0, normalized));
                return normalized * MAX_OPENING_DISTANCE;
            }
        };
    };
} // namespace gripper_hardware_common
