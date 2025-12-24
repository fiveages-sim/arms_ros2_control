//
// Jodell Command Builder - Utility for building Jodell gripper Modbus commands
//
#pragma once


#include "gripper_hardware_common/utils/PositionConverter.h"
#include <vector>
#include <cstdint>

namespace gripper_hardware_common
{
    /**
     * @brief Utility class for building Jodell gripper Modbus commands
     * 
     * Jodell gripper uses a specific 3-register command format:
     * - Register 1: Trigger value (0x09)
     * - Register 2: Position (pos_set << 8)
     * - Register 3: Speed and force combined ((trq_set << 8) | vel_set)
     */
    class JodellCommandBuilder
    {
    public:
        /**
         * @brief Build Jodell gripper command data
         * 
         * @param pos_set Position setting (0-255, 0=closed, 255=open)
         * @param trq_set Torque setting (typically 20-100 or 0-255)
         * @param vel_set Velocity setting (typically 1-100 or 0-255)
         * @return Vector of 3 uint16_t values ready for Modbus write
         */
        static std::vector<uint16_t> buildCommand(int pos_set, int trq_set, int vel_set)
        {
            std::vector<uint16_t> values;
            
            // Register 1: Trigger value
            values.push_back(0x09);
            
            // Register 2: Position (high byte)
            values.push_back(static_cast<uint16_t>(pos_set << 8));
            
            // Register 3: Speed and force combined
            // High byte: torque, Low byte: velocity
            values.push_back(static_cast<uint16_t>((trq_set << 8) | (vel_set & 0xFF)));
            
            return values;
        }

        /**
         * @brief Build Jodell gripper command data with default torque and velocity
         * 
         * Uses default values: trq_set = 255, vel_set = 255
         * 
         * @param pos_set Position setting (0-255, 0=closed, 255=open)
         * @return Vector of 3 uint16_t values ready for Modbus write
         */
        static std::vector<uint16_t> buildCommand(int pos_set)
        {
            return buildCommand(pos_set, 255, 255);
        }

        /**
         * @brief Build Jodell gripper command data from normalized position
         * 
         * Converts normalized position (0.0-1.0) to Jodell position (0-255)
         * and builds command with default torque and velocity.
         * 
         * @param normalized Normalized position (0.0=closed, 1.0=open)
         * @param trq_set Torque setting (default: 255)
         * @param vel_set Velocity setting (default: 255)
         * @return Vector of 3 uint16_t values ready for Modbus write
         */
        static std::vector<uint16_t> buildCommandFromNormalized(
            double normalized, 
            int trq_set = 255, 
            int vel_set = 255)
        {
            using namespace gripper_hardware_common;
            int pos_set = PositionConverter::Jodell::normalizedToJodell(normalized);
            return buildCommand(pos_set, trq_set, vel_set);
        }
    };
} // namespace gripper_hardware_common
