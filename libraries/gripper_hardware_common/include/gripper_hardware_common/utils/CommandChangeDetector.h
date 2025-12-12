//
// Command Change Detector - Utility for detecting significant command changes
//
#pragma once

#include <cmath>

namespace gripper_hardware_common
{
    /**
     * @brief Utility class for detecting significant command changes
     * 
     * This class helps avoid sending redundant commands to grippers by
     * detecting when the command has changed significantly enough to warrant
     * sending a new command.
     */
    class CommandChangeDetector
    {
    public:
        /**
         * @brief Default threshold for command change detection (1%)
         */
        static constexpr double DEFAULT_THRESHOLD = 0.01;

        /**
         * @brief Check if command has changed significantly
         * 
         * @param current_command Current command value
         * @param last_command Last sent command value
         * @param threshold Change threshold (default: 0.01 = 1%)
         * @return true if change is significant enough to send command
         */
        static bool hasChanged(double current_command, double last_command, double threshold = DEFAULT_THRESHOLD)
        {
            return std::abs(current_command - last_command) >= threshold;
        }

        /**
         * @brief Check if command has changed significantly (absolute difference)
         * 
         * Useful for physical position commands (e.g., in meters)
         * 
         * @param current_command Current command value
         * @param last_command Last sent command value
         * @param absolute_threshold Absolute change threshold
         * @return true if change is significant enough to send command
         */
        static bool hasChangedAbsolute(double current_command, double last_command, double absolute_threshold)
        {
            return std::abs(current_command - last_command) >= absolute_threshold;
        }

        /**
         * @brief Check if command has changed significantly (percentage-based)
         * 
         * @param current_command Current command value
         * @param last_command Last sent command value
         * @param max_value Maximum possible value (for percentage calculation)
         * @param percentage_threshold Percentage threshold (e.g., 0.01 = 1%)
         * @return true if change is significant enough to send command
         */
        static bool hasChangedPercentage(
            double current_command, 
            double last_command, 
            double max_value, 
            double percentage_threshold = DEFAULT_THRESHOLD)
        {
            double absolute_threshold = max_value * percentage_threshold;
            return hasChangedAbsolute(current_command, last_command, absolute_threshold);
        }
    };
} // namespace gripper_hardware_common
