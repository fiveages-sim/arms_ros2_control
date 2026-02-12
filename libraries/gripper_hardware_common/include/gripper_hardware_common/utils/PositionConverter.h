//
// Position Converter - Common utility functions for gripper position conversion
//
#pragma once

#include <cmath>
#include <algorithm>
#include <string>
#include <unordered_map>
#include <array>

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
                double pos_double = 1.0 - (static_cast<double>(pos_set) / static_cast<double>(MAX_POSITION));
                // Jodell: 0(closed) -> 0.0, 255(open) -> 1.0
                // Formula: normalized = 1.0 - (pos_set / 255.0)
                std::cout << " ++++++ pos set is " << pos_double << std::endl;
                return normalizedToPhysical(pos_double);
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

        /**
         * @brief DexterousHand position conversion utilities
         * 
         * Provides generic conversion functions for dexterous hand joints:
         * - Radians to degrees conversion
         * - Degrees to radians conversion
         * - Radian clamping to joint limits
         */
        class DexterousHand
        {
        public:
            /**
             * @brief Convert radians to degrees
             * @param rad Angle in radians
             * @return Angle in degrees
             */
            static double radToDegree(double rad)
            {
                return rad * 180.0 / M_PI;
            }

            /**
             * @brief Convert degrees to radians
             * @param degree Angle in degrees
             * @return Angle in radians
             */
            static double degreeToRad(double degree)
            {
                return degree * M_PI / 180.0;
            }

            /**
             * @brief Clamp radians to joint limits
             * 
             * Note: This function returns the value unchanged as it doesn't have access to hardware limits.
             * Actual clamping should be performed using the hardware's position limits.
             * 
             * @param rad Angle in radians
             * @param joint_idx Joint index (unused, kept for API compatibility)
             * @return Angle in radians (unchanged)
             */
            static double clampRadToLimits(double rad, size_t joint_idx)
            {
                (void)joint_idx; // Suppress unused parameter warning
                return rad;
            }
        };

        /**
         * @brief LinkerHand O7 (7-DOF) position conversion utilities
         * 
         * Joint limits extracted from linkerhand_description/xacro/o7.xacro
         * All limits are in radians
         */
        class LinkerHandO7
        {
        public:
            // Joint limits (lower, upper) in radians
            static constexpr double THUMB_JOINT3_LOWER = 0.0;
            static constexpr double THUMB_JOINT3_UPPER = 1.1339;
            
            static constexpr double THUMB_JOINT2_LOWER = 0.0;
            static constexpr double THUMB_JOINT2_UPPER = 1.9189;
            
            static constexpr double THUMB_JOINT1_LOWER = 0.0;
            static constexpr double THUMB_JOINT1_UPPER = 0.5146;
            
            static constexpr double THUMB_MCP_LOWER = 0.0;
            static constexpr double THUMB_MCP_UPPER = 0.7152;
            
            static constexpr double THUMB_IP_LOWER = 0.0;
            static constexpr double THUMB_IP_UPPER = 0.7763;
            
            static constexpr double INDEX_JOINT_LOWER = 0.0;
            static constexpr double INDEX_JOINT_UPPER = 1.3607;
            
            static constexpr double INDEX_PIP_LOWER = 0.0;
            static constexpr double INDEX_PIP_UPPER = 1.8317;
            
            static constexpr double INDEX_DIP_LOWER = 0.0;
            static constexpr double INDEX_DIP_UPPER = 0.638; // Max value (direction-dependent: 0.628 or 0.638)
            
            static constexpr double MIDDLE_JOINT_LOWER = 0.0;
            static constexpr double MIDDLE_JOINT_UPPER = 1.3607;
            
            static constexpr double MIDDLE_PIP_LOWER = 0.0;
            static constexpr double MIDDLE_PIP_UPPER = 1.8317;
            
            static constexpr double MIDDLE_DIP_LOWER = 0.0;
            static constexpr double MIDDLE_DIP_UPPER = 0.628;
            
            static constexpr double RING_JOINT_LOWER = 0.0;
            static constexpr double RING_JOINT_UPPER = 1.3607;
            
            static constexpr double RING_PIP_LOWER = 0.0;
            static constexpr double RING_PIP_UPPER = 1.8317;
            
            static constexpr double RING_DIP_LOWER = 0.0;
            static constexpr double RING_DIP_UPPER = 0.628;
            
            static constexpr double PINKY_JOINT_LOWER = 0.0;
            static constexpr double PINKY_JOINT_UPPER = 1.3607;
            
            static constexpr double PINKY_PIP_LOWER = 0.0;
            static constexpr double PINKY_PIP_UPPER = 1.8317;
            
            static constexpr double PINKY_DIP_LOWER = 0.0;
            static constexpr double PINKY_DIP_UPPER = 0.628;

        private:
            // Lookup table for joint limits: joint_name -> {lower, upper}
            static const std::unordered_map<std::string, std::pair<double, double>>& getLimitsMap()
            {
                static const std::unordered_map<std::string, std::pair<double, double>> limits_map = {
                    {"thumb_joint1", {THUMB_JOINT1_LOWER, THUMB_JOINT1_UPPER}},
                    {"thumb_joint2", {THUMB_JOINT2_LOWER, THUMB_JOINT2_UPPER}},
                    {"thumb_joint3", {THUMB_JOINT3_LOWER, THUMB_JOINT3_UPPER}},
                    {"thumb_mcp", {THUMB_MCP_LOWER, THUMB_MCP_UPPER}},
                    {"thumb_ip", {THUMB_IP_LOWER, THUMB_IP_UPPER}},
                    {"index_joint", {INDEX_JOINT_LOWER, INDEX_JOINT_UPPER}},
                    {"index_pip", {INDEX_PIP_LOWER, INDEX_PIP_UPPER}},
                    {"index_dip", {INDEX_DIP_LOWER, INDEX_DIP_UPPER}},
                    {"middle_joint", {MIDDLE_JOINT_LOWER, MIDDLE_JOINT_UPPER}},
                    {"middle_pip", {MIDDLE_PIP_LOWER, MIDDLE_PIP_UPPER}},
                    {"middle_dip", {MIDDLE_DIP_LOWER, MIDDLE_DIP_UPPER}},
                    {"ring_joint", {RING_JOINT_LOWER, RING_JOINT_UPPER}},
                    {"ring_pip", {RING_PIP_LOWER, RING_PIP_UPPER}},
                    {"ring_dip", {RING_DIP_LOWER, RING_DIP_UPPER}},
                    {"pinky_joint", {PINKY_JOINT_LOWER, PINKY_JOINT_UPPER}},
                    {"pinky_pip", {PINKY_PIP_LOWER, PINKY_PIP_UPPER}},
                    {"pinky_dip", {PINKY_DIP_LOWER, PINKY_DIP_UPPER}}
                };
                return limits_map;
            }

            // Joint names in order (O7: 7 joints)
            static const std::array<const char*, 7>& getJointNames()
            {
                static const std::array<const char*, 7> joint_names = {
                    "thumb_joint1", "thumb_joint2", "thumb_joint3",
                    "index_joint", "middle_joint", "ring_joint", "pinky_joint"
                };
                return joint_names;
            }

        public:

            /**
             * @brief Get joint limits by name
             * @param joint_name Joint name (e.g., "thumb_joint3", "index_joint")
             * @param lower Output lower limit in radians
             * @param upper Output upper limit in radians
             * @return true if joint found, false otherwise
             */
            static bool getLimits(const std::string& joint_name, double& lower, double& upper)
            {
                const auto& limits_map = getLimitsMap();
                auto it = limits_map.find(joint_name);
                if (it != limits_map.end())
                {
                    lower = it->second.first;
                    upper = it->second.second;
                    return true;
                }
                return false;
            }

            /**
             * @brief Get joint name by index (O7 hand order)
             * @param index Joint index (0-6)
             * @return Joint name, or empty string if invalid
             */
            static std::string getJointNameByIndex(size_t index)
            {
                const auto& joint_names = getJointNames();
                return (index < joint_names.size()) ? joint_names[index] : "";
            }

            /**
             * @brief Clamp joint position to limits
             * @param rad Angle in radians
             * @param joint_name Joint name (e.g., "thumb_joint3", "index_joint")
             * @return Clamped angle in radians
             */
            static double clampToLimits(double rad, const std::string& joint_name)
            {
                double lower = 0.0;
                double upper = 0.0;
                if (!getLimits(joint_name, lower, upper))
                {
                    // Unknown joint, return unchanged
                    return rad;
                }
                return std::max(lower, std::min(upper, rad));
            }
        };

        /**
         * @brief LinkerHand O6 (6-DOF) position conversion utilities
         * 
         * Joint limits extracted from linkerhand_description/xacro/o6.xacro
         * All limits are in radians
         */
        class LinkerHandO6
        {
        public:
            // Joint limits (lower, upper) in radians
            // Note: thumb_joint2 upper limit is direction-dependent (1.3 for direction=1, 1.36 for direction=-1)
            // Using max value 1.36 for safety
            static constexpr double THUMB_JOINT2_LOWER = 0.0;
            static constexpr double THUMB_JOINT2_UPPER = 1.36;
            
            static constexpr double THUMB_JOINT1_LOWER = 0.0;
            static constexpr double THUMB_JOINT1_UPPER = 0.58;
            
            static constexpr double THUMB_DIP_LOWER = 0.0;
            static constexpr double THUMB_DIP_UPPER = 1.08;
            
            static constexpr double INDEX_JOINT_LOWER = 0.0;
            static constexpr double INDEX_JOINT_UPPER = 1.60;
            
            static constexpr double INDEX_DIP_LOWER = 0.0;
            static constexpr double INDEX_DIP_UPPER = 1.43;
            
            static constexpr double MIDDLE_JOINT_LOWER = 0.0;
            static constexpr double MIDDLE_JOINT_UPPER = 1.60;
            
            static constexpr double MIDDLE_DIP_LOWER = 0.0;
            static constexpr double MIDDLE_DIP_UPPER = 1.43;
            
            static constexpr double RING_JOINT_LOWER = 0.0;
            static constexpr double RING_JOINT_UPPER = 1.60;
            
            static constexpr double RING_DIP_LOWER = 0.0;
            static constexpr double RING_DIP_UPPER = 1.43;
            
            static constexpr double PINKY_JOINT_LOWER = 0.0;
            static constexpr double PINKY_JOINT_UPPER = 1.60;
            
            static constexpr double PINKY_DIP_LOWER = 0.0;
            static constexpr double PINKY_DIP_UPPER = 1.43;

        private:
            // Lookup table for joint limits: joint_name -> {lower, upper}
            static const std::unordered_map<std::string, std::pair<double, double>>& getLimitsMap()
            {
                static const std::unordered_map<std::string, std::pair<double, double>> limits_map = {
                    {"thumb_joint1", {THUMB_JOINT1_LOWER, THUMB_JOINT1_UPPER}},
                    {"thumb_joint2", {THUMB_JOINT2_LOWER, THUMB_JOINT2_UPPER}},
                    {"thumb_dip", {THUMB_DIP_LOWER, THUMB_DIP_UPPER}},
                    {"index_joint", {INDEX_JOINT_LOWER, INDEX_JOINT_UPPER}},
                    {"index_dip", {INDEX_DIP_LOWER, INDEX_DIP_UPPER}},
                    {"middle_joint", {MIDDLE_JOINT_LOWER, MIDDLE_JOINT_UPPER}},
                    {"middle_dip", {MIDDLE_DIP_LOWER, MIDDLE_DIP_UPPER}},
                    {"ring_joint", {RING_JOINT_LOWER, RING_JOINT_UPPER}},
                    {"ring_dip", {RING_DIP_LOWER, RING_DIP_UPPER}},
                    {"pinky_joint", {PINKY_JOINT_LOWER, PINKY_JOINT_UPPER}},
                    {"pinky_dip", {PINKY_DIP_LOWER, PINKY_DIP_UPPER}}
                };
                return limits_map;
            }

            // Joint names in order (O6: 6 joints)
            static const std::array<const char*, 6>& getJointNames()
            {
                static const std::array<const char*, 6> joint_names = {
                    "thumb_joint1", "thumb_joint2",
                    "index_joint", "middle_joint", "ring_joint", "pinky_joint"
                };
                return joint_names;
            }

        public:

            /**
             * @brief Get joint limits by name
             * @param joint_name Joint name (e.g., "thumb_joint2", "index_joint")
             * @param lower Output lower limit in radians
             * @param upper Output upper limit in radians
             * @return true if joint found, false otherwise
             */
            static bool getLimits(const std::string& joint_name, double& lower, double& upper)
            {
                const auto& limits_map = getLimitsMap();
                auto it = limits_map.find(joint_name);
                if (it != limits_map.end())
                {
                    lower = it->second.first;
                    upper = it->second.second;
                    return true;
                }
                return false;
            }

            /**
             * @brief Get joint name by index (O6 hand order)
             * @param index Joint index (0-5)
             * @return Joint name, or empty string if invalid
             */
            static std::string getJointNameByIndex(size_t index)
            {
                const auto& joint_names = getJointNames();
                return (index < joint_names.size()) ? joint_names[index] : "";
            }

            /**
             * @brief Clamp joint position to limits
             * @param rad Angle in radians
             * @param joint_name Joint name (e.g., "thumb_joint2", "index_joint")
             * @return Clamped angle in radians
             */
            static double clampToLimits(double rad, const std::string& joint_name)
            {
                double lower = 0.0;
                double upper = 0.0;
                if (!getLimits(joint_name, lower, upper))
                {
                    // Unknown joint, return unchanged
                    return rad;
                }
                return std::max(lower, std::min(upper, rad));
            }
        };

        /**
         * @brief LinkerHand L6 (6-DOF) position conversion utilities
         * 
         * Joint limits extracted from linkerhand_description/xacro/l6.xacro
         * All limits are in radians
         */
        class LinkerHandL6
        {
        public:
            // Joint limits (lower, upper) in radians
            static constexpr double THUMB_JOINT2_LOWER = 0.0;
            static constexpr double THUMB_JOINT2_UPPER = 1.39;
            
            static constexpr double THUMB_JOINT1_LOWER = 0.0;
            static constexpr double THUMB_JOINT1_UPPER = 0.99;
            
            static constexpr double THUMB_DIP_LOWER = 0.0;
            static constexpr double THUMB_DIP_UPPER = 1.22;
            
            static constexpr double INDEX_JOINT_LOWER = 0.0;
            static constexpr double INDEX_JOINT_UPPER = 1.26;
            
            static constexpr double INDEX_DIP_LOWER = 0.0;
            static constexpr double INDEX_DIP_UPPER = 1.14;
            
            static constexpr double MIDDLE_JOINT_LOWER = 0.0;
            static constexpr double MIDDLE_JOINT_UPPER = 1.26;
            
            static constexpr double MIDDLE_DIP_LOWER = 0.0;
            static constexpr double MIDDLE_DIP_UPPER = 1.14;
            
            static constexpr double RING_JOINT_LOWER = 0.0;
            static constexpr double RING_JOINT_UPPER = 1.26;
            
            static constexpr double RING_DIP_LOWER = 0.0;
            static constexpr double RING_DIP_UPPER = 1.14;
            
            static constexpr double PINKY_JOINT_LOWER = 0.0;
            static constexpr double PINKY_JOINT_UPPER = 1.26;
            
            static constexpr double PINKY_DIP_LOWER = 0.0;
            static constexpr double PINKY_DIP_UPPER = 1.14;

        private:
            // Lookup table for joint limits: joint_name -> {lower, upper}
            static const std::unordered_map<std::string, std::pair<double, double>>& getLimitsMap()
            {
                static const std::unordered_map<std::string, std::pair<double, double>> limits_map = {
                    {"thumb_joint1", {THUMB_JOINT1_LOWER, THUMB_JOINT1_UPPER}},
                    {"thumb_joint2", {THUMB_JOINT2_LOWER, THUMB_JOINT2_UPPER}},
                    {"thumb_dip", {THUMB_DIP_LOWER, THUMB_DIP_UPPER}},
                    {"index_joint", {INDEX_JOINT_LOWER, INDEX_JOINT_UPPER}},
                    {"index_dip", {INDEX_DIP_LOWER, INDEX_DIP_UPPER}},
                    {"middle_joint", {MIDDLE_JOINT_LOWER, MIDDLE_JOINT_UPPER}},
                    {"middle_dip", {MIDDLE_DIP_LOWER, MIDDLE_DIP_UPPER}},
                    {"ring_joint", {RING_JOINT_LOWER, RING_JOINT_UPPER}},
                    {"ring_dip", {RING_DIP_LOWER, RING_DIP_UPPER}},
                    {"pinky_joint", {PINKY_JOINT_LOWER, PINKY_JOINT_UPPER}},
                    {"pinky_dip", {PINKY_DIP_LOWER, PINKY_DIP_UPPER}}
                };
                return limits_map;
            }

            // Joint names in order (L6: 6 joints)
            static const std::array<const char*, 6>& getJointNames()
            {
                static const std::array<const char*, 6> joint_names = {
                    "thumb_joint1", "thumb_joint2",
                    "index_joint", "middle_joint", "ring_joint", "pinky_joint"
                };
                return joint_names;
            }

        public:

            /**
             * @brief Get joint limits by name
             * @param joint_name Joint name (e.g., "thumb_joint2", "index_joint")
             * @param lower Output lower limit in radians
             * @param upper Output upper limit in radians
             * @return true if joint found, false otherwise
             */
            static bool getLimits(const std::string& joint_name, double& lower, double& upper)
            {
                const auto& limits_map = getLimitsMap();
                auto it = limits_map.find(joint_name);
                if (it != limits_map.end())
                {
                    lower = it->second.first;
                    upper = it->second.second;
                    return true;
                }
                return false;
            }

            /**
             * @brief Get joint name by index (L6 hand order)
             * @param index Joint index (0-5)
             * @return Joint name, or empty string if invalid
             */
            static std::string getJointNameByIndex(size_t index)
            {
                const auto& joint_names = getJointNames();
                return (index < joint_names.size()) ? joint_names[index] : "";
            }

            /**
             * @brief Clamp joint position to limits
             * @param rad Angle in radians
             * @param joint_name Joint name (e.g., "thumb_joint2", "index_joint")
             * @return Clamped angle in radians
             */
            static double clampToLimits(double rad, const std::string& joint_name)
            {
                double lower = 0.0;
                double upper = 0.0;
                if (!getLimits(joint_name, lower, upper))
                {
                    // Unknown joint, return unchanged
                    return rad;
                }
                return std::max(lower, std::min(upper, rad));
            }
        };
    };
} // namespace gripper_hardware_common
