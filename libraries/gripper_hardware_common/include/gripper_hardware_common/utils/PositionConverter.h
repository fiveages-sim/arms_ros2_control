//
// Position Converter - Common utility functions for gripper position conversion
//
#pragma once

#include <cmath>
#include <algorithm>
#include <string>
#include <unordered_map>
#include <array>
#include <cstdint>

namespace gripper_hardware_common
{
    namespace detail
    {
        /** @brief 0=关/1=开 归一化 ↔ 0~max Modbus；Inverted 时 0=开、max=关（90 系列） */
        enum class ChangingtekRangeDirection { Linear, Inverted };

        template<ChangingtekRangeDirection Dir>
        struct ChangingtekRangeConverter
        {
            static uint16_t normalizedToModbus(double normalized, uint16_t max_position)
            {
                normalized = std::clamp(normalized, 0.0, 1.0);
                if constexpr (Dir == ChangingtekRangeDirection::Inverted)
                    return static_cast<uint16_t>((1.0 - normalized) * max_position);
                return static_cast<uint16_t>(normalized * max_position);
            }

            static double modbusToNormalized(uint32_t modbus_pos, uint16_t max_position)
            {
                if (modbus_pos > max_position)
                    modbus_pos = max_position;
                if constexpr (Dir == ChangingtekRangeDirection::Inverted)
                    return std::clamp(1.0 - static_cast<double>(modbus_pos) / max_position, 0.0, 1.0);
                return std::clamp(static_cast<double>(modbus_pos) / max_position, 0.0, 1.0);
            }
        };

        namespace LinkerHand
        {
            /** Map joint angle [lower, upper] rad to [0, 1] (0=open/straight, 1=bent/closed). */
            inline double rad_to_normalized(double rad, double lower, double upper)
            {
                const double range = upper - lower;
                if (range <= 1e-6)
                {
                    return 0.0;
                }
                return std::clamp((rad - lower) / range, 0.0, 1.0);
            }

            /** Map normalized [0, 1] to radians within [lower, upper]. */
            inline double normalized_to_rad(double normalized, double lower, double upper)
            {
                normalized = std::clamp(normalized, 0.0, 1.0);
                return lower + normalized * (upper - lower);
            }

            /** Modbus byte: 0=bent, 255=straight -> normalized [0, 1]. */
            inline double raw_to_normalized(uint8_t raw)
            {
                return std::clamp((255.0 - static_cast<double>(raw)) / 255.0, 0.0, 1.0);
            }

            /** Normalized [0, 1] -> Modbus byte (inverted: 0=bent, 255=straight). */
            inline uint16_t normalized_to_modbus_raw(double normalized)
            {
                normalized = std::clamp(normalized, 0.0, 1.0);
                return static_cast<uint16_t>(255 - std::round(normalized * 255.0));
            }
        }
    }

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
        /** @brief Changingtek 系列共用（90/120S/120S D）；Modbus 0=开、max=关，量程由调用方传入。 */
        class Changingtek
        {
        public:
            static constexpr uint16_t DEFAULT_MAX_POSITION = 9000;

            static uint16_t normalizedToModbus(double normalized, uint16_t max_position)
            {
                normalized = std::clamp(normalized, 0.0, 1.0);
                return static_cast<uint16_t>((1.0 - normalized) * max_position);
            }

            static double modbusToNormalized(uint32_t modbus_pos, uint16_t max_position)
            {
                if (modbus_pos > max_position)
                    modbus_pos = max_position;
                return std::clamp(1.0 - static_cast<double>(modbus_pos) / max_position, 0.0, 1.0);
            }

            static uint16_t normalizedToModbus(double normalized)
            {
                return normalizedToModbus(normalized, DEFAULT_MAX_POSITION);
            }

            static double modbusToNormalized(uint32_t modbus_pos)
            {
                return modbusToNormalized(modbus_pos, DEFAULT_MAX_POSITION);
            }
        };

        using Changingtek90 = Changingtek;

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
                normalized = std::clamp(normalized, 0.0, 1.0);
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
         * @brief LinkerHand dexterous hand position conversion (O7 / O6 / L6)
         *
         * Joint limits from linkerhand_description/xacro/*.xacro; only Modbus-controlled joints.
         */
        template<typename Limits>
        struct LinkerHandModel
        {
            static bool getLimits(const std::string& joint_name, double& lower, double& upper)
            {
                const auto& map = Limits::limitsMap();
                const auto it = map.find(joint_name);
                if (it == map.end())
                {
                    return false;
                }
                lower = it->second.first;
                upper = it->second.second;
                return true;
            }

            static std::string getJointNameByIndex(size_t index)
            {
                const auto& names = Limits::jointNames();
                return (index < names.size()) ? names[index] : "";
            }

            static double clampToLimits(double rad, const std::string& joint_name)
            {
                double lower = 0.0;
                double upper = 0.0;
                if (!getLimits(joint_name, lower, upper))
                {
                    return rad;
                }
                return std::max(lower, std::min(upper, rad));
            }

            static double rad_to_normalized(double rad, const std::string& joint_name)
            {
                double lower = 0.0;
                double upper = 0.0;
                if (!getLimits(joint_name, lower, upper))
                {
                    return 0.0;
                }
                return detail::LinkerHand::rad_to_normalized(clampToLimits(rad, joint_name), lower, upper);
            }

            static double normalized_to_rad(double normalized, const std::string& joint_name)
            {
                double lower = 0.0;
                double upper = 0.0;
                if (!getLimits(joint_name, lower, upper))
                {
                    return normalized;
                }
                return detail::LinkerHand::normalized_to_rad(normalized, lower, upper);
            }

            static uint16_t rad_to_modbus_raw(double rad, const std::string& joint_name)
            {
                return detail::LinkerHand::normalized_to_modbus_raw(rad_to_normalized(rad, joint_name));
            }

            static double modbus_raw_to_rad(uint8_t raw, const std::string& joint_name)
            {
                return normalized_to_rad(detail::LinkerHand::raw_to_normalized(raw), joint_name);
            }
        };

        struct LinkerHandO7Limits
        {
            static const std::unordered_map<std::string, std::pair<double, double>>& limitsMap()
            {
                static const std::unordered_map<std::string, std::pair<double, double>> map = {
                    {"thumb_joint1", {0.0, 0.5146}},
                    {"thumb_joint2", {0.0, 1.9189}},
                    {"thumb_joint3", {0.0, 1.1339}},
                    {"index_joint",  {0.0, 1.3607}},
                    {"middle_joint", {0.0, 1.3607}},
                    {"ring_joint",   {0.0, 1.3607}},
                    {"pinky_joint",  {0.0, 1.3607}},
                };
                return map;
            }

            static const std::array<const char*, 7>& jointNames()
            {
                static const std::array<const char*, 7> names = {
                    "thumb_joint1", "thumb_joint2", "thumb_joint3",
                    "index_joint", "middle_joint", "ring_joint", "pinky_joint"
                };
                return names;
            }
        };

        struct LinkerHandO6Limits
        {
            static const std::unordered_map<std::string, std::pair<double, double>>& limitsMap()
            {
                static const std::unordered_map<std::string, std::pair<double, double>> map = {
                    {"thumb_joint1", {0.0, 0.58}},
                    {"thumb_joint2", {0.0, 1.36}},
                    {"index_joint",  {0.0, 1.60}},
                    {"middle_joint", {0.0, 1.60}},
                    {"ring_joint",   {0.0, 1.60}},
                    {"pinky_joint",  {0.0, 1.60}},
                };
                return map;
            }

            static const std::array<const char*, 6>& jointNames()
            {
                static const std::array<const char*, 6> names = {
                    "thumb_joint1", "thumb_joint2",
                    "index_joint", "middle_joint", "ring_joint", "pinky_joint"
                };
                return names;
            }
        };

        struct LinkerHandL6Limits
        {
            static const std::unordered_map<std::string, std::pair<double, double>>& limitsMap()
            {
                static const std::unordered_map<std::string, std::pair<double, double>> map = {
                    {"thumb_joint1", {0.0, 0.99}},
                    {"thumb_joint2", {0.0, 1.39}},
                    {"index_joint",  {0.0, 1.26}},
                    {"middle_joint", {0.0, 1.26}},
                    {"ring_joint",   {0.0, 1.26}},
                    {"pinky_joint",  {0.0, 1.26}},
                };
                return map;
            }

            static const std::array<const char*, 6>& jointNames()
            {
                static const std::array<const char*, 6> names = {
                    "thumb_joint1", "thumb_joint2",
                    "index_joint", "middle_joint", "ring_joint", "pinky_joint"
                };
                return names;
            }
        };

        using LinkerHandO7 = LinkerHandModel<LinkerHandO7Limits>;
        using LinkerHandO6 = LinkerHandModel<LinkerHandO6Limits>;
        using LinkerHandL6 = LinkerHandModel<LinkerHandL6Limits>;
    };
} // namespace gripper_hardware_common
