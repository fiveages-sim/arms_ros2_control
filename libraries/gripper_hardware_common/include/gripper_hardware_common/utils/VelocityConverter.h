//
// VelocityConverter - normalized gripper velocity -> device-specific wire values
//
#pragma once

#include <algorithm>
#include <cmath>

namespace gripper_hardware_common
{
    /**
     * @brief Per-gripper-type velocity conversion (mirrors TorqueConverter usage pattern).
     *
     * Call sites pass normalized velocity in [0, 1]; each nested class maps to that device's register/byte.
     */
    class VelocityConverter
    {
    public:
        /**
         * @brief Jodell (RG75 / JD) — low byte of speed/torque combined Modbus word (0–255).
         */
        class Jodell
        {
        public:
            static constexpr int MAX_LOW_BYTE = 255;

            static int normalizedToLowByte(double normalized)
            {
                normalized = std::clamp(normalized, 0.0, 1.0);
                const int v = static_cast<int>(std::lround(normalized * static_cast<double>(MAX_LOW_BYTE)));
                return std::max(0, std::min(MAX_LOW_BYTE, v));
            }
        };

        /**
         * @brief Changingtek 90 — VELOCITY_REG value used in Marvin stack (0–255 range).
         */
        class Changingtek90
        {
        public:
            static constexpr int MAX_REGISTER_VALUE = 255;

            static int normalizedToVelocityRegister(double normalized)
            {
                normalized = std::clamp(normalized, 0.0, 1.0);
                const int v = static_cast<int>(std::lround(normalized * static_cast<double>(MAX_REGISTER_VALUE)));
                return std::max(0, std::min(MAX_REGISTER_VALUE, v));
            }
        };

        /** @brief Changingtek 120S — VELOCITY_REG 0–100 (%). */
        class Changingtek120S
        {
        public:
            static constexpr int MAX_REGISTER_VALUE = 100;

            static int normalizedToVelocityRegister(double normalized)
            {
                normalized = std::clamp(normalized, 0.0, 1.0);
                const int v = static_cast<int>(std::lround(normalized * static_cast<double>(MAX_REGISTER_VALUE)));
                return std::max(0, std::min(MAX_REGISTER_VALUE, v));
            }
        };
    };
} // namespace gripper_hardware_common
