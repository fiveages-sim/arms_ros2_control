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

        /** @brief Changingtek VELOCITY_REG；默认 0–255，120S 等由调用方传入 max。 */
        class Changingtek
        {
        public:
            static constexpr int DEFAULT_MAX_REGISTER_VALUE = 255;

            static int normalizedToVelocityRegister(double normalized, int max_register_value)
            {
                normalized = std::clamp(normalized, 0.0, 1.0);
                const int v = static_cast<int>(std::lround(normalized * static_cast<double>(max_register_value)));
                return std::max(0, std::min(max_register_value, v));
            }

            static int normalizedToVelocityRegister(double normalized)
            {
                return normalizedToVelocityRegister(normalized, DEFAULT_MAX_REGISTER_VALUE);
            }
        };

        using Changingtek90 = Changingtek;
    };
} // namespace gripper_hardware_common
