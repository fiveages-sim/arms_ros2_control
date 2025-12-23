#pragma once

#include <algorithm>
#include <cctype>
#include <string>

namespace arms_controller_common
{
    /**
     * @brief Common interpolation type used by FSM states (e.g. Home/MoveJ).
     */
    enum class InterpolationType
    {
        TANH,
        LINEAR
    };

    inline std::string toLowerCopy(std::string s)
    {
        std::transform(s.begin(), s.end(), s.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return s;
    }

    inline const char* toString(InterpolationType t)
    {
        switch (t)
        {
        case InterpolationType::LINEAR:
            return "linear";
        case InterpolationType::TANH:
        default:
            return "tanh";
        }
    }

    /**
     * @brief Parse interpolation type from string (case-insensitive).
     * @param type "tanh" or "linear"
     * @param fallback Returned when type is unknown
     */
    inline InterpolationType parseInterpolationType(const std::string& type,
                                                    InterpolationType fallback = InterpolationType::TANH)
    {
        const std::string t = toLowerCopy(type);
        if (t == "linear")
        {
            return InterpolationType::LINEAR;
        }
        if (t == "tanh")
        {
            return InterpolationType::TANH;
        }
        return fallback;
    }
} // namespace arms_controller_common


