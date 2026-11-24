#pragma once
#include <algorithm>

namespace arms_rviz_control_plugin {
inline float teleopHudCardWidth(float scale) {
    const float s = scale;
    const float pad = 12.f * s;
    const float split_w = std::max(1.f, s);
    const float pair_gap = 4.f * s;
    const float section_gap = 4.f * s;
    const float mirror_pair_w = 72.f * s + pair_gap + 36.f * s;
    const float speed_pair_w = 40.f * s + pair_gap + 48.f * s;
    return mirror_pair_w + section_gap + split_w + section_gap + speed_pair_w + 2.f * pad;
}
}
