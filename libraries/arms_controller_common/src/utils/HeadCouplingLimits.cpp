#include "arms_controller_common/utils/HeadCouplingLimits.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace arms_controller_common
{
    namespace
    {
        void require(bool condition, const char* message)
        {
            if (!condition)
            {
                throw std::invalid_argument(message);
            }
        }

        void validateRange(HeadAngleRange range)
        {
            require(std::isfinite(range.lower) && std::isfinite(range.upper) &&
                    range.lower <= range.upper, "Invalid head URDF range");
        }
    }

    void validateHeadCouplingConfig(const HeadCouplingConfig& c)
    {
        if (!c.enabled)
        {
            return;
        }
        require(!c.yaw_joint.empty() && !c.pitch_joint.empty() && c.yaw_joint != c.pitch_joint,
                "Head coupling requires two distinct joint names");
        for (double v : {c.yaw_limit, c.yaw_boundary, c.pitch_lower, c.pitch_center_upper,
                         c.pitch_side_upper, c.yaw_margin, c.pitch_margin,
                         c.position_tolerance, c.transition_timeout})
        {
            require(std::isfinite(v), "Head coupling parameters must be finite");
        }
        require(c.yaw_boundary > 0 && c.yaw_limit > c.yaw_boundary &&
                c.pitch_lower <= c.pitch_side_upper && c.pitch_side_upper <= c.pitch_center_upper,
                "Invalid head coupling boundary order");
        require(c.yaw_margin >= 0 && c.yaw_margin < c.yaw_boundary && c.pitch_margin >= 0 &&
                c.position_tolerance >= 0 && c.transition_timeout > 0,
                "Invalid head coupling margin/tolerance/timeout");
    }

    std::optional<HeadJointIndices> findHeadJointIndices(
        const HeadCouplingConfig& c, const std::vector<std::string>& names)
    {
        if (!c.enabled)
        {
            return std::nullopt;
        }
        validateHeadCouplingConfig(c);
        require(std::count(names.begin(), names.end(), c.yaw_joint) == 1 &&
                std::count(names.begin(), names.end(), c.pitch_joint) == 1,
                "Head coupling joints must each occur exactly once");
        return HeadJointIndices{static_cast<size_t>(std::find(names.begin(), names.end(), c.yaw_joint) - names.begin()),
                                static_cast<size_t>(std::find(names.begin(), names.end(), c.pitch_joint) - names.begin())};
    }

    HeadCouplingLimits::HeadCouplingLimits(HeadCouplingConfig c, HeadAngleRange y, HeadAngleRange p)
        : config_(std::move(c))
    {
        require(config_.enabled, "Cannot construct disabled head coupling limits");
        validateHeadCouplingConfig(config_);
        validateRange(y);
        validateRange(p);
        yaw_ = {std::max(y.lower, -config_.yaw_limit), std::min(y.upper, config_.yaw_limit)};
        pitch_ = p;
        validateRange(yaw_);
        // Check only reachable regions. A narrow yaw URDF need not reach the side region.
        const double center_lower = std::max(yaw_.lower, -config_.yaw_boundary);
        const double center_upper = std::min(yaw_.upper, config_.yaw_boundary);
        if (center_lower <= center_upper)
        {
            (void)pitchRange(center_lower);
        }
        if (yaw_.lower < -config_.yaw_boundary)
        {
            (void)pitchRange(yaw_.lower);
        }
        if (yaw_.upper > config_.yaw_boundary)
        {
            (void)pitchRange(yaw_.upper);
        }
        const bool spans_boundary = (yaw_.lower < -config_.yaw_boundary && yaw_.upper >= -config_.yaw_boundary) ||
                                   (yaw_.upper > config_.yaw_boundary && yaw_.lower <= config_.yaw_boundary);
        if (yaw_.lower < -config_.yaw_boundary && yaw_.upper >= -config_.yaw_boundary)
        {
            require(-config_.yaw_boundary + config_.yaw_margin <= yaw_.upper,
                    "No negative head transition yaw clearance");
        }
        if (yaw_.upper > config_.yaw_boundary && yaw_.lower <= config_.yaw_boundary)
        {
            require(config_.yaw_boundary - config_.yaw_margin >= yaw_.lower,
                    "No positive head transition yaw clearance");
        }
        if (spans_boundary)
        {
            const double low = std::max(p.lower, config_.pitch_lower);
            const double high = std::min(p.upper, config_.pitch_side_upper);
            require(low <= high - config_.pitch_margin, "No head transition pitch clearance");
        }
    }

    HeadAngleRange HeadCouplingLimits::pitchRange(double y) const
    {
        require(std::isfinite(y) && y >= yaw_.lower && y <= yaw_.upper,
                "Head yaw outside effective range");
        HeadAngleRange p{std::max(pitch_.lower, config_.pitch_lower),
            std::min(pitch_.upper, std::abs(y) <= config_.yaw_boundary ?
                     config_.pitch_center_upper : config_.pitch_side_upper)};
        validateRange(p);
        return p;
    }

    bool HeadCouplingLimits::contains(HeadPose p) const
    {
        if (!std::isfinite(p.yaw) || !std::isfinite(p.pitch) ||
            p.yaw < yaw_.lower || p.yaw > yaw_.upper)
        {
            return false;
        }
        const auto range = pitchRange(p.yaw);
        return p.pitch >= range.lower && p.pitch <= range.upper;
    }

    bool HeadCouplingLimits::containsSegment(HeadPose start, HeadPose target) const
    {
        if (!contains(start) || !contains(target))
        {
            return false;
        }

        std::vector<double> phases{0.0, 1.0};
        const double delta_yaw = target.yaw - start.yaw;
        if (delta_yaw != 0.0)
        {
            for (double boundary : {-config_.yaw_boundary, config_.yaw_boundary})
            {
                const double phase = (boundary - start.yaw) / delta_yaw;
                if (phase > 0.0 && phase < 1.0)
                {
                    phases.push_back(phase);
                }
            }
        }
        std::sort(phases.begin(), phases.end());
        for (size_t i = 1; i < phases.size(); ++i)
        {
            const double begin = phases[i - 1];
            const double end = phases[i];
            const double middle_yaw = start.yaw + (begin + end) * 0.5 * delta_yaw;
            const auto range = pitchRange(middle_yaw);
            const double first = start.pitch + begin * (target.pitch - start.pitch);
            const double last = start.pitch + end * (target.pitch - start.pitch);
            // Include one-sided boundary limits: an unsafe open interval cannot
            // be accepted merely because its boundary point belongs to center.
            if (std::min(first, last) < range.lower || std::max(first, last) > range.upper)
            {
                return false;
            }
        }
        return true;
    }

    HeadPitchTarget HeadCouplingLimits::clipPitch(HeadPose p) const
    {
        require(std::isfinite(p.pitch), "Head pitch must be finite");
        const auto range = pitchRange(p.yaw);
        const double clipped = std::clamp(p.pitch, range.lower, range.upper);
        return {clipped, clipped != p.pitch};
    }

    std::optional<HeadPose> HeadCouplingLimits::transition(
        HeadPose start, HeadPose target, bool allow_yaw, bool allow_pitch) const
    {
        require(contains(start) && contains(target), "Head transition requires legal endpoints");
        require(allow_yaw || start.yaw == target.yaw, "Head yaw movement is not authorized");
        require(allow_pitch || start.pitch == target.pitch, "Head pitch movement is not authorized");
        const double upper = std::min(pitch_.upper, config_.pitch_side_upper);
        if (std::max(std::abs(start.yaw), std::abs(target.yaw)) <= config_.yaw_boundary ||
            std::max(start.pitch, target.pitch) <= upper)
        {
            return std::nullopt;
        }
        require(allow_yaw && allow_pitch, "Head avoidance requires both authorized joints");
        const bool outward = std::abs(target.yaw) > config_.yaw_boundary;
        const double side_yaw = outward ? target.yaw : start.yaw;
        HeadPose waypoint{std::copysign(config_.yaw_boundary - config_.yaw_margin, side_yaw),
                         std::min({start.pitch, target.pitch, upper - config_.pitch_margin})};
        require(contains(waypoint), "Head transition waypoint has no URDF clearance");
        return waypoint;
    }
}
