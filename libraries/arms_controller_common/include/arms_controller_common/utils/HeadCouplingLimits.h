#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace arms_controller_common
{
// Angles are radians: yaw left positive, pitch down positive.
struct HeadCouplingConfig
{
    bool enabled{false};
    std::string yaw_joint{"head_joint1"}, pitch_joint{"head_joint2"};
    static constexpr double yaw_limit{80.0 * 3.14159265358979323846 / 180.0};
    static constexpr double yaw_boundary{40.0 * 3.14159265358979323846 / 180.0};
    static constexpr double pitch_lower{-23.0 * 3.14159265358979323846 / 180.0};
    static constexpr double pitch_center_upper{30.0 * 3.14159265358979323846 / 180.0};
    static constexpr double pitch_side_upper{14.0 * 3.14159265358979323846 / 180.0};
    // Initial tuning values; validate on the actual mechanism before deployment.
    static constexpr double yaw_margin{2.0 * 3.14159265358979323846 / 180.0};
    static constexpr double pitch_margin{2.0 * 3.14159265358979323846 / 180.0};
    // Stage handoff checks position and coupling bounds, not measured velocity.
    static constexpr double position_tolerance{0.5 * 3.14159265358979323846 / 180.0};
    // Feedback-only tolerance; never applied to target or path limits.
    static constexpr double measurement_tolerance{0.1 * 3.14159265358979323846 / 180.0};
    // Seconds after the first planned segment completes.
    static constexpr double transition_timeout{2.0};
};

struct HeadJointIndices { size_t yaw, pitch; };
struct HeadAngleRange { double lower, upper; };
struct HeadPose { double yaw, pitch; };
struct HeadPitchTarget { double value; bool clipped; };

// Disabled configuration returns nullopt and does not require head joints.
void validateHeadCouplingConfig(const HeadCouplingConfig& config);
std::optional<HeadJointIndices> findHeadJointIndices(
    const HeadCouplingConfig& config, const std::vector<std::string>& names);

class HeadCouplingLimits
{
public:
    // Construct only for enabled configuration with finite URDF position limits.
    HeadCouplingLimits(HeadCouplingConfig config, HeadAngleRange yaw_urdf,
                       HeadAngleRange pitch_urdf);
    HeadAngleRange yawRange() const { return yaw_; }
    HeadAngleRange pitchRange(double yaw) const;
    bool contains(HeadPose pose) const;
    bool containsMeasured(HeadPose pose) const;
    // Only corrects tolerance-sized initial command errors; other starts are rejected.
    HeadPose normalizedStart(HeadPose pose) const;
    // Exact geometric check for a common monotone scalar phase in [0, 1].
    bool containsSegment(HeadPose start, HeadPose target) const;
    HeadPitchTarget clipPitch(HeadPose target) const;
    // Conservative rectangle check, not a trajectory validator. nullopt means no
    // extra waypoint is needed by this check; the actual trajectory still needs validation.
    std::optional<HeadPose> transition(HeadPose start, HeadPose target,
                                      bool allow_yaw, bool allow_pitch) const;
private:
    HeadAngleRange pitchBounds(double yaw) const;
    HeadCouplingConfig config_;
    HeadAngleRange yaw_, pitch_;
};
}
