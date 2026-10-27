#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace arms_controller_common::compliance_detail
{
// Rate [1/s] for an outer loop around a position servo. The delay bound reserves
// phase for unmodelled dynamics; it is a tuning rule, not a stability proof for
// an arbitrary third-party drive. delay includes transport/servo delay, while
// filter_delay accounts for the outer-loop measurement/output filters.
inline double servoOuterRate(double delay, double bandwidth_hz, double ratio,
                             double filter_delay)
{
    if (!std::isfinite(delay) || !std::isfinite(filter_delay) ||
        !std::isfinite(bandwidth_hz) || !std::isfinite(ratio) ||
        delay < 0.0 || filter_delay < 0.0 || bandwidth_hz <= 0.0 || ratio <= 0.0)
        return 0.0;
    constexpr double two_pi = 6.2831853071795864769;
    const double bandwidth_rate = std::min(
        two_pi * bandwidth_hz * std::min(ratio, 1.0), std::numeric_limits<double>::max());
    const double total_delay = delay + filter_delay;
    return total_delay > 0.0
        ? std::min(bandwidth_rate, 0.5 / total_delay) : bandwidth_rate;
}

// k_environment / damping is the low-frequency force-loop rate for a linear
// contact. A configured stiffness bound therefore sets a minimum damping.
inline double servoForceDamping(double configured_damping, double stiffness_bound,
                                double rate)
{
    constexpr double disabled = std::numeric_limits<double>::max();
    if (!std::isfinite(configured_damping) || configured_damping <= 0.0 ||
        !std::isfinite(stiffness_bound) || stiffness_bound < 0.0 ||
        !std::isfinite(rate) || rate <= 0.0)
        return disabled;
    const double minimum = stiffness_bound / rate;
    return std::max(configured_damping, std::min(minimum, disabled));
}

// Limit the demand to a speed the position servo can realize within one
// response horizon, so the command never runs ahead of the measured motion.
// Magnitudes only; the caller owns direction.
//
// Deliberately no environment-stiffness term: the first-contact peak of a blind
// approach is k_env * v * T_act whichever control law is used, so a stiffness
// budget cannot make the approach both fast and soft. The reachable peak is
// documented as F_des + k_env * v * T_act instead of being bounded by a guess.
inline double servoApproachVelocity(double configured_velocity, double response_horizon,
                                    double max_lead)
{
    if (!std::isfinite(configured_velocity) || configured_velocity <= 0.0 ||
        !std::isfinite(response_horizon) || response_horizon <= 0.0 ||
        !std::isfinite(max_lead) || max_lead <= 0.0)
        return 0.0;
    return std::min(configured_velocity, max_lead / response_horizon);
}

// Largest path-preserving scale for one step of a commanded-minus-measured
// position. If the existing lead is already outside the band, do not snap the
// command: permit retreat, but never worsen that violation or create a mirrored
// violation on the opposite side. The minimum across selected Cartesian axes
// can scale the whole joint increment without changing its direction.
inline double commandLeadScale(double lead, double increment, double limit)
{
    if (!std::isfinite(lead) || !std::isfinite(increment) ||
        !std::isfinite(limit) || limit < 0.0)
        return 0.0;
    if (increment == 0.0) return 1.0;
    const double lower = std::min(-limit, lead);
    const double upper = std::max(limit, lead);
    const double distance = increment > 0.0 ? upper - lead : lower - lead;
    return std::clamp(distance / increment, 0.0, 1.0);
}

// Exact pole conversion of an EMA specified at 500 Hz. Unlike alpha*dt/.002,
// this preserves the time constant when periods vary or feedback is slower.
inline double servoLpfAlpha(double alpha_500hz, double dt)
{
    if (!std::isfinite(alpha_500hz) || !std::isfinite(dt) || dt <= 0.0)
        return 0.0;
    if (alpha_500hz <= 0.0) return 0.0;
    if (alpha_500hz >= 1.0) return 1.0;
    return -std::expm1(std::log1p(-alpha_500hz) * dt / 0.002);
}

// An output filter must not keep pushing after the force loop requests braking,
// hold or retreat. Bypass its old velocity on deceleration/hold/reversal; smooth
// acceleration in the same direction. Position commands remain continuous.
inline double brakingVelocityFilter(double previous, double requested, double alpha)
{
    if (!std::isfinite(requested)) return 0.0;
    if (!std::isfinite(previous)) previous = 0.0;
    if (requested == 0.0 || std::abs(requested) < std::abs(previous) ||
        (previous > 0.0 && requested < 0.0) ||
        (previous < 0.0 && requested > 0.0))
        return requested;
    const double weight = std::isfinite(alpha) ? std::clamp(alpha, 0.0, 1.0) : 0.0;
    return weight * requested + (1.0 - weight) * previous;
}
}
