#pragma once

#include <algorithm>
#include <cmath>

namespace ocs2::mobile_manipulator
{
struct ForceAdmittanceAxisState
{
    double position{0.0};
    double velocity{0.0};
    double acceleration{0.0};
    // -1/0/+1: latched negative/none/positive workspace-bound braking.
    int saturation_direction{0};
};

struct ForceAdmittanceAxisLimits
{
    double virtual_mass{100.0};
    double virtual_damping{6000.0};
    double max_velocity{0.001};
    double max_acceleration{0.003};
    double max_jerk{0.01};
    double max_displacement{0.01};
    double min_dt{0.001};
    double max_dt{0.02};
};

/**
 * One bounded Cartesian force-admittance integration step.
 *
 *   M a_raw + D v = force_error
 *
 * Acceleration is jerk-limited before velocity and position integration. A
 * stopping-velocity envelope brakes before the displacement boundary. Invalid
 * inputs do not propagate NaN/Inf; they produce a smooth braking step.
 */
inline double predictJerkLimitedStoppingDistance(
    double velocity, double acceleration, double max_acceleration,
    double max_jerk, double /* dt */)
{
    // Closed-form stop: ramp acceleration with -j until either velocity reaches
    // zero or -amax is reached, then finish with constant -amax. O(1), suitable
    // for a controller callback.
    const double v0 = std::max(0.0, velocity);
    const double a0 = std::clamp(acceleration, -max_acceleration, max_acceleration);
    if (v0 <= 0.0)
    {
        return 0.0;
    }
    const double t_to_zero =
        (a0 + std::sqrt(std::max(0.0, a0 * a0 + 2.0 * max_jerk * v0))) /
        max_jerk;
    const double t_to_max_decel = (a0 + max_acceleration) / max_jerk;
    const double ramp_time = std::min(t_to_zero, t_to_max_decel);
    double distance = v0 * ramp_time + 0.5 * a0 * ramp_time * ramp_time -
                      max_jerk * ramp_time * ramp_time * ramp_time / 6.0;
    if (t_to_zero > t_to_max_decel)
    {
        const double velocity_after_ramp = std::max(
            0.0, v0 + a0 * ramp_time - 0.5 * max_jerk * ramp_time * ramp_time);
        distance += velocity_after_ramp * velocity_after_ramp /
                    (2.0 * max_acceleration);
    }
    return std::max(0.0, distance);
}

inline ForceAdmittanceAxisState stepForceAdmittanceAxis(
    ForceAdmittanceAxisState state, double force_error, double dt,
    const ForceAdmittanceAxisLimits& limits)
{
    const double mass = std::max(std::abs(limits.virtual_mass), 1e-6);
    const double damping = std::max(limits.virtual_damping, 0.0);
    const double vmax = std::max(limits.max_velocity, 0.0);
    const double amax = std::max(limits.max_acceleration, 1e-6);
    const double jmax = std::max(limits.max_jerk, 1e-6);
    const double xmax = std::max(limits.max_displacement, 0.0);
    const double dt_min = std::max(limits.min_dt, 1e-6);
    const double dt_max = std::max(limits.max_dt, dt_min);
    const double h = std::clamp(std::isfinite(dt) ? dt : dt_min, dt_min, dt_max);

    if (!std::isfinite(state.position) || !std::isfinite(state.velocity) ||
        !std::isfinite(state.acceleration))
    {
        state = {};
    }
    state.saturation_direction = std::clamp(state.saturation_direction, -1, 1);
    if (!std::isfinite(force_error))
    {
        force_error = 0.0;
    }

    // Release a workspace latch only when the force command points back inward.
    if ((state.saturation_direction > 0 && force_error < 0.0) ||
        (state.saturation_direction < 0 && force_error > 0.0))
    {
        state.saturation_direction = 0;
    }
    double desired_acceleration = std::clamp(
        (force_error - damping * state.velocity) / mass, -amax, amax);

    // Start reducing acceleration before velocity saturation. The a²/(2j)
    // term is the velocity gained while jerk ramps acceleration back to zero.
    const double positive_velocity_reserve =
        std::max(0.0, state.acceleration) * std::max(0.0, state.acceleration) /
        (2.0 * jmax) + std::max(0.0, state.acceleration) * h;
    const double negative_velocity_reserve =
        std::max(0.0, -state.acceleration) * std::max(0.0, -state.acceleration) /
        (2.0 * jmax) + std::max(0.0, -state.acceleration) * h;
    if (state.velocity >= vmax - positive_velocity_reserve)
    {
        desired_acceleration = std::min(desired_acceleration, 0.0);
    }
    if (state.velocity <= -vmax + negative_velocity_reserve)
    {
        desired_acceleration = std::max(desired_acceleration, 0.0);
    }

    // Jerk-aware position braking. Mirror negative motion into the positive
    // direction, predict the complete S-curve stop, and brake before the bound.
    const double stop_positive = predictJerkLimitedStoppingDistance(
        state.velocity, state.acceleration, amax, jmax, h);
    const double stop_negative = predictJerkLimitedStoppingDistance(
        -state.velocity, -state.acceleration, amax, jmax, h);
    if (state.saturation_direction == 0 && state.velocity > 0.0 &&
        stop_positive >= std::max(0.0, xmax - state.position))
    {
        state.saturation_direction = 1;
    }
    else if (state.saturation_direction == 0 && state.velocity < 0.0 &&
             stop_negative >= std::max(0.0, xmax + state.position))
    {
        state.saturation_direction = -1;
    }
    if (state.saturation_direction != 0)
    {
        // Ignore persistent outward force while latched and dissipate velocity.
        // This asymptotically stops without driving through zero into a reversal.
        desired_acceleration = std::clamp(
            -damping * state.velocity / mass, -amax, amax);
    }

    const double max_da = jmax * h;
    state.acceleration += std::clamp(
        desired_acceleration - state.acceleration, -max_da, max_da);
    state.acceleration = std::clamp(state.acceleration, -amax, amax);

    const double next_velocity = state.velocity + state.acceleration * h;
    // The predictive envelopes should make these clamps numerical-only. If an
    // impossible initial state is supplied, prefer a bounded command.
    state.velocity = std::clamp(next_velocity, -vmax, vmax);
    state.position += state.velocity * h;
    state.position = std::clamp(state.position, -xmax, xmax);
    return state;
}

} // namespace ocs2::mobile_manipulator
