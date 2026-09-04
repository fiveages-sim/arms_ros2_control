#pragma once

#include <algorithm>
#include <cmath>

namespace arms_controller_common
{
/**
 * @brief Jerk-limited online position reference for one Cartesian axis.
 *
 * The generator owns a reference position, velocity and acceleration.  Each
 * call advances that reference toward a target position with zero terminal
 * velocity and acceleration while respecting velocity, acceleration and jerk
 * limits.  A stopping-velocity envelope starts braking before the target
 * instead of waiting for delayed position feedback to cross it.
 *
 * This class is intentionally independent of ROS and allocation-free in
 * update(), so one instance can be kept for every position-controlled axis in
 * a real-time controller.
 */
class OnlinePositionReference1D
{
public:
    struct State
    {
        double position{0.0};
        double velocity{0.0};
        double acceleration{0.0};
    };

    struct Limits
    {
        double max_velocity{0.05};
        double max_acceleration{0.10};
        double max_jerk{0.50};
        double min_dt{1.0e-4};
        double max_dt{5.0e-2};
    };

    OnlinePositionReference1D() = default;

    explicit OnlinePositionReference1D(const Limits& limits)
    {
        setLimits(limits);
    }

    /**
     * @return true when all supplied limits are finite and strictly positive.
     * Invalid limits are rejected and the previous limits remain active.
     */
    bool setLimits(const Limits& limits)
    {
        if (!isValidLimits(limits))
        {
            return false;
        }
        limits_ = limits;
        limits_.max_dt = std::max(limits_.max_dt, limits_.min_dt);
        state_.velocity = std::clamp(
            state_.velocity, -limits_.max_velocity, limits_.max_velocity);
        state_.acceleration = std::clamp(
            state_.acceleration, -limits_.max_acceleration,
            limits_.max_acceleration);
        return true;
    }

    const Limits& limits() const noexcept
    {
        return limits_;
    }

    /**
     * @brief Initialize the online reference from a known controller state.
     *
     * Velocity and acceleration are clamped to the configured limits. Invalid
     * state values reset the generator to a finite zero state and return false.
     */
    bool reset(double position, double velocity = 0.0, double acceleration = 0.0)
    {
        if (!std::isfinite(position) || !std::isfinite(velocity) ||
            !std::isfinite(acceleration))
        {
            state_ = {};
            target_position_ = 0.0;
            return false;
        }

        state_.position = position;
        state_.velocity = std::clamp(
            velocity, -limits_.max_velocity, limits_.max_velocity);
        state_.acceleration = std::clamp(
            acceleration, -limits_.max_acceleration, limits_.max_acceleration);
        target_position_ = position;
        return true;
    }

    const State& state() const noexcept
    {
        return state_;
    }

    double targetPosition() const noexcept
    {
        return target_position_;
    }

    double lastDt() const noexcept
    {
        return last_dt_;
    }

    /**
     * @brief Advance the reference by one bounded controller period.
     *
     * A non-finite target, non-finite period, or non-positive period leaves the
     * state unchanged. Positive periods are clamped to [min_dt, max_dt].
     */
    State update(double target_position, double dt)
    {
        if (!std::isfinite(target_position) || !std::isfinite(dt) || dt <= 0.0)
        {
            return state_;
        }
        if (!isFinite(state_))
        {
            reset(std::isfinite(target_position) ? target_position : 0.0);
            return state_;
        }

        const double h = std::clamp(dt, limits_.min_dt, limits_.max_dt);
        last_dt_ = h;

        target_position_ = target_position;

        const State previous = state_;
        const double error = target_position - state_.position;

        // A position-to-velocity envelope makes the reference slow down before
        // reaching the target. Its bandwidth is derived from the configured
        // dynamic limits; it is deliberately below both the acceleration and
        // jerk boundaries so online retargeting remains well damped.
        const double position_bandwidth = 0.5 * std::min(
            limits_.max_acceleration / limits_.max_velocity,
            std::sqrt(limits_.max_jerk / limits_.max_velocity));
        double desired_velocity = std::clamp(
            position_bandwidth * error,
            -limits_.max_velocity, limits_.max_velocity);

        // The square-root envelope is the acceleration-limited stopping speed.
        // Combining it with the linear terminal envelope provides early
        // braking at cruise speed and a non-oscillatory approach near zero.
        const double stopping_velocity = std::sqrt(
            2.0 * limits_.max_acceleration * std::abs(error));
        desired_velocity = std::copysign(
            std::min(std::abs(desired_velocity), stopping_velocity), error);

        const double velocity_bandwidth = 0.5 *
            limits_.max_jerk / limits_.max_acceleration;
        double desired_acceleration = std::clamp(
            velocity_bandwidth * (desired_velocity - state_.velocity),
            -limits_.max_acceleration, limits_.max_acceleration);

        // Explicitly reserve the distance needed to remove the current
        // velocity with bounded jerk. The extra acceleration-unwind interval
        // is conservative by design: stopping a little early is preferable to
        // crossing the target and asking the delayed joint loop to reverse.
        const double direction = error >= 0.0 ? 1.0 : -1.0;
        const double directed_velocity = direction * state_.velocity;
        const double directed_acceleration = direction * state_.acceleration;
        if (directed_velocity > 0.0)
        {
            const double jerk = limits_.max_jerk;
            const double acceleration = limits_.max_acceleration;
            const double ramp_to_stop = std::min(
                (directed_acceleration + std::sqrt(std::max(
                    0.0, directed_acceleration * directed_acceleration +
                             2.0 * jerk * directed_velocity))) / jerk,
                std::max(0.0, directed_acceleration + acceleration) / jerk);
            double stop_distance =
                directed_velocity * ramp_to_stop +
                0.5 * directed_acceleration * ramp_to_stop * ramp_to_stop -
                jerk * ramp_to_stop * ramp_to_stop * ramp_to_stop / 6.0;
            const double velocity_after_ramp = std::max(
                0.0, directed_velocity +
                         directed_acceleration * ramp_to_stop -
                         0.5 * jerk * ramp_to_stop * ramp_to_stop);
            stop_distance += velocity_after_ramp * velocity_after_ramp /
                             (2.0 * acceleration);
            stop_distance += directed_velocity * acceleration / jerk;
            if (std::abs(error) <= stop_distance + directed_velocity * h)
                desired_acceleration = -direction * acceleration;
        }
        const double acceleration_delta = std::clamp(
            desired_acceleration - state_.acceleration,
            -limits_.max_jerk * h, limits_.max_jerk * h);
        state_.acceleration += acceleration_delta;
        state_.velocity +=
            0.5 * (previous.acceleration + state_.acceleration) * h;
        state_.velocity = std::clamp(
            state_.velocity, -limits_.max_velocity, limits_.max_velocity);
        state_.position +=
            0.5 * (previous.velocity + state_.velocity) * h;

        if (!isFinite(state_))
        {
            state_ = {target_position, 0.0, 0.0};
        }
        state_.velocity = std::clamp(
            state_.velocity, -limits_.max_velocity, limits_.max_velocity);
        state_.acceleration = std::clamp(
            state_.acceleration, -limits_.max_acceleration,
            limits_.max_acceleration);

        return state_;
    }

    bool isSettled(
        double target_position, double position_tolerance = 1.0e-7,
        double velocity_tolerance = 1.0e-6,
        double acceleration_tolerance = 1.0e-5) const noexcept
    {
        return std::isfinite(target_position) &&
               std::abs(target_position - state_.position) <=
                   std::max(0.0, position_tolerance) &&
               std::abs(state_.velocity) <= std::max(0.0, velocity_tolerance) &&
               std::abs(state_.acceleration) <=
                   std::max(0.0, acceleration_tolerance);
    }

private:
    static bool isValidLimits(const Limits& limits) noexcept
    {
        return std::isfinite(limits.max_velocity) && limits.max_velocity > 0.0 &&
               std::isfinite(limits.max_acceleration) &&
               limits.max_acceleration > 0.0 &&
               std::isfinite(limits.max_jerk) && limits.max_jerk > 0.0 &&
               std::isfinite(limits.min_dt) && limits.min_dt > 0.0 &&
               std::isfinite(limits.max_dt) && limits.max_dt > 0.0;
    }

    static bool isFinite(const State& state) noexcept
    {
        return std::isfinite(state.position) && std::isfinite(state.velocity) &&
               std::isfinite(state.acceleration);
    }

    double terminalPositionTolerance(double dt) const noexcept
    {
        return std::max(
            2.0e-5, limits_.max_jerk * dt * dt * dt / 6.0);
    }

    double terminalVelocityTolerance(double dt) const noexcept
    {
        return std::max(2.0e-5, 0.5 * limits_.max_jerk * dt * dt);
    }

    double terminalAccelerationTolerance(double dt) const noexcept
    {
        return std::max(1.0e-5, 0.9 * limits_.max_jerk * dt);
    }

    Limits limits_{};
    State state_{};
    double target_position_{0.0};
    double last_dt_{0.0};
};
} // namespace arms_controller_common
