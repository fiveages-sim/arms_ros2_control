#include "arms_controller_common/utils/OnlineTrajectoryFilter.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace arms_controller_common
{
    namespace
    {
        constexpr double kEpsilon = 1.0e-9;
    }

    OnlineTrajectoryFilter::OnlineTrajectoryFilter(const Limits& limits)
    {
        configure(limits);
    }

    bool OnlineTrajectoryFilter::configure(const Limits& limits)
    {
        if (!std::isfinite(limits.min_velocity) || !std::isfinite(limits.max_velocity) ||
            !std::isfinite(limits.min_acceleration) || !std::isfinite(limits.max_acceleration) ||
            !std::isfinite(limits.max_jerk) || !std::isfinite(limits.tracking_frequency) ||
            !std::isfinite(limits.target_filter_alpha) ||
            limits.min_velocity >= limits.max_velocity ||
            limits.min_acceleration >= limits.max_acceleration || limits.max_jerk <= kEpsilon ||
            limits.tracking_frequency <= kEpsilon || limits.target_filter_alpha <= 0.0 ||
            limits.target_filter_alpha > 1.0)
        {
            configured_ = false;
            return false;
        }

        limits_ = limits;
        configured_ = true;
        velocity_ = std::clamp(velocity_, limits_.min_velocity, limits_.max_velocity);
        acceleration_ = std::clamp(
            acceleration_, limits_.min_acceleration, limits_.max_acceleration);
        return true;
    }

    void OnlineTrajectoryFilter::reset(double position, double velocity, double acceleration)
    {
        position_ = std::isfinite(position) ? position : 0.0;
        velocity_ = std::isfinite(velocity) ? velocity : 0.0;
        acceleration_ = std::isfinite(acceleration) ? acceleration : 0.0;
        if (configured_)
        {
            velocity_ = std::clamp(velocity_, limits_.min_velocity, limits_.max_velocity);
            acceleration_ = std::clamp(
                acceleration_, limits_.min_acceleration, limits_.max_acceleration);
        }
        target_position_ = position_;
        target_velocity_ = 0.0;
        target_acceleration_ = 0.0;
        requested_target_position_ = target_position_;
        requested_target_velocity_ = target_velocity_;
        requested_target_acceleration_ = target_acceleration_;
        last_jerk_ = 0.0;
    }

    void OnlineTrajectoryFilter::setTarget(double position, double velocity, double acceleration)
    {
        if (std::isfinite(position))
        {
            requested_target_position_ = position;
        }
        requested_target_velocity_ = std::isfinite(velocity) ? velocity : 0.0;
        requested_target_acceleration_ = std::isfinite(acceleration) ? acceleration : 0.0;
    }

    double OnlineTrajectoryFilter::calculateDesiredJerk() const
    {
        const double frequency = limits_.tracking_frequency;
        const double position_error = target_position_ - position_;
        const double velocity_error = target_velocity_ - velocity_;
        const double acceleration_error = target_acceleration_ - acceleration_;

        // q''' = w^3 e + 3 w^2 e' + 3 w e'' gives three identical
        // real closed-loop poles at -w for a stationary target: no overshoot
        // in the unsaturated terminal response.
        const double desired_jerk =
            frequency * frequency * frequency * position_error +
            3.0 * frequency * frequency * velocity_error +
            3.0 * frequency * acceleration_error;
        return std::isfinite(desired_jerk) ? desired_jerk : 0.0;
    }

    double OnlineTrajectoryFilter::enforceDiscreteLimits(double desired_jerk, double dt) const
    {
        double lower = -limits_.max_jerk;
        double upper = limits_.max_jerk;

        lower = std::max(lower, (limits_.min_acceleration - acceleration_) / dt);
        upper = std::min(upper, (limits_.max_acceleration - acceleration_) / dt);

        const double half_dt_squared = 0.5 * dt * dt;
        lower = std::max(
            lower,
            (limits_.min_velocity - velocity_ - acceleration_ * dt) / half_dt_squared);
        upper = std::min(
            upper,
            (limits_.max_velocity - velocity_ - acceleration_ * dt) / half_dt_squared);

        if (lower <= upper)
        {
            return std::clamp(desired_jerk, lower, upper);
        }

        // This is only reachable if the supplied initial state is already too
        // close to a limit to remain feasible in one sample. Prefer braking.
        return std::clamp(-acceleration_ / dt, -limits_.max_jerk, limits_.max_jerk);
    }

    double OnlineTrajectoryFilter::update(double dt)
    {
        if (!configured_ || !std::isfinite(dt) || dt <= kEpsilon)
        {
            last_jerk_ = 0.0;
            return position_;
        }

        const double alpha = limits_.target_filter_alpha;
        target_position_ += alpha * (requested_target_position_ - target_position_);
        target_velocity_ += alpha * (requested_target_velocity_ - target_velocity_);
        target_acceleration_ += alpha *
                                (requested_target_acceleration_ - target_acceleration_);

        const double jerk = enforceDiscreteLimits(calculateDesiredJerk(), dt);
        const double next_acceleration = std::clamp(
            acceleration_ + dt * jerk,
            limits_.min_acceleration,
            limits_.max_acceleration);
        const double next_velocity = std::clamp(
            velocity_ + dt * (acceleration_ + next_acceleration) / 2.0,
            limits_.min_velocity,
            limits_.max_velocity);
        const double next_position =
            position_ + dt * (velocity_ + next_velocity) / 2.0;

        if (!std::isfinite(next_position) || !std::isfinite(next_velocity) ||
            !std::isfinite(next_acceleration))
        {
            velocity_ = 0.0;
            acceleration_ = 0.0;
            last_jerk_ = 0.0;
            return position_;
        }

        position_ = next_position;
        velocity_ = next_velocity;
        acceleration_ = next_acceleration;
        last_jerk_ = jerk;
        return position_;
    }

    bool OnlineTrajectoryFilter::isSettled(double position_tolerance,
                                            double velocity_tolerance,
                                            double acceleration_tolerance) const
    {
        return std::abs(requested_target_position_ - target_position_) <=
                   std::abs(position_tolerance) &&
               std::abs(position_ - target_position_) <= std::abs(position_tolerance) &&
               std::abs(velocity_ - target_velocity_) <= std::abs(velocity_tolerance) &&
               std::abs(acceleration_ - target_acceleration_) <=
                   std::abs(acceleration_tolerance);
    }
}  // namespace arms_controller_common
