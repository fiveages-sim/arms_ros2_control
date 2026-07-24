#include "arms_controller_common/utils/OnlineTrajectoryFilter.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include <ruckig/ruckig.hpp>

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

        ruckig::Ruckig<1> otg(dt);
        ruckig::InputParameter<1> input;
        ruckig::OutputParameter<1> output;

        input.current_position = {position_};
        input.current_velocity = {velocity_};
        input.current_acceleration = {acceleration_};
        input.target_position = {target_position_};
        input.target_velocity = {target_velocity_};
        input.target_acceleration = {target_acceleration_};
        input.max_velocity = {limits_.max_velocity};
        input.max_acceleration = {limits_.max_acceleration};
        input.max_jerk = {limits_.max_jerk};
        input.min_velocity = std::array<double, 1>{limits_.min_velocity};
        input.min_acceleration = std::array<double, 1>{limits_.min_acceleration};
        input.duration_discretization = ruckig::DurationDiscretization::Discrete;

        const auto result = otg.update(input, output);
        if (result < 0)
        {
            // Fail closed on an invalid or numerically infeasible state. Keep
            // the last continuous command instead of emitting a discontinuity.
            velocity_ = 0.0;
            acceleration_ = 0.0;
            last_jerk_ = 0.0;
            return position_;
        }

        const double next_position = output.new_position[0];
        const double next_velocity = output.new_velocity[0];
        const double next_acceleration = output.new_acceleration[0];

        if (!std::isfinite(next_position) || !std::isfinite(next_velocity) ||
            !std::isfinite(next_acceleration))
        {
            velocity_ = 0.0;
            acceleration_ = 0.0;
            last_jerk_ = 0.0;
            return position_;
        }

        const double previous_acceleration = acceleration_;
        position_ = next_position;
        velocity_ = next_velocity;
        acceleration_ = next_acceleration;
        last_jerk_ = std::clamp(
            (next_acceleration - previous_acceleration) / dt,
            -limits_.max_jerk,
            limits_.max_jerk);
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
