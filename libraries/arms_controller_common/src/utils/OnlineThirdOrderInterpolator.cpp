#include "arms_controller_common/utils/OnlineThirdOrderInterpolator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace arms_controller_common
{
namespace
{
constexpr double kEpsilon = 1.0e-9;

bool finiteVector(const std::vector<double>& values)
{
    return std::all_of(values.begin(), values.end(),
                       [](double value) { return std::isfinite(value); });
}
}  // namespace

OnlineThirdOrderInterpolator::OnlineThirdOrderInterpolator(OnlineThirdOrderConfig config)
    : config_(std::move(config))
{
    validateConfig();
}

void OnlineThirdOrderInterpolator::validateConfig() const
{
    const std::size_t n = config_.tracking_frequency.size();
    if (n == 0)
    {
        throw std::invalid_argument("online3 requires at least one joint");
    }

    const auto valid_size = [n](const std::vector<double>& values)
    {
        return values.size() == n && finiteVector(values);
    };
    if (!valid_size(config_.max_velocity) || !valid_size(config_.min_velocity) ||
        !valid_size(config_.max_acceleration) || !valid_size(config_.min_acceleration) ||
        !valid_size(config_.max_jerk) || !valid_size(config_.min_jerk) ||
        !valid_size(config_.max_input_velocity) ||
        !valid_size(config_.spike_margin) || !valid_size(config_.spike_cluster_tolerance) ||
        !valid_size(config_.deadband) || !valid_size(config_.position_tolerance) ||
        !valid_size(config_.velocity_tolerance) || !valid_size(config_.acceleration_tolerance))
    {
        throw std::invalid_argument("online3 parameter vector size mismatch or non-finite value");
    }

    for (std::size_t i = 0; i < n; ++i)
    {
        if (!std::isfinite(config_.tracking_frequency[i]) || config_.tracking_frequency[i] <= 0.0 ||
            config_.max_velocity[i] <= 0.0 || config_.min_velocity[i] >= 0.0 ||
            config_.max_acceleration[i] <= 0.0 || config_.min_acceleration[i] >= 0.0 ||
            config_.max_jerk[i] <= 0.0 || config_.min_jerk[i] >= 0.0 ||
            config_.max_input_velocity[i] <= 0.0 ||
            config_.spike_margin[i] < 0.0 || config_.spike_cluster_tolerance[i] < 0.0 ||
            config_.deadband[i] < 0.0 || config_.position_tolerance[i] < 0.0 ||
            config_.velocity_tolerance[i] < 0.0 || config_.acceleration_tolerance[i] < 0.0)
        {
            throw std::invalid_argument("invalid online3 per-joint limit");
        }
    }

    if (!std::isfinite(config_.alpha) || config_.alpha <= 0.0 || config_.alpha > 1.0 ||
        !std::isfinite(config_.beta) || config_.beta < 0.0 || config_.beta > 1.0 ||
        !std::isfinite(config_.input_timeout) || config_.input_timeout <= 0.0 ||
        !std::isfinite(config_.decision_timeout) || config_.decision_timeout <= 0.0 ||
        !std::isfinite(config_.max_integrator_dt) || config_.max_integrator_dt <= 0.0)
    {
        throw std::invalid_argument("invalid online3 scalar parameter");
    }
}

void OnlineThirdOrderInterpolator::reset(const std::vector<double>& position,
                                         const std::vector<double>& velocity,
                                         const std::vector<double>& acceleration,
                                         double stamp_seconds)
{
    const std::size_t n = config_.tracking_frequency.size();
    if (position.size() != n || !finiteVector(position) ||
        (!velocity.empty() && (velocity.size() != n || !finiteVector(velocity))) ||
        (!acceleration.empty() && (acceleration.size() != n || !finiteVector(acceleration))))
    {
        throw std::invalid_argument("invalid online3 reset state");
    }

    position_ = position;
    velocity_.assign(n, 0.0);
    acceleration_.assign(n, 0.0);
    if (!velocity.empty())
    {
        for (std::size_t i = 0; i < n; ++i)
        {
            velocity_[i] = std::clamp(velocity[i], config_.min_velocity[i], config_.max_velocity[i]);
        }
    }
    if (!acceleration.empty())
    {
        for (std::size_t i = 0; i < n; ++i)
        {
            acceleration_[i] = std::clamp(
                acceleration[i], config_.min_acceleration[i], config_.max_acceleration[i]);
        }
    }

    accepted_target_ = position;
    estimate_position_ = position;
    estimate_velocity_.assign(n, 0.0);
    committed_frame_ = TargetFrame{position, stamp_seconds};
    pending_frame_.reset();
    last_measurement_stamp_ = stamp_seconds;
    last_accepted_stamp_ = stamp_seconds;
    last_valid_input_stamp_ = stamp_seconds;
    initialized_ = true;
    has_target_ = false;
}

bool OnlineThirdOrderInterpolator::vectorNear(
    const std::vector<double>& lhs, const std::vector<double>& rhs,
    const std::vector<double>& tolerance) const
{
    if (lhs.size() != rhs.size() || lhs.size() != tolerance.size())
    {
        return false;
    }
    for (std::size_t i = 0; i < lhs.size(); ++i)
    {
        if (std::abs(lhs[i] - rhs[i]) > tolerance[i])
        {
            return false;
        }
    }
    return true;
}

void OnlineThirdOrderInterpolator::acceptMeasurement(
    const std::vector<double>& measurement, double stamp_seconds)
{
    double dt = stamp_seconds - last_accepted_stamp_;
    if (!has_target_ || !std::isfinite(dt) || dt <= kEpsilon)
    {
        estimate_position_ = measurement;
        std::fill(estimate_velocity_.begin(), estimate_velocity_.end(), 0.0);
    }
    else
    {
        for (std::size_t i = 0; i < measurement.size(); ++i)
        {
            const double residual = measurement[i] - estimate_position_[i];
            estimate_position_[i] += config_.alpha * residual;
            estimate_velocity_[i] += config_.beta * residual / dt;
            estimate_velocity_[i] = std::clamp(
                estimate_velocity_[i], -config_.max_input_velocity[i], config_.max_input_velocity[i]);
        }
    }
    accepted_target_ = measurement;
    last_accepted_stamp_ = stamp_seconds;
    has_target_ = true;
}

OnlineThirdOrderInterpolator::TargetResult OnlineThirdOrderInterpolator::pushTarget(
    const std::vector<double>& raw_target, double stamp_seconds)
{
    if (!initialized_ || raw_target.size() != position_.size() ||
        !finiteVector(raw_target) || !std::isfinite(stamp_seconds))
    {
        return TargetResult::INVALID;
    }

    if (stamp_seconds + kEpsilon < last_measurement_stamp_)
    {
        return TargetResult::INVALID;
    }
    last_measurement_stamp_ = stamp_seconds;
    last_valid_input_stamp_ = stamp_seconds;
    const TargetFrame next{raw_target, stamp_seconds};
    if (!pending_frame_)
    {
        pending_frame_ = next;
        return TargetResult::PENDING_SPIKE;
    }

    const bool spike = isMiddleFrameSpike(committed_frame_, *pending_frame_, next);
    if (!spike)
    {
        commitFrame(*pending_frame_, false);
    }
    pending_frame_ = next;
    return spike ? TargetResult::PENDING_SPIKE : TargetResult::ACCEPTED;
}

bool OnlineThirdOrderInterpolator::isMiddleFrameSpike(
    const TargetFrame& previous, const TargetFrame& middle, const TargetFrame& next) const
{
    const double previous_to_middle_dt = std::max(kEpsilon, middle.stamp - previous.stamp);
    const double middle_to_next_dt = std::max(kEpsilon, next.stamp - middle.stamp);
    const double previous_to_next_dt = std::max(kEpsilon, next.stamp - previous.stamp);

    bool endpoints_near = true;
    bool middle_is_far = false;
    for (std::size_t i = 0; i < middle.position.size(); ++i)
    {
        const double endpoint_tolerance = config_.max_input_velocity[i] * previous_to_next_dt +
                                          config_.spike_cluster_tolerance[i];
        endpoints_near = endpoints_near &&
            std::abs(next.position[i] - previous.position[i]) <= endpoint_tolerance;

        const double from_previous = config_.max_input_velocity[i] * previous_to_middle_dt +
                                     config_.spike_margin[i];
        const double from_next = config_.max_input_velocity[i] * middle_to_next_dt +
                                 config_.spike_margin[i];
        middle_is_far = middle_is_far ||
            (std::abs(middle.position[i] - previous.position[i]) > from_previous &&
             std::abs(middle.position[i] - next.position[i]) > from_next);
    }
    return endpoints_near && middle_is_far;
}

void OnlineThirdOrderInterpolator::commitFrame(const TargetFrame& frame, bool stationary)
{
    committed_frame_ = frame;
    if (has_target_ && vectorNear(frame.position, accepted_target_, config_.deadband))
    {
        acceptMeasurement(accepted_target_, frame.stamp);
    }
    else
    {
        acceptMeasurement(frame.position, frame.stamp);
    }
    if (stationary)
    {
        estimate_position_ = accepted_target_;
        std::fill(estimate_velocity_.begin(), estimate_velocity_.end(), 0.0);
    }
}

void OnlineThirdOrderInterpolator::flushPendingTarget(double now_seconds)
{
    if (pending_frame_ && now_seconds - pending_frame_->stamp >= config_.decision_timeout)
    {
        commitFrame(*pending_frame_, true);
        pending_frame_.reset();
    }
}

bool OnlineThirdOrderInterpolator::targetTimedOut(double now_seconds) const
{
    return has_target_ && std::isfinite(now_seconds) &&
           now_seconds - last_valid_input_stamp_ > config_.input_timeout;
}

void OnlineThirdOrderInterpolator::integrateStep(double dt, double now_seconds)
{
    const bool timed_out = targetTimedOut(now_seconds);
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;

    for (std::size_t i = 0; i < position_.size(); ++i)
    {
        double target_velocity = timed_out ? 0.0 : estimate_velocity_[i];
        if (!timed_out)
        {
            estimate_position_[i] += target_velocity * dt;
        }
        else
        {
            estimate_velocity_[i] = 0.0;
        }

        const double frequency = config_.tracking_frequency[i];
        const double position_error = estimate_position_[i] - position_[i];
        const double velocity_error = target_velocity - velocity_[i];
        const double desired_jerk =
            frequency * frequency * frequency * position_error +
            3.0 * frequency * frequency * velocity_error -
            3.0 * frequency * acceleration_[i];

        double lower_jerk = config_.min_jerk[i];
        double upper_jerk = config_.max_jerk[i];
        lower_jerk = std::max(
            lower_jerk, (config_.min_acceleration[i] - acceleration_[i]) / dt);
        upper_jerk = std::min(
            upper_jerk, (config_.max_acceleration[i] - acceleration_[i]) / dt);
        lower_jerk = std::max(
            lower_jerk,
            2.0 * (config_.min_velocity[i] - velocity_[i] - acceleration_[i] * dt) / dt2);
        upper_jerk = std::min(
            upper_jerk,
            2.0 * (config_.max_velocity[i] - velocity_[i] - acceleration_[i] * dt) / dt2);

        double jerk = 0.0;
        if (lower_jerk <= upper_jerk)
        {
            jerk = std::clamp(desired_jerk, lower_jerk, upper_jerk);
        }
        else
        {
            jerk = std::clamp(-acceleration_[i] / dt,
                              config_.min_jerk[i], config_.max_jerk[i]);
        }

        const double next_acceleration = std::clamp(
            acceleration_[i] + jerk * dt,
            config_.min_acceleration[i], config_.max_acceleration[i]);
        const double next_velocity = std::clamp(
            velocity_[i] + acceleration_[i] * dt + 0.5 * jerk * dt2,
            config_.min_velocity[i], config_.max_velocity[i]);
        const double next_position = position_[i] + velocity_[i] * dt +
            0.5 * acceleration_[i] * dt2 + jerk * dt3 / 6.0;

        if (std::isfinite(next_position))
        {
            position_[i] = next_position;
            velocity_[i] = next_velocity;
            acceleration_[i] = next_acceleration;
        }

        if (std::abs(estimate_position_[i] - position_[i]) <= config_.position_tolerance[i] &&
            std::abs(target_velocity) <= config_.velocity_tolerance[i] &&
            std::abs(velocity_[i]) <= config_.velocity_tolerance[i] &&
            std::abs(acceleration_[i]) <= config_.acceleration_tolerance[i])
        {
            position_[i] = estimate_position_[i];
            velocity_[i] = target_velocity;
            acceleration_[i] = 0.0;
        }
    }
}

std::vector<double> OnlineThirdOrderInterpolator::update(double dt, double now_seconds)
{
    if (!initialized_ || !std::isfinite(dt) || dt <= kEpsilon)
    {
        return position_;
    }
    flushPendingTarget(now_seconds);
    if (!has_target_)
    {
        return position_;
    }

    double remaining = dt;
    while (remaining > kEpsilon)
    {
        const double step = std::min(remaining, config_.max_integrator_dt);
        integrateStep(step, now_seconds - remaining + step);
        remaining -= step;
    }
    return position_;
}

}  // namespace arms_controller_common
