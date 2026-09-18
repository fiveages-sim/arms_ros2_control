#include "arms_controller_common/utils/JointSpeedStopPlanner.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace arms_controller_common
{
    void JointSpeedStopPlanner::reset()
    {
        active_ = truncated_ = false;
        joints_.clear(); positions_.clear(); velocities_.clear(); accelerations_.clear();
        lower_.clear(); upper_.clear();
        synchronized_ = false;
        scalar_stop_.reset();
        initial_positions_.clear(); initial_velocities_.clear(); initial_accelerations_.clear();
        base_positions_.clear(); directions_.clear();
        transition_duration_ = transition_elapsed_ = release_duration_ = 0;
        error_.clear();
    }

    bool JointSpeedStopPlanner::init(
        const std::vector<double>& positions, const std::vector<double>& velocities,
        const std::vector<double>& accelerations, double vmax, double amax, double jmax,
        const std::vector<double>& lower, const std::vector<double>& upper, bool truncate)
    {
        reset();
        auto fail = [this](const std::string& reason) {
            reset(); error_ = reason; return false;
        };
        const size_t n = positions.size();
        if (!n || velocities.size() != n || accelerations.size() != n ||
            lower.size() != n || upper.size() != n ||
            !std::isfinite(vmax) || !std::isfinite(amax) || !std::isfinite(jmax) ||
            vmax <= 0 || amax <= 0 || jmax <= 0)
            return fail("invalid synchronized stop dimensions or limits");
        size_t reference = 0;
        double peak_velocity = 0, peak_acceleration = 0;
        for (size_t i = 0; i < n; ++i)
        {
            if (!std::isfinite(positions[i]) || !std::isfinite(velocities[i]) ||
                !std::isfinite(accelerations[i]) || std::isnan(lower[i]) || std::isnan(upper[i]) ||
                lower[i] > upper[i] || positions[i] < lower[i] || positions[i] > upper[i])
                return fail("invalid synchronized stop state at joint " + std::to_string(i));
            if (!truncate && (std::abs(velocities[i]) > vmax + 1e-7 ||
                              std::abs(accelerations[i]) > amax + 1e-7))
                return fail("initial state exceeds synchronized stop limits");
            if (std::abs(velocities[i]) > peak_velocity)
            {
                peak_velocity = std::abs(velocities[i]); reference = i;
            }
            peak_acceleration = std::max(peak_acceleration, std::abs(accelerations[i]));
        }
        initial_positions_ = positions; initial_velocities_ = velocities;
        initial_accelerations_ = accelerations;
        positions_ = positions; velocities_ = velocities; accelerations_ = accelerations;
        lower_ = lower; upper_ = upper;
        base_positions_ = positions;
        directions_.assign(n, 0.0);
        double scalar_velocity = peak_velocity, scalar_acceleration = 0;
        bool compatible = true;
        if (peak_velocity > 0)
        {
            for (size_t i = 0; i < n; ++i) directions_[i] = velocities[i]/peak_velocity;
            scalar_acceleration = accelerations[reference]/directions_[reference];
            for (size_t i = 0; i < n; ++i)
                if (std::abs(accelerations[i]-directions_[i]*scalar_acceleration) >
                    1e-8 * std::max(1.0, peak_acceleration)) compatible = false;
        }
        else if (peak_acceleration > 0)
        {
            // All joints start at rest, but nonzero acceleration still needs a
            // continuous stop. Its direction defines the common path.
            for (size_t i = 0; i < n; ++i) directions_[i] = accelerations[i]/peak_acceleration;
            scalar_acceleration = peak_acceleration;
        }

        if (!compatible)
        {
            // Arbitrary a cannot preserve v ratios and acceleration continuity
            // simultaneously. A shared release interval preserves any linear
            // joint constraint satisfied by both initial v and a.
            release_duration_ = transition_duration_ = peak_acceleration/jmax;
            auto q_at = [&](size_t i, double t) {
                return positions[i] + velocities[i]*t + .5*accelerations[i]*t*t -
                       accelerations[i]*t*t*t/(6*release_duration_);
            };
            for (size_t i = 0; i < n; ++i)
            {
                const double jerk = -accelerations[i]/release_duration_;
                std::vector<double> times{0, release_duration_};
                if (jerk != 0)
                {
                    const double d = accelerations[i]*accelerations[i]-2*jerk*velocities[i];
                    if (d >= 0)
                        for (double t : {(-accelerations[i]-std::sqrt(d))/jerk,
                                         (-accelerations[i]+std::sqrt(d))/jerk})
                            if (t > 0 && t < release_duration_) times.push_back(t);
                }
                std::sort(times.begin(), times.end());
                for (size_t k = 1; k < times.size(); ++k)
                {
                    const double q = q_at(i, times[k]);
                    if (!std::isfinite(q)) return fail("nonfinite acceleration release");
                    if (q < lower[i] || q > upper[i])
                    {
                        if (!truncate) return fail("acceleration release exceeds position limits");
                        const bool increasing = q > upper[i];
                        const double bound = increasing ? upper[i] : lower[i];
                        double lo = times[k-1], hi = times[k];
                        for (int iteration = 0; iteration < 60; ++iteration)
                        {
                            const double mid = .5*(lo+hi);
                            if (increasing ? q_at(i,mid) >= bound : q_at(i,mid) <= bound) hi = mid;
                            else lo = mid;
                        }
                        transition_duration_ = std::min(transition_duration_, hi);
                        truncated_ = true;
                        break;
                    }
                }
            }
            peak_velocity = 0;
            for (size_t i = 0; i < n; ++i)
            {
                base_positions_[i] = std::clamp(q_at(i, transition_duration_), lower[i], upper[i]);
                const double v = velocities[i]+.5*accelerations[i]*release_duration_;
                if (!truncate && std::abs(v) > vmax + 1e-7)
                    return fail("acceleration release exceeds velocity limit");
                directions_[i] = v;
                peak_velocity = std::max(peak_velocity, std::abs(v));
            }
            if (truncated_)
            {
                // First boundary crossing stops every joint at the same time.
                synchronized_ = active_ = true;
                return true;
            }
            for (double& direction : directions_)
                direction = peak_velocity > 0 ? direction/peak_velocity : 0;
            scalar_velocity = peak_velocity;
            scalar_acceleration = 0;
        }

        // Intersect each joint's bounds in the common displacement coordinate.
        double path_lower = -std::numeric_limits<double>::infinity();
        double path_upper = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < n; ++i)
        {
            if (directions_[i] == 0) continue;
            double lo = (lower[i]-base_positions_[i])/directions_[i];
            double hi = (upper[i]-base_positions_[i])/directions_[i];
            if (lo > hi) std::swap(lo,hi);
            path_lower = std::max(path_lower,lo);
            path_upper = std::min(path_upper,hi);
        }
        scalar_stop_ = std::make_unique<JointSpeedStopPlanner>();
        if (!scalar_stop_->initScalar({0}, {scalar_velocity}, {scalar_acceleration},
                                     vmax, amax, jmax, {path_lower}, {path_upper}, truncate))
        {
            const std::string reason = scalar_stop_->error();
            return fail("synchronized stop: " + reason);
        }
        truncated_ = scalar_stop_->wasTruncated();
        synchronized_ = active_ = true;
        return true;
    }

    bool JointSpeedStopPlanner::initScalar(
        const std::vector<double>& positions, const std::vector<double>& velocities,
        const std::vector<double>& accelerations, double vmax, double amax, double jmax,
        const std::vector<double>& lower, const std::vector<double>& upper, bool truncate)
    {
        reset();
        const auto n = positions.size();
        auto fail = [this](const std::string& reason) {
            reset(); error_ = reason; return false;
        };
        if (!n || velocities.size() != n || accelerations.size() != n ||
            lower.size() != n || upper.size() != n ||
            !std::isfinite(vmax) || !std::isfinite(amax) || !std::isfinite(jmax) ||
            vmax <= 0 || amax <= 0 || jmax <= 0)
            return fail("invalid stop dimensions or motion limits");
        positions_ = positions; velocities_ = velocities; accelerations_ = accelerations;
        lower_ = lower; upper_ = upper;
        joints_.resize(n);
        for (size_t i = 0; i < n; ++i)
        {
            double q = positions[i], v = velocities[i], a = accelerations[i];
            if (!std::isfinite(q) || !std::isfinite(v) || !std::isfinite(a) ||
                std::isnan(lower[i]) || std::isnan(upper[i]) || lower[i] > upper[i] ||
                q < lower[i] || q > upper[i])
                return fail("invalid initial state or position bounds at joint " + std::to_string(i));
            if (!truncate && (std::abs(v) > vmax + 1e-7 || std::abs(a) > amax + 1e-7))
                return fail("initial state exceeds motion limits at joint " + std::to_string(i));
            auto& joint = joints_[i];
            bool clipped = false, invalid = false;
            auto append = [&](double jerk, double duration) {
                if (clipped || invalid || duration <= 1e-14) return;
                if (!std::isfinite(duration) || duration < 0) { invalid = true; return; }
                Segment segment{q, v, a, jerk, duration};
                auto position = [&](double t) { return q + v*t + .5*a*t*t + jerk*t*t*t/6; };
                // Split at velocity zeros: each resulting position interval is monotone.
                std::vector<double> times{0, duration};
                if (jerk != 0)
                {
                    const double discriminant = a*a - 2*jerk*v;
                    if (discriminant >= 0)
                    {
                        const double root = std::sqrt(discriminant);
                        for (double t : {(-a-root)/jerk, (-a+root)/jerk})
                            if (t > 0 && t < duration) times.push_back(t);
                    }
                }
                else if (a != 0)
                {
                    const double t = -v/a;
                    if (t > 0 && t < duration) times.push_back(t);
                }
                std::sort(times.begin(), times.end());
                for (size_t k = 1; k < times.size(); ++k)
                {
                    const double end = position(times[k]);
                    if (!std::isfinite(end)) { invalid = true; return; }
                    if (end < lower[i] || end > upper[i])
                    {
                        // Small endpoint roundoff is handled separately from a real crossing.
                        const double bound = end < lower[i] ? lower[i] : upper[i];
                        if (!truncate && std::abs(end-bound) > 1e-10) { invalid = true; return; }
                        if (!truncate) continue;
                        double lo = times[k-1], hi = times[k];
                        const bool increasing = end > upper[i];
                        for (int iteration = 0; iteration < 60; ++iteration)
                        {
                            const double mid = .5*(lo+hi);
                            if (increasing ? position(mid) >= bound : position(mid) <= bound) hi = mid;
                            else lo = mid;
                        }
                        segment.duration = hi;
                        joint.segments.push_back(segment);
                        q = bound; v = a = 0;
                        clipped = truncated_ = true;
                        return;
                    }
                }
                // Velocity extrema occur at a(t)=0, as well as at segment endpoints.
                if (!truncate)
                {
                    auto check_velocity = [&](double t) {
                        return std::abs(v+a*t+.5*jerk*t*t) <= vmax + 1e-7;
                    };
                    if (!check_velocity(0) || !check_velocity(duration) ||
                        (jerk != 0 && -a/jerk > 0 && -a/jerk < duration && !check_velocity(-a/jerk)))
                    { invalid = true; return; }
                }
                joint.segments.push_back(segment);
                q = position(duration);
                v += a*duration + .5*jerk*duration*duration;
                a += jerk*duration;
            };
            // An existing linear command can already exceed the stop acceleration
            // limit. Recover at maximum jerk, never silently reset its initial a.
            if (std::abs(a) > amax)
                append(-std::copysign(jmax, a), (std::abs(a)-amax)/jmax);
            if (!clipped && !invalid)
            {
                // Switching surface: velocity remaining after bringing a to zero.
                const double residual = v + a*std::abs(a)/(2*jmax);
                const double direction = residual >= 0 ? 1.0 : -1.0;
                const double b = direction*a, w = direction*v;
                const double peak = std::sqrt(std::max(0.0, jmax*w + .5*b*b));
                if (peak <= amax)
                {
                    append(-direction*jmax, std::max(0.0, (b+peak)/jmax));
                    append(direction*jmax, peak/jmax);
                }
                else
                {
                    const double hold = (w + (b*b-2*amax*amax)/(2*jmax))/amax;
                    append(-direction*jmax, std::max(0.0, (b+amax)/jmax));
                    append(0, std::max(0.0, hold));
                    append(direction*jmax, amax/jmax);
                }
            }
            if (invalid || !std::isfinite(q) || !std::isfinite(v) || !std::isfinite(a) ||
                std::abs(v) > 1e-6 || std::abs(a) > 1e-6)
                return fail("no feasible bounded stop at joint " + std::to_string(i));
            joint.final_position = std::clamp(q, lower[i], upper[i]);
        }
        active_ = true;
        return true;
    }

    std::vector<double> JointSpeedStopPlanner::run(double dt)
    {
        if (!active_ || !std::isfinite(dt) || dt <= 0) return positions_;
        if (synchronized_)
        {
            if (transition_elapsed_ < transition_duration_)
            {
                const double step = std::min(dt, transition_duration_-transition_elapsed_);
                transition_elapsed_ += step;
                dt -= step;
                const double t = transition_elapsed_;
                for (size_t i = 0; i < positions_.size(); ++i)
                {
                    const double a = initial_accelerations_[i], j = -a/release_duration_;
                    positions_[i] = std::clamp(initial_positions_[i]+initial_velocities_[i]*t+
                                               .5*a*t*t+j*t*t*t/6, lower_[i], upper_[i]);
                    velocities_[i] = initial_velocities_[i]+a*t+.5*j*t*t;
                    accelerations_[i] = a+j*t;
                }
                if (transition_elapsed_ < transition_duration_) return positions_;
            }
            if (!scalar_stop_)
            {
                positions_ = base_positions_;
                std::fill(velocities_.begin(), velocities_.end(), 0.0);
                std::fill(accelerations_.begin(), accelerations_.end(), 0.0);
                return positions_;
            }
            const auto path = scalar_stop_->run(dt);
            for (size_t i = 0; i < positions_.size(); ++i)
            {
                positions_[i] = std::clamp(base_positions_[i]+directions_[i]*path[0], lower_[i], upper_[i]);
                velocities_[i] = directions_[i]*scalar_stop_->velocities()[0];
                accelerations_[i] = directions_[i]*scalar_stop_->accelerations()[0];
            }
            return positions_;
        }
        for (size_t i = 0; i < joints_.size(); ++i)
        {
            auto& joint = joints_[i];
            double remaining = dt;
            while (joint.index < joint.segments.size())
            {
                const auto& s = joint.segments[joint.index];
                const double step = std::min(remaining, s.duration-joint.elapsed);
                joint.elapsed += step;
                remaining -= step;
                const double t = joint.elapsed;
                // All true crossings were handled during planning; this clamp
                // only removes floating-point roundoff at a validated boundary.
                positions_[i] = std::clamp(s.q+s.v*t+.5*s.a*t*t+s.jerk*t*t*t/6, lower_[i], upper_[i]);
                velocities_[i] = s.v+s.a*t+.5*s.jerk*t*t;
                accelerations_[i] = s.a+s.jerk*t;
                if (joint.elapsed < s.duration) break;
                ++joint.index;
                joint.elapsed = 0;
                if (remaining <= 0) break;
            }
            if (joint.index == joint.segments.size())
            {
                positions_[i] = joint.final_position;
                velocities_[i] = accelerations_[i] = 0;
            }
        }
        return positions_;
    }

    bool JointSpeedStopPlanner::isMotionOver() const
    {
        if (synchronized_)
            return transition_elapsed_ >= transition_duration_ &&
                   (!scalar_stop_ || scalar_stop_->isMotionOver());
        return !active_ || std::all_of(joints_.begin(), joints_.end(),
            [](const Joint& joint) { return joint.index == joint.segments.size(); });
    }
}
