//
// Joint Trajectory Manager Implementation
//
#include "arms_controller_common/utils/JointTrajectoryManager.h"
#include <algorithm>
#include <cmath>
#include <utility>
#include <stdexcept>

namespace
{
    bool areJointPositionsSame(
        const std::vector<double>& start_pos,
        const std::vector<double>& target_pos,
        double epsilon = 1.0e-9)
    {
        if (start_pos.size() != target_pos.size())
        {
            return false;
        }

        for (size_t i = 0; i < start_pos.size(); ++i)
        {
            if (std::abs(start_pos[i] - target_pos[i]) > epsilon)
            {
                return false;
            }
        }

        return true;
    }
}

namespace arms_controller_common
{
    JointTrajectoryManager::JointTrajectoryManager(rclcpp::Logger logger)
        : logger_(std::move(logger))
    {
    }

    bool JointTrajectoryManager::isDoublesAvailable()
    {
#ifdef HAS_LINA_PLANNING
        return true;
#else
        return false;
#endif
    }

    JointTrajectoryManager::TimedPoint JointTrajectoryManager::HeadTimedCandidate::evaluateAt(double t) const
    {
        auto result = base->evaluateAt(t);
        const auto apply = [&](const auto& axis, size_t index, double delay) {
            if (!axis) return;
            auto point = axis->evaluateAt(std::max(0.0, t-delay));
            if (t < delay)
            {
                point.velocities[0] = 0.0;
                point.accelerations[0] = 0.0;
            }
            result.positions[index] = point.positions[0];
            result.velocities[index] = point.velocities[0];
            result.accelerations[index] = point.accelerations[0];
        };
        apply(yaw, indices.yaw, yaw_start_delay);
        apply(pitch, indices.pitch, pitch_start_delay);
        return result;
    }

    double JointTrajectoryManager::HeadTimedCandidate::duration() const
    {
        return std::max({base->singleNodeDuration(), yaw ? yaw_start_delay + yaw->singleNodeDuration() : 0.0,
                         pitch ? pitch_start_delay + pitch->singleNodeDuration() : 0.0});
    }

    bool JointTrajectoryManager::buildHeadTimedCandidate(
        const std::vector<double>& start,
        const std::vector<double>& target,
        HeadJointIndices indices,
        const HeadCouplingLimits& limits,
        bool allow_yaw,
        bool allow_pitch,
        const HeadCandidateOptions& options,
        std::unique_ptr<HeadTimedCandidate>& candidate,
        std::string& error) const
    {
        candidate.reset();
        error.clear();
        try
        {
            const auto require = [](bool ok, const char* message) {
                if (!ok) throw std::invalid_argument(message);
            };
            require(options.type == InterpolationType::DOUBLES || options.type == InterpolationType::LINEAR ||
                    options.type == InterpolationType::TANH, "Unsupported head timing interpolation");
            require(!start.empty() && start.size() == target.size() && indices.yaw != indices.pitch &&
                    indices.yaw < start.size() && indices.pitch < start.size(), "Invalid head timing dimensions");
            for (const auto* values : {&start, &target})
                for (double v : *values) require(std::isfinite(v), "Non-finite head timing position");
            for (double v : {options.duration, options.frequency, options.tanh_scale,
                             options.max_velocity, options.max_acceleration, options.max_jerk})
                require(std::isfinite(v) && v > 0.0, "Invalid head timing options");
            const HeadPose from{start[indices.yaw], start[indices.pitch]};
            const HeadPose to{target[indices.yaw], target[indices.pitch]};
            require(limits.contains(from) && limits.contains(to), "Head timing endpoints outside limits");
            require((allow_yaw || from.yaw == to.yaw) && (allow_pitch || from.pitch == to.pitch),
                    "Head timing moves an unauthorized joint");

            // Each initialization enforces its own limits. Stationary vectors use basic holding.
            const auto plan = [&](const std::vector<double>& a, const std::vector<double>& b,
                                  double requested) {
                auto trajectory = std::make_unique<JointTrajectoryManager>(logger_);
                auto type = a == b ? InterpolationType::LINEAR : options.type;
                double duration = requested;
                if (type != InterpolationType::DOUBLES || !isDoublesAvailable())
                {
                    double distance = 0.0;
                    for (size_t i = 0; i < a.size(); ++i) distance = std::max(distance, std::abs(b[i]-a[i]));
                    const double scale = type == InterpolationType::TANH ? options.tanh_scale : 1.0;
                    duration = std::max(duration, scale * distance / options.max_velocity);
                }
                require(trajectory->initSingleNode(a, b, duration, type, options.frequency,
                    options.tanh_scale, true, options.max_velocity, options.max_acceleration,
                    options.max_jerk), "Failed to initialize head timing trajectory");
                require(options.auto_extend_duration || trajectory->singleNodeDuration() <= options.duration + 1e-9,
                        "Head timing requires duration extension but extension is disabled");
                return trajectory;
            };
            auto result = std::make_unique<HeadTimedCandidate>();
            result->indices = indices;
            result->base = plan(start, target, options.duration);
            if (limits.containsSegment(from, to))
            {
                candidate = std::move(result);
                return true;
            }
            require(allow_yaw && allow_pitch, "Head timing avoidance requires both joints");
            result->yaw = plan({from.yaw}, {to.yaw}, result->base->singleNodeDuration());
            result->pitch = plan({from.pitch}, {to.pitch}, result->base->singleNodeDuration());

            const double boundary = HeadCouplingConfig::yaw_boundary;
            const bool outward = std::abs(to.yaw) > boundary;
            const double side_yaw = outward ? to.yaw : from.yaw;
            const auto side_range = limits.pitchRange(side_yaw);
            // Advance outward clearance / postpone inward release by the existing yaw margin.
            // If a center endpoint lies inside the margin, use half its remaining clearance.
            const double center_yaw = outward ? std::abs(from.yaw) : std::abs(to.yaw);
            const double clearance = std::min(HeadCouplingConfig::yaw_margin,
                                             std::max(0.0, (boundary-center_yaw)/2.0));
            const double guarded_boundary = boundary - clearance;
            const double side_upper = outward ? std::max(to.pitch,
                side_range.upper - HeadCouplingConfig::pitch_margin) : side_range.upper;

            // Find the first crossing with a bracket. The upper/lower bounds let us
            // align safety events conservatively, including tanh's endpoint jump.
            const auto crossing = [](const JointTrajectoryManager& axis, double start,
                                     double target, double value) {
                double low = 0.0, high = axis.singleNodeDuration();
                if (start == value) return std::pair<double, double>{0.0, 0.0};
                const bool increasing = target > start;
                for (int i = 0; i < 60; ++i)
                {
                    const double mid = (low+high)/2.0;
                    const double position = axis.evaluateAt(mid).positions[0];
                    if (increasing ? position < value : position > value) low = mid;
                    else high = mid;
                }
                return std::pair<double, double>{low, high};
            };
            const double guarded_yaw = std::copysign(guarded_boundary, side_yaw);
            const auto yaw_crossing = crossing(*result->yaw, from.yaw, to.yaw, guarded_yaw);
            const auto pitch_crossing = crossing(*result->pitch, from.pitch, to.pitch, side_upper);
            // Preserve both curves: delay only the axis that would cross too early.
            // One control period supplies temporal clearance in addition to angle margins.
            if (outward)
            {
                result->yaw_start_delay = std::max(0.0, pitch_crossing.second - yaw_crossing.first)
                                          + 1.0/options.frequency;
            }
            else
            {
                result->pitch_start_delay = std::max(0.0, yaw_crossing.second - pitch_crossing.first)
                                            + 1.0/options.frequency;
            }
            require(options.auto_extend_duration || result->duration() <= options.duration + 1e-9,
                    "Head start delay exceeds requested duration while extension is disabled");

            // Monotone single-target interpolation: partition at every yaw boundary.
            // Pitch extrema on each interval are its endpoints, including one-sided limits.
            const auto safe = [&]() {
                const double end = result->duration();
                std::vector<double> times{0.0, end, result->yaw_start_delay,
                                          result->pitch_start_delay};
                for (double boundary : {-guarded_boundary, guarded_boundary})
                {
                    if (boundary < std::min(from.yaw, to.yaw) || boundary > std::max(from.yaw, to.yaw)) continue;
                    double low = 0.0, high = result->yaw->singleNodeDuration();
                    const bool increasing = to.yaw > from.yaw;
                    for (int i = 0; i < 60; ++i)
                    {
                        const double mid = (low + high) / 2.0;
                        const double y = result->yaw->evaluateAt(mid).positions[0];
                        if (increasing ? y < boundary : y > boundary) low = mid;
                        else high = mid;
                    }
                    times.push_back(result->yaw_start_delay + low);
                    times.push_back(result->yaw_start_delay + high);
                }
                std::sort(times.begin(), times.end());
                for (size_t i = 1; i < times.size(); ++i)
                {
                    const double a = times[i-1], b = times[i];
                    if (a == b) continue;
                    const double y = result->evaluateAt((a+b)/2.0).positions[indices.yaw];
                    auto range = limits.pitchRange(y);
                    if (std::abs(y) > guarded_boundary)
                    {
                        range.upper = std::min(range.upper, side_upper);
                    }
                    const double p = result->evaluateAt(a).positions[indices.pitch];
                    const double q = result->evaluateAt(b).positions[indices.pitch];
                    if (std::min(p,q) < range.lower || std::max(p,q) > range.upper) return false;
                }
                return true;
            };
            require(safe(), "Delayed head trajectories leave coupling range");
            candidate = std::move(result);
            return true;
        }
        catch (const std::exception& exception)
        {
            error = exception.what();
            return false;
        }
    }

    bool JointTrajectoryManager::initSingleNode(
        const std::vector<double>& start_pos,
        const std::vector<double>& target_pos,
        double duration,
        InterpolationType type,
        double controller_frequency,
        double tanh_scale,
        bool auto_extend_duration,
        double max_velocity,
        double max_acceleration,
        double max_jerk)
    {
        if (type == InterpolationType::SERVO)
        {
            RCLCPP_ERROR(logger_, "servo is supported only by MOVEJ target_joint_position streaming");
            return false;
        }
        if (!std::isfinite(max_velocity) || max_velocity <= 0 ||
            !std::isfinite(max_acceleration) || max_acceleration <= 0 ||
            !std::isfinite(max_jerk) || max_jerk <= 0)
        {
            RCLCPP_ERROR(logger_, "Invalid joint motion limits");
            return false;
        }
        if (!validateSingleNodeParams(start_pos, target_pos, duration, type))
        {
            return false;
        }

        // Reset state
        reset();

        active_max_velocity_ = max_velocity;
        active_max_acceleration_ = max_acceleration;
        active_max_jerk_ = max_jerk;
        // Store parameters
        start_pos_ = start_pos;
        target_pos_ = target_pos;
        duration_ = duration;
        interpolation_type_ = type;
        tanh_scale_ = tanh_scale;
        controller_frequency_ = controller_frequency;
        period_ = (controller_frequency_ > 0.0) ? (1.0 / controller_frequency_) : 0.01;

        // Initialize based on interpolation type
        if (type == InterpolationType::DOUBLES)
        {
            if (areJointPositionsSame(start_pos_, target_pos_))
            {
                RCLCPP_WARN(logger_,
                            "Skipping DOUBLES moveJ interpolation because start and target joint positions are identical. This is a no-op target, not a planner failure. joint_count=%zu, duration=%.3f",
                            start_pos_.size(), duration_);
                reset();
                return false;
            }

            if (!isDoublesAvailable())
            {
                (void)auto_extend_duration;
                (void)max_velocity;
                (void)max_acceleration;
                (void)max_jerk;
                RCLCPP_WARN(logger_,
                            "DOUBLES interpolation requested but lina_planning is not available. "
                            "Falling back to LINEAR interpolation.");
                interpolation_type_ = InterpolationType::LINEAR;
                mode_ = TrajectoryMode::SINGLE_NODE;
            }
            else
            {
#ifdef HAS_LINA_PLANNING
                const double vmax = std::max(max_velocity, 1e-6);
                const double amax = std::max(max_acceleration, 1e-6);
                const double jmax = std::max(max_jerk, 1e-6);

                auto applyJointLimits = [vmax, amax, jmax](planning::TrajectoryParameter& param, size_t n)
                {
                    for (size_t i = 0; i < n; ++i)
                    {
                        param.joint_max_vel(i) = vmax;
                        param.joint_max_acc(i) = amax;
                        param.joint_max_jerk(i) = jmax;
                    }
                };

                auto initDoublesPlanner = [&](double total_time, bool time_mode) -> bool
                {
                    try
                    {
                        movej_planner_ = std::make_unique<planning::moveJ>();

                        const size_t nr_of_joints = start_pos_.size();
                        planning::TrajectPoint start_joint_point(nr_of_joints);
                        planning::TrajectPoint end_joint_point(nr_of_joints);
                        planning::TrajectoryParameter traj_param = time_mode
                            ? planning::TrajectoryParameter(total_time, nr_of_joints)
                            : planning::TrajectoryParameter(nr_of_joints);

                        for (size_t i = 0; i < nr_of_joints; ++i)
                        {
                            start_joint_point.joint_pos(i) = start_pos_[i];
                            end_joint_point.joint_pos(i) = target_pos_[i];
                        }
                        applyJointLimits(traj_param, nr_of_joints);

                        planning::TrajectoryInitParameters movej_init_para(
                            start_joint_point, end_joint_point, traj_param, period_);

                        if (!movej_planner_->init(movej_init_para))
                        {
                            RCLCPP_ERROR(logger_, "Failed to initialize moveJ planner for DOUBLES interpolation");
                            return false;
                        }

                        evaluation_planner_ = std::make_unique<planning::moveJ>();
                        if (!evaluation_planner_->init(movej_init_para))
                        {
                            return false;
                        }
                        evaluation_planner_->setRealStartTime(0.0);
                        movej_planner_->setRealStartTime(0.0);
                        return true;
                    }
                    catch (const std::exception& e)
                    {
                        RCLCPP_ERROR(logger_, "Exception while initializing moveJ planner: %s", e.what());
                        return false;
                    }
                };

                try
                {
                    {

                        if (!initDoublesPlanner(duration_, false))
                        {
                            reset();
                            return false;
                        }
                        const double t_min = movej_planner_->getTotalTime();
                        if (!auto_extend_duration && duration_ + 1e-9 < t_min)
                        {
                            RCLCPP_ERROR(logger_, "DOUBLES duration violates motion limits; enable auto extension");
                            reset();
                            return false;
                        }
                        const double actual = std::max(duration_, t_min);
                        if (actual > duration_ + 1e-6)
                        {
                            static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
                            RCLCPP_INFO_THROTTLE(
                                logger_, steady_clock, 2000,
                                "MoveJ duration extended from %.3fs to %.3fs to satisfy limits",
                                duration_, actual);
                        }
                        // Constraint mode only yields t_min. Re-init in time mode so
                        // the planner runs for `actual` (requested duration when it
                        // is already long enough, otherwise the extended t_min).
                        if (!initDoublesPlanner(actual, true))
                        {
                            reset();
                            return false;
                        }
                        duration_ = actual;
                        planningTime_ = actual;
                    }
                    mode_ = TrajectoryMode::SINGLE_NODE;
                }
                catch (const std::exception& e)
                {
                    RCLCPP_ERROR(logger_, "Exception while initializing moveJ planner: %s", e.what());
                    reset();
                    return false;
                }
#else
                (void)auto_extend_duration;
                (void)max_velocity;
                (void)max_acceleration;
                (void)max_jerk;
#endif
            }
        }
        else
        {
            // Basic interpolation types (TANH, LINEAR, NONE)
            (void)auto_extend_duration;
            (void)max_velocity;
            (void)max_acceleration;
            (void)max_jerk;
            mode_ = TrajectoryMode::SINGLE_NODE;
        }

        initialized_ = true;
        // NONE is a direct target, not a timed motion. Mark it complete before
        // another HOME request can mistake home_duration for an active trajectory
        // and start stop-to-zero. getNextPoint() supplies the target with zero
        // command derivatives; the physical device handles its own motion.
        timed_evaluation_ready_ = true;
        completed_ = (type == InterpolationType::NONE);
        percent_ = completed_ ? 1.0 : 0.0;

        RCLCPP_DEBUG(logger_,
                     "Initialized single-node trajectory: %zu joints, duration=%.3f, type=%s",
                     start_pos_.size(), duration_, toString(interpolation_type_));

        return true;
    }

    bool JointTrajectoryManager::initMultiNode(
        const std::vector<std::vector<double>>& waypoints,
        const std::vector<double>& joint_blend_ratios,
        const std::vector<double>& durations,
        InterpolationType type,
        double controller_frequency,
        double tanh_scale)
    {
#ifndef HAS_LINA_PLANNING
        (void)joint_blend_ratios;
#endif

        if (type == InterpolationType::SERVO)
        {
            RCLCPP_ERROR(logger_, "servo does not support timed multi-node trajectories");
            return false;
        }
        if (!validateMultiNodeParams(waypoints, durations))
        {
            return false;
        }

        // Reset state
        reset();
        active_max_velocity_ = default_max_velocity_;
        active_max_acceleration_ = default_max_acceleration_;
        active_max_jerk_ = default_max_jerk_;


        // Store parameters
        waypoints_ = waypoints;
        interpolation_type_ = type;
        tanh_scale_ = tanh_scale;
        controller_frequency_ = controller_frequency;
        period_ = controller_frequency_ > 0.0 ? 1.0 / controller_frequency_ : 0.01;

        // Prepare segment durations
        size_t num_segments = waypoints.size() - 1;
        segment_durations_.clear();
        segment_durations_.reserve(num_segments);

        if (durations.empty())
        {
            // Use trajectory_duration_ divided by number of segments as default
            // This ensures consistent behavior when durations are not provided
            double default_segment_duration = (num_segments > 0)
                                                  ? (trajectory_duration_ / static_cast<double>(num_segments))
                                                  : trajectory_duration_;
            segment_durations_.assign(num_segments, default_segment_duration);
        }
        else if (durations.size() == num_segments)
        {
            segment_durations_ = durations;
        }
        else
        {
            RCLCPP_WARN(logger_,
                        "Duration vector size (%zu) doesn't match number of segments (%zu). "
                        "Using trajectory_duration_ / num_segments for all segments.",
                        durations.size(), num_segments);
            double default_segment_duration = (num_segments > 0)
                                                  ? (trajectory_duration_ / static_cast<double>(num_segments))
                                                  : trajectory_duration_;
            segment_durations_.assign(num_segments, default_segment_duration);
        }

        // Initialize based on interpolation type
        if (type == InterpolationType::DOUBLES)
        {
            if (!isDoublesAvailable())
            {
                RCLCPP_WARN(logger_,
                            "DOUBLES interpolation requested but lina_planning is not available. "
                            "Falling back to LINEAR interpolation.");
                interpolation_type_ = InterpolationType::LINEAR;
                mode_ = TrajectoryMode::MULTI_NODE_BASIC;
            }
            else
            {
#ifdef HAS_LINA_PLANNING
                // Initialize SmoothCurveOfMultiJointsUsingBlending for DOUBLES
                try
                {
                    multi_node_planner_ = std::make_unique<planning::SmoothCurveOfMultiJointsUsingBlending>();

                    size_t nr_of_joints = waypoints_[0].size();
                    std::vector<planning::TrajectPoint> trajectory_points;
                    size_t nr_of_waypoints = waypoints.size();
                    trajectory_points.reserve(nr_of_waypoints);

                    // Convert waypoints to TrajectPoint
                    for (size_t i = 0; i < nr_of_waypoints; i++)
                    {
                        const auto& waypoint = waypoints_[i];
                        planning::TrajectPoint point(nr_of_joints);
                        for (size_t j = 0; j < nr_of_joints; ++j)
                        {
                            point.joint_pos(j) = waypoint[j];
                        }
                        if (joint_blend_ratios.empty())
                        {
                            if (i == 0 || i == nr_of_waypoints - 1)
                            {
                                point.blend_tolerance_ratio_of_movej = 0.0;
                            }
                            else
                            {
                                point.blend_tolerance_ratio_of_movej = common_joint_blend_ratios;
                            }
                        }
                        else
                        {
                            point.blend_tolerance_ratio_of_movej = joint_blend_ratios[i];
                        }
                        RCLCPP_INFO(logger_, "%zu th point blend ratio is %.2f", i,
                                    point.blend_tolerance_ratio_of_movej);
                        trajectory_points.push_back(point);
                    }

                    // For DOUBLES mode, use automatic time calculation (time_mode = true)
                    // lina planning will automatically calculate segment durations based on
                    // trajectory_duration_ and joint_max_vel
                    // Note: trajectory_duration_ should be the TOTAL time for all segments
                    size_t num_segments = waypoints_.size() - 1;
                    RCLCPP_INFO(logger_,
                                "Initializing DOUBLES multi-node trajectory: %zu waypoints, %zu segments, "
                                "total_time=%.3f seconds (will be distributed proportionally)",
                                waypoints_.size(), num_segments, trajectory_duration_);

                    planning::TrajectoryParameter param(trajectory_duration_, nr_of_joints);
                    param.time_mode = true; // Enable automatic time calculation
                    for (size_t i = 0; i < nr_of_joints; ++i)
                    {
                        param.joint_max_vel(i) = active_max_velocity_;
                        param.joint_max_acc(i) = active_max_acceleration_;
                        param.joint_max_jerk(i) = active_max_jerk_;
                    }

                    // Use single parameter, lina planning will auto-calculate segment times
                    // The total_time in param will be distributed proportionally to all segments
                    planning::TrajectoryInitParameters init_params(
                        trajectory_points, param, period_);

                    if (!multi_node_planner_->init(init_params))
                    {
                        RCLCPP_ERROR(logger_,
                                     "Failed to initialize SmoothCurveOfMultiJointsUsingBlending for DOUBLES interpolation")
                        ;
                        reset();
                        return false;
                    }

                    mode_ = TrajectoryMode::MULTI_NODE_ADVANCED;
                }
                catch (const std::exception& e)
                {
                    RCLCPP_ERROR(logger_,
                                 "Exception while initializing multi-node planner: %s", e.what());
                    reset();
                    return false;
                }
#endif
            }
        }
        else
        {
            // Basic interpolation types (TANH, LINEAR, NONE)
            mode_ = TrajectoryMode::MULTI_NODE_BASIC;
        }

        // Initialize segment progress tracking for basic mode
        if (mode_ == TrajectoryMode::MULTI_NODE_BASIC)
        {
            segment_progress_.clear();
            segment_progress_.assign(num_segments, 0.0);
            current_segment_ = 0;
        }

        initialized_ = true;
        completed_ = false;

        RCLCPP_DEBUG(logger_,
                     "Initialized multi-node trajectory: %zu waypoints, %zu segments, type=%s",
                     waypoints_.size(), num_segments, toString(interpolation_type_));

        return true;
    }

    std::vector<double> JointTrajectoryManager::getNextPoint(double step_seconds)
    {
        last_velocities_.clear();
        last_accelerations_.clear();
        if (!initialized_)
        {
            // Use simple warning since we don't have a clock in this context
            static bool warned = false;
            if (!warned)
            {
                RCLCPP_WARN(logger_,
                            "getNextPoint() called but trajectory is not initialized");
                warned = true;
            }
            return std::vector<double>();
        }

        if (completed_)
        {
            const size_t n = mode_ == TrajectoryMode::SINGLE_NODE ? target_pos_.size()
                : (waypoints_.empty() ? 0 : waypoints_.back().size());
            last_velocities_.assign(n, 0.0);
            last_accelerations_.assign(n, 0.0);
            // Return final position
            if (mode_ == TrajectoryMode::SINGLE_NODE)
            {
                return target_pos_;
            }
            else if (!waypoints_.empty())
            {
                return waypoints_.back();
            }
            else
            {
                return std::vector<double>();
            }
        }

        std::vector<double> result;

        switch (mode_)
        {
        case TrajectoryMode::SINGLE_NODE:
            result = computeSingleNodePoint(step_seconds);
            break;
        case TrajectoryMode::MULTI_NODE_BASIC:
            result = computeMultiNodeBasic(step_seconds);
            break;
#ifdef HAS_LINA_PLANNING
        case TrajectoryMode::MULTI_NODE_ADVANCED:
            if (step_seconds > 0.0)
            {
                period_ = step_seconds;
            }
            result = computeMultiNodeAdvanced();
            break;
#endif
        default:
            RCLCPP_ERROR(logger_, "Invalid trajectory mode");
            return std::vector<double>();
        }

        // Check if trajectory is completed
        if (percent_ >= 1.0 || completed_)
        {
            completed_ = true;
        }

        return result;
    }

    bool JointTrajectoryManager::isCompleted() const
    {
        return completed_;
    }

    void JointTrajectoryManager::reset()
    {
        timed_evaluation_ready_ = false;
#ifdef HAS_LINA_PLANNING
        evaluation_planner_.reset();
#endif

        mode_ = TrajectoryMode::NONE;
        initialized_ = false;
        completed_ = false;

        // Clear trajectory data
        start_pos_.clear();
        target_pos_.clear();
        waypoints_.clear();
        segment_durations_.clear();
        segment_progress_.clear();

        // Reset progress tracking
        duration_ = 0.0;
        percent_ = 0.0;
        current_segment_ = 0;

        // Reset planners (but keep configuration parameters unchanged)
#ifdef HAS_LINA_PLANNING
        movej_planner_.reset();
        multi_node_planner_.reset();
#endif
    }

    double JointTrajectoryManager::getProgress() const
    {
        return percent_;
    }

    bool JointTrajectoryManager::isInitialized() const
    {
        return initialized_;
    }

    void JointTrajectoryManager::setTrajectoryDuration(double duration)
    {
        if (duration > 0.0)
        {
            trajectory_duration_ = duration;
        }
        else
        {
            RCLCPP_WARN(logger_,
                        "Invalid trajectory_duration: %.3f, must be positive. Keeping current value: %.3f",
                        duration, trajectory_duration_);
        }
    }

    double JointTrajectoryManager::getTrajectoryDuration() const
    {
        return trajectory_duration_;
    }

    void JointTrajectoryManager::setControllerFrequency(double controller_frequency)
    {
        if (controller_frequency > 0.0)
        {
            controller_frequency_ = controller_frequency;
            period_ = 1.0 / controller_frequency_;
        }
        else
        {
            RCLCPP_WARN(logger_,
                        "Invalid controller frequency: %.3f, keeping current value %.3f",
                        controller_frequency, controller_frequency_);
        }
    }

    void JointTrajectoryManager::setCommonJointBlendRatios(double blend_ratios)
    {
        common_joint_blend_ratios = std::clamp(blend_ratios, 0.0, 1.0);
    }

    void JointTrajectoryManager::setDefaultJointMotionLimits(
        double max_velocity, double max_acceleration, double max_jerk)
    {
        default_max_velocity_ = std::max(max_velocity, 1e-6);
        default_max_acceleration_ = std::max(max_acceleration, 1e-6);
        default_max_jerk_ = std::max(max_jerk, 1e-6);
    }

    double JointTrajectoryManager::singleNodeDuration() const
    {
        if (!initialized_ || !timed_evaluation_ready_ || !isSingleNode())
        {
            throw std::invalid_argument("Timed evaluation requires initSingleNode");
        }
        return interpolation_type_ == InterpolationType::NONE ? 0.0 : duration_;
    }

    JointTrajectoryManager::TimedPoint JointTrajectoryManager::evaluateAt(double elapsed_seconds) const
    {
        const double duration = singleNodeDuration();
        if (!std::isfinite(elapsed_seconds) || elapsed_seconds < 0.0)
        {
            throw std::invalid_argument("Trajectory time must be finite and nonnegative");
        }
        TimedPoint result{target_pos_, std::vector<double>(target_pos_.size(), 0.0),
                          std::vector<double>(target_pos_.size(), 0.0)};
        if (elapsed_seconds >= duration)
        {
            return result;
        }
#ifdef HAS_LINA_PLANNING
        if (interpolation_type_ == InterpolationType::DOUBLES)
        {
            const auto point = evaluation_planner_->calculatePointAtTInRealTime(elapsed_seconds);
            for (size_t i = 0; i < target_pos_.size(); ++i)
            {
                result.positions[i] = point.joint_pos(i);
                result.velocities[i] = point.joint_vel(i);
                result.accelerations[i] = point.joint_acc(i);
            }
            return result;
        }
#endif
        const double percent = elapsed_seconds / duration;
        const double phase = calculatePhase(interpolation_type_, percent);
        double velocity = 1.0 / duration;
        double acceleration = 0.0;
        if (interpolation_type_ == InterpolationType::TANH)
        {
            const double rate = (tanh_scale_ > 0.0 ? tanh_scale_ : 3.0) / duration;
            velocity = rate * (1.0 - phase * phase);
            acceleration = -2.0 * rate * rate * phase * (1.0 - phase * phase);
        }
        for (size_t i = 0; i < target_pos_.size(); ++i)
        {
            const double distance = target_pos_[i] - start_pos_[i];
            result.positions[i] = start_pos_[i] + phase * distance;
            result.velocities[i] = velocity * distance;
            result.accelerations[i] = acceleration * distance;
        }
        return result;
    }

    std::vector<double> JointTrajectoryManager::computeSingleNodePoint(double step_seconds)
    {
        std::vector<double> result;
        result.reserve(start_pos_.size());

        if (interpolation_type_ == InterpolationType::DOUBLES)
        {
#ifdef HAS_LINA_PLANNING
            if (movej_planner_)
            {
                if (std::isfinite(step_seconds) && step_seconds > 0.0)
                    movej_planner_->period = step_seconds;
                planning::TrajectPoint movej_point = movej_planner_->run();

                if (movej_planner_->isMotionOver())
                {
                    completed_ = true;
                    percent_ = 1.0;
                }
                else
                {
                    // Estimate progress from planner (approximate)
                    double total_time = movej_planner_->getTotalTime();
                    double current_time = movej_planner_->current_real_time;
                    percent_ = std::min(1.0, current_time / total_time);
                }

                for (size_t i = 0; i < static_cast<size_t>(movej_point.joint_pos.getJointSize()); ++i)
                {
                    result.push_back(movej_point.joint_pos(i));
                    last_velocities_.push_back(movej_point.joint_vel(i));
                    last_accelerations_.push_back(movej_point.joint_acc(i));
                }

                return result;
            }
            else
            {
                RCLCPP_ERROR(logger_, "moveJ planner not initialized for DOUBLES interpolation");
                return target_pos_; // Fallback to target
            }
#else
            // Should not reach here if validation is correct
            RCLCPP_ERROR(logger_, "DOUBLES interpolation not available");
            return target_pos_;
#endif
        }
        else
        {
            // Basic interpolation (TANH, LINEAR, NONE)
            updateProgress(step_seconds);
            double phase = calculatePhase(interpolation_type_, percent_);
            recordBasicDerivatives(percent_, duration_, start_pos_, target_pos_);

            for (size_t i = 0; i < start_pos_.size() && i < target_pos_.size(); ++i)
            {
                double interpolated_value = phase * target_pos_[i] + (1.0 - phase) * start_pos_[i];
                result.push_back(interpolated_value);
            }
        }

        return result;
    }

    std::vector<double> JointTrajectoryManager::computeMultiNodeBasic(double step_seconds)
    {
        // Check if we need to move to next segment
        if (current_segment_ < segment_progress_.size())
        {
            double segment_duration = segment_durations_[current_segment_];

            // Update progress for current segment
            double runtime_step = step_seconds > 0.0 ? step_seconds : period_;
            if (segment_duration > 0.0 && runtime_step > 0.0)
            {
                segment_progress_[current_segment_] += runtime_step;
                segment_progress_[current_segment_] = std::min(
                    segment_progress_[current_segment_], segment_duration);
            }
            else
            {
                segment_progress_[current_segment_] = segment_duration;
            }

            // Calculate phase for current segment
            double segment_percent = segment_duration > 0.0
                                         ? (segment_progress_[current_segment_] / segment_duration)
                                         : 1.0;
            segment_percent = std::clamp(segment_percent, 0.0, 1.0);

            double phase = calculatePhase(interpolation_type_, segment_percent);

            // Interpolate between current segment's start and end waypoints
            const auto& start_waypoint = waypoints_[current_segment_];
            const auto& end_waypoint = waypoints_[current_segment_ + 1];
            recordBasicDerivatives(segment_percent, segment_duration, start_waypoint, end_waypoint);

            std::vector<double> result;
            result.reserve(start_waypoint.size());

            for (size_t i = 0; i < start_waypoint.size() && i < end_waypoint.size(); ++i)
            {
                double interpolated_value = phase * end_waypoint[i] + (1.0 - phase) * start_waypoint[i];
                result.push_back(interpolated_value);
            }

            // Check if current segment is complete
            if (segment_percent >= 1.0)
            {
                // Move to next segment
                current_segment_++;

                if (current_segment_ >= segment_progress_.size())
                {
                    // All segments completed
                    completed_ = true;
                    percent_ = 1.0;
                }
                else
                {
                    // Calculate overall progress
                    double total_duration = 0.0;
                    double elapsed_duration = 0.0;

                    for (size_t i = 0; i < segment_durations_.size(); ++i)
                    {
                        total_duration += segment_durations_[i];
                        if (i < current_segment_)
                        {
                            elapsed_duration += segment_durations_[i];
                        }
                        else if (i == current_segment_)
                        {
                            elapsed_duration += segment_progress_[i];
                        }
                    }

                    percent_ = total_duration > 0.0 ? elapsed_duration / total_duration : 1.0;
                    percent_ = std::clamp(percent_, 0.0, 1.0);
                }
            }
            else
            {
                // Calculate overall progress
                double total_duration = 0.0;
                double elapsed_duration = 0.0;

                for (size_t i = 0; i < segment_durations_.size(); ++i)
                {
                    total_duration += segment_durations_[i];
                    if (i < current_segment_)
                    {
                        elapsed_duration += segment_durations_[i];
                    }
                    else if (i == current_segment_)
                    {
                        elapsed_duration += segment_progress_[i];
                    }
                }

                percent_ = total_duration > 0.0 ? elapsed_duration / total_duration : 1.0;
                percent_ = std::clamp(percent_, 0.0, 1.0);
            }

            return result;
        }
        else
        {
            // All segments completed, return final waypoint
            completed_ = true;
            percent_ = 1.0;
            return waypoints_.back();
        }
    }

#ifdef HAS_LINA_PLANNING
    std::vector<double> JointTrajectoryManager::computeMultiNodeAdvanced()
    {
        if (!multi_node_planner_)
        {
            RCLCPP_ERROR(logger_, "Multi-node planner not initialized for DOUBLES interpolation");
            return waypoints_.back(); // Fallback to final waypoint
        }

        planning::TrajectPoint traj_point = multi_node_planner_->run();

        if (multi_node_planner_->isMotionOver())
        {
            completed_ = true;
            percent_ = 1.0;
        }
        else
        {
            // Estimate progress from planner (approximate)
            double total_time = multi_node_planner_->getTotalTime();
            double current_time = multi_node_planner_->current_real_time;
            percent_ = std::min(1.0, current_time / total_time);
        }

        std::vector<double> result;
        size_t num_joints = static_cast<size_t>(traj_point.joint_pos.getJointSize());
        result.reserve(num_joints);

        for (size_t i = 0; i < num_joints; ++i)
        {
            result.push_back(traj_point.joint_pos(i));
            last_velocities_.push_back(traj_point.joint_vel(i));
            last_accelerations_.push_back(traj_point.joint_acc(i));
        }

        return result;
    }
#endif

    void JointTrajectoryManager::recordBasicDerivatives(
        double percent, double duration, const std::vector<double>& start,
        const std::vector<double>& target)
    {
        const size_t count = std::min(start.size(), target.size());
        last_velocities_.assign(count, 0.0);
        last_accelerations_.assign(count, 0.0);
        if (duration <= 0.0 || percent >= 1.0 || interpolation_type_ == InterpolationType::NONE)
            return;
        double phase_velocity = 1.0 / duration;
        double phase_acceleration = 0.0;
        if (interpolation_type_ == InterpolationType::TANH)
        {
            const double scale = tanh_scale_ > 0.0 ? tanh_scale_ : 3.0;
            const double phase = std::tanh(percent * scale);
            phase_velocity = scale * (1.0 - phase * phase) / duration;
            phase_acceleration = -2.0 * scale * phase * phase_velocity / duration;
        }
        for (size_t i = 0; i < count; ++i)
        {
            last_velocities_[i] = (target[i] - start[i]) * phase_velocity;
            last_accelerations_[i] = (target[i] - start[i]) * phase_acceleration;
        }
    }

    double JointTrajectoryManager::calculatePhase(InterpolationType type, double percent) const
    {
        if (type == InterpolationType::NONE)
        {
            return 1.0;
        }

        if (percent >= 1.0)
        {
            return 1.0;
        }

        double phase = 0.0;
        if (type == InterpolationType::LINEAR)
        {
            phase = percent;
        }
        else if (type == InterpolationType::TANH)
        {
            const double scale = tanh_scale_ > 0.0 ? tanh_scale_ : 3.0;
            phase = std::tanh(percent * scale);
        }
        else
        {
            // Default to linear for unknown types
            phase = percent;
        }

        return std::clamp(phase, 0.0, 1.0);
    }

    void JointTrajectoryManager::updateProgress(double step_seconds)
    {
        double runtime_step = step_seconds > 0.0 ? step_seconds : period_;
        if (duration_ <= 0.0 || runtime_step <= 0.0)
        {
            percent_ = 1.0; // Already normalized
            completed_ = true;
        }
        else
        {
            // percent_ stores normalized progress [0, 1]
            // Increment by the fraction of duration that one period represents
            double increment = runtime_step / duration_;
            percent_ += increment;
            percent_ = std::min(percent_, 1.0);

            if (percent_ >= 1.0)
            {
                completed_ = true;
                percent_ = 1.0; // Ensure exact 1.0
            }
        }
    }

    bool JointTrajectoryManager::useAdvancedMode(InterpolationType type) const
    {
        return (type == InterpolationType::DOUBLES) && isDoublesAvailable();
    }

    bool JointTrajectoryManager::validateSingleNodeParams(
        const std::vector<double>& start_pos,
        const std::vector<double>& target_pos,
        double duration,
        InterpolationType type) const
    {
        if (start_pos.empty())
        {
            RCLCPP_ERROR(logger_, "Start position vector is empty");
            return false;
        }

        if (target_pos.empty())
        {
            RCLCPP_ERROR(logger_, "Target position vector is empty");
            return false;
        }

        if (start_pos.size() != target_pos.size())
        {
            RCLCPP_ERROR(logger_,
                         "Start position size (%zu) doesn't match target position size (%zu)",
                         start_pos.size(), target_pos.size());
            return false;
        }

        if (duration < 0.0)
        {
            RCLCPP_ERROR(logger_, "Duration must be non-negative, got %.3f", duration);
            return false;
        }

        if (type == InterpolationType::DOUBLES && !isDoublesAvailable())
        {
            RCLCPP_WARN(logger_,
                        "DOUBLES interpolation requested but lina_planning is not available");
            // Don't fail validation, will fall back to LINEAR
        }

        return true;
    }

    bool JointTrajectoryManager::validateMultiNodeParams(
        const std::vector<std::vector<double>>& waypoints,
        const std::vector<double>& durations) const
    {
        if (waypoints.size() < 2)
        {
            RCLCPP_ERROR(logger_,
                         "Multi-node trajectory requires at least 2 waypoints, got %zu",
                         waypoints.size());
            return false;
        }

        // Check all waypoints have the same size
        size_t expected_size = waypoints[0].size();
        for (size_t i = 1; i < waypoints.size(); ++i)
        {
            if (waypoints[i].size() != expected_size)
            {
                RCLCPP_ERROR(logger_,
                             "Waypoint %zu has size %zu, but expected %zu",
                             i, waypoints[i].size(), expected_size);
                return false;
            }
        }

        // Check durations if provided
        if (!durations.empty())
        {
            size_t expected_num_segments = waypoints.size() - 1;
            if (durations.size() != expected_num_segments)
            {
                RCLCPP_WARN(logger_,
                            "Duration vector size (%zu) doesn't match number of segments (%zu). "
                            "Will use default duration for all segments.",
                            durations.size(), expected_num_segments);
            }

            for (size_t i = 0; i < durations.size(); ++i)
            {
                if (durations[i] < 0.0)
                {
                    RCLCPP_ERROR(logger_,
                                 "Segment %zu duration must be non-negative, got %.3f",
                                 i, durations[i]);
                    return false;
                }
            }
        }

        return true;
    }

    bool JointTrajectoryManager::planSingleTarget(
        const std::vector<double>& start_pos,
        const arms_ros2_control_msgs::msg::JointWaypoint& waypoint)
    {
        clearPlan();
#ifdef HAS_LINA_PLANNING
        interpolation_type_ = InterpolationType::DOUBLES;
        size_t num_joints = start_pos.size();

        //  起始点和目标点
        planning::TrajectPoint start_point(num_joints);
        planning::TrajectPoint end_point(num_joints);

        for (size_t i = 0; i < num_joints; i++)
        {
            start_point.joint_pos(i) = start_pos[i];
            end_point.joint_pos(i) = waypoint.position[i];
        }

        // 规划参数
        planning::TrajectoryParameter param;
        if (!getTrajectoryParameter(waypoint, param))
        {
            RCLCPP_ERROR(logger_, "Failed to resolve joint trajectory parameters for single target");
            return false;
        }

        // 3. 使用moveJ规划器

        try
        {
            movej_planner_ = std::make_unique<planning::moveJ>();

            planning::TrajectoryInitParameters init_params(
                start_point, end_point, param, period_);

            if (!movej_planner_->init(init_params))
            {
                RCLCPP_ERROR(logger_, "Failed to initialize moveJ planner for single target");
                reset();
                return false;
            }
            movej_planner_->setRealStartTime(0.0);
            mode_ = TrajectoryMode::SINGLE_NODE;
            planningTime_ = movej_planner_->getTotalTime();

            initialized_ = true;
            completed_ = false;
            percent_ = 0.0;

            RCLCPP_DEBUG(logger_,
                         "Initialized single-node trajectory: %zu joints, duration=%.3f, type=%s",
                         start_pos_.size(), planningTime_, toString(interpolation_type_));


            return true;
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(logger_, "Exception in single target planning: %s", e.what());
            return false;
        }
#else
        (void)start_pos;
        (void)waypoint;
        RCLCPP_ERROR(logger_, "lina_planning not available");
        return false;
#endif
    }

    bool JointTrajectoryManager::planMultiTrajectory(const std::vector<double>& start_pos,
                                                     const std::vector<arms_ros2_control_msgs::msg::JointWaypoint>&
                                                     waypoints)
    {
        clearPlan();
#ifndef HAS_LINA_PLANNING
        (void)start_pos;
#endif

        if (waypoints.size() < 2)
        {
            RCLCPP_ERROR(logger_, "At least 2 waypoints required for multi-trajectory planning");
            return false;
        }
#ifdef HAS_LINA_PLANNING
        interpolation_type_ = InterpolationType::DOUBLES;
        size_t num_joints = start_pos.size();
        size_t num_waypoints = waypoints.size();
        std::vector<planning::TrajectPoint> trajectory_points;
        trajectory_points.reserve(num_waypoints + 1);
        planning::TrajectPoint point(num_joints);
        for (size_t i = 0; i < num_joints; i++)
        {
            point.joint_pos(i) = start_pos[i];
        }
        trajectory_points.push_back(point);

        for (size_t i = 0; i < num_waypoints; i++)
        {
            for (size_t j = 0; j < num_joints; j++)
            {
                point.joint_pos(j) = waypoints[i].position[j];
            }
            point.blend_tolerance_ratio_of_movej = std::clamp(waypoints[i].blend_ratio_percent, 0.0, 1.0);
            trajectory_points.push_back(point);
        }

        // 2. 创建公共参数（使用第一个路点的参数）
        planning::TrajectoryParameter first_param;
        if (!getTrajectoryParameter(waypoints[0], first_param))
        {
            return false;
        }
        bool all_waypoints_has_parameter = true;
        std::vector<planning::TrajectoryParameter> multiple_parameters;
        multiple_parameters.reserve(num_waypoints);
        multiple_parameters.push_back(first_param);
        for (size_t i = 1; i < num_waypoints; i++)
        {
            planning::TrajectoryParameter param;
            if (!getTrajectoryParameter(waypoints[i], param))
            {
                all_waypoints_has_parameter = false;
                break;
            }
            multiple_parameters.push_back(param);
        }
        planning::TrajectoryInitParameters init_params;
        if (all_waypoints_has_parameter)
        {
            planning::TrajectoryInitParameters tmp_params(trajectory_points, multiple_parameters, period_);
            init_params = tmp_params;
        }
        else
        {
            planning::TrajectoryInitParameters tmp_params(trajectory_points, first_param, period_);
            init_params = tmp_params;
        }

        try
        {
            multi_node_planner_ = std::make_unique<planning::SmoothCurveOfMultiJointsUsingBlending>();


            if (!multi_node_planner_->init(init_params))
            {
                RCLCPP_ERROR(
                    logger_, "Failed to initialize SmoothCurveOfMultiJointsUsingBlending for DOUBLES interpolation");
                reset();
                return false;
            }


            mode_ = TrajectoryMode::MULTI_NODE_ADVANCED;

            multi_node_planner_->setRealStartTime(0.0);

            planningTime_ = multi_node_planner_->getTotalTime();

            initialized_ = true;
            completed_ = false;
            RCLCPP_DEBUG(logger_,
                                     "Initialized single-node trajectory: %zu joints, duration=%.3f, type=%s",
                                     start_pos_.size(), planningTime_, toString(interpolation_type_));
            RCLCPP_DEBUG(logger_,
                         "Initialized multi-node trajectory: %zu waypoints, duration=%.3f, type=%s",
                         num_waypoints, planningTime_, toString(interpolation_type_));

            return true;
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(logger_, "Exception in multi-trajectory planning: %s", e.what());
            return false;
        }
#else
        RCLCPP_ERROR(logger_, "lina_planning not available");
        return false;
#endif
    }


    void JointTrajectoryManager::clearPlan()
    {
        timed_evaluation_ready_ = false;
#ifdef HAS_LINA_PLANNING
        movej_planner_.reset();
        multi_node_planner_.reset();
#endif
    }

#ifdef HAS_LINA_PLANNING
    bool JointTrajectoryManager::getTrajectoryParameter(const arms_ros2_control_msgs::msg::JointWaypoint& waypoint,
                                                        planning::TrajectoryParameter& res)
    {
        const size_t num_joints = waypoint.position.size();
        const auto fillFromMessageOrDefault =
            [this, num_joints](const std::vector<double>& values, double default_value, planning::JointVector& out)
            {
                const bool use_message = !values.empty() && values.size() == num_joints;
                for (size_t i = 0; i < num_joints; ++i)
                {
                    out(i) = use_message ? values[i] : default_value;
                }
                return use_message;
            };

        if (waypoint.time_mode)
        {
            const double total_time = waypoint.total_time > 0 ? waypoint.total_time : 3.0;
            planning::TrajectoryParameter param(total_time, num_joints);
            const bool used_vel = fillFromMessageOrDefault(
                waypoint.max_velocity, default_max_velocity_, param.joint_max_vel);
            const bool used_acc = fillFromMessageOrDefault(
                waypoint.max_acceleration, default_max_acceleration_, param.joint_max_acc);
            const bool used_jerk = fillFromMessageOrDefault(
                waypoint.max_jerk, default_max_jerk_, param.joint_max_jerk);
            if (!used_vel || !used_acc || !used_jerk)
            {
                RCLCPP_INFO(logger_,
                            "Joint trajectory time mode missing vel/acc/jerk arrays; "
                            "using movej defaults (v=%.3f, a=%.3f, j=%.3f)",
                            default_max_velocity_, default_max_acceleration_, default_max_jerk_);
            }
            res = param;
        }
        else
        {
            planning::TrajectoryParameter param(num_joints);
            param.time_mode = false;
            const bool used_vel = fillFromMessageOrDefault(
                waypoint.max_velocity, default_max_velocity_, param.joint_max_vel);
            const bool used_acc = fillFromMessageOrDefault(
                waypoint.max_acceleration, default_max_acceleration_, param.joint_max_acc);
            const bool used_jerk = fillFromMessageOrDefault(
                waypoint.max_jerk, default_max_jerk_, param.joint_max_jerk);
            if (!used_vel || !used_acc || !used_jerk)
            {
                RCLCPP_INFO(logger_,
                            "Joint trajectory constraint mode missing vel/acc/jerk arrays; "
                            "using movej defaults (v=%.3f, a=%.3f, j=%.3f)",
                            default_max_velocity_, default_max_acceleration_, default_max_jerk_);
            }
            res = param;
        }
        return true;
    }
#endif
    double JointTrajectoryManager::getPlanningTime() const
    {
        if (isDoublesAvailable() && interpolation_type_ == InterpolationType::DOUBLES)
        {
            return planningTime_;
        }
        else
        {
         if (mode_==TrajectoryMode::SINGLE_NODE)
         {
             return duration_;
         }
         else
         {
             return trajectory_duration_;
         }
        }

    }
} // namespace arms_controller_common
