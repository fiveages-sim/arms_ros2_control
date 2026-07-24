//
// Joint Trajectory Manager Implementation
//
#include "arms_controller_common/utils/JointTrajectoryManager.h"
#include <algorithm>
#include <cmath>
#include <utility>

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

    bool JointTrajectoryManager::initSingleNode(
        const std::vector<double>& start_pos,
        const std::vector<double>& target_pos,
        double duration,
        InterpolationType type,
        double controller_frequency,
        double tanh_scale)
    {
        if (!validateSingleNodeParams(start_pos, target_pos, duration, type))
        {
            return false;
        }

        // Reset state
        reset();

        // Store parameters
        start_pos_ = start_pos;
        target_pos_ = target_pos;
        duration_ = duration;
        interpolation_type_ = type;
        tanh_scale_ = tanh_scale;
        controller_frequency_ = controller_frequency;
        period_ = (controller_frequency_ > 0.0) ? (1.0 / controller_frequency_) : 0.01;

        // Initialize based on interpolation type
        if (type == InterpolationType::ONLINE)
        {
            online_filters_.clear();
            online_filters_.reserve(start_pos_.size());
            for (size_t i = 0; i < start_pos_.size(); ++i)
            {
                OnlineTrajectoryFilter filter(online_limits_);
                filter.reset(start_pos_[i]);
                filter.setTarget(target_pos_[i]);
                online_filters_.push_back(filter);
            }
            mode_ = TrajectoryMode::SINGLE_NODE_ONLINE;
        }
        else if (type == InterpolationType::DOUBLES)
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
                RCLCPP_WARN(logger_,
                            "DOUBLES interpolation requested but lina_planning is not available. "
                            "Falling back to LINEAR interpolation.");
                interpolation_type_ = InterpolationType::LINEAR;
                mode_ = TrajectoryMode::SINGLE_NODE;
            }
            else
            {
#ifdef HAS_LINA_PLANNING
                // Initialize moveJ planner for DOUBLES
                try
                {
                    movej_planner_ = std::make_unique<planning::moveJ>();

                    size_t nr_of_joints = start_pos_.size();
                    planning::TrajectPoint start_joint_point(nr_of_joints);
                    planning::TrajectPoint end_joint_point(nr_of_joints);
                    planning::TrajectoryParameter traj_param(duration_, nr_of_joints);

                    for (size_t i = 0; i < nr_of_joints; i++)
                    {
                        start_joint_point.joint_pos(i) = start_pos_[i];
                        end_joint_point.joint_pos(i) = target_pos_[i];
                    }

                    planning::TrajectoryInitParameters movej_init_para(
                        start_joint_point, end_joint_point, traj_param, period_);

                    if (!movej_planner_->init(movej_init_para))
                    {
                        RCLCPP_ERROR(logger_, "Failed to initialize moveJ planner for DOUBLES interpolation");
                        reset();
                        return false;
                    }

                    movej_planner_->setRealStartTime(0.0);
                    mode_ = TrajectoryMode::SINGLE_NODE;
                }
                catch (const std::exception& e)
                {
                    RCLCPP_ERROR(logger_, "Exception while initializing moveJ planner: %s", e.what());
                    reset();
                    return false;
                }
#endif
            }
        }
        else
        {
            // Basic interpolation types (TANH, LINEAR, NONE)
            mode_ = TrajectoryMode::SINGLE_NODE;
        }

        initialized_ = true;
        completed_ = false;
        percent_ = 0.0;

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

        if (!validateMultiNodeParams(waypoints, durations))
        {
            return false;
        }

        // Reset state
        reset();

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

        // ONLINE is intended for a continuously updated single target. Preserve
        // the established linear behavior for multi-waypoint trajectories.
        if (type == InterpolationType::ONLINE)
        {
            RCLCPP_WARN(logger_,
                        "ONLINE interpolation does not support multi-node trajectories; "
                        "using LINEAR interpolation for this trajectory");
            interpolation_type_ = InterpolationType::LINEAR;
            mode_ = TrajectoryMode::MULTI_NODE_BASIC;
        }
        // Initialize based on interpolation type
        else if (type == InterpolationType::DOUBLES)
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
            // Return final position
            if (mode_ == TrajectoryMode::SINGLE_NODE ||
                mode_ == TrajectoryMode::SINGLE_NODE_ONLINE)
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
        case TrajectoryMode::SINGLE_NODE_ONLINE:
            result = computeOnlinePoint(step_seconds);
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
        mode_ = TrajectoryMode::NONE;
        initialized_ = false;
        completed_ = false;

        // Clear trajectory data
        start_pos_.clear();
        target_pos_.clear();
        online_filters_.clear();
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

    bool JointTrajectoryManager::setOnlineLimits(
        double max_velocity,
        double max_acceleration,
        double max_jerk,
        double tracking_frequency,
        double target_filter_alpha,
        double position_tolerance,
        double velocity_tolerance,
        double acceleration_tolerance)
    {
        OnlineTrajectoryFilter::Limits limits;
        limits.min_velocity = -std::abs(max_velocity);
        limits.max_velocity = std::abs(max_velocity);
        limits.min_acceleration = -std::abs(max_acceleration);
        limits.max_acceleration = std::abs(max_acceleration);
        limits.max_jerk = std::abs(max_jerk);
        limits.tracking_frequency = tracking_frequency;
        limits.target_filter_alpha = target_filter_alpha;

        // Streaming ONLINE targets call StateMoveJ::updateParam() at the input
        // rate. Avoid validating, assigning and logging the same configuration
        // on every 500 Hz target message while still allowing live parameter
        // changes to take effect.
        if (online_limits_configured_ &&
            limits.min_velocity == online_limits_.min_velocity &&
            limits.max_velocity == online_limits_.max_velocity &&
            limits.min_acceleration == online_limits_.min_acceleration &&
            limits.max_acceleration == online_limits_.max_acceleration &&
            limits.max_jerk == online_limits_.max_jerk &&
            limits.tracking_frequency == online_limits_.tracking_frequency &&
            limits.target_filter_alpha == online_limits_.target_filter_alpha &&
            position_tolerance == online_position_tolerance_ &&
            velocity_tolerance == online_velocity_tolerance_ &&
            acceleration_tolerance == online_acceleration_tolerance_)
        {
            return true;
        }

        OnlineTrajectoryFilter validator;
        if (!validator.configure(limits) || !std::isfinite(position_tolerance) ||
            !std::isfinite(velocity_tolerance) || !std::isfinite(acceleration_tolerance) ||
            position_tolerance <= 0.0 || velocity_tolerance <= 0.0 ||
            acceleration_tolerance <= 0.0)
        {
            RCLCPP_ERROR(logger_,
                         "Invalid ONLINE interpolation limits: velocity=%.6f, acceleration=%.6f, "
                         "jerk=%.6f, frequency=%.6f, alpha=%.6f, tolerances=[%.6g, %.6g, %.6g]",
                         max_velocity, max_acceleration, max_jerk, tracking_frequency,
                         target_filter_alpha, position_tolerance,
                         velocity_tolerance, acceleration_tolerance);
            return false;
        }

        online_limits_ = limits;
        online_position_tolerance_ = position_tolerance;
        online_velocity_tolerance_ = velocity_tolerance;
        online_acceleration_tolerance_ = acceleration_tolerance;
        online_limits_configured_ = true;
        RCLCPP_INFO(
            logger_,
            "Ruckig ONLINE configured: velocity=%.3f rad/s, acceleration=%.3f rad/s^2, "
            "jerk=%.3f rad/s^3, target_alpha=%.3f "
            "(movej_online_tracking_frequency is compatibility-only)",
            max_velocity, max_acceleration, max_jerk, target_filter_alpha);
        return true;
    }

    bool JointTrajectoryManager::updateOnlineTarget(const std::vector<double>& target_pos)
    {
        if (mode_ != TrajectoryMode::SINGLE_NODE_ONLINE || !initialized_ ||
            target_pos.size() != online_filters_.size())
        {
            return false;
        }

        target_pos_ = target_pos;
        for (size_t i = 0; i < online_filters_.size(); ++i)
        {
            online_filters_[i].setTarget(target_pos_[i]);
        }
        completed_ = false;
        percent_ = 0.0;
        return true;
    }

    bool JointTrajectoryManager::isOnlineMode() const
    {
        return initialized_ && mode_ == TrajectoryMode::SINGLE_NODE_ONLINE;
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

            for (size_t i = 0; i < start_pos_.size() && i < target_pos_.size(); ++i)
            {
                double interpolated_value = phase * target_pos_[i] + (1.0 - phase) * start_pos_[i];
                result.push_back(interpolated_value);
            }
        }

        return result;
    }

    std::vector<double> JointTrajectoryManager::computeOnlinePoint(double step_seconds)
    {
        if (online_filters_.empty() || online_filters_.size() != target_pos_.size())
        {
            RCLCPP_ERROR(logger_, "ONLINE interpolation state is not initialized correctly");
            return {};
        }

        const double dt = step_seconds > 0.0 ? step_seconds : period_;
        if (!std::isfinite(dt) || dt <= 0.0)
        {
            RCLCPP_ERROR(logger_, "Invalid ONLINE interpolation step: %.9f", dt);
            return {};
        }

        std::vector<double> result;
        result.reserve(online_filters_.size());
        bool all_settled = true;
        for (auto& filter : online_filters_)
        {
            result.push_back(filter.update(dt));
            all_settled = all_settled && filter.isSettled(
                online_position_tolerance_,
                online_velocity_tolerance_,
                online_acceleration_tolerance_);
        }

        if (all_settled)
        {
            result = target_pos_;
            completed_ = true;
            percent_ = 1.0;
        }
        else
        {
            completed_ = false;
            percent_ = 0.0;  // A streaming target has no fixed-duration progress.
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
        }

        return result;
    }
#endif

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
        getTrajectoryParameter(waypoint, param);

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
#ifdef HAS_LINA_PLANNING
        movej_planner_.reset();
        multi_node_planner_.reset();
#endif
    }

#ifdef HAS_LINA_PLANNING
    bool JointTrajectoryManager::getTrajectoryParameter(const arms_ros2_control_msgs::msg::JointWaypoint& waypoint,
                                                        planning::TrajectoryParameter& res)
    {
        size_t num_joints = waypoint.position.size();
        bool is_time_mode = waypoint.time_mode;
        if (is_time_mode)
        {
            double total_time = waypoint.total_time > 0 ? waypoint.total_time : 3.0;
            planning::TrajectoryParameter param(total_time, num_joints);
            if (!waypoint.max_velocity.empty() && !waypoint.max_acceleration.empty() && !waypoint.max_jerk.empty() &&
                waypoint.max_velocity.size() == num_joints && waypoint.max_acceleration.size() == num_joints && waypoint
                .max_jerk.size() == num_joints)
            {
                //时间模式下也可以使用用户设置的参数，不用内部的默认参数，时间模式其实是一个相对比例关系
                for (size_t i = 0; i < num_joints; i++)
                {
                    param.joint_max_vel(i) = waypoint.max_velocity[i];
                    param.joint_max_acc(i) = waypoint.max_acceleration[i];
                    param.joint_max_jerk(i) = waypoint.max_jerk[i];
                }
            }
            res = param;
        }
        else
        {
            if (waypoint.max_velocity.empty() || waypoint.max_acceleration.empty() || waypoint.max_jerk.empty() ||
                waypoint.max_velocity.size() != num_joints || waypoint.max_acceleration.size() != num_joints || waypoint
                .max_jerk.size() != num_joints)
            {
                RCLCPP_INFO(logger_, "In non-time mode, please enter the correct planned parameters");
                return false;
            }
            planning::TrajectoryParameter param(num_joints);
            param.time_mode = false;
            for (size_t i = 0; i < num_joints; i++)
            {
                param.joint_max_vel(i) = waypoint.max_velocity[i];
                param.joint_max_acc(i) = waypoint.max_acceleration[i];
                param.joint_max_jerk(i) = waypoint.max_jerk[i];
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
