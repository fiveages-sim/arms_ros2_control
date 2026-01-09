//
// Joint Trajectory Manager Implementation
//
#include "arms_controller_common/utils/JointTrajectoryManager.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace arms_controller_common
{
    JointTrajectoryManager::JointTrajectoryManager(rclcpp::Logger logger)
        : logger_(logger)
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
        if (type == InterpolationType::DOUBLES)
        {
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
        const std::vector<double>& durations,
        InterpolationType type,
        double controller_frequency,
        double default_duration,
        double tanh_scale)
    {
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
        period_ = (controller_frequency_ > 0.0) ? (1.0 / controller_frequency_) : 0.01;

        // Prepare segment durations
        size_t num_segments = waypoints.size() - 1;
        segment_durations_.clear();
        segment_durations_.reserve(num_segments);

        if (durations.empty())
        {
            // Use default duration for all segments
            segment_durations_.assign(num_segments, default_duration);
        }
        else if (durations.size() == num_segments)
        {
            segment_durations_ = durations;
        }
        else
        {
            RCLCPP_WARN(logger_,
                        "Duration vector size (%zu) doesn't match number of segments (%zu). "
                        "Using default duration for all segments.",
                        durations.size(), num_segments);
            segment_durations_.assign(num_segments, default_duration);
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
                    trajectory_points.reserve(waypoints_.size());
                    
                    // Convert waypoints to TrajectPoint
                    for (const auto& waypoint : waypoints_)
                    {
                        planning::TrajectPoint point(nr_of_joints);
                        for (size_t i = 0; i < nr_of_joints; ++i)
                        {
                            point.joint_pos(i) = waypoint[i];
                        }
                        trajectory_points.push_back(point);
                    }
                    
                    // Create trajectory parameters for each segment
                    std::vector<planning::TrajectoryParameter> trajectory_parameters;
                    trajectory_parameters.reserve(num_segments);
                    
                    for (size_t i = 0; i < num_segments; ++i)
                    {
                        planning::TrajectoryParameter param(segment_durations_[i], nr_of_joints);
                        trajectory_parameters.push_back(param);
                    }
                    
                    planning::TrajectoryInitParameters init_params(
                        trajectory_points, trajectory_parameters, period_);
                    
                    if (!multi_node_planner_->init(init_params))
                    {
                        RCLCPP_ERROR(logger_,
                                    "Failed to initialize SmoothCurveOfMultiJointsUsingBlending for DOUBLES interpolation");
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

    std::vector<double> JointTrajectoryManager::getNextPoint()
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
            result = computeSingleNodePoint();
            break;
        case TrajectoryMode::MULTI_NODE_BASIC:
            result = computeMultiNodeBasic();
            break;
#ifdef HAS_LINA_PLANNING
        case TrajectoryMode::MULTI_NODE_ADVANCED:
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

        start_pos_.clear();
        target_pos_.clear();
        waypoints_.clear();
        segment_durations_.clear();
        segment_progress_.clear();

        duration_ = 0.0;
        percent_ = 0.0;
        current_segment_ = 0;
        interpolation_type_ = InterpolationType::TANH;
        tanh_scale_ = 3.0;
        controller_frequency_ = 100.0;
        period_ = 0.01;

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

    std::vector<double> JointTrajectoryManager::computeSingleNodePoint()
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
                return target_pos_;  // Fallback to target
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
            updateProgress();
            double phase = calculatePhase(interpolation_type_, percent_);

            for (size_t i = 0; i < start_pos_.size() && i < target_pos_.size(); ++i)
            {
                double interpolated_value = phase * target_pos_[i] + (1.0 - phase) * start_pos_[i];
                result.push_back(interpolated_value);
            }
        }

        return result;
    }

    std::vector<double> JointTrajectoryManager::computeMultiNodeBasic()
    {
        // Check if we need to move to next segment
        if (current_segment_ < segment_progress_.size())
        {
            double segment_duration = segment_durations_[current_segment_];
            
            // Update progress for current segment
            if (segment_duration > 0.0 && controller_frequency_ > 0.0)
            {
                segment_progress_[current_segment_] += period_;
                segment_progress_[current_segment_] = std::min(
                    segment_progress_[current_segment_], segment_duration);
            }
            else
            {
                segment_progress_[current_segment_] = segment_duration;
            }

            // Calculate phase for current segment
            double segment_percent = (segment_duration > 0.0) 
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
                    
                    percent_ = (total_duration > 0.0) ? (elapsed_duration / total_duration) : 1.0;
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
                
                percent_ = (total_duration > 0.0) ? (elapsed_duration / total_duration) : 1.0;
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
            return waypoints_.back();  // Fallback to final waypoint
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
            const double scale = (tanh_scale_ > 0.0) ? tanh_scale_ : 3.0;
            phase = std::tanh(percent * scale);
        }
        else
        {
            // Default to linear for unknown types
            phase = percent;
        }

        return std::clamp(phase, 0.0, 1.0);
    }

    void JointTrajectoryManager::updateProgress()
    {
        if (duration_ <= 0.0 || controller_frequency_ <= 0.0)
        {
            percent_ = 1.0;  // Already normalized
            completed_ = true;
        }
        else
        {
            // percent_ stores normalized progress [0, 1]
            // Increment by the fraction of duration that one period represents
            double increment = period_ / duration_;
            percent_ += increment;
            percent_ = std::min(percent_, 1.0);

            if (percent_ >= 1.0)
            {
                completed_ = true;
                percent_ = 1.0;  // Ensure exact 1.0
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
} // namespace arms_controller_common

