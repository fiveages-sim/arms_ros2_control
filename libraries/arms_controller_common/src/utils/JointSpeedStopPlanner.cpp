#include "arms_controller_common/utils/JointSpeedStopPlanner.h"
#include <algorithm>
#include <cmath>

#ifdef HAS_LINA_PLANNING
#include "lina_planning/planning/common/trajectory_init_parameters.h"
#endif

namespace arms_controller_common
{
    namespace
    {
        constexpr double kVelEpsilon = 1.0e-4;
        constexpr double kMinPeriod = 1.0e-4;
    }

    void JointSpeedStopPlanner::reset()
    {
        active_ = false;
        joint_pos_cache_.clear();
#ifdef HAS_LINA_PLANNING
        speedj_planner_.reset();
#else
        fallback_state_ = {};
#endif
    }

    bool JointSpeedStopPlanner::init(
        const std::vector<double>& joint_pos,
        const std::vector<double>& joint_vel,
        double period,
        double max_acc,
        double max_jerk)
    {
        reset();

        if (joint_pos.empty())
        {
            return false;
        }

        period_ = std::max(period, kMinPeriod);
        max_acc_ = std::max(max_acc, 0.1);
        max_jerk_ = std::max(max_jerk, 0.5);
        joint_pos_cache_ = joint_pos;

        const size_t num_joints = joint_pos.size();
        std::vector<double> start_vel = joint_vel;
        if (start_vel.size() != num_joints)
        {
            start_vel.assign(num_joints, 0.0);
        }

        bool already_stopped = true;
        for (double v : start_vel)
        {
            if (std::abs(v) > kVelEpsilon)
            {
                already_stopped = false;
                break;
            }
        }

        if (already_stopped)
        {
            active_ = true;
            return true;
        }

#ifdef HAS_LINA_PLANNING
        speedj_planner_ = std::make_unique<planning::SpeedJ>();

        planning::TrajectPoint start_point(num_joints);
        planning::TrajectPoint end_point(num_joints);
        planning::TrajectoryParameter traj_param(num_joints);

        for (size_t i = 0; i < num_joints; ++i)
        {
            start_point.joint_pos(i) = joint_pos[i];
            start_point.joint_vel(i) = start_vel[i];
            end_point.joint_pos(i) = joint_pos[i];
            end_point.joint_vel(i) = 0.0;
            traj_param.joint_max_acc(i) = max_acc_;
            traj_param.joint_max_jerk(i) = max_jerk_;
        }

        traj_param.total_time = 0.0;

        const planning::TrajectoryInitParameters speedj_init_para(
            start_point, end_point, traj_param, period_);

        if (!speedj_planner_->init(speedj_init_para))
        {
            reset();
            return false;
        }

        speedj_planner_->setRealStartTime(0.0);
        active_ = true;
        return true;
#else
        fallback_state_.joint_vel = start_vel;
        fallback_state_.motion_over = false;
        active_ = true;
        return true;
#endif
    }

    std::vector<double> JointSpeedStopPlanner::run()
    {
        if (!active_)
        {
            return joint_pos_cache_;
        }

#ifdef HAS_LINA_PLANNING
        if (!speedj_planner_)
        {
            return joint_pos_cache_;
        }

        const size_t num_joints = joint_pos_cache_.size();
        planning::TrajectPoint point(num_joints);
        point = speedj_planner_->run();

        for (size_t i = 0; i < num_joints; ++i)
        {
            joint_pos_cache_[i] = point.joint_pos(i);
        }
        return joint_pos_cache_;
#else
        const double dt = period_;
        bool all_stopped = true;
        for (size_t i = 0; i < fallback_state_.joint_vel.size(); ++i)
        {
            double& v = fallback_state_.joint_vel[i];
            if (std::abs(v) <= kVelEpsilon)
            {
                v = 0.0;
                continue;
            }

            all_stopped = false;
            const double sign = (v > 0.0) ? -1.0 : 1.0;
            if (max_acc_ > kVelEpsilon)
            {
                v += sign * max_acc_ * dt;
                if ((sign < 0.0 && v > 0.0) || (sign > 0.0 && v < 0.0))
                {
                    v = 0.0;
                }
            }
            else
            {
                v = 0.0;
            }

            joint_pos_cache_[i] += v * dt;
        }

        if (all_stopped)
        {
            fallback_state_.motion_over = true;
        }
        return joint_pos_cache_;
#endif
    }

    bool JointSpeedStopPlanner::isMotionOver() const
    {
        if (!active_)
        {
            return true;
        }

#ifdef HAS_LINA_PLANNING
        if (!speedj_planner_)
        {
            return true;
        }
        return speedj_planner_->isMotionOver();
#else
        return fallback_state_.motion_over;
#endif
    }
} // namespace arms_controller_common
