#include "arms_controller_common/utils/WaistLiftingPlaner.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace arms_controller_common
{
    bool WaistLiftingPlaner::initSpeedJPlannerFromState(
        const double start_pos, const double start_vel, const double target_vel,
        const double max_acc, const double max_jerk, const double total_time,
        const double period)
    {
#ifdef HAS_LINA_PLANNING
        speedj_planner_ = std::make_unique<planning::SpeedJ>();
        size_t nr_of_joints = 1;
        planning::TrajectPoint start_joint_point(nr_of_joints);
        planning::TrajectPoint end_joint_point(nr_of_joints);
        planning::TrajectoryParameter traj_param(nr_of_joints);
        start_joint_point.joint_pos(0) = start_pos;
        start_joint_point.joint_vel(0) = start_vel;
        end_joint_point.joint_vel(0) = target_vel;
        traj_param.joint_max_acc(0) = max_acc;
        traj_param.joint_max_jerk(0) = max_jerk;
        if (std::fabs(target_vel - start_vel) > min_val)
        {
            traj_param.total_time = total_time;
        }
        else
        {
            traj_param.total_time = 0.0;
        }
        planning::TrajectoryInitParameters speedj_init_para(
            start_joint_point, end_joint_point, traj_param, period);
        if (!speedj_planner_->init(speedj_init_para))
        {
            std::cerr << "Failed to init speedj planning" << std::endl;
            return false;
        }
        speedj_planner_->setRealStartTime(0.0);
        return true;
#else
        (void)start_pos;
        (void)start_vel;
        (void)target_vel;
        (void)max_acc;
        (void)max_jerk;
        (void)total_time;
        (void)period;
        return false;
#endif
    }

    bool WaistLiftingPlaner::resolveSpeedModeMaxReachablePos(
        const Eigen::Vector3d& init_joint_angle, const double start_pos,
        const double target_speed, double& max_reachable_pos)
    {
        if (std::abs(target_speed) <= min_val)
        {
            max_reachable_pos = start_pos;
            return true;
        }
        const double direction = (target_speed > 0.0) ? 1.0 : -1.0;
        if (type_three_joint_)
        {
            const double max_workspace = std::abs(l1_) + std::abs(l2_);
            const double requested_end = start_pos + direction * (2.0 * max_workspace + 0.01);
            return resolveFeasibleLiftingEnd(init_joint_angle, start_pos, requested_end,
                                             max_reachable_pos);
        }
        max_reachable_pos = (direction > 0.0) ? single_joint_limit_upper_ : single_joint_limit_lower_;
        return true;
    }

    bool WaistLiftingPlaner::resolveFeasibleLiftingEndSingleJoint(
        const double start_pos, const double requested_end, double& feasible_end)
    {
        feasible_end = std::clamp(requested_end, single_joint_limit_lower_, single_joint_limit_upper_);
        (void)start_pos;
        return true;
    }

    bool WaistLiftingPlaner::resolveFeasibleLiftingEndThreeJoint(
        const Eigen::Vector3d& init_joint_angle, const double start_pos,
        const double requested_end, double& feasible_end)
    {
        feasible_end = start_pos;
        const double requested_delta = requested_end - start_pos;
        if (std::abs(requested_delta) <= min_val)
        {
            return true;
        }

        Eigen::Vector3d endpoint_joint_angle;
        if (threeLinkPlanerEndpointIK(init_joint_angle, x_, requested_end, phi_,
                                      endpoint_joint_angle, false))
        {
            feasible_end = requested_end;
            return true;
        }

        double near = start_pos;
        double far = requested_end;
        double best = start_pos;
        constexpr int kMaxBinarySearchIterations = 32;
        for (int i = 0; i < kMaxBinarySearchIterations; ++i)
        {
            if (std::abs(far - near) <= min_val)
            {
                break;
            }
            const double mid = 0.5 * (near + far);
            if (threeLinkPlanerEndpointIK(init_joint_angle, x_, mid, phi_, endpoint_joint_angle,
                                          false))
            {
                best = mid;
                near = mid;
            }
            else
            {
                far = mid;
            }
        }
        feasible_end = best;
        return true;
    }

    bool WaistLiftingPlaner::resolveFeasibleLiftingEnd(
        const Eigen::Vector3d& init_joint_angle, const double start_pos,
        const double requested_end, double& feasible_end)
    {
        if (type_three_joint_)
        {
            return resolveFeasibleLiftingEndThreeJoint(
                init_joint_angle, start_pos, requested_end, feasible_end);
        }
        return resolveFeasibleLiftingEndSingleJoint(start_pos, requested_end, feasible_end);
    }

    bool WaistLiftingPlaner::initTargetLiftingLength(
        const Eigen::Vector3d& init_joint_angle, const double lifting_length,
        const double duration, const double period)
    {
        type_speed_ = false;
        speed_mode_max_reachable_valid_ = false;
        speed_mode_stop_replanned_ = false;
        current_joint_angle_ = init_joint_angle;
        double start_pos = 0.0;
        double requested_end_pos = 0.0;
        double end_pos = 0.0;

        if (type_three_joint_)
        {
            // 对于三关节，我们规划的是z方向的变化
            double init_x, init_z, init_phi;
            threeLinkPlanerFK(init_joint_angle, &init_x, &init_z, &init_phi);
            x_ = init_x;
            phi_ = init_phi;
            start_pos = init_z;
            requested_end_pos = init_z + lifting_length;
        }
        else
        {
            // 对于单关节，直接规划关节角度
            start_pos = init_joint_angle(0);
            requested_end_pos = init_joint_angle(0) + lifting_length;
        }

        if (!resolveFeasibleLiftingEnd(init_joint_angle, start_pos, requested_end_pos, end_pos))
        {
            last_planned_lifting_length_ = 0.0;
            return false;
        }
        last_planned_lifting_length_ = end_pos - start_pos;

#ifdef HAS_LINA_PLANNING
        movej_planner_ = std::make_unique<planning::moveJ>();
        size_t nr_of_joints = 1;
        planning::TrajectPoint start_joint_point(nr_of_joints);
        planning::TrajectPoint end_joint_point(nr_of_joints);
        planning::TrajectoryParameter traj_param(duration, nr_of_joints);
        start_joint_point.joint_pos(0) = start_pos;
        end_joint_point.joint_pos(0) = end_pos;

        planning::TrajectoryInitParameters movej_init_para(
            start_joint_point, end_joint_point, traj_param, period);

        if (!movej_planner_->init(movej_init_para))
        {
            return false;
        }

        movej_planner_->setRealStartTime(0.0);
#else
        speed_plan_state_.period = std::max(period, 1.0e-4);
        speed_plan_state_.elapsed_time = 0.0;
        speed_plan_state_.total_time = std::max(duration, 0.0);
        speed_plan_state_.start_pos = start_pos;
        speed_plan_state_.target_pos = end_pos;
        speed_plan_state_.target_speed = 0.0;
        speed_plan_state_.max_acc = 0.0;
        speed_plan_state_.motion_over = (speed_plan_state_.total_time <= min_val);
        waist_position_cache_ = (speed_plan_state_.total_time <= min_val) ? end_pos : start_pos;
        waist_velocity_cache_ = 0.0;
#endif
#ifdef HAS_LINA_PLANNING
        waist_position_cache_ = start_pos;
        waist_velocity_cache_ = 0.0;
#endif
        return true;
    }

    bool WaistLiftingPlaner::initTargetLiftingSpeed(
        const Eigen::Vector3d& init_joint_angle, const double target_lifting_speed,
        const double max_lifting_acc, const double max_lifting_jerk,
        const double total_time, const double period)
    {
        type_speed_ = true;
        speed_mode_direction_ = (target_lifting_speed > 0.0) ? 1.0 :
            ((target_lifting_speed < 0.0) ? -1.0 : 0.0);
        speed_mode_max_acc_ = max_lifting_acc;
        speed_mode_max_jerk_ = max_lifting_jerk;
        speed_mode_total_time_ = total_time;
        speed_mode_period_ = period;
        speed_mode_max_reachable_valid_ = false;
        speed_mode_stop_replanned_ = false;
        current_joint_angle_ = init_joint_angle;
        double start_pos = 0.0;

        if (type_three_joint_)
        {
            // 对于三关节，我们规划的是z方向的变化
            double init_x, init_z, init_phi;
            threeLinkPlanerFK(init_joint_angle, &init_x, &init_z, &init_phi);
            x_ = init_x;
            phi_ = init_phi;
            start_pos = init_z;
        }
        else
        {
            // 对于单关节，直接规划关节角度
            start_pos = init_joint_angle(0);
        }

        if (std::abs(target_lifting_speed) > min_val)
        {
            if (!resolveSpeedModeMaxReachablePos(
                    init_joint_angle, start_pos, target_lifting_speed, speed_mode_max_reachable_pos_))
            {
                return false;
            }
            speed_mode_max_reachable_valid_ = true;
        }
#ifdef HAS_LINA_PLANNING
        if (!initSpeedJPlannerFromState(start_pos, waist_velocity_cache_,
                                        target_lifting_speed, max_lifting_acc,
                                        max_lifting_jerk, total_time, period))
        {
            return false;
        }
#else
        (void)max_lifting_jerk;
        speed_plan_state_.period = std::max(period, 1.0e-4);
        speed_plan_state_.elapsed_time = 0.0;
        speed_plan_state_.total_time = std::max(total_time, 0.0);
        speed_plan_state_.start_pos = start_pos;
        speed_plan_state_.target_pos = start_pos;
        waist_position_cache_ = start_pos;
        speed_plan_state_.target_speed = target_lifting_speed;
        speed_plan_state_.max_acc = std::max(max_lifting_acc, 0.0);
        speed_plan_state_.motion_over = (std::abs(target_lifting_speed) <= min_val);
        if (speed_plan_state_.total_time <= min_val || std::abs(target_lifting_speed) <= min_val)
        {
            speed_plan_state_.target_speed = 0.0;
            waist_velocity_cache_ = 0.0;
            speed_plan_state_.motion_over = true;
        }
#endif
#ifdef HAS_LINA_PLANNING
        waist_position_cache_ = start_pos;
#endif
        return true;
    }

    void WaistLiftingPlaner::setCurrentVelToZero()
    {
        waist_velocity_cache_ = 0.0;
        speed_plan_state_.target_speed = 0.0;
        speed_plan_state_.motion_over = true;
    }

    bool WaistLiftingPlaner::calNextPoint(std::vector<double>& next_point)
    {
        double curr_z = 0.0;
#ifdef HAS_LINA_PLANNING
        planning::TrajectPoint point(1);
        if (type_speed_)
        {
            if (!speedj_planner_)
            {
                return false;
            }
            point = speedj_planner_->run();
            const double current_pos = point.joint_pos(0);
            const double current_vel = point.joint_vel(0);
            if (speed_mode_max_reachable_valid_ && !speed_mode_stop_replanned_)
            {
                const double moving_direction =
                    (std::abs(current_vel) > min_val) ? ((current_vel > 0.0) ? 1.0 : -1.0)
                                                      : speed_mode_direction_;
                if (std::abs(moving_direction) > min_val)
                {
                    const double stop_dist = speedj_planner_->calculateDecDistanceOfV(
                        current_vel, speed_mode_max_acc_, speed_mode_max_jerk_);
                    const double stop_pos = current_pos + moving_direction * stop_dist;
                    const bool need_stop_replan =
                        (moving_direction > 0.0) ?
                        (stop_pos >= speed_mode_max_reachable_pos_ - min_val) :
                        (stop_pos <= speed_mode_max_reachable_pos_ + min_val);
                    if (need_stop_replan && std::abs(current_vel) > min_val)
                    {
                        std::cout << "Waist speedj reaches deceleration point, replan to stop. "
                            << "current_pos: " << current_pos
                            << ", current_vel: " << current_vel
                            << ", stop_pos: " << stop_pos
                            << ", max_reachable_pos: " << speed_mode_max_reachable_pos_
                            << std::endl;
                        speed_mode_stop_replanned_ = true;
                        if (!initSpeedJPlannerFromState(
                                current_pos, current_vel, 0.0, speed_mode_max_acc_,
                                speed_mode_max_jerk_, 0.0, speed_mode_period_))
                        {
                            return false;
                        }
                        point = speedj_planner_->run();
                    }
                }
            }
        }
        else
        {
            point = movej_planner_->run();
        }
        curr_z = point.joint_pos(0);
        waist_position_cache_ = curr_z;
        waist_velocity_cache_ = point.joint_vel(0);
#else
        if (type_speed_)
        {
            if (!speed_plan_state_.motion_over)
            {
                const double dt = speed_plan_state_.period;
                if (speed_plan_state_.max_acc > min_val)
                {
                    const double max_delta_v = speed_plan_state_.max_acc * dt;
                    const double delta_v = std::clamp(speed_plan_state_.target_speed - waist_velocity_cache_,
                                                      -max_delta_v, max_delta_v);
                    waist_velocity_cache_ += delta_v;
                }
                else
                {
                    waist_velocity_cache_ = speed_plan_state_.target_speed;
                }

                waist_position_cache_ += waist_velocity_cache_ * dt;
                speed_plan_state_.elapsed_time += dt;
                if (speed_plan_state_.total_time > min_val &&
                    speed_plan_state_.elapsed_time >= speed_plan_state_.total_time)
                {
                    speed_plan_state_.motion_over = true;
                }
            }
            curr_z = waist_position_cache_;
        }
        else
        {
            if (!speed_plan_state_.motion_over)
            {
                const double duration = std::max(speed_plan_state_.total_time, min_val);
                speed_plan_state_.elapsed_time = std::min(
                    speed_plan_state_.elapsed_time + speed_plan_state_.period, duration);
                const double p = std::clamp(speed_plan_state_.elapsed_time / duration, 0.0, 1.0);
                const double smooth_p = p * p * (3.0 - 2.0 * p);
                waist_position_cache_ = speed_plan_state_.start_pos +
                    (speed_plan_state_.target_pos - speed_plan_state_.start_pos) * smooth_p;
                if (p >= 1.0 - min_val)
                {
                    speed_plan_state_.motion_over = true;
                }
            }
            curr_z = waist_position_cache_;
            waist_velocity_cache_ = 0.0;
        }
#endif
        if (type_three_joint_)
        {
            // 三关节腰部
            Eigen::Vector3d cal_angles;
            if (!threeLinkPlanerIK(current_joint_angle_, x_, curr_z, phi_,
                                   cal_angles))
            {
                return false;
            }
            current_joint_angle_ = cal_angles;
            next_point.resize(3);
            for (size_t i = 0; i < 3; i++)
            {
                next_point[i] = cal_angles(i);
            }
            return true;
        }
        else
        {
            // 单关节腰部
            if (isSingleJointsOverLimts(curr_z))
            {
                return false;
            }
            next_point.resize(1);
            next_point[0] = curr_z;
            current_joint_angle_(0) = curr_z;
            return true;
        }
    }

    void WaistLiftingPlaner::setThreeJointParameter(
        double l1, double l2, const Eigen::Vector3d& rotation_direction,
        const Eigen::Vector3d& angle_offset)
    {
        l1_ = l1;
        l2_ = l2;
        rotation_direction_ = rotation_direction;
        angle_offset_ = angle_offset;
    }

    void WaistLiftingPlaner::setThreeJointLimit(
        const Eigen::Vector3d& angle_lower, const Eigen::Vector3d& angle_upper)
    {
        limit_angler_lower_ = angle_lower;
        limit_angle_upper_ = angle_upper;
    }

    void WaistLiftingPlaner::setSingleJointLimit(const double angle_lower,
                                                 const double angle_upper)
    {
        single_joint_limit_lower_ = angle_lower;
        single_joint_limit_upper_ = angle_upper;
    };

    void WaistLiftingPlaner::threeLinkPlanerFK(const Eigen::Vector3d& joint_angle,
                                               double* x, double* z, double* phi)
    {
        double q1 = joint_angle(0) * rotation_direction_(0) + angle_offset_(0);
        double q2 = joint_angle(1) * rotation_direction_(1) + angle_offset_(1);
        double q3 = joint_angle(2) * rotation_direction_(2) + angle_offset_(2);

        *x = l1_ * sin(q1) + l2_ * sin(q1 + q2);
        *z = l1_ * cos(q1) + l2_ * cos(q1 + q2);
        *phi = q1 + q2 + q3;
    }

    bool WaistLiftingPlaner::threeLinkPlanerEndpointIK(
        const Eigen::Vector3d& init_joint_angle, const double x, const double z,
        const double phi, Eigen::Vector3d& output_joint_angle, const bool log_errors)
    {
        std::array<Eigen::Vector3d, 2> solutions;
        std::array<Eigen::Vector3d, 2> solutions_with_constraints;
        if (!threeLinkPlanerFullIK(x, z, phi, solutions, log_errors))
        {
            if (log_errors)
            {
                std::cerr << "Joint singularity without inverse solution" << std::endl;
            }
            return false;
        }
        size_t size_of_solutions_with_constraints = 0;
        for (size_t i = 0; i < 2; i++)
        {
            const Eigen::Vector3d& curr_sol = solutions[i];
            if (!isThreeJointsOverLimits(curr_sol))
            {
                solutions_with_constraints[size_of_solutions_with_constraints] = curr_sol;
                size_of_solutions_with_constraints++;
            }
        }
        if (size_of_solutions_with_constraints == 0)
        {
            if (log_errors)
            {
                std::cerr << "All solutions over joint limt" << std::endl;
            }
            return false;
        }
        if (size_of_solutions_with_constraints == 1)
        {
            output_joint_angle = solutions_with_constraints[0];
        }
        else
        {
            output_joint_angle = choose_nearest_solution_of_body_joint3(
                init_joint_angle, solutions_with_constraints);
        }
        return true;
    }

    bool WaistLiftingPlaner::threeLinkPlanerIK(
        const Eigen::Vector3d& init_joint_angle, const double x, const double z,
        const double phi, Eigen::Vector3d& output_joint_angle)
    {
        if (!threeLinkPlanerEndpointIK(init_joint_angle, x, z, phi, output_joint_angle, true))
        {
            return false;
        }
        const Eigen::Vector3d delta_joint = output_joint_angle - init_joint_angle;
        // 轨迹跟踪时防止跳到较远的另一组逆解
        for (int i = 0; i < 3; ++i)
        {
            if (fabs(delta_joint(i)) > 0.05)
            {
                std::cerr << "The inverse solution position is too far from the current point"
                    << std::endl;
                return false;
            }
        }
        return true;
    }

    bool WaistLiftingPlaner::threeLinkPlanerFullIK(
        const double x, const double z, const double phi,
        std::array<Eigen::Vector3d, 2>& solutions, const bool log_errors)
    {
        Eigen::Vector3d joint_angle1, joint_angle2;
        if (sqrt(x * x + z * z) > l1_ + l2_)
        {
            if (log_errors)
            {
                std::cerr << "The target pose is unreachable, as it falls outside the "
                    "working space range"
                    << std::endl;
            }
            return false;
        }
        double cos_th2 = (x * x + z * z - l1_ * l1_ - l2_ * l2_) / (2.0 * l1_ * l2_);
        cos_th2 = std::clamp(cos_th2, -1.0, 1.0);
        double th2_1 = acos(cos_th2);
        double th2_2 = -th2_1;
        double beta = atan2(x, z);
        double cos_psi = (x * x + z * z + l1_ * l1_ - l2_ * l2_) /
            (2.0 * l1_ * sqrt(x * x + z * z));

        cos_psi = std::clamp(cos_psi, -1.0, 1.0);
        double psi = acos(cos_psi);
        double th1_1 = beta - psi;
        double th1_2 = beta + psi;
        double th3_1 = phi - th1_1 - th2_1;
        double th3_2 = phi - th1_2 - th2_2;
        joint_angle1 << th1_1, th2_1, th3_1;
        joint_angle2 << th1_2, th2_2, th3_2;
        for (size_t i = 0; i < 3; i++)
        {
            joint_angle1(i) -= angle_offset_(i);
            joint_angle1(i) *= rotation_direction_(i);
            joint_angle2(i) -= angle_offset_(i);
            joint_angle2(i) *= rotation_direction_(i);
        }

        solutions[0] = joint_angle1;
        solutions[1] = joint_angle2;
        return true;
    };

    Eigen::Vector3d WaistLiftingPlaner::choose_nearest_solution_of_body_joint3(
        const Eigen::Vector3d& q0, std::array<Eigen::Vector3d, 2>& solutions)
    {
        double d1 = (q0 - solutions[0]).squaredNorm();
        double d2 = (q0 - solutions[1]).squaredNorm();
        return (d1 < d2) ? solutions[0] : solutions[1];
    };

    bool WaistLiftingPlaner::isThreeJointsOverLimits(
        const Eigen::Vector3d& joint_angle)
    {
        for (size_t i = 0; i < 3; i++)
        {
            if (joint_angle(i) < limit_angler_lower_(i) - min_val ||
                joint_angle(i) > limit_angle_upper_(i) + min_val)
            {
                return true;
            }
        }
        return false;
    };

    bool WaistLiftingPlaner::isSingleJointsOverLimts(const double joint_angle)
    {
        return (joint_angle < single_joint_limit_lower_ - min_val ||
            joint_angle > single_joint_limit_upper_ + min_val);
    };
} // namespace arms_controller_common
