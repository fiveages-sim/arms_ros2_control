#include "arms_controller_common/utils/WaistLiftingPlaner.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace
{
    double position_of_waist = 0.0;
    double velocity_of_waist = 0.0;
} // namespace

namespace arms_controller_common
{
    bool WaistLiftingPlaner::initTargetLiftingLength(
        const Eigen::Vector3d& init_joint_angle, const double lifting_length,
        const double duration, const double period)
    {
        type_speed_ = false;
        current_joint_angle_ = init_joint_angle;

        movej_planner_ = std::make_unique<planning::moveJ>();

        size_t nr_of_joints = 1;
        planning::TrajectPoint start_joint_point(nr_of_joints);
        planning::TrajectPoint end_joint_point(nr_of_joints);
        planning::TrajectoryParameter traj_param(duration, nr_of_joints);

        if (type_three_joint_)
        {
            // 对于三关节，我们规划的是z方向的变化
            double init_x, init_z, init_phi;
            threeLinkPlanerFK(init_joint_angle, &init_x, &init_z, &init_phi);
            x_ = init_x;
            phi_ = init_phi;
            start_joint_point.joint_pos(0) = init_z;
            end_joint_point.joint_pos(0) = init_z + lifting_length;
        }
        else
        {
            // 对于单关节，直接规划关节角度
            start_joint_point.joint_pos(0) = init_joint_angle(0);
            end_joint_point.joint_pos(0) = init_joint_angle(0) + lifting_length;
        }

        planning::TrajectoryInitParameters movej_init_para(
            start_joint_point, end_joint_point, traj_param, period);

        if (!movej_planner_->init(movej_init_para))
        {
            return false;
        }

        movej_planner_->setRealStartTime(0.0);
        return true;
    }

    bool WaistLiftingPlaner::initTargetLiftingSpeed(
        const Eigen::Vector3d& init_joint_angle, const double target_lifting_speed,
        const double max_lifting_acc, const double max_lifting_jerk,
        const double total_time, const double period)
    {
        type_speed_ = true;
        current_joint_angle_ = init_joint_angle;

        speedj_planner_ = std::make_unique<planning::SpeedJ>();

        size_t nr_of_joints = 1;
        planning::TrajectPoint start_joint_point(nr_of_joints);
        planning::TrajectPoint end_joint_point(nr_of_joints);
        planning::TrajectoryParameter traj_param(nr_of_joints);

        if (type_three_joint_)
        {
            // 对于三关节，我们规划的是z方向的变化
            double init_x, init_z, init_phi;
            threeLinkPlanerFK(init_joint_angle, &init_x, &init_z, &init_phi);
            x_ = init_x;
            phi_ = init_phi;
            start_joint_point.joint_pos(0) = init_z;
        }
        else
        {
            // 对于单关节，直接规划关节角度
            start_joint_point.joint_pos(0) = init_joint_angle(0);
        }
        start_joint_point.joint_vel(0) = velocity_of_waist;
        end_joint_point.joint_vel(0) = target_lifting_speed;
        traj_param.joint_max_acc(0) = max_lifting_acc;
        traj_param.joint_max_jerk(0) = max_lifting_jerk;

        if (fabs(target_lifting_speed) > min_val)
        {
            traj_param.total_time = total_time;
        }
        else
        {
            // 停止的时候这个时间一定要设置为0
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
    }

    void WaistLiftingPlaner::setCurrentVelToZero()
    {
        velocity_of_waist = 0.0;
    }

    bool WaistLiftingPlaner::calNextPoint(std::vector<double>& next_point)
    {
        planning::TrajectPoint point(1);
        if (type_speed_)
        {
            point = speedj_planner_->run();
        }
        else
        {
            point = movej_planner_->run();
        }
        double curr_z = point.joint_pos(0);
        position_of_waist = curr_z;
        velocity_of_waist = point.joint_vel(0);
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

    bool WaistLiftingPlaner::threeLinkPlanerIK(
        const Eigen::Vector3d& init_joint_angle, const double x, const double z,
        const double phi, Eigen::Vector3d& output_joint_angle)
    {
        std::array<Eigen::Vector3d, 2> solutions;
        std::array<Eigen::Vector3d, 2> solutions_with_constraints;
        if (!threeLinkPlanerFullIK(x, z, phi, solutions))
        {
            std::cerr << "Joint singularity without inverse solution" << std::endl;
            return false;
        }
        size_t size_of_solutions_with_constraints = 0;
        for (size_t i = 0; i < 2; i++)
        {
            Eigen::Vector3d curr_sol = solutions[i];
            if (!isThreeJointsOverLimits(curr_sol))
            {
                solutions_with_constraints[size_of_solutions_with_constraints] = curr_sol;
                size_of_solutions_with_constraints++;
            }
        }
        if (size_of_solutions_with_constraints == 0)
        {
            std::cerr << "All solutions over joint limt" << std::endl;
            return false;
        }
        else if (size_of_solutions_with_constraints == 1)
        {
            output_joint_angle = solutions_with_constraints[0];
        }
        else if (size_of_solutions_with_constraints == 2)
        {
            output_joint_angle = choose_nearest_solution_of_body_joint3(
                init_joint_angle, solutions_with_constraints);
        }
        else
        {
            return false;
        }
        return true;
    }

    bool WaistLiftingPlaner::threeLinkPlanerFullIK(
        const double x, const double z, const double phi,
        std::array<Eigen::Vector3d, 2>& solutions)
    {
        Eigen::Vector3d joint_angle1, joint_angle2;
        if (sqrt(x * x + z * z) > l1_ + l2_)
        {
            std::cerr << "The target pose is unreachable, as it falls outside the "
                "working space range"
                << std::endl;
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
