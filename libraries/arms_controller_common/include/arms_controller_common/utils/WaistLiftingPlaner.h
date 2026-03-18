#pragma once
#include "lina_planning/planning/path_planner/movej.h"
#include "lina_planning/planning/path_planner/speedj.h"
#include <array>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>

namespace arms_controller_common
{
    class WaistLiftingPlaner
    {
    public:
        WaistLiftingPlaner() = default;
        ~WaistLiftingPlaner() = default;
        // 腰部移动一定距离，
        bool initTargetLiftingLength(const Eigen::Vector3d& init_joint_angle,
                                     const double lifting_length,
                                     const double duration,
                                     const double period = 0.01);

        // 腰部speedj到达目标速度：
        bool initTargetLiftingSpeed(const Eigen::Vector3d& init_joint_angle,
                                    const double target_lifting_speed,
                                    const double max_lifting_acc,
                                    const double max_lifting_jerk,
                                    const double total_time,
                                    const double period = 0.01);

        bool calNextPoint(std::vector<double>& next_point);

        void setBodyJointThreeJointType(bool is_three_rotation_joint)
        {
            type_three_joint_ = is_three_rotation_joint;
        };

        bool isBodyThreeJoint() { return type_three_joint_; };
        void setThreeJointParameter(double l1, double l2,
                                    const Eigen::Vector3d& rotation_direction,
                                    const Eigen::Vector3d& angle_offset);
        void setThreeJointLimit(const Eigen::Vector3d& angle_lower,
                                const Eigen::Vector3d& angle_upper);
        void setSingleJointLimit(const double angle_lower, const double angle_upper);

        bool isMotionOver()
        {
            if (type_speed_)
            {
                return speedj_planner_->isMotionOver();
            }
            else
            {
                return movej_planner_->isMotionOver();
            }
        }

        void setCurrentVelToZero();

    private:
        bool type_three_joint_; // true为三个平行腰部关节，false为单个腰部移动关节

        bool type_speed_; // true 为speedj，false为movej

        /*三关节腰部参数*/
        double l1_;
        double l2_;
        double x_; // 由于只是升降腰部，只有z改变，其他参数不变
        double phi_;
        Eigen::Vector3d rotation_direction_;
        Eigen::Vector3d angle_offset_;
        Eigen::Vector3d limit_angler_lower_;
        Eigen::Vector3d limit_angle_upper_;
        Eigen::Vector3d current_joint_angle_;
        // 几何法求平行三连杆的正逆解
        void threeLinkPlanerFK(const Eigen::Vector3d& joint_angle, double* x,
                               double* z, double* phi);
        bool threeLinkPlanerIK(const Eigen::Vector3d& init_joint_angle,
                               const double x, const double z, const double phi,
                               Eigen::Vector3d& output_joint_angle);
        bool threeLinkPlanerFullIK(const double x, const double z, const double phi,
                                   std::array<Eigen::Vector3d, 2>& solutions);
        Eigen::Vector3d choose_nearest_solution_of_body_joint3(
            const Eigen::Vector3d& q0, std::array<Eigen::Vector3d, 2>& solutions);
        bool isThreeJointsOverLimits(const Eigen::Vector3d& joint_angle);

        /*单关节限制参数*/
        double single_joint_limit_lower_;
        double single_joint_limit_upper_;
        bool isSingleJointsOverLimts(const double joint_angle);

        /*由于只升降腰部，就使用简单的movej单关节*/
        std::unique_ptr<planning::moveJ> movej_planner_;

        /*假如需要手柄操作或者是按下按钮上升或者下降，松手停止，使用speedj*/

        std::unique_ptr<planning::SpeedJ> speedj_planner_;

        double min_val = 1.0e-9;
    };
} // namespace arms_controller_common
