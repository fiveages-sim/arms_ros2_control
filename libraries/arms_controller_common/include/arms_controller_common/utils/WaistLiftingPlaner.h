#pragma once
#include <array>
#include <eigen3/Eigen/Dense>
#include <memory>
#include <vector>

// Optional dependency: lina_planning
#ifdef HAS_LINA_PLANNING
#include "lina_planning/planning/path_planner/movej.h"
#include "lina_planning/planning/path_planner/speedj.h"
#endif

namespace arms_controller_common
{
    class WaistLiftingPlaner
    {
    public:
        WaistLiftingPlaner() = default;
        ~WaistLiftingPlaner() = default;
        // 腰部 movej 定距升降（lifting_length 为 z/关节角增量，可能被限位裁剪）
        bool initTargetLiftingLength(const Eigen::Vector3d& init_joint_angle,
                                     const double lifting_length,
                                     const double duration,
                                     const double period = 0.01);

        // 腰部 speedj 从关节角起点规划到目标速度
        bool initTargetLiftingSpeed(const Eigen::Vector3d& init_joint_angle,
                                    const double target_lifting_speed,
                                    const double max_lifting_acc,
                                    const double max_lifting_jerk,
                                    const double total_time,
                                    const double period = 0.01);

        // 用缓存的升降位置/速度续接 speedj 规划（松手停升等，避免从关节角重算起点）
        bool initTargetLiftingSpeedFromCache(double target_lifting_speed,
                                             double max_lifting_acc,
                                             double max_lifting_jerk,
                                             double total_time,
                                             double period = 0.01);

        // 计算下一控制周期的腰部关节目标角（movej/speedj 插值后经逆解输出）
        bool calNextPoint(std::vector<double>& next_point);

        // 设置腰部为三平行旋转关节或单升降关节
        void setBodyJointThreeJointType(bool is_three_rotation_joint)
        {
            type_three_joint_ = is_three_rotation_joint;
        };

        // 是否为三平行旋转关节腰部
        bool isBodyThreeJoint() { return type_three_joint_; };
        // 设置三连杆几何参数（杆长、旋转方向、角度偏置）
        void setThreeJointParameter(double l1, double l2,
                                    const Eigen::Vector3d& rotation_direction,
                                    const Eigen::Vector3d& angle_offset);
        // 设置三关节角度上下限
        void setThreeJointLimit(const Eigen::Vector3d& angle_lower,
                                const Eigen::Vector3d& angle_upper);
        // 设置单关节升降角度上下限
        void setSingleJointLimit(const double angle_lower, const double angle_upper);

        // 当前 movej/speedj 轨迹是否已执行完毕
        bool isMotionOver()
        {
            if (type_speed_)
            {
#ifdef HAS_LINA_PLANNING
                if (speedj_planner_)
                {
                    return speedj_planner_->isMotionOver();
                }
#endif
                return speed_plan_state_.motion_over;
            }
            else
            {
#ifdef HAS_LINA_PLANNING
                if (movej_planner_)
                {
                    return movej_planner_->isMotionOver();
                }
#endif
                return speed_plan_state_.motion_over;
            }
        }

        // 将缓存速度置零并标记运动结束（规划失败或急停时用）
        void setCurrentVelToZero();

        // 最近一次定距升降实际规划的距离（可能因限位/逆解被裁剪）
        double getLastPlannedLiftingLength() const { return last_planned_lifting_length_; }

    private:
        // 用给定位置/速度初始化 speedj 规划器
        bool initSpeedJPlannerFromState(double start_pos, double start_vel,
                                        double target_vel, double max_acc,
                                        double max_jerk, double total_time,
                                        double period);
        // speedj 模式下按目标速度方向求可达极限位置（限位/工作空间）
        bool resolveSpeedModeMaxReachablePos(const Eigen::Vector3d& init_joint_angle,
                                             double start_pos, double target_speed,
                                             double& max_reachable_pos);

        // 根据终点逆解/关节限位，将目标升降坐标裁剪到可达范围
        bool resolveFeasibleLiftingEnd(const Eigen::Vector3d& init_joint_angle,
                                       double start_pos, double requested_end,
                                       double& feasible_end);

        // 单关节模式下将目标升降角裁剪到关节限位内
        bool resolveFeasibleLiftingEndSingleJoint(double start_pos, double requested_end,
                                                  double& feasible_end);

        // 三关节模式下将目标 z 裁剪到逆解可达范围（二分搜索）
        bool resolveFeasibleLiftingEndThreeJoint(const Eigen::Vector3d& init_joint_angle,
                                                 double start_pos, double requested_end,
                                                 double& feasible_end);

        // 定距规划终点逆解：仅检查工作空间与关节限位，不做 0.05rad 步长约束
        bool threeLinkPlanerEndpointIK(const Eigen::Vector3d& init_joint_angle,
                                       double x, double z, double phi,
                                       Eigen::Vector3d& output_joint_angle,
                                       bool log_errors = false);
        bool type_three_joint_{false}; // true为三个平行腰部关节，false为单个腰部移动关节

        bool type_speed_{false}; // true 为speedj，false为movej

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
        // 平行三连杆正解：关节角 -> 末端 (x, z, phi)
        void threeLinkPlanerFK(const Eigen::Vector3d& joint_angle, double* x,
                               double* z, double* phi);
        // 轨迹跟踪逆解：在 endpoint IK 基础上限制相对当前角不超过 0.05rad
        bool threeLinkPlanerIK(const Eigen::Vector3d& init_joint_angle,
                               const double x, const double z, const double phi,
                               Eigen::Vector3d& output_joint_angle);
        // 平行三连杆完整逆解，返回两组关节角解
        bool threeLinkPlanerFullIK(const double x, const double z, const double phi,
                                   std::array<Eigen::Vector3d, 2>& solutions,
                                   bool log_errors = true);
        // 从两组逆解中选取更接近初始关节角的一组
        Eigen::Vector3d choose_nearest_solution_of_body_joint3(
            const Eigen::Vector3d& q0, std::array<Eigen::Vector3d, 2>& solutions);
        // 检查三关节角是否超出限位
        bool isThreeJointsOverLimits(const Eigen::Vector3d& joint_angle);

        /*单关节限制参数*/
        double single_joint_limit_lower_;
        double single_joint_limit_upper_;
        // 检查单关节升降角是否超出限位
        bool isSingleJointsOverLimts(const double joint_angle);

        /*由于只升降腰部，就使用简单的movej单关节*/
#ifdef HAS_LINA_PLANNING
        std::unique_ptr<planning::moveJ> movej_planner_;

        /*假如需要手柄操作或者是按下按钮上升或者下降，松手停止，使用speedj*/

        std::unique_ptr<planning::SpeedJ> speedj_planner_;
#endif

        double min_val = 1.0e-9;
        double last_planned_lifting_length_{0.0};
        double speed_mode_max_reachable_pos_{0.0};
        double speed_mode_direction_{0.0};
        double speed_mode_max_acc_{0.0};
        double speed_mode_max_jerk_{0.0};
        double speed_mode_total_time_{0.0};
        double speed_mode_period_{0.01};
        bool speed_mode_max_reachable_valid_{false};
        bool speed_mode_stop_replanned_{false};
        // Cached waist state shared by speed/length planning in this instance.
        double waist_position_cache_{0.0};
        double waist_velocity_cache_{0.0};

        struct SpeedPlanState
        {
            double period{0.01};
            double elapsed_time{0.0};
            double total_time{0.0};
            double start_pos{0.0};
            double target_pos{0.0};
            double target_speed{0.0};
            double max_acc{0.0};
            bool motion_over{true};
        } speed_plan_state_;
    };
} // namespace arms_controller_common
