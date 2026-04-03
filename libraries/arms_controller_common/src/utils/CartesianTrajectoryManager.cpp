#include "arms_controller_common/utils/CartesianTrajectoryManager.h"
#include <Eigen/Geometry>
#include <arms_ros2_control_msgs/msg/linear_message.hpp>
#include <iostream>

namespace arms_controller_common
{
    CartesianTrajectoryManager::CartesianTrajectoryManager()
        : logger_(rclcpp::get_logger("cartesian_trajectory_manager"))
    {
    }

    CartesianTrajectoryManager::~CartesianTrajectoryManager() = default;

    void CartesianTrajectoryManager::setKinematicsSolver(
        const std::shared_ptr<ArmKinematics>& kinematics)
    {
        arm_kinematics_ = kinematics;
    }

    bool CartesianTrajectoryManager::planSingleArmMoveL(
        const std::vector<double>& start_joint_pos,
        const arms_ros2_control_msgs::msg::LinearMessage& target_point_msg,
        const double period)
    {
        if (!arm_kinematics_)
        {
            std::cout << "kinematics is not set" << std::endl;
            return false;
        }
        arm_type_ = target_point_msg.arm_name;
        RobotState js(arm_kinematics_->getLeftArmJointCount(),
                      arm_kinematics_->getRightArmJointCount());
        if (arm_type_ == "left")
        {
            js.leftArmJoints = Eigen::Map<const Eigen::VectorXd>(
                start_joint_pos.data(), start_joint_pos.size());
        }
        else if (arm_type_ == "right")
        {
            js.rightArmJoints = Eigen::Map<const Eigen::VectorXd>(
                start_joint_pos.data(), start_joint_pos.size());
        }
        else
        {
            std::cout << "invalid arm type: " << arm_type_ << std::endl;
            return false;
        }
        current_joint_pos_ = Eigen::Map<const Eigen::VectorXd>(
            start_joint_pos.data(), start_joint_pos.size());

        EndEffectorPose start_pose =
            arm_kinematics_->computeSingleEndEffectorPose(js, arm_type_);

        // 初始化movel_planner_
        movel_planner_ = std::make_unique<planning::moveL>();
        planning::TrajectPoint ps;
        ps.cart_pos = start_pose.position;
        ps.quaternion_point.q.w = start_pose.quaternion.w();
        ps.quaternion_point.q.x = start_pose.quaternion.x();
        ps.quaternion_point.q.y = start_pose.quaternion.y();
        ps.quaternion_point.q.z = start_pose.quaternion.z();

        planning::TrajectPoint pe;
        pe.cart_pos(0) = target_point_msg.endpoint.position.x;
        pe.cart_pos(1) = target_point_msg.endpoint.position.y;
        pe.cart_pos(2) = target_point_msg.endpoint.position.z;
        pe.quaternion_point.q.x = target_point_msg.endpoint.orientation.x;
        pe.quaternion_point.q.y = target_point_msg.endpoint.orientation.y;
        pe.quaternion_point.q.z = target_point_msg.endpoint.orientation.z;
        pe.quaternion_point.q.w = target_point_msg.endpoint.orientation.w;

        if (target_point_msg.time_mode)
        {
            // 时间模式
            double duration = target_point_msg.duration;
            if (duration <= 0.001)
            {
                std::cout << "Invalid duration: " << duration << std::endl;
                return false;
            }
            planning::TrajectoryParameter time_mode_para(true, duration);
            if (fabs(target_point_msg.max_linear_velocity) > min_val &&
                fabs(target_point_msg.max_linear_acceleration) > min_val &&
                fabs(target_point_msg.max_linear_jerk) > min_val &&
                fabs(target_point_msg.max_angular_velocity) > min_val &&
                fabs(target_point_msg.max_angular_acceleration) > min_val &&
                fabs(target_point_msg.max_angular_jerk) > min_val)
            {
                time_mode_para.max_linear_vel = target_point_msg.max_linear_velocity;
                time_mode_para.max_linear_acc = target_point_msg.max_linear_acceleration;
                time_mode_para.max_linear_jerk = target_point_msg.max_linear_jerk;
                time_mode_para.max_angular_vel = target_point_msg.max_angular_velocity;
                time_mode_para.max_angular_acc =
                    target_point_msg.max_angular_acceleration;
                time_mode_para.max_angular_jerk = target_point_msg.max_angular_jerk;
            }

            planning::TrajectoryInitParameters init_para(ps, pe, time_mode_para,
                                                         period);
            if (!movel_planner_->init(init_para))
            {
                std::cout
                    << "Failed to initialize moveL planner with time mode parameters."
                    << std::endl;
                return false;
            }
        }
        else
        {
            // 速度模式
            planning::TrajectoryParameter speed_mode_para(
                target_point_msg.max_linear_velocity,
                target_point_msg.max_linear_acceleration,
                target_point_msg.max_linear_jerk, target_point_msg.max_angular_velocity,
                target_point_msg.max_angular_acceleration,
                target_point_msg.max_angular_jerk, true);
            planning::TrajectoryInitParameters init_para(ps, pe, speed_mode_para,
                                                         period);
            if (!movel_planner_->init(init_para))
            {
                std::cout
                    << "Failed to initialize moveL planner with speed mode parameters."
                    << std::endl;
                return false;
            }
        }
        movel_planner_->setRealStartTime(0.0);
        planningTime_ = movel_planner_->getTotalTime();
        return true;
    }

    bool CartesianTrajectoryManager::getNextJointPos(std::vector<double>& res)
    {
        if (movel_planner_)
        {
            planning::TrajectPoint movel_point = movel_planner_->run();
            if (movel_planner_->isMotionOver())
            {
                completed_ = true;
            }

            EndEffectorPose pose;
            pose.position = movel_point.cart_pos;
            Eigen::Quaterniond q(
                movel_point.quaternion_point.q.w, movel_point.quaternion_point.q.x,
                movel_point.quaternion_point.q.y, movel_point.quaternion_point.q.z);
            pose.setQuaternion(q);
            Eigen::VectorXd solution;
            if (!arm_kinematics_->solveSingleArmIK(pose, current_joint_pos_, solution,
                                                   arm_type_, 50))
            {
                RCLCPP_ERROR(logger_, "Failed to calculate the inverse solution for the "
                             "current Cartesian point.");
                return false;
            }
            // 更新当前关节角度
            current_joint_pos_ = solution;
            res.clear();
            for (size_t i = 0; i < static_cast<size_t>(solution.size()); i++)
            {
                res.push_back(current_joint_pos_(i));
            }
            return true;
        }
        else
        {
            RCLCPP_ERROR(logger_,
                         "moveL planner not initialized for DOUBLES interpolation");
            return false;
        }
    };

    std::vector<std::string> CartesianTrajectoryManager::getMovelJointNames(std::string arm_name)
    {
        if (!arm_kinematics_)
        {
            std::cout << "kinematics is not set" << std::endl;
            return {};
        }
        if (arm_name == "left")
        {
            return arm_kinematics_->getLeftArmJointNames();
        }
        else if (arm_name == "right")
        {
            return arm_kinematics_->getRightArmJointNames();
        }
        else
        {
            std::cout << "invalid arm name: " << arm_name << std::endl;
            return {};
        }
    }


    void CartesianTrajectoryManager::clearPlanner()
    {
        movel_planner_.reset();
        completed_ = false;
    }
} // namespace arms_controller_common
