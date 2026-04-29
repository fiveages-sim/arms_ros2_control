#include "arms_controller_common/utils/CartesianTrajectoryManager.h"
#include <Eigen/Geometry>
#include <arms_ros2_control_msgs/msg/linear_message.hpp>
#include <iostream>
#include <sstream>

#include "../../include/arms_controller_common/utils/Kinematics.h"

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
        last_error_.clear();
        completed_ = false;
        dual_arm_mode_ = false;
        left_arm_active_ = false;
        right_arm_active_ = false;
        if (!arm_kinematics_)
        {
            last_error_ = "kinematics is not set";
            std::cout << last_error_ << std::endl;
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
            last_error_ = "invalid arm type: " + arm_type_;
            std::cout << last_error_ << std::endl;
            return false;
        }
        Eigen::VectorXd current_joint_pos = Eigen::Map<const Eigen::VectorXd>(
            start_joint_pos.data(), start_joint_pos.size());

        EndEffectorPose start_pose =
            arm_kinematics_->computeSingleEndEffectorPose(js, arm_type_);

        std::unique_ptr<planning::moveL> planner = std::make_unique<planning::moveL>();
        planning::TrajectPoint ps = setCartesianPoint(start_pose);

        planning::TrajectPoint pe = setCartesianPoint(target_point_msg.endpoint);

        planning::TrajectoryParameter para = setCartesianParameter(target_point_msg);
        planning::TrajectoryInitParameters init_para(ps, pe, para, period);
        if (!planner->init(init_para))
        {
            last_error_ = "Failed to initialize moveL planner";
            std::cout << last_error_ << std::endl;
            return false;
        }
        ArmKinematics::SolverType ik_tp;
        if (target_point_msg.ik_type=="BFGS")
        {
            ik_tp = ArmKinematics::SolverType::BFGS;
            std::cout << "BFGS is set" << std::endl;
        }else if (target_point_msg.ik_type=="DLS")
        {
            ik_tp = ArmKinematics::SolverType::DLS;
            std::cout << "DLS is set" << std::endl;
        }else if (target_point_msg.ik_type=="SDK")
        {
            ik_tp=ArmKinematics::SolverType::SDK;
            std::cout << "SDK is set" << std::endl;
        }
        else
        {
            ik_tp = ArmKinematics::SolverType::AUTO;
            std::cout << "Auto is set" << std::endl;
        }
        arm_kinematics_->setSolverType(ik_tp);
        planner->setRealStartTime(0.0);
        planningTime_ = planner->getTotalTime();
        path_type_ = PathType::LINE;
        if (arm_type_ == "left")
        {
            left_movel_planner_ = std::move(planner);
            left_current_joint_pos_ = current_joint_pos;
            right_movel_planner_.reset();
            right_current_joint_pos_.resize(0);
            left_arm_active_ = true;
            right_arm_active_ = false;
        }
        else
        {
            right_movel_planner_ = std::move(planner);
            right_current_joint_pos_ = current_joint_pos;
            left_movel_planner_.reset();
            left_current_joint_pos_.resize(0);
            left_arm_active_ = false;
            right_arm_active_ = true;
        }
        return true;
    }

    bool CartesianTrajectoryManager::planSingleArmMoveC(
        const std::vector<double>& start_joint_pos,
        const arms_ros2_control_msgs::msg::CircleMessage& target_circle_msg,
        const double period)
    {
        last_error_.clear();
        completed_ = false;
        dual_arm_mode_ = false;
        left_arm_active_ = false;
        right_arm_active_ = false;
        if (!arm_kinematics_)
        {
            last_error_ = "kinematics is not set";
            std::cout << last_error_ << std::endl;
            return false;
        }
        std::string error_msg;
        if (!validateCircleMsg(target_circle_msg, error_msg))
        {
            last_error_ = error_msg;
            std::cout << last_error_ << std::endl;
            return false;
        }

        arm_type_ = target_circle_msg.arm_name;
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
            last_error_ = "invalid arm type: " + arm_type_;
            std::cout << last_error_ << std::endl;
            return false;
        }
        Eigen::VectorXd current_joint_pos = Eigen::Map<const Eigen::VectorXd>(
            start_joint_pos.data(), start_joint_pos.size());

        EndEffectorPose start_pose =
            arm_kinematics_->computeSingleEndEffectorPose(js, arm_type_);

        // 更新起点
        planning::TrajectPoint ps = setCartesianPoint(start_pose);
        // 更新终点
        planning::TrajectPoint pe = setCartesianPoint(target_circle_msg.endpoint);

        planning::TrajectoryParameter para = setCartesianParameter(target_circle_msg);

        std::unique_ptr<planning::CircularCurver> planner = std::make_unique<planning::CircularCurver>();

        if (!target_circle_msg.use_three_point_method)
        {
            planning::TrajectoryInitParameters init_para(ps, pe, para, period);
            if (!planner->init(init_para))
            {
                last_error_ = "Failed to initialize MoveC planner";
                return false;
            }
        }
        else
        {
            planning::TrajectPoint pm = setCartesianPoint(target_circle_msg.midpoint);
            planning::TrajectoryInitParameters init_para(ps, pm, pe, para, period);
            if (!planner->init(init_para))
            {
                last_error_ = "Failed to initialize MoveC planner";
                return false;
            }
        }
        ArmKinematics::SolverType ik_tp;
        if (target_circle_msg.ik_type=="BFGS")
        {
            ik_tp = ArmKinematics::SolverType::BFGS;
        }else if (target_circle_msg.ik_type=="DLS")
        {
            ik_tp = ArmKinematics::SolverType::DLS;
        }else if (target_circle_msg.ik_type=="SDK")
        {
            ik_tp=ArmKinematics::SolverType::SDK;
        }
        else
        {
            ik_tp = ArmKinematics::SolverType::AUTO;
        }

        arm_kinematics_->setSolverType(ik_tp);
        planner->setRealStartTime(0.0);
        planningTime_ = planner->getTotalTime();
        path_type_ = PathType::CIRCLE;
        if (arm_type_ == "left")
        {
            left_movec_planner_ = std::move(planner);
            left_current_joint_pos_ = current_joint_pos;
            right_movec_planner_.reset();
            right_current_joint_pos_.resize(0);
            left_arm_active_ = true;
            right_arm_active_ = false;
        }
        else
        {
            right_movec_planner_ = std::move(planner);
            right_current_joint_pos_ = current_joint_pos;
            left_movec_planner_.reset();
            left_current_joint_pos_.resize(0);
            left_arm_active_ = false;
            right_arm_active_ = true;
        }
        return true;
    }

    bool CartesianTrajectoryManager::planDualArmMoveL(
        const std::vector<double>& start_joint_pos,
        const arms_ros2_control_msgs::msg::LinearMessage& target_point_msg,
        const double period)
    {
        last_error_.clear();
        completed_ = false;
        dual_arm_mode_ = true;
        left_arm_active_ = true;
        right_arm_active_ = true;

        if (!arm_kinematics_)
        {
            last_error_ = "kinematics is not set";
            return false;
        }

        const size_t left_count = arm_kinematics_->getLeftArmJointCount();
        const size_t right_count = arm_kinematics_->getRightArmJointCount();
        if (start_joint_pos.size() != left_count + right_count)
        {
            last_error_ = "Dual-arm MoveL requires " + std::to_string(left_count + right_count) +
                          " start joints, got " + std::to_string(start_joint_pos.size());
            return false;
        }

        arms_ros2_control_msgs::msg::LinearMessage left_msg = target_point_msg;
        left_msg.arm_name = "left";
        arms_ros2_control_msgs::msg::LinearMessage right_msg = target_point_msg;
        right_msg.arm_name = "right";
        right_msg.endpoint = target_point_msg.right_endpoint;

        std::vector<double> left_start(start_joint_pos.begin(), start_joint_pos.begin() + left_count);
        std::vector<double> right_start(start_joint_pos.begin() + left_count, start_joint_pos.end());

        if (!planSingleArmMoveL(left_start, left_msg, period))
        {
            dual_arm_mode_ = false;
            return false;
        }

        const double left_time = planningTime_;
        auto left_planner = std::move(left_movel_planner_);
        auto left_joint_pos = left_current_joint_pos_;

        if (!planSingleArmMoveL(right_start, right_msg, period))
        {
            dual_arm_mode_ = false;
            left_movel_planner_.reset();
            left_current_joint_pos_.resize(0);
            return false;
        }

        const double right_time = planningTime_;
        auto right_planner = std::move(right_movel_planner_);
        auto right_joint_pos = right_current_joint_pos_;

        left_movel_planner_ = std::move(left_planner);
        left_current_joint_pos_ = left_joint_pos;
        right_movel_planner_ = std::move(right_planner);
        right_current_joint_pos_ = right_joint_pos;
        path_type_ = PathType::LINE;
        planningTime_ = std::max(left_time, right_time);
        arm_type_ = "both";
        dual_arm_mode_ = true;
        left_arm_active_ = true;
        right_arm_active_ = true;
        completed_ = false;
        return true;
    }

    bool CartesianTrajectoryManager::planDualArmMoveC(
        const std::vector<double>& start_joint_pos,
        const arms_ros2_control_msgs::msg::CircleMessage& target_circle_msg,
        const double period)
    {
        last_error_.clear();
        completed_ = false;
        dual_arm_mode_ = true;
        left_arm_active_ = true;
        right_arm_active_ = true;

        if (!arm_kinematics_)
        {
            last_error_ = "kinematics is not set";
            return false;
        }

        const size_t left_count = arm_kinematics_->getLeftArmJointCount();
        const size_t right_count = arm_kinematics_->getRightArmJointCount();
        if (start_joint_pos.size() != left_count + right_count)
        {
            last_error_ = "Dual-arm MoveC requires " + std::to_string(left_count + right_count) +
                          " start joints, got " + std::to_string(start_joint_pos.size());
            return false;
        }

        arms_ros2_control_msgs::msg::CircleMessage left_msg = target_circle_msg;
        left_msg.arm_name = "left";
        arms_ros2_control_msgs::msg::CircleMessage right_msg = target_circle_msg;
        right_msg.arm_name = "right";
        right_msg.midpoint = target_circle_msg.right_midpoint;
        right_msg.endpoint = target_circle_msg.right_endpoint;
        right_msg.center = target_circle_msg.right_center;
        right_msg.axis = target_circle_msg.right_axis;
        right_msg.rotate_angle = target_circle_msg.right_rotate_angle;

        std::vector<double> left_start(start_joint_pos.begin(), start_joint_pos.begin() + left_count);
        std::vector<double> right_start(start_joint_pos.begin() + left_count, start_joint_pos.end());

        if (!planSingleArmMoveC(left_start, left_msg, period))
        {
            dual_arm_mode_ = false;
            return false;
        }

        const double left_time = planningTime_;
        auto left_planner = std::move(left_movec_planner_);
        auto left_joint_pos = left_current_joint_pos_;

        if (!planSingleArmMoveC(right_start, right_msg, period))
        {
            dual_arm_mode_ = false;
            left_movec_planner_.reset();
            left_current_joint_pos_.resize(0);
            return false;
        }

        const double right_time = planningTime_;
        auto right_planner = std::move(right_movec_planner_);
        auto right_joint_pos = right_current_joint_pos_;

        left_movec_planner_ = std::move(left_planner);
        left_current_joint_pos_ = left_joint_pos;
        right_movec_planner_ = std::move(right_planner);
        right_current_joint_pos_ = right_joint_pos;
        path_type_ = PathType::CIRCLE;
        planningTime_ = std::max(left_time, right_time);
        arm_type_ = "both";
        dual_arm_mode_ = true;
        left_arm_active_ = true;
        right_arm_active_ = true;
        completed_ = false;
        return true;
    }

    bool CartesianTrajectoryManager::getNextJointPos(std::vector<double>& res)
    {
        last_error_.clear();
        if (dual_arm_mode_ && path_type_ == PathType::LINE)
        {
            std::vector<double> left_res;
            std::vector<double> right_res;
            if (left_arm_active_ && !stepLineArm("left", left_movel_planner_, left_current_joint_pos_, left_res))
            {
                return false;
            }
            if (right_arm_active_ && !stepLineArm("right", right_movel_planner_, right_current_joint_pos_, right_res))
            {
                return false;
            }
            res.clear();
            res.insert(res.end(), left_res.begin(), left_res.end());
            res.insert(res.end(), right_res.begin(), right_res.end());
            const bool left_done = !left_arm_active_ || !left_movel_planner_ || left_movel_planner_->isMotionOver();
            const bool right_done = !right_arm_active_ || !right_movel_planner_ || right_movel_planner_->isMotionOver();
            completed_ = left_done && right_done;
            return true;
        }
        else if (dual_arm_mode_ && path_type_ == PathType::CIRCLE)
        {
            std::vector<double> left_res;
            std::vector<double> right_res;
            if (left_arm_active_ && !stepCircleArm("left", left_movec_planner_, left_current_joint_pos_, left_res))
            {
                return false;
            }
            if (right_arm_active_ && !stepCircleArm("right", right_movec_planner_, right_current_joint_pos_, right_res))
            {
                return false;
            }
            res.clear();
            res.insert(res.end(), left_res.begin(), left_res.end());
            res.insert(res.end(), right_res.begin(), right_res.end());
            const bool left_done = !left_arm_active_ || !left_movec_planner_ || left_movec_planner_->isMotionOver();
            const bool right_done = !right_arm_active_ || !right_movec_planner_ || right_movec_planner_->isMotionOver();
            completed_ = left_done && right_done;
            return true;
        }
        else if (left_movel_planner_ && path_type_ == PathType::LINE && left_arm_active_)
        {
            return stepLineArm("left", left_movel_planner_, left_current_joint_pos_, res);
        }
        else if (right_movel_planner_ && path_type_ == PathType::LINE && right_arm_active_)
        {
            return stepLineArm("right", right_movel_planner_, right_current_joint_pos_, res);
        }
        else if (left_movec_planner_ && path_type_ == PathType::CIRCLE && left_arm_active_)
        {
            return stepCircleArm("left", left_movec_planner_, left_current_joint_pos_, res);
        }
        else if (right_movec_planner_ && path_type_ == PathType::CIRCLE && right_arm_active_)
        {
            return stepCircleArm("right", right_movec_planner_, right_current_joint_pos_, res);
        }
        else
        {
            last_error_ = "moveL planner or moveC planner not initialized for DOUBLES interpolation";
            RCLCPP_ERROR(logger_, "moveL planner or moveC planner not initialized for "
                         "DOUBLES interpolation");
            return false;
        }
    };

    std::vector<std::string>
    CartesianTrajectoryManager::getMovelJointNames(std::string arm_name)
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
        else if (arm_name == "both")
        {
            auto left_names = arm_kinematics_->getLeftArmJointNames();
            auto right_names = arm_kinematics_->getRightArmJointNames();
            left_names.insert(left_names.end(), right_names.begin(), right_names.end());
            return left_names;
        }
        else
        {
            std::cout << "invalid arm name: " << arm_name << std::endl;
            return {};
        }
    }

    void CartesianTrajectoryManager::clearPlanner()
    {
        left_movel_planner_.reset();
        right_movel_planner_.reset();
        left_movec_planner_.reset();
        right_movec_planner_.reset();
        left_current_joint_pos_.resize(0);
        right_current_joint_pos_.resize(0);
        dual_arm_mode_ = false;
        left_arm_active_ = false;
        right_arm_active_ = false;
        completed_ = false;
        last_error_.clear();
    }

    bool CartesianTrajectoryManager::validatePose(
        const geometry_msgs::msg::Pose& pose,
        const std::string& pose_name,
        std::string& error_message) const
    {
        if (std::isnan(pose.position.x) || std::isnan(pose.position.y) || std::isnan(pose.position.z))
        {
            error_message = pose_name + " position contains NaN values";
            return false;
        }

        const double qx = pose.orientation.x;
        const double qy = pose.orientation.y;
        const double qz = pose.orientation.z;
        const double qw = pose.orientation.w;
        if (std::isnan(qx) || std::isnan(qy) || std::isnan(qz) || std::isnan(qw))
        {
            error_message = pose_name + " orientation contains NaN values";
            return false;
        }
        return true;
    }

    bool CartesianTrajectoryManager::stepLineArm(
        const std::string& arm_name,
        std::unique_ptr<planning::moveL>& planner,
        Eigen::VectorXd& current_joint_pos,
        std::vector<double>& result)
    {
        if (!planner)
        {
            last_error_ = "MoveL planner is not initialized for " + arm_name + " arm";
            return false;
        }

        if (planner->isMotionOver())
        {
            result.assign(current_joint_pos.data(), current_joint_pos.data() + current_joint_pos.size());
            return true;
        }

        planning::TrajectPoint movel_point = planner->run();
        EndEffectorPose pose;
        pose.position = movel_point.cart_pos;
        Eigen::Quaterniond q(
            movel_point.quaternion_point.q.w, movel_point.quaternion_point.q.x,
            movel_point.quaternion_point.q.y, movel_point.quaternion_point.q.z);
        pose.setQuaternion(q);

        Eigen::VectorXd solution;
        ArmKinematics::SolutionInfo info;
        if (!arm_kinematics_->solveSingleArmIKWithInfo(
                pose, current_joint_pos, solution, info, arm_name, 50))
        {
            std::ostringstream oss;
            oss << "IK failed while calculating MoveL joint position for " << arm_name
                << " arm: solver=" << ArmKinematics::solverTypeName(info.usedSolver)
                << ", status=" << info.status
                << ", final_error=" << info.poseErrorNorm;
            last_error_ = oss.str();
            RCLCPP_ERROR(logger_, "%s", last_error_.c_str());
            return false;
        }

        current_joint_pos = solution;
        result.assign(solution.data(), solution.data() + solution.size());
        if (planner->isMotionOver())
        {
            completed_ = !dual_arm_mode_;
        }
        return true;
    }

    bool CartesianTrajectoryManager::stepCircleArm(
        const std::string& arm_name,
        std::unique_ptr<planning::CircularCurver>& planner,
        Eigen::VectorXd& current_joint_pos,
        std::vector<double>& result)
    {
        if (!planner)
        {
            last_error_ = "MoveC planner is not initialized for " + arm_name + " arm";
            return false;
        }

        if (planner->isMotionOver())
        {
            result.assign(current_joint_pos.data(), current_joint_pos.data() + current_joint_pos.size());
            return true;
        }

        planning::TrajectPoint movec_point = planner->run();
        EndEffectorPose pose;
        pose.position = movec_point.cart_pos;
        Eigen::Quaterniond q(
            movec_point.quaternion_point.q.w, movec_point.quaternion_point.q.x,
            movec_point.quaternion_point.q.y, movec_point.quaternion_point.q.z);
        pose.setQuaternion(q);

        Eigen::VectorXd solution;
        ArmKinematics::SolutionInfo info;
        if (!arm_kinematics_->solveSingleArmIKWithInfo(
                pose, current_joint_pos, solution, info, arm_name, 50, 1e-4))
        {
            std::ostringstream oss;
            oss << "IK failed while calculating MoveC joint position for " << arm_name
                << " arm: solver=" << ArmKinematics::solverTypeName(info.usedSolver)
                << ", status=" << info.status
                << ", final_error=" << info.poseErrorNorm;
            last_error_ = oss.str();
            RCLCPP_ERROR(logger_, "%s", last_error_.c_str());
            return false;
        }

        current_joint_pos = solution;
        result.assign(solution.data(), solution.data() + solution.size());
        if (planner->isMotionOver())
        {
            completed_ = !dual_arm_mode_;
        }
        return true;
    }

    bool CartesianTrajectoryManager::validateCircleMsg(
        const arms_ros2_control_msgs::msg::CircleMessage& target_circle_msg,
        std::string& error_message)
    {
        // 检查时间模式下的时间参数
        if (target_circle_msg.time_mode && target_circle_msg.duration < 0.001)
        {
            error_message = "In time mode, a valid time parameter (duration > 0.001) "
                "must be provided";
            return false;
        }

        // 检查速度模式下的速度参数
        if (!target_circle_msg.time_mode)
        {
            if (fabs(target_circle_msg.max_linear_velocity) < min_val)
            {
                error_message =
                    "In speed mode, the maximum linear speed must be provided";
                return false;
            }
            if (fabs(target_circle_msg.max_angular_velocity) < min_val)
            {
                error_message = "When using attitude interpolation in velocity mode, the "
                    "maximum angular velocity must be provided";
                return false;
            }
            if (fabs(target_circle_msg.max_linear_acceleration) < min_val)
            {
                error_message =
                    "In speed mode, the maximum linear acceleration must be provided";
                return false;
            }
            if (fabs(target_circle_msg.max_angular_acceleration) < min_val)
            {
                error_message = "When using pose interpolation in velocity mode, the "
                    "maximum angular acceleration must be provided";
                return false;
            }
            if (fabs(target_circle_msg.max_linear_jerk) < min_val)
            {
                error_message =
                    "In speed mode, the maximum linear acceleration must be provided";
                return false;
            }
            if (fabs(target_circle_msg.max_angular_jerk) < min_val)
            {
                error_message = "When using attitude interpolation in velocity mode, the "
                    "maximum angular jerk must be provided";
                return false;
            }
        }

        if (target_circle_msg.use_three_point_method)
        {
            // 检查输入的中间点和终点是不是有问题
            if (fabs(target_circle_msg.midpoint.position.x) < min_val &&
                fabs(target_circle_msg.midpoint.position.y) < min_val &&
                fabs(target_circle_msg.midpoint.position.z) < min_val &&
                fabs(target_circle_msg.midpoint.orientation.x) < min_val &&
                fabs(target_circle_msg.midpoint.orientation.y) < min_val &&
                fabs(target_circle_msg.midpoint.orientation.z) < min_val &&
                fabs(target_circle_msg.midpoint.orientation.w) < min_val)
            {
                error_message = "The midpoint of the arc must be inputted";
                return false;
            }
            if (std::isnan(target_circle_msg.midpoint.position.x) ||
                std::isnan(target_circle_msg.midpoint.position.y) ||
                std::isnan(target_circle_msg.midpoint.position.z) ||
                std::isnan(target_circle_msg.midpoint.orientation.x) ||
                std::isnan(target_circle_msg.midpoint.orientation.y) ||
                std::isnan(target_circle_msg.midpoint.orientation.z) ||
                std::isnan(target_circle_msg.midpoint.orientation.w))
            {
                error_message = "The midpoint of the arc must be inputted";
                return false;
            }
            if (fabs(target_circle_msg.endpoint.position.x) < min_val &&
                fabs(target_circle_msg.endpoint.position.y) < min_val &&
                fabs(target_circle_msg.endpoint.position.z) < min_val &&
                fabs(target_circle_msg.endpoint.orientation.x) < min_val &&
                fabs(target_circle_msg.endpoint.orientation.y) < min_val &&
                fabs(target_circle_msg.endpoint.orientation.z) < min_val &&
                fabs(target_circle_msg.endpoint.orientation.w) < min_val)
            {
                error_message = "The endpoint of the arc must be inputted";
                return false;
            }
            if (std::isnan(target_circle_msg.endpoint.position.x) ||
                std::isnan(target_circle_msg.endpoint.position.y) ||
                std::isnan(target_circle_msg.endpoint.position.z) ||
                std::isnan(target_circle_msg.endpoint.orientation.x) ||
                std::isnan(target_circle_msg.endpoint.orientation.y) ||
                std::isnan(target_circle_msg.endpoint.orientation.z) ||
                std::isnan(target_circle_msg.endpoint.orientation.w))
            {
                error_message = "The endpoint of the arc must be inputted";
                return false;
            }
        }
        else
        {
            if (fabs(target_circle_msg.axis.x) < min_val && fabs(target_circle_msg.axis.y) < min_val &&
                fabs(target_circle_msg.axis.z) < min_val)
            {
                error_message = "Please input the axis of rotation for the circular arc";
                return false;
            }

            if (target_circle_msg.rotate_angle < min_val)
            {
                error_message = "Please input the axis of rotation for the circular arc";
                return false;
            }
        }

        return true;
    }

    planning::TrajectPoint CartesianTrajectoryManager::setCartesianPoint(
        const EndEffectorPose& input_point)
    {
        planning::TrajectPoint res;
        res.cart_pos = input_point.position;
        res.quaternion_point.q.w = input_point.quaternion.w();
        res.quaternion_point.q.x = input_point.quaternion.x();
        res.quaternion_point.q.y = input_point.quaternion.y();
        res.quaternion_point.q.z = input_point.quaternion.z();
        return res;
    }

    planning::TrajectPoint CartesianTrajectoryManager::setCartesianPoint(
        const geometry_msgs::msg::Pose& input_point)
    {
        planning::TrajectPoint res;
        res.cart_pos(0) = input_point.position.x;
        res.cart_pos(1) = input_point.position.y;
        res.cart_pos(2) = input_point.position.z;
        res.quaternion_point.q.x = input_point.orientation.x;
        res.quaternion_point.q.y = input_point.orientation.y;
        res.quaternion_point.q.z = input_point.orientation.z;
        res.quaternion_point.q.w = input_point.orientation.w;
        return res;
    }

    planning::TrajectoryParameter CartesianTrajectoryManager::setCartesianParameter(
        const arms_ros2_control_msgs::msg::LinearMessage& target_point_msg)
    {
        if (target_point_msg.time_mode)
        {
            // 时间模式
            double duration = target_point_msg.duration;
            duration = duration > 0.01 ? duration : 5.0;
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
            return time_mode_para;
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
            return speed_mode_para;
        }
    }

    planning::TrajectoryParameter CartesianTrajectoryManager::setCartesianParameter(
        const arms_ros2_control_msgs::msg::CircleMessage& target_circle_msg)
    {
        if (target_circle_msg.time_mode)
        {
            // 时间模式
            double duration = target_circle_msg.duration;
            duration = duration > 0.01 ? duration : 5.0;

            planning::TrajectoryParameter time_mode_para(
                target_circle_msg.use_slerp_for_orientation, duration);
            if (fabs(target_circle_msg.max_linear_velocity) > min_val &&
                fabs(target_circle_msg.max_linear_acceleration) > min_val &&
                fabs(target_circle_msg.max_linear_jerk) > min_val &&
                fabs(target_circle_msg.max_angular_velocity) > min_val &&
                fabs(target_circle_msg.max_angular_acceleration) > min_val &&
                fabs(target_circle_msg.max_angular_jerk) > min_val)
            {
                time_mode_para.max_linear_vel = target_circle_msg.max_linear_velocity;
                time_mode_para.max_linear_acc = target_circle_msg.max_linear_acceleration;
                time_mode_para.max_linear_jerk = target_circle_msg.max_linear_jerk;
                time_mode_para.max_angular_vel = target_circle_msg.max_angular_velocity;
                time_mode_para.max_angular_acc =
                    target_circle_msg.max_angular_acceleration;
                time_mode_para.max_angular_jerk = target_circle_msg.max_angular_jerk;
            }
            if (!target_circle_msg.use_three_point_method)
            {
                // 直接输入参数
                time_mode_para.circular_rotation_axis(0) = target_circle_msg.axis.x;
                time_mode_para.circular_rotation_axis(1) = target_circle_msg.axis.y;
                time_mode_para.circular_rotation_axis(2) = target_circle_msg.axis.z;

                time_mode_para.center_of_circle(0) = target_circle_msg.center.x;
                time_mode_para.center_of_circle(1) = target_circle_msg.center.y;
                time_mode_para.center_of_circle(2) = target_circle_msg.center.z;


                time_mode_para.no_need_to_calculate_circle_parameters_with_three_point =
                    true;
            }
            time_mode_para.circular_angle = target_circle_msg.rotate_angle;
            return time_mode_para;
        }
        else
        {
            // 速度模式
            planning::TrajectoryParameter speed_mode_para(
                target_circle_msg.max_linear_velocity,
                target_circle_msg.max_linear_acceleration,
                target_circle_msg.max_linear_jerk,
                target_circle_msg.max_angular_velocity,
                target_circle_msg.max_angular_acceleration,
                target_circle_msg.max_angular_jerk,
                target_circle_msg.use_slerp_for_orientation);

            if (!target_circle_msg.use_three_point_method)
            {
                // 直接输入参数
                speed_mode_para.circular_rotation_axis(0) = target_circle_msg.axis.x;
                speed_mode_para.circular_rotation_axis(1) = target_circle_msg.axis.y;
                speed_mode_para.circular_rotation_axis(2) = target_circle_msg.axis.z;

                speed_mode_para.center_of_circle(0) = target_circle_msg.center.x;
                speed_mode_para.center_of_circle(1) = target_circle_msg.center.y;
                speed_mode_para.center_of_circle(2) = target_circle_msg.center.z;


                speed_mode_para.no_need_to_calculate_circle_parameters_with_three_point =
                    true;
            }
            speed_mode_para.circular_angle = target_circle_msg.rotate_angle;
            return speed_mode_para;
        }
    }

} // namespace arms_controller_common
