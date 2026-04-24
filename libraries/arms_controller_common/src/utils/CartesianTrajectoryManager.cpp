#include "arms_controller_common/utils/CartesianTrajectoryManager.h"
#include <Eigen/Geometry>
#include <arms_ros2_control_msgs/msg/linear_message.hpp>
#include <iostream>

#include "../../include/arms_controller_common/utils/Kinematics.h"

namespace arms_controller_common
{
    CartesianTrajectoryManager::CartesianTrajectoryManager()
        : logger_(rclcpp::get_logger("cartesian_trajectory_manager"))
    {
        save_data_ = true;
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
        if (!movel_planner_)
        {
            movel_planner_ = std::make_unique<planning::moveL>();
        }

        current_joint_pos_ = Eigen::Map<const Eigen::VectorXd>(
            start_joint_pos.data(), start_joint_pos.size());

        EndEffectorPose start_pose =
            arm_kinematics_->computeSingleEndEffectorPose(js, arm_type_);

        // 初始化movel_planner_
        movel_planner_ = std::make_unique<planning::moveL>();
        planning::TrajectPoint ps = setCartesianPoint(start_pose);

        planning::TrajectPoint pe = setCartesianPoint(target_point_msg.endpoint);

        planning::TrajectoryParameter para = setCartesianParameter(target_point_msg);
        planning::TrajectoryInitParameters init_para(ps, pe, para, period);
        if (!movel_planner_->init(init_para))
        {
            std::cout << "Failed to initialize moveL planner" << std::endl;
            return false;
        }
        ArmKinematics::SolverType ik_tp;
        if (target_point_msg.ik_type=="BFGS")
        {
            ik_tp = ArmKinematics::SolverType::BFGS;
        }else if (target_point_msg.ik_type=="DLS")
        {
            ik_tp = ArmKinematics::SolverType::DLS;
        }else if (target_point_msg.ik_type=="SDK")
        {
            ik_tp=ArmKinematics::SolverType::SDK;
        }
        else
        {
            ik_tp = ArmKinematics::SolverType::AUTO;
        }
        arm_kinematics_->setSolverType(ik_tp);
        movel_planner_->setRealStartTime(0.0);
        planningTime_ = movel_planner_->getTotalTime();
        path_type_ = PathType::LINE;
        return true;
    }

    bool CartesianTrajectoryManager::planSingleArmMoveC(
        const std::vector<double>& start_joint_pos,
        const arms_ros2_control_msgs::msg::CircleMessage& target_circle_msg,
        const double period)
    {
        if (!arm_kinematics_)
        {
            std::cout << "kinematics is not set" << std::endl;
            return false;
        }
        std::string error_msg;
        if (!validateCircleMsg(target_circle_msg, error_msg))
        {
            std::cout << error_msg << std::endl;
            return false;
        }

        if (!movec_planner_)
        {
            movec_planner_ = std::make_unique<planning::CircularCurver>();
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
            std::cout << "invalid arm type: " << arm_type_ << std::endl;
            return false;
        }
        current_joint_pos_ = Eigen::Map<const Eigen::VectorXd>(
            start_joint_pos.data(), start_joint_pos.size());

        EndEffectorPose start_pose =
            arm_kinematics_->computeSingleEndEffectorPose(js, arm_type_);

        // 更新起点
        planning::TrajectPoint ps = setCartesianPoint(start_pose);
        // 更新终点
        planning::TrajectPoint pe = setCartesianPoint(target_circle_msg.endpoint);

        planning::TrajectoryParameter para = setCartesianParameter(target_circle_msg);

        if (!target_circle_msg.use_three_point_method)
        {
            planning::TrajectoryInitParameters init_para(ps, pe, para, period);
            if (!movec_planner_->init(init_para))
            {
                return false;
            }
        }
        else
        {
            planning::TrajectPoint pm = setCartesianPoint(target_circle_msg.midpoint);
            planning::TrajectoryInitParameters init_para(ps, pm, pe, para, period);
            if (!movec_planner_->init(init_para))
            {
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
        movec_planner_->setRealStartTime(period);
        planningTime_ = movec_planner_->getTotalTime();
        path_type_ = PathType::CIRCLE;
        return true;
    }

    bool CartesianTrajectoryManager::getNextJointPos(std::vector<double>& res)
    {
        if (movel_planner_&& path_type_==PathType::LINE)
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
            //增加安全检查，为了避免求出的逆解不是一个合适的构型，距离当前解的构型较远，这里增加检查，如果求出的逆解距离当前解很远就返回false
            // Eigen::VectorXd delta_joint = solution - current_joint_pos_;

            // for (int i = 0; i < delta_joint.size(); ++i)
            // {
            //     if (fabs(delta_joint(i)) > 0.05)
            //     {
            //         std::cerr << "The inverse solution position is too far from the current point" << std::endl;
            //         return false;
            //     }
            // }
            // 更新当前关节角度
            current_joint_pos_ = solution;
            //存储一下数据
            if (save_data_)
            {
                std::string file_name = "/home/lina/lina/data/movel_data.csv";
                savedata(file_name, movel_point, solution);
            }

            res.clear();
            for (size_t i = 0; i < static_cast<size_t>(solution.size()); i++)
            {
                res.push_back(current_joint_pos_(i));
            }
            return true;
        }
        else
        if (movec_planner_&& path_type_
        ==
        PathType::CIRCLE
        )
        {
            planning::TrajectPoint movec_point = movec_planner_->run();
            if (movec_planner_->isMotionOver())
            {
                completed_ = true;
            }

            EndEffectorPose pose;
            pose.position = movec_point.cart_pos;
            Eigen::Quaterniond q(
                movec_point.quaternion_point.q.w, movec_point.quaternion_point.q.x,
                movec_point.quaternion_point.q.y, movec_point.quaternion_point.q.z);
            pose.setQuaternion(q);
            Eigen::VectorXd solution;
            if (!arm_kinematics_->solveSingleArmIK(pose, current_joint_pos_, solution,
                                                   arm_type_, 50, 1e-4))
            {
                RCLCPP_ERROR(logger_, "Failed to calculate the inverse solution for the "
                             "current Cartesian point: position:[%f,%f,%f],quaternion:[%f,%f,%f,%f]", pose.position[0],
                             pose.position[1], pose.position[2], pose.quaternion.x(), pose.quaternion.y(),
                             pose.quaternion.z(), pose.quaternion.w());
                return false;
            }
            //增加安全检查，为了避免求出的逆解不是一个合适的构型，距离当前解的构型较远，这里增加检查，如果求出的逆解距离当前解很远就返回false
            // Eigen::VectorXd delta_joint = solution - current_joint_pos_;

            // for (int i = 0; i < delta_joint.size(); ++i)
            // {
            //     if (fabs(delta_joint(i)) > 0.05)
            //     {
            //         std::cerr << "The inverse solution position is too far from the current point" << std::endl;
            //         return false;
            //     }
            // }
            if (save_data_)
            {
                std::string file_name = "/home/lina/lina/data/movec_data.csv";
                savedata(file_name, movec_point, solution);
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
        else
        {
            std::cout << "invalid arm name: " << arm_name << std::endl;
            return {};
        }
    }

    void CartesianTrajectoryManager::clearPlanner()
    {
        movel_planner_.reset();
        movec_planner_.reset();
        completed_ = false;
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
                target_circle_msg.max_angular_jerk, true);

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

    void CartesianTrajectoryManager::savedata(const std::string& filepath, const planning::TrajectPoint& point,
                                              const Eigen::VectorXd& joint_angle)
    {
        // 检查文件是否存在
        bool file_exists = false;
        std::ifstream infile(filepath);
        if (infile.good())
        {
            file_exists = true;
        }
        infile.close();

        // 以追加模式打开文件（如果文件不存在会自动创建）
        std::ofstream outfile;
        outfile.open(filepath, std::ios::out | std::ios::app);

        if (!outfile.is_open())
        {
            std::cerr << "错误：无法打开文件 " << filepath << std::endl;
            return;
        }

        // 如果文件不存在（刚创建的空文件），则写入表头
        if (!file_exists)
        {
            // 假设关节数量固定为7个（人形机械臂常见自由度）
            // 您可以根据实际关节数量修改这里的表头
            outfile << "time" << "," << "x" << "," << "y" << "," << "z" << "," << "qw" << "," << "qx" << "," << "qy" <<
                "," << "qz" << "," << "joint0" << "," << "joint1" << "," << "joint2" << "," << "joint3" << "," <<
                "joint4" << "," << "joint5" << "," << "joint6" << std::endl;
        }
        outfile << point.time << "," << point.cart_pos(0) << "," << point.cart_pos(1) << "," << point.cart_pos(2) << ","
            << point.quaternion_point.q.w << "," << point.quaternion_point.q.x << "," << point.quaternion_point.q.y <<
            "," << point.quaternion_point.q.z << "," << joint_angle(0) << "," << joint_angle(1) << "," << joint_angle(2)
            << "," << joint_angle(3) << "," <<
            joint_angle(4) << "," << joint_angle(5) << "," << joint_angle(6) << std::endl;
        outfile.close();
    }
} // namespace arms_controller_common
