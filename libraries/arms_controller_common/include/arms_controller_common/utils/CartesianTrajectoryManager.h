#pragma once
#include "arms_controller_common/utils/Kinematics.h"
#include "lina_planning/planning/path_planner/circular_curve.h"
#include "lina_planning/planning/path_planner/movel.h"
#include <Eigen/Dense>
#include <arms_ros2_control_msgs/msg/circle_message.hpp>
#include <arms_ros2_control_msgs/msg/linear_message.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace arms_controller_common
{
    class CartesianTrajectoryManager
    {
    public:
        CartesianTrajectoryManager();
        ~CartesianTrajectoryManager();

        void setKinematicsSolver(
            const std::shared_ptr<ArmKinematics>& kinematics = nullptr);

        bool planSingleArmMoveL(
            const std::vector<double>& start_joint_pos,
            const arms_ros2_control_msgs::msg::LinearMessage& target_point_msg,
            const double period = 0.01);
        bool planSingleArmMoveC(
            const std::vector<double>& start_joint_pos,
            const arms_ros2_control_msgs::msg::CircleMessage& target_circle_msg,
            const double period = 0.01);
        bool planDualArmMoveL(
            const std::vector<double>& start_joint_pos,
            const arms_ros2_control_msgs::msg::LinearMessage& target_point_msg,
            const double period = 0.01);
        bool planDualArmMoveC(
            const std::vector<double>& start_joint_pos,
            const arms_ros2_control_msgs::msg::CircleMessage& target_circle_msg,
            const double period = 0.01);

        bool getNextJointPos(std::vector<double>& res);

        // 在 public 部分添加
        double getPlanningTime() const { return planningTime_; }
        bool isCompleted() const { return completed_; }
        std::string getLastError() const { return last_error_; }
        std::vector<std::string> getMovelJointNames(std::string arm_name);
        void clearPlanner();

        bool hasKinematics() { return arm_kinematics_ != nullptr; }

    private:
        enum PathType { LINE, CIRCLE };

        PathType path_type_;

        std::shared_ptr<ArmKinematics> arm_kinematics_;
        std::unique_ptr<planning::moveL> left_movel_planner_;
        std::unique_ptr<planning::moveL> right_movel_planner_;
        std::unique_ptr<planning::CircularCurver> left_movec_planner_;
        std::unique_ptr<planning::CircularCurver> right_movec_planner_;
        Eigen::VectorXd left_current_joint_pos_;
        Eigen::VectorXd right_current_joint_pos_;
        std::string arm_type_;
        bool dual_arm_mode_{false};
        bool left_arm_active_{false};
        bool right_arm_active_{false};
        bool completed_ = false;
        std::string last_error_;
        double planningTime_ = 0.0;

        //辅助函数
        bool validateCircleMsg(
            const arms_ros2_control_msgs::msg::CircleMessage& target_circle_msg, std::string& error_message);
        planning::TrajectPoint setCartesianPoint(const EndEffectorPose& input_point);
        planning::TrajectPoint setCartesianPoint(const geometry_msgs::msg::Pose& input_point);
        planning::TrajectoryParameter setCartesianParameter(
            const arms_ros2_control_msgs::msg::LinearMessage& target_point_msg);
        planning::TrajectoryParameter setCartesianParameter(
            const arms_ros2_control_msgs::msg::CircleMessage& target_circle_msg);
        bool validatePose(const geometry_msgs::msg::Pose& pose,
                          const std::string& pose_name,
                          std::string& error_message) const;
        bool stepLineArm(const std::string& arm_name,
                         std::unique_ptr<planning::moveL>& planner,
                         Eigen::VectorXd& current_joint_pos,
                         std::vector<double>& result);
        bool stepCircleArm(const std::string& arm_name,
                           std::unique_ptr<planning::CircularCurver>& planner,
                           Eigen::VectorXd& current_joint_pos,
                           std::vector<double>& result);
        double min_val = 1.0e-6;
        rclcpp::Logger logger_;
    };
} // namespace arms_controller_common
