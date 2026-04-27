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
        std::unique_ptr<planning::moveL> movel_planner_;
        std::unique_ptr<planning::CircularCurver> movec_planner_;
        Eigen::VectorXd current_joint_pos_;
        std::string arm_type_;
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
        bool save_data_ = false;
        void savedata(const std::string &filepath, const planning::TrajectPoint &point,
                      const Eigen::VectorXd &joint_angle);
        double min_val = 1.0e-6;
        rclcpp::Logger logger_;
    };
} // namespace arms_controller_common
