#pragma once
#include <rclcpp/rclcpp.hpp>
#include "lina_planning/planning/path_planner/movel.h"
#include "arms_controller_common/utils/Kinematics.h"
#include <arms_ros2_control_msgs/msg/linear_message.hpp>
#include <Eigen/Dense>
#include <memory>
#include <vector>


namespace arms_controller_common
{
    class CartesianTrajectoryManager
    {
    public:
        CartesianTrajectoryManager();
        ~CartesianTrajectoryManager();

        void setKinematicsSolver(const std::shared_ptr<ArmKinematics>& kinematics = nullptr);

        bool planSingleArmMoveL(const std::vector<double>& start_joint_pos,
                                const arms_ros2_control_msgs::msg::LinearMessage& target_point_msg,
                                const double period = 0.01);

        bool getNextJointPos(std::vector<double>& res);

        // 在 public 部分添加
        double getPlanningTime() const { return planningTime_; }
        bool isCompleted() const { return completed_; }
        std::vector<std::string> getMovelJointNames(std::string arm_name);
        void clearPlanner();

        bool hasKinematics() { return arm_kinematics_ != nullptr; }

    private:
        std::shared_ptr<ArmKinematics> arm_kinematics_;
        std::unique_ptr<planning::moveL> movel_planner_;
        Eigen::VectorXd current_joint_pos_;
        std::string arm_type_;
        bool completed_ = false;
        double planningTime_ = 0.0;

        double min_val = 1.0e-6;
        rclcpp::Logger logger_;
    };
}
