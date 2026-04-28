//
// Common StateMoveJ for Arm Controllers
//
#pragma once

#include "arms_controller_common/FSM/FSMState.h"
#include "arms_controller_common/utils/Interpolation.h"
#include "arms_controller_common/utils/GravityCompensation.h"
#include "arms_controller_common/utils/JointTrajectoryManager.h"
#include "arms_controller_common/utils/JointLimitsManager.h"
#include <vector>
#include <memory>
#include <mutex>
#include <string>
#include <functional>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <eigen3/Eigen/Dense>
#include "arms_controller_common/utils/WaistLiftingPlaner.h"
#include "arms_ros2_control_msgs/action/joint_trajectory.hpp"
#include "arms_ros2_control_msgs/msg/joint_waypoint.hpp"
#include "arms_ros2_control_msgs/srv/joint_trajectory.hpp"
#include "arms_controller_common/utils/Kinematics.h"
#include "arms_controller_common/utils/CartesianTrajectoryManager.h"
#include "arms_ros2_control_msgs/action/execute_linear.hpp"
#include "arms_ros2_control_msgs/action/movec_use_ik.hpp"
#include "arms_ros2_control_msgs/srv/execute_linear.hpp"
#include "arms_ros2_control_msgs/srv/movec_use_ik.hpp"

namespace arms_controller_common
{
    /**
     * @brief StateMoveJ - Moves arm joints to target position with smooth interpolation
     * 
     * Supports:
     * - Smooth interpolation to target joint positions
     * - Thread-safe target position updates
     * - Optional gravity compensation (if hardware supports)
     * - Configurable interpolation duration
     */
    class StateMoveJ : public FSMState
    {
    public:
        /**
         * @brief Constructor
         * @param ctrl_interfaces Control interfaces
         * @param node ROS lifecycle node for parameter access
         * @param joint_names Joint names (empty vector to auto-initialize from interfaces)
         * @param gravity_compensation Optional gravity compensation utility (nullptr if not needed)
         */
        explicit StateMoveJ(CtrlInterfaces& ctrl_interfaces,
                            const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node = nullptr,
                            const std::vector<std::string>& joint_names = {},
                            const std::shared_ptr<GravityCompensation>& gravity_compensation = nullptr);

        void enter() override;
        void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        void exit() override;
        FSMStateName checkChange() override;

        /**
         * @brief Set target position (thread-safe)
         * @param target_pos Target joint positions
         */
        void setTargetPosition(const std::vector<double>& target_pos);

        /**
         * @brief Set target position for joints with specific prefix (thread-safe)
         * @param prefix Joint name prefix (e.g., "left" for "left_joint1", "left_joint2", etc.)
         * @param target_pos Target joint positions (should match the number of joints with the prefix)
         */
        void setTargetPosition(const std::string& prefix, const std::vector<double>& target_pos);

        /**
         * @brief Set joint names (should be called before setupSubscriptions if joint names are known)
         * @param joint_names Vector of joint names
         */
        void setJointNames(const std::vector<std::string>& joint_names);

        /**
         * @brief Setup ROS subscriptions for target position topics
         * @param topic_base_name Base name for topics (default: "target_joint_position")
         * @param enable_prefix_topics If true, automatically detect prefixes (left, right, body) and subscribe to corresponding topics
         *                             If false, only subscribe to the base topic (default: false)
         */
        void setupSubscriptions(const std::string& topic_base_name = "target_joint_position",
                                bool enable_prefix_topics = false);

        /**
         * @brief Update joint limits from URDF
         * 
         * Updates joint limits by parsing the provided URDF string.
         * Joint limits manager is automatically created in constructor.
         * 
         * @param robot_description URDF XML string
         */
        void updateJointLimitsFromURDF(const std::string& robot_description);

        /**
         * @brief Set joint limit checker callback function
         * 
         * This callback will be called whenever a target position is set to apply joint limits.
         * The callback should take the target position vector and return a clamped position vector.
         * 
         * Example usage:
         * - For ocs2_arm_controller: Get limits from Pinocchio model
         * 
         * @param limit_checker Callback function: std::vector<double>(const std::vector<double>& target_pos)
         *                       Input: target position vector, Output: clamped position vector
         *                       If nullptr, joint limit checking is disabled
         * 
         * @note This will override any automatic limits set by enableJointLimitsFromURDF()
         */
        void setJointLimitChecker(std::function<std::vector<double>(const std::vector<double>&)> limit_checker);

        /**
         * @brief Set trajectory from ROS2 JointTrajectory message
         * @param trajectory JointTrajectory message containing waypoints
         * @note Current joint position will be used as the first waypoint
         * @note Requires at least 2 points in trajectory (total >= 3 with current position)
         * @note Uses trajectory_duration from JointTrajectoryManager for total duration
         */
        void setTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory);

        /**
         * @brief Setup ROS subscription for trajectory messages
         * @param topic_name Topic name for trajectory messages (default: "target_joint_trajectory")
         */
        void setupTrajectorySubscription(const std::string& topic_name = "target_joint_trajectory");

        // ---------- added for moveL / waist motion ----------
        /**
         * @brief Switch to MOVEJ vs MOVEL mode (default MOVEJ)
         */
        enum class MotionMode
        {
            MOVEJ,
            MOVECARTESIAN,
            WAIST_CONTROL
        };

        /**
         * @brief Set motion type for next command
         */
        void setMotionMode(MotionMode mode) { motion_mode_ = mode; }

        /** 
         * @brief 控制腰部升降指令，让腰部相对当前位置移动lifting_distance的距离
         */
        bool moveWaistLifting(double lifting_distance);

        /**
         * @brief 控制腰部升降指令，command=0 停止，command=1 上升 ，command=2 下降
         */
        bool setWaistLiftingCommand(int command);

        /**
         * @brief 控制腰部升降速度系数，factor取值建议[-1, 1]
         * 实际目标速度 = factor * default_waist_para_[0]
         */
        bool setWaistLiftingFactor(double factor);

        /**
         * @brief 控制腰部转向速度系数，factor取值建议[-1, 1]
         * 实际目标速度 = factor * default_waist_para_[0]
         */
        bool setWaistTurningFactor(double factor);


        /**
         * @brief Setup joint trajectory service
         * @param service_name Service name (default: "joint_trajectory")
         */
        void setupJointTrajectoryService(const std::string& service_name = "joint_trajectory");
        void setupJointTrajectoryAction(const std::string& action_name = "joint_trajectory");

        void setKinematicsSolver(const std::shared_ptr<ArmKinematics>& kinematics = nullptr);
        // 在 StateMoveJ.h 的 public 部分添加
        /**
        * @brief Setup linear trajectory service for MoveL planning
        * @param service_name Service name (default: "execute_linear")
        */
        void setupLinearTrajectoryService(const std::string& service_name = "execute_linear");

        /**
        * @brief Setup linear trajectory action for MoveL planning and execution
        * @param action_name Action name (default: "execute_linear")
        */
        void setupLinearTrajectoryAction(const std::string& action_name = "execute_linear");

        /**
        * @brief Setup linear trajectory service for MoveL planning
        * @param service_name Service name (default: "execute_linear")
        */
        void setupCircleTrajectoryService(const std::string& service_name = "execute_circle_use_ik");

        /**
        * @brief Setup circle trajectory action for MoveC planning and execution
        * @param action_name Action name (default: "execute_circle_use_ik")
        */
        void setupCircleTrajectoryAction(const std::string& action_name = "execute_circle_use_ik");

    private:
        void updateParam();

        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        std::shared_ptr<GravityCompensation> gravity_compensation_;

        double duration_{3.0}; // Interpolation duration in seconds

        std::vector<double> start_pos_; // Starting position when entering state or starting new movement
        std::vector<double> target_pos_; // Target position to move to
        std::vector<double> hold_positions_; // Position cache used to hold uncommanded joints

        std::mutex target_mutex_; // Mutex for thread-safe target position updates
        bool has_target_{false}; // Whether a target position has been set
        bool interpolation_active_{false}; // Whether interpolation is currently active
        bool state_active_{false}; // Whether the state is currently active (entered)

        // Joint filtering support
        std::vector<std::string> joint_names_; // Joint names extracted from interfaces
        std::string active_prefix_; // Active prefix for filtered control (empty means all joints)
        std::vector<bool> joint_mask_; // Mask indicating which joints to control (true = control, false = hold)
        bool use_prefix_filter_{false}; // Whether to use prefix filtering

        // ROS subscriptions
        std::string topic_base_name_; // Base name for target position topics
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_position_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_position_left_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_position_right_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_position_body_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_position_head_subscription_;

        // Trajectory message subscription
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_subscription_;

        // Current target joint publisher (shared convention across controllers)
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr current_target_joint_publisher_;
        void publishCurrentTargetJoint(const std::vector<double>& target_positions);

        // Joint limit checking
        std::shared_ptr<JointLimitsManager> joint_limits_manager_;
        // Optional joint limits manager for URDF-based limits
        std::function<std::vector<double>(const std::vector<double>&)> joint_limit_checker_;
        // Optional joint limit checker callback

        // Interpolation configuration
        InterpolationType interpolation_type_{InterpolationType::TANH};
        double tanh_scale_{3.0};

        // Unified trajectory manager
        JointTrajectoryManager trajectory_manager_;

        /**
         * @brief Update joint mask based on prefix
         * @param prefix Joint name prefix to filter
         */
        void updateJointMask(const std::string& prefix);

        /**
         * @brief Apply joint limit checking to target position
         * @param target_pos Input target position to check
         * @param log_message Optional log message prefix (empty string to skip logging)
         * @return Clamped target position after applying joint limits
         */
        std::vector<double> applyJointLimits(const std::vector<double>& target_pos,
                                             const std::string& log_message = "");

        /**
         * @brief Validate trajectory message
         * @param trajectory Trajectory message to validate
         * @return True if valid, false otherwise
         */
        bool validateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory);

        /**
         * @brief Map trajectory joint names to controller joint indices
         * @param trajectory_joint_names Joint names from trajectory message
         * @return Vector of controller joint indices (empty if mapping fails)
         */
        std::vector<size_t> mapJointNames(const std::vector<std::string>& trajectory_joint_names);

        /**
         * @brief Calculate segment durations based on path length
         * @param waypoints Vector of waypoints
         * @param total_duration Total trajectory duration
         * @return Vector of segment durations (size = waypoints.size() - 1)
         */
        std::vector<double> calculateSegmentDurations(
            const std::vector<std::vector<double>>& waypoints,
            double total_duration);

        /**
         * @brief Capture the latest commanded/observed joint positions for hold behavior
         */
        void refreshHoldPositions();

        static constexpr double TARGET_EPSILON = 1e-6; // Tolerance for comparing target positions

        MotionMode motion_mode_{MotionMode::MOVEJ};
        // indices of joints that belong to body/waist (determined from joint_names_)
        std::vector<size_t> body_joint_indices_;


        // Waist lifting support
        std::shared_ptr<arms_controller_common::WaistLiftingPlaner> waist_lifting_planer_;
        bool waist_lifting_active_{false};
        std::shared_ptr<arms_controller_common::WaistLiftingPlaner> waist_turning_planer_;
        bool waist_turning_active_{false};
        double waist_lifting_duration_{3.0};
        Eigen::Vector3d default_waist_para_;
        void updateWaistParam();

        double last_waist_factor_{0.0};
        double last_waist_turning_factor_{0.0};
        static constexpr double waist_factor_epsilon_{1e-6};
        size_t waist_turning_joint_index_{0};

        std::vector<std::string> waist_joint_names_; // 腰部关节名称（前三个关节）
        void setWaistLiftingPlaner();

        /**
         * @brief Update waist lifting limits from joint limits manager
        */
        void updateWaistLiftingLimits();
        size_t waist_joint_count_ = 3;
        /**
         * @brief Get current waist joint angles
         * @return Current waist joint angles as Eigen::Vector3d
         */

        /**
         * @brief Get current waist joint angles
         */
        Eigen::VectorXd getCurrentWaistAngles();
        std::vector<double> applyWaistJointLimits(const std::vector<double>& waist_positions);


        // Service server
        rclcpp::Service<arms_ros2_control_msgs::srv::JointTrajectory>::SharedPtr joint_trajectory_service_;

        using JointTrajectoryAction = arms_ros2_control_msgs::action::JointTrajectory;
        using JointTrajectoryGoalHandle = rclcpp_action::ServerGoalHandle<JointTrajectoryAction>;
        rclcpp_action::Server<JointTrajectoryAction>::SharedPtr joint_trajectory_action_server_;
        std::shared_ptr<JointTrajectoryGoalHandle> active_joint_trajectory_goal_;
        rclcpp::Time joint_trajectory_action_start_time_;
        double joint_trajectory_planned_duration_{0.0};
        bool joint_trajectory_action_active_{false};

        // Service handler
        void handleJointTrajectory(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<arms_ros2_control_msgs::srv::JointTrajectory::Request> request,
            const std::shared_ptr<arms_ros2_control_msgs::srv::JointTrajectory::Response> response);
        rclcpp_action::GoalResponse handleJointTrajectoryGoal(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const JointTrajectoryAction::Goal> goal);
        rclcpp_action::CancelResponse handleJointTrajectoryCancel(
            const std::shared_ptr<JointTrajectoryGoalHandle> goal_handle);
        void handleJointTrajectoryAccepted(
            const std::shared_ptr<JointTrajectoryGoalHandle> goal_handle);
        bool startJointTrajectoryRequest(
            const std::vector<std::string>& joint_names,
            const std::vector<arms_ros2_control_msgs::msg::JointWaypoint>& waypoints,
            std::string& message,
            double& planned_duration);
        void publishJointTrajectoryFeedback();
        void finishJointTrajectoryAction(bool success, bool canceled, const std::string& message);

        // 辅助函数
        bool validateJointNames(const std::vector<std::string>& request_joint_names, std::string& error_msg);
        std::vector<double> getCurrentJointPositions(const std::vector<std::string>& joint_names);
        std::vector<double> mapToFullJointPositions(
            const std::vector<std::string>& request_joint_names,
            const std::vector<double>& request_positions);

        //增加movel相关的代码
        CartesianTrajectoryManager cartesian_manager_;

        using ExecuteLinearAction = arms_ros2_control_msgs::action::ExecuteLinear;
        using ExecuteLinearGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteLinearAction>;
        using MovecUseIKAction = arms_ros2_control_msgs::action::MovecUseIK;
        using MovecUseIKGoalHandle = rclcpp_action::ServerGoalHandle<MovecUseIKAction>;

        // Service server for MoveL
        rclcpp::Service<arms_ros2_control_msgs::srv::ExecuteLinear>::SharedPtr linear_trajectory_service_;

        // Action server for MoveL
        rclcpp_action::Server<ExecuteLinearAction>::SharedPtr linear_trajectory_action_server_;
        std::shared_ptr<ExecuteLinearGoalHandle> active_linear_goal_;
        rclcpp::Time linear_action_start_time_;
        double linear_action_estimated_duration_{0.0};
        bool linear_action_active_{false};

        // Service handler for MoveL
        void handleLinearTrajectory(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<arms_ros2_control_msgs::srv::ExecuteLinear::Request> request,
            const std::shared_ptr<arms_ros2_control_msgs::srv::ExecuteLinear::Response> response);

        rclcpp_action::GoalResponse handleLinearGoal(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const ExecuteLinearAction::Goal> goal);
        rclcpp_action::CancelResponse handleLinearCancel(
            const std::shared_ptr<ExecuteLinearGoalHandle> goal_handle);
        void handleLinearAccepted(
            const std::shared_ptr<ExecuteLinearGoalHandle> goal_handle);
        bool startLinearTrajectory(
            const arms_ros2_control_msgs::msg::LinearMessage& linear_params,
            std::string& message,
            double& estimated_duration);
        void publishLinearFeedback();
        void finishLinearAction(bool success, bool canceled, const std::string& message);

        // 添加 MoveL 相关的辅助方法
        bool validateLinearRequest(const arms_ros2_control_msgs::msg::LinearMessage& linear_params,
                                   std::string& error_msg);

        bool move_cartesian_active_{false};
        std::vector<std::string> move_cartesian_joint_names_;
        //Service for movec
        rclcpp::Service<arms_ros2_control_msgs::srv::MovecUseIK>::SharedPtr circle_trajectory_service_;

        // Action server for MoveC
        rclcpp_action::Server<MovecUseIKAction>::SharedPtr circle_trajectory_action_server_;
        std::shared_ptr<MovecUseIKGoalHandle> active_circle_goal_;
        rclcpp::Time circle_action_start_time_;
        double circle_action_estimated_duration_{0.0};
        bool circle_action_active_{false};

        // Service handler for Movec
        void handleCircleTrajectory(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<arms_ros2_control_msgs::srv::MovecUseIK::Request> request,
            const std::shared_ptr<arms_ros2_control_msgs::srv::MovecUseIK::Response> response);

        rclcpp_action::GoalResponse handleCircleGoal(
            const rclcpp_action::GoalUUID& uuid,
            std::shared_ptr<const MovecUseIKAction::Goal> goal);
        rclcpp_action::CancelResponse handleCircleCancel(
            const std::shared_ptr<MovecUseIKGoalHandle> goal_handle);
        void handleCircleAccepted(
            const std::shared_ptr<MovecUseIKGoalHandle> goal_handle);
        bool startCircleTrajectory(
            const arms_ros2_control_msgs::msg::CircleMessage& circle_params,
            std::string& message,
            double& estimated_duration);
        void publishCircleFeedback();
        void finishCircleAction(bool success, bool canceled, const std::string& message);

        // 添加 MoveL 相关的辅助方法
        bool validateCircleRequest(const arms_ros2_control_msgs::msg::CircleMessage& circle_params,
                                   std::string& error_msg);
    };
} // namespace arms_controller_common
