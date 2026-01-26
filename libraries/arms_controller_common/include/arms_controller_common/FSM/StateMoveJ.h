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
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

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

    private:
        void updateParam();

        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        std::shared_ptr<GravityCompensation> gravity_compensation_;

        double duration_{3.0}; // Interpolation duration in seconds

        std::vector<double> start_pos_; // Starting position when entering state or starting new movement
        std::vector<double> target_pos_; // Target position to move to

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

        // Trajectory message subscription
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_subscription_;

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

        static constexpr double TARGET_EPSILON = 1e-6; // Tolerance for comparing target positions
    };
} // namespace arms_controller_common
