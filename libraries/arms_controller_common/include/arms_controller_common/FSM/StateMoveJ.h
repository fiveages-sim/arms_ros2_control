//
// Common StateMoveJ for Arm Controllers
//
#pragma once

#include "arms_controller_common/FSM/FSMState.h"
#include "arms_controller_common/utils/GravityCompensation.h"
#include <vector>
#include <memory>
#include <mutex>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

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
         * @param logger ROS logger
         * @param duration Interpolation duration in seconds
         * @param gravity_compensation Optional gravity compensation utility (nullptr if not needed)
         */
        explicit StateMoveJ(CtrlInterfaces& ctrl_interfaces,
                           const rclcpp::Logger& logger,
                           double duration = 3.0,
                           std::shared_ptr<GravityCompensation> gravity_compensation = nullptr);

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
         * @param node ROS lifecycle node for creating subscriptions
         * @param topic_base_name Base name for topics (default: "target_joint_position")
         * @param enable_prefix_topics If true, automatically detect prefixes (left, right, body) and subscribe to corresponding topics
         *                             If false, only subscribe to the base topic (default: false)
         */
        void setupSubscriptions(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, 
                               const std::string& topic_base_name = "target_joint_position",
                               bool enable_prefix_topics = false);

    private:
        rclcpp::Logger logger_;
        std::shared_ptr<GravityCompensation> gravity_compensation_;
        
        double duration_;                 // Interpolation duration in seconds
        double percent_{0.0};             // Interpolation progress (0.0 to 1.0)
        
        std::vector<double> start_pos_;   // Starting position when entering state or starting new movement
        std::vector<double> target_pos_;  // Target position to move to
        
        std::mutex target_mutex_;         // Mutex for thread-safe target position updates
        bool has_target_{false};          // Whether a target position has been set
        bool interpolation_active_{false}; // Whether interpolation is currently active
        bool state_active_{false};        // Whether the state is currently active (entered)
        
        // Joint filtering support
        std::vector<std::string> joint_names_;  // Joint names extracted from interfaces
        std::string active_prefix_;             // Active prefix for filtered control (empty means all joints)
        std::vector<bool> joint_mask_;          // Mask indicating which joints to control (true = control, false = hold)
        bool use_prefix_filter_{false};         // Whether to use prefix filtering
        
        // ROS subscriptions
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;     // ROS lifecycle node for subscriptions
        std::string topic_base_name_;            // Base name for target position topics
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_position_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_position_left_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_position_right_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_position_body_subscription_;
        bool subscriptions_setup_{false};        // Whether subscriptions have been set up
        
        /**
         * @brief Initialize joint names from interfaces
         */
        void initializeJointNames();
        
        /**
         * @brief Update joint mask based on prefix
         * @param prefix Joint name prefix to filter
         */
        void updateJointMask(const std::string& prefix);
        
        static constexpr double TARGET_EPSILON = 1e-6;  // Tolerance for comparing target positions
    };
} // namespace arms_controller_common

