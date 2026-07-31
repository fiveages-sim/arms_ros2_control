//
// Joint Limits Manager - Common utility for parsing and applying joint limits
//
#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <limits>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace arms_controller_common
{
    /**
     * @brief URDF joint motion type for controllable joints
     */
    enum class JointMotionType
    {
        UNKNOWN,
        REVOLUTE,
        CONTINUOUS,
        PRISMATIC
    };

    /**
     * @brief Joint limits structure
     */
    struct JointLimits
    {
        double lower = -std::numeric_limits<double>::max();
        double upper = std::numeric_limits<double>::max();
        bool initialized = false;
        JointMotionType motion_type = JointMotionType::UNKNOWN;
    };

    /**
     * @brief Joint Limits Manager - Utility class for parsing and applying joint limits
     * 
     * This class provides a common interface for:
     * - Parsing joint limits from URDF robot description
     * - Applying limits to target joint positions (clamping)
     * - Querying limits for specific joints
     * - Creating limit checker callback functions
     * 
     * Usage examples:
     * 1. For basic_joint_controller: Parse from URDF and apply to target positions
     * 2. For rviz control plugin: Parse limits and validate user input
     * 3. For arms_target_manager: Validate joint angles before sending commands
     */
    class JointLimitsManager
    {
    public:
        /**
         * @brief Constructor
         * @param logger ROS logger for logging messages (optional, can be nullptr)
         */
        explicit JointLimitsManager(rclcpp::Logger logger = rclcpp::get_logger("joint_limits_manager"));

        /**
         * @brief Parse joint limits and motion types from URDF robot description
         * 
         * Uses urdf::Model to extract motion type and position limits for the
         * requested joints. Return value is the number of joints with successfully
         * parsed finite position limits (continuous joints contribute type only).
         * 
         * @param robot_description URDF XML string
         * @param joint_names List of joint names to parse limits for
         *                     If empty, no joints are parsed
         * @param log_summary If true, emit one INFO summary after parsing (disable for RViz etc.)
         * @return Number of joints with successfully parsed position limits
         */
        size_t parseFromURDF(const std::string& robot_description, 
                            const std::vector<std::string>& joint_names = {},
                            bool log_summary = true);

        /**
         * @brief Set joint limits manually
         * @param joint_name Name of the joint
         * @param lower Lower limit (minimum position)
         * @param upper Upper limit (maximum position)
         *
         * Does not invent a motion type; leaves motion_type as UNKNOWN unless
         * previously set by parseFromURDF().
         */
        void setJointLimits(const std::string& joint_name, double lower, double upper);

        /**
         * @brief Get joint limits for a specific joint
         * @param joint_name Name of the joint
         * @return JointLimits structure, or default limits if joint not found
         */
        JointLimits getJointLimits(const std::string& joint_name) const;

        /**
         * @brief Check if limits are initialized for a joint
         * @param joint_name Name of the joint
         * @return True if limits are initialized for this joint
         */
        bool hasLimits(const std::string& joint_name) const;

        /**
         * @brief Get URDF motion type for a joint
         * @return UNKNOWN if joint is missing or type was not parsed
         */
        JointMotionType getJointMotionType(const std::string& joint_name) const;

        /**
         * @brief Check if a supported motion type has been parsed for a joint
         */
        bool hasJointMotionType(const std::string& joint_name) const;

        /**
         * @brief True only when the joint motion type is explicitly PRISMATIC
         */
        bool isPrismaticJoint(const std::string& joint_name) const;

        /**
         * @brief Apply joint limits to target positions (clamp values)
         * 
         * Clamps each position value to its joint limits if limits are initialized.
         * Joints without initialized limits are left unchanged.
         * 
         * @param joint_names List of joint names (must match target_positions size)
         * @param target_positions Target joint positions to clamp
         * @return Clamped positions
         */
        std::vector<double> applyLimits(const std::vector<std::string>& joint_names,
                                        const std::vector<double>& target_positions) const;

        /**
         * @brief Apply joint limits to target positions (clamp values)
         * 
         * Simplified version that assumes joint_names_ order matches target_positions.
         * Use this when joint order is already known and matches the internal order.
         * 
         * @param target_positions Target joint positions to clamp
         * @return Clamped positions
         */
        std::vector<double> applyLimits(const std::vector<double>& target_positions) const;

        /**
         * @brief Create a limit checker callback function
         * 
         * This creates a std::function that can be used as a joint limit checker
         * callback (e.g., for StateMoveJ::setJointLimitChecker).
         * 
         * The callback assumes the input vector matches the order of joint_names_
         * set during parsing or via setJointNames().
         * 
         * @return Callback function: std::vector<double>(const std::vector<double>&)
         */
        std::function<std::vector<double>(const std::vector<double>&)> createLimitChecker() const;

        /**
         * @brief Set joint names order
         * 
         * This sets the expected order of joints for applyLimits() and createLimitChecker().
         * Should be called after parsing or before using these methods.
         * 
         * @param joint_names Ordered list of joint names
         */
        void setJointNames(const std::vector<std::string>& joint_names);

        /**
         * @brief Get all joint names with initialized limits
         * @return Vector of joint names that have initialized limits
         */
        std::vector<std::string> getInitializedJointNames() const;

        /**
         * @brief Get number of joints with initialized limits
         * @return Count of joints with initialized limits
         */
        size_t getInitializedCount() const;

        /**
         * @brief Clear all limits
         */
        void clear();

        /**
         * @brief Check if manager has any initialized limits
         * @return True if at least one joint has initialized limits
         */
        bool hasAnyLimits() const;

    private:
        rclcpp::Logger logger_;
        std::unordered_map<std::string, JointLimits> joint_limits_;
        std::vector<std::string> joint_names_;  // Order of joints for applyLimits()
    };
} // namespace arms_controller_common
