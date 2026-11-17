/******************************************************************************
 * Resolves OCS2 HOME joint soft-reference from controller params + FSM homes.
 *
 * Priority on each refresh:
 *   1) home_joint_reference (double[]) when non-empty
 *   2) home_joint_reference_id (1-based → home_N) when >= 1
 *   3) home_1
 *
 * Exit OCS2 should push home1Joints() into the solver; enter should refresh +
 * push sessionJoints() so HOLD-time param changes take effect next session.
 *
 * Also owns URDF joint-limit snapshots used to gate enable of the soft
 * reference (same semantics as StateHome / JointLimitsManager::validatePositions).
 * Vectors whose size does not match the controller joint list are rejected.
 ******************************************************************************/

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <arms_controller_common/utils/JointLimitsManager.h>
#include <rclcpp/parameter.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace ocs2::controller_common {

struct HomeJointReferenceParamUpdate {
    bool touched{false};   // home_joint_reference(_id) present in the update batch
    bool rejected{false};  // invalid; caller should fail SetParametersResult (value not committed)
    std::string reason;
};

class HomeJointReferenceManager {
public:
    /** Expected joint order/count (same as controller `joints` param). */
    void setExpectedJointNames(std::vector<std::string> joint_names);

    /** Cache FSM home_1..home_N (index 0 = home_1). */
    void setHomeConfigurations(std::vector<std::vector<double>> home_configs);

    /**
     * Re-read home_joint_reference / home_joint_reference_id and resolve session.
     * Safe to call from HOLD or on every OCS2 enter.
     */
    void refreshFromParameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);

    /**
     * Handle on_set_parameters_callback updates for home_joint_reference(_id).
     * Uses values from @p updates (params are not committed yet during the callback).
     * On rejection, leave session unchanged so the node can fail the set and keep
     * the previous parameter value (empty → stays empty).
     */
    HomeJointReferenceParamUpdate handleParameterUpdates(
        const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
        const std::vector<rclcpp::Parameter>& updates);

    /** Replace joint-limit snapshot from robot_description (same joint order as controllers). */
    void updateJointLimitsFromURDF(const std::string& robot_description,
                                   const std::vector<std::string>& joint_names,
                                   const rclcpp::Logger& logger);

    /**
     * Require @p home_joints.size() == @p joint_names.size(), then validate URDF limits.
     * Logs ERROR on failure (no truncate/pad).
     */
    [[nodiscard]] bool validateAgainstJointLimits(
        const std::vector<double>& home_joints,
        const std::vector<std::string>& joint_names,
        const rclcpp::Logger& logger) const;

    [[nodiscard]] bool hasJointLimits() const;

    [[nodiscard]] bool hasHome1() const;
    [[nodiscard]] const std::vector<double>& home1Joints() const;
    [[nodiscard]] const std::vector<double>& sessionJoints() const { return session_joints_; }
    [[nodiscard]] const std::string& sourceDescription() const { return source_; }

private:
    /** @return false if candidate rejected; session unchanged. */
    bool resolve(const std::vector<double>& explicit_ref, int id,
                 const rclcpp::Logger& logger, bool log_result);
    [[nodiscard]] bool acceptJointVector(const std::vector<double>& joints,
                                         const char* label,
                                         const rclcpp::Logger& logger) const;
    [[nodiscard]] bool acceptLimitsIfReady(const std::vector<double>& joints,
                                           const rclcpp::Logger& logger) const;

    std::vector<std::string> expected_joint_names_;
    std::vector<std::vector<double>> home_configs_;
    std::vector<double> session_joints_;
    std::string source_{"home_1"};
    static const std::vector<double> kEmpty;

    mutable std::mutex limits_mutex_;
    std::unique_ptr<arms_controller_common::JointLimitsManager> joint_limits_;
};

}  // namespace ocs2::controller_common
