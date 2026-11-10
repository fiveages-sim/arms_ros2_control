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
 ******************************************************************************/

#pragma once

#include <string>
#include <vector>

#include <rclcpp/parameter.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace ocs2::controller_common {

class HomeJointReferenceManager {
public:
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
     * @return true if either home-reference param was present in @p updates.
     */
    bool handleParameterUpdates(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                const std::vector<rclcpp::Parameter>& updates);

    [[nodiscard]] bool hasHome1() const;
    [[nodiscard]] const std::vector<double>& home1Joints() const;
    [[nodiscard]] const std::vector<double>& sessionJoints() const { return session_joints_; }
    [[nodiscard]] const std::string& sourceDescription() const { return source_; }

private:
    void resolve(const std::vector<double>& explicit_ref, int id,
                 const rclcpp::Logger& logger, bool log_result);

    std::vector<std::vector<double>> home_configs_;
    std::vector<double> session_joints_;
    std::string source_{"home_1"};
    static const std::vector<double> kEmpty;
};

}  // namespace ocs2::controller_common
