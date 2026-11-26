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
    void setExpectedJointNames(std::vector<std::string> joint_names);

    void setHomeConfigurations(std::vector<std::vector<double>> home_configs);

    void refreshFromParameters(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);

    HomeJointReferenceParamUpdate handleParameterUpdates(
        const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
        const std::vector<rclcpp::Parameter>& updates);

    void updateJointLimitsFromURDF(const std::string& robot_description,
                                   const std::vector<std::string>& joint_names,
                                   const rclcpp::Logger& logger);

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
