#include "ocs2_controller_common/reference/HomeJointReferenceManager.h"

#include <utility>

#include <rclcpp/logging.hpp>

namespace ocs2::controller_common {

const std::vector<double> HomeJointReferenceManager::kEmpty{};

void HomeJointReferenceManager::setHomeConfigurations(
    std::vector<std::vector<double>> home_configs) {
    home_configs_ = std::move(home_configs);
    if (hasHome1()) {
        session_joints_ = home_configs_[0];
        source_ = "home_1";
    } else {
        session_joints_.clear();
        source_ = "home_1";
    }
}

bool HomeJointReferenceManager::hasHome1() const {
    return !home_configs_.empty() && !home_configs_[0].empty();
}

const std::vector<double>& HomeJointReferenceManager::home1Joints() const {
    return hasHome1() ? home_configs_[0] : kEmpty;
}

void HomeJointReferenceManager::resolve(const std::vector<double>& explicit_ref, int id,
                                        const rclcpp::Logger& logger, bool log_result) {
    if (!hasHome1()) {
        session_joints_.clear();
        source_ = "home_1";
        if (log_result) {
            RCLCPP_WARN(logger, "HOME joint reference: no home_1 configured; session cleared");
        }
        return;
    }

    if (!explicit_ref.empty()) {
        session_joints_ = explicit_ref;
        source_ = "home_joint_reference";
    } else if (id >= 1) {
        const size_t index = static_cast<size_t>(id - 1);
        if (index < home_configs_.size() && !home_configs_[index].empty()) {
            session_joints_ = home_configs_[index];
            source_ = "home_" + std::to_string(id);
        } else {
            RCLCPP_WARN(logger,
                        "home_joint_reference_id=%d has no matching home_%d; falling back to home_1",
                        id, id);
            session_joints_ = home_configs_[0];
            source_ = "home_1";
        }
    } else {
        session_joints_ = home_configs_[0];
        source_ = "home_1";
    }

    if (log_result) {
        RCLCPP_INFO(logger,
                    "HOME joint reference preset updated: source=%s, joints=%zu, "
                    "home_joint_reference_id=%d, explicit_vector=%s "
                    "(live-applied if OCS2 is active; otherwise on next enter)",
                    source_.c_str(), session_joints_.size(), id,
                    explicit_ref.empty() ? "empty" : "set");
    }
}

void HomeJointReferenceManager::refreshFromParameters(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
    if (!node) {
        session_joints_.clear();
        source_ = "home_1";
        return;
    }

    std::vector<double> explicit_ref;
    int id = 0;
    if (node->has_parameter("home_joint_reference")) {
        explicit_ref = node->get_parameter("home_joint_reference").as_double_array();
    }
    if (node->has_parameter("home_joint_reference_id")) {
        id = static_cast<int>(node->get_parameter("home_joint_reference_id").as_int());
    }
    resolve(explicit_ref, id, node->get_logger(), /*log_result=*/false);
}

bool HomeJointReferenceManager::handleParameterUpdates(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::vector<rclcpp::Parameter>& updates) {
    if (!node) {
        return false;
    }

    bool touched = false;
    std::vector<double> explicit_ref;
    int id = 0;
    if (node->has_parameter("home_joint_reference")) {
        explicit_ref = node->get_parameter("home_joint_reference").as_double_array();
    }
    if (node->has_parameter("home_joint_reference_id")) {
        id = static_cast<int>(node->get_parameter("home_joint_reference_id").as_int());
    }

    for (const auto& param : updates) {
        if (param.get_name() == "home_joint_reference") {
            touched = true;
            explicit_ref = param.as_double_array();
            RCLCPP_INFO(node->get_logger(),
                        "Parameter home_joint_reference changed (%zu joints)",
                        explicit_ref.size());
        } else if (param.get_name() == "home_joint_reference_id") {
            touched = true;
            id = static_cast<int>(param.as_int());
            RCLCPP_INFO(node->get_logger(),
                        "Parameter home_joint_reference_id changed to %d", id);
        }
    }

    if (touched) {
        resolve(explicit_ref, id, node->get_logger(), /*log_result=*/true);
    }
    return touched;
}

}  // namespace ocs2::controller_common
