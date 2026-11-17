#include "ocs2_controller_common/reference/HomeJointReferenceManager.h"

#include <utility>

#include <rclcpp/logging.hpp>

namespace ocs2::controller_common {

const std::vector<double> HomeJointReferenceManager::kEmpty{};

void HomeJointReferenceManager::setExpectedJointNames(std::vector<std::string> joint_names) {
    expected_joint_names_ = std::move(joint_names);
}

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

void HomeJointReferenceManager::updateJointLimitsFromURDF(
    const std::string& robot_description,
    const std::vector<std::string>& joint_names,
    const rclcpp::Logger& logger) {
    expected_joint_names_ = joint_names;
    std::lock_guard<std::mutex> lock(limits_mutex_);
    joint_limits_ =
        std::make_unique<arms_controller_common::JointLimitsManager>(logger);
    joint_limits_->parseFromURDF(robot_description, joint_names);
}

bool HomeJointReferenceManager::hasJointLimits() const {
    std::lock_guard<std::mutex> lock(limits_mutex_);
    return joint_limits_ != nullptr;
}

bool HomeJointReferenceManager::acceptJointVector(const std::vector<double>& joints,
                                                  const char* label,
                                                  const rclcpp::Logger& logger) const {
    if (expected_joint_names_.empty()) {
        return true;  // joint list not known yet; validateAgainstJointLimits still gates apply
    }
    if (joints.size() != expected_joint_names_.size()) {
        RCLCPP_ERROR(logger,
                     "HOME joint reference rejected: %s has %zu joints, expected %zu "
                     "(same order/count as controller 'joints')",
                     label != nullptr ? label : "vector", joints.size(),
                     expected_joint_names_.size());
        return false;
    }
    return true;
}

bool HomeJointReferenceManager::acceptLimitsIfReady(
    const std::vector<double>& joints, const rclcpp::Logger& /*logger*/) const {
    std::lock_guard<std::mutex> lock(limits_mutex_);
    if (!joint_limits_ || expected_joint_names_.empty()) {
        return true;
    }
    return joint_limits_->validatePositions(
        expected_joint_names_, joints, "HOME joint reference");
}

bool HomeJointReferenceManager::validateAgainstJointLimits(
    const std::vector<double>& home_joints,
    const std::vector<std::string>& joint_names,
    const rclcpp::Logger& logger) const {
    if (home_joints.size() != joint_names.size()) {
        RCLCPP_ERROR(logger,
                     "HOME joint reference rejected: got %zu joints, expected %zu "
                     "(same order/count as controller 'joints')",
                     home_joints.size(), joint_names.size());
        return false;
    }

    std::lock_guard<std::mutex> lock(limits_mutex_);
    if (!joint_limits_) {
        RCLCPP_ERROR(logger, "HOME joint reference rejected: joint limits are not ready");
        return false;
    }
    return joint_limits_->validatePositions(joint_names, home_joints, "HOME joint reference");
}

bool HomeJointReferenceManager::resolve(const std::vector<double>& explicit_ref, int id,
                                        const rclcpp::Logger& logger, bool log_result) {
    if (!hasHome1()) {
        session_joints_.clear();
        source_ = "home_1";
        if (log_result) {
            RCLCPP_WARN(logger, "HOME joint reference: no home_1 configured; session cleared");
        }
        return true;
    }

    std::vector<double> candidate;
    std::string candidate_source;

    if (!explicit_ref.empty()) {
        candidate = explicit_ref;
        candidate_source = "home_joint_reference";
    } else if (id >= 1) {
        const size_t index = static_cast<size_t>(id - 1);
        if (index >= home_configs_.size() || home_configs_[index].empty()) {
            RCLCPP_ERROR(logger,
                         "HOME joint reference rejected: home_joint_reference_id=%d has no "
                         "matching home_%d",
                         id, id);
            return false;
        }
        candidate = home_configs_[index];
        candidate_source = "home_" + std::to_string(id);
    } else if (id < 0) {
        RCLCPP_ERROR(logger,
                     "HOME joint reference rejected: home_joint_reference_id=%d is invalid "
                     "(use 0 for home_1, or 1..N for home_N)",
                     id);
        return false;
    } else {
        candidate = home_configs_[0];
        candidate_source = "home_1";
    }

    if (!acceptJointVector(candidate, candidate_source.c_str(), logger) ||
        !acceptLimitsIfReady(candidate, logger)) {
        return false;
    }

    session_joints_ = std::move(candidate);
    source_ = std::move(candidate_source);

    if (log_result) {
        RCLCPP_INFO(logger,
                    "HOME joint reference preset updated: source=%s, joints=%zu, "
                    "home_joint_reference_id=%d, explicit_vector=%s "
                    "(live-applied if OCS2 is active; otherwise on next enter)",
                    source_.c_str(), session_joints_.size(), id,
                    explicit_ref.empty() ? "empty" : "set");
    }
    return true;
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
    if (resolve(explicit_ref, id, node->get_logger(), /*log_result=*/false)) {
        return;
    }
    // Stale invalid preset: session → home_1, then clear the offending param(s).
    resolve(/*explicit_ref=*/{}, /*id=*/0, node->get_logger(), /*log_result=*/false);
    if (!explicit_ref.empty() && node->has_parameter("home_joint_reference")) {
        try {
            node->set_parameter(rclcpp::Parameter("home_joint_reference", std::vector<double>{}));
            RCLCPP_WARN(node->get_logger(),
                        "Cleared invalid home_joint_reference (was %zu joints); fall back to home_1",
                        explicit_ref.size());
        } catch (const std::exception& e) {
            RCLCPP_WARN(node->get_logger(),
                        "Failed to clear invalid home_joint_reference: %s", e.what());
        }
    } else if (id != 0 && node->has_parameter("home_joint_reference_id")) {
        try {
            node->set_parameter(rclcpp::Parameter("home_joint_reference_id", 0));
            RCLCPP_WARN(node->get_logger(),
                        "Reset invalid home_joint_reference_id=%d → 0 (home_1)", id);
        } catch (const std::exception& e) {
            RCLCPP_WARN(node->get_logger(),
                        "Failed to reset invalid home_joint_reference_id: %s", e.what());
        }
    }
}

HomeJointReferenceParamUpdate HomeJointReferenceManager::handleParameterUpdates(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
    const std::vector<rclcpp::Parameter>& updates) {
    HomeJointReferenceParamUpdate out;
    if (!node) {
        return out;
    }

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
            out.touched = true;
            explicit_ref = param.as_double_array();
            RCLCPP_INFO(node->get_logger(),
                        "Parameter home_joint_reference changed (%zu joints)",
                        explicit_ref.size());
        } else if (param.get_name() == "home_joint_reference_id") {
            out.touched = true;
            id = static_cast<int>(param.as_int());
            RCLCPP_INFO(node->get_logger(),
                        "Parameter home_joint_reference_id changed to %d", id);
        }
    }

    if (!out.touched) {
        return out;
    }

    if (!resolve(explicit_ref, id, node->get_logger(), /*log_result=*/true)) {
        out.rejected = true;
        out.reason =
            "invalid home_joint_reference / home_joint_reference_id "
            "(missing home_N, joint count, or limits); "
            "parameter change rejected, previous value kept";
        RCLCPP_ERROR(node->get_logger(), "%s", out.reason.c_str());
    }
    return out;
}

}  // namespace ocs2::controller_common
