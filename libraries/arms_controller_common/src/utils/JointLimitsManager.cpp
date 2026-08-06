//
// Joint Limits Manager Implementation
//
#include "arms_controller_common/utils/JointLimitsManager.h"
#include <urdf/model.h>
#include <algorithm>
#include <cmath>

namespace arms_controller_common
{
    JointLimitsManager::JointLimitsManager(rclcpp::Logger logger)
        : logger_(logger)
    {
    }

    size_t JointLimitsManager::parseFromURDF(
        const std::string& robot_description,
        const std::vector<std::string>& joint_names,
        bool log_summary)
    {
        urdf::Model model;
        if (!model.initString(robot_description))
        {
            RCLCPP_ERROR(logger_, "Failed to parse robot description as URDF");
            return 0;
        }

        if (joint_names.empty())
        {
            RCLCPP_WARN(logger_,
                        "parseFromURDF called without joint names; no joints were parsed");
            return 0;
        }

        size_t limits_count = 0;
        size_t type_count = 0;

        for (const auto& joint_name : joint_names)
        {
            JointLimits& metadata = joint_limits_[joint_name];
            metadata = JointLimits{};

            const urdf::JointConstSharedPtr joint = model.getJoint(joint_name);
            if (!joint)
            {
                RCLCPP_ERROR(logger_, "Joint %s not found in robot description",
                             joint_name.c_str());
                continue;
            }

            switch (joint->type)
            {
            case urdf::Joint::REVOLUTE:
                metadata.motion_type = JointMotionType::REVOLUTE;
                break;
            case urdf::Joint::CONTINUOUS:
                metadata.motion_type = JointMotionType::CONTINUOUS;
                break;
            case urdf::Joint::PRISMATIC:
                metadata.motion_type = JointMotionType::PRISMATIC;
                break;
            default:
                RCLCPP_ERROR(logger_, "Joint %s has unsupported URDF type %d",
                             joint_name.c_str(), joint->type);
                continue;
            }
            ++type_count;

            if (joint->type != urdf::Joint::CONTINUOUS && joint->limits)
            {
                metadata.lower = joint->limits->lower;
                metadata.upper = joint->limits->upper;
                metadata.initialized = true;
                ++limits_count;
            }
        }

        if (log_summary)
        {
            RCLCPP_INFO(logger_,
                        "Loaded joint metadata for %zu/%zu joints; %zu have position limits",
                        type_count, joint_names.size(), limits_count);
        }
        return limits_count;
    }

    void JointLimitsManager::setJointLimits(const std::string& joint_name, double lower, double upper)
    {
        joint_limits_[joint_name].lower = lower;
        joint_limits_[joint_name].upper = upper;
        joint_limits_[joint_name].initialized = true;
        // Intentionally leave motion_type unchanged (UNKNOWN for new entries)

        RCLCPP_DEBUG(logger_,
                    "Set joint limits for %s: lower=%.6f, upper=%.6f",
                    joint_name.c_str(), lower, upper);
    }

    JointLimits JointLimitsManager::getJointLimits(const std::string& joint_name) const
    {
        auto it = joint_limits_.find(joint_name);
        if (it != joint_limits_.end())
        {
            return it->second;
        }
        return JointLimits();  // Return default limits
    }

    bool JointLimitsManager::hasLimits(const std::string& joint_name) const
    {
        auto it = joint_limits_.find(joint_name);
        return it != joint_limits_.end() && it->second.initialized;
    }

    JointMotionType JointLimitsManager::getJointMotionType(
        const std::string& joint_name) const
    {
        const auto it = joint_limits_.find(joint_name);
        return it == joint_limits_.end()
                   ? JointMotionType::UNKNOWN
                   : it->second.motion_type;
    }

    bool JointLimitsManager::hasJointMotionType(const std::string& joint_name) const
    {
        return getJointMotionType(joint_name) != JointMotionType::UNKNOWN;
    }

    bool JointLimitsManager::isPrismaticJoint(const std::string& joint_name) const
    {
        return getJointMotionType(joint_name) == JointMotionType::PRISMATIC;
    }

    std::vector<double> JointLimitsManager::applyLimits(
        const std::vector<std::string>& joint_names,
        const std::vector<double>& target_positions) const
    {
        std::vector<double> clamped_positions = target_positions;
        
        if (clamped_positions.size() != joint_names.size())
        {
            RCLCPP_WARN(logger_, 
                       "Target position size (%zu) does not match joint count (%zu), skipping limit check",
                       clamped_positions.size(), joint_names.size());
            return clamped_positions;
        }

        for (size_t i = 0; i < joint_names.size() && i < clamped_positions.size(); ++i)
        {
            const std::string& joint_name = joint_names[i];
            auto it = joint_limits_.find(joint_name);
            
            if (it != joint_limits_.end() && it->second.initialized)
            {
                double original = clamped_positions[i];
                clamped_positions[i] = std::clamp(original, it->second.lower, it->second.upper);
                
                if (std::abs(clamped_positions[i] - original) > 1e-6)
                {
                    RCLCPP_DEBUG(logger_,
                                "Joint %s target position %.6f clamped to [%.6f, %.6f] -> %.6f",
                                joint_name.c_str(), original, it->second.lower, 
                                it->second.upper, clamped_positions[i]);
                }
            }
        }

        return clamped_positions;
    }

    std::vector<double> JointLimitsManager::applyLimits(
        const std::vector<double>& target_positions) const
    {
        if (joint_names_.empty())
        {
            RCLCPP_WARN(logger_, 
                       "Joint names not set, cannot apply limits. Use setJointNames() first or use applyLimits(joint_names, positions).");
            return target_positions;
        }

        return applyLimits(joint_names_, target_positions);
    }

    std::function<std::vector<double>(const std::vector<double>&)> 
    JointLimitsManager::createLimitChecker() const
    {
        // Capture joint_names_ by value to ensure thread safety
        std::vector<std::string> joint_names = joint_names_;
        
        // Capture a pointer to this manager's limits map
        // Note: This assumes the manager's lifetime exceeds the callback's lifetime
        const std::unordered_map<std::string, JointLimits>* limits_map = &joint_limits_;
        rclcpp::Logger logger = logger_;
        
        return [joint_names, limits_map, logger](const std::vector<double>& target_pos) -> std::vector<double>
        {
            std::vector<double> clamped_positions = target_pos;
            
            if (clamped_positions.size() != joint_names.size())
            {
                RCLCPP_DEBUG(logger,
                           "Target position size (%zu) does not match joint count (%zu), skipping limit check",
                           clamped_positions.size(), joint_names.size());
                return clamped_positions;
            }

            for (size_t i = 0; i < joint_names.size() && i < clamped_positions.size(); ++i)
            {
                const std::string& joint_name = joint_names[i];
                auto it = limits_map->find(joint_name);
                
                if (it != limits_map->end() && it->second.initialized)
                {
                    double original = clamped_positions[i];
                    clamped_positions[i] = std::clamp(original, it->second.lower, it->second.upper);
                    
                if (std::abs(clamped_positions[i] - original) > 1e-6)
                {
                    // Use RCLCPP_WARN instead of RCLCPP_WARN_THROTTLE since we don't have clock access in lambda
                    RCLCPP_DEBUG(logger,
                                "Joint %s target position %.6f clamped to [%.6f, %.6f] -> %.6f",
                                joint_name.c_str(), original, it->second.lower, 
                                it->second.upper, clamped_positions[i]);
                }
                }
            }

            return clamped_positions;
        };
    }

    void JointLimitsManager::setJointNames(const std::vector<std::string>& joint_names)
    {
        joint_names_ = joint_names;
        RCLCPP_DEBUG(logger_, "Set %zu joint names", joint_names_.size());
    }

    std::vector<std::string> JointLimitsManager::getInitializedJointNames() const
    {
        std::vector<std::string> initialized_names;
        for (const auto& [name, limits] : joint_limits_)
        {
            if (limits.initialized)
            {
                initialized_names.push_back(name);
            }
        }
        return initialized_names;
    }

    size_t JointLimitsManager::getInitializedCount() const
    {
        size_t count = 0;
        for (const auto& [name, limits] : joint_limits_)
        {
            if (limits.initialized)
            {
                count++;
            }
        }
        return count;
    }

    void JointLimitsManager::clear()
    {
        joint_limits_.clear();
        joint_names_.clear();
        RCLCPP_DEBUG(logger_, "Cleared all joint limits");
    }

    bool JointLimitsManager::hasAnyLimits() const
    {
        return getInitializedCount() > 0;
    }
} // namespace arms_controller_common
