//
// Joint Limits Manager Implementation
//
#include "arms_controller_common/utils/JointLimitsManager.h"
#include <algorithm>
#include <cmath>

namespace arms_controller_common
{
    JointLimitsManager::JointLimitsManager(rclcpp::Logger logger)
        : logger_(logger)
    {
    }

    size_t JointLimitsManager::parseFromURDF(const std::string& robot_description, 
                                             const std::vector<std::string>& joint_names)
    {
        size_t initialized_count = 0;

        // If joint_names is provided, parse only those joints
        if (!joint_names.empty())
        {
            // Initialize joint limits map for all specified joints
            for (const auto& joint_name : joint_names)
            {
                if (joint_limits_.find(joint_name) == joint_limits_.end())
                {
                    joint_limits_[joint_name] = JointLimits();
                }
            }

            // Parse limits for each joint
            for (const auto& joint_name : joint_names)
            {
                if (parseJointLimitsFromURDF(robot_description, joint_name))
                {
                    initialized_count++;
                }
            }
        }
        else
        {
            // If no joint names provided, try to find all joints in URDF
            // This is a simplified approach - in practice, it's better to provide joint names
            RCLCPP_WARN(logger_, 
                       "parseFromURDF called without joint names. "
                       "Please provide joint names for accurate parsing.");
        }

        // Log summary
        if (!joint_names.empty())
        {
            RCLCPP_INFO(logger_, 
                       "Loaded joint limits for %zu/%zu joints from URDF", 
                       initialized_count, joint_names.size());
        }

        return initialized_count;
    }

    bool JointLimitsManager::parseJointLimitsFromURDF(const std::string& robot_description, 
                                                      const std::string& joint_name)
    {
        try
        {
            // Find the joint definition in URDF
            std::string joint_search = "<joint name=\"" + joint_name + "\"";
            size_t joint_pos = robot_description.find(joint_search);
            
            if (joint_pos == std::string::npos)
            {
                // Try to find in ros2_control section
                size_t search_start = 0;
                while (true)
                {
                    size_t ros2_control_start = robot_description.find("<ros2_control", search_start);
                    if (ros2_control_start == std::string::npos)
                    {
                        break;
                    }
                    
                    size_t ros2_control_end = robot_description.find("</ros2_control>", ros2_control_start);
                    if (ros2_control_end == std::string::npos)
                    {
                        ros2_control_end = robot_description.size();
                    }
                    
                    joint_pos = robot_description.find(joint_search, ros2_control_start);
                    if (joint_pos != std::string::npos && joint_pos < ros2_control_end)
                    {
                        break;
                    }
                    
                    search_start = ros2_control_end;
                }
            }

            if (joint_pos == std::string::npos)
            {
                RCLCPP_DEBUG(logger_, 
                            "Joint %s not found in robot description", joint_name.c_str());
                return false;
            }

            // Find limit tag within this joint
            size_t limit_pos = robot_description.find("<limit", joint_pos);
            if (limit_pos == std::string::npos)
            {
                // Try to find limit in command_interface
                size_t command_interface_pos = robot_description.find("<command_interface", joint_pos);
                if (command_interface_pos != std::string::npos)
                {
                    // Look for min/max attributes in command_interface
                    size_t min_pos = robot_description.find("min=\"", command_interface_pos);
                    size_t max_pos = robot_description.find("max=\"", command_interface_pos);
                    
                    if (min_pos != std::string::npos && min_pos < robot_description.find("</joint>", joint_pos))
                    {
                        min_pos += 5; // Skip "min=\""
                        size_t min_end = robot_description.find('"', min_pos);
                        if (min_end != std::string::npos)
                        {
                            std::string lower_str = robot_description.substr(min_pos, min_end - min_pos);
                            try
                            {
                                joint_limits_[joint_name].lower = std::stod(lower_str);
                            }
                            catch (...)
                            {
                                RCLCPP_WARN(logger_, 
                                            "Failed to parse lower limit for joint %s", joint_name.c_str());
                            }
                        }
                    }
                    
                    if (max_pos != std::string::npos && max_pos < robot_description.find("</joint>", joint_pos))
                    {
                        max_pos += 5; // Skip "max=\""
                        size_t max_end = robot_description.find('"', max_pos);
                        if (max_end != std::string::npos)
                        {
                            std::string upper_str = robot_description.substr(max_pos, max_end - max_pos);
                            try
                            {
                                joint_limits_[joint_name].upper = std::stod(upper_str);
                            }
                            catch (...)
                            {
                                RCLCPP_WARN(logger_, 
                                            "Failed to parse upper limit for joint %s", joint_name.c_str());
                            }
                        }
                    }
                    
                    if (joint_limits_[joint_name].lower != -std::numeric_limits<double>::max() &&
                        joint_limits_[joint_name].upper != std::numeric_limits<double>::max())
                    {
                        joint_limits_[joint_name].initialized = true;
                        RCLCPP_INFO(logger_,
                                    "Loaded joint limits for %s: lower=%.6f, upper=%.6f",
                                    joint_name.c_str(), joint_limits_[joint_name].lower, 
                                    joint_limits_[joint_name].upper);
                        return true;
                    }
                }
                return false;
            }

            // Parse upper and lower limits from <limit> tag
            size_t upper_pos = robot_description.find("upper=\"", limit_pos);
            size_t lower_pos = robot_description.find("lower=\"", limit_pos);

            if (upper_pos != std::string::npos && lower_pos != std::string::npos)
            {
                upper_pos += 7; // Skip "upper=\""
                lower_pos += 7; // Skip "lower=\""

                size_t upper_end = robot_description.find('"', upper_pos);
                size_t lower_end = robot_description.find('"', lower_pos);

                if (upper_end != std::string::npos && lower_end != std::string::npos)
                {
                    std::string upper_str = robot_description.substr(upper_pos, upper_end - upper_pos);
                    std::string lower_str = robot_description.substr(lower_pos, lower_end - lower_pos);

                    try
                    {
                        joint_limits_[joint_name].upper = std::stod(upper_str);
                        joint_limits_[joint_name].lower = std::stod(lower_str);
                        joint_limits_[joint_name].initialized = true;

                        RCLCPP_INFO(logger_,
                                    "Loaded joint limits for %s: lower=%.6f, upper=%.6f",
                                    joint_name.c_str(), joint_limits_[joint_name].lower, 
                                    joint_limits_[joint_name].upper);
                        return true;
                    }
                    catch (const std::exception& e)
                    {
                        RCLCPP_WARN(logger_, 
                                    "Failed to parse limits for joint %s: %s", 
                                    joint_name.c_str(), e.what());
                    }
                }
            }
            else
            {
                RCLCPP_DEBUG(logger_, 
                            "Missing upper or lower limit attributes for joint %s", joint_name.c_str());
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(logger_, "Error parsing joint limits for %s: %s", joint_name.c_str(), e.what());
        }

        return false;
    }

    void JointLimitsManager::setJointLimits(const std::string& joint_name, double lower, double upper)
    {
        joint_limits_[joint_name].lower = lower;
        joint_limits_[joint_name].upper = upper;
        joint_limits_[joint_name].initialized = true;
        
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

