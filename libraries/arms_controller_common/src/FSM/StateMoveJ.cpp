//
// Common StateMoveJ Implementation
//
#include "arms_controller_common/FSM/StateMoveJ.h"
#include <cmath>
#include <algorithm>

namespace arms_controller_common
{
    StateMoveJ::StateMoveJ(CtrlInterfaces& ctrl_interfaces,
                           const rclcpp::Logger& logger,
                           double duration,
                           std::shared_ptr<GravityCompensation> gravity_compensation)
        : FSMState(FSMStateName::MOVEJ, "movej", ctrl_interfaces),
          logger_(logger),
          gravity_compensation_(gravity_compensation),
          duration_(duration),
          joint_limit_checker_(nullptr),
          trajectory_manager_(logger)
    {
    }

    void StateMoveJ::enter()
    {
        // Mark state as active
        {
            std::lock_guard lock(target_mutex_);
            state_active_ = true;
        }

        // Initialize joint names if not already done
        if (joint_names_.empty())
        {
            initializeJointNames();
        }

        // Get current joint positions as starting positions
        start_pos_.clear();
        for (auto i : ctrl_interfaces_.joint_position_state_interface_)
        {
            auto value = i.get().get_optional();
            start_pos_.push_back(value.value_or(0.0));
        }

        interpolation_active_ = false;

        // Reset prefix filtering
        use_prefix_filter_ = false;
        active_prefix_.clear();
        joint_mask_.clear();
        joint_mask_.resize(joint_names_.size(), true); // Default: control all joints

        // Check if we have a target position
        std::lock_guard lock(target_mutex_);
        if (has_target_ && target_pos_.size() == start_pos_.size())
        {
            // Initialize trajectory manager
            trajectory_manager_.initSingleNode(
                start_pos_,
                target_pos_,
                duration_,
                interpolation_type_,
                ctrl_interfaces_.frequency_,
                tanh_scale_
            );
            interpolation_active_ = true;
            RCLCPP_INFO(logger_,
                        "Starting interpolation to target position over %.1f seconds (type=%s)",
                        duration_,
                        toString(interpolation_type_));
        }
        else
        {
            RCLCPP_WARN(logger_,
                        "No target position set, waiting for target position...");
        }
    }

    void StateMoveJ::run(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        std::lock_guard lock(target_mutex_);

        // If no target or target size mismatch, maintain current position
        if (!has_target_ || target_pos_.size() != start_pos_.size())
        {
            // Maintain current position
            for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size() &&
                 i < start_pos_.size(); ++i)
            {
                std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(start_pos_[i]);
            }
            return;
        }

        // If interpolation just started, update start position to current position
        if (!interpolation_active_)
        {
            start_pos_.clear();
            for (auto i : ctrl_interfaces_.joint_position_state_interface_)
            {
                auto value = i.get().get_optional();
                start_pos_.push_back(value.value_or(0.0));
            }
            
            // Initialize trajectory manager
            trajectory_manager_.initSingleNode(
                start_pos_,
                target_pos_,
                duration_,
                interpolation_type_,
                ctrl_interfaces_.frequency_,
                tanh_scale_
            );
            
            interpolation_active_ = true;
            RCLCPP_INFO(logger_,
                        "Target position received, starting interpolation from current position");
        }

        // Get next trajectory point from unified manager
        std::vector<double> next_positions = trajectory_manager_.getNextPoint();
        
        if (!next_positions.empty() && 
            next_positions.size() == ctrl_interfaces_.joint_position_command_interface_.size())
        {
            // Apply interpolated position to joints (with prefix filtering if enabled)
            for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size() &&
                 i < next_positions.size(); ++i)
            {
                double position_to_set;
                
                // Check if joint filtering is enabled
                if (use_prefix_filter_ && i < joint_mask_.size())
                {
                    // Apply joint mask: control if true, hold if false
                    position_to_set = joint_mask_[i] ? next_positions[i] : start_pos_[i];
                }
                else
                {
                    // No filtering: use interpolated position
                    position_to_set = next_positions[i];
                }
                
                std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(position_to_set);
            }
        }
        else if (!next_positions.empty())
        {
            static bool warned = false;
            if (!warned)
            {
                RCLCPP_WARN(logger_,
                           "Trajectory manager returned positions with size mismatch");
                warned = true;
            }
        }

        // In force control mode, calculate static torques
        if (ctrl_interfaces_.control_mode_ == ControlMode::MIX && gravity_compensation_)
        {
            // Get interpolated joint positions
            std::vector<double> interpolated_positions;
            for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size(); ++i)
            {
                auto value = ctrl_interfaces_.joint_position_command_interface_[i].get().get_optional();
                interpolated_positions.push_back(value.value_or(0.0));
            }

            // Calculate static torques
            std::vector<double> static_torques =
                gravity_compensation_->calculateStaticTorques(interpolated_positions);

            // Set effort commands
            for (size_t i = 0; i < ctrl_interfaces_.joint_force_command_interface_.size() &&
                 i < static_torques.size(); ++i)
            {
                std::ignore = ctrl_interfaces_.joint_force_command_interface_[i].get().set_value(static_torques[i]);
            }
        }
    }

    void StateMoveJ::exit()
    {
        std::lock_guard lock(target_mutex_);
        // Mark state as inactive
        state_active_ = false;

        // Reset all state variables
        interpolation_active_ = false;
        has_target_ = false;
        target_pos_.clear();
        start_pos_.clear();

        // Reset trajectory manager
        trajectory_manager_.reset();

        // Reset prefix filtering
        use_prefix_filter_ = false;
        active_prefix_.clear();
        joint_mask_.clear();

        RCLCPP_DEBUG(logger_, "StateMoveJ exited, all state variables reset");
    }

    void StateMoveJ::setInterpolationType(const std::string& type)
    {
        interpolation_type_ = parseInterpolationType(type);
        RCLCPP_INFO(logger_, "Interpolation type set to '%s'", toString(interpolation_type_));
    }

    void StateMoveJ::setTanhScale(double scale)
    {
        if (scale <= 0.0 || !std::isfinite(scale))
        {
            RCLCPP_WARN(logger_, "Invalid movej tanh scale %.3f, keeping %.3f", scale, tanh_scale_);
            return;
        }
        tanh_scale_ = scale;
    }

    FSMStateName StateMoveJ::checkChange()
    {
        // Check FSM command to determine state transition
        // Default implementation: can transition to HOME or HOLD
        // Derived classes or controllers can override this behavior
        switch (ctrl_interfaces_.fsm_command_)
        {
        case 1:
            return FSMStateName::HOME;
        case 2:
            return FSMStateName::HOLD;
        default:
            return FSMStateName::MOVEJ;
        }
    }

    void StateMoveJ::setTargetPosition(const std::vector<double>& target_pos)
    {
        std::lock_guard lock(target_mutex_);

        // Check if state is active
        if (!state_active_)
        {
            RCLCPP_WARN(
                logger_, "Cannot set target position: StateMoveJ is not active. Please enter MOVEJ state first.");
            return;
        }

        // Disable prefix filtering when using the original method
        use_prefix_filter_ = false;
        active_prefix_.clear();
        joint_mask_.clear();
        if (!joint_names_.empty())
        {
            joint_mask_.resize(joint_names_.size(), true);
        }

        // Check if the new target is the same as the current target_pos_
        bool is_same_target = false;
        if (has_target_ && target_pos_.size() == target_pos.size())
        {
            is_same_target = true;
            for (size_t i = 0; i < target_pos.size(); ++i)
            {
                if (std::abs(target_pos_[i] - target_pos[i]) > TARGET_EPSILON)
                {
                    is_same_target = false;
                    break;
                }
            }
        }

        // If target is the same, skip re-interpolation
        if (is_same_target && interpolation_active_)
        {
            RCLCPP_DEBUG(logger_, "Received same target position, skipping re-interpolation");
            return;
        }

        // Apply joint limit checking if callback is set
        target_pos_ = applyJointLimits(target_pos, "target position");
        has_target_ = true;

        // Reset interpolation if we're already in the state
        if (interpolation_active_)
        {
            // Update start position to current position for new interpolation
            start_pos_.clear();
            for (auto i : ctrl_interfaces_.joint_position_state_interface_)
            {
                auto value = i.get().get_optional();
                start_pos_.push_back(value.value_or(0.0));
            }
            
            // Reinitialize trajectory manager with new target
            trajectory_manager_.initSingleNode(
                start_pos_,
                target_pos_,
                duration_,
                interpolation_type_,
                ctrl_interfaces_.frequency_,
                tanh_scale_
            );
            
            RCLCPP_INFO(logger_, "New target position received, restarting interpolation");
        }
    }

    void StateMoveJ::setTargetPosition(const std::string& prefix, const std::vector<double>& target_pos)
    {
        std::lock_guard lock(target_mutex_);

        // Check if state is active
        if (!state_active_)
        {
            RCLCPP_WARN(
                logger_,
                "Cannot set target position for prefix '%s': StateMoveJ is not active. Please enter MOVEJ state first.",
                prefix.c_str());
            return;
        }

        // Initialize joint names if not already done
        if (joint_names_.empty())
        {
            initializeJointNames();
        }

        // Update joint mask based on prefix
        updateJointMask(prefix);

        // Count how many joints match the prefix
        size_t matching_joints = 0;
        for (bool mask : joint_mask_)
        {
            if (mask) matching_joints++;
        }

        // Validate target position size
        if (target_pos.size() != matching_joints)
        {
            RCLCPP_WARN(logger_,
                        "Target position size (%zu) does not match number of joints with prefix '%s' (%zu). "
                        "Ignoring target position.",
                        target_pos.size(), prefix.c_str(), matching_joints);
            return;
        }

        // Create full target position vector (only set values for matching joints)
        if (target_pos_.size() != joint_names_.size())
        {
            target_pos_.resize(joint_names_.size());
            // Initialize with current positions for all joints
            for (size_t i = 0; i < ctrl_interfaces_.joint_position_state_interface_.size() &&
                 i < target_pos_.size(); ++i)
            {
                auto value = ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional();
                target_pos_[i] = value.value_or(0.0);
            }
        }

        // Set target positions only for joints matching the prefix
        size_t target_idx = 0;
        for (size_t i = 0; i < joint_mask_.size() && i < target_pos_.size(); ++i)
        {
            if (joint_mask_[i])
            {
                if (target_idx < target_pos.size())
                {
                    target_pos_[i] = target_pos[target_idx++];
                }
            }
        }

        // Apply joint limit checking if callback is set
        target_pos_ = applyJointLimits(target_pos_, "target position for prefix '" + prefix + "'");

        has_target_ = true;
        use_prefix_filter_ = true;
        active_prefix_ = prefix;

        // Reset interpolation if we're already in the state
        if (interpolation_active_)
        {
            // Update start position to current position for new interpolation
            start_pos_.clear();
            for (auto i : ctrl_interfaces_.joint_position_state_interface_)
            {
                auto value = i.get().get_optional();
                start_pos_.push_back(value.value_or(0.0));
            }
            
            // Reinitialize trajectory manager with new target
            trajectory_manager_.initSingleNode(
                start_pos_,
                target_pos_,
                duration_,
                interpolation_type_,
                ctrl_interfaces_.frequency_,
                tanh_scale_
            );
            
            RCLCPP_INFO(logger_,
                        "New target position received for joints with prefix '%s', restarting interpolation",
                        prefix.c_str());
        }
    }

    void StateMoveJ::setJointNames(const std::vector<std::string>& joint_names)
    {
        joint_names_ = joint_names;
        RCLCPP_INFO(logger_, "Set %zu joint names", joint_names_.size());
    }

    void StateMoveJ::initializeJointNames()
    {
        // Only initialize from interfaces if joint names haven't been set
        if (!joint_names_.empty())
        {
            return;
        }

        joint_names_.clear();

        // Extract joint names from position command interfaces as fallback
        for (const auto& interface : ctrl_interfaces_.joint_position_command_interface_)
        {
            std::string full_name = interface.get().get_prefix_name();
            joint_names_.push_back(full_name);
        }

        RCLCPP_INFO(logger_, "Initialized %zu joint names from interfaces", joint_names_.size());
    }

    void StateMoveJ::updateJointMask(const std::string& prefix)
    {
        joint_mask_.clear();
        joint_mask_.resize(joint_names_.size(), false);

        size_t matching_count = 0;
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            // Check if joint name starts with the prefix
            if (joint_names_[i].find(prefix) == 0)
            {
                joint_mask_[i] = true;
                matching_count++;
            }
            else
            {
                joint_mask_[i] = false;
            }
        }

        RCLCPP_INFO(logger_,
                    "Updated joint mask for prefix '%s': %zu joints will be controlled, %zu will be held",
                    prefix.c_str(), matching_count, joint_names_.size() - matching_count);
    }

    void StateMoveJ::setupSubscriptions(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
                                        const std::string& topic_base_name,
                                        bool enable_prefix_topics)
    {
        if (!node)
        {
            RCLCPP_WARN(logger_, "Cannot setup subscriptions: node is nullptr");
            return;
        }

        if (subscriptions_setup_)
        {
            RCLCPP_DEBUG(logger_, "Subscriptions already set up, skipping");
            return;
        }

        node_ = node;
        topic_base_name_ = topic_base_name;

        // Initialize joint names if not already done
        if (joint_names_.empty())
        {
            initializeJointNames();
        }

        // Use the same approach as basic_joint_controller: node_name + "/" + topic_base_name
        std::string base_topic = node->get_name() + std::string("/") + topic_base_name_;

        // Subscribe to default target position topic (always created)
        target_position_subscription_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
            base_topic, 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
            {
                std::vector<double> target_pos;
                for (const auto& val : msg->data)
                {
                    target_pos.push_back(val);
                }
                if (!target_pos.empty())
                {
                    setTargetPosition(target_pos);
                }
            });
        RCLCPP_INFO(logger_, "Subscribed to %s for all joints", base_topic.c_str());

        // Only create prefix-based topics if enabled
        if (!enable_prefix_topics)
        {
            subscriptions_setup_ = true;
            return;
        }

        // Check which prefixes exist in joint names
        bool has_left = false;
        bool has_right = false;
        bool has_body = false;

        for (const auto& joint_name : joint_names_)
        {
            // Check if joint name starts with prefix (case-sensitive)
            if (joint_name.find("left") == 0)
            {
                has_left = true;
            }
            if (joint_name.find("right") == 0)
            {
                has_right = true;
            }
            if (joint_name.find("body") == 0)
            {
                has_body = true;
            }
        }

        // Subscribe to left prefix topic if left joints exist
        if (has_left)
        {
            target_position_left_subscription_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
                base_topic + "/left", 10,
                [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
                {
                    std::vector<double> target_pos;
                    for (const auto& val : msg->data)
                    {
                        target_pos.push_back(val);
                    }
                    if (!target_pos.empty())
                    {
                        setTargetPosition("left", target_pos);
                    }
                });
            RCLCPP_INFO(logger_, "Subscribed to %s/left for left-prefixed joints", base_topic.c_str());
        }

        // Subscribe to right prefix topic if right joints exist
        if (has_right)
        {
            target_position_right_subscription_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
                base_topic + "/right", 10,
                [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
                {
                    std::vector<double> target_pos;
                    for (const auto& val : msg->data)
                    {
                        target_pos.push_back(val);
                    }
                    if (!target_pos.empty())
                    {
                        setTargetPosition("right", target_pos);
                    }
                });
            RCLCPP_INFO(logger_, "Subscribed to %s/right for right-prefixed joints", base_topic.c_str());
        }

        // Subscribe to body prefix topic if body joints exist
        if (has_body)
        {
            target_position_body_subscription_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
                base_topic + "/body", 10,
                [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
                {
                    std::vector<double> target_pos;
                    for (const auto& val : msg->data)
                    {
                        target_pos.push_back(val);
                    }
                    if (!target_pos.empty())
                    {
                        setTargetPosition("body", target_pos);
                    }
                });
            RCLCPP_INFO(logger_, "Subscribed to %s/body for body-prefixed joints", base_topic.c_str());
        }

        subscriptions_setup_ = true;

        // Log summary
        if (!has_left && !has_right && !has_body)
        {
            RCLCPP_DEBUG(logger_,
                         "No left/right/body prefixed joints found, only using default target_joint_position topic");
        }
    }

    void StateMoveJ::setJointLimitChecker(std::function<std::vector<double>(const std::vector<double>&)> limit_checker)
    {
        joint_limit_checker_ = limit_checker;
        if (limit_checker)
        {
            RCLCPP_INFO(logger_, "Joint limit checker enabled");
        }
        else
        {
            RCLCPP_DEBUG(logger_, "Joint limit checker disabled");
        }
    }


    std::vector<double> StateMoveJ::applyJointLimits(const std::vector<double>& target_pos, 
                                                      const std::string& log_message)
    {
        std::vector<double> clamped_target_pos = target_pos;
        
        if (joint_limit_checker_)
        {
            clamped_target_pos = joint_limit_checker_(target_pos);

            // Check if any values were clamped
            bool was_clamped = false;
            for (size_t i = 0; i < target_pos.size() && i < clamped_target_pos.size(); ++i)
            {
                if (std::abs(target_pos[i] - clamped_target_pos[i]) > TARGET_EPSILON)
                {
                    was_clamped = true;
                    break;
                }
            }

            // Log warning if limits were applied and log message is provided
            if (was_clamped && !log_message.empty())
            {
                std::string full_message = "Joint limits applied to " + log_message;
                RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 1000, "%s", full_message.c_str());
            }
        }

        return clamped_target_pos;
    }
} // namespace arms_controller_common
