//
// Common StateMoveJ Implementation
//
#include "arms_controller_common/FSM/StateMoveJ.h"
#include <cmath>

namespace arms_controller_common
{
    StateMoveJ::StateMoveJ(CtrlInterfaces& ctrl_interfaces,
                           const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                           const std::vector<std::string>& joint_names,
                           const std::shared_ptr<GravityCompensation>& gravity_compensation)
        : FSMState(FSMStateName::MOVEJ, "movej", ctrl_interfaces),
          node_(node),
          gravity_compensation_(gravity_compensation),
          joint_names_(joint_names),
          joint_limit_checker_(nullptr),
          trajectory_manager_(node->get_logger())
    {
        joint_limits_manager_ = std::make_shared<JointLimitsManager>(node_->get_logger());
        joint_limits_manager_->setJointNames(joint_names_);
    }

    void StateMoveJ::updateParam()
    {
        duration_ = node_->get_parameter("movej_duration").as_double();
        interpolation_type_ = parseInterpolationType(node_->get_parameter("movej_interpolation_type").as_string(),
                                                     InterpolationType::TANH);
        tanh_scale_ = node_->get_parameter("movej_tanh_scale").as_double();

        trajectory_manager_.setTrajectoryDuration(node_->get_parameter("movej_trajectory_duration").as_double());
        trajectory_manager_.setCommonJointBlendRatios(node_->get_parameter("movej_trajectory_blend_ratio").as_double());
    }

    void StateMoveJ::enter()
    {
        // Mark state as active
        {
            std::lock_guard lock(target_mutex_);
            state_active_ = true;
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
            RCLCPP_INFO(node_->get_logger(),
                        "Starting interpolation to target position over %.1f seconds (type=%s)",
                        duration_,
                        toString(interpolation_type_));
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                        "No target position set, waiting for target position...");
        }
    }

    void StateMoveJ::run(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        std::lock_guard lock(target_mutex_);

        // Check if multi-node trajectory is initialized (from setTrajectory)
        if (trajectory_manager_.isInitialized())
        {
            // Multi-node trajectory is active, directly execute it
            if (!interpolation_active_)
            {
                interpolation_active_ = true;
                RCLCPP_INFO(node_->get_logger(), "Multi-node trajectory execution started");
            }
        }
        // Check single-node trajectory conditions
        else if (!has_target_ || target_pos_.size() != start_pos_.size())
        {
            // No valid target, maintain current position
            for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size() &&
                 i < start_pos_.size(); ++i)
            {
                ctrl_interfaces_.setJointPositionCommand(i, start_pos_[i]);
            }
            return;
        }
        else if (!interpolation_active_)
        {
            start_pos_.clear();
            for (size_t i = 0; i < ctrl_interfaces_.joint_position_state_interface_.size(); ++i)
            {
                start_pos_.push_back(ctrl_interfaces_.last_sent_joint_positions_[i]);
            }
            trajectory_manager_.initSingleNode(
                start_pos_,
                target_pos_,
                duration_,
                interpolation_type_,
                ctrl_interfaces_.frequency_,
                tanh_scale_
            );

            interpolation_active_ = true;
            RCLCPP_INFO(node_->get_logger(),
                        "Target position received, starting interpolation from last sent position");
        }

        // Get next trajectory point from unified manager (works for both single and multi-node)
        std::vector<double> next_positions = trajectory_manager_.getNextPoint();

        if (next_positions.empty())
        {
            // Trajectory not initialized or error occurred
            if (trajectory_manager_.isInitialized())
            {
                // Trajectory is initialized but returned empty - this shouldn't happen normally
                static bool warned = false;
                if (!warned)
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "Trajectory manager returned empty positions (trajectory may be completed or error occurred)")
                    ;
                    warned = true;
                }
                // Maintain current position
                for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size() &&
                     i < start_pos_.size(); ++i)
                {
                    ctrl_interfaces_.setJointPositionCommand(i, start_pos_[i]);
                }
            }
            else
            {
                // Trajectory not initialized - maintain current position
                for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size() &&
                     i < start_pos_.size(); ++i)
                {
                    ctrl_interfaces_.setJointPositionCommand(i, start_pos_[i]);
                }
            }
            return;
        }

        if (next_positions.size() != ctrl_interfaces_.joint_position_command_interface_.size())
        {
            static bool warned = false;
            if (!warned)
            {
                RCLCPP_WARN(node_->get_logger(),
                            "Trajectory manager returned positions with size mismatch: expected %zu, got %zu",
                            ctrl_interfaces_.joint_position_command_interface_.size(),
                            next_positions.size());
                warned = true;
            }
            // Maintain current position on size mismatch
            for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size() &&
                 i < start_pos_.size(); ++i)
            {
                ctrl_interfaces_.setJointPositionCommand(i, start_pos_[i]);
            }
            return;
        }

        // Apply interpolated position to joints (with joint mask filtering for multi-node trajectory)
        // For joints not in the mask, use current real-time position to avoid jumping back
        std::vector<double> current_real_time_positions;
        if (use_prefix_filter_ && !joint_mask_.empty())
        {
            // Get current real-time positions for joints that are not controlled
            current_real_time_positions.reserve(joint_names_.size());
            for (auto i : ctrl_interfaces_.joint_position_state_interface_)
            {
                auto value = i.get().get_optional();
                current_real_time_positions.push_back(value.value_or(0.0));
            }
        }

        for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size() &&
             i < next_positions.size(); ++i)
        {
            double position_to_set;

            // Check if joint filtering is enabled (for both single-node prefix filter and multi-node trajectory)
            if (use_prefix_filter_ && i < joint_mask_.size())
            {
                // Apply joint mask: control if true, hold if false
                // For multi-node trajectory: only joints specified in trajectory message are controlled
                // For single-node trajectory: only joints matching prefix are controlled
                if (joint_mask_[i])
                {
                    // Controlled joint: use interpolated position
                    position_to_set = next_positions[i];
                }
                else
                {
                    if (i < current_real_time_positions.size())
                    {
                        position_to_set = current_real_time_positions[i];
                    }
                    else
                    {
                        // Fallback to start_pos_ if real-time position not available
                        position_to_set = (i < start_pos_.size()) ? start_pos_[i] : 0.0;
                    }
                }
            }
            else
            {
                // No filtering: use interpolated position for all joints
                position_to_set = next_positions[i];
            }

            ctrl_interfaces_.setJointPositionCommand(i, position_to_set);
        }

        // In force control mode, calculate static torques
        if (ctrl_interfaces_.control_mode_ == ControlMode::MIX && gravity_compensation_)
        {
            // Get interpolated joint positions
            std::vector<double> interpolated_positions;
            for (auto i : ctrl_interfaces_.joint_position_command_interface_)
            {
                auto value = i.get().get_optional();
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

        RCLCPP_DEBUG(node_->get_logger(), "StateMoveJ exited, all state variables reset");
    }


    FSMStateName StateMoveJ::checkChange()
    {
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
        updateParam();
        std::lock_guard lock(target_mutex_);

        // Check if state is active
        if (!state_active_)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "Cannot set target position: StateMoveJ is not active. Please enter MOVEJ state first.");
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
            RCLCPP_DEBUG(node_->get_logger(), "Received same target position, skipping re-interpolation");
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

            RCLCPP_INFO(node_->get_logger(), "New target position received, restarting interpolation");
        }
    }

    void StateMoveJ::setTargetPosition(const std::string& prefix, const std::vector<double>& target_pos)
    {
        updateParam();
        std::lock_guard lock(target_mutex_);

        // Check if state is active
        if (!state_active_)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "Cannot set target position for prefix '%s': StateMoveJ is not active. Please enter MOVEJ state first.",
                prefix.c_str());
            return;
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
            RCLCPP_WARN(node_->get_logger(),
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

            RCLCPP_INFO(node_->get_logger(),
                        "New target position received for joints with prefix '%s', restarting interpolation",
                        prefix.c_str());
        }
    }

    void StateMoveJ::setJointNames(const std::vector<std::string>& joint_names)
    {
        joint_names_ = joint_names;

        // Update joint limits manager if it exists
        if (joint_limits_manager_)
        {
            joint_limits_manager_->setJointNames(joint_names_);
        }

        RCLCPP_INFO(node_->get_logger(), "Set %zu joint names", joint_names_.size());
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

        RCLCPP_INFO(node_->get_logger(),
                    "Updated joint mask for prefix '%s': %zu joints will be controlled, %zu will be held",
                    prefix.c_str(), matching_count, joint_names_.size() - matching_count);
    }

    void StateMoveJ::setupSubscriptions(const std::string& topic_base_name,
                                        bool enable_prefix_topics)
    {
        topic_base_name_ = topic_base_name;

        // Use the same approach as basic_joint_controller: node_name + "/" + topic_base_name
        std::string base_topic = node_->get_name() + std::string("/") + topic_base_name_;

        // Subscribe to default target position topic (always created)
        target_position_subscription_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
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
        RCLCPP_INFO(node_->get_logger(), "Subscribed to %s for all joints", base_topic.c_str());

        // Only create prefix-based topics if enabled
        if (!enable_prefix_topics)
        {
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
            target_position_left_subscription_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
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
            RCLCPP_INFO(node_->get_logger(), "Subscribed to %s/left for left-prefixed joints", base_topic.c_str());
        }

        // Subscribe to right prefix topic if right joints exist
        if (has_right)
        {
            target_position_right_subscription_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
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
            RCLCPP_INFO(node_->get_logger(), "Subscribed to %s/right for right-prefixed joints", base_topic.c_str());
        }

        // Subscribe to body prefix topic if body joints exist
        if (has_body)
        {
            target_position_body_subscription_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
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
            RCLCPP_INFO(node_->get_logger(), "Subscribed to %s/body for body-prefixed joints", base_topic.c_str());
        }


        // Log summary
        if (!has_left && !has_right && !has_body)
        {
            RCLCPP_DEBUG(node_->get_logger(),
                         "No left/right/body prefixed joints found, only using default target_joint_position topic");
        }
    }

    void StateMoveJ::updateJointLimitsFromURDF(const std::string& robot_description)
    {
        if (robot_description.empty())
        {
            return;
        }

        size_t parsed_count = joint_limits_manager_->parseFromURDF(robot_description, joint_names_);
        if (parsed_count > 0)
        {
            joint_limit_checker_ = joint_limits_manager_->createLimitChecker();
            RCLCPP_INFO(node_->get_logger(),
                        "Joint limits updated from URDF: %zu joints with limits", parsed_count);
        }
    }

    void StateMoveJ::setJointLimitChecker(std::function<std::vector<double>(const std::vector<double>&)> limit_checker)
    {
        joint_limit_checker_ = limit_checker;
        if (limit_checker)
        {
            RCLCPP_INFO(node_->get_logger(), "Joint limit checker enabled (custom)");
        }
        else
        {
            RCLCPP_DEBUG(node_->get_logger(), "Joint limit checker disabled");
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
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "%s", full_message.c_str());
            }
        }

        return clamped_target_pos;
    }

    void StateMoveJ::setTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory)
    {
        updateParam();
        std::lock_guard lock(target_mutex_);

        // 1. Validate message
        if (!validateTrajectory(trajectory))
        {
            return;
        }

        // 2. Map joint names
        std::vector<size_t> joint_indices = mapJointNames(trajectory.joint_names);
        if (joint_indices.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "Cannot map trajectory joint names to controller joints");
            return;
        }

        // 2.5. Setup joint mask based on trajectory joint names (only control specified joints)
        // This allows controlling only left arm, right arm, or any subset of joints
        use_prefix_filter_ = true; // Enable joint filtering for multi-node trajectory
        joint_mask_.clear();
        joint_mask_.resize(joint_names_.size(), false); // Default: hold all joints

        size_t controlled_joints = 0;
        for (size_t idx : joint_indices)
        {
            if (idx < joint_mask_.size())
            {
                joint_mask_[idx] = true; // Mark joints in trajectory for control
                controlled_joints++;
            }
        }

        RCLCPP_INFO(node_->get_logger(),
                    "Multi-node trajectory joint mask set: %zu joints will be controlled, %zu will be held",
                    controlled_joints, joint_names_.size() - controlled_joints);

        // 3. Get current positions as starting point (first waypoint)
        std::vector<double> current_positions;
        current_positions.reserve(joint_names_.size());
        for (auto i : ctrl_interfaces_.joint_position_state_interface_)
        {
            auto value = i.get().get_optional();
            current_positions.push_back(value.value_or(0.0));
        }

        // Apply joint limits to current position
        current_positions = applyJointLimits(current_positions, "current position");

        // 4. Extract input trajectory positions (mapped to controller joint order)
        std::vector<std::vector<double>> waypoints;
        waypoints.reserve(trajectory.points.size() + 1); // +1 for current position

        // Add current position as first waypoint
        waypoints.push_back(current_positions);

        // Add input trajectory points
        for (const auto& point : trajectory.points)
        {
            if (point.positions.size() != trajectory.joint_names.size())
            {
                RCLCPP_ERROR(node_->get_logger(), "Position size mismatch in trajectory point");
                return;
            }

            std::vector<double> mapped_positions(joint_names_.size());
            // Initialize: use current positions (for unmatched joints)
            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                if (i < current_positions.size())
                {
                    mapped_positions[i] = current_positions[i];
                }
            }

            // Map trajectory positions
            for (size_t i = 0; i < joint_indices.size(); ++i)
            {
                size_t controller_idx = joint_indices[i];
                if (controller_idx < mapped_positions.size() && i < point.positions.size())
                {
                    mapped_positions[controller_idx] = point.positions[i];
                }
            }

            // Apply joint limits
            mapped_positions = applyJointLimits(mapped_positions, "trajectory waypoint");

            waypoints.push_back(mapped_positions);
        }

        // 5. Calculate segment durations (only for basic mode)
        // Note: For DOUBLES mode, lina planning will auto-calculate
        std::vector<double> durations;
        double trajectory_duration = trajectory_manager_.getTrajectoryDuration();

        if (interpolation_type_ != InterpolationType::DOUBLES ||
            !JointTrajectoryManager::isDoublesAvailable())
        {
            // Basic mode: manually calculate segment durations
            durations = calculateSegmentDurations(waypoints, trajectory_duration);
        }
        // DOUBLES mode: durations stays empty, lina planning will auto-calculate

        //temporarily using the common blend ratio
        std::vector<double> joint_blend_ratios;
        // 6. Initialize multi-node trajectory
        // waypoints now contains: current position + input trajectory points (at least 2) = at least 3 points
        // This satisfies lina planning's SmoothCurveOfMultiJointsUsingBlending requirement (>= 3 points)
        if (waypoints.size() >= 3)
        {
            size_t num_segments = waypoints.size() - 1;

            if (!trajectory_manager_.initMultiNode(
                waypoints,
                joint_blend_ratios,
                durations, // Basic mode: contains calculated segment durations; DOUBLES mode: empty, auto-calculated
                interpolation_type_,
                ctrl_interfaces_.frequency_,
                tanh_scale_))
            {
                RCLCPP_ERROR(node_->get_logger(), "Failed to initialize multi-node trajectory");
                return;
            }

            has_target_ = true;
            interpolation_active_ = false; // Will be activated in run()

            RCLCPP_INFO(node_->get_logger(),
                        "Trajectory loaded: %zu waypoints (1 current + %zu input), %zu segments. "
                        "Compatible with lina planning multi-node planner.",
                        waypoints.size(), trajectory.points.size(), num_segments);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "Invalid trajectory: need at least 2 input points (total >= 3 with current position), got %zu",
                         trajectory.points.size());
        }
    }

    bool StateMoveJ::validateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory)
    {
        // Check input trajectory point count (at least 2 points, with current position total >= 3 points)
        if (trajectory.points.size() < 2)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "Trajectory must have at least 2 input points (current position will be added as first point), got %zu",
                         trajectory.points.size());
            return false;
        }

        // Check joint names
        if (trajectory.joint_names.empty())
        {
            RCLCPP_ERROR(node_->get_logger(), "Trajectory joint_names is empty");
            return false;
        }

        // Check position count for each point
        for (size_t i = 0; i < trajectory.points.size(); ++i)
        {
            if (trajectory.points[i].positions.size() != trajectory.joint_names.size())
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "Point %zu: position size (%zu) != joint_names size (%zu)",
                             i, trajectory.points[i].positions.size(), trajectory.joint_names.size());
                return false;
            }
        }

        return true;
    }

    std::vector<size_t> StateMoveJ::mapJointNames(const std::vector<std::string>& trajectory_joint_names)
    {
        std::vector<size_t> indices;
        indices.reserve(trajectory_joint_names.size());

        for (const auto& traj_joint_name : trajectory_joint_names)
        {
            bool found = false;
            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                if (joint_names_[i] == traj_joint_name)
                {
                    indices.push_back(i);
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                RCLCPP_WARN(node_->get_logger(),
                            "Trajectory joint '%s' not found in controller joints",
                            traj_joint_name.c_str());
                // Return empty for strict matching
                return {};
            }
        }

        return indices;
    }

    std::vector<double> StateMoveJ::calculateSegmentDurations(
        const std::vector<std::vector<double>>& waypoints,
        double total_duration)
    {
        std::vector<double> durations;

        if (waypoints.size() < 2)
        {
            return durations; // Empty vector, will use default duration
        }

        size_t num_segments = waypoints.size() - 1;
        durations.reserve(num_segments);

        // Calculate path length (Euclidean distance) for each segment
        std::vector<double> segment_lengths;
        segment_lengths.reserve(num_segments);

        for (size_t i = 0; i < num_segments; ++i)
        {
            double length = 0.0;
            const auto& start = waypoints[i];
            const auto& end = waypoints[i + 1];

            if (start.size() != end.size())
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "Waypoint size mismatch: waypoint %zu has %zu joints, waypoint %zu has %zu joints",
                             i, start.size(), i + 1, end.size());
                durations.clear();
                return durations;
            }

            for (size_t j = 0; j < start.size(); ++j)
            {
                double diff = end[j] - start[j];
                length += diff * diff; // Euclidean distance squared
            }
            segment_lengths.push_back(std::sqrt(length));
        }

        // Calculate total length
        double total_length = 0.0;
        for (double len : segment_lengths)
        {
            total_length += len;
        }

        // Allocate time proportionally to path length
        if (total_length > 1e-6) // Avoid division by zero
        {
            for (double len : segment_lengths)
            {
                double segment_duration = total_duration * (len / total_length);
                durations.push_back(segment_duration);
            }
        }
        else
        {
            // If all segments have zero length (all points identical), evenly distribute
            double avg_duration = total_duration / num_segments;
            durations.assign(num_segments, avg_duration);
            RCLCPP_WARN(node_->get_logger(),
                        "All waypoints are identical, evenly distributing duration %.3f across %zu segments",
                        total_duration, num_segments);
        }

        return durations;
    }

    void StateMoveJ::setupTrajectorySubscription(const std::string& topic_name)
    {
        // Use the same approach as setupSubscriptions: node_name + "/" + topic_name
        std::string full_topic = node_->get_name() + std::string("/") + topic_name;

        // Subscribe to trajectory topic
        trajectory_subscription_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            full_topic, 10,
            [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
            {
                if (msg)
                {
                    setTrajectory(*msg);
                }
            });

        RCLCPP_INFO(node_->get_logger(), "Subscribed to trajectory topic: %s", full_topic.c_str());
    }
} // namespace arms_controller_common
