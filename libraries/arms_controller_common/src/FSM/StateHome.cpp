//
// Common StateHome Implementation
//
#include "arms_controller_common/FSM/StateHome.h"
#include "arms_controller_common/utils/SharedPublishers.h"
#include <cmath>
#include <sstream>
#include <limits>
#include <tuple>

namespace arms_controller_common
{
    void StateHome::publishCurrentTargetJoint(const std::vector<double>& target_positions)
    {
        if (!current_target_joint_publisher_)
        {
            return;
        }

        std_msgs::msg::Float64MultiArray msg;
        msg.data = target_positions;
        current_target_joint_publisher_->publish(msg);
    }

    StateHome::StateHome(CtrlInterfaces& ctrl_interfaces,
                         const std::shared_ptr<GravityCompensation>& gravity_compensation,
                         const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node)
        : FSMState(FSMStateName::HOME, "home", ctrl_interfaces),
          gravity_compensation_(gravity_compensation),
          node_(node),
          trajectory_manager_(node->get_logger())
    {
        for (const auto& entry : std::vector<std::pair<std::string, double>>{
                 {"home_max_velocity", 2.0}, {"home_max_acceleration", 4.0}, {"home_max_jerk", 20.0}})
            if (!node_->has_parameter(entry.first)) node_->declare_parameter(entry.first, entry.second);
        // Get switch_command_base from parameter server
        switch_command_base_ = node->get_parameter("switch_command_base").as_int();
        if (node_)
        {
            current_target_joint_publisher_ =
                arms_controller_common::utils::getOrCreateCurrentTargetJointPublisher(node_);
        }
    }

    void StateHome::updateJointLimitsFromURDF(
        const std::string& description, const std::vector<std::string>& joint_names)
    {
        std::lock_guard<std::mutex> lock(limits_mutex_);
        joint_names_ = joint_names;
        // Replace the snapshot so malformed/new descriptions cannot retain stale limits.
        joint_limits_ = std::make_unique<JointLimitsManager>(node_->get_logger());
        joint_limits_->parseFromURDF(description, joint_names_);
    }

    bool StateHome::validateTarget(const std::vector<double>& target) const
    {
        const auto count = ctrl_interfaces_.joint_position_command_interface_.size();
        if (target.empty() || target.size() != count)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "HOME target rejected: got %zu positions, expected %zu",
                         target.size(), count);
            return false;
        }
        std::lock_guard<std::mutex> lock(limits_mutex_);
        if (!joint_limits_ || joint_names_.size() != count)
        {
            RCLCPP_ERROR(node_->get_logger(), "HOME target rejected: joint limits are not ready");
            return false;
        }
        for (size_t i = 0; i < count; ++i)
        {
            const auto& name = joint_names_[i];
            const auto limits = joint_limits_->getJointLimits(name);
            if (!std::isfinite(target[i]))
            {
                RCLCPP_ERROR(node_->get_logger(), "HOME target rejected: %s is not finite", name.c_str());
                return false;
            }
            if (limits.motion_type == JointMotionType::CONTINUOUS)
                continue;
            if (!limits.initialized || !std::isfinite(limits.lower) ||
                !std::isfinite(limits.upper) || limits.lower > limits.upper)
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "HOME target rejected: %s has no valid position limits", name.c_str());
                return false;
            }
            if (target[i] < limits.lower || target[i] > limits.upper)
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "HOME target rejected: %s target %.9f outside [%.9f, %.9f]",
                             name.c_str(), target[i], limits.lower, limits.upper);
                return false;
            }
        }
        return true;
    }

    void StateHome::setHomePosition(const std::vector<double>& home_pos)
    {
        home_configs_.clear();
        home_configs_.push_back(home_pos);
        current_target_ = home_pos;
        current_config_index_ = 0;
        cycle_config_index_ = 0;
        has_multiple_configs_ = false;
    }

    void StateHome::setHomeConfigurations(const std::vector<std::vector<double>>& home_configs)
    {
        home_configs_ = home_configs;
        if (!home_configs_.empty())
        {
            current_target_ = home_configs_[0];
            current_config_index_ = 0;
            cycle_config_index_ = 0;
            has_multiple_configs_ = home_configs_.size() > 1;
        }
    }

    void StateHome::setRestPose(const std::vector<double>& rest_pos)
    {
        if (rest_pos.empty())
        {
            return;
        }

        if (home_configs_.size() == 1)
        {
            home_configs_.push_back(rest_pos);
            has_multiple_configs_ = true;
            RCLCPP_INFO(node_->get_logger(),
                        "Rest pose added as second configuration (index 1). Total configurations: %zu",
                        home_configs_.size());
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "Cannot set rest pose: expected exactly 1 home configuration, but found %zu",
                         home_configs_.size());
        }
    }

    void StateHome::enter()
    {
        // Always start with first configuration when entering the state
        if (!home_configs_.empty())
        {
            current_config_index_ = 0;
            cycle_config_index_ = 0;
            current_target_ = home_configs_[0];
        }

        trajectory_manager_.reset();
        clearPendingMotion();
        stop_to_zero_active_ = false;
        speed_stop_planner_.reset();
        if (!validateTarget(current_target_))
            return;

        // Get current joint positions as starting positions
        start_pos_.clear();
        for (auto i : ctrl_interfaces_.joint_position_state_interface_)
        {
            auto value = i.get().get_optional();
            start_pos_.push_back(value.value_or(0.0));
        }

        // Update parameters from node
        updateParam();

        // Initialize trajectory manager
        if (!start_pos_.empty() && !current_target_.empty() && start_pos_.size() == current_target_.size())
        {
            if (!trajectory_manager_.initSingleNode(
                start_pos_,
                current_target_,
                duration_,
                interpolation_type_,
                ctrl_interfaces_.frequency_,
                tanh_scale_, true, home_max_velocity_, home_max_acceleration_, home_max_jerk_))
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "Failed to initialize trajectory manager in StateHome::enter()");
            }
            else
            {
                publishCurrentTargetJoint(current_target_);
            }
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Cannot initialize trajectory manager: start_pos size (%zu) != target_pos size (%zu)",
                        start_pos_.size(), current_target_.size());
        }

        // Set kp and kd gains for force control if available
        if (ctrl_interfaces_.control_mode_ == ControlMode::MIX &&
            ctrl_interfaces_.default_gains_.size() >= 2)
        {
            double kp = ctrl_interfaces_.default_gains_[0];
            double kd = ctrl_interfaces_.default_gains_[1];

            for (auto& kp_interface : ctrl_interfaces_.joint_kp_command_interface_)
            {
                std::ignore = kp_interface.get().set_value(kp);
            }

            for (auto& kd_interface : ctrl_interfaces_.joint_kd_command_interface_)
            {
                std::ignore = kd_interface.get().set_value(kd);
            }
        }

        if (interpolation_type_ == InterpolationType::NONE)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "Direct home target selected: configuration %zu (no interpolation)",
                        current_config_index_);
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(),
                        "Starting interpolation to home configuration %zu over %.1f seconds (type=%s)",
                        current_config_index_, duration_, toString(interpolation_type_));
        }
    }

    void StateHome::run(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
    {

        const int32_t fsm_cmd = ctrl_interfaces_.fsm_command_;
        if (fsm_cmd == 2 || fsm_cmd == 3 || fsm_cmd == 4)
        {
            clearPendingMotion();
            stop_to_zero_active_ = false;
            speed_stop_planner_.reset();
            abortActiveMotionForStop();
        }

        int32_t current_command = ctrl_interfaces_.fsm_command_;
        bool command_changed = (current_command != last_command_);

        // Handle multi-configuration switching (command >= switch_command_base_)
        bool is_switch_command = current_command >= switch_command_base_;
        if (is_switch_command && command_changed && has_multiple_configs_)
        {
            if (current_command == switch_command_base_)
            {
                switchConfiguration();
            }
            else if (current_command >= switch_command_base_ + 1)
            {
                if (auto target_index = static_cast<size_t>(current_command - (switch_command_base_ + 1));
                    target_index < home_configs_.size())
                {
                    selectConfiguration(target_index);
                }
            }
        }

        last_command_ = current_command;

        if (stop_to_zero_active_)
        {
            runStopToZero(period);
            return;
        }

        // Get next trajectory point from unified manager
        double runtime_step = period.seconds();
        std::vector<double> next_positions = trajectory_manager_.getNextPoint(runtime_step);

        if (!next_positions.empty() &&
            next_positions.size() == ctrl_interfaces_.joint_position_command_interface_.size())
        {
            // Apply interpolated position to joints
            for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size() &&
                 i < next_positions.size(); ++i)
            {
                ctrl_interfaces_.setJointPositionCommand(i, next_positions[i]);
                if (trajectory_manager_.lastVelocities().size() == next_positions.size())
                    ctrl_interfaces_.setCommandDerivatives(i, trajectory_manager_.lastVelocities()[i],
                                                          trajectory_manager_.lastAccelerations()[i]);
            }
        }
        else if (next_positions.empty())
        {
            // If trajectory manager returns empty, maintain current position or use target
            static bool warned = false;
            if (!warned && !trajectory_manager_.isInitialized())
            {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *std::make_shared<rclcpp::Clock>(), 1000,
                                     "Trajectory manager not initialized, maintaining current position");
                warned = true;
            }
            // Hold last commanded position (avoid jumping to current_target_ mid-motion)
            for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size(); ++i)
            {
                if (i < ctrl_interfaces_.last_sent_joint_positions_.size())
                {
                    ctrl_interfaces_.setJointPositionCommand(
                        i, ctrl_interfaces_.last_sent_joint_positions_[i]);
                }
            }
        }
        else
        {
            static bool warned = false;
            if (!warned)
            {
                RCLCPP_WARN(node_->get_logger(),
                            "Trajectory manager returned positions with size mismatch: got %zu, expected %zu",
                            next_positions.size(), ctrl_interfaces_.joint_position_command_interface_.size());
                warned = true;
            }
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

    void StateHome::exit()
    {
        clearPendingMotion();
        stop_to_zero_active_ = false;
        speed_stop_planner_.reset();
        trajectory_manager_.reset();
    }

    bool StateHome::isMotionBusy() const
    {
        return trajectory_manager_.isInitialized() && !trajectory_manager_.isCompleted();
    }

    void StateHome::clearPendingMotion()
    {
        pending_motion_ = nullptr;
    }

    void StateHome::abortActiveMotionForStop()
    {
        trajectory_manager_.reset();
        const size_t num_joints = ctrl_interfaces_.joint_position_command_interface_.size();
        for (size_t i = 0; i < num_joints; ++i)
        {
            if (i < ctrl_interfaces_.last_sent_joint_positions_.size())
            {
                ctrl_interfaces_.setJointPositionCommand(
                    i, ctrl_interfaces_.last_sent_joint_positions_[i]);
            }
        }
    }

    bool StateHome::beginStopToZero()
    {
        const auto command = ctrl_interfaces_.getCommandMotionState();
        std::vector<double> lower, upper;
        std::lock_guard<std::mutex> limits_lock(limits_mutex_);
        const auto* limits_manager = joint_limits_.get();
        if (!limits_manager || joint_names_.size() != command.positions.size())
        {
            RCLCPP_ERROR(node_->get_logger(), "Stop rejected: joint limits not ready; retaining current motion");
            return false;
        }
        for (const auto& name : joint_names_)
        {
            const auto limits = limits_manager->getJointLimits(name);
            if (limits.motion_type == JointMotionType::CONTINUOUS)
            {
                lower.push_back(-std::numeric_limits<double>::infinity());
                upper.push_back(std::numeric_limits<double>::infinity());
            }
            else if (limits.initialized)
            {
                lower.push_back(limits.lower); upper.push_back(limits.upper);
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Stop rejected: missing limits for %s", name.c_str());
                return false;
            }
        }
        double vmax, amax, jmax;
        if (trajectory_manager_.isInitialized())
        {
            vmax = trajectory_manager_.stopMaxVelocity();
            amax = trajectory_manager_.stopMaxAcceleration();
            jmax = trajectory_manager_.stopMaxJerk();
        }
        else
        {
            std::tie(vmax, amax, jmax) = std::make_tuple(home_max_velocity_, home_max_acceleration_, home_max_jerk_);
        }
        const bool truncate = !trajectory_manager_.isInitialized() || !trajectory_manager_.isDoubles();
        if (!speed_stop_planner_.init(command.positions, command.velocities, command.accelerations,
                                      vmax, amax, jmax, lower, upper, truncate))
        {
            RCLCPP_ERROR(node_->get_logger(), "Stop rejected: %s; retaining current motion",
                         speed_stop_planner_.error().c_str());
            return false;
        }
        if (speed_stop_planner_.wasTruncated())
            RCLCPP_WARN(node_->get_logger(), "Synchronized stop will truncate all joints at the first position limit");
        stop_to_zero_active_ = true;
        RCLCPP_INFO(node_->get_logger(), "Synchronized joint stop started from command q/v/a (v=%.3f, a=%.3f, j=%.3f)",
                    vmax, amax, jmax);
        return true;
    }

    bool StateHome::requestMotionOrDefer(std::function<void()> apply_motion)
    {
        if (stop_to_zero_active_)
        {
            pending_motion_ = std::move(apply_motion);
            return true;
        }
        if (!isMotionBusy())
        {
            apply_motion();
            return true;
        }
        if (!beginStopToZero()) return false;
        // The stop is fully validated; only now discard the old trajectory.
        abortActiveMotionForStop();
        pending_motion_ = std::move(apply_motion);
        return true;
    }

    bool StateHome::runStopToZero(const rclcpp::Duration& period)
    {
        if (!stop_to_zero_active_)
        {
            return false;
        }

        std::vector<double> next_positions = speed_stop_planner_.run(period.seconds());

        if (!next_positions.empty() &&
            next_positions.size() == ctrl_interfaces_.joint_position_command_interface_.size())
        {
            for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size(); ++i)
            {
                ctrl_interfaces_.setJointPositionCommand(i, next_positions[i]);
                ctrl_interfaces_.setCommandDerivatives(i, speed_stop_planner_.velocities()[i],
                                                      speed_stop_planner_.accelerations()[i]);
            }
        }

        if (speed_stop_planner_.isMotionOver())
        {
            stop_to_zero_active_ = false;
            speed_stop_planner_.reset();
            RCLCPP_INFO(node_->get_logger(), "Stop-to-zero completed");

            if (pending_motion_)
            {
                auto apply = std::move(pending_motion_);
                pending_motion_ = nullptr;
                apply();
                RCLCPP_INFO(node_->get_logger(), "Pending home motion applied");
            }
        }

        return true;
    }

    void StateHome::updateParam()
    {
        home_max_velocity_ = node_->get_parameter("home_max_velocity").as_double();
        home_max_acceleration_ = node_->get_parameter("home_max_acceleration").as_double();
        home_max_jerk_ = node_->get_parameter("home_max_jerk").as_double();
        // Update duration from node parameter
        duration_ = node_->get_parameter("home_duration").as_double();

        // Update tanh_scale from node parameter
        double new_tanh_scale = node_->get_parameter("home_tanh_scale").as_double();
        if (new_tanh_scale > 0.0 && std::isfinite(new_tanh_scale))
        {
            tanh_scale_ = new_tanh_scale;
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Invalid home tanh scale %.3f from parameter, keeping current value %.3f",
                        new_tanh_scale, tanh_scale_);
        }

        // Update interpolation_type from node parameter
        std::string interpolation_type_str = node_->get_parameter("home_interpolation_type").as_string();
        interpolation_type_ = parseInterpolationType(interpolation_type_str, InterpolationType::LINEAR);

        // Future parameters can be added here
    }

    FSMStateName StateHome::checkChange()
    {
        switch (ctrl_interfaces_.fsm_command_)
        {
        case 2:
        case 3:
        case 4:
            return FSMStateName::HOLD;
        default:
            return FSMStateName::HOME;
        }
    }

    void StateHome::selectConfiguration(size_t config_index)
    {
        if (config_index >= home_configs_.size())
        {
            RCLCPP_ERROR(node_->get_logger(), "HOME target rejected: invalid configuration index %zu", config_index);
            return;
        }
        if (!validateTarget(home_configs_[config_index]))
            return;
        requestMotionOrDefer([this, config_index]() { selectConfigurationImpl(config_index); });
    }

    void StateHome::selectConfigurationImpl(size_t config_index)
    {
        if (config_index >= home_configs_.size())
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Invalid configuration index %zu (max: %zu)",
                        config_index, home_configs_.size() - 1);
            return;
        }

        if (!validateTarget(home_configs_[config_index]))
            return;
        current_config_index_ = config_index;
        cycle_config_index_ = config_index;
        current_target_ = home_configs_[config_index];
        startInterpolationImpl();
    }

    std::vector<double> StateHome::getConfiguration(size_t config_index) const
    {
        if (config_index >= home_configs_.size())
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Invalid configuration index %zu (max: %zu), returning empty vector",
                        config_index, home_configs_.size() - 1);
            return {};
        }

        return home_configs_[config_index];
    }

    void StateHome::switchConfiguration()
    {
        if (has_multiple_configs_)
        {
            // Consume this slot even if validation rejects it; the next request
            // must be able to reach later configurations without executing this one.
            cycle_config_index_ = (cycle_config_index_ + 1) % home_configs_.size();
            selectConfiguration(cycle_config_index_);
        }
    }

    void StateHome::startInterpolation()
    {
        if (validateTarget(current_target_))
            requestMotionOrDefer([this]() { startInterpolationImpl(); });
    }

    void StateHome::startInterpolationImpl()
    {
        if (!validateTarget(current_target_))
            return;
        start_pos_.clear();
        for (size_t i = 0; i < ctrl_interfaces_.joint_position_state_interface_.size(); ++i)
        {
            start_pos_.push_back(ctrl_interfaces_.last_sent_joint_positions_[i]);
        }

        // Update parameters from node
        updateParam();

        // Initialize trajectory manager with new target
        if (!start_pos_.empty() && !current_target_.empty() && start_pos_.size() == current_target_.size())
        {
            trajectory_manager_.initSingleNode(
                start_pos_,
                current_target_,
                duration_,
                interpolation_type_,
                ctrl_interfaces_.frequency_,
                tanh_scale_, true, home_max_velocity_, home_max_acceleration_, home_max_jerk_
            );
            publishCurrentTargetJoint(current_target_);

            if (interpolation_type_ == InterpolationType::NONE)
            {
                RCLCPP_INFO(node_->get_logger(),
                            "Direct home target selected: configuration %zu (no interpolation)",
                            current_config_index_);
            }
            else
            {
                RCLCPP_INFO(node_->get_logger(),
                            "Starting interpolation to configuration %zu over %.1f seconds (type=%s)",
                            current_config_index_, duration_, toString(interpolation_type_));
            }
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Cannot start interpolation: start_pos size (%zu) != target_pos size (%zu)",
                        start_pos_.size(), current_target_.size());
        }
    }
} // namespace arms_controller_common
