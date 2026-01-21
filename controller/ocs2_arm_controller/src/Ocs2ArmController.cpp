//
// Created for OCS2 Arm Controller
//

#include "ocs2_arm_controller/Ocs2ArmController.h"
#include <algorithm>
#include "ocs2_arm_controller/FSM/StateHome.h"
#include "ocs2_arm_controller/FSM/StateOCS2.h"
#include "ocs2_arm_controller/FSM/StateHold.h"
#include "ocs2_arm_controller/FSM/StateMoveJ.h"
#include <arms_controller_common/utils/GravityCompensation.h>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <rclcpp/rclcpp.hpp>

namespace ocs2::mobile_manipulator
{
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::InterfaceConfiguration Ocs2ArmController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : command_interface_types_)
            {
                if (!command_prefix_.empty())
                {
                    conf.names.push_back(command_prefix_ + "/" + joint_name + "/" + interface_type);
                }
                else
                {
                    conf.names.push_back(joint_name + "/" + interface_type);
                }
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration Ocs2ArmController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : state_interface_types_)
            {
                conf.names.push_back(joint_name + "/" + interface_type);
            }
        }

        return conf;
    }

    controller_interface::return_type Ocs2ArmController::update(const rclcpp::Time& time,
                                                                const rclcpp::Duration& period)
    {
        // Publish end effector pose (regardless of current state)
        ctrl_comp_->updateObservation(time);

        if (mode_ == FSMMode::NORMAL)
        {
            current_state_->run(time, period);
            next_state_name_ = current_state_->checkChange();
            if (next_state_name_ != current_state_->state_name)
            {
                mode_ = FSMMode::CHANGE;
                next_state_ = getNextState(next_state_name_);
                RCLCPP_INFO(get_node()->get_logger(), "Switched from %s to %s",
                            current_state_->state_name_string.c_str(), next_state_->state_name_string.c_str());
            }
        }
        else if (mode_ == FSMMode::CHANGE)
        {
            current_state_->exit();
            current_state_ = next_state_;
            current_state_->enter();
            mode_ = FSMMode::NORMAL;
            
            // Reset FSM command after state transition is complete
            // This prevents the command from triggering another transition
            ctrl_interfaces_.fsm_command_ = 0;
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn Ocs2ArmController::on_init()
    {
        try
        {
            // Get update frequency
            get_node()->get_parameter("update_rate", ctrl_interfaces_.frequency_);
            RCLCPP_INFO(get_node()->get_logger(), "Controller Manager Update Rate: %d Hz", ctrl_interfaces_.frequency_);

            // Hardware parameters
            command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);
            joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
            command_interface_types_ =
                auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
            state_interface_types_ =
                auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

            // Control input parameters
            control_input_name_ = auto_declare<std::string>("control_input_name", control_input_name_);
            control_input_interface_types_ =
                auto_declare<std::vector<std::string>>("control_input_interfaces", control_input_interface_types_);

            // State machine parameters
            home_pos_ = auto_declare<std::vector<double>>("home_pos", home_pos_);
            rest_pos_ = auto_declare<std::vector<double>>("rest_pos", rest_pos_);

            // Force control parameters
            ctrl_interfaces_.default_gains_ = auto_declare<std::vector<double>>(
                "default_gains", ctrl_interfaces_.default_gains_);

            // PD control parameters (for OCS2 state)
            ctrl_interfaces_.pd_gains_ = auto_declare<std::vector<double>>("pd_gains", ctrl_interfaces_.pd_gains_);

            // MPC frequency parameter - default value 0 if not set
            auto_declare<int>("mpc_frequency", 0);

            // Hold state parameters
            auto_declare<double>("hold_position_threshold", 0.1); // Default: 0.1 rad (~5.7 degrees)

            // Home state parameters
            double home_duration = auto_declare<double>("home_duration", 3.0); // Default: 3.0 seconds
            int switch_command_base = auto_declare<int>("switch_command_base", 100);
            // Default: 100 for multi-home switching

            // Declare trajectory_duration parameter (used by both CtrlComponent and StateMoveJ)
            // This must be declared before CtrlComponent is created
            double trajectory_duration = auto_declare<double>("trajectory_duration", 2.0);

            // Create CtrlComponent (auto-initialize interface)
            // Pass auto_declare function to CtrlComponent so it can declare its own parameters
            auto auto_declare_func = [this](const std::string& name, const auto& default_value) {
                using T = std::decay_t<decltype(default_value)>;
                return this->auto_declare<T>(name, default_value);
            };
            ctrl_comp_ = std::make_shared<CtrlComponent>(get_node(), ctrl_interfaces_, auto_declare_func);

            // Create GravityCompensation from CtrlComponent's Pinocchio model
            std::shared_ptr<arms_controller_common::GravityCompensation> gravity_compensation = nullptr;
            if (ctrl_comp_->interface_)
            {
                const auto& pinocchio_model = ctrl_comp_->interface_->getPinocchioInterface().getModel();
                gravity_compensation = std::make_shared<arms_controller_common::GravityCompensation>(pinocchio_model);
                RCLCPP_INFO(get_node()->get_logger(),
                            "Gravity compensation initialized from OCS2 Pinocchio model");
            }

            // Get logger for FSM states
            rclcpp::Logger logger = get_node()->get_logger();

            // Create StateHome using common implementation
            state_list_.home = std::make_shared<StateHome>(
                ctrl_interfaces_, logger, home_duration, gravity_compensation);

            // Home interpolation parameters
            std::string home_interpolation_type = auto_declare<std::string>("home_interpolation_type", "linear");
            double home_tanh_scale = auto_declare<double>("home_tanh_scale", 3.0);
            state_list_.home->setInterpolationType(home_interpolation_type);
            state_list_.home->setTanhScale(home_tanh_scale);

            // Try to load multiple home configurations (home_1, home_2, etc.)
            // This supports the new multi-home configuration mechanism
            // init() returns true if at least one configuration (home_1) was loaded
            bool configs_loaded = state_list_.home->init(
                [this](const std::string& name, const std::vector<double>& default_value)
                {
                    return this->auto_declare<std::vector<double>>(name, default_value);
                });

            state_list_.home->setSwitchCommandBase(switch_command_base);

            if (configs_loaded)
            {
                // Multi-home configuration was loaded successfully
                RCLCPP_INFO(get_node()->get_logger(),
                            "Using multi-home configuration (home_1, home_2, etc.) with switch_command_base=%d",
                            switch_command_base);
            }
            else if (!home_pos_.empty())
            {
                // Fallback to legacy single home_pos configuration (backward compatibility)
                state_list_.home->setHomePosition(home_pos_);
                RCLCPP_INFO(get_node()->get_logger(),
                            "Using legacy home_pos configuration with %zu joints (switch_command_base=%d)",
                            home_pos_.size(), switch_command_base);
            }
            else
            {
                RCLCPP_WARN(get_node()->get_logger(),
                            "No home configuration found (neither home_1 nor home_pos)");
            }

            // Configure rest pose if available
            if (!rest_pos_.empty())
            {
                state_list_.home->setRestPose(rest_pos_);
                RCLCPP_INFO(get_node()->get_logger(),
                            "Rest pose configured with %zu joints", rest_pos_.size());
            }
            else
            {
                RCLCPP_INFO(get_node()->get_logger(), "No rest pose configured, using home pose only");
            }

            state_list_.ocs2 = std::make_shared<StateOCS2>(ctrl_interfaces_, get_node(), ctrl_comp_);

            // Hold state parameters
            double hold_position_threshold = auto_declare<double>("hold_position_threshold", 0.1);

            // Create StateHold using common implementation
            state_list_.hold = std::make_shared<StateHold>(
                ctrl_interfaces_, logger, hold_position_threshold, gravity_compensation);

            // MoveJ state parameters
            double move_duration = auto_declare<double>("move_duration", 3.0);
            std::string movej_interpolation_type = auto_declare<std::string>("movej_interpolation_type", "linear");
            double movej_tanh_scale = auto_declare<double>("movej_tanh_scale", 3.0);

            // Create StateMoveJ using common implementation
            state_list_.movej = std::make_shared<StateMoveJ>(
                ctrl_interfaces_, logger, move_duration, gravity_compensation);

            // Configure interpolation type/shape
            state_list_.movej->setInterpolationType(movej_interpolation_type);
            state_list_.movej->setTanhScale(movej_tanh_scale);

            // Set joint names from controller parameters
            state_list_.movej->setJointNames(joint_names_);
            state_list_.movej->setTrajectoryDuration(trajectory_duration);

            // Set trajectory common joint blend ratio for multi-node trajectory planning
            double joint_blend_ratio=auto_declare<double>("joint_trajectory_common_blend_ratio",0.0);
            state_list_.movej->setCommonJointBlendRatios(joint_blend_ratio);
            RCLCPP_INFO(get_node()->get_logger(),"Trajectory blend ratio set to %.2f ",joint_blend_ratio);

            // Set joint limit checker from Pinocchio model
            if (ctrl_comp_->interface_)
            {
                const auto& pinocchio_model = ctrl_comp_->interface_->getPinocchioInterface().getModel();

                // Get joint limits from Pinocchio model
                // Note: Pinocchio stores limits for all DOFs, we need to extract only the arm joints
                // The arm joints typically start after base DOFs
                int arm_state_dim = ctrl_comp_->interface_->getManipulatorModelInfo().armDim;

                // Extract arm joint limits (tail of the position limits vector)
                Eigen::VectorXd lower_limits = pinocchio_model.lowerPositionLimit.tail(arm_state_dim);
                Eigen::VectorXd upper_limits = pinocchio_model.upperPositionLimit.tail(arm_state_dim);

                // Create limit checker callback
                state_list_.movej->setJointLimitChecker(
                    [lower_limits, upper_limits, arm_state_dim](
                    const std::vector<double>& target_pos) -> std::vector<double>
                    {
                        std::vector<double> clamped_pos = target_pos;

                        // Clamp each joint position to its limits
                        for (size_t i = 0; i < clamped_pos.size() && i < static_cast<size_t>(arm_state_dim); ++i)
                        {
                            clamped_pos[i] = std::clamp(clamped_pos[i],
                                                        static_cast<double>(lower_limits(i)),
                                                        static_cast<double>(upper_limits(i)));
                        }

                        return clamped_pos;
                    });

                RCLCPP_INFO(get_node()->get_logger(),
                            "Joint limit checker initialized from Pinocchio model (arm joints: %d)",
                            arm_state_dim);
            }

            return CallbackReturn::SUCCESS;
        }
        catch (const std::exception& e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    controller_interface::CallbackReturn Ocs2ArmController::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // Subscribe to FSM command (dedicated topic for state transitions)
        fsm_command_subscription_ = get_node()->create_subscription<std_msgs::msg::Int32>(
            "/fsm_command", 10, [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                ctrl_interfaces_.fsm_command_ = msg->data;
            });

        // Setup subscriptions for target positions in StateMoveJ
        // Enable prefix topics (left/right/body) for ocs2_arm_controller
        if (state_list_.movej)
        {
            state_list_.movej->setupSubscriptions(get_node(), "target_joint_position", true);
            // Setup trajectory subscription for multi-node trajectory planning (uses default topic name)
            state_list_.movej->setupTrajectorySubscription(get_node());
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2ArmController::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // clear out vectors in case of restart
        ctrl_interfaces_.clear();

        // assign command interfaces
        for (auto& interface : command_interfaces_)
        {
            std::string interface_name = interface.get_interface_name();
            if (const size_t pos = interface_name.find('/'); pos != std::string::npos)
            {
                command_interface_map_[interface_name.substr(pos + 1)]->push_back(interface);
            }
            else
            {
                command_interface_map_[interface_name]->push_back(interface);
            }
        }

        // assign state interfaces
        for (auto& interface : state_interfaces_)
        {
            state_interface_map_[interface.get_interface_name()]->push_back(interface);
        }

        // Auto-detect control mode based on available interfaces
        ctrl_interfaces_.detectAndSetControlMode();

        // Log detected control mode
        if (ctrl_interfaces_.control_mode_ == ControlMode::MIX)
        {
            RCLCPP_INFO(get_node()->get_logger(),
                        "Mixed control mode enabled - detected kp, kd, velocity, effort, position interfaces");
        }
        else
        {
            RCLCPP_INFO(get_node()->get_logger(),
                        "Position control mode enabled - standard position control interfaces detected");
        }

        ctrl_interfaces_.initializeLastSentPositions();
        // Initialize FSM
        current_state_ = state_list_.hold;
        current_state_->enter();
        next_state_ = current_state_;
        next_state_name_ = current_state_->state_name;
        mode_ = FSMMode::NORMAL;

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2ArmController::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2ArmController::on_cleanup(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2ArmController::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn Ocs2ArmController::on_shutdown(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    std::shared_ptr<FSMState> Ocs2ArmController::getNextState(FSMStateName stateName) const
    {
        switch (stateName)
        {
        case FSMStateName::INVALID:
            return state_list_.invalid;
        case FSMStateName::HOME:
            return state_list_.home;
        case FSMStateName::OCS2:
            return state_list_.ocs2;
        case FSMStateName::HOLD:
            return state_list_.hold;
        case FSMStateName::MOVEJ:
            return state_list_.movej;
        default:
            return state_list_.invalid;
        }
    }
} // namespace mobile_manipulator

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ocs2::mobile_manipulator::Ocs2ArmController, controller_interface::ControllerInterface);
