//
// Created for OCS2 Arm Controller - CtrlComponent
//

#include "ocs2_arm_controller/control/CtrlComponent.h"
#include "ocs2_arm_controller/Ocs2ArmController.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include "ocs2_arm_controller/control/PoseBasedReferenceManager.h"
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <exception>
#include <filesystem>

namespace ocs2::mobile_manipulator
{

    rcl_interfaces::msg::SetParametersResult CtrlComponent::on_parameter_change(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        // Check if the 'speed' parameter was changed
        for (const auto &param : parameters)
        {
            if (param.get_name() == "hardware_latency")
            {
                hardware_latency_ = param.as_double();  // Update the speed variable
                RCLCPP_INFO(node_->get_logger(), "Updated hardware_latency to: %f", hardware_latency_);
            }
        }

        return result;
    }


    void CtrlComponent::setupInterface(const std::string& task_file,
                                       const std::string& lib_folder,
                                       const std::string& urdf_file)
    {
        // Create Mobile Manipulator interface
        interface_ = std::make_shared<MobileManipulatorInterface>(task_file, lib_folder, urdf_file);

        // Setup publishers
        setupPublisher();
    }

    void CtrlComponent::setupPublisher()
    {
        // Initialize MPC observation publisher
        mpc_observation_publisher_ = node_->create_publisher<ocs2_msgs::msg::MpcObservation>(
            robot_name_ + "_mpc_observation", 1);
        
        // Initialize FSM command publisher (for stopping all controllers)
        fsm_command_publisher_ = node_->create_publisher<std_msgs::msg::Int32>("/fsm_command", 10);
    }

    void CtrlComponent::updateObservation(const rclcpp::Time& time)
    {
        if (!mpc_mrt_interface_)
        {
            return;
        }
        // Update observation state
        observation_.time = time.seconds();
            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                auto value = ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional();
                observation_.state[i] = value.value_or(0.0);
            }
        observation_.input = vector_t::Zero(interface_->getManipulatorModelInfo().inputDim);

        // 更新PoseBasedReferenceManager的当前观测
        if (pose_reference_manager_)
        {
            pose_reference_manager_->setCurrentObservation(observation_);
        }

        visualizer_->publishSelfCollisionVisualization(observation_.state);
        visualizer_->publishEndEffectorPose(time, observation_.state);
    }

    void CtrlComponent::evaluatePolicy(const rclcpp::Time& time)
    {
        if (!mpc_mrt_interface_)
        {
            RCLCPP_WARN(node_->get_logger(), "MPC MRT interface not available");
            return;
        }
        mpc_mrt_interface_->updatePolicy();
        // use cached action as current state if the hardware has some latency to predict next action
        bool trigger_cached_state = (time - last_execute_time_).seconds() < hardware_latency_;
        if (trigger_cached_state && cached_ob_state_)
        {
            observation_.state = cached_last_action_;
        }
        
        mpc_mrt_interface_->setCurrentObservation(observation_);

        const auto observation_msg = ros_msg_conversions::createObservationMsg(observation_);
        mpc_observation_publisher_->publish(observation_msg);

        size_t planned_mode = 0;
        // Evaluate MPC policy
        mpc_mrt_interface_->evaluatePolicy(time.seconds(), observation_.state, optimized_state_, optimized_input_,
                                           planned_mode);
        try
        {
            // Get complete trajectory from MPC policy
            const auto& policy = mpc_mrt_interface_->getPolicy();

            // Calculate future time point (using configurable time offset)
            double future_time = observation_.time + future_time_offset_ / ctrl_interfaces_.frequency_;

            // Use linear interpolation to get state at future time point
            vector_t future_state = LinearInterpolation::interpolate(
                future_time,
                policy.timeTrajectory_,
                policy.stateTrajectory_
            );

            vector_t future_input = LinearInterpolation::interpolate(
                future_time,
                policy.timeTrajectory_,
                policy.inputTrajectory_
            );
            
            // Extract joint positions from state and set as commands
            if (ctrl_interfaces_.control_mode_ == ControlMode::POSITION)
            {
                for (size_t i = 0; i < joint_names_.size() && i < static_cast<size_t>(future_state.size()); ++i)
                {
                    std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(future_state(i));
                }
                cached_last_action_ = future_state;
                last_execute_time_ = time;
            }
            else if (ctrl_interfaces_.control_mode_ == ControlMode::MIX)
            {
                // Calculate static torques for force control
                vector_t static_torques = calculateStaticTorques();

                for (size_t i = 0; i < joint_names_.size() && i < static_cast<size_t>(static_torques.size()); ++i)
                {
                    std::ignore = ctrl_interfaces_.joint_force_command_interface_[i].get().set_value(static_torques(i));
                }

                for (size_t i = 0; i < joint_names_.size() && i < static_cast<size_t>(future_state.size()); ++i)
                {
                    std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(future_state(i));
                }

                for (size_t i = 0; i < joint_names_.size() && i < static_cast<size_t>(future_input.size()); ++i)
                {
                    std::ignore = ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(future_input(i));
                }
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Unknown control output mode");
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(node_->get_logger(), "Failed to get trajectory, falling back to integration: %s", e.what());
            // Fallback to integration method
            vector_t current_positions(joint_names_.size());
            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                auto value = ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional();
                current_positions(i) = value.value_or(0.0);
            }

            double dt = 1.0 / ctrl_interfaces_.frequency_;
            for (size_t i = 0; i < joint_names_.size() && i < static_cast<size_t>(optimized_input_.size()); ++i)
            {
                double new_position = current_positions(i) + optimized_input_(i) * dt;
                std::ignore = ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(new_position);
            }
        }
        visualizer_->updateEndEffectorTrajectory(mpc_mrt_interface_->getPolicy());
        visualizer_->publishEndEffectorTrajectory(node_->now());
    }

    void CtrlComponent::resetMpc()
    {
        // Update observation time to current time
        observation_.time = node_->now().seconds();

        TargetTrajectories target_trajectories;

        if (dual_arm_mode_)
        {
            vector_t left_initial_ee_state = visualizer_->computeEndEffectorPose(observation_.state);
            vector_t right_initial_ee_state = visualizer_->computeRightEndEffectorPose(observation_.state);

            // Output initial target trajectory information
            RCLCPP_INFO(node_->get_logger(),
                        "Initial Target Trajectory - Time: %.3f", observation_.time);
            RCLCPP_INFO(node_->get_logger(),
                        "Left EE State: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                        left_initial_ee_state(0), left_initial_ee_state(1), left_initial_ee_state(2),
                        // Position x, y, z
                        left_initial_ee_state(3), left_initial_ee_state(4), left_initial_ee_state(5),
                        left_initial_ee_state(6)); // Quaternion w, x, y, z
            RCLCPP_INFO(node_->get_logger(),
                        "Right EE State: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                        right_initial_ee_state(0), right_initial_ee_state(1), right_initial_ee_state(2),
                        // Position x, y, z
                        right_initial_ee_state(3), right_initial_ee_state(4), right_initial_ee_state(5),
                        right_initial_ee_state(6)); // Quaternion w, x, y, z

            // 将当前末端执行器pose设置到PoseBasedReferenceManager缓存
            if (pose_reference_manager_)
            {
                pose_reference_manager_->setCurrentObservation(observation_);
                pose_reference_manager_->setCurrentEndEffectorPoses(left_initial_ee_state, right_initial_ee_state);
            }

            // Dual arm mode: create target trajectory containing two end effectors
            // 14-dimensional state vector: [left_x, left_y, left_z, left_qw, left_qx, left_qy, left_qz,
            //                               right_x, right_y, right_z, right_qw, right_qx, right_qy, right_qz]
            vector_t dual_arm_target = (vector_t(14) <<
                left_initial_ee_state, right_initial_ee_state).finished();

            target_trajectories = TargetTrajectories({observation_.time},
                                                     {dual_arm_target},
                                                     {observation_.input});
        }
        else
        {
            // Single arm mode: maintain original logic
            // Calculate initial end effector position and orientation
            vector_t initial_ee_state = visualizer_->computeEndEffectorPose(observation_.state);

            // Output initial target trajectory information
            RCLCPP_INFO(node_->get_logger(),
                        "Initial Target Trajectory - Time: %.3f, EE State: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                        observation_.time,
                        initial_ee_state(0), initial_ee_state(1), initial_ee_state(2), // Position x, y, z
                        initial_ee_state(3), initial_ee_state(4), initial_ee_state(5),
                        initial_ee_state(6)); // Quaternion w, x, y, z

            // 将当前末端执行器pose设置到PoseBasedReferenceManager缓存（单臂模式，右臂使用零向量）
            if (pose_reference_manager_)
            {
                pose_reference_manager_->setCurrentObservation(observation_);
                vector_t zero_pose = vector_t::Zero(7);
                pose_reference_manager_->setCurrentEndEffectorPoses(initial_ee_state, zero_pose);
            }

            // Initialize TargetTrajectories - use end effector position and orientation
            target_trajectories = TargetTrajectories({observation_.time},
                                                     {initial_ee_state},
                                                     {observation_.input});
        }

        // Set initial observation and target trajectory
        mpc_mrt_interface_->reset();
        mpc_mrt_interface_->setCurrentObservation(observation_);
        mpc_mrt_interface_->resetMpcNode(target_trajectories);

        RCLCPP_INFO(node_->get_logger(), "Waiting for the initial policy ...");
        while (!mpc_mrt_interface_->initialPolicyReceived())
        {
            advanceMpc();
            rclcpp::WallRate(interface_->mpcSettings().mrtDesiredFrequency_).sleep();
        }
    }

    void CtrlComponent::advanceMpc()
    {
        mpc_mrt_interface_->advanceMpc();
    }

    void CtrlComponent::clearTrajectoryVisualization()
    {
        visualizer_->clearTrajectoryHistory();
        pose_reference_manager_->resetTargetStateCache();
    }

    std::string CtrlComponent::generateUrdfPath(const std::string& robot_name,
                                                const std::string& robot_type,
                                                const std::string& config_path) const
    {
        const std::string urdf_dir = config_path + "/urdf/";

        // If robot type is specified, try to use type-specific URDF
        if (!robot_type.empty())
        {
            const std::string type_specific_urdf = urdf_dir + robot_name + "_" + robot_type + ".urdf";

            if (std::filesystem::exists(type_specific_urdf))
            {
                RCLCPP_INFO(node_->get_logger(), "Using type-specific URDF: %s", type_specific_urdf.c_str());
                return type_specific_urdf;
            }
            RCLCPP_WARN(node_->get_logger(),
                        "Type-specific URDF not found: %s, falling back to default", type_specific_urdf.c_str());
        }

        // Use default URDF
        const std::string default_urdf = urdf_dir + robot_name + ".urdf";
        RCLCPP_INFO(node_->get_logger(), "Using default URDF: %s", default_urdf.c_str());
        return default_urdf;
    }

    vector_t CtrlComponent::calculateStaticTorques() const
    {
        // Get the Pinocchio model and data from the interface
        const auto& pinocchio_model = interface_->getPinocchioInterface().getModel();
        auto pinocchio_data = interface_->getPinocchioInterface().getData();


        Eigen::VectorXd q = observation_.state;
        return pinocchio::rnea(pinocchio_model, pinocchio_data, q,
                               Eigen::VectorXd::Zero(pinocchio_model.nv),
                               Eigen::VectorXd::Zero(pinocchio_model.nv));
    }

    bool CtrlComponent::isCollisionDetected(scalar_t threshold) const
    {
        // Reuse the cached value from visualization (no extra computation)
        return visualizer_->isCollisionDetected(threshold);
    }

    void CtrlComponent::publishFsmCommand(int32_t command) const
    {
        if (fsm_command_publisher_)
        {
            std_msgs::msg::Int32 fsm_cmd;
            fsm_cmd.data = command;
            fsm_command_publisher_->publish(fsm_cmd);
        }
    }
} // namespace ocs2::mobile_manipulator
