//
// Created for OCS2 Arm Controller - CtrlComponent
//

#include "ocs2_arm_controller/control/CtrlComponent.h"
#include "ocs2_arm_controller/Ocs2ArmController.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <exception>
#include <filesystem> // Added for modern file operations

namespace ocs2::mobile_manipulator
{
    CtrlComponent::CtrlComponent(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                                 CtrlInterfaces& ctrl_interfaces)
        : node_(node), ctrl_interfaces_(ctrl_interfaces)
    {
        robot_name_ = node_->get_parameter("robot_name").as_string();
        robot_type_ = node_->get_parameter("robot_type").as_string();
        future_time_offset_ = node_->get_parameter("future_time_offset").as_double();
        joint_names_ = node_->get_parameter("joints").as_string_array();
        const std::string info_file_name = node_->get_parameter("info_file_name").as_string();

        // Automatically build file paths and initialize interface
        const std::string robot_pkg = robot_name_ + "_description";
        const std::string config_path = ament_index_cpp::get_package_share_directory(robot_pkg);
        const std::string task_file = config_path + "/config/ocs2/" + info_file_name + ".info";
        const std::string lib_folder = config_path + "/ocs2";

        // Generate URDF file path
        const std::string urdf_file = generateUrdfPath(robot_name_, robot_type_, config_path);

        // Initialize interface
        setupInterface(task_file, lib_folder, urdf_file);

        // Detect if dual arm mode is enabled
        dual_arm_mode_ = interface_->dual_arm_;
        RCLCPP_INFO(node_->get_logger(), "Dual arm mode: %s", dual_arm_mode_ ? "enabled" : "disabled");

        // Initialize MPC components
        setupMpcComponents();

        // Initialize publishers
        setupPublisher();

        // Initialize visualization component
        visualizer_ = std::make_unique<Visualizer>(node_, interface_, robot_name_);
        visualizer_->initialize();
        RCLCPP_INFO(node_->get_logger(), "Visualization component initialized");

        RCLCPP_INFO(node_->get_logger(), "CtrlComponent initialized for robot: %s", robot_name_.c_str());
        if (!robot_type_.empty())
        {
            RCLCPP_INFO(node_->get_logger(), "Robot type: %s", robot_type_.c_str());
        }
        RCLCPP_INFO(node_->get_logger(), "Future time offset: %.2f seconds", future_time_offset_);
    }

    void CtrlComponent::setupInterface(const std::string& task_file,
                                       const std::string& lib_folder,
                                       const std::string& urdf_file)
    {
        // Create Mobile Manipulator interface
        interface_ = std::make_shared<MobileManipulatorInterface>(task_file, lib_folder, urdf_file);

        // Setup publishers
        setupPublisher();

        RCLCPP_INFO(node_->get_logger(), "Mobile Manipulator Interface setup completed");
        RCLCPP_INFO(node_->get_logger(), "Task file: %s", task_file.c_str());
        RCLCPP_INFO(node_->get_logger(), "URDF file: %s", urdf_file.c_str());
    }

    void CtrlComponent::setupPublisher()
    {
        // Initialize MPC observation publisher
        mpc_observation_publisher_ = node_->create_publisher<ocs2_msgs::msg::MpcObservation>(
            robot_name_ + "_mpc_observation", 1);

        RCLCPP_INFO(node_->get_logger(), "MPC observation publisher created for %s_mpc_observation topic",
                    robot_name_.c_str());
    }

    void CtrlComponent::setupMpcComponents()
    {
        // Create RosReferenceManager and subscribe to ROS topics
        ros_reference_manager_ = std::make_shared<RosReferenceManager>(
            robot_name_, interface_->getReferenceManagerPtr());
        ros_reference_manager_->subscribe(node_);

        // Create MPC solver
        mpc_ = std::make_unique<GaussNewtonDDP_MPC>(
            interface_->mpcSettings(),
            interface_->ddpSettings(),
            interface_->getRollout(),
            interface_->getOptimalControlProblem(),
            interface_->getInitializer());

        // Create unified MPC_MRT_Interface using RosReferenceManager
        mpc_mrt_interface_ = std::make_unique<MPC_MRT_Interface>(*mpc_);

        // Important: Set RosReferenceManager to MPC solver
        mpc_->getSolverPtr()->setReferenceManager(ros_reference_manager_);

        observation_.state = interface_->getInitialState();
        observation_.input = vector_t::Zero(interface_->getManipulatorModelInfo().inputDim);
        observation_.time = 0.0; // Will be updated to current time in enter() method

        RCLCPP_INFO(node_->get_logger(), "MPC components setup completed");
        RCLCPP_INFO(node_->get_logger(), "RosReferenceManager subscribed to %s_mpc_target topic", robot_name_.c_str());
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
            observation_.state[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
        }
        observation_.input = vector_t::Zero(interface_->getManipulatorModelInfo().inputDim);
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

            // CHENHZHU: get velocity commands from optimized_input_ if in force or mixed control mode
            vector_t future_input = LinearInterpolation::interpolate(
                future_time,
                policy.timeTrajectory_,
                policy.inputTrajectory_
            );

            // CHENHZHU: Set commands based on control mode
            // Extract joint positions from state and set as commands
            if (ctrl_interfaces_.control_mode_ == ControlMode::POSITION)
            {
                for (size_t i = 0; i < joint_names_.size() && i < future_state.size(); ++i)
                {
                    ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(future_state(i));
                }
            }
            else if (ctrl_interfaces_.control_mode_ == ControlMode::FORCE)
            {
                // Calculate static torques for force control
                vector_t static_torques = calculateStaticTorques(future_state);
                
                // Set effort commands (torques) for force control
                for (size_t i = 0; i < joint_names_.size() && i < static_torques.size(); ++i)
                {
                    ctrl_interfaces_.joint_force_command_interface_[i].get().set_value(static_torques(i));
                }
                
                // Set position commands as reference
                for (size_t i = 0; i < joint_names_.size() && i < future_state.size(); ++i)
                {
                    ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(future_state(i));
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
                current_positions(i) = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
            }

            double dt = 1.0 / ctrl_interfaces_.frequency_;
            for (size_t i = 0; i < joint_names_.size() && i < optimized_input_.size(); ++i)
            {
                double new_position = current_positions(i) + optimized_input_(i) * dt;
                ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(new_position);
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
            // Dual arm mode: calculate initial end effector positions for left and right arms
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
        if (visualizer_) {
            visualizer_->clearTrajectoryHistory();
        }
    }

    std::string CtrlComponent::generateUrdfPath(const std::string& robot_name, 
                                               const std::string& robot_type,
                                               const std::string& config_path) const
    {
        const std::string urdf_dir = config_path + "/urdf/";
        
        // If robot type is specified, try to use type-specific URDF
        if (!robot_type.empty()) {
            const std::string type_specific_urdf = urdf_dir + robot_name + "_" + robot_type + ".urdf";
            
            if (std::filesystem::exists(type_specific_urdf)) {
                RCLCPP_INFO(node_->get_logger(), "Using type-specific URDF: %s", type_specific_urdf.c_str());
                return type_specific_urdf;
            } else {
                RCLCPP_WARN(node_->get_logger(), 
                    "Type-specific URDF not found: %s, falling back to default", type_specific_urdf.c_str());
            }
        }
        
        // Use default URDF
        const std::string default_urdf = urdf_dir + robot_name + ".urdf";
        RCLCPP_INFO(node_->get_logger(), "Using default URDF: %s", default_urdf.c_str());
        return default_urdf;
    }

    vector_t CtrlComponent::calculateStaticTorques(const vector_t& joint_positions, const vector_t& joint_velocities) const
    {
        // Get the Pinocchio model and data from the interface
        const auto& pinocchio_model = interface_->getPinocchioInterface().getModel();
        auto pinocchio_data = interface_->getPinocchioInterface().getData();

        // Convert OCS2 vector_t to Eigen::VectorXd for Pinocchio
        Eigen::VectorXd q = joint_positions;
        Eigen::VectorXd v = joint_velocities.size() > 0 ? joint_velocities : Eigen::VectorXd::Zero(pinocchio_model.nv);
        Eigen::VectorXd a = Eigen::VectorXd::Zero(pinocchio_model.nv);  // Zero acceleration for static torques

        // Calculate torques using RNEA (Recursive Newton-Euler Algorithm)
        // This computes the torques needed to maintain the given position with zero velocity and acceleration
        Eigen::VectorXd torques = pinocchio::rnea(pinocchio_model, pinocchio_data, q, v, a);

        // Convert back to OCS2 vector_t
        vector_t result = vector_t::Zero(torques.size());
        for (size_t i = 0; i < torques.size(); ++i)
        {
            result(i) = torques(i);
        }

        return result;
    }
} // namespace ocs2::mobile_manipulator
