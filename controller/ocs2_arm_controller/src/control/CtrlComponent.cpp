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
#include <exception>

namespace ocs2::mobile_manipulator
{
    CtrlComponent::CtrlComponent(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                                 CtrlInterfaces& ctrl_interfaces)
        : node_(node), ctrl_interfaces_(ctrl_interfaces)
    {
        robot_name_ = node_->get_parameter("robot_name").as_string();
        future_time_offset_ = node_->get_parameter("future_time_offset").as_double();
        joint_names_ = node_->get_parameter("joints").as_string_array();
        const std::string info_file_name = node_->get_parameter("info_file_name").as_string();
        // Automatically build file paths and initialize interface
        const std::string robot_pkg = robot_name_ + "_description";
        const std::string config_path = ament_index_cpp::get_package_share_directory(robot_pkg);
        const std::string task_file = config_path + "/config/ocs2/" + info_file_name + ".info";
        const std::string lib_folder = config_path + "/ocs2";
        const std::string urdf_file = config_path + "/urdf/" + robot_name_ + ".urdf";

        // Initialize interface
        setupInterface(task_file, lib_folder, urdf_file);

        // Detect if dual arm mode is enabled
        dual_arm_mode_ = interface_->dual_arm_;
        RCLCPP_INFO(node_->get_logger(), "Dual arm mode: %s", dual_arm_mode_ ? "enabled" : "disabled");

        // Initialize MPC components
        setupMpcComponents();

        // Initialize publishers
        setupPublisher();

        RCLCPP_INFO(node_->get_logger(), "CtrlComponent initialized for robot: %s", robot_name_.c_str());
        RCLCPP_INFO(node_->get_logger(), "Future time offset: %.2f seconds", future_time_offset_);
    }

    void CtrlComponent::setupInterface(const std::string& task_file,
                                       const std::string& lib_folder,
                                       const std::string& urdf_file)
    {
        // Create Mobile Manipulator interface
        interface_ = std::make_shared<MobileManipulatorInterface>(task_file, lib_folder, urdf_file);

        // Get baseFrame information
        base_frame_ = interface_->getManipulatorModelInfo().baseFrame;
        RCLCPP_INFO(node_->get_logger(), "Base frame: %s", base_frame_.c_str());

        // Setup publishers
        setupPublisher();

        RCLCPP_INFO(node_->get_logger(), "Mobile Manipulator Interface setup completed");
        RCLCPP_INFO(node_->get_logger(), "Task file: %s", task_file.c_str());
        RCLCPP_INFO(node_->get_logger(), "URDF file: %s", urdf_file.c_str());
    }

    void CtrlComponent::setupPublisher()
    {
        // Create unified left and right arm end effector pose publishers
        left_end_effector_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            robot_name_ + "_left_end_effector_pose", 1);
        right_end_effector_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            robot_name_ + "_right_end_effector_pose", 1);

        if (dual_arm_mode_)
        {
            RCLCPP_INFO(node_->get_logger(), "Dual arm mode: Left and right end effector pose publishers created");
            RCLCPP_INFO(node_->get_logger(), "  Left: %s_left_end_effector_pose", robot_name_.c_str());
            RCLCPP_INFO(node_->get_logger(), "  Right: %s_right_end_effector_pose", robot_name_.c_str());
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Single arm mode: Using left end effector pose publisher");
            RCLCPP_INFO(node_->get_logger(), "  Left: %s_left_end_effector_pose", robot_name_.c_str());
        }

        // Initialize MPC observation publisher
        mpc_observation_publisher_ = node_->create_publisher<ocs2_msgs::msg::MpcObservation>(
            robot_name_ + "_mpc_observation", 1);

        RCLCPP_INFO(node_->get_logger(), "MPC observation publisher created for %s_mpc_observation topic",
                    robot_name_.c_str());
    }

    void CtrlComponent::publishEndEffectorPose(const rclcpp::Time& time) const
    {
        if (!interface_)
        {
            RCLCPP_WARN(node_->get_logger(), "Interface not available, cannot publish end effector pose");
            return;
        }

        if (dual_arm_mode_)
        {
            // Dual arm mode: publish left and right arm end effector poses
            publishLeftEndEffectorPose(time);
            publishRightEndEffectorPose(time);
        }
        else
        {
            // Single arm mode: use left arm publisher
            publishLeftEndEffectorPose(time);
        }
    }

    void CtrlComponent::publishLeftEndEffectorPose(const rclcpp::Time& time) const
    {
        // Calculate left arm end effector pose
        const auto ee_pose = computeLeftEndEffectorPose(observation_.state);
        // Publish left arm pose information
        geometry_msgs::msg::PoseStamped ee_pose_msg;
        ee_pose_msg.header.stamp = time;
        ee_pose_msg.header.frame_id = base_frame_; // Use baseFrame instead of "world"
        ee_pose_msg.pose.position.x = ee_pose(0);
        ee_pose_msg.pose.position.y = ee_pose(1);
        ee_pose_msg.pose.position.z = ee_pose(2);
        ee_pose_msg.pose.orientation.w = ee_pose(6);
        ee_pose_msg.pose.orientation.x = ee_pose(3);
        ee_pose_msg.pose.orientation.y = ee_pose(4);
        ee_pose_msg.pose.orientation.z = ee_pose(5);
        left_end_effector_pose_publisher_->publish(ee_pose_msg);
    }

    void CtrlComponent::publishRightEndEffectorPose(const rclcpp::Time& time) const
    {
        // Calculate right arm end effector pose
        const auto ee_pose = computeRightEndEffectorPose(observation_.state);
        // Publish right arm pose information
        geometry_msgs::msg::PoseStamped ee_pose_msg;
        ee_pose_msg.header.stamp = time;
        ee_pose_msg.header.frame_id = base_frame_; // Use baseFrame instead of "world"
        ee_pose_msg.pose.position.x = ee_pose(0);
        ee_pose_msg.pose.position.y = ee_pose(1);
        ee_pose_msg.pose.position.z = ee_pose(2);
        ee_pose_msg.pose.orientation.w = ee_pose(6);
        ee_pose_msg.pose.orientation.x = ee_pose(3);
        ee_pose_msg.pose.orientation.y = ee_pose(4);
        ee_pose_msg.pose.orientation.z = ee_pose(5);
        right_end_effector_pose_publisher_->publish(ee_pose_msg);
    }

    vector_t CtrlComponent::computeEndEffectorPose(const vector_t& joint_positions) const
    {
        vector_t ee_state = vector_t::Zero(7); // 3D position + 4D quaternion

        try
        {
            // Use Pinocchio to compute end effector position and orientation
            const auto& pinocchioInterface = interface_->getPinocchioInterface();
            const auto& model = pinocchioInterface.getModel();
            auto data = pinocchioInterface.getData();

            // Forward kinematics
            pinocchio::forwardKinematics(model, data, joint_positions);
            pinocchio::updateFramePlacements(model, data);

            // Get end effector frame ID
            const auto& eeFrameName = interface_->getManipulatorModelInfo().eeFrame;
            const auto eeFrameId = model.getFrameId(eeFrameName);

            // Get end effector position and orientation
            const auto& framePlacement = data.oMf[eeFrameId];
            ee_state.head<3>() = framePlacement.translation();

            // Get quaternion and convert to ROS format [w, x, y, z]
            Eigen::Quaterniond quat(framePlacement.rotation());
            ee_state(3) = quat.x(); // x
            ee_state(4) = quat.y(); // y
            ee_state(5) = quat.z(); // z
            ee_state(6) = quat.w(); // w
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(node_->get_logger(), "Failed to compute end-effector pose: %s", e.what());
            // If computation fails, use default values
            ee_state.head<3>() = vector_t::Zero(3);
            ee_state.tail<4>() = vector_t::Zero(4);
            ee_state(3) = 1.0; // Set w component to 1
        }

        return ee_state;
    }

    vector_t CtrlComponent::computeLeftEndEffectorPose(const vector_t& joint_positions) const
    {
        vector_t ee_state = vector_t::Zero(7); // 3D position + 4D quaternion

        try
        {
            // Use Pinocchio to compute left arm end effector position and orientation
            const auto& pinocchioInterface = interface_->getPinocchioInterface();
            const auto& model = pinocchioInterface.getModel();
            auto data = pinocchioInterface.getData();

            // Forward kinematics
            pinocchio::forwardKinematics(model, data, joint_positions);
            pinocchio::updateFramePlacements(model, data);

            // Get left arm end effector frame ID (eeFrame - left arm)
            const auto& eeFrameName = interface_->getManipulatorModelInfo().eeFrame;
            const auto eeFrameId = model.getFrameId(eeFrameName);

            // Get left arm end effector position and orientation
            const auto& framePlacement = data.oMf[eeFrameId];
            ee_state.head<3>() = framePlacement.translation();

            // Get quaternion and convert to ROS format [w, x, y, z]
            Eigen::Quaterniond quat(framePlacement.rotation());
            ee_state(3) = quat.x(); // x
            ee_state(4) = quat.y(); // y
            ee_state(5) = quat.z(); // z
            ee_state(6) = quat.w(); // w
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(node_->get_logger(), "Failed to compute left end-effector pose: %s", e.what());
            // If computation fails, use default values
            ee_state.head<3>() = vector_t::Zero(3);
            ee_state.tail<4>() = vector_t::Zero(4);
            ee_state(3) = 1.0; // Set w component to 1
        }

        return ee_state;
    }

    vector_t CtrlComponent::computeRightEndEffectorPose(const vector_t& joint_positions) const
    {
        vector_t ee_state = vector_t::Zero(7); // 3D position + 4D quaternion

        try
        {
            // Use Pinocchio to compute right arm end effector position and orientation
            const auto& pinocchioInterface = interface_->getPinocchioInterface();
            const auto& model = pinocchioInterface.getModel();
            auto data = pinocchioInterface.getData();

            // Forward kinematics
            pinocchio::forwardKinematics(model, data, joint_positions);
            pinocchio::updateFramePlacements(model, data);

            // Get right arm end effector frame ID (eeFrame1 - right arm)
            const auto& eeFrameName = interface_->getManipulatorModelInfo().eeFrame1;
            const auto eeFrameId = model.getFrameId(eeFrameName);

            // Get right arm end effector position and orientation
            const auto& framePlacement = data.oMf[eeFrameId];
            ee_state.head<3>() = framePlacement.translation();

            // Get quaternion and convert to ROS format [w, x, y, z]
            Eigen::Quaterniond quat(framePlacement.rotation());
            ee_state(3) = quat.x(); // x
            ee_state(4) = quat.y(); // y
            ee_state(5) = quat.z(); // z
            ee_state(6) = quat.w(); // w
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(node_->get_logger(), "Failed to compute right end-effector pose: %s", e.what());
            // If computation fails, use default values
            ee_state.head<3>() = vector_t::Zero(3);
            ee_state.tail<4>() = vector_t::Zero(4);
            ee_state(3) = 1.0; // Set w component to 1
        }

        return ee_state;
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
        for (int i = 0; i < joint_names_.size(); ++i)
        {
            observation_.state[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
        }
        observation_.input = vector_t::Zero(interface_->getManipulatorModelInfo().inputDim);
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
            double future_time = observation_.time + future_time_offset_/ctrl_interfaces_.frequency_;

            // Use linear interpolation to get state at future time point
            vector_t future_state = LinearInterpolation::interpolate(
                future_time,
                policy.timeTrajectory_,
                policy.stateTrajectory_
            );

            // Extract joint positions from state and set as commands
            for (size_t i = 0; i < joint_names_.size() && i < future_state.size(); ++i)
            {
                ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(future_state(i));
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(node_->get_logger(), "Failed to get trajectory, falling back to integration: %s", e.what());
            // Fallback to integration method
            vector_t current_positions(joint_names_.size());
            for (int i = 0; i < joint_names_.size(); ++i)
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
    }

    void CtrlComponent::resetMpc()
    {
        // Update observation time to current time
        observation_.time = node_->now().seconds();

        TargetTrajectories target_trajectories;

        if (dual_arm_mode_)
        {
            // Dual arm mode: calculate initial end effector positions for left and right arms
            vector_t left_initial_ee_state = computeLeftEndEffectorPose(observation_.state);
            vector_t right_initial_ee_state = computeRightEndEffectorPose(observation_.state);

            // Output initial target trajectory information
            RCLCPP_INFO(node_->get_logger(),
                        "Initial Target Trajectory - Time: %.3f", observation_.time);
            RCLCPP_INFO(node_->get_logger(),
                        "Left EE State: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                        left_initial_ee_state(0), left_initial_ee_state(1), left_initial_ee_state(2), // Position x, y, z
                        left_initial_ee_state(3), left_initial_ee_state(4), left_initial_ee_state(5),
                        left_initial_ee_state(6)); // Quaternion w, x, y, z
            RCLCPP_INFO(node_->get_logger(),
                        "Right EE State: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                        right_initial_ee_state(0), right_initial_ee_state(1), right_initial_ee_state(2), // Position x, y, z
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
            vector_t initial_ee_state = computeEndEffectorPose(observation_.state);

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
        mpc_mrt_interface_->setCurrentObservation(observation_);
        mpc_mrt_interface_->resetMpcNode(target_trajectories);

        RCLCPP_INFO(node_->get_logger(), "Waiting for the initial policy ...");
        while (!mpc_mrt_interface_->initialPolicyReceived())
        {
            advanceMpc();
            rclcpp::WallRate(interface_->mpcSettings().mrtDesiredFrequency_).sleep();
        }
    }

    void CtrlComponent::advanceMpc() const
    {
        mpc_mrt_interface_->advanceMpc();
    }
} // namespace ocs2::mobile_manipulator
