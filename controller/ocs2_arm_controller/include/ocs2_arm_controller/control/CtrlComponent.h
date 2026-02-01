//
// Created for OCS2 Arm Controller - CtrlComponent
//
#pragma once


#include <memory>
#include <string>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_core/Types.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include "ocs2_arm_controller/control/PoseBasedReferenceManager.h"
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <std_msgs/msg/int32.hpp>
#include "ocs2_arm_controller/control/Visualizer.h"

#include <arms_controller_common/CtrlInterfaces.h>

namespace ocs2::mobile_manipulator
{
    // Use CtrlInterfaces from arms_controller_common
    using CtrlInterfaces = arms_controller_common::CtrlInterfaces;

    class CtrlComponent
    {
    public:
        template<typename AutoDeclareFunc>
        explicit CtrlComponent(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                               CtrlInterfaces& ctrl_interfaces,
                               AutoDeclareFunc auto_declare)
            : node_(node), ctrl_interfaces_(ctrl_interfaces)
        {
            cached_ob_state_ = auto_declare("cached_ob_state", true);
            joint_speed_threshold_ = auto_declare("joint_speed_threshold", 0.1);
            hardware_latency_ = auto_declare("hardware_latency", 0.2);

            robot_name_ = auto_declare("robot_name", std::string("cr5"));
            robot_type_ = auto_declare("robot_type", std::string(""));
            future_time_offset_ = auto_declare("future_time_offset", 1.0);
            const std::string info_file_name = auto_declare("info_file_name", std::string("task"));
            joint_names_ = node_->get_parameter("joints").as_string_array();
            const std::string robot_pkg = robot_name_ + "_description";
            const std::string config_path = ament_index_cpp::get_package_share_directory(robot_pkg);

            const std::string task_file = config_path + "/config/ocs2/" + info_file_name + ".info";
            const std::string lib_folder = config_path + "/ocs2";
            const std::string urdf_file = generateUrdfPath(robot_name_, robot_type_, config_path);

            setupInterface(task_file, lib_folder, urdf_file);

            dual_arm_mode_ = interface_->dual_arm_;

            setupPublisher();

            visualizer_ = std::make_unique<Visualizer>(node_, interface_, robot_name_, urdf_file);
            visualizer_->initialize();
            RCLCPP_INFO(node_->get_logger(), "Future time offset: %.2f seconds", future_time_offset_);
            
            auto_declare("movel_trajectory_duration", 2.0);
            auto_declare("movel_duration", 2.0);
            
            pose_reference_manager_ = std::make_shared<PoseBasedReferenceManager>(
                robot_name_, interface_->getReferenceManagerPtr(), interface_);
            pose_reference_manager_->subscribe(node_);
            
            mpc_ = std::make_unique<GaussNewtonDDP_MPC>(
                interface_->mpcSettings(),
                interface_->ddpSettings(),
                interface_->getRollout(),
                interface_->getOptimalControlProblem(),
                interface_->getInitializer());

            mpc_mrt_interface_ = std::make_unique<MPC_MRT_Interface>(*mpc_);

            mpc_->getSolverPtr()->setReferenceManager(pose_reference_manager_);

            observation_.state = interface_->getInitialState();
            observation_.input = vector_t::Zero(interface_->getManipulatorModelInfo().inputDim);
            observation_.time = 0.0;
            
            // Initialize cached state
            last_execute_time_ = node_->now();
            cached_last_action_ = observation_.state;
        }

        void updateObservation(const rclcpp::Time& time);
        void evaluatePolicy(const rclcpp::Time& time);
        void resetMpc();
        void advanceMpc();

        // Visualization management
        void clearTrajectoryVisualization();

        // Torque calculation for force control
        vector_t calculateStaticTorques() const;

        // Collision detection (uses cached values from visualization, no extra computation)
        // @param threshold: Distance threshold to consider as collision (default 0.0 = actual penetration)
        bool isCollisionDetected(scalar_t threshold = 0.0) const;

        // Publish FSM command to stop all controllers
        // @param command: FSM command value (typically 2 for HOLD)
        void publishFsmCommand(int32_t command) const;

        // Get node reference
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> getNode() const { return node_; }

        // OCS2 interface (public access)
        std::shared_ptr<MobileManipulatorInterface> interface_;

        // MPC components
        std::unique_ptr<MPC_BASE> mpc_;
        std::unique_ptr<MPC_MRT_Interface> mpc_mrt_interface_;
        std::shared_ptr<PoseBasedReferenceManager> pose_reference_manager_;

        // Observation state
        SystemObservation observation_;
        vector_t optimized_state_;
        vector_t optimized_input_;

    private:
        void setupInterface(const std::string& task_file,
                            const std::string& lib_folder,
                            const std::string& urdf_file);
        void setupPublisher();

        // URDF path generation helper
        std::string generateUrdfPath(const std::string& robot_name, 
                                    const std::string& robot_type,
                                    const std::string& config_path) const;

        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        CtrlInterfaces& ctrl_interfaces_;
        
        rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr mpc_observation_publisher_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr fsm_command_publisher_;

        // Visualization component
        std::unique_ptr<Visualizer> visualizer_;

        // Configuration
        std::string robot_name_;
        std::string robot_type_;  // Robot type/variant (e.g., red, blue, long_arm, short_arm, etc.)
        std::vector<std::string> joint_names_;
        bool dual_arm_mode_;
        double future_time_offset_; // Future time offset
        /// cached MPC observation state (which is different from real observation on purpose) 
        bool                  cached_ob_state_;
        double                joint_speed_threshold_;
        rclcpp::Time          last_execute_time_;
        vector_t     cached_last_action_;
        double       hardware_latency_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
        rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter> &parameters);
    };
}
