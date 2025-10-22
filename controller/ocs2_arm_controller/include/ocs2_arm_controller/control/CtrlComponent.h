//
// Created for OCS2 Arm Controller - CtrlComponent
//

#ifndef CTRLCOMPONENT_H
#define CTRLCOMPONENT_H

#include <memory>
#include <string>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_core/Types.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_mpc/MPC_BASE.h>
#include "ocs2_arm_controller/control/PoseBasedReferenceManager.h"
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include "ocs2_arm_controller/control/Visualizer.h"
#include <std_msgs/msg/float64_multi_array.hpp>
#include "ocs2_arm_controller/filter/ButterworthFilter.h"

namespace ocs2::mobile_manipulator
{
    // Forward declaration
    struct CtrlInterfaces;

    class CtrlComponent
    {
    public:
        explicit CtrlComponent(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                               CtrlInterfaces& ctrl_interfaces);

        // MPC management
        void setupMpcComponents();
        void updateObservation(const rclcpp::Time& time);
        void evaluatePolicy(const rclcpp::Time& time);
        void resetMpc();
        void advanceMpc();

        // Visualization management
        void clearTrajectoryVisualization();

        // Torque calculation for force control
        vector_t calculateStaticTorques() const;

        //Check if the robot is close enough to the goal
        double getGoalDistance(const vector_t& goal_state, const vector_t& current_state);

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
        void loadParameters();

        // URDF path generation helper
        std::string generateUrdfPath(const std::string& robot_name, 
                                    const std::string& robot_type,
                                    const std::string& config_path) const;

        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        CtrlInterfaces& ctrl_interfaces_;
        
        rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr mpc_observation_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mpc_performance_publisher_;

        // Visualization component
        std::unique_ptr<Visualizer> visualizer_;

        // Configuration
        std::string robot_name_;
        std::string robot_type_;  // Robot type/variant (e.g., red, blue, long_arm, short_arm, etc.)
        std::vector<std::string> joint_names_;
        bool dual_arm_mode_;
        double future_time_offset_; // Future time offset
        /// filter
        bool               use_filter_;
        ButterworthFilter  joint_filter_;
        uint8_t               skin_counter_;     
        uint8_t               cur_counter_;
        /// cached MPC observation state (which is different from real observation on purpose) 
        bool                  cached_ob_state_;
        rclcpp::Time          last_execute_time_;
        vector_t     cached_last_action;
        double       hardware_latency_;
        /// safety option
        bool               use_early_stop_;
        bool               clip_mpc_cost_;
        double             max_mpc_cost_;

    };
}

#endif // CTRLCOMPONENT_H
