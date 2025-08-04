//
// Created for OCS2 Arm Controller - StateOCS2
//

#ifndef STATEOCS2_H
#define STATEOCS2_H

#include <memory>
#include <atomic>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <ocs2_core/Types.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>

#include "ocs2_arm_controller/Ocs2ArmController.h"

namespace ocs2::mobile_manipulator
{
    class StateOCS2 : public FSMState
    {
    public:
        StateOCS2(CtrlInterfaces& ctrl_interfaces,
                  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node);

        ~StateOCS2() override = default;

        void enter() override;
        void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        void exit() override;
        FSMStateName checkChange() override;

    private:
        void setupOCS2Components();
        void updateObservationFromHardware(const rclcpp::Time& time, const rclcpp::Duration& period);
        void setJointCommands();
        vector_t computeEndEffectorPose(const vector_t& joint_positions) const;

        // OCS2 components
        std::shared_ptr<MobileManipulatorInterface> interface_;
        std::unique_ptr<MPC_BASE> mpc_;
        std::unique_ptr<MPC_MRT_Interface> mpc_mrt_interface_;
        std::shared_ptr<RosReferenceManager> ros_reference_manager_;
        
        // ROS publishers
        rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr mpc_observation_publisher_;
        
        // State variables
        SystemObservation observation_;
        vector_t optimized_state_;
        vector_t optimized_input_;
        
        // Configuration
        std::string task_file_;
        std::string lib_folder_;
        std::string urdf_file_;
        std::string robot_name_;
        std::vector<std::string> joint_names_;
        
        // Timing
        rclcpp::Time last_mpc_update_time_;
        double mpc_period_;
        

        

        
        // Control interfaces
        CtrlInterfaces& ctrl_interfaces_;
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    };
}

#endif // STATEOCS2_H
