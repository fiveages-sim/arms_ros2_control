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
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>

namespace ocs2::mobile_manipulator
{
    // Forward declaration
    struct CtrlInterfaces;

    class CtrlComponent
    {
    public:
        explicit CtrlComponent(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                               CtrlInterfaces& ctrl_interfaces);

        // End effector pose computation and publishing
        vector_t computeEndEffectorPose(const vector_t& joint_positions) const;
        vector_t computeLeftEndEffectorPose(const vector_t& joint_positions) const;
        vector_t computeRightEndEffectorPose(const vector_t& joint_positions) const;
        void publishEndEffectorPose(const rclcpp::Time& time) const;
        void publishLeftEndEffectorPose(const rclcpp::Time& time) const;
        void publishRightEndEffectorPose(const rclcpp::Time& time) const;

        // MPC management
        void setupMpcComponents();
        void updateObservation(const rclcpp::Time& time);
        void evaluatePolicy(const rclcpp::Time& time);
        void resetMpc();
        void advanceMpc() const;

        // OCS2 interface (public access)
        std::shared_ptr<MobileManipulatorInterface> interface_;

        // MPC components
        std::unique_ptr<MPC_BASE> mpc_;
        std::unique_ptr<MPC_MRT_Interface> mpc_mrt_interface_;
        std::shared_ptr<RosReferenceManager> ros_reference_manager_;

        // Observation state
        SystemObservation observation_;
        vector_t optimized_state_;
        vector_t optimized_input_;

    private:
        void setupInterface(const std::string& task_file,
                            const std::string& lib_folder,
                            const std::string& urdf_file);
        void setupPublisher();

        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        CtrlInterfaces& ctrl_interfaces_;

        // ROS publishers - 统一使用左臂和右臂的发布器
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_end_effector_pose_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_end_effector_pose_publisher_;
        rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr mpc_observation_publisher_;

        // Configuration
        std::string robot_name_;
        std::vector<std::string> joint_names_;
        bool dual_arm_mode_;
        std::string base_frame_; // 存储baseFrame信息
    };
}

#endif // CTRLCOMPONENT_H
