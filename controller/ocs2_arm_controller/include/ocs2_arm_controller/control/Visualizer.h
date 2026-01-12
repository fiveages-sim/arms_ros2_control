//
// Created by fiveages on 8/21/25.
//

#pragma once

#include <memory>
#include <vector>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <ocs2_core/Types.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_self_collision_visualization/GeometryInterfaceVisualization.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "ocs2_oc/oc_data/PrimalSolution.h"

namespace ocs2::mobile_manipulator
{
    class Visualizer
    {
    public:
        Visualizer(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                   const std::shared_ptr<MobileManipulatorInterface>& interface,
                   const std::string& robot_name);

        ~Visualizer() = default;

        // Initialize visualization components
        void initialize();

        // Update end effector trajectory visualization
        void updateEndEffectorTrajectory(const PrimalSolution& policy);

        // Publish end effector trajectory visualization
        void publishEndEffectorTrajectory(const rclcpp::Time& time);

        // Publish self-collision distance visualization
        void publishSelfCollisionVisualization(const vector_t& state) const;

        // Check if collision was detected in last visualization update
        // @param threshold: Distance threshold to consider as collision (default 0.0 = actual penetration)
        bool isCollisionDetected(scalar_t threshold = 0.0) const;

        // End effector pose publishing
        void publishEndEffectorPose(const rclcpp::Time& time, const vector_t& state) const;

        // End effector pose computation
        vector_t computeEndEffectorPose(const vector_t& state) const;
        vector_t computeRightEndEffectorPose(const vector_t& state) const;

        // Clear trajectory history data
        void clearTrajectoryHistory();

        // Clear visualization (publish empty marker array)
        void clearVisualization(const rclcpp::Time& time) const;

    private:
        void publishLeftEndEffectorPose(const rclcpp::Time& time, const vector_t& state) const;
        void publishRightEndEffectorPose(const rclcpp::Time& time, const vector_t& state) const;

        // Create trajectory line marker
        visualization_msgs::msg::Marker createTrajectoryLineMarker(
            const std::vector<geometry_msgs::msg::Point>& points,
            const std::array<double, 3>& color,
            double line_width,
            const std::string& namespace_name) const;

        // Node and interface
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        std::shared_ptr<MobileManipulatorInterface> interface_;

        // Publishers
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_marker_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_end_effector_pose_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_end_effector_pose_publisher_; // Only created in dual arm mode

        // Self-collision visualization component
        std::unique_ptr<GeometryInterfaceVisualization> geometry_visualization_;

        // Trajectory history data
        std::vector<vector_t> left_arm_trajectory_history_;
        std::vector<vector_t> right_arm_trajectory_history_;

        // Configuration parameters
        bool enable_self_collision_;
        bool dual_arm_mode_;
        std::string robot_name_;
        std::string base_frame_;

        // Visualization parameters
        double trajectory_line_width_;
        std::array<double, 3> left_arm_color_;
        std::array<double, 3> right_arm_color_;
    };
}
