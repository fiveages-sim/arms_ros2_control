//
// Pinocchio + optional self-collision visualization for in-process OCS2 controllers.
//
#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>
#include <ocs2_self_collision_visualization/GeometryInterfaceVisualization.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>

#include <ocs2_oc/oc_data/PrimalSolution.h>

namespace ocs2::controller_common {

/** Frame names and optional geometry — no MobileManipulatorInterface dependency. */
struct Ocs2VisualizerConfig {
    std::string urdf_file;
    bool dual_arm{false};
    std::string base_frame;
    std::string left_ee_frame;
    std::string right_ee_frame;
    /// Body/torso frame for reference FK (wheel humanoid). Empty: callers use identity pose for body reference block.
    std::string body_frame;
    std::optional<PinocchioGeometryInterface> pinocchio_geometry;
    scalar_t self_collision_activation_distance{0};
};

class Ocs2PinocchioVisualizer {
public:
    Ocs2PinocchioVisualizer(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                            PinocchioInterface pinocchio_interface, Ocs2VisualizerConfig config);

    void initialize();

    void updateEndEffectorTrajectory(const PrimalSolution& policy);
    /** Same as updateEndEffectorTrajectory but from a copied state trajectory (viz thread). */
    void updateEndEffectorTrajectoryFromStates(const vector_array_t& stateTrajectory);
    void publishEndEffectorTrajectory(const rclcpp::Time& time);

    /**
     * Self-collision markers via GeometryInterfaceVisualization::publishDistances.
     * Used by the async viz thread only. Returns true if FCL ran (lastMinDistance updated).
     */
    bool publishSelfCollisionVisualization(const vector_t& state) const;
    /** Min distance from the last successful publishDistances under the mutex. */
    scalar_t getLastMinDistance() const;

    void publishEndEffectorPose(const rclcpp::Time& time, const vector_t& state) const;

    vector_t computeEndEffectorPose(const vector_t& state) const;
    vector_t computeRightEndEffectorPose(const vector_t& state) const;
    /** 7-dim pose (x,y,z, qx,qy,qz,qw) of body_frame in base; identity at origin if body_frame empty or missing. */
    vector_t computeBodyFramePose(const vector_t& state) const;

    void clearTrajectoryHistory();

private:
    void publishLeftEndEffectorPose(const rclcpp::Time& time, const vector_t& state) const;
    void publishRightEndEffectorPose(const rclcpp::Time& time, const vector_t& state) const;

    visualization_msgs::msg::Marker createTrajectoryLineMarker(const std::vector<geometry_msgs::msg::Point>& points,
                                                                 const std::array<double, 3>& color, double line_width,
                                                                 const std::string& namespace_name) const;

    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
    PinocchioInterface pinocchio_interface_;
    Ocs2VisualizerConfig config_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_marker_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_end_effector_pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_end_effector_pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr body_frame_pose_publisher_;

    std::unique_ptr<GeometryInterfaceVisualization> geometry_visualization_;
    /** Guards geometry_visualization_ (viz-thread publishDistances / getLastMinDistance). */
    mutable std::mutex self_collision_mutex_;

    std::vector<vector_t> left_arm_trajectory_history_;
    std::vector<vector_t> right_arm_trajectory_history_;

    bool dual_arm_mode_{false};
    std::string base_frame_;
    std::string urdf_file_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_description_publisher_;

    double trajectory_line_width_{0.005};
    std::array<double, 3> left_arm_color_{{0.0, 0.4470, 0.7410}};
    std::array<double, 3> right_arm_color_{{0.6350, 0.0780, 0.1840}};
};

} // namespace ocs2::controller_common
