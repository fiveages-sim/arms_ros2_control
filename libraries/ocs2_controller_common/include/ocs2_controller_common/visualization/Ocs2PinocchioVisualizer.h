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
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

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

    /** One FK into RT Data; extract left / right / body (7-vectors). No ROS publish. */
    struct EndEffectorPoses {
        vector_t left = vector_t::Zero(7);
        vector_t right = vector_t::Zero(7);
        vector_t body = vector_t::Zero(7);
    };
    EndEffectorPoses computeEndEffectorPoses(const vector_t& state) const;

    /** Throttled PoseStamped publish (default 50 Hz). Control FK must use computeEndEffectorPoses. */
    void publishEndEffectorPoses(const rclcpp::Time& time, const EndEffectorPoses& poses) const;
    void publishEndEffectorPose(const rclcpp::Time& time, const vector_t& state) const;
    void setPosePublishPeriod(double period_sec);

    vector_t computeEndEffectorPose(const vector_t& state) const;
    vector_t computeRightEndEffectorPose(const vector_t& state) const;
    /** 7-dim pose (x,y,z, qx,qy,qz,qw) of body_frame in base; identity at origin if body_frame empty or missing. */
    vector_t computeBodyFramePose(const vector_t& state) const;

    void clearTrajectoryHistory();

private:
    void forwardKinematicsInto(pinocchio::Data& data, const vector_t& state) const;
    vector_t extractFramePose7(const pinocchio::Data& data, pinocchio::FrameIndex frame_id) const;
    static void fillPoseStamped(geometry_msgs::msg::PoseStamped& msg, const rclcpp::Time& time,
                                const std::string& frame_id, const vector_t& pose7);
    void publishPose(const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& pub,
                     const rclcpp::Time& time, const vector_t& pose7) const;

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

    /** Dedicated Data so RT and viz threads never share a pinocchio cache. */
    mutable pinocchio::Data rt_data_;
    mutable pinocchio::Data viz_data_;
    pinocchio::FrameIndex left_ee_frame_id_{0};
    pinocchio::FrameIndex right_ee_frame_id_{0};
    pinocchio::FrameIndex body_frame_id_{0};
    bool left_ee_frame_valid_{false};
    bool right_ee_frame_valid_{false};
    bool body_frame_valid_{false};

    double pose_publish_period_sec_{0.02};
    mutable rclcpp::Time last_pose_publish_time_{0, 0, RCL_ROS_TIME};
};

} // namespace ocs2::controller_common
