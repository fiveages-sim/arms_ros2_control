//
// Pose-based reference: ROS pose/path targets -> OCS2 TargetTrajectories (robot-agnostic context).
//
#pragma once

#include <arms_ros2_control_msgs/msg/circle_message.hpp>
#include <arms_ros2_control_msgs/srv/execute_circle.hpp>
#include <arms_ros2_control_msgs/srv/execute_path.hpp>
#include <chrono>
#include <fstream>  /* 记录实际末端位姿 */
#include <functional>
#include <mutex>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#ifdef HAS_LINA_PLANNING
#include <lina_planning/planning/path_planner/circular_curve.h>
#endif
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <ocs2_controller_common/reference/Ocs2ReferenceTargetContext.hpp>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerDecorator.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace ocs2::controller_common {

/**
 * Subscribes to pose/path topics and writes TargetTrajectories to the decorated ReferenceManager.
 * Uses Ocs2ReferenceTargetContext for dual-arm flag, base frame, and zero-input dimension.
 */
class PoseBasedReferenceManager : public ReferenceManagerDecorator {
public:
    PoseBasedReferenceManager(std::string topicPrefix,
                              std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr,
                              Ocs2ReferenceTargetContext target_context);

    ~PoseBasedReferenceManager() override = default;

    void subscribe(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);

    void setCurrentObservation(const SystemObservation& observation);

    void resetTargetStateCache();

    void setCurrentEndEffectorPoses(const vector_t& left_ee_pose, const vector_t& right_ee_pose);

    /** 记录实际末端位姿（用于轨迹日志记录） */
    bool isLoggingActive() const { return logging_active_; }
    void logActualEePose(double t, const vector_t& left_ee, const vector_t& right_ee);

    /** Body pose (7: x,y,z, qx,qy,qz,qw) for indices [14:21] when using wheel-humanoid 21-dim layout. */
    void setBodyPoseReference(const vector_t& body_pose_xyzw_7);

    /** Build full reference state for SwitchedHumanoidReferenceManager (dual arms + body). */
    static vector_t assembleWheelHumanoidTargetState(const vector_t& left_pose7_xyzw, const vector_t& right_pose7_xyzw,
                                                     const vector_t& body_pose7_xyzw);

private:
    void updateParam();
    void leftPoseCallback(geometry_msgs::msg::Pose::SharedPtr msg);
    void rightPoseCallback(geometry_msgs::msg::Pose::SharedPtr msg);
    void leftPoseStampedCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void rightPoseStampedCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void dualTargetStampedCallback(nav_msgs::msg::Path::SharedPtr msg);
    void pathCallback(nav_msgs::msg::Path::SharedPtr msg);
    void runInterpolatedPathTrajectory(
        const std::vector<vector_t>& left_arm_waypoints,
        const std::vector<vector_t>& right_arm_waypoints,
        double trajectory_duration_sec);
    void handleExecutePathService(
        const std::shared_ptr<arms_ros2_control_msgs::srv::ExecutePath::Request> request,
        std::shared_ptr<arms_ros2_control_msgs::srv::ExecutePath::Response> response);
    void updateTargetTrajectory();
    void updateTrajectory(const vector_t& previous_left_target_state, const vector_t& previous_right_target_state);

    [[nodiscard]] int effectiveTargetStateDim() const;
    [[nodiscard]] vector_t identityBodyPose7() const;
    [[nodiscard]] vector_t bodySegmentForAssembly() const;
    [[nodiscard]] vector_t assembleDualArmReferenceState(const vector_t& left7, const vector_t& right7) const;

    void leftPoseStampedPoseCallback(geometry_msgs::msg::Pose::SharedPtr msg);
    void rightPoseStampedPoseCallback(geometry_msgs::msg::Pose::SharedPtr msg);

    void processPoseStamped(const geometry_msgs::msg::PoseStamped::SharedPtr& msg,
                            std::function<void(geometry_msgs::msg::Pose::SharedPtr)> callback);

    void publishCurrentTargets(const std::string& arm_type = "");

    /** Wheel-humanoid COUPLED: after updating one arm target, set the other from captured relative pose (matches WheelHumanoidTargetNode). */
    void syncWheelHumanoidCoupledOppositeArmIfNeeded(bool left_target_was_updated);

    const std::string topic_prefix_;
    Ocs2ReferenceTargetContext target_context_;
    bool dual_arm_mode_{false};

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr left_pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr right_pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_pose_stamped_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_pose_stamped_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr dual_target_stamped_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;

    rclcpp::Service<arms_ros2_control_msgs::srv::ExecutePath>::SharedPtr execute_path_service_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_target_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_target_publisher_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<rclcpp::Clock> clock_;
    std::string base_frame_;

    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    rclcpp::Logger logger_{rclcpp::get_logger("PoseBasedReferenceManager")};

    SystemObservation current_observation_;
    vector_t left_target_state_;
    vector_t right_target_state_;
    vector_t body_pose_7_xyzw_;



    double trajectory_duration_{2.0};
    double moveL_duration_{2.0};

    bool logging_active_{false};
    double logging_end_time_{0.0};  /* 记录实际末端位姿结束时间 */
    double logging_hard_end_time_{0.0};  /* 超时兜底结束时间 */
    bool has_previous_logged_ee_{false};
    double previous_logged_ee_time_{0.0};
    vector_t previous_left_logged_ee_;
    vector_t previous_right_logged_ee_;
    size_t stationary_sample_count_{0};
    std::ofstream ee_log_file_;

#ifdef HAS_LINA_PLANNING
    std::shared_ptr<planning::CircularCurver> left_circle_curve_;
    std::shared_ptr<planning::CircularCurver> right_circle_curve_;

    rclcpp::Service<arms_ros2_control_msgs::srv::ExecuteCircle>::SharedPtr left_circle_service_;
    rclcpp::Service<arms_ros2_control_msgs::srv::ExecuteCircle>::SharedPtr right_circle_service_;

    void handleLeftCircleService(const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<arms_ros2_control_msgs::srv::ExecuteCircle::Request> request,
                                 std::shared_ptr<arms_ros2_control_msgs::srv::ExecuteCircle::Response> response);

    void handleRightCircleService(const std::shared_ptr<rmw_request_id_t> request_header,
                                  const std::shared_ptr<arms_ros2_control_msgs::srv::ExecuteCircle::Request> request,
                                  std::shared_ptr<arms_ros2_control_msgs::srv::ExecuteCircle::Response> response);

    struct ServiceExecutionState {
        bool is_executing = false;
        std::chrono::steady_clock::time_point start_time;
        double total_duration = 0.0;
        vector_t start_state;
        std::shared_ptr<planning::CircularCurver> curve;
        std::string arm_name;
        scalar_array_t time_trajectory;
        vector_array_t state_trajectory;
        size_t current_point_index = 0;
        std::mutex mutex;
    };

    ServiceExecutionState left_service_state_;
    ServiceExecutionState right_service_state_;

    void startServiceExecution(ServiceExecutionState& state, const vector_t& start_pose,
                               const arms_ros2_control_msgs::msg::CircleMessage& msg, const std::string& arm_name);

    void sendPlannedTrajectoryToOCS2(const scalar_array_t& time_traj, const vector_array_t& pose_trajectory,
                                       const std::string& arm_name);

    bool validateCircleRequest(vector_t start_pose, const arms_ros2_control_msgs::srv::ExecuteCircle::Request::SharedPtr request,
                               std::string& error_message);

    void transCircleMessageToBaseFrame(const arms_ros2_control_msgs::msg::CircleMessage& msg,
                                       arms_ros2_control_msgs::msg::CircleMessage::SharedPtr base_msg);
    bool initCircleCurve(vector_t start_pose, arms_ros2_control_msgs::msg::CircleMessage::SharedPtr msg,
                         std::shared_ptr<planning::CircularCurver> circle_ptr);
    double min_val = 1.0e-6;
#endif
};

} // namespace ocs2::controller_common
