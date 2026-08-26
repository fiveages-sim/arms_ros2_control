//
// Pose-based reference: ROS pose/path targets -> OCS2 TargetTrajectories (robot-agnostic context).
//
#pragma once

#include <arms_ros2_control_msgs/msg/circle_message.hpp>
#include <arms_ros2_control_msgs/srv/execute_circle.hpp>
#include <arms_ros2_control_msgs/srv/execute_path.hpp>
#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
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
#include <vector>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace ocs2::controller_common {

/**
 * Subscribes to pose/path/twist/relative topics and writes TargetTrajectories.
 * Uses Ocs2ReferenceTargetContext for dual-arm flag, base frame, and zero-input dimension.
 *
 * Inbound (left; right symmetric when dual):
 * - left_target            Pose          immediate absolute (preempts this arm's moveL;
 *                                        other active left/right/body moveL preserved via rebuild)
 * - left_target/stamped    PoseStamped   OCS2: absolute moveL (frame_id non-base → TF→base);
 *                                        not-OCS2: forwarded to setStampedTargetHandlers (e.g. MOVEJ)
 * dual_target/stamped with 3 poses plans left+right+body together (OCS2); MOVEJ uses left/right only.
 * - left_target/twist      Twist         velocity stream (m/s, rad/s), latch + dt integrate
 *                                        (same preempt / other-preserve semantics as Pose)
 * - left_target/relative   TwistStamped  one-shot SE(3) delta (m, rad) + moveL; frame_id selects base/EE/TF
 * Body (wheel-humanoid, when body_target_enabled):
 * - body_target / body_target/stamped — same immediate vs moveL split as arms
 * - body_target/relative   TwistStamped  one-shot SE(3) delta + moveL; frame_id selects base/body/TF
 * dual_target/stamped with 3 poses plans left+right+body together.
 */
class PoseBasedReferenceManager : public ReferenceManagerDecorator {
public:
    PoseBasedReferenceManager(std::string topicPrefix,
                              std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr,
                              Ocs2ReferenceTargetContext target_context);

    ~PoseBasedReferenceManager() override = default;

    void subscribe(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node);

    /** When false, pose/twist/relative/path stay ignored; stamped is forwarded to setStampedTargetHandlers. */
    void setAcceptingTargets(bool accept);

    using PoseStampedHandler = std::function<void(const geometry_msgs::msg::PoseStamped&)>;
    using DualStampedHandler = std::function<void(const nav_msgs::msg::Path&)>;

    /**
     * Called for left/right/dual_target/stamped when accepting_targets_ is false (e.g. MOVEJ).
     * Subscriptions and TF stay in this manager; do not create a second subscriber or Buffer.
     */
    void setStampedTargetHandlers(PoseStampedHandler left,
                                  PoseStampedHandler right = {},
                                  DualStampedHandler dual = {});

    [[nodiscard]] std::shared_ptr<tf2_ros::Buffer> getTfBuffer() const;

    void setCurrentObservation(const SystemObservation& observation);

    void resetTargetStateCache();

    void setCurrentEndEffectorPoses(const vector_t& left_ee_pose, const vector_t& right_ee_pose,
                                    bool update_target_trajectory = true);

    void resetLeftEndEffectorTarget(
        const vector_t& left_ee_pose, bool update_target_trajectory = true);
    void resetRightEndEffectorTarget(
        const vector_t& right_ee_pose, bool update_target_trajectory = true);

    /** Body pose (7: x,y,z, qx,qy,qz,qw) for indices [14:21] when using wheel-humanoid 21-dim layout. */
    void setBodyPoseReference(const vector_t& body_pose_xyzw_7);
    /** Update only body target (keep arm targets unchanged), optionally pushing trajectory to MPC. */
    void setBodyPoseTargetOnly(const vector_t& body_pose_xyzw_7, bool update_target_trajectory = true);
    /** Publish cached current targets for visualization/marker refresh, without modifying MPC targets. */
    void publishCurrentTargetsFromCache();

    /**
     * Under wheel-humanoid COUPLED: update only the right-arm marker/cache from a left pose + captured
     * relative (matches BimanualCouplingConstraint when left is the measured EE). Does not push MPC
     * TargetTrajectories.
     */
    void setCoupledRightMarkerFromLeftPose(const vector_t& left_pose7_xyzw, bool publish = true);

    /**
     * Coupled leadership: record that the left arm just commanded.
     * Call when entering BIMANUAL_COUPLED so VR right packets cannot steal leadership
     * before the first left heartbeat.
     */
    void markLeftCouplingCommand();

    /** Build full reference state for SwitchedHumanoidReferenceManager (dual arms + body). */
    static vector_t assembleWheelHumanoidTargetState(const vector_t& left_pose7_xyzw, const vector_t& right_pose7_xyzw,
                                                     const vector_t& body_pose7_xyzw);

private:
    void updateParam();
    void leftPoseCallback(geometry_msgs::msg::Pose::SharedPtr msg);
    void rightPoseCallback(geometry_msgs::msg::Pose::SharedPtr msg);
    void bodyPoseCallback(geometry_msgs::msg::Pose::SharedPtr msg);
    void leftPoseStampedCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void rightPoseStampedCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void leftTwistCallback(geometry_msgs::msg::Twist::SharedPtr msg);
    void rightTwistCallback(geometry_msgs::msg::Twist::SharedPtr msg);
    void leftRelativeCallback(geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void rightRelativeCallback(geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void bodyRelativeCallback(geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void dualTargetStampedCallback(nav_msgs::msg::Path::SharedPtr msg);
    void bodyPoseStampedCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);
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
    void updateTrajectoryWithBody(const vector_t& previous_left_target_state, const vector_t& previous_right_target_state,
                                  const vector_t& previous_body_target_state);
    void updateBodyTrajectory(const vector_t& previous_body_target_state);

    [[nodiscard]] int effectiveTargetStateDim() const;
    [[nodiscard]] vector_t identityBodyPose7() const;
    [[nodiscard]] vector_t bodySegmentForAssembly() const;
    [[nodiscard]] vector_t assembleDualArmReferenceState(const vector_t& left7, const vector_t& right7) const;

    void leftPoseStampedPoseCallback(geometry_msgs::msg::Pose::SharedPtr msg);
    void rightPoseStampedPoseCallback(geometry_msgs::msg::Pose::SharedPtr msg);
    void bodyPoseStampedPoseCallback(geometry_msgs::msg::Pose::SharedPtr msg);

    /** Apply absolute 7D target [x,y,z,qx,qy,qz,qw]. interpolate=true uses moveL buffer path. */
    void applyLeftAbsoluteTarget(const vector_t& goal7, bool interpolate);
    void applyRightAbsoluteTarget(const vector_t& goal7, bool interpolate);
    void applyBodyAbsoluteTarget(const vector_t& goal7, bool interpolate);

    /** Compose one-shot relative Twist (linear=m, angular=rad RPY) onto base7. */
    [[nodiscard]] static vector_t composeTwistDelta(const vector_t& base7,
                                                    const geometry_msgs::msg::Twist& delta);

    /** Express TwistStamped delta in base_frame_ (TF-rotate free vectors); empty/base frame_id = as-is. */
    [[nodiscard]] bool twistStampedToBaseTwist(const geometry_msgs::msg::TwistStamped& msg,
                                               geometry_msgs::msg::Twist& out_twist_base) const;

    /** Integrate latched twist velocity (m/s, rad/s) for dt seconds onto base7. */
    [[nodiscard]] static vector_t integrateTwistVelocity(const vector_t& base7,
                                                         const geometry_msgs::msg::Twist& velocity,
                                                         double dt);

    void integrateLatchedTwists(double dt);
    void clearTwistLatchIfTimedOut();

    void processPoseStamped(const geometry_msgs::msg::PoseStamped::SharedPtr& msg,
                            std::function<void(geometry_msgs::msg::Pose::SharedPtr)> callback);

    /** 将 PoseStamped 转为控制器内部 7 维位姿状态 [x,y,z,qx,qy,qz,qw]。
     * 若 header.frame_id 与 base_frame_ 不同，先通过 TF 变换到 base_frame_ 再转换。
     * tag 用于转换失败时的日志标识；成功返回 true，TF 失败返回 false。 */
    [[nodiscard]] bool parsePoseStampedToState(const geometry_msgs::msg::PoseStamped& pose_stamped,
                                               const char* tag,
                                               vector_t& out_state) const;

    void publishCurrentTargets(const std::string& target_type = "");

    /** Wheel-humanoid COUPLED: after updating one arm target, set the other from captured relative pose (matches WheelHumanoidTargetNode).
     *  Returns true if the opposite arm target was synchronized. */
    [[nodiscard]] bool syncWheelHumanoidCoupledOppositeArmIfNeeded(bool left_target_was_updated);

    /** True when dual-arm wheel-humanoid is in captured BIMANUAL_COUPLED. */
    [[nodiscard]] bool isWheelHumanoidBimanualCoupled() const;

    /** True if a left-originated command arrived within kCoupledLeftLeaderTimeoutSec_. */
    [[nodiscard]] bool isCoupledLeftLeaderFresh() const;

    void rebuildTargetTrajectoriesFromActiveArmReferenceBuffers(double start_time,
                                                                  double minimum_duration);

    const std::string topic_prefix_;
    Ocs2ReferenceTargetContext target_context_;
    bool dual_arm_mode_{false};

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr left_pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr right_pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr body_pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_pose_stamped_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_pose_stamped_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr left_twist_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr right_twist_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr left_relative_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr right_relative_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr body_relative_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr dual_target_stamped_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr body_pose_stamped_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;

    rclcpp::Service<arms_ros2_control_msgs::srv::ExecutePath>::SharedPtr execute_path_service_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_target_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_target_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr body_target_publisher_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<rclcpp::Clock> clock_;
    std::string base_frame_;

    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    rclcpp::Logger logger_{rclcpp::get_logger("PoseBasedReferenceManager")};

    struct ArmReferenceBuffer {
        double startTime{0.0};
        double duration{0.0};
        std::vector<vector_t> path;
        /** Duration of each path segment. Each segment is a zero-to-zero velocity S-curve. */
        std::vector<double> segmentDurations;
    };

    ArmReferenceBuffer left_arm_reference_buffer_;
    ArmReferenceBuffer right_arm_reference_buffer_;
    /** Body/waist moveL buffer (wheel-humanoid 21-dim); same layout as arm buffers. */
    ArmReferenceBuffer body_reference_buffer_;

    /** True if any left/right/body reference buffer is still active at time. */
    [[nodiscard]] bool anyReferenceBufferActive(double time) const;

    /** 在两点 7 维位姿 [x,y,z,qx,qy,qz,qw] 之间插值；alpha∈[0,1] 时位置线性插值，姿态四元数 slerp（最短路径）。 */
    static vector_t interpolatePose7(const vector_t& start, const vector_t& goal, double alpha);
    /** 按归一化进度 alpha∈[0,1] 在 7 维位姿路径上均匀分段采样；段内调用 interpolatePose7。 */
    static vector_t samplePose7Path(const std::vector<vector_t>& path, double alpha);
    [[nodiscard]] double minimumPoseDuration(const vector_t& start, const vector_t& goal) const;
    [[nodiscard]] double samplePoseProgress(const vector_t& start, const vector_t& goal,
                                            double elapsed, double duration) const;

    void resetArmReferenceBuffer(ArmReferenceBuffer& buffer, const vector_t& pose, double time);
    /** 用 execute_path 的 waypoints 覆盖单臂 reference buffer（waypoints 不含起点）。
     *  若旧 buffer 在 start_time 仍有效，起点从旧轨迹采样；否则使用 fallback_start_pose。
     *  写入 path = [start_pose, ...waypoints]，并设置 start_time 与 duration。 */
    void setArmReferenceBufferFromWaypoints(ArmReferenceBuffer& buffer,
                                            const vector_t& fallback_start_pose,
                                            const std::vector<vector_t>& waypoints,
                                            double start_time,
                                            double duration);
    [[nodiscard]] vector_t sampleArmReferenceBuffer(const ArmReferenceBuffer& buffer,
                                                    const vector_t& fallback_pose,
                                                    double time) const;
    [[nodiscard]] bool isArmReferenceBufferActive(const ArmReferenceBuffer& buffer, double time) const;
    void rebuildTargetTrajectoriesFromArmReferenceBuffers(double start_time, double end_time);

    SystemObservation current_observation_;
    bool has_observation_time_{false};
    double last_observation_time_{0.0};
    vector_t left_target_state_;
    vector_t right_target_state_;
    vector_t body_pose_7_xyzw_;

    geometry_msgs::msg::Twist left_latched_twist_{};
    geometry_msgs::msg::Twist right_latched_twist_{};
    bool left_twist_active_{false};
    bool right_twist_active_{false};
    std::chrono::steady_clock::time_point left_twist_stamp_{};
    std::chrono::steady_clock::time_point right_twist_stamp_{};
    static constexpr double kTwistTimeoutSec_{0.2};
    /** Coupled: left remains leader while it keeps commanding; after this gap, right may lead. */
    std::chrono::steady_clock::time_point last_left_coupling_command_stamp_{};
    static constexpr double kCoupledLeftLeaderTimeoutSec_{0.2};

    double trajectory_duration_{2.0};
    double moveL_duration_{2.0};
    double moveL_sample_interval_{0.04};
    double moveL_max_linear_velocity_{0.3};
    double moveL_max_linear_acceleration_{1.0};
    double moveL_max_linear_jerk_{2.0};
    double moveL_max_angular_velocity_{1.0};
    double moveL_max_angular_acceleration_{2.0};
    double moveL_max_angular_jerk_{4.0};
    bool moveL_auto_extend_duration_{true};

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

    [[nodiscard]] bool shouldAcceptExternalTargets() const
    {
        return accepting_targets_.load(std::memory_order_acquire);
    }

    std::atomic<bool> accepting_targets_{true};
    PoseStampedHandler stamped_left_handler_;
    PoseStampedHandler stamped_right_handler_;
    DualStampedHandler stamped_dual_handler_;
};

} // namespace ocs2::controller_common
