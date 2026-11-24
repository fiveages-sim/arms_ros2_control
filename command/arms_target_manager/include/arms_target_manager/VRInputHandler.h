//
// Created for Arms ROS2 Control - VRInputHandler
//
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <atomic>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <optional>
#include "arms_target_manager/ArmsTargetManager.h"
#include "arms_controller_common/utils/FSMCommandPublisher.h"
#include <arms_ros2_control_msgs/msg/wbc_capability.hpp>
#include <arms_ros2_control_msgs/msg/teleop_mode_state.hpp>

namespace arms_ros2_control::command
{
    class VRInputHandler
    {
    public:
        VRInputHandler(
            rclcpp::Node::SharedPtr node,
            ArmsTargetManager* targetManager,
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_left_target,
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_right_target,
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_left_twist,
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_right_twist,
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_head_target,
            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dual_target_stamped,
            std::shared_ptr<tf2_ros::Buffer> tf_buffer,
            const std::vector<std::string>& handControllers,
            double linear_scale,
            double angular_scale,
            double vr_pose_scale,
            const std::string& reference_link,
            const std::string& vr_follow_frame);

        ~VRInputHandler() = default;

        void enable();

        void disable();

        bool isEnabled() const { return enabled_.load(); }

        enum class MirrorModeProfile : uint8_t
        {
            MODE_A = 0,
            MODE_B = 1,
        };

        bool isMirrorMode() const { return mirror_mode_.load(); }

        bool isMirrorModeProfileA() const
        {
            return mirror_mode_.load() &&
                   mirror_mode_profile_.load() == static_cast<int>(MirrorModeProfile::MODE_A);
        }

        bool isMirrorModeProfileB() const
        {
            return mirror_mode_.load() &&
                   mirror_mode_profile_.load() == static_cast<int>(MirrorModeProfile::MODE_B);
        }

        enum class ControlTopology
        {
            UNKNOWN,
            SPLIT_BODY,
            FULL_BODY,
        };

        ControlTopology controlTopology() const noexcept
        {
            return control_topology_.load();
        }

        bool isSplitBodyMode() const noexcept
        {
            return controlTopology() == ControlTopology::SPLIT_BODY;
        }

        bool isFullBodyMode() const noexcept
        {
            return controlTopology() == ControlTopology::FULL_BODY;
        }

        void fsmCommandCallback(std_msgs::msg::Int32::SharedPtr msg);

        void robotLeftPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

        void robotRightPoseCallback(geometry_msgs::msg::PoseStamped::SharedPtr msg);

        void wbcStateCallback(
            arms_ros2_control_msgs::msg::WbcCurrentState::ConstSharedPtr msg);

    private:
        enum class WbcToggleTarget : uint8_t
        {
            NONE = 0,
            BASE,
            BIMANUAL,
            LEFT_ARM,
            RIGHT_ARM,
            HOME_JOINT
        };

        void wbcCapabilityCallback(
            arms_ros2_control_msgs::msg::WbcCapability::ConstSharedPtr msg);

        void vrLeftCallback(geometry_msgs::msg::Pose::SharedPtr msg);
        
        void vrRightCallback(geometry_msgs::msg::Pose::SharedPtr msg);

        void vrHeadCallback(geometry_msgs::msg::Pose::SharedPtr msg);
        void robotHeadPoseCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
        bool headTrackingReady() const;
        void stopHeadTracking();
        void publishHeadTarget();

        void thumbstickAxesCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

        void triggerValuesCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
        
        void processLeftThumbstickAxes();
        void processRightThumbstickAxes();
        void processBodyThumbstickAxes();
        void publishArmTwist(bool is_left_arm, const geometry_msgs::msg::Twist& twist);
        void enterArmStickDrive(bool is_left_arm);
        void exitArmStickDrive(bool is_left_arm, bool use_left_vr);
        void syncBodyTrackingArmPoseHold();
        [[nodiscard]] bool isArmPoseMuted(bool is_left_arm) const;

        bool isBodyTrackingActive() const;

        bool updateBodyModeRequestState();

        void requestBodyMode(
            int32_t eventCase,
            const char* direction,
            const char* modeName,
            const char* command,
            int expectedBodyState);

        bool readWbcToggleState(
            WbcToggleTarget target,
            bool& enabled) const;

        void requestWbcToggle(
            int32_t eventCase,
            const char* sourceDescription,
            WbcToggleTarget target);

        bool updateRightWbcToggleRequestState();

        void updateArmWbcVrState(
            WbcToggleTarget arm,
            int armState);
        bool isArmVrInputSuppressed(WbcToggleTarget arm) const;
        bool prepareArmVrInput(WbcToggleTarget arm);
        bool isFullBodyYbPauseMode() const noexcept
        {
            return full_body_yb_pause_mode_.load();
        }
        bool isYbModeConversionPending() const;
        void resetYbModeLatchAndConversions();
        void updateYbModeConversionState();
        void applyPauseAfterRebase(WbcToggleTarget arm);
        void clearArmPause(WbcToggleTarget arm);
        bool handleCase16YbModeToggle();
        bool rebaseArmVrControlFromCurrentPose(WbcToggleTarget arm);
        void clearLastPublishedTarget(const std::string& armType);

        void publishHumanoidModeCommand(const std::string& command);

        void processChassisAxes();

        void publishChassisVelocity(const geometry_msgs::msg::Twist& velocity);
        [[nodiscard]] bool wbcOwnsChassisVelocity() const;

        void toggleChassisMode();

        void resetChassisAndWaistCommands();

        void processButtonEvent(const std_msgs::msg::Int32::SharedPtr msg);

        void handleGripActiveEdge(bool isLeft, bool pressed);
        void applyGripShortPress(bool isLeft);
        void markGripComboConsumed(bool left, bool right);

        void detectGripperControllers(const std::vector<std::string>& hand_controllers);

        void publishGripperCommand(const std::string& controller_name, int32_t command);

        void publishGripperPercent(const std::string& controller_name, double percent);

        bool isGripperPercentMode(bool is_target_left_arm) const;

        void toggleGripperPercentMode(bool is_left_trigger);

        void prepareGripperForModeSwitch(bool is_target_left_arm, bool percent_mode);

        void runScaleCalibration(bool use_left_z);

        void publishTeleopModeState(bool force = false);

        void opticalR0ReadyCallback(const std_msgs::msg::Int32::SharedPtr msg);
        void opticalR0CapsCallback(const std_msgs::msg::Int32::SharedPtr msg);
        bool opticalR0HandshakeSupported() const;
        bool opticalR0HandshakeAlive() const;
        void superviseOpticalHandshake();
        void cancelOpticalPending(const char* reason);
        void unlearnOpticalR0Peer(const char* reason);
        void publishOpticalR0Request(int32_t bits);
        int32_t desiredOpticalR0RequestBits() const;
        void refreshOpticalR0Request();
        void tryCompleteOpticalUpdateEnter();
        bool tryCompleteOpticalSideSnap(bool useLeftVr);
        void tryCompleteOpticalResumes();
        bool resumeArmVrControl(WbcToggleTarget arm, const char* buttonTag);
        bool toggleArmVrPause(WbcToggleTarget arm, const char* tag);

        void resolveTriggerTarget(bool is_left_trigger, std::string& controller_name, bool& is_target_left_arm) const;

        void toggleGripperByTrigger(bool is_left_trigger);

        void leftGripperStateCallback(const std_msgs::msg::Int32::SharedPtr msg);

        void rightGripperStateCallback(const std_msgs::msg::Int32::SharedPtr msg);

        void sendFsmCommand(int32_t command);

        int32_t resolvedFsmState() const;

        bool publishTargetPoseDirect(const std::string& armType,
                                     const Eigen::Vector3d& position,
                                     const Eigen::Quaterniond& orientation);

        void applyStaleCatchUpRamp(bool isLeft,
                                   Eigen::Vector3d& position,
                                   Eigen::Quaterniond& orientation);

        void resetStaleCatchUpRamp(bool isLeft);

        void resetStaleCatchUpRamp();

        bool isBimanualCoupled() const;

        void rebaseRightArmVrControl();

        bool transformPoseBetweenFrames(
            const std::string& armType,
            const Eigen::Vector3d& inputPosition,
            const Eigen::Quaterniond& inputOrientation,
            const std::string& sourceFrame,
            const std::string& targetFrame,
            Eigen::Vector3d& outputPosition,
            Eigen::Quaterniond& outputOrientation);

        bool calculateAndPublishTarget(
            const std::string& armType,
            const Eigen::Vector3d& vrCurrentPosition,
            const Eigen::Quaterniond& vrCurrentOrientation,
            const Eigen::Vector3d& vrBasePosition,
            const Eigen::Quaterniond& vrBaseOrientation,
            Eigen::Vector3d& calculatedPosition,
            Eigen::Quaterniond& calculatedOrientation);

        void recordLastPublishedTarget(const std::string& armType,
                                       const Eigen::Vector3d& position,
                                       const Eigen::Quaterniond& orientation);

        void clearLastPublishedTargets();

        bool setRobotBase(const std::string& armType, bool preferLastCommand);

        Eigen::Matrix4d poseMsgToMatrix(geometry_msgs::msg::Pose::SharedPtr msg);

        void matrixToPosOri(const Eigen::Matrix4d& matrix,
                            Eigen::Vector3d& position,
                            Eigen::Quaterniond& orientation);

        void calculatePoseFromDifference(const Eigen::Vector3d& vrCurrentPos,
                                         const Eigen::Quaterniond& vrCurrentOri,
                                         const Eigen::Vector3d& vrBasePos,
                                         const Eigen::Quaterniond& vrBaseOri,
                                         const Eigen::Vector3d& robotBasePos,
                                         const Eigen::Quaterniond& robotBaseOri,
                                         Eigen::Vector3d& resultPos,
                                         Eigen::Quaterniond& resultOri);

        using ListControllers = controller_manager_msgs::srv::ListControllers;

        void detectControlTopology();

        rclcpp::Client<ListControllers>::SharedPtr list_controllers_client_;
        rclcpp::TimerBase::SharedPtr control_topology_timer_;
        std::atomic<ControlTopology> control_topology_{ControlTopology::UNKNOWN};
        std::optional<rclcpp::Client<ListControllers>::FutureAndRequestId>
            pending_topology_request_;
        std::chrono::steady_clock::time_point topology_request_deadline_{};

        rclcpp::Node::SharedPtr node_;
        ArmsTargetManager* target_manager_;

        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_left_target_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_right_target_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dual_target_stamped_;
        std::unique_ptr<arms_controller_common::FSMCommandPublisher> fsm_command_publisher_;

        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_left_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_right_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_head_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_controller_state_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_optical_r0_caps_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_optical_r0_ready_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_optical_r0_request_;
        std::atomic<bool> optical_r0_peer_supports_{false};
        std::atomic<bool> optical_caps_seen_{false};
        std::atomic<int32_t> optical_r0_ready_bits_{0};
        std::atomic<int64_t> optical_r0_ready_last_ns_{0};
        std::atomic<int32_t> optical_r0_request_bits_{0};
        std::atomic<bool> optical_update_enter_pending_{false};
        std::chrono::steady_clock::time_point optical_update_enter_pending_since_{};
        std::atomic<bool> optical_resume_pending_left_{false};
        std::atomic<bool> optical_resume_pending_right_{false};
        std::chrono::steady_clock::time_point optical_resume_pending_left_since_{};
        std::chrono::steady_clock::time_point optical_resume_pending_right_since_{};
        std::atomic<bool> optical_resume_use_left_vr_left_{false};
        std::atomic<bool> optical_resume_use_left_vr_right_{false};
        std::atomic<bool> optical_fallback_warned_{false};
        rclcpp::TimerBase::SharedPtr optical_handshake_timer_;
        static constexpr int32_t optical_r0_caps_version_ = 1;
        static constexpr auto optical_r0_handshake_timeout_ =
            std::chrono::milliseconds(3000);
        static constexpr auto optical_r0_ready_alive_window_ =
            std::chrono::milliseconds(750);
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_thumbstick_axes_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_trigger_values_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_left_gripper_state_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_right_gripper_state_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_waist_lifting_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_waist_turning_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_left_twist_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_right_twist_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_body_twist_;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_mode_command_;
        rclcpp::Publisher<arms_ros2_control_msgs::msg::TeleopModeState>::SharedPtr
            pub_teleop_mode_state_;
        bool has_published_teleop_mode_state_{false};
        arms_ros2_control_msgs::msg::TeleopModeState last_teleop_mode_state_{};
        rclcpp::Subscription<arms_ros2_control_msgs::msg::WbcCapability>::SharedPtr
            sub_wbc_capability_;

        Eigen::Matrix4d left_ee_pose_ = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d right_ee_pose_ = Eigen::Matrix4d::Identity();

        Eigen::Vector3d vr_left_position_raw_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d vr_right_position_raw_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond left_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d left_position_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d right_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond right_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d vr_head_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond vr_head_orientation_ = Eigen::Quaterniond::Identity();

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_robot_head_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_head_target_;
        geometry_msgs::msg::PoseStamped robot_head_pose_;
        bool has_robot_head_pose_{false};
        bool has_vr_head_pose_{false};
        bool head_tracking_active_{false};
        Eigen::Vector3d vr_head_base_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond vr_head_base_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d robot_head_base_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond robot_head_base_orientation_ = Eigen::Quaterniond::Identity();
        std::optional<geometry_msgs::msg::PoseStamped> last_head_target_;

        Eigen::Vector3d prev_calculated_left_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond prev_calculated_left_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d prev_calculated_right_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond prev_calculated_right_orientation_ = Eigen::Quaterniond::Identity();

        bool has_last_published_left_target_ = false;
        Eigen::Vector3d last_published_left_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond last_published_left_orientation_ = Eigen::Quaterniond::Identity();
        bool has_last_published_right_target_ = false;
        Eigen::Vector3d last_published_right_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond last_published_right_orientation_ = Eigen::Quaterniond::Identity();

        struct StaleCatchUpRamp
        {
            bool has_previous_input = false;
            Eigen::Vector3d previous_input_position = Eigen::Vector3d::Zero();
            Eigen::Quaterniond previous_input_orientation = Eigen::Quaterniond::Identity();
            int frozen_frames = 0;
            Eigen::Vector3d output_position = Eigen::Vector3d::Zero();
            Eigen::Quaterniond output_orientation = Eigen::Quaterniond::Identity();
            Eigen::Vector3d residual_position = Eigen::Vector3d::Zero();
            Eigen::Quaterniond residual_orientation = Eigen::Quaterniond::Identity();
            int remaining_frames = 0;
        };
        StaleCatchUpRamp left_stale_ramp_;
        StaleCatchUpRamp right_stale_ramp_;

        std::atomic<bool> enabled_;
        std::atomic<bool> is_update_mode_;
        std::atomic<bool> mirror_mode_;
        std::atomic<int> mirror_mode_profile_{static_cast<int>(MirrorModeProfile::MODE_A)};
        std::atomic<bool> left_arm_paused_;
        std::atomic<bool> right_arm_paused_;
        std::atomic<bool> left_grip_mode_;
        std::atomic<bool> right_grip_mode_;
        std::atomic<bool> chassis_mode_;
        std::atomic<bool> left_grip_active_{false};
        std::atomic<bool> right_grip_active_{false};
        std::atomic<bool> left_grip_press_armed_{false};
        std::atomic<bool> right_grip_press_armed_{false};
        std::atomic<bool> left_grip_combo_consumed_{false};
        std::atomic<bool> right_grip_combo_consumed_{false};
        std::chrono::steady_clock::time_point left_grip_press_start_{};
        std::chrono::steady_clock::time_point right_grip_press_start_{};
        static constexpr auto grip_short_press_max_ = std::chrono::milliseconds(400);
        std::atomic<bool> left_grip_direction_suppressed_{false};
        std::atomic<bool> right_grip_direction_suppressed_{false};

        std::atomic<bool> has_home_joint_reference_{false};
        std::atomic<bool> home_joint_reference_enabled_{false};

        std::atomic<bool> right_wbc_toggle_request_pending_{false};
        std::atomic<WbcToggleTarget> requested_wbc_toggle_target_{
            WbcToggleTarget::NONE};
        std::atomic<bool> requested_wbc_toggle_enabled_{false};
        std::chrono::steady_clock::time_point right_wbc_toggle_request_time_{};
        static constexpr auto right_wbc_toggle_request_timeout_ =
            std::chrono::seconds(2);

        std::atomic<int> observed_left_wbc_arm_state_{-1};
        std::atomic<int> observed_right_wbc_arm_state_{-1};
        std::atomic<bool> left_wbc_vr_suppressed_{false};
        std::atomic<bool> right_wbc_vr_suppressed_{false};
        std::atomic<bool> left_wbc_rebase_pending_{false};
        std::chrono::steady_clock::time_point left_wbc_rebase_pending_since_{};
        std::atomic<bool> left_wbc_rebase_use_left_vr_{false};
        std::atomic<bool> right_wbc_rebase_pending_{false};
        std::chrono::steady_clock::time_point right_wbc_rebase_pending_since_{};
        std::atomic<bool> right_wbc_rebase_use_left_vr_{false};

        std::atomic<bool> full_body_yb_pause_mode_{false};
        std::atomic<bool> left_pause_after_enable_{false};
        std::atomic<bool> right_pause_after_enable_{false};
        std::atomic<bool> left_clear_pause_after_disable_{false};
        std::atomic<bool> right_clear_pause_after_disable_{false};
        std::chrono::steady_clock::time_point left_yb_conversion_time_{};
        std::chrono::steady_clock::time_point right_yb_conversion_time_{};
        static constexpr auto yb_conversion_timeout_ =
            std::chrono::seconds(2);

        std::atomic<bool> has_robot_current_left_pose_{false};
        std::atomic<bool> has_robot_current_right_pose_{false};
        std::atomic<bool> has_vr_left_pose_{false};
        std::atomic<bool> has_vr_right_pose_{false};

        bool right_vr_target_suppressed_by_bimanual_{false};

        std::atomic<int32_t> current_fsm_state_;

        std::atomic<bool> body_left_stick_alt_mode_{false};
        std::atomic<bool> body_right_stick_alt_mode_{false};
        std::atomic<bool> body_mode_request_pending_{false};
        std::atomic<int> requested_body_state_{-1};
        std::chrono::steady_clock::time_point body_mode_request_time_{};
        static constexpr auto body_mode_request_timeout_ = std::chrono::seconds(2);

        Eigen::Vector3d vr_base_left_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond vr_base_left_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d vr_base_right_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond vr_base_right_orientation_ = Eigen::Quaterniond::Identity();

        Eigen::Quaterniond left_follow_rotation_ = Eigen::Quaterniond::Identity();
        Eigen::Quaterniond right_follow_rotation_ = Eigen::Quaterniond::Identity();

        Eigen::Vector3d robot_base_left_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond robot_base_left_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d robot_base_right_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond robot_base_right_orientation_ = Eigen::Quaterniond::Identity();
        bool left_robot_base_valid_ = false;
        bool right_robot_base_valid_ = false;

        Eigen::Vector3d robot_current_left_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond robot_current_left_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d robot_current_right_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond robot_current_right_orientation_ = Eigen::Quaterniond::Identity();

        Eigen::Vector2d left_thumbstick_axes_ = Eigen::Vector2d::Zero();
        Eigen::Vector2d right_thumbstick_axes_ = Eigen::Vector2d::Zero();

        Eigen::Vector2d left_thumbstick_axes_raw_ = Eigen::Vector2d::Zero();
        Eigen::Vector2d right_thumbstick_axes_raw_ =
            Eigen::Vector2d::Zero();

        std::atomic<bool> left_arm_pose_muted_{false};
        std::atomic<bool> right_arm_pose_muted_{false};
        std::atomic<bool> body_tracking_arms_held_{false};
        static constexpr double arm_stick_deadzone_ = 0.12;

        Eigen::Vector3d paused_left_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond paused_left_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d paused_right_position_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond paused_right_orientation_ = Eigen::Quaterniond::Identity();

        std::vector<std::string> hand_controllers_;
        std::map<std::string, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> gripper_command_publishers_;
        std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> gripper_percent_publishers_;
        std::map<std::string, double> last_gripper_percent_;
        std::string left_gripper_controller_name_;
        std::string right_gripper_controller_name_;
        std::atomic<bool> left_gripper_open_;
        std::atomic<bool> right_gripper_open_;
        std::atomic<bool> left_gripper_percent_mode_;
        std::atomic<bool> right_gripper_percent_mode_;
        std::atomic<bool> left_gripper_percent_armed_;
        std::atomic<bool> right_gripper_percent_armed_;
        std::atomic<bool> left_gripper_percent_saw_release_;
        std::atomic<bool> right_gripper_percent_saw_release_;
        std::atomic<double> left_gripper_percent_hold_;
        std::atomic<double> right_gripper_percent_hold_;
        std::chrono::steady_clock::time_point last_left_gripper_mode_toggle_time_{};
        std::chrono::steady_clock::time_point last_right_gripper_mode_toggle_time_{};

        double linear_scale_;
        double angular_scale_;
        double vr_pose_scale_;

        static constexpr double chassis_linear_scale_ = 0.25;
        static constexpr double chassis_angular_scale_ = 0.5;
        static constexpr double waist_lifting_scale_ = 1.0;
        static constexpr double waist_turning_scale_ = 1.0;
        enum class WaistStickAxis { NONE, LIFTING, TURNING };
        std::atomic<WaistStickAxis> waist_stick_axis_{WaistStickAxis::NONE};
        static constexpr double waist_axis_switch_margin_ = 0.1;


        std::string reference_link_;

        std::string vr_follow_frame_;

        std::string ee_frame_id_;
        bool ee_frame_id_initialized_ = false;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

        static const int STALE_MIN_FROZEN_FRAMES;
    };
}
