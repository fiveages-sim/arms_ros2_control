//
// Common StateCompliance — first-order hybrid force/position control
//
// Shared by ocs2_arm_controller (and any controller that instantiates it).
// Entered from HOLD via fsm_command=5; returns to HOLD on fsm_command=2.
//
// Per-axis control law (base frame, each cycle):
//   S(i)=1 (force):  v = (F_err + I) / D_force   (first-order admittance)
//   S(i)=0 (pos):    v = K_pos/(1+D) · Δx + I     (softened proportional)
//
//   qdot = J⁺_DLS · v,  q += qdot · dt,  clamp.
//
#pragma once

#include "arms_controller_common/FSM/FSMState.h"
#include "arms_controller_common/utils/GravityCompensation.h"
#include "arms_controller_common/utils/Kinematics.h"

#include <atomic>
#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <arms_ros2_control_msgs/msg/compliance_force_status.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace arms_controller_common
{
    class StateCompliance : public FSMState
    {
    public:
        explicit StateCompliance(CtrlInterfaces& ctrl_interfaces,
                                 std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = nullptr,
                                 const std::shared_ptr<GravityCompensation>& gravity_compensation = nullptr,
                                 const std::shared_ptr<ArmKinematics>& kinematics = nullptr);

        void enter() override;
        void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        void exit() override;
        FSMStateName checkChange() override;

    private:
        // ── Per-arm runtime state (index 0 = left, 1 = right) ──
        struct ArmSide
        {
            EndEffectorPose cmd_pose;
            EndEffectorPose target;
            bool target_valid{false};

            geometry_msgs::msg::Pose pending_pose;
            bool pending_pose_valid{false};

            Eigen::Matrix<double, 6, 1> pos_integral{Eigen::Matrix<double, 6, 1>::Zero()};
            Eigen::Matrix<double, 6, 1> force_integral{Eigen::Matrix<double, 6, 1>::Zero()};
            Eigen::Matrix<double, 6, 1> force_disp{Eigen::Matrix<double, 6, 1>::Zero()};
            Eigen::Matrix<double, 6, 1> wrench_filt{Eigen::Matrix<double, 6, 1>::Zero()};
            // Low-pass filtered force-axis velocity (adds virtual inertia / damping
            // to the admittance law, suppresses low-frequency drag oscillation).
            Eigen::Matrix<double, 6, 1> v_des_filt{Eigen::Matrix<double, 6, 1>::Zero()};

            std::array<double, 6> zero_cal_sum{};
            long zero_cal_samples{0};
            std::array<double, 6> wrench_bias{};

            std::array<double, 6> wrench{};
            bool ft_active{false};
            rclcpp::Time ft_stamp{0, 0, RCL_ROS_TIME};
            std::string wrench_frame_id;

            Eigen::VectorXd joint_lower;
            Eigen::VectorXd joint_upper;
            std::vector<double> dyn_param;
            size_t joint_count{0};

            std::string wrench_topic;
            rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub;
            rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_sub;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_stamped_sub;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub;
            rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr filtered_wrench_pub;

            void resetControlState()
            {
                ft_active = false;
                wrench_filt.setZero();
                v_des_filt.setZero();
                force_integral.setZero();
                pos_integral.setZero();
                force_disp.setZero();
                wrench_bias.fill(0.0);
                zero_cal_sum.fill(0.0);
                zero_cal_samples = 0;
                target_valid = false;
                pending_pose_valid = false;
            }

            void resetIo()
            {
                wrench_sub.reset();
                target_sub.reset();
                target_stamped_sub.reset();
                pose_pub.reset();
                target_pub.reset();
                filtered_wrench_pub.reset();
            }
        };

        ArmSide& arm(bool is_left) { return arms_[is_left ? 0 : 1]; }

        void updateParam();
        void setupWrenchSubscriptions();
        void setupTeleopSubscriptions();
        void setupZeroWrenchService();
        bool kinematicsAvailable() const;
        /** measured=true: joint state interfaces; else hold_positions_. */
        RobotState makeRobotState(bool measured) const;

        bool computeToolGravityWrenchInSensorFrame(
            int side_index, const std::string& sensor_frame,
            Eigen::Matrix<double, 6, 1>& wrench_sensor) const;
        Eigen::Matrix<double, 6, 1> wrenchToBase(int side_index,
                                                  const std::string& sensor_frame,
                                                  const std::array<double, 6>& wrench_ee) const;

        bool stampedPoseToBase(const geometry_msgs::msg::PoseStamped& msg,
                               geometry_msgs::msg::Pose& pose_base) const;
        bool updateTargetFromPose(bool is_left, const geometry_msgs::msg::Pose& pose_base);

        void stepHybridControl(bool is_left, double dt,
                               const Eigen::Matrix<double, 6, 1>& contact_wrench,
                               bool ft_active);

        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        std::shared_ptr<GravityCompensation> gravity_compensation_;
        std::shared_ptr<ArmKinematics> kinematics_;

        std::array<ArmSide, 2> arms_;

        // ── Shared joint command buffer (left then right) ──
        std::vector<double> hold_positions_;

        // ── Teleop / selection ──
        bool teleop_enable_{true};
        std::string teleop_base_frame_{"base_link"};
        std::mutex teleop_mutex_;
        std::vector<double> task_selection_{1.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // ── Position-axis gains ──
        // Stability: K·dt ≤ 0.3 (~20 @ 60 Hz).  D softens as K/(1+D).
        std::vector<double> hybrid_pos_stiffness_{20.0, 20.0, 20.0, 10.0, 10.0, 10.0};
        std::vector<double> hybrid_pos_damping_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double hybrid_pos_ki_{0.0};
        double hybrid_pos_ki_max_{0.02};

        // ── Force-axis gains ──
        std::vector<double> hybrid_force_damping_{5000.0, 5000.0, 5000.0, 250.0, 250.0, 250.0};
        double hybrid_force_ki_{2.0};
        double hybrid_force_ki_max_{10.0};
        double hybrid_force_ki_leak_{0.5};        // integral leakage [1/s], anti-windup during drag
        double hybrid_force_deadband_{0.5};
        std::vector<double> force_setpoint_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double force_feedback_sign_{-1.0};
        // Low-pass on admittance output v_des (virtual inertia). Smaller = more damping.
        double force_vel_lpf_alpha_{0.3};

        // ── Limits ──
        std::vector<double> hybrid_cart_vmax_{0.15, 0.15, 0.15, 0.6, 0.6, 0.6};
        double hybrid_force_xmax_lin_{0.2};
        double hybrid_force_xmax_ang_{0.3};
        double hybrid_force_xmax_margin_ratio_{0.2};  // soft-limit fade margin as fraction of xmax
        double hybrid_joint_vmax_{0.8};
        double hybrid_joint_limit_margin_{0.02};
        double hybrid_dls_lambda_{0.05};

        // ── Wrench / zero-cal ──
        double wrench_lpf_alpha_{0.15};
        double zero_cal_duration_{10.0};
        double zero_cal_settle_{0.2};
        double zero_cal_still_vel_{0.02};
        bool zero_cal_pending_{false};
        bool zero_cal_running_{false};
        bool zero_cal_done_{false};
        rclcpp::Time zero_cal_start_;

        Eigen::VectorXd q_meas_prev_all_;
        double measured_joint_vel_max_{0.0};
        double gravity_accel_{9.81};

        // ── TF / topics ──
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::string gravity_frame_{"world"};
        double ft_timeout_sec_{0.2};

        static constexpr const char* kFtFrame[2]  = {"left_ft_sensor_link", "right_ft_sensor_link"};
        static constexpr const char* kTcpFrame[2] = {"left_tcp", "right_tcp"};
        static constexpr const char* kTargetTopic[2] = {"left_target", "right_target"};
        static constexpr const char* kTargetStampedTopic[2] = {
            "left_target/stamped", "right_target/stamped"};
        static constexpr const char* kCurrentPoseTopic[2] = {
            "left_current_pose", "right_current_pose"};
        static constexpr const char* kCurrentTargetTopic[2] = {
            "left_current_target", "right_current_target"};
        static constexpr const char* kDefaultWrenchTopic[2] = {
            "/left_ft_broadcaster/wrench", "/right_ft_broadcaster/wrench"};
        static constexpr const char* kFilteredWrenchTopic[2] = {
            "/left_ft_broadcaster/wrench_filtered",
            "/right_ft_broadcaster/wrench_filtered"};
        static constexpr const char* kArmName[2] = {"left", "right"};

        rclcpp::Publisher<arms_ros2_control_msgs::msg::ComplianceForceStatus>::SharedPtr
            force_status_pub_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_wrench_service_;
        std::atomic_bool zero_cal_requested_{false};
        rclcpp::Time last_force_status_pub_{0, 0, RCL_ROS_TIME};

        std::mutex wrench_mutex_;
    };
} // namespace arms_controller_common
