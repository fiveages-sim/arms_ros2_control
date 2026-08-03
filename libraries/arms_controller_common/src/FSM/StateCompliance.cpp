//
// Common StateCompliance — first-order hybrid force/position control
//
#include "arms_controller_common/FSM/StateCompliance.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace arms_controller_common
{
    StateCompliance::StateCompliance(
        CtrlInterfaces& ctrl_interfaces,
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
        const std::shared_ptr<GravityCompensation>& gravity_compensation,
        const std::shared_ptr<ArmKinematics>& kinematics)
        : FSMState(FSMStateName::COMPLIANCE, "COMPLIANCE", ctrl_interfaces),
          node_(std::move(node)),
          gravity_compensation_(gravity_compensation),
          kinematics_(kinematics)
    {
        arms_[0].wrench_topic = kDefaultWrenchTopic[0];
        arms_[1].wrench_topic = kDefaultWrenchTopic[1];
        if (kinematics_)
        {
            arms_[0].joint_count = kinematics_->getLeftArmJointCount();
            arms_[1].joint_count = kinematics_->getRightArmJointCount();
        }
        if (node_)
        {
            tf_buffer_   = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }
    }

    FSMStateName StateCompliance::checkChange()
    {
        switch (ctrl_interfaces_.fsm_command_)
        {
        case 2:  return FSMStateName::HOLD;
        default: return FSMStateName::COMPLIANCE;
        }
    }

    void StateCompliance::updateParam()
    {
        if (!node_) return;

        auto get_double = [this](const std::string& n, double d) -> double {
            try { return node_->get_parameter(n).get_value<double>(); }
            catch (...) { return d; }
        };
        auto get_bool = [this](const std::string& n, bool b) -> bool {
            try { return node_->get_parameter(n).as_bool(); }
            catch (...) { return b; }
        };
        auto get_array = [this](const std::string& n, std::vector<double>& v) {
            try {
                const auto p = node_->get_parameter(n).as_double_array();
                if (p.size() == v.size()) v = p;
            } catch (...) {}
        };

        get_array("compliance_task_selection", task_selection_);
        get_array("compliance_hybrid_pos_stiffness", hybrid_pos_stiffness_);
        get_array("compliance_hybrid_pos_damping", hybrid_pos_damping_);
        hybrid_pos_ki_     = get_double("compliance_hybrid_pos_ki", hybrid_pos_ki_);
        hybrid_pos_ki_max_ = get_double("compliance_hybrid_pos_ki_max", hybrid_pos_ki_max_);
        get_array("compliance_hybrid_force_damping", hybrid_force_damping_);
        hybrid_force_ki_       = get_double("compliance_hybrid_force_ki", hybrid_force_ki_);
        hybrid_force_ki_max_   = get_double("compliance_hybrid_force_ki_max", hybrid_force_ki_max_);
        hybrid_force_ki_leak_  = std::max(0.0, get_double("compliance_hybrid_force_ki_leak", hybrid_force_ki_leak_));
        hybrid_force_deadband_ = get_double("compliance_hybrid_force_deadband", hybrid_force_deadband_);
        get_array("compliance_force_setpoint", force_setpoint_);
        force_feedback_sign_ = get_double("compliance_force_feedback_sign", force_feedback_sign_);
        force_vel_lpf_alpha_ = std::clamp(
            get_double("compliance_force_vel_lpf_alpha", force_vel_lpf_alpha_), 0.02, 1.0);

        get_array("compliance_hybrid_cart_vmax", hybrid_cart_vmax_);
        hybrid_force_xmax_lin_ = get_double("compliance_hybrid_force_xmax_lin", hybrid_force_xmax_lin_);
        hybrid_force_xmax_ang_ = get_double("compliance_hybrid_force_xmax_ang", hybrid_force_xmax_ang_);
        hybrid_force_xmax_margin_ratio_ = std::clamp(
            get_double("compliance_hybrid_force_xmax_margin_ratio", hybrid_force_xmax_margin_ratio_), 0.0, 0.9);

        hybrid_joint_vmax_         = get_double("compliance_hybrid_joint_vmax", hybrid_joint_vmax_);
        hybrid_joint_limit_margin_ = get_double("compliance_hybrid_joint_limit_margin", hybrid_joint_limit_margin_);
        hybrid_dls_lambda_         = get_double("compliance_hybrid_dls_lambda", hybrid_dls_lambda_);

        wrench_lpf_alpha_ = std::clamp(
            get_double("compliance_wrench_lpf_alpha", wrench_lpf_alpha_), 0.01, 1.0);

        zero_cal_duration_  = get_double("compliance_zero_cal_duration", zero_cal_duration_);
        zero_cal_settle_    = get_double("compliance_zero_cal_settle", zero_cal_settle_);
        zero_cal_still_vel_ = get_double("compliance_zero_cal_still_vel", zero_cal_still_vel_);

        try { arms_[0].dyn_param = node_->get_parameter("left_dyn_param").as_double_array(); }
        catch (...) {}
        try { arms_[1].dyn_param = node_->get_parameter("right_dyn_param").as_double_array(); }
        catch (...) {}
        gravity_accel_ = get_double("compliance_gravity_accel", gravity_accel_);

        teleop_enable_ = get_bool("compliance_teleop_enable", teleop_enable_);
        try {
            teleop_base_frame_ = node_->get_parameter("compliance_teleop_base_frame").as_string();
        } catch (...) {}
        ft_timeout_sec_ = get_double("compliance_ft_timeout_sec", ft_timeout_sec_);
        try {
            gravity_frame_ = node_->get_parameter("compliance_gravity_frame").as_string();
        } catch (...) {}
    }

    void StateCompliance::setupWrenchSubscriptions()
    {
        if (!node_) return;

        auto qos = rclcpp::SensorDataQoS();
        for (int s = 0; s < 2; ++s)
        {
            auto& a = arms_[s];
            a.wrench_sub.reset();
            if (a.wrench_topic.empty()) continue;
            a.wrench_sub = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
                a.wrench_topic, qos,
                [this, s](const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
                {
                    std::lock_guard<std::mutex> lk(wrench_mutex_);
                    auto& arr = arms_[s].wrench;
                    arr[0] = msg->wrench.force.x;
                    arr[1] = msg->wrench.force.y;
                    arr[2] = msg->wrench.force.z;
                    arr[3] = msg->wrench.torque.x;
                    arr[4] = msg->wrench.torque.y;
                    arr[5] = msg->wrench.torque.z;
                    arms_[s].ft_active = true;
                    arms_[s].ft_stamp  = msg->header.stamp;
                });
        }
    }

    void StateCompliance::setupTeleopSubscriptions()
    {
        if (!node_) return;

        auto qos = rclcpp::SensorDataQoS();
        for (int s = 0; s < 2; ++s)
        {
            auto& a = arms_[s];
            a.target_sub.reset();
            a.target_stamped_sub.reset();
            a.pose_pub.reset();
            a.target_pub.reset();

            a.target_sub = node_->create_subscription<geometry_msgs::msg::Pose>(
                kTargetTopic[s], qos,
                [this, s](const geometry_msgs::msg::Pose::SharedPtr msg) {
                    std::lock_guard<std::mutex> lk(teleop_mutex_);
                    arms_[s].pending_pose = *msg;
                    arms_[s].pending_pose_valid = true;
                });

            a.target_stamped_sub = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
                kTargetStampedTopic[s], qos,
                [this, s](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    geometry_msgs::msg::Pose pose_base;
                    if (stampedPoseToBase(*msg, pose_base))
                    {
                        std::lock_guard<std::mutex> lk(teleop_mutex_);
                        arms_[s].pending_pose = pose_base;
                        arms_[s].pending_pose_valid = true;
                    }
                });

            a.pose_pub = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
                kCurrentPoseTopic[s], 10);
            a.target_pub = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
                kCurrentTargetTopic[s], 10);
        }

        force_status_pub_ = node_->create_publisher<
            arms_ros2_control_msgs::msg::ComplianceForceStatus>(
            "compliance_force_status", 10);
    }

    void StateCompliance::enter()
    {
        updateParam();

        const size_t nj = ctrl_interfaces_.joint_position_state_interface_.size();
        hold_positions_.resize(nj);
        for (size_t i = 0; i < nj; ++i)
        {
            hold_positions_[i] = ctrl_interfaces_.last_sent_joint_positions_.size() > i
                ? ctrl_interfaces_.last_sent_joint_positions_[i] : 0.0;
        }

        if (kinematicsAvailable())
        {
            const RobotState state = makeRobotState(/*measured=*/false);
            for (int s = 0; s < 2; ++s)
            {
                if (arms_[s].joint_count > 0)
                    arms_[s].cmd_pose = kinematics_->computeSingleEndEffectorPose(state, kArmName[s]);
            }
        }

        for (auto& a : arms_) a.resetControlState();
        zero_cal_pending_ = true;
        zero_cal_running_ = false;
        zero_cal_done_    = false;
        measured_joint_vel_max_ = 0.0;
        q_meas_prev_all_.resize(0);

        if (kinematics_)
        {
            kinematics_->getJointLimits("left",  arms_[0].joint_lower, arms_[0].joint_upper);
            kinematics_->getJointLimits("right", arms_[1].joint_lower, arms_[1].joint_upper);
        }

        setupWrenchSubscriptions();
        setupTeleopSubscriptions();

        if (node_)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "COMPLIANCE (first-order hybrid): entered. "
                        "sel=[%.0f %.0f %.0f %.0f %.0f %.0f], "
                        "Fd=[%.1f %.1f %.1f] N, zero_cal=%.1f s, "
                        "kin=%s (L=%zu R=%zu)",
                        task_selection_[0], task_selection_[1], task_selection_[2],
                        task_selection_[3], task_selection_[4], task_selection_[5],
                        force_setpoint_.empty() ? 0.0 : force_setpoint_[0],
                        force_setpoint_.size() > 1 ? force_setpoint_[1] : 0.0,
                        force_setpoint_.size() > 2 ? force_setpoint_[2] : 0.0,
                        zero_cal_duration_,
                        kinematicsAvailable() ? "ok" : "MISSING",
                        arms_[0].joint_count, arms_[1].joint_count);
        }
    }

    void StateCompliance::exit()
    {
        for (auto& a : arms_) a.resetIo();
        force_status_pub_.reset();
    }

    void StateCompliance::run(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        updateParam();
        const double dt = period.seconds();

        // ── FT freshness ──
        if (node_)
        {
            const rclcpp::Time now = node_->now();
            std::lock_guard<std::mutex> lk(wrench_mutex_);
            for (auto& a : arms_)
            {
                if (!a.ft_active) continue;
                try {
                    if ((now - a.ft_stamp).seconds() > ft_timeout_sec_) {
                        a.ft_active = false;
                        RCLCPP_WARN_THROTTLE(
                            node_->get_logger(), *node_->get_clock(), 2000,
                            "COMPLIANCE FT stale (>%.0f ms)", ft_timeout_sec_ * 1000.0);
                    }
                } catch (...) { a.ft_active = false; }
            }
        }

        std::array<std::array<double, 6>, 2> raw{};
        {
            std::lock_guard<std::mutex> lk(wrench_mutex_);
            for (int s = 0; s < 2; ++s) raw[s] = arms_[s].wrench;
        }

        // ── Measured joint velocity (for zero-cal stillness) ──
        {
            const auto& pos_if = ctrl_interfaces_.joint_position_state_interface_;
            const size_t npos = pos_if.size();
            if (npos > 0 && q_meas_prev_all_.size() == static_cast<Eigen::Index>(npos) && dt > 1e-6)
            {
                double vmax = 0.0;
                for (size_t i = 0; i < npos; ++i)
                {
                    const double q = pos_if[i].get().get_optional().value_or(0.0);
                    vmax = std::max(vmax, std::abs(
                        q - q_meas_prev_all_(static_cast<Eigen::Index>(i))) / dt);
                    q_meas_prev_all_(static_cast<Eigen::Index>(i)) = q;
                }
                measured_joint_vel_max_ = 0.5 * vmax + 0.5 * measured_joint_vel_max_;
            }
            else if (npos > 0)
            {
                q_meas_prev_all_.resize(static_cast<Eigen::Index>(npos));
                for (size_t i = 0; i < npos; ++i)
                    q_meas_prev_all_(static_cast<Eigen::Index>(i)) =
                        pos_if[i].get().get_optional().value_or(0.0);
                measured_joint_vel_max_ = 0.0;
            }
        }

        // ── Tool gravity + net wrench ──
        Eigen::Matrix<double, 6, 1> w_grav[2] = {
            Eigen::Matrix<double, 6, 1>::Zero(),
            Eigen::Matrix<double, 6, 1>::Zero()};
        std::array<std::array<double, 6>, 2> net = raw;
        for (int s = 0; s < 2; ++s)
        {
            if (!arms_[s].ft_active) continue;
            if (computeToolGravityWrenchInSensorFrame(s, w_grav[s]))
                for (int i = 0; i < 6; ++i) net[s][i] -= w_grav[s](i);
        }

        // ── Zero-cal (FT tare; does NOT block position teleop) ──
        if (zero_cal_pending_ && (arms_[0].ft_active || arms_[1].ft_active))
        {
            zero_cal_pending_ = false;
            zero_cal_running_ = true;
            zero_cal_start_   = time;
            RCLCPP_INFO(node_->get_logger(),
                        "COMPLIANCE zero_cal started (%.2f s). Keep arm still.", zero_cal_duration_);
        }
        if (zero_cal_pending_ && !arms_[0].ft_active && !arms_[1].ft_active)
        {
            if (zero_cal_start_.nanoseconds() == 0)
                zero_cal_start_ = time;
            else if ((time - zero_cal_start_).seconds() > 2.0)
            {
                zero_cal_pending_ = false;
                zero_cal_done_ = true;
                RCLCPP_WARN(node_->get_logger(),
                            "COMPLIANCE: no FT within 2 s — continuing with "
                            "position teleop only (force axes inactive until FT).");
            }
        }
        if (zero_cal_running_)
        {
            const double elapsed = (time - zero_cal_start_).seconds();
            if (elapsed >= zero_cal_settle_ &&
                measured_joint_vel_max_ < zero_cal_still_vel_)
            {
                for (int s = 0; s < 2; ++s)
                {
                    for (int i = 0; i < 6; ++i)
                        arms_[s].zero_cal_sum[i] += net[s][i];
                    ++arms_[s].zero_cal_samples;
                }
            }
            if (elapsed >= zero_cal_duration_)
            {
                if (arms_[0].zero_cal_samples < 10 && elapsed < 3.0 * zero_cal_duration_)
                {
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                         "COMPLIANCE zero_cal: too few still samples (%ld) — extending...",
                                         arms_[0].zero_cal_samples);
                }
                else
                {
                    for (int s = 0; s < 2; ++s)
                    {
                        if (arms_[s].zero_cal_samples <= 0) continue;
                        for (int i = 0; i < 6; ++i)
                            arms_[s].wrench_bias[i] = arms_[s].zero_cal_sum[i] /
                                static_cast<double>(arms_[s].zero_cal_samples);
                    }
                    zero_cal_running_ = false;
                    zero_cal_done_    = true;
                    RCLCPP_INFO(node_->get_logger(),
                                "COMPLIANCE zero_cal done (%ld samples). "
                                "Bias L=[%.3f %.3f %.3f] R=[%.3f %.3f %.3f] N.",
                                arms_[0].zero_cal_samples,
                                arms_[0].wrench_bias[0], arms_[0].wrench_bias[1], arms_[0].wrench_bias[2],
                                arms_[1].wrench_bias[0], arms_[1].wrench_bias[1], arms_[1].wrench_bias[2]);
                }
            }
        }

        // ── Tare → contact wrench in base ──
        Eigen::Matrix<double, 6, 1> contact_wrench[2];
        for (int s = 0; s < 2; ++s)
        {
            std::array<double, 6> tared = net[s];
            for (int i = 0; i < 6; ++i) tared[i] -= arms_[s].wrench_bias[i];
            contact_wrench[s] = arms_[s].ft_active
                ? wrenchToBase(s, tared)
                : Eigen::Matrix<double, 6, 1>::Zero();
        }

        // ── Teleop targets ──
        if (teleop_enable_)
        {
            geometry_msgs::msg::Pose pending[2];
            bool have[2] = {false, false};
            {
                std::lock_guard<std::mutex> lk(teleop_mutex_);
                for (int s = 0; s < 2; ++s)
                {
                    if (!arms_[s].pending_pose_valid) continue;
                    pending[s] = arms_[s].pending_pose;
                    arms_[s].pending_pose_valid = false;
                    have[s] = true;
                }
            }
            for (int s = 0; s < 2; ++s)
            {
                if (!have[s]) continue;
                if (!updateTargetFromPose(s == 0, pending[s]))
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                         "COMPLIANCE teleop: %s target REJECTED (kin=%s)",
                                         kArmName[s],
                                         kinematicsAvailable() ? "ok" : "MISSING");
            }
        }

        // ── Hybrid control ──
        if (kinematicsAvailable())
        {
            stepHybridControl(true,  dt, contact_wrench[0]);
            stepHybridControl(false, dt, contact_wrench[1]);
        }
        else
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "COMPLIANCE: kinematics unavailable (L=%zu R=%zu) — not moving",
                arms_[0].joint_count, arms_[1].joint_count);
        }

        // ── Emit position commands ──
        const size_t N = std::min(
            ctrl_interfaces_.joint_position_command_interface_.size(),
            hold_positions_.size());
        for (size_t i = 0; i < N; ++i)
        {
            double cmd = hold_positions_[i];
            const bool la = i < arms_[0].joint_count;
            const Eigen::Index ai = static_cast<Eigen::Index>(
                la ? i : i - arms_[0].joint_count);
            const auto& lo = la ? arms_[0].joint_lower : arms_[1].joint_lower;
            const auto& hi = la ? arms_[0].joint_upper : arms_[1].joint_upper;
            if (ai >= 0 && ai < lo.size() && ai < hi.size() && lo(ai) < hi(ai))
            {
                cmd = std::clamp(cmd,
                    lo(ai) + hybrid_joint_limit_margin_,
                    hi(ai) - hybrid_joint_limit_margin_);
            }
            if (!std::isfinite(cmd))
            {
                cmd = i < ctrl_interfaces_.last_sent_joint_positions_.size()
                          ? ctrl_interfaces_.last_sent_joint_positions_[i] : cmd;
            }
            ctrl_interfaces_.setJointPositionCommand(i, cmd);
        }

        // ── MIX gravity feed-forward ──
        if (ctrl_interfaces_.control_mode_ == ControlMode::MIX && gravity_compensation_)
        {
            std::vector<double> current_positions;
            current_positions.reserve(
                ctrl_interfaces_.joint_position_state_interface_.size());
            for (const auto& si : ctrl_interfaces_.joint_position_state_interface_)
                current_positions.push_back(si.get().get_optional().value_or(0.0));

            const std::vector<double> static_torques =
                gravity_compensation_->calculateStaticTorques(current_positions);

            for (size_t i = 0;
                 i < ctrl_interfaces_.joint_force_command_interface_.size() &&
                 i < static_torques.size();
                 ++i)
            {
                std::ignore = ctrl_interfaces_.joint_force_command_interface_[i]
                                  .get().set_value(static_torques[i]);
            }

            if (ctrl_interfaces_.default_gains_.size() >= 2)
            {
                const double kp = ctrl_interfaces_.default_gains_[0];
                const double kd = ctrl_interfaces_.default_gains_[1];
                for (auto& kp_if : ctrl_interfaces_.joint_kp_command_interface_)
                    std::ignore = kp_if.get().set_value(kp);
                for (auto& kd_if : ctrl_interfaces_.joint_kd_command_interface_)
                    std::ignore = kd_if.get().set_value(kd);
            }
        }

        // ── Diagnostics ──
        if (node_)
        {
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 3000,
                "COMPLIANCE status: zero_cal=%s FT(L=%s R=%s) "
                "force_setpoint=%.1f N I_L=%.1f F_filt_L=%.2f N",
                zero_cal_done_ ? "DONE" :
                    (zero_cal_running_ ? "running" : "waiting_FT"),
                arms_[0].ft_active ? "on" : "OFF",
                arms_[1].ft_active ? "on" : "OFF",
                force_setpoint_.empty() ? 0.0 : force_setpoint_[0],
                arms_[0].force_integral(0),
                arms_[0].wrench_filt(0));
        }
        if (node_ && kinematicsAvailable())
        {
            auto log_arm = [&](const char* tag, const Eigen::Matrix<double, 6, 1>& wrench,
                               const ArmSide& a)
            {
                Eigen::Vector3d de = Eigen::Vector3d::Zero();
                if (a.target_valid) de = a.target.position - a.cmd_pose.position;
                double pos_err = 0, force_err = 0;
                for (int i = 0; i < 3; ++i)
                    (task_selection_[i] > 0.5 ? force_err : pos_err) += de(i) * de(i);

                double ang_err = 0, ang_pos = 0, ang_force = 0;
                if (a.target_valid)
                {
                    const auto Rr = a.target.rotationMatrix * a.cmd_pose.rotationMatrix.transpose();
                    const double ca = std::clamp((Rr.trace() - 1.0) * 0.5, -1.0, 1.0);
                    const double ang = std::acos(ca);
                    Eigen::Vector3d rv = Eigen::Vector3d::Zero();
                    if (ang > 1e-9) {
                        rv << Rr(2,1)-Rr(1,2), Rr(0,2)-Rr(2,0), Rr(1,0)-Rr(0,1);
                        const double s2 = 2.0 * std::sin(ang);
                        if (std::abs(s2) > 1e-9) rv /= s2;
                        rv *= ang;
                    }
                    ang_err = rv.norm();
                    for (int i = 0; i < 3; ++i)
                        (task_selection_[i+3] > 0.5 ? ang_force : ang_pos) += rv(i) * rv(i);
                }
                const double kR = 180.0 / M_PI;
                RCLCPP_INFO_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 200,
                    "COMPLIANCE %s: F=[%.2f %.2f %.2f]N M=[%.2f %.2f %.2f]Nm "
                    "pos=%.1fmm force=%.1fmm ang=%.1fdeg ang_p=%.1fdeg ang_f=%.1fdeg "
                    "disp_lin=[%.3f %.3f %.3f]mm disp_ang=[%.2f %.2f %.2f]deg",
                    tag, wrench(0),wrench(1),wrench(2), wrench(3),wrench(4),wrench(5),
                    std::sqrt(pos_err)*1e3, std::sqrt(force_err)*1e3,
                    ang_err*kR, std::sqrt(ang_pos)*kR, std::sqrt(ang_force)*kR,
                    std::abs(a.force_disp(0))*1e3, std::abs(a.force_disp(1))*1e3,
                    std::abs(a.force_disp(2))*1e3,
                    std::abs(a.force_disp(3))*kR, std::abs(a.force_disp(4))*kR,
                    std::abs(a.force_disp(5))*kR);
            };
            log_arm("L", contact_wrench[0], arms_[0]);
            log_arm("R", contact_wrench[1], arms_[1]);
        }

        // ── Publish TCP / teleop target for RViz ──
        if (node_)
        {
            auto pub_pose = [this](const EndEffectorPose& pose,
                                   const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& pub)
            {
                if (!pub) return;
                geometry_msgs::msg::PoseStamped msg;
                msg.header.stamp = node_->now();
                msg.header.frame_id = teleop_base_frame_.empty() ? "base_link" : teleop_base_frame_;
                msg.pose.position.x = pose.position.x();
                msg.pose.position.y = pose.position.y();
                msg.pose.position.z = pose.position.z();
                Eigen::Quaterniond q(pose.rotationMatrix);
                msg.pose.orientation.x = q.x();
                msg.pose.orientation.y = q.y();
                msg.pose.orientation.z = q.z();
                msg.pose.orientation.w = q.w();
                pub->publish(msg);
            };
            for (int s = 0; s < 2; ++s)
            {
                pub_pose(arms_[s].cmd_pose, arms_[s].pose_pub);
                pub_pose(arms_[s].target_valid ? arms_[s].target : arms_[s].cmd_pose,
                         arms_[s].target_pub);
            }
        }

        // ── Force status (~20 Hz) ──
        if (force_status_pub_)
        {
            const bool first = last_force_status_pub_.nanoseconds() == 0;
            if (first || (time - last_force_status_pub_).seconds() >= 0.05)
            {
                arms_ros2_control_msgs::msg::ComplianceForceStatus msg;
                msg.header.stamp = time;
                msg.header.frame_id =
                    teleop_base_frame_.empty() ? "base_link" : teleop_base_frame_;
                for (int i = 0; i < 6; ++i)
                {
                    msg.task_selection[i] =
                        i < static_cast<int>(task_selection_.size()) ? task_selection_[i] : 0.0;
                    msg.force_setpoint[i] =
                        i < static_cast<int>(force_setpoint_.size()) ? force_setpoint_[i] : 0.0;
                    msg.force_measured_left[i] =
                        force_feedback_sign_ * arms_[0].wrench_filt(i);
                    msg.force_measured_right[i] =
                        force_feedback_sign_ * arms_[1].wrench_filt(i);
                }
                msg.left_ft_active = arms_[0].ft_active;
                msg.right_ft_active = arms_[1].ft_active;
                msg.zero_cal_done = zero_cal_done_;
                msg.force_feedback_sign = force_feedback_sign_;
                force_status_pub_->publish(msg);
                last_force_status_pub_ = time;
            }
        }
    }

    void StateCompliance::stepHybridControl(
        bool is_left, double dt,
        const Eigen::Matrix<double, 6, 1>& contact_wrench)
    {
        if (dt <= 1e-6 || !kinematicsAvailable()) return;

        ArmSide& a = arm(is_left);
        const size_t offset = is_left ? 0 : arms_[0].joint_count;
        const size_t nq = a.joint_count;
        if (nq == 0 || offset + nq > hold_positions_.size()) return;

        auto S = [&](int i) -> double {
            return i < static_cast<int>(task_selection_.size()) ? task_selection_[i] : 0.0;
        };

        // Actual FK/Jacobian (closes loop around real robot)
        RobotState state_actual = makeRobotState(/*measured=*/true);

        const EndEffectorPose cur =
            kinematics_->computeSingleEndEffectorPose(state_actual, kArmName[is_left ? 0 : 1]);
        const EndEffectorPose tgt = a.target_valid ? a.target : cur;
        // Orientation error uses commanded FK (rotation barely affected by sag)
        const Eigen::Matrix3d R_cur_for_rot = a.cmd_pose.rotationMatrix;

        const Eigen::MatrixXd J = kinematics_->computeJacobian(
            state_actual, kArmName[is_left ? 0 : 1]);
        if (J.rows() < 6 || J.cols() != static_cast<Eigen::Index>(nq) || !J.allFinite())
            return;

        a.wrench_filt = wrench_lpf_alpha_ * contact_wrench +
                        (1.0 - wrench_lpf_alpha_) * a.wrench_filt;

        Eigen::Matrix<double, 6, 1> v_des = Eigen::Matrix<double, 6, 1>::Zero();

        // Position axes: v = K/(1+D)·Δx + I
        {
            const double ki_dt = hybrid_pos_ki_ * dt;
            const Eigen::Vector3d dp = tgt.position - cur.position;
            for (int i = 0; i < 3; ++i) {
                if (S(i) > 0.5) continue;
                const double K = i < static_cast<int>(hybrid_pos_stiffness_.size())
                                     ? hybrid_pos_stiffness_[i] : 20.0;
                const double D = i < static_cast<int>(hybrid_pos_damping_.size())
                                     ? std::max(0.0, hybrid_pos_damping_[i]) : 0.0;
                a.pos_integral(i) += ki_dt * dp(i);
                a.pos_integral(i) = std::clamp(
                    a.pos_integral(i), -hybrid_pos_ki_max_, hybrid_pos_ki_max_);
                v_des(i) = (K / (1.0 + D)) * dp(i) + a.pos_integral(i);
            }

            const Eigen::Matrix3d R_rel = tgt.rotationMatrix * R_cur_for_rot.transpose();
            const double cos_a = std::clamp((R_rel.trace() - 1.0) * 0.5, -1.0, 1.0);
            const double angle = std::acos(cos_a);
            Eigen::Vector3d rotvec = Eigen::Vector3d::Zero();
            if (angle > 1e-9) {
                rotvec << R_rel(2,1)-R_rel(1,2), R_rel(0,2)-R_rel(2,0), R_rel(1,0)-R_rel(0,1);
                const double s2 = 2.0 * std::sin(angle);
                if (std::abs(s2) > 1e-9) rotvec /= s2;
                rotvec *= angle;
            }
            for (int i = 0; i < 3; ++i) {
                if (S(i+3) > 0.5) continue;
                const double K = i+3 < static_cast<int>(hybrid_pos_stiffness_.size())
                                     ? hybrid_pos_stiffness_[i+3] : 10.0;
                const double D = i+3 < static_cast<int>(hybrid_pos_damping_.size())
                                     ? std::max(0.0, hybrid_pos_damping_[i+3]) : 0.0;
                a.pos_integral(i+3) += ki_dt * rotvec(i);
                a.pos_integral(i+3) = std::clamp(
                    a.pos_integral(i+3), -hybrid_pos_ki_max_, hybrid_pos_ki_max_);
                v_des(i+3) = (K / (1.0 + D)) * rotvec(i) + a.pos_integral(i+3);
            }
        }

        // Force axes: v = (F_err + I) / D
        {
            const bool ft_ok = zero_cal_done_ && a.ft_active;
            for (int i = 0; i < 6; ++i) {
                if (S(i) < 0.5) continue;
                if (!ft_ok) { v_des(i) = 0.0; continue; }

                const double F_des = i < static_cast<int>(force_setpoint_.size())
                                         ? force_setpoint_[i] : 0.0;
                const double f_err = F_des - force_feedback_sign_ * a.wrench_filt(i);
                const double D = i < static_cast<int>(hybrid_force_damping_.size())
                                     ? hybrid_force_damping_[i] : (i < 3 ? 2000.0 : 100.0);

                if (std::abs(f_err) >= hybrid_force_deadband_)
                {
                    // Leakage: force_integral *= (1 - leak·dt). Prevents wind-up
                    // accumulation during sustained drag (main oscillation driver).
                    a.force_integral(i) *= std::max(0.0, 1.0 - hybrid_force_ki_leak_ * dt);
                    a.force_integral(i) += hybrid_force_ki_ * dt * f_err;
                    a.force_integral(i) = std::clamp(
                        a.force_integral(i), -hybrid_force_ki_max_, hybrid_force_ki_max_);
                }
                v_des(i) = (f_err + a.force_integral(i)) / std::max(D, 1e-3);

                // Idle force axis: blend teleop position tracking
                if (std::abs(f_err) < hybrid_force_deadband_ && a.target_valid && i < 3)
                {
                    const double e_pos = tgt.position(i) - cur.position(i);
                    if (std::abs(e_pos) > 1e-6)
                    {
                        const double K = i < static_cast<int>(hybrid_pos_stiffness_.size())
                                             ? hybrid_pos_stiffness_[i] : 20.0;
                        v_des(i) += K * e_pos;
                    }
                }
            }

            if (node_)
            {
                std::string axes;
                for (int i = 0; i < 6; ++i)
                {
                    if (S(i) < 0.5) continue;
                    const double F_des = i < static_cast<int>(force_setpoint_.size())
                                             ? force_setpoint_[i] : 0.0;
                    const double F_meas = force_feedback_sign_ * a.wrench_filt(i);
                    char buf[128];
                    std::snprintf(buf, sizeof(buf),
                                  "  F%d: set=%.2f meas=%.2f err=%.2f int=%.2f v=%.4f",
                                  i, F_des, F_meas, F_des - F_meas,
                                  a.force_integral(i), v_des(i));
                    axes += buf;
                }
                if (!axes.empty())
                {
                    RCLCPP_INFO_THROTTLE(
                        node_->get_logger(), *node_->get_clock(), 2000,
                        "COMPLIANCE force_ctl_%s: zero_cal=%d ft_ok=%d dt=%.4f%s",
                        is_left ? "L" : "R",
                        (int)zero_cal_done_, (int)ft_ok, dt, axes.c_str());
                }
            }
        }

        // Force-axis admittance output damping: low-pass v_des to add virtual
        // inertia. This is the primary suppressor of the ~1-3 Hz drag oscillation
        // (pure proportional admittance + joint-tracking lag → positive feedback).
        for (int i = 0; i < 6; ++i)
        {
            if (S(i) < 0.5) { a.v_des_filt(i) = 0.0; continue; }
            a.v_des_filt(i) = force_vel_lpf_alpha_ * v_des(i) +
                              (1.0 - force_vel_lpf_alpha_) * a.v_des_filt(i);
            v_des(i) = a.v_des_filt(i);
        }

        for (int i = 0; i < 6; ++i) {
            const double vmax = i < static_cast<int>(hybrid_cart_vmax_.size())
                                    ? hybrid_cart_vmax_[i] : (i < 3 ? 0.05 : 0.3);
            v_des(i) = std::clamp(v_des(i), -vmax, vmax);
        }

        // Force-axis displacement soft limit
        for (int i = 0; i < 6; ++i) {
            if (S(i) < 0.5) { a.force_disp(i) = 0.0; continue; }
            const double xmax = i < 3 ? hybrid_force_xmax_lin_ : hybrid_force_xmax_ang_;
            const double margin = hybrid_force_xmax_margin_ratio_ * xmax;
            const double proposed = a.force_disp(i) + v_des(i) * dt;
            if (std::abs(proposed) > xmax)
                v_des(i) = (std::copysign(xmax, proposed) - a.force_disp(i)) / dt;
            else if (std::abs(proposed) > xmax - margin)
                v_des(i) *= std::clamp((xmax - std::abs(proposed)) / margin, 0.0, 1.0);
            a.force_disp(i) += v_des(i) * dt;
            a.force_disp(i) = std::clamp(a.force_disp(i), -xmax, xmax);
        }

        const Eigen::MatrixXd JJt = J * J.transpose();
        const Eigen::MatrixXd reg = JJt + hybrid_dls_lambda_ * hybrid_dls_lambda_ *
            Eigen::MatrixXd::Identity(JJt.rows(), JJt.cols());
        Eigen::VectorXd qdot = J.transpose() * reg.ldlt().solve(v_des);
        if (!qdot.allFinite()) return;

        for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(nq); ++i)
            qdot(i) = std::clamp(qdot(i), -hybrid_joint_vmax_, hybrid_joint_vmax_);

        for (size_t i = 0; i < nq; ++i) {
            const Eigen::Index ai = static_cast<Eigen::Index>(i);
            const double old = hold_positions_[offset + i];
            double new_pos = old + qdot(ai) * dt;
            if (ai < a.joint_lower.size() && ai < a.joint_upper.size() &&
                a.joint_lower(ai) < a.joint_upper(ai))
            {
                const double lo = a.joint_lower(ai) + hybrid_joint_limit_margin_;
                const double hi = a.joint_upper(ai) - hybrid_joint_limit_margin_;
                const double clamped = std::clamp(new_pos, lo, hi);
                if (clamped != new_pos) qdot(ai) = (clamped - old) / dt;
                new_pos = clamped;
            }
            hold_positions_[offset + i] = new_pos;
        }

        a.cmd_pose = kinematics_->computeSingleEndEffectorPose(
            makeRobotState(/*measured=*/false), kArmName[is_left ? 0 : 1]);
    }

    bool StateCompliance::kinematicsAvailable() const
    {
        return kinematics_ != nullptr &&
               (arms_[0].joint_count > 0 || arms_[1].joint_count > 0);
    }

    RobotState StateCompliance::makeRobotState(bool measured) const
    {
        RobotState state(arms_[0].joint_count, arms_[1].joint_count);
        const size_t total = arms_[0].joint_count + arms_[1].joint_count;
        Eigen::VectorXd q_all(static_cast<Eigen::Index>(total));
        const auto& pos_if = ctrl_interfaces_.joint_position_state_interface_;
        for (size_t i = 0; i < total; ++i)
        {
            const double fallback = i < hold_positions_.size() ? hold_positions_[i] : 0.0;
            if (measured && i < pos_if.size())
                q_all(static_cast<Eigen::Index>(i)) =
                    pos_if[i].get().get_optional().value_or(fallback);
            else
                q_all(static_cast<Eigen::Index>(i)) = fallback;
        }
        if (arms_[0].joint_count > 0)
            state.leftArmJoints = q_all.head(static_cast<Eigen::Index>(arms_[0].joint_count));
        if (arms_[1].joint_count > 0)
            state.rightArmJoints = q_all.tail(static_cast<Eigen::Index>(arms_[1].joint_count));
        return state;
    }

    bool StateCompliance::computeToolGravityWrenchInSensorFrame(
        int side_index, Eigen::Matrix<double, 6, 1>& wrench_sensor) const
    {
        wrench_sensor.setZero();
        if (!tf_buffer_ || side_index < 0 || side_index > 1) return false;

        const auto& dyn = arms_[side_index].dyn_param;
        if (dyn.size() < 4 || dyn[0] < 1e-6) return false;

        const double mass = dyn[0];
        const Eigen::Vector3d com_tcp(dyn[1] * 1e-3, dyn[2] * 1e-3, dyn[3] * 1e-3);

        geometry_msgs::msg::TransformStamped tf_sensor_from_tcp;
        geometry_msgs::msg::TransformStamped tf_world_from_sensor;
        try
        {
            tf_sensor_from_tcp =
                tf_buffer_->lookupTransform(kFtFrame[side_index], kTcpFrame[side_index],
                                            tf2::TimePointZero);
            try
            {
                tf_world_from_sensor =
                    tf_buffer_->lookupTransform(gravity_frame_, kFtFrame[side_index],
                                                tf2::TimePointZero);
            }
            catch (const tf2::TransformException&)
            {
                tf_world_from_sensor =
                    tf_buffer_->lookupTransform("base_link", kFtFrame[side_index],
                                                tf2::TimePointZero);
            }
        }
        catch (const tf2::TransformException&)
        {
            return false;
        }

        const auto& q_st = tf_sensor_from_tcp.transform.rotation;
        Eigen::Matrix3d R_st =
            Eigen::Quaterniond(q_st.w, q_st.x, q_st.y, q_st.z).toRotationMatrix();
        Eigen::Vector3d t_st(tf_sensor_from_tcp.transform.translation.x,
                             tf_sensor_from_tcp.transform.translation.y,
                             tf_sensor_from_tcp.transform.translation.z);
        const Eigen::Vector3d com_s = R_st * com_tcp + t_st;

        const auto& q_ws = tf_world_from_sensor.transform.rotation;
        Eigen::Matrix3d R_ws =
            Eigen::Quaterniond(q_ws.w, q_ws.x, q_ws.y, q_ws.z).toRotationMatrix();
        const Eigen::Vector3d F_world(0.0, 0.0, -mass * gravity_accel_);
        const Eigen::Vector3d F_s = R_ws.transpose() * F_world;
        const Eigen::Vector3d tau_s = com_s.cross(F_s);

        wrench_sensor << F_s, tau_s;
        return true;
    }

    Eigen::Matrix<double, 6, 1>
    StateCompliance::wrenchToBase(int side_index,
                                   const std::array<double, 6>& wrench_ee) const
    {
        Eigen::Matrix<double, 6, 1> w_base;
        for (int i = 0; i < 6; ++i) w_base(i) = wrench_ee[i];
        if (side_index < 0 || side_index > 1) return w_base;

        if (tf_buffer_)
        {
            try
            {
                geometry_msgs::msg::TransformStamped tf;
                try
                {
                    tf = tf_buffer_->lookupTransform(
                        gravity_frame_, kFtFrame[side_index], tf2::TimePointZero);
                }
                catch (const tf2::TransformException&)
                {
                    tf = tf_buffer_->lookupTransform(
                        "base_link", kFtFrame[side_index], tf2::TimePointZero);
                }
                const auto& q = tf.transform.rotation;
                Eigen::Matrix3d R =
                    Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
                Eigen::Vector3d f(wrench_ee[0], wrench_ee[1], wrench_ee[2]);
                Eigen::Vector3d t(wrench_ee[3], wrench_ee[4], wrench_ee[5]);
                w_base << (R * f), (R * t);
                return w_base;
            }
            catch (const tf2::TransformException&) {}
        }

        if (!kinematics_) return w_base;

        try
        {
            auto pose = kinematics_->computeFramePose(
                makeRobotState(/*measured=*/true),
                side_index == 0 ? "left_eef" : "right_eef");
            Eigen::Vector3d f_ee(wrench_ee[0], wrench_ee[1], wrench_ee[2]);
            Eigen::Vector3d t_ee(wrench_ee[3], wrench_ee[4], wrench_ee[5]);
            w_base << (pose.rotationMatrix * f_ee), (pose.rotationMatrix * t_ee);
        }
        catch (...) {}

        return w_base;
    }

    bool StateCompliance::stampedPoseToBase(
        const geometry_msgs::msg::PoseStamped& msg,
        geometry_msgs::msg::Pose& pose_base) const
    {
        if (!tf_buffer_) return false;

        const std::string target_frame = teleop_base_frame_.empty()
                                             ? "base_link" : teleop_base_frame_;
        if (msg.header.frame_id == target_frame)
        {
            pose_base = msg.pose;
            return true;
        }

        geometry_msgs::msg::TransformStamped tf;
        try
        {
            tf = tf_buffer_->lookupTransform(
                target_frame, msg.header.frame_id, msg.header.stamp,
                rclcpp::Duration::from_seconds(0.1));
        }
        catch (const tf2::TransformException&)
        {
            return false;
        }

        tf2::doTransform(msg.pose, pose_base, tf);
        return true;
    }

    bool StateCompliance::updateTargetFromPose(
        bool is_left, const geometry_msgs::msg::Pose& pose_base)
    {
        if (!kinematicsAvailable()) return false;

        EndEffectorPose tgt;
        tgt.position << pose_base.position.x, pose_base.position.y, pose_base.position.z;
        tgt.setQuaternion(Eigen::Quaterniond(pose_base.orientation.w,
                                              pose_base.orientation.x,
                                              pose_base.orientation.y,
                                              pose_base.orientation.z).normalized());

        ArmSide& a = arm(is_left);
        a.target = tgt;
        a.target_valid = true;
        return true;
    }
} // namespace arms_controller_common
