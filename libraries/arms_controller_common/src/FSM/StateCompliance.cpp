//
// Common StateCompliance — first-order hybrid force/position control
//
#include "arms_controller_common/FSM/StateCompliance.h"

#include <algorithm>
#include <cmath>
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

        // FT telemetry belongs to the controller lifetime, not to one FSM state.
        // Keep this subscription/publisher alive so wrench_filtered also runs in
        // HOLD/HOME/MOVEJ/OCS2.
        updateParam();
        setupWrenchSubscriptions();
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

        // 读取（缺失时用默认值声明）：yaml 未提供的参数也会出现在节点上，
        // 保证 ros2 param set 可在线调整（未声明的参数 set 会被拒绝）。
        auto get_double = [this](const std::string& n, double d) -> double {
            if (!node_->has_parameter(n))
                node_->declare_parameter(n, rclcpp::ParameterValue(d));
            try { return node_->get_parameter(n).get_value<double>(); }
            catch (...) { return d; }
        };
        auto get_bool = [this](const std::string& n, bool b) -> bool {
            if (!node_->has_parameter(n))
                node_->declare_parameter(n, rclcpp::ParameterValue(b));
            try { return node_->get_parameter(n).as_bool(); }
            catch (...) { return b; }
        };
        auto get_array = [this](const std::string& n, std::vector<double>& v) {
            if (!node_->has_parameter(n))
                node_->declare_parameter(n, rclcpp::ParameterValue(v));
            try {
                const auto p = node_->get_parameter(n).as_double_array();
                if (p.size() == v.size()) v = p;
            } catch (...) {}
        };

        get_array("compliance_task_selection", task_selection_);
        get_array("compliance_hybrid_pos_stiffness", hybrid_pos_stiffness_);
        get_array("compliance_hybrid_pos_damping", hybrid_pos_damping_);
        hybrid_pos_vel_damping_ = std::max(0.0, get_double("compliance_hybrid_pos_vel_damping", hybrid_pos_vel_damping_));
        hybrid_pos_accel_ramp_ = std::max(0.0, get_double("compliance_hybrid_pos_accel_ramp", hybrid_pos_accel_ramp_));
        hybrid_pos_jerk_tau_   = std::max(0.0, get_double("compliance_hybrid_pos_jerk_tau", hybrid_pos_jerk_tau_));
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
        hybrid_joint_limit_avoidance_gain_ = std::max(
            0.0, get_double("compliance_hybrid_joint_limit_avoidance_gain",
                            hybrid_joint_limit_avoidance_gain_));
        wrist_coupling_max_       = get_double("compliance_wrist_coupling_max", wrist_coupling_max_);
        hybrid_dls_lambda_         = get_double("compliance_hybrid_dls_lambda", hybrid_dls_lambda_);
        hybrid_qp_lambda_          = std::max(1e-3, get_double("compliance_hybrid_qp_lambda", hybrid_qp_lambda_));
        hybrid_lin_task_weight_    = std::max(1.0, get_double("compliance_hybrid_lin_task_weight", hybrid_lin_task_weight_));
        try {
            hybrid_inverse_method_ = node_->get_parameter("compliance_hybrid_inverse_method").as_string();
        } catch (...) {}
        if (hybrid_inverse_method_ != "QP") hybrid_inverse_method_ = "DLS";

        wrench_lpf_alpha_ = std::clamp(
            get_double("compliance_wrench_lpf_alpha", wrench_lpf_alpha_), 0.01, 1.0);

        zero_cal_duration_  = get_double("compliance_zero_cal_duration", zero_cal_duration_);
        zero_cal_settle_    = get_double("compliance_zero_cal_settle", zero_cal_settle_);
        zero_cal_still_vel_ = get_double("compliance_zero_cal_still_vel", zero_cal_still_vel_);
        diag_log_ = get_bool("compliance_diag_log", diag_log_);
        diag_log_period_ = std::max(0.1, get_double("compliance_diag_log_period", diag_log_period_));

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

        // 力控→位置控边沿检测（面板取消勾选柔顺轴）：置 pending_retarget_，
        // 由 run() 用当前实测位姿重捕获 target，避免位置轴全速拉回旧目标。
        if (task_selection_prev_.size() == task_selection_.size())
        {
            for (size_t i = 0; i < task_selection_.size(); ++i)
            {
                if (task_selection_prev_[i] > 0.5 && task_selection_[i] <= 0.5)
                {
                    pending_retarget_ = true;
                    break;
                }
            }
        }
        task_selection_prev_ = task_selection_;
    }

    void StateCompliance::setupWrenchSubscriptions()
    {
        if (!node_) return;

        auto qos = rclcpp::SensorDataQoS();
        for (int s = 0; s < 2; ++s)
        {
            auto& a = arms_[s];
            a.wrench_sub.reset();
            a.filtered_wrench_pub.reset();
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
                    arms_[s].wrench_frame_id = msg->header.frame_id;
                });
            a.filtered_wrench_pub = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
                kFilteredWrenchTopic[s], rclcpp::SensorDataQoS());
        }
    }

    StateCompliance::WrenchSnapshot
    StateCompliance::sampleAndPublishWrenches(const rclcpp::Time& time)
    {
        WrenchSnapshot snapshot;
        if (!node_) return snapshot;

        std::array<std::array<double, 6>, 2> raw{};
        std::array<rclcpp::Time, 2> ft_stamp{
            rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Time(0, 0, RCL_ROS_TIME)};
        {
            const rclcpp::Time now = node_->now();
            std::lock_guard<std::mutex> lk(wrench_mutex_);
            for (int s = 0; s < 2; ++s)
            {
                auto& a = arms_[s];
                if (a.ft_active)
                {
                    try
                    {
                        if ((now - a.ft_stamp).seconds() > ft_timeout_sec_)
                        {
                            a.ft_active = false;
                            RCLCPP_WARN_THROTTLE(
                                node_->get_logger(), *node_->get_clock(), 2000,
                                "FT stale (>%.0f ms)", ft_timeout_sec_ * 1000.0);
                        }
                    }
                    catch (...) { a.ft_active = false; }
                }

                raw[s] = a.wrench;
                snapshot.active[s] = a.ft_active;
                ft_stamp[s] = a.ft_stamp;
                snapshot.frame_id[s] = a.wrench_frame_id;
            }
        }

        for (int s = 0; s < 2; ++s)
        {
            snapshot.net[s] = raw[s];
            if (!snapshot.active[s]) continue;

            const std::string sensor_frame = snapshot.frame_id[s].empty()
                                                 ? kFtFrame[s] : snapshot.frame_id[s];
            Eigen::Matrix<double, 6, 1> gravity_wrench =
                Eigen::Matrix<double, 6, 1>::Zero();
            if (computeToolGravityWrenchInSensorFrame(s, sensor_frame, gravity_wrench))
            {
                for (int i = 0; i < 6; ++i)
                    snapshot.net[s][i] -= gravity_wrench(i);
            }

            if (!arms_[s].filtered_wrench_pub) continue;
            std::array<double, 6> tared = snapshot.net[s];
            for (int i = 0; i < 6; ++i) tared[i] -= arms_[s].wrench_bias[i];

            geometry_msgs::msg::WrenchStamped msg;
            msg.header.stamp = ft_stamp[s].nanoseconds() == 0 ? time : ft_stamp[s];
            msg.header.frame_id = sensor_frame;
            msg.wrench.force.x = tared[0];
            msg.wrench.force.y = tared[1];
            msg.wrench.force.z = tared[2];
            msg.wrench.torque.x = tared[3];
            msg.wrench.torque.y = tared[4];
            msg.wrench.torque.z = tared[5];
            arms_[s].filtered_wrench_pub->publish(msg);
        }

        return snapshot;
    }

    void StateCompliance::publishWrenchTelemetry(const rclcpp::Time& time)
    {
        (void)sampleAndPublishWrenches(time);
    }

    void StateCompliance::setupZeroWrenchService()
    {
        if (!node_) return;

        zero_wrench_service_ = node_->create_service<std_srvs::srv::Trigger>(
            "compliance_zero_wrench",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /* request */,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
            {
                zero_cal_requested_.store(true, std::memory_order_release);
                response->success = true;
                response->message =
                    "Wrench zero calibration requested; keep the arms still and unloaded";
            });
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

        // Clear the previous COMPLIANCE session before capturing the new
        // initial target.  In particular, resetControlState() invalidates the
        // target, so it must run before the FK result is stored below.
        for (auto& a : arms_) a.resetControlState();

        const size_t nj = ctrl_interfaces_.joint_position_state_interface_.size();
        hold_positions_.resize(nj);
        for (size_t i = 0; i < nj; ++i)
        {
            const double measured = ctrl_interfaces_.joint_position_state_interface_[i]
                                        .get().get_optional().value_or(0.0);
            hold_positions_[i] = ctrl_interfaces_.last_sent_joint_positions_.size() > i
                ? ctrl_interfaces_.last_sent_joint_positions_[i] : measured;
        }

        // Match StateOCS2::resetMpc(): the target on state entry is the FK of
        // the last commanded joint state.  Store it as a real target (instead
        // of relying on the target_valid=false publishing fallback), so
        // left/right_current_target start at the pose where control takes over.
        if (kinematicsAvailable())
        {
            const RobotState state = makeRobotState(/*measured=*/false);
            for (int s = 0; s < 2; ++s)
            {
                if (arms_[s].joint_count > 0)
                {
                    const EndEffectorPose initial_pose =
                        kinematics_->computeSingleEndEffectorPose(state, kArmName[s]);
                    arms_[s].cmd_pose = initial_pose;
                    arms_[s].target = initial_pose;
                    arms_[s].target_valid = true;
                }
            }
        }

        zero_cal_pending_ = true;
        zero_cal_running_ = false;
        zero_cal_done_    = false;
        zero_cal_requested_.store(false, std::memory_order_relaxed);
        zero_cal_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        measured_joint_vel_max_ = 0.0;
        q_meas_prev_all_.resize(0);

        if (kinematics_)
        {
            kinematics_->getJointLimits("left",  arms_[0].joint_lower, arms_[0].joint_upper);
            kinematics_->getJointLimits("right", arms_[1].joint_lower, arms_[1].joint_upper);
        }

        setupTeleopSubscriptions();
        setupZeroWrenchService();

        if (node_)
        {
            // build 时间戳随每次编译自动更新：入口一眼核对机器人上跑的是
            // 否为最新二进制（此前多次误测旧 .so，结论全部作废）。
            RCLCPP_INFO(node_->get_logger(),
                        "COMPLIANCE (first-order hybrid): entered. "
                        "build=%s %s. "
                        "sel=[%.0f %.0f %.0f %.0f %.0f %.0f], "
                        "Fd=[%.1f %.1f %.1f] N, zero_cal=%.1f s, "
                        "kin=%s (L=%zu R=%zu), ik=%s dls_lambda=%.3f qp_lambda=%.3f "
                        "joint_vmax=%.3f margin=%.3f joint_avoid=%.3f wrist_limit=%.4f "
                        "lin_weight=%.2f "
                        "pos_ramp=%.3f jerk_tau=%.3f vel_damping=%.3f",
                        __DATE__, __TIME__,
                        task_selection_[0], task_selection_[1], task_selection_[2],
                        task_selection_[3], task_selection_[4], task_selection_[5],
                        force_setpoint_.empty() ? 0.0 : force_setpoint_[0],
                        force_setpoint_.size() > 1 ? force_setpoint_[1] : 0.0,
                        force_setpoint_.size() > 2 ? force_setpoint_[2] : 0.0,
                        zero_cal_duration_,
                        kinematicsAvailable() ? "ok" : "MISSING",
                        arms_[0].joint_count, arms_[1].joint_count,
                        hybrid_inverse_method_.c_str(), hybrid_dls_lambda_, hybrid_qp_lambda_,
                        hybrid_joint_vmax_, hybrid_joint_limit_margin_,
                        hybrid_joint_limit_avoidance_gain_, wrist_coupling_max_,
                        hybrid_lin_task_weight_,
                        hybrid_pos_accel_ramp_, hybrid_pos_jerk_tau_, hybrid_pos_vel_damping_);

            for (int s = 0; s < 2; ++s)
            {
                if (arms_[s].dyn_param.size() < 4 || arms_[s].dyn_param[0] < 1e-6)
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "COMPLIANCE: %s tool gravity compensation NOT configured "
                                "(left_dyn_param/right_dyn_param). Zero-cal absorbs gravity at the "
                                "current pose only — expect force errors when the arm moves.",
                                kArmName[s]);
                }
            }
        }
    }

    void StateCompliance::exit()
    {
        for (auto& a : arms_) a.resetComplianceIo();
        force_status_pub_.reset();
        zero_wrench_service_.reset();
    }

    void StateCompliance::run(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        updateParam();
        const double dt = period.seconds();

        if (zero_cal_requested_.exchange(false, std::memory_order_acq_rel))
        {
            for (auto& a : arms_)
            {
                a.zero_cal_sum.fill(0.0);
                a.zero_cal_samples = 0;
                a.force_integral.setZero();
                a.wrench_filt.setZero();
                a.v_des_filt.setZero();
            }
            zero_cal_pending_ = true;
            zero_cal_running_ = false;
            zero_cal_done_ = false;
            zero_cal_start_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
            measured_joint_vel_max_ = 0.0;
            q_meas_prev_all_.resize(0);
            if (node_)
            {
                RCLCPP_INFO(node_->get_logger(),
                            "COMPLIANCE wrench zero requested; keep arms still and unloaded");
            }
        }

        const WrenchSnapshot wrench_snapshot = sampleAndPublishWrenches(time);
        const auto& net = wrench_snapshot.net;
        const auto& ft_active = wrench_snapshot.active;
        const auto& wrench_frame_id = wrench_snapshot.frame_id;

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

        // ── Zero-cal (FT tare; does NOT block position teleop) ──
        // FT late arrival / previous zero-cal failure: re-arm automatically once
        // a sensor is available and no calibration was ever completed.
        if (!zero_cal_pending_ && !zero_cal_running_ && !zero_cal_done_ &&
            (ft_active[0] || ft_active[1]))
        {
            zero_cal_pending_ = true;
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "COMPLIANCE: FT available but zero-cal not completed — "
                                 "restarting zero-cal, keep arm still");
        }
        if (zero_cal_pending_ && (ft_active[0] || ft_active[1]))
        {
            zero_cal_pending_ = false;
            zero_cal_running_ = true;
            zero_cal_start_   = time;
            RCLCPP_INFO(node_->get_logger(),
                        "COMPLIANCE zero_cal started (%.2f s). Keep arm still.", zero_cal_duration_);
        }
        if (zero_cal_pending_ && !ft_active[0] && !ft_active[1])
        {
            if (zero_cal_start_.nanoseconds() == 0)
                zero_cal_start_ = time;
            else if ((time - zero_cal_start_).seconds() > 2.0)
            {
                zero_cal_pending_ = false;
                // NOT zero_cal_done_: no calibration was performed; it will be
                // re-armed automatically above once FT becomes available.
                RCLCPP_WARN(node_->get_logger(),
                            "COMPLIANCE: no FT within 2 s — position teleop only; "
                            "zero-cal starts automatically once FT is available.");
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
                    if (!ft_active[s]) continue;
                    for (int i = 0; i < 6; ++i)
                        arms_[s].zero_cal_sum[i] += net[s][i];
                    ++arms_[s].zero_cal_samples;
                }
            }
            if (elapsed >= zero_cal_duration_)
            {
                bool active_side_needs_samples = false;
                for (int s = 0; s < 2; ++s)
                    active_side_needs_samples |= ft_active[s] && arms_[s].zero_cal_samples < 10;
                if (active_side_needs_samples && elapsed < 3.0 * zero_cal_duration_)
                {
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                         "COMPLIANCE zero_cal: too few still samples (L=%ld R=%ld) — extending...",
                                         arms_[0].zero_cal_samples, arms_[1].zero_cal_samples);
                }
                else
                {
                    bool any_failed = false;
                    bool any_calibrated = false;
                    for (int s = 0; s < 2; ++s)
                    {
                        if (arms_[s].zero_cal_samples <= 0)
                        {
                            // Sensor present but no still samples: calibration
                            // failed — keep force axes OFF (no untared data in control).
                            if (ft_active[s])
                            {
                                any_failed = true;
                                RCLCPP_ERROR(node_->get_logger(),
                                             "COMPLIANCE zero_cal FAILED on %s: no still samples "
                                             "(arm moving or too noisy) — force axes stay inactive; retrying",
                                             kArmName[s]);
                            }
                            continue;
                        }
                        for (int i = 0; i < 6; ++i)
                            arms_[s].wrench_bias[i] = arms_[s].zero_cal_sum[i] /
                                static_cast<double>(arms_[s].zero_cal_samples);
                        any_calibrated = true;
                    }
                    zero_cal_running_ = false;
                    zero_cal_done_    = any_calibrated && !any_failed;
                    if (any_failed)
                    {
                        RCLCPP_WARN(node_->get_logger(),
                                    "COMPLIANCE zero_cal incomplete — will retry automatically");
                    }
                    else
                    {
                        RCLCPP_INFO(node_->get_logger(),
                                    "COMPLIANCE zero_cal done (%ld samples). "
                                    "Bias L=[%.3f %.3f %.3f] R=[%.3f %.3f %.3f] N.",
                                    arms_[0].zero_cal_samples,
                                    arms_[0].wrench_bias[0], arms_[0].wrench_bias[1], arms_[0].wrench_bias[2],
                                    arms_[1].wrench_bias[0], arms_[1].wrench_bias[1], arms_[1].wrench_bias[2]);
                    }
                }
            }
        }

        // ── Tare → contact wrench in base ──
        Eigen::Matrix<double, 6, 1> contact_wrench[2];
        for (int s = 0; s < 2; ++s)
        {
            std::array<double, 6> tared = net[s];
            for (int i = 0; i < 6; ++i) tared[i] -= arms_[s].wrench_bias[i];

            contact_wrench[s] = ft_active[s]
                ? wrenchToBase(s,
                               wrench_frame_id[s].empty() ? kFtFrame[s] : wrench_frame_id[s],
                               tared)
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

        // ── 柔顺轴关闭：就地重捕获 target ──
        // 拖动期间力控轴不更新 target；取消勾选后该轴变为位置轴，若沿用旧 target
        // 会以 K·Δx 全速回拉（危险）。这里把 target 重置为当前实测位姿并清位置
        // 积分，使位置轴输出为零（关闭后原地不动）。
        if (pending_retarget_)
        {
            pending_retarget_ = false;
            if (kinematicsAvailable())
            {
                const RobotState state_actual = makeRobotState(/*measured=*/true);
                for (int s = 0; s < 2; ++s)
                {
                    if (arms_[s].joint_count == 0) continue;
                    arms_[s].target =
                        kinematics_->computeSingleEndEffectorPose(state_actual, kArmName[s]);
                    arms_[s].target_valid = true;
                    arms_[s].pos_integral.setZero();
                }
                if (node_)
                {
                    RCLCPP_INFO(node_->get_logger(),
                                "COMPLIANCE: force axis disabled; target re-captured to current pose (no pull-back)");
                }
            }
        }

        // ── Hybrid control ──
        if (kinematicsAvailable())
        {
            stepHybridControl(true,  dt, contact_wrench[0], ft_active[0]);
            stepHybridControl(false, dt, contact_wrench[1], ft_active[1]);
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
                msg.left_ft_active = ft_active[0];
                msg.right_ft_active = ft_active[1];
                msg.zero_cal_done = zero_cal_done_;
                msg.force_feedback_sign = force_feedback_sign_;
                force_status_pub_->publish(msg);
                last_force_status_pub_ = time;
            }
        }
    }

    void StateCompliance::stepHybridControl(
        bool is_left, double dt,
        const Eigen::Matrix<double, 6, 1>& contact_wrench,
        bool ft_active)
    {
        if (dt <= 1e-6 || !kinematicsAvailable()) return;

        ArmSide& a = arm(is_left);
        const size_t offset = is_left ? 0 : arms_[0].joint_count;
        const size_t nq = a.joint_count;
        if (nq == 0 || offset + nq > hold_positions_.size()) return;

        // 诊断快照：err/v/f/qdot/限位事件在各自作用域内填，函数尾统一输出。
        const bool diag_due = diag_log_ && std::chrono::steady_clock::now() - last_diag_log_[is_left ? 0 : 1]
            >= std::chrono::duration<double>(diag_log_period_);
        std::array<double, 6> diag_err{};   // [x y z rx ry rz]，力控轴无位置追踪语义
        std::string diag_jlim;
        std::string diag_bounds;
        std::string diag_saturated;
        std::array<double, 6> diag_f_err{}, diag_f_eff{};

        auto S = [&](int i) -> double {
            return i < static_cast<int>(task_selection_.size()) ? task_selection_[i] : 0.0;
        };

        // Actual FK/Jacobian (closes loop around real robot)
        RobotState state_actual = makeRobotState(/*measured=*/true);

        const EndEffectorPose cur =
            kinematics_->computeSingleEndEffectorPose(state_actual, kArmName[is_left ? 0 : 1]);
        const EndEffectorPose tgt = a.target_valid ? a.target : cur;
        // Orientation error must close on the MEASURED pose (like translation).
        // Using the commanded FK hides the position-loop lag from the rotation
        // loop, so a rotating target is tracked only with pure open-loop lag.
        const Eigen::Matrix3d R_cur_for_rot = cur.rotationMatrix;

        const Eigen::MatrixXd J = kinematics_->computeJacobian(
            state_actual, kArmName[is_left ? 0 : 1]);
        if (J.rows() < 6 || J.cols() != static_cast<Eigen::Index>(nq) || !J.allFinite())
            return;

        // 速度阻尼反馈：上一周期指令速度经 J 映射的末端速度。用指令而非
        // 实测差分——底层位置反馈的延迟/刷新节拍/量化会让差分产生与速度
        // 成正比的纹波（实测反馈会放大高频抖动）；指令是发给底层的量，
        // 无噪声，且底层以 ~34ms 跟随指令，指令侧刹车同样在误差过零前生效。
        Eigen::Matrix<double, 6, 1> v_ee_fb = Eigen::Matrix<double, 6, 1>::Zero();
        if (a.qdot_prev.size() == static_cast<Eigen::Index>(nq) &&
            a.qdot_prev.allFinite())
        {
            v_ee_fb = J * a.qdot_prev;
        }

        // wrench EMA：α 按 500 Hz 标定，按实际 dt 折算保持带宽恒定。
        const double alpha_wrench = std::clamp(wrench_lpf_alpha_ * dt / 0.002, 0.0, 1.0);
        a.wrench_filt = alpha_wrench * contact_wrench +
                        (1.0 - alpha_wrench) * a.wrench_filt;

        Eigen::Matrix<double, 6, 1> v_des = Eigen::Matrix<double, 6, 1>::Zero();

        // Position axes: v = K/(1+D)·Δx + I
        {
            const double ki_dt = hybrid_pos_ki_ * dt;
            const Eigen::Vector3d dp = tgt.position - cur.position;
            if (diag_due) { diag_err[0]=dp(0); diag_err[1]=dp(1); diag_err[2]=dp(2); }
            for (int i = 0; i < 3; ++i) {
                if (S(i) > 0.5) continue;
                const double K = i < static_cast<int>(hybrid_pos_stiffness_.size())
                                     ? hybrid_pos_stiffness_[i] : 20.0;
                const double D = i < static_cast<int>(hybrid_pos_damping_.size())
                                     ? std::max(0.0, hybrid_pos_damping_[i]) : 0.0;
                a.pos_integral(i) += ki_dt * dp(i);
                a.pos_integral(i) = std::clamp(
                    a.pos_integral(i), -hybrid_pos_ki_max_, hybrid_pos_ki_max_);
                // 速度阻尼（随速度渐隐）：只在 |v| 超过 vmax/3 时全额生效——
                // 高速时提前刹车，抑制纯 P 律 + 底层延迟的饱和极限环；低速
                // 接近目标时线性淡出，避免把指令速度提前压塌（离目标几毫米
                // 刹停再爬行 → 末端一次抖动），末段交给纯 P 指数收敛。
                // 符号守卫：阻尼只刹车不助推——近奇异时解可能给出与指令反号
                // 的速度反馈，若照常反馈会形成正反馈（指令越推越大 → 振荡）。
                // 且阻尼被限制在 |v_p| 内，输出不会越过零点反向。
                const double vmax_i = i < static_cast<int>(hybrid_cart_vmax_.size())
                                          ? hybrid_cart_vmax_[i] : 0.05;
                const double fade = std::clamp(
                    std::abs(v_ee_fb(i)) / std::max(vmax_i / 3.0, 1e-3), 0.0, 1.0);
                const double v_p = (K / (1.0 + D)) * dp(i) + a.pos_integral(i);
                const double damp = hybrid_pos_vel_damping_ * fade * v_ee_fb(i);
                v_des(i) = (v_p * damp > 0.0)
                    ? v_p - std::clamp(damp, -std::abs(v_p), std::abs(v_p))
                    : v_p;
            }

            // 姿态误差用四元数 log（AngleAxis）计算：acos(trace) 在收敛附近
            // 导数无界（放大测角噪声 → 跟踪抖动），skew/(2·sin) 在 180° 附近
            // 数值发散；w<0 翻号保证走最短旋转路径，消除 ±180° 跳变。
            Eigen::Quaterniond q_err(tgt.rotationMatrix * R_cur_for_rot.transpose());
            if (q_err.coeffs().w() < 0.0) q_err.coeffs() *= -1.0;
            const Eigen::AngleAxisd aa_err(q_err);
            Eigen::Vector3d rotvec = Eigen::Vector3d::Zero();
            if (aa_err.angle() > 1e-9) rotvec = aa_err.angle() * aa_err.axis();
            if (diag_due) { diag_err[3]=rotvec(0); diag_err[4]=rotvec(1); diag_err[5]=rotvec(2); }
            for (int i = 0; i < 3; ++i) {
                if (S(i+3) > 0.5) continue;
                const double K = i+3 < static_cast<int>(hybrid_pos_stiffness_.size())
                                     ? hybrid_pos_stiffness_[i+3] : 10.0;
                const double D = i+3 < static_cast<int>(hybrid_pos_damping_.size())
                                     ? std::max(0.0, hybrid_pos_damping_[i+3]) : 0.0;
                a.pos_integral(i+3) += ki_dt * rotvec(i);
                a.pos_integral(i+3) = std::clamp(
                    a.pos_integral(i+3), -hybrid_pos_ki_max_, hybrid_pos_ki_max_);
                // 速度阻尼随速度渐隐（平移轴 vmax/3；旋转轴 vmax/6）。旋转
                // vmax 定得宽（0.6），实际拖动旋转多在 0.05~0.15 rad/s，若按
                // vmax/3=0.2 取转折点则长期处于渐隐区，反转振荡压不住（旋转
                // 残余抖动）；vmax/6=0.1 让常见旋转速度拿到全额阻尼。
                // 符号守卫同平移轴：阻尼只刹车不助推（防近奇异反号正反馈），
                // 且限制在 |v_p| 内保证输出不越零反向。
                const double vmax_i = i+3 < static_cast<int>(hybrid_cart_vmax_.size())
                                          ? hybrid_cart_vmax_[i+3] : 0.3;
                const double fade = std::clamp(
                    std::abs(v_ee_fb(i+3)) / std::max(vmax_i / 6.0, 1e-3), 0.0, 1.0);
                const double v_p = (K / (1.0 + D)) * rotvec(i) + a.pos_integral(i+3);
                const double damp = hybrid_pos_vel_damping_ * fade * v_ee_fb(i+3);
                v_des(i+3) = (v_p * damp > 0.0)
                    ? v_p - std::clamp(damp, -std::abs(v_p), std::abs(v_p))
                    : v_p;
            }
        }

        // Force axes: v = (F_err + I) / D
        {
            const bool ft_ok = zero_cal_done_ && ft_active;
            for (int i = 0; i < 6; ++i) {
                if (S(i) < 0.5) continue;
                if (!ft_ok) { v_des(i) = 0.0; continue; }

                const double F_des = i < static_cast<int>(force_setpoint_.size())
                                         ? force_setpoint_[i] : 0.0;
                const double f_err = F_des - force_feedback_sign_ * a.wrench_filt(i);
                const double D = i < static_cast<int>(hybrid_force_damping_.size())
                                     ? hybrid_force_damping_[i] : (i < 3 ? 2000.0 : 100.0);

                // Anti-drift deadzone: a static residual (tool-gravity model
                // error, sensor drift) must not produce continuous creep. With
                // v=(f_err+I)/D the axis drifts at (f_err+I)/D forever and can
                // never balance (力控轴漂移). Zero the output inside the
                // deadband; outside, subtract the band so the response stays
                // continuous. Increase compliance_hybrid_force_deadband if the
                // gravity-compensation residual exceeds it.
                const double db = hybrid_force_deadband_;
                const double f_eff = std::abs(f_err) > db
                    ? f_err - std::copysign(db, f_err) : 0.0;
                if (diag_due) { diag_f_err[i] = f_err; diag_f_eff[i] = f_eff; }

                // Leakage: force_integral *= (1 - leak·dt). Prevents wind-up
                // accumulation during sustained drag (main oscillation driver).
                // Applied also inside the deadband so a stale integral cannot
                // cause a velocity jump when the error re-enters the deadband.
                a.force_integral(i) *= std::max(0.0, 1.0 - hybrid_force_ki_leak_ * dt);
                // 实现度冻结/反算：上周期请求（v_des_filt）明显未被 QP 实现
                // （奇异/关节限位卡住）时不再积累——积分不感知可实现性会在
                // 卡住期间持续充能，恢复后猛冲。停滞时额外以 2×leak 退积分。
                const double req_prev = std::abs(a.v_des_filt(i));
                const double ach_prev = std::abs(v_ee_fb(i));
                const bool realized = req_prev < 1e-4 || ach_prev > 0.3 * req_prev;
                if (!realized)
                {
                    a.force_integral(i) *= std::max(
                        0.0, 1.0 - 2.0 * hybrid_force_ki_leak_ * dt);
                }
                else if (std::abs(f_err) >= db)
                {
                    a.force_integral(i) += hybrid_force_ki_ * dt * f_eff;
                }
                a.force_integral(i) = std::clamp(
                    a.force_integral(i), -hybrid_force_ki_max_, hybrid_force_ki_max_);
                v_des(i) = (f_eff + a.force_integral(i)) / std::max(D, 1e-3);
            }

        }

        // Force-axis admittance output damping: low-pass v_des to add virtual
        // inertia. This is the primary suppressor of the ~1-3 Hz drag oscillation
        // (pure proportional admittance + joint-tracking lag → positive feedback).
        // α 按 500 Hz 标定，实际控制周期偏离时按 dt 比例折算（EMA 时间常数
        // τ≈(1−α)·dt/α 随 dt 保持恒定），避免带宽随周期漂移。
        for (int i = 0; i < 6; ++i)
        {
            if (S(i) < 0.5) { a.v_des_filt(i) = 0.0; continue; }
            const double alpha_eff = std::clamp(
                force_vel_lpf_alpha_ * dt / 0.002, 0.0, 1.0);
            a.v_des_filt(i) = alpha_eff * v_des(i) +
                              (1.0 - alpha_eff) * a.v_des_filt(i);
            v_des(i) = a.v_des_filt(i);
        }

        // 位控平移轴、旋转轴分别共享同一个速度缩放量。相比逐轴 clamp，
        // 这会保持同组各分量比例，因此固定目标下不会因限速改变直线方向。
        auto limit_position_group = [&](int first, int last)
        {
            double normalized_norm_sq = 0.0;
            for (int i = first; i < last; ++i)
            {
                if (S(i) >= 0.5) continue;
                const double vmax = i < static_cast<int>(hybrid_cart_vmax_.size())
                    ? std::max(std::abs(hybrid_cart_vmax_[i]), 1e-6)
                    : (i < 3 ? 0.05 : 0.3);
                normalized_norm_sq += std::pow(v_des(i) / vmax, 2);
            }
            const double scale = std::max(1.0, std::sqrt(normalized_norm_sq));
            for (int i = first; i < last; ++i)
                if (S(i) < 0.5) v_des(i) /= scale;
        };
        limit_position_group(0, 3);
        limit_position_group(3, 6);

        // 力控轴仍按各自的导纳速度上限约束，不参与直线方向缩放。
        for (int i = 0; i < 6; ++i)
        {
            if (S(i) < 0.5) continue;
            const double vmax = i < static_cast<int>(hybrid_cart_vmax_.size())
                ? hybrid_cart_vmax_[i] : (i < 3 ? 0.05 : 0.3);
            v_des(i) = std::clamp(v_des(i), -vmax, vmax);
        }

        // 位控轴在线 S 曲线：平移组、旋转组分别共享同一个加减速比例，
        // 再用相同低通系数圆滑加速度阶跃。固定目标方向下，速度、加速度
        // 和低通均保持组内分量比例；力控轴不受影响。
        if (hybrid_pos_accel_ramp_ > 0.0)
        {
            const double lpf_tau = hybrid_pos_jerk_tau_ > 0.0
                ? hybrid_pos_jerk_tau_
                : hybrid_pos_accel_ramp_ / 3.0;
            const double alpha = std::clamp(dt / (lpf_tau + dt), 0.0, 1.0);
            auto ramp_position_group = [&](int first, int last)
            {
                Eigen::Vector3d u_prev = Eigen::Vector3d::Zero();
                Eigen::Vector3d u_des = Eigen::Vector3d::Zero();
                bool have_position_axis = false;
                for (int i = first; i < last; ++i)
                {
                    const int k = i - first;
                    if (S(i) >= 0.5)
                    {
                        a.v_pos_prev(i) = 0.0;
                        a.v_pos_filt(i) = 0.0;
                        continue;
                    }
                    const double vmax = i < static_cast<int>(hybrid_cart_vmax_.size())
                        ? std::max(std::abs(hybrid_cart_vmax_[i]), 1e-6)
                        : (i < 3 ? 0.05 : 0.3);
                    u_prev(k) = a.v_pos_prev(i) / vmax;
                    u_des(k) = v_des(i) / vmax;
                    have_position_axis = true;
                }
                if (!have_position_axis) return;

                Eigen::Vector3d du = u_des - u_prev;
                const bool accelerating = u_prev.dot(du) >= 0.0;
                const double du_max = (accelerating ? 1.0 : 2.0) *
                    dt / hybrid_pos_accel_ramp_;
                const double du_norm = du.norm();
                if (du_norm > du_max && du_norm > 1e-12)
                    du *= du_max / du_norm;
                const Eigen::Vector3d u_slew = u_prev + du;

                for (int i = first; i < last; ++i)
                {
                    if (S(i) >= 0.5) continue;
                    const int k = i - first;
                    const double vmax = i < static_cast<int>(hybrid_cart_vmax_.size())
                        ? std::max(std::abs(hybrid_cart_vmax_[i]), 1e-6)
                        : (i < 3 ? 0.05 : 0.3);
                    const double v_slew = u_slew(k) * vmax;
                    a.v_pos_prev(i) = v_slew;
                    a.v_pos_filt(i) += alpha * (v_slew - a.v_pos_filt(i));
                    v_des(i) = a.v_pos_filt(i);
                }
            };
            ramp_position_group(0, 3);
            ramp_position_group(3, 6);
        }

        // Force-axis displacement soft limit. 请求按软限整形；位移积分改用
        // 上一周期实际达成的任务速度（J·qdot_prev）——按请求积分会在奇异/
        // 限位卡住期间虚耗行程余量，恢复后软限提前触发。
        for (int i = 0; i < 6; ++i) {
            if (S(i) < 0.5) { a.force_disp(i) = 0.0; continue; }
            const double xmax = i < 3 ? hybrid_force_xmax_lin_ : hybrid_force_xmax_ang_;
            const double margin = hybrid_force_xmax_margin_ratio_ * xmax;
            const double proposed = a.force_disp(i) + v_des(i) * dt;
            if (std::abs(proposed) > xmax)
                v_des(i) = (std::copysign(xmax, proposed) - a.force_disp(i)) / dt;
            else if (std::abs(proposed) > xmax - margin)
                v_des(i) *= std::clamp((xmax - std::abs(proposed)) / margin, 0.0, 1.0);
            a.force_disp(i) += v_ee_fb(i) * dt;
            a.force_disp(i) = std::clamp(a.force_disp(i), -xmax, xmax);
        }

        // 不做阈值式可行性滤除：λ 阻尼最小二乘对奇异方向按 σ²/(σ²+λ²)
        // 平滑衰减（无阈值、不连续点，不会冻结也不会抖动）。目标不可
        // 达时解会自然收敛到"当前构型下距离目标最近的可达点"（静止条件
        // Jᵀ·v=0 即距离的临界点），而不是停在半路；速度规划由底层臂插值承担。

        // 同时限制关节速度及下一周期的位置，避免事后截断破坏末端速度方向。
        // 限位采用软减速：除了单步位置约束 (hi−q)/dt，再叠加比例减速
        // k·(hi−q)——靠近限位 ~vmax/k 内速度线性衰减，QP 在求解时就避让，
        // 不再出现“sat 常驻 + 抵达边界后指令与钳制每周期对抗”的顿挫。
        const Eigen::Index n = static_cast<Eigen::Index>(nq);
        const double joint_vmax = std::max(std::abs(hybrid_joint_vmax_), 1e-6);
        constexpr double kApproachRate = 8.0;  // 限位/耦合边界软减速率 [1/s]
        Eigen::VectorXd vmax_up = Eigen::VectorXd::Constant(n, joint_vmax);
        Eigen::VectorXd vmax_dn = vmax_up;
        if (dt > 1e-6)
        {
            for (Eigen::Index i = 0; i < n && i < a.joint_lower.size() && i < a.joint_upper.size(); ++i)
            {
                if (a.joint_lower(i) >= a.joint_upper(i)) continue;
                const double q = hold_positions_[offset + static_cast<size_t>(i)];
                const double lo = a.joint_lower(i) + hybrid_joint_limit_margin_;
                const double hi = a.joint_upper(i) - hybrid_joint_limit_margin_;
                vmax_up(i) = std::min(joint_vmax, std::min(
                    std::max(0.0, (hi - q) / dt),
                    kApproachRate * std::max(0.0, hi - q)));
                vmax_dn(i) = std::min(joint_vmax, std::min(
                    std::max(0.0, (q - lo) / dt),
                    kApproachRate * std::max(0.0, q - lo)));
                if (diag_due && (vmax_up(i) < joint_vmax - 1e-6 ||
                                 vmax_dn(i) < joint_vmax - 1e-6))
                {
                    char item[48];
                    std::snprintf(item, sizeof(item), "j%ld(-%.3f,+%.3f)",
                                  static_cast<long>(i + 1), vmax_dn(i), vmax_up(i));
                    diag_bounds += (diag_bounds.empty() ? "" : " ") + std::string(item);
                }
            }

            // 6/7 轴耦合边界进入速度盒：接近 |q6|+|q7| 上限时，限制两关节
            // “向外”（增大 |q|）方向的速度为 k·(lim−s)。此前 QP 不感知耦合
            // 边界，旋转拖动时持续把解顶进边界（sat=[j6] 常驻），再由事后
            // 投影每周期掰回 ~0.001 rad——手腕处吸附/顿挫的直接来源。盒内
            // 逐关节上限是保守近似（两关节同向外向时和可达 2c），残余越界
            // 仍由 L1 投影兜底。
            if (n >= 2 && wrist_coupling_max_ > hybrid_joint_limit_margin_)
            {
                const Eigen::Index i6 = n - 2, i7 = n - 1;
                if (i6 < a.joint_lower.size() && i7 < a.joint_upper.size())
                {
                    const double q6 = hold_positions_[offset + static_cast<size_t>(i6)];
                    const double q7 = hold_positions_[offset + static_cast<size_t>(i7)];
                    const double s = std::abs(q6) + std::abs(q7);
                    const double lim = wrist_coupling_max_ - hybrid_joint_limit_margin_;
                    if (s > lim - joint_vmax / kApproachRate)
                    {
                        const double c = std::max(0.0, kApproachRate * (lim - s));
                        for (const Eigen::Index i : {i6, i7})
                        {
                            const double q = hold_positions_[offset + static_cast<size_t>(i)];
                            if (q > 0.0) vmax_up(i) = std::min(vmax_up(i), c);
                            else if (q < 0.0) vmax_dn(i) = std::min(vmax_dn(i), c);
                        }
                    }
                }
            }
        }

        Eigen::VectorXd qdot;
        if (hybrid_inverse_method_ == "QP")
        {
            // 加权单层 QP：平移行权重 = lin_task_weight（机型 YAML 配置 5）。冲突时
            // 平移残差权重高 w² 倍（软优先级，平移达成 ~95%），且全局最小
            // 二乘对任何奇异方向增益非负、弱方向关节速度自动衰减。此前严格
            // 分层在近奇异+关节限位时：一级解对旋转零约束（顺带产生反向
            // 旋转），二级矫正经 A2⁺ 在弱方向放大关节速度并触发分量限速，
            // 误差不减 → 指令饱和 → 持续振荡。
            // 盒约束由求解器对整体问题一次处理。
            Eigen::MatrixXd Jw = J;
            Eigen::VectorXd vw = v_des;
            for (int r = 0; r < 3; ++r)
            {
                Jw.row(r) *= hybrid_lin_task_weight_;
                vw(r) *= hybrid_lin_task_weight_;
            }
            qdot = solveVelocityQp(Jw, vw, hybrid_qp_lambda_, vmax_up, vmax_dn);
        }
        else
        {
            const Eigen::MatrixXd JJt = J * J.transpose();
            const Eigen::MatrixXd reg = JJt + hybrid_dls_lambda_ * hybrid_dls_lambda_ *
                Eigen::MatrixXd::Identity(JJt.rows(), JJt.cols());
            qdot = J.transpose() * reg.ldlt().solve(v_des);
        }
        if (!qdot.allFinite()) return;

        // 7 自由度手臂在完整 6D 任务下仍有一个自运动自由度。原 QP 的
        // λ²||qdot||² 会把纯零空间速度压成 0，因此 j4 到上限后没有主动
        // 退限动作，并在近奇异区形成日志中的“v 非零、vach≈0”。这里只在
        // 关节行程末端 20% 内生成回中速度，再投影到 J 的精确零空间；所以
        // 不改变本周期的笛卡尔速度，也不会重新引入平移/旋转任务冲突。
        double diag_limit_escape = 0.0;
        double diag_limit_escape_scale = 0.0;
        if (hybrid_inverse_method_ == "QP" &&
            hybrid_joint_limit_avoidance_gain_ > 0.0)
        {
            constexpr double kActivation = 0.8;
            Eigen::VectorXd center_velocity = Eigen::VectorXd::Zero(n);
            for (Eigen::Index i = 0;
                 i < n && i < a.joint_lower.size() && i < a.joint_upper.size(); ++i)
            {
                const double lo = a.joint_lower(i) + hybrid_joint_limit_margin_;
                const double hi = a.joint_upper(i) - hybrid_joint_limit_margin_;
                const double half_range = 0.5 * (hi - lo);
                if (half_range <= 1e-6) continue;
                const double q = hold_positions_[offset + static_cast<size_t>(i)];
                const double normalized = (q - 0.5 * (lo + hi)) / half_range;
                const double activation = std::clamp(
                    (std::abs(normalized) - kActivation) / (1.0 - kActivation),
                    0.0, 1.0);
                if (activation > 0.0)
                    center_velocity(i) = -std::copysign(
                        hybrid_joint_limit_avoidance_gain_ * activation, normalized);
            }

            if (center_velocity.squaredNorm() > 1e-12)
            {
                Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinV);
                Eigen::MatrixXd nullspace = Eigen::MatrixXd::Identity(n, n);
                const double rank_eps = 1e-6 * std::max(
                    1e-12, svd.singularValues().maxCoeff());
                for (Eigen::Index k = 0; k < svd.singularValues().size(); ++k)
                {
                    if (svd.singularValues()(k) <= rank_eps) break;
                    nullspace.noalias() -=
                        svd.matrixV().col(k) * svd.matrixV().col(k).transpose();
                }

                const Eigen::VectorXd qdot_escape = nullspace * center_velocity;
                if (qdot_escape.allFinite() && qdot_escape.squaredNorm() > 1e-12)
                {
                    double scale = 1.0;
                    for (Eigen::Index i = 0; i < n; ++i)
                    {
                        if (qdot_escape(i) > 1e-12)
                            scale = std::min(scale, std::max(
                                0.0, (vmax_up(i) - qdot(i)) / qdot_escape(i)));
                        else if (qdot_escape(i) < -1e-12)
                            scale = std::min(scale, std::max(
                                0.0, (qdot(i) + vmax_dn(i)) / -qdot_escape(i)));
                    }
                    scale = std::clamp(scale, 0.0, 1.0);
                    qdot.noalias() += scale * qdot_escape;
                    diag_limit_escape = (scale * qdot_escape).norm();
                    diag_limit_escape_scale = scale;
                }
            }
        }

        // KKT/饱和诊断在 escape 合并之后对最终输出计算（此前在 escape 之前，
        // 字段不代表实际发出的 qdot）。
        double diag_kkt = -1.0;
        if (diag_due)
        {
            for (Eigen::Index i = 0; i < n; ++i)
            {
                if (std::abs(qdot(i) + vmax_dn(i)) < 1e-5 ||
                    std::abs(qdot(i) - vmax_up(i)) < 1e-5)
                {
                    diag_saturated += (diag_saturated.empty() ? "" : " ") +
                        std::string("j") + std::to_string(static_cast<long>(i + 1));
                }
            }
            if (hybrid_inverse_method_ == "QP")
            {
                // 加权单层问题的 KKT 站点性残差（验证盒约束 QP 收敛正常）。
                Eigen::MatrixXd Jw = J;
                Eigen::VectorXd vw = v_des;
                for (int r = 0; r < 3; ++r)
                {
                    Jw.row(r) *= hybrid_lin_task_weight_;
                    vw(r) *= hybrid_lin_task_weight_;
                }
                const Eigen::VectorXd gradient =
                    Jw.transpose() * (Jw * qdot - vw) +
                    hybrid_qp_lambda_ * hybrid_qp_lambda_ * qdot;
                diag_kkt = 0.0;
                for (Eigen::Index i = 0; i < n; ++i)
                {
                    double violation = std::abs(gradient(i));
                    const double lower = -vmax_dn(i);
                    const double upper = vmax_up(i);
                    if (upper - lower < 1e-9) violation = 0.0;
                    else if (qdot(i) <= lower + 1e-5) violation = std::max(0.0, -gradient(i));
                    else if (qdot(i) >= upper - 1e-5) violation = std::max(0.0, gradient(i));
                    diag_kkt = std::max(diag_kkt, violation);
                }
            }
        }

        const Eigen::VectorXd qdot_solved = qdot;

        // 不可达时只诊断，不改写关节位置命令。旧实现会在持续 0.5 s 后
        // 把 hold_positions_ 拉向实测值；奇异判定误触发时，这条路径就是
        // 位置轴漂移的直接来源。停滞时输出完整 QP 快照（q 指令/实测、
        // 速度盒、σ(Jw)、Uᵀ·W·v），用于离线区分：物理不可达 / 被关节
        // 上限困住 / 正则过强 / 平移-旋转任务冲突。
        {
            const Eigen::Matrix<double, 6, 1> v_res = v_des - J * qdot;
            double num = 0.0, den = 0.0;
            for (int i = 0; i < 6; ++i)
            {
                if (S(i) >= 0.5) continue;
                num += v_res(i) * v_res(i);
                den += v_des(i) * v_des(i);
            }
            const bool stalled = den > 1e-4 && std::sqrt(num / den) > 0.9;
            if (stalled && node_)
            {
                static rclcpp::Clock kStallWarnClock(RCL_STEADY_TIME);
                RCLCPP_WARN_THROTTLE(node_->get_logger(), kStallWarnClock, 5000,
                    "COMPLIANCE %s: target unreachable from current posture — holding "
                    "nearest reachable pose (unrealizable velocity %.3f); "
                    "see stall snapshot below", kArmName[is_left ? 0 : 1], std::sqrt(num));

                // 快照（与上面同节流 5s）：单条多行日志，便于拷贝离线分析。
                char joints[512] = {0};
                char boxes[512] = {0};
                size_t off_j = 0, off_b = 0;
                const auto& pos_if = ctrl_interfaces_.joint_position_state_interface_;
                for (Eigen::Index i = 0; i < n; ++i)
                {
                    const double q_meas = offset + i < pos_if.size()
                        ? pos_if[offset + i].get().get_optional().value_or(
                              hold_positions_[offset + static_cast<size_t>(i)])
                        : hold_positions_[offset + static_cast<size_t>(i)];
                    const double lo = i < a.joint_lower.size() ? a.joint_lower(i) : 0.0;
                    const double hi = i < a.joint_upper.size() ? a.joint_upper(i) : 0.0;
                    off_j += std::snprintf(joints + off_j, sizeof(joints) - off_j,
                        "%s%.4f/%.4f", i ? " " : "", q_meas,
                        hold_positions_[offset + static_cast<size_t>(i)]);
                    off_b += std::snprintf(boxes + off_b, sizeof(boxes) - off_b,
                        "%sj%ld[%.3f,%.3f]qdot<%.3f,%.3f>", i ? " " : "",
                        static_cast<long>(i + 1), lo, hi, -vmax_dn(i), vmax_up(i));
                }
                char sig[512] = {0};
                char proj[512] = {0};
                size_t off_s = 0, off_p = 0;
                {
                    Eigen::MatrixXd Jw = J;
                    Eigen::VectorXd vw = v_des;
                    for (int r = 0; r < 3; ++r)
                    {
                        Jw.row(r) *= hybrid_lin_task_weight_;
                        vw(r) *= hybrid_lin_task_weight_;
                    }
                    Eigen::JacobiSVD<Eigen::MatrixXd> svd_w(
                        Jw, Eigen::ComputeThinU | Eigen::ComputeThinV);
                    for (int k = 0; k < svd_w.singularValues().size(); ++k)
                        off_s += std::snprintf(sig + off_s, sizeof(sig) - off_s,
                            "%s%.4f", k ? " " : "", svd_w.singularValues()(k));
                    const Eigen::VectorXd uv = svd_w.matrixU().transpose() * vw;
                    for (int k = 0; k < uv.size(); ++k)
                        off_p += std::snprintf(proj + off_p, sizeof(proj) - off_p,
                            "%s%.3f", k ? " " : "", uv(k));
                }
                const Eigen::Matrix<double, 6, 1> v_ach = J * qdot;
                RCLCPP_WARN(node_->get_logger(),
                    "COMPLIANCE stall snapshot %s:\n"
                    "  q_meas/q_cmd=[%s]\n"
                    "  bounds: %s\n"
                    "  v_des=[%.3f %.3f %.3f %.3f %.3f %.3f] "
                    "vach=[%.3f %.3f %.3f %.3f %.3f %.3f]\n"
                    "  W=%s(1,1,1) lambda=%.3f sigma(Jw)=[%s] U^T*W*v=[%s] "
                    "escape=%.3f/%.2f dt=%.4f",
                    kArmName[is_left ? 0 : 1], joints, boxes,
                    v_des(0), v_des(1), v_des(2), v_des(3), v_des(4), v_des(5),
                    v_ach(0), v_ach(1), v_ach(2), v_ach(3), v_ach(4), v_ach(5),
                    std::to_string(hybrid_lin_task_weight_).c_str(),
                    hybrid_qp_lambda_, sig, proj,
                    diag_limit_escape, diag_limit_escape_scale, dt);
            }
        }

        for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(nq); ++i)
            qdot(i) = std::clamp(qdot(i), -vmax_dn(i), vmax_up(i));

        for (size_t i = 0; i < nq; ++i) {
            const Eigen::Index ai = static_cast<Eigen::Index>(i);
            const double old = hold_positions_[offset + i];
            double new_pos = old + qdot(ai) * dt;
            // 求解器已考虑限位；这里保留数值保护和诊断。
            if (ai < a.joint_lower.size() && ai < a.joint_upper.size() &&
                a.joint_lower(ai) < a.joint_upper(ai))
            {
                const double lo = a.joint_lower(ai) + hybrid_joint_limit_margin_;
                const double hi = a.joint_upper(ai) - hybrid_joint_limit_margin_;
                const double clamped = std::clamp(new_pos, lo, hi);
                if (clamped != new_pos)
                {
                    qdot(ai) = (clamped - old) / dt;
                    if (diag_due)
                        diag_jlim += (diag_jlim.empty() ? "" : " ") +
                            std::to_string(static_cast<int>(ai)) +
                            (new_pos > hi ? "@hi" : "@lo");
                }
                new_pos = clamped;
            }
            hold_positions_[offset + i] = new_pos;
        }

        // 6/7 轴关节位置耦合限位（说明书《Marvin系列机器人使用说明书》图 4-4）：
        // 可行域为八边形，四条直边（|q6|≤60°、|q7|≤90°）已被 URDF 限位覆盖，
        // 四条斜边统一为 L1 约束 |q6|+|q7| ≤ wrist_coupling_max_（110°，左右臂相同）。
        // 越界时按 lim/s 等比缩小两关节幅值（L1 球的最近点投影：符号不变、
        // 幅值只减不增，故不会破坏上方已满足的 URDF 限位，无需再收回）。
        if (nq >= 2 && wrist_coupling_max_ > 0.0)
        {
            const Eigen::Index i6 = static_cast<Eigen::Index>(nq) - 2;
            const Eigen::Index i7 = static_cast<Eigen::Index>(nq) - 1;
            const double q6 = hold_positions_[offset + i6];
            const double q7 = hold_positions_[offset + i7];
            const double s = std::abs(q6) + std::abs(q7);
            const double lim = wrist_coupling_max_ - hybrid_joint_limit_margin_;
            if (s > lim)
            {
                const double k = lim / s;
                hold_positions_[offset + i6] = q6 * k;
                hold_positions_[offset + i7] = q7 * k;
                if (diag_due) diag_jlim += " coupling";
                if (node_)
                {
                    static rclcpp::Clock kCouplingWarnClock(RCL_STEADY_TIME);
                    RCLCPP_WARN_THROTTLE(
                        node_->get_logger(), kCouplingWarnClock, 1000,
                        "COMPLIANCE %s wrist projection: q6=%.4f q7=%.4f sum=%.4f > %.4f, "
                        "command jump=[%.4f %.4f] rad",
                        kArmName[is_left ? 0 : 1], q6, q7, s, lim,
                        q6 * k - q6, q7 * k - q7);
                }
            }
        }

        // 诊断输出（节流）：字段含义见 COMPLIANCE.md"诊断日志"。
        // err大+qdot小+σmin小 → 奇异衰减；jlim 非空 → 关节限位钳制；
        // 力轴 |f_err|>死区且静止 → 零力残差；track_gap 大 → 硬件跟踪下垂。
        if (diag_due && node_)
        {
            last_diag_log_[is_left ? 0 : 1] = std::chrono::steady_clock::now();
            double sigma_min = -1.0;
            {
                Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
                sigma_min = svd.singularValues().minCoeff();
            }
            const double track_gap = (cur.position - a.cmd_pose.position).norm();
            Eigen::Quaterniond q_track(a.cmd_pose.rotationMatrix * cur.rotationMatrix.transpose());
            if (q_track.w() < 0.0) q_track.coeffs() *= -1.0;
            const double track_rot_gap = Eigen::AngleAxisd(q_track).angle();

            double q_gap = 0.0;
            Eigen::Index q_gap_joint = -1;
            const auto& pos_if = ctrl_interfaces_.joint_position_state_interface_;
            for (Eigen::Index i = 0; i < n; ++i)
            {
                const size_t index = offset + static_cast<size_t>(i);
                if (index >= pos_if.size()) break;
                const double measured = pos_if[index].get().get_optional()
                    .value_or(hold_positions_[index]);
                const double gap = hold_positions_[index] - measured;
                if (std::abs(gap) > std::abs(q_gap))
                {
                    q_gap = gap;
                    q_gap_joint = i;
                }
            }

            const size_t target_updates = a.target_updates - a.diag_target_updates;
            double target_shift = 0.0;
            double target_rot_shift = 0.0;
            if (a.diag_target_valid)
            {
                target_shift = (tgt.position - a.diag_target.position).norm();
                Eigen::Quaterniond q_target(
                    tgt.rotationMatrix * a.diag_target.rotationMatrix.transpose());
                if (q_target.w() < 0.0) q_target.coeffs() *= -1.0;
                target_rot_shift = Eigen::AngleAxisd(q_target).angle();
            }
            a.diag_target = tgt;
            a.diag_target_valid = true;
            a.diag_target_updates = a.target_updates;

            // 误差/达成速度只对位控轴有意义：力控轴不追踪位置，显示 --。
            const Eigen::Matrix<double, 6, 1> v_ach = J * qdot;
            const Eigen::Matrix<double, 6, 1> v_ach_solved = J * qdot_solved;
            const double align_solved = v_des.dot(v_ach_solved);
            const double align_final = v_des.dot(v_ach);
            double position_align = 0.0;
            for (int i = 0; i < 6; ++i)
                if (S(i) < 0.5) position_align += v_des(i) * v_ach(i);
            const double post_clip = (qdot - qdot_solved).lpNorm<Eigen::Infinity>();
            // 不可达残差（位控轴）：v_des 与可达成速度的差，持续大 = 顶在
            // 可达边界、停在最近可达点。
            double diag_res = 0.0;
            for (int i = 0; i < 6; ++i)
                if (S(i) < 0.5) diag_res += (v_des(i) - v_ach(i)) * (v_des(i) - v_ach(i));
            diag_res = std::sqrt(diag_res);
            char e[6][12], va[6][12];
            for (int i = 0; i < 6; ++i)
            {
                if (S(i) >= 0.5)
                {
                    std::snprintf(e[i], sizeof(e[i]), "--");
                    std::snprintf(va[i], sizeof(va[i]), "--");
                }
                else
                {
                    std::snprintf(e[i], sizeof(e[i]), "%+.3f", diag_err[i]);
                    std::snprintf(va[i], sizeof(va[i]), "%+.3f", v_ach(i));
                }
            }
            char buf[1024];
            std::snprintf(buf, sizeof(buf),
                "[COMPLIANCE diag] %s err=[%s %s %s %s %s %s] | "
                "v=[%.3f %.3f %.3f %.2f %.2f %.2f] vach=[%s %s %s %s %s %s] res=%.3f | "
                "|qdot|=%.4f escape=%.4f/%.2f smin=%.3f kkt=%.2e "
                "align=[%.3e %.3e pos=%.3e] clip=%.3e "
                "bounds=[%s] sat=[%s] jlim=[%s] | "
                "qgap=[j%ld %+.4f] tcp_gap=[%.3f m %.3f rad] target=[%zu %.3f m %.3f rad] | "
                "f_err=[%.2f %.2f %.2f %.2f %.2f %.2f] f_eff=[%.2f %.2f %.2f %.2f %.2f %.2f]",
                kArmName[is_left ? 0 : 1],
                e[0], e[1], e[2], e[3], e[4], e[5],
                v_des(0), v_des(1), v_des(2), v_des(3), v_des(4), v_des(5),
                va[0], va[1], va[2], va[3], va[4], va[5], diag_res,
                qdot.norm(), diag_limit_escape, diag_limit_escape_scale,
                sigma_min, diag_kkt,
                align_solved, align_final, position_align, post_clip,
                diag_bounds.c_str(), diag_saturated.c_str(), diag_jlim.c_str(),
                static_cast<long>(q_gap_joint + 1), q_gap,
                track_gap, track_rot_gap,
                target_updates, target_shift, target_rot_shift,
                diag_f_err[0], diag_f_err[1], diag_f_err[2], diag_f_err[3], diag_f_err[4], diag_f_err[5],
                diag_f_eff[0], diag_f_eff[1], diag_f_eff[2], diag_f_eff[3], diag_f_eff[4], diag_f_eff[5]);
            RCLCPP_INFO(node_->get_logger(), "%s", buf);
        }

        // 存下本周期的最终关节速度（含限位/耦合钳制修正），供下一周期
        // 速度阻尼反馈使用。
        a.qdot_prev = qdot;

        a.cmd_pose = kinematics_->computeSingleEndEffectorPose(
            makeRobotState(/*measured=*/false), kArmName[is_left ? 0 : 1]);
    }

    Eigen::VectorXd StateCompliance::solveVelocityQp(
        const Eigen::MatrixXd& J,
        const Eigen::VectorXd& v_des,
        double lambda, const Eigen::VectorXd& vmax_up,
        const Eigen::VectorXd& vmax_dn) const
    {
        const Eigen::Index n = J.cols();
        if (n <= 0 || J.rows() <= 0) return Eigen::VectorXd::Zero(std::max<Eigen::Index>(0, n));

        // H = JᵀJ + λ²I (对称正定), c = -Jᵀv
        const Eigen::MatrixXd H = J.transpose() * J +
            lambda * lambda * Eigen::MatrixXd::Identity(n, n);
        const Eigen::VectorXd c = -J.transpose() * v_des;
        const Eigen::VectorXd lo = -vmax_dn;
        const Eigen::VectorXd hi = vmax_up;

        // 坐标下降精确解：每坐标无约束最优 = (−c_i − Σ_{j≠i}H_ij·x_j)/H_ii，
        // 投影回盒内；对 H 的条件数不敏感（此前 PGD 在小 λ 下收敛慢，被迫用
        // 大 λ 牺牲任务增益——σ≈0.5 的健康方向被 λ=1 压到 ~25%）。初值用
        // 直接解+投影，n=7 时 60 轮扫描开销可忽略。
        Eigen::VectorXd qdot = H.ldlt().solve(-c);
        if (!qdot.allFinite()) qdot.setZero(n);
        qdot = qdot.cwiseMax(lo).cwiseMin(hi);

        for (int sweep = 0; sweep < 60; ++sweep)
        {
            double delta = 0.0;
            for (Eigen::Index i = 0; i < n; ++i)
            {
                double r = -c(i);
                for (Eigen::Index j = 0; j < n; ++j)
                    if (j != i) r -= H(i, j) * qdot(j);
                const double xi = std::clamp(r / H(i, i), lo(i), hi(i));
                delta = std::max(delta, std::abs(xi - qdot(i)));
                qdot(i) = xi;
            }
            if (delta < 1e-9) break;
        }
        return qdot;
    }

    // 分层（字典序优先级）逆解已移除：近奇异+关节限位时一级平移解对旋转
    // 零约束（顺带产生反向旋转），二级矫正经 A2⁺ 在弱方向放大关节速度并
    // 触发分量限速，实测出现 v_r 与 vach_r 反号和持续振荡。
    // 由加权单层 QP（平移行权重 lin_task_weight）取代：软优先级 + 全局
    // 最小二乘的 graceful degradation。

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
        int side_index, const std::string& sensor_frame,
        Eigen::Matrix<double, 6, 1>& wrench_sensor) const
    {
        wrench_sensor.setZero();
        if (!tf_buffer_ || sensor_frame.empty() || side_index < 0 || side_index > 1)
            return false;

        const auto& dyn = arms_[side_index].dyn_param;
        if (dyn.size() < 4 || dyn[0] < 1e-6) return false;

        const double mass = dyn[0];
        const Eigen::Vector3d com_tcp(dyn[1] * 1e-3, dyn[2] * 1e-3, dyn[3] * 1e-3);

        geometry_msgs::msg::TransformStamped tf_sensor_from_tcp;
        geometry_msgs::msg::TransformStamped tf_world_from_sensor;
        try
        {
            // Tool CoM is normally expressed in the TCP frame.  Some models
            // expose only *_eef; use it as a compatible fallback.
            const std::array<const char*, 2> tool_frames = {
                kTcpFrame[side_index], side_index == 0 ? "left_eef" : "right_eef"};
            bool have_tool_tf = false;
            for (const char* tool_frame : tool_frames)
            {
                try
                {
                    tf_sensor_from_tcp = tf_buffer_->lookupTransform(
                        sensor_frame, tool_frame, tf2::TimePointZero);
                    have_tool_tf = true;
                    break;
                }
                catch (const tf2::TransformException&) {}
            }
            if (!have_tool_tf) return false;

            try
            {
                tf_world_from_sensor =
                    tf_buffer_->lookupTransform(gravity_frame_, sensor_frame,
                                                tf2::TimePointZero);
            }
            catch (const tf2::TransformException&)
            {
                tf_world_from_sensor =
                    tf_buffer_->lookupTransform("base_link", sensor_frame,
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
                                   const std::string& sensor_frame,
                                   const std::array<double, 6>& wrench_ee) const
    {
        // 旋转到 base 系，并把力矩参考点从传感器原点平移到 EE 原点
        //（J 的角速度行参考 EE 原点）：t_ee = R·t_sensor + r × (R·f)，
        // r = p_sensor − p_ee。不做平移时角力控的 wrench 与 twist 不满足
        // 同参考点的功率共轭（r×F 残差，传感器离 EE 越远影响越大）。
        // 原始/重力 wrench 均为传感器系原点参考，减法后统一平移，保持一致。
        Eigen::Matrix<double, 6, 1> w_base;
        for (int i = 0; i < 6; ++i) w_base(i) = wrench_ee[i];
        if (side_index < 0 || side_index > 1) return w_base;

        Eigen::Vector3d p_sensor_base = Eigen::Vector3d::Zero();
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        bool have_sensor_pose = false;

        if (tf_buffer_)
        {
            try
            {
                geometry_msgs::msg::TransformStamped tf;
                try
                {
                    tf = tf_buffer_->lookupTransform(
                        gravity_frame_, sensor_frame, tf2::TimePointZero);
                }
                catch (const tf2::TransformException&)
                {
                    tf = tf_buffer_->lookupTransform(
                        "base_link", sensor_frame, tf2::TimePointZero);
                }
                const auto& q = tf.transform.rotation;
                R = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
                const auto& tr = tf.transform.translation;
                p_sensor_base = Eigen::Vector3d(tr.x, tr.y, tr.z);
                have_sensor_pose = true;
            }
            catch (const tf2::TransformException&) {}
        }

        if (!have_sensor_pose && kinematics_)
        {
            // Prefer the FT sensor frame (matches the TF path above); fall back to
            // the legacy eef frame name if the sensor frame is absent from the model.
            const std::array<std::string, 3> fallback_frames = {
                sensor_frame, kFtFrame[side_index], side_index == 0 ? "left_eef" : "right_eef"};
            for (const auto& frame_name : fallback_frames)
            {
                try
                {
                    auto pose = kinematics_->computeFramePose(
                        makeRobotState(/*measured=*/true), frame_name);
                    R = pose.rotationMatrix;
                    p_sensor_base = pose.position;
                    have_sensor_pose = true;
                    break;
                }
                catch (...) {}
            }
        }
        if (!have_sensor_pose) return w_base;

        Eigen::Vector3d f(wrench_ee[0], wrench_ee[1], wrench_ee[2]);
        Eigen::Vector3d t(wrench_ee[3], wrench_ee[4], wrench_ee[5]);
        const Eigen::Vector3d f_base = R * f;
        const Eigen::Vector3d t_base = R * t;

        // EE 原点在 base 系的位置（与 Jacobian 参考点一致）；取不到则退回
        // 仅旋转（保持旧行为）。
        Eigen::Vector3d p_ee_base;
        bool have_ee = false;
        if (kinematics_ && arms_[side_index].joint_count > 0)
        {
            try
            {
                const auto ee_pose = kinematics_->computeSingleEndEffectorPose(
                    makeRobotState(/*measured=*/true), kArmName[side_index]);
                p_ee_base = ee_pose.position;
                have_ee = true;
            }
            catch (...) {}
        }
        const Eigen::Vector3d t_ref =
            have_ee ? t_base + (p_sensor_base - p_ee_base).cross(f_base) : t_base;
        w_base << f_base, t_ref;
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
        ++a.target_updates;
        return true;
    }
} // namespace arms_controller_common
