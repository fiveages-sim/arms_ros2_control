//
// Common StateCompliance Implementation
//
// Enter → zero_cal + world-frame tool gravity → force outer / stiff position inner.
//

#include "arms_controller_common/FSM/StateCompliance.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include <tf2/exceptions.h>

namespace arms_controller_common
{
    StateCompliance::StateCompliance(CtrlInterfaces& ctrl_interfaces,
                                     std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
                                     const std::shared_ptr<GravityCompensation>& gravity_compensation,
                                     const std::shared_ptr<ArmKinematics>& kinematics)
        : FSMState(FSMStateName::COMPLIANCE, "COMPLIANCE", ctrl_interfaces),
          node_(std::move(node)),
          gravity_compensation_(gravity_compensation),
          kinematics_(kinematics)
    {
        if (kinematics_)
        {
            left_joint_count_ = kinematics_->getLeftArmJointCount();
            right_joint_count_ = kinematics_->getRightArmJointCount();
        }
        if (node_)
        {
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }
    }

    void StateCompliance::updateParam()
    {
        if (!node_)
        {
            return;
        }

        auto get_double = [this](const std::string& name, double fallback) -> double
        {
            try
            {
                return node_->get_parameter(name).get_value<double>();
            }
            catch (...)
            {
                return fallback;
            }
        };

        admittance_max_displacement_ =
            get_double("compliance_admittance_max_displacement", admittance_max_displacement_);
        force_deadband_ = get_double("compliance_force_deadband", force_deadband_);
        torque_deadband_ = get_double("compliance_torque_deadband", torque_deadband_);
        wrench_lpf_alpha_ = std::clamp(
            get_double("compliance_wrench_lpf_alpha", wrench_lpf_alpha_), 0.01, 1.0);
        zero_cal_duration_ = std::max(0.05, get_double("compliance_zero_cal_duration", 1.0));
        gravity_accel_ = get_double("compliance_gravity_accel", 9.81);

        try
        {
            admittance_mass_ = node_->get_parameter("compliance_admittance_mass").as_double_array();
        }
        catch (...)
        {
        }
        try
        {
            admittance_damping_ =
                node_->get_parameter("compliance_admittance_damping").as_double_array();
        }
        catch (...)
        {
        }
        try
        {
            left_dyn_param_ = node_->get_parameter("left_dyn_param").as_double_array();
        }
        catch (...)
        {
        }
        try
        {
            right_dyn_param_ = node_->get_parameter("right_dyn_param").as_double_array();
        }
        catch (...)
        {
        }
    }

    void StateCompliance::setupWrenchSubscriptions()
    {
        if (!node_)
        {
            return;
        }

        auto cb = [this](bool is_left, const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lk(wrench_mutex_);
            auto& wrench = is_left ? left_wrench_ : right_wrench_;
            wrench[0] = msg->wrench.force.x;
            wrench[1] = msg->wrench.force.y;
            wrench[2] = msg->wrench.force.z;
            wrench[3] = msg->wrench.torque.x;
            wrench[4] = msg->wrench.torque.y;
            wrench[5] = msg->wrench.torque.z;
            if (is_left)
            {
                left_ft_active_ = true;
            }
            else
            {
                right_ft_active_ = true;
            }
        };

        left_wrench_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
            kLeftWrenchTopic, rclcpp::SystemDefaultsQoS(),
            [cb](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) { cb(true, msg); });
        right_wrench_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
            kRightWrenchTopic, rclcpp::SystemDefaultsQoS(),
            [cb](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) { cb(false, msg); });
    }

    bool StateCompliance::kinematicsAvailable() const
    {
        return kinematics_ != nullptr && left_joint_count_ > 0 && right_joint_count_ > 0;
    }

    void StateCompliance::splitJoints(const std::vector<double>& all,
                                       Eigen::VectorXd& left, Eigen::VectorXd& right) const
    {
        left.resize(static_cast<Eigen::Index>(left_joint_count_));
        right.resize(static_cast<Eigen::Index>(right_joint_count_));
        for (size_t i = 0; i < left_joint_count_ && i < all.size(); ++i)
        {
            left(static_cast<Eigen::Index>(i)) = all[i];
        }
        for (size_t i = 0; i < right_joint_count_ && (left_joint_count_ + i) < all.size(); ++i)
        {
            right(static_cast<Eigen::Index>(i)) = all[left_joint_count_ + i];
        }
    }

    bool StateCompliance::computeToolGravityWrenchInSensorFrame(
        int side_index, Eigen::Matrix<double, 6, 1>& wrench_sensor) const
    {
        wrench_sensor.setZero();
        if (!tf_buffer_)
        {
            return false;
        }

        const std::vector<double>& dyn =
            (side_index == 0) ? left_dyn_param_ : right_dyn_param_;
        if (dyn.size() < 4 || dyn[0] < 1e-6)
        {
            return false;
        }
        const double mass = dyn[0];
        // Load ID: COM in tcp [mm] → m
        const Eigen::Vector3d com_tcp(dyn[1] * 1e-3, dyn[2] * 1e-3, dyn[3] * 1e-3);

        const char* tcp_frame = (side_index == 0) ? kLeftTcpFrame : kRightTcpFrame;
        const char* sensor_frame = (side_index == 0) ? kLeftFtFrame : kRightFtFrame;

        geometry_msgs::msg::TransformStamped tf_sensor_from_tcp;
        geometry_msgs::msg::TransformStamped tf_world_from_sensor;
        try
        {
            tf_sensor_from_tcp =
                tf_buffer_->lookupTransform(sensor_frame, tcp_frame, tf2::TimePointZero);
            try
            {
                tf_world_from_sensor =
                    tf_buffer_->lookupTransform(gravity_frame_, sensor_frame, tf2::TimePointZero);
            }
            catch (const tf2::TransformException&)
            {
                // world may be absent; base_link Z ≈ vertical when robot stands upright
                tf_world_from_sensor =
                    tf_buffer_->lookupTransform("base_link", sensor_frame, tf2::TimePointZero);
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

        // Gravity free vector: world −Z → sensor
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
    StateCompliance::wrenchToBase(int side_index, const std::array<double, 6>& wrench_ee) const
    {
        Eigen::Matrix<double, 6, 1> w_base;
        for (int i = 0; i < 6; ++i)
        {
            w_base(i) = wrench_ee[i];
        }

        // Prefer TF: gravity_frame/base ← sensor (same axes as admittance)
        if (tf_buffer_)
        {
            const char* sensor_frame = (side_index == 0) ? kLeftFtFrame : kRightFtFrame;
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
                Eigen::Matrix3d R =
                    Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
                Eigen::Vector3d f(wrench_ee[0], wrench_ee[1], wrench_ee[2]);
                Eigen::Vector3d t(wrench_ee[3], wrench_ee[4], wrench_ee[5]);
                w_base << (R * f), (R * t);
                return w_base;
            }
            catch (const tf2::TransformException&)
            {
            }
        }

        if (!kinematics_)
        {
            return w_base;
        }

        RobotState state(left_joint_count_, right_joint_count_);
        std::vector<double> current(left_joint_count_ + right_joint_count_, 0.0);
        size_t n = ctrl_interfaces_.joint_position_state_interface_.size();
        for (size_t i = 0; i < n && i < current.size(); ++i)
        {
            current[i] =
                ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional().value_or(0.0);
        }
        Eigen::VectorXd left, right;
        splitJoints(current, left, right);
        state.leftArmJoints = left;
        state.rightArmJoints = right;

        const std::string ee_name = (side_index == 0) ? "left_eef" : "right_eef";
        try
        {
            auto pose = kinematics_->computeFramePose(state, ee_name);
            Eigen::Matrix3d R = pose.rotationMatrix;
            Eigen::Vector3d f_ee(wrench_ee[0], wrench_ee[1], wrench_ee[2]);
            Eigen::Vector3d t_ee(wrench_ee[3], wrench_ee[4], wrench_ee[5]);
            w_base << (R * f_ee), (R * t_ee);
        }
        catch (...)
        {
        }
        return w_base;
    }

    Eigen::VectorXd
    StateCompliance::computeAdmittanceOffset(int side_index,
                                              const Eigen::Matrix<double, 6, 1>& wrench_base,
                                              double dt)
    {
        const Eigen::Index nq = static_cast<Eigen::Index>(
            (side_index == 0) ? left_joint_count_ : right_joint_count_);
        auto& vel = (side_index == 0) ? adm_vel_left_ : adm_vel_right_;
        auto& pos = (side_index == 0) ? adm_pos_left_ : adm_pos_right_;
        auto& w_filt = (side_index == 0) ? wrench_filt_left_ : wrench_filt_right_;

        const bool got_ft = (side_index == 0) ? left_ft_active_ : right_ft_active_;
        if (!got_ft || !kinematicsAvailable() || dt <= 1e-6)
        {
            vel.setZero();
            pos.setZero();
            w_filt.setZero();
            return Eigen::VectorXd::Zero(nq);
        }

        // Low-pass tared wrench, then deadband (noise / residual must not drive)
        w_filt = wrench_lpf_alpha_ * wrench_base + (1.0 - wrench_lpf_alpha_) * w_filt;
        Eigen::Matrix<double, 6, 1> w = w_filt;
        for (int i = 0; i < 3; ++i)
        {
            if (std::abs(w(i)) < force_deadband_)
            {
                w(i) = 0.0;
            }
        }
        for (int i = 3; i < 6; ++i)
        {
            if (std::abs(w(i)) < torque_deadband_)
            {
                w(i) = 0.0;
            }
        }

        Eigen::DiagonalMatrix<double, 6> M, D;
        for (int i = 0; i < 6; ++i)
        {
            double m = (i < static_cast<int>(admittance_mass_.size())) ? admittance_mass_[i] : 5.0;
            double d = (i < static_cast<int>(admittance_damping_.size())) ? admittance_damping_[i] : 80.0;
            M.diagonal()(i) = std::max(m, 1e-3);
            D.diagonal()(i) = std::max(d, 1e-3);
        }

        // M v̇ + D v = F  → accumulate Cartesian offset x (not one-step dx only)
        Eigen::Matrix<double, 6, 1> accel = M.inverse() * (w - D * vel);
        vel = vel + dt * accel;
        constexpr double vmax_lin = 0.08;
        constexpr double vmax_ang = 0.4;
        for (int i = 0; i < 3; ++i)
        {
            vel(i) = std::clamp(vel(i), -vmax_lin, vmax_lin);
        }
        for (int i = 3; i < 6; ++i)
        {
            vel(i) = std::clamp(vel(i), -vmax_ang, vmax_ang);
        }
        pos = pos + vel * dt;
        for (int i = 0; i < 6; ++i)
        {
            pos(i) = std::clamp(pos(i), -admittance_max_displacement_, admittance_max_displacement_);
        }

        std::vector<double> current(left_joint_count_ + right_joint_count_, 0.0);
        size_t n = ctrl_interfaces_.joint_position_state_interface_.size();
        for (size_t i = 0; i < n && i < current.size(); ++i)
        {
            current[i] =
                ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional().value_or(0.0);
        }
        Eigen::VectorXd left, right;
        splitJoints(current, left, right);
        RobotState state(left_joint_count_, right_joint_count_);
        state.leftArmJoints = left;
        state.rightArmJoints = right;

        const std::string arm = (side_index == 0) ? "left" : "right";
        Eigen::MatrixXd J = kinematics_->computeJacobian(state, arm);
        if (J.cols() == 0)
        {
            return Eigen::VectorXd::Zero(nq);
        }

        const double lambda = 0.05;
        Eigen::MatrixXd JJt =
            J * J.transpose() + lambda * lambda * Eigen::Matrix<double, 6, 6>::Identity();
        return J.transpose() * JJt.inverse() * pos;
    }

    void StateCompliance::enter()
    {
        updateParam();

        size_t num_joints = ctrl_interfaces_.joint_position_state_interface_.size();
        hold_positions_.resize(num_joints);
        for (size_t i = 0; i < num_joints; ++i)
        {
            hold_positions_[i] = ctrl_interfaces_.last_sent_joint_positions_[i];
        }

        adm_vel_left_.setZero();
        adm_vel_right_.setZero();
        adm_pos_left_.setZero();
        adm_pos_right_.setZero();
        wrench_filt_left_.setZero();
        wrench_filt_right_.setZero();
        wrench_bias_left_.fill(0.0);
        wrench_bias_right_.fill(0.0);
        zero_cal_sum_left_.fill(0.0);
        zero_cal_sum_right_.fill(0.0);
        zero_cal_samples_left_ = 0;
        zero_cal_samples_right_ = 0;
        zero_cal_pending_ = true;
        zero_cal_running_ = false;
        zero_cal_done_ = false;
        left_ft_active_ = false;
        right_ft_active_ = false;

        setupWrenchSubscriptions();

        const double m_l = left_dyn_param_.size() >= 1 ? left_dyn_param_[0] : 0.0;
        const double m_r = right_dyn_param_.size() >= 1 ? right_dyn_param_[0] : 0.0;
        RCLCPP_INFO(node_->get_logger(),
                    "COMPLIANCE: force outer + stiff position inner. "
                    "zero_cal=%.2fs; tool_gravity world-Z (m_L=%.3f kg, m_R=%.3f kg); "
                    "kinematics=%s",
                    zero_cal_duration_, m_l, m_r,
                    kinematicsAvailable() ? "ok" : "MISSING");
        if (m_l < 1e-6 && m_r < 1e-6)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "COMPLIANCE: left/right_dyn_param mass is zero — "
                        "tool gravity TF skipped; only zero_cal bias will be removed. "
                        "Set hardware.left_dyn_param in robot.local.yaml.");
        }
    }

    void StateCompliance::run(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        updateParam();
        const double dt = period.seconds();

        std::array<double, 6> raw_left{}, raw_right{};
        {
            std::lock_guard<std::mutex> lk(wrench_mutex_);
            raw_left = left_wrench_;
            raw_right = right_wrench_;
        }

        // 1) Tool gravity in sensor frame (world −Z via TF)
        Eigen::Matrix<double, 6, 1> w_grav_l = Eigen::Matrix<double, 6, 1>::Zero();
        Eigen::Matrix<double, 6, 1> w_grav_r = Eigen::Matrix<double, 6, 1>::Zero();
        bool grav_ok_l = false;
        bool grav_ok_r = false;
        if (left_ft_active_)
        {
            grav_ok_l = computeToolGravityWrenchInSensorFrame(0, w_grav_l);
        }
        if (right_ft_active_)
        {
            grav_ok_r = computeToolGravityWrenchInSensorFrame(1, w_grav_r);
        }

        std::array<double, 6> net_left = raw_left;
        std::array<double, 6> net_right = raw_right;
        if (grav_ok_l)
        {
            for (int i = 0; i < 6; ++i)
            {
                net_left[i] -= w_grav_l(i);
            }
        }
        if (grav_ok_r)
        {
            for (int i = 0; i < 6; ++i)
            {
                net_right[i] -= w_grav_r(i);
            }
        }

        // 2) Zero-cal: average residual after gravity removal (no contact)
        if (zero_cal_pending_ && (left_ft_active_ || right_ft_active_))
        {
            zero_cal_pending_ = false;
            zero_cal_running_ = true;
            zero_cal_start_ = time;
            RCLCPP_INFO(node_->get_logger(),
                        "COMPLIANCE zero_cal started (%.2fs). Keep arm still, no contact. "
                        "FT L=%s R=%s",
                        zero_cal_duration_,
                        left_ft_active_ ? "yes" : "no",
                        right_ft_active_ ? "yes" : "no");
        }

        if (zero_cal_running_)
        {
            for (int i = 0; i < 6; ++i)
            {
                zero_cal_sum_left_[i] += net_left[i];
                zero_cal_sum_right_[i] += net_right[i];
            }
            ++zero_cal_samples_left_;
            ++zero_cal_samples_right_;

            if ((time - zero_cal_start_).seconds() >= zero_cal_duration_)
            {
                for (int i = 0; i < 6; ++i)
                {
                    if (zero_cal_samples_left_ > 0)
                    {
                        wrench_bias_left_[i] =
                            zero_cal_sum_left_[i] / static_cast<double>(zero_cal_samples_left_);
                    }
                    if (zero_cal_samples_right_ > 0)
                    {
                        wrench_bias_right_[i] =
                            zero_cal_sum_right_[i] / static_cast<double>(zero_cal_samples_right_);
                    }
                }
                zero_cal_running_ = false;
                zero_cal_done_ = true;

                const double bias_f = std::sqrt(
                    wrench_bias_left_[0] * wrench_bias_left_[0] +
                    wrench_bias_left_[1] * wrench_bias_left_[1] +
                    wrench_bias_left_[2] * wrench_bias_left_[2]);
                const double mg =
                    (left_dyn_param_.size() >= 1 ? left_dyn_param_[0] : 0.0) * gravity_accel_;
                RCLCPP_INFO(node_->get_logger(),
                            "COMPLIANCE zero_cal done (%ld samples). "
                            "Bias_s L=[%.3f,%.3f,%.3f, %.3f,%.3f,%.3f] (|F|=%.2f N). "
                            "tool_gravity L=%s (mg=%.2f N). Force outer-loop ACTIVE.",
                            zero_cal_samples_left_,
                            wrench_bias_left_[0], wrench_bias_left_[1], wrench_bias_left_[2],
                            wrench_bias_left_[3], wrench_bias_left_[4], wrench_bias_left_[5],
                            bias_f,
                            grav_ok_l ? "OK" : "FAIL",
                            mg);
                if (left_dyn_param_.size() >= 1 && left_dyn_param_[0] > 1e-6 && !grav_ok_l)
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "COMPLIANCE tool gravity TF failed "
                                "(need %s←%s and world/base_link←%s).",
                                kLeftFtFrame, kLeftTcpFrame, kLeftFtFrame);
                }
            }
        }

        // Hold stiff position during zero_cal; enable admittance after done
        Eigen::VectorXd adm_left = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(left_joint_count_));
        Eigen::VectorXd adm_right = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(right_joint_count_));

        if (zero_cal_done_ && kinematicsAvailable())
        {
            std::array<double, 6> tared_left = net_left;
            std::array<double, 6> tared_right = net_right;
            for (int i = 0; i < 6; ++i)
            {
                tared_left[i] -= wrench_bias_left_[i];
                tared_right[i] -= wrench_bias_right_[i];
            }
            adm_left = computeAdmittanceOffset(0, wrenchToBase(0, tared_left), dt);
            adm_right = computeAdmittanceOffset(1, wrenchToBase(1, tared_right), dt);
        }

        for (size_t i = 0;
             i < ctrl_interfaces_.joint_position_command_interface_.size() && i < hold_positions_.size();
             ++i)
        {
            double cmd = hold_positions_[i];
            if (zero_cal_done_ && kinematicsAvailable())
            {
                if (i < left_joint_count_)
                {
                    cmd += adm_left.size() > static_cast<Eigen::Index>(i)
                               ? adm_left(static_cast<Eigen::Index>(i)) : 0.0;
                }
                else
                {
                    size_t idx = i - left_joint_count_;
                    cmd += adm_right.size() > static_cast<Eigen::Index>(idx)
                               ? adm_right(static_cast<Eigen::Index>(idx)) : 0.0;
                }
            }
            ctrl_interfaces_.setJointPositionCommand(i, cmd);
        }

        if (ctrl_interfaces_.control_mode_ == ControlMode::MIX)
        {
            if (ctrl_interfaces_.default_gains_.size() >= 2)
            {
                const double kp = ctrl_interfaces_.default_gains_[0];
                const double kd = ctrl_interfaces_.default_gains_[1];
                for (auto& iface : ctrl_interfaces_.joint_kp_command_interface_)
                {
                    std::ignore = iface.get().set_value(kp);
                }
                for (auto& iface : ctrl_interfaces_.joint_kd_command_interface_)
                {
                    std::ignore = iface.get().set_value(kd);
                }
            }
            if (gravity_compensation_)
            {
                std::vector<double> q;
                q.reserve(ctrl_interfaces_.joint_position_state_interface_.size());
                for (auto& iface : ctrl_interfaces_.joint_position_state_interface_)
                {
                    q.push_back(iface.get().get_optional().value_or(0.0));
                }
                const auto tau_g = gravity_compensation_->calculateStaticTorques(q);
                for (size_t i = 0;
                     i < ctrl_interfaces_.joint_force_command_interface_.size() && i < tau_g.size();
                     ++i)
                {
                    std::ignore =
                        ctrl_interfaces_.joint_force_command_interface_[i].get().set_value(tau_g[i]);
                }
            }
        }
    }

    void StateCompliance::exit()
    {
        left_wrench_sub_.reset();
        right_wrench_sub_.reset();
        zero_cal_pending_ = false;
        zero_cal_running_ = false;
        zero_cal_done_ = false;
    }

    FSMStateName StateCompliance::checkChange()
    {
        switch (ctrl_interfaces_.fsm_command_)
        {
            case 2:
                return FSMStateName::HOLD;
            default:
                return FSMStateName::COMPLIANCE;
        }
    }
} // namespace arms_controller_common
