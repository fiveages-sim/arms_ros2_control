//
// Common StateCompliance Implementation
//
// Full force/position hybrid control:
//   - Position baseline (held joints)
//   - Soft PD gains (compliance_gains) + gravity compensation in MIX mode
//   - Force feed-forward: τ_ff = J^T · W_base  (wrench rotated EE→base)
//   - Optional admittance: Cartesian M-D yields along external force
//

#include "arms_controller_common/FSM/StateCompliance.h"

#include <algorithm>
#include <utility>

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
    }

    void StateCompliance::setWrenchTopic(const std::string& left_topic, const std::string& right_topic)
    {
        left_wrench_topic_ = left_topic;
        right_wrench_topic_ = right_topic;
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
        auto get_bool = [this](const std::string& name, bool fallback) -> bool
        {
            try
            {
                return node_->get_parameter(name).get_value<bool>();
            }
            catch (...)
            {
                return fallback;
            }
        };

        try
        {
            compliance_gains_ = node_->get_parameter("compliance_gains").as_double_array();
        }
        catch (...)
        {
            if (compliance_gains_.size() < 2)
            {
                compliance_gains_ = {10.0, 1.0};
            }
        }

        force_feedforward_scale_ = get_double("compliance_force_ff_scale", 1.0);
        admittance_enabled_ = get_bool("compliance_admittance_enabled", false);
        admittance_max_displacement_ = get_double("compliance_admittance_max_displacement", 0.05);

        try
        {
            admittance_mass_ = node_->get_parameter("compliance_admittance_mass").as_double_array();
        }
        catch (...)
        {
            admittance_mass_ = {1.0, 1.0, 1.0, 0.1, 0.1, 0.1};
        }
        try
        {
            admittance_damping_ = node_->get_parameter("compliance_admittance_damping").as_double_array();
        }
        catch (...)
        {
            admittance_damping_ = {20.0, 20.0, 20.0, 2.0, 2.0, 2.0};
        }
    }

    void StateCompliance::setupWrenchSubscriptions()
    {
        if (!node_ || left_wrench_topic_.empty() || right_wrench_topic_.empty())
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
        };

        left_wrench_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
            left_wrench_topic_, rclcpp::SystemDefaultsQoS(),
            [cb](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) { cb(true, msg); });
        right_wrench_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
            right_wrench_topic_, rclcpp::SystemDefaultsQoS(),
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

    Eigen::Matrix<double, 6, 1>
    StateCompliance::wrenchToBase(int side_index, const std::array<double, 6>& wrench_ee) const
    {
        Eigen::Matrix<double, 6, 1> w_base;
        // Default: assume wrench already in base frame if no kinematics
        for (int i = 0; i < 6; ++i) w_base(i) = wrench_ee[i];

        if (!kinematics_) return w_base;

        // Build a dual-arm RobotState using zero for the other side; only the queried
        // side affects that side's EE pose.
        RobotState state(left_joint_count_, right_joint_count_);
        if (side_index == 0)
        {
            for (size_t i = 0; i < left_joint_count_ && i < 6; ++i)
                state.leftArmJoints(static_cast<Eigen::Index>(i)) = 0.0;
        }
        // Re-read current joint positions from interfaces instead of zero
        std::vector<double> current(left_joint_count_ + right_joint_count_, 0.0);
        size_t n = ctrl_interfaces_.joint_position_state_interface_.size();
        for (size_t i = 0; i < n && i < current.size(); ++i)
        {
            auto v = ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional();
            current[i] = v.value_or(0.0);
        }
        Eigen::VectorXd left, right;
        splitJoints(current, left, right);
        state.leftArmJoints = left;
        state.rightArmJoints = right;

        const std::string ee_name = (side_index == 0)
            ? kinematics_->getLeftArmJointNames().empty() ? std::string("left_eef") : std::string("left_eef")
            : std::string("right_eef");

        try
        {
            auto pose = kinematics_->computeFramePose(state, ee_name);
            Eigen::Matrix3d R = pose.rotationMatrix;  // base <- EE
            // Force transforms with R; torque transforms with R as well (free vector).
            Eigen::Vector3d f_ee(wrench_ee[0], wrench_ee[1], wrench_ee[2]);
            Eigen::Vector3d t_ee(wrench_ee[3], wrench_ee[4], wrench_ee[5]);
            Eigen::Vector3d f_base = R * f_ee;
            Eigen::Vector3d t_base = R * t_ee;
            w_base << f_base, t_base;
        }
        catch (...)
        {
            // Keep base = ee on failure (best effort)
        }
        return w_base;
    }

    Eigen::VectorXd
    StateCompliance::computeForceFeedforward(int side_index,
                                              const Eigen::Matrix<double, 6, 1>& wrench_base)
    {
        if (!kinematicsAvailable())
        {
            return Eigen::VectorXd{};
        }

        // Build current state
        std::vector<double> current(left_joint_count_ + right_joint_count_, 0.0);
        size_t n = ctrl_interfaces_.joint_position_state_interface_.size();
        for (size_t i = 0; i < n && i < current.size(); ++i)
        {
            auto v = ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional();
            current[i] = v.value_or(0.0);
        }
        Eigen::VectorXd left, right;
        splitJoints(current, left, right);
        RobotState state(left_joint_count_, right_joint_count_);
        state.leftArmJoints = left;
        state.rightArmJoints = right;

        const std::string arm = (side_index == 0) ? "left" : "right";
        Eigen::MatrixXd J = kinematics_->computeJacobian(state, arm);  // 6 x N (base frame)
        if (J.cols() == 0)
        {
            return Eigen::VectorXd{};
        }
        Eigen::VectorXd tau = J.transpose() * wrench_base * force_feedforward_scale_;
        return tau;
    }

    Eigen::VectorXd
    StateCompliance::computeAdmittanceOffset(int side_index,
                                              const Eigen::Matrix<double, 6, 1>& wrench_base,
                                              double dt)
    {
        if (!kinematicsAvailable() || !admittance_enabled_ || dt <= 1e-6)
        {
            return Eigen::VectorXd::Zero(static_cast<Eigen::Index>(
                (side_index == 0) ? left_joint_count_ : right_joint_count_));
        }

        // Build current state and Jacobian
        std::vector<double> current(left_joint_count_ + right_joint_count_, 0.0);
        size_t n = ctrl_interfaces_.joint_position_state_interface_.size();
        for (size_t i = 0; i < n && i < current.size(); ++i)
        {
            auto v = ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional();
            current[i] = v.value_or(0.0);
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
            return Eigen::VectorXd::Zero(static_cast<Eigen::Index>(
                (side_index == 0) ? left_joint_count_ : right_joint_count_));
        }

        // Mass / damping diagonal matrices
        Eigen::DiagonalMatrix<double, 6> M, D;
        for (int i = 0; i < 6; ++i)
        {
            double m = (i < static_cast<int>(admittance_mass_.size())) ? admittance_mass_[i] : 1.0;
            double d = (i < static_cast<int>(admittance_damping_.size())) ? admittance_damping_[i] : 10.0;
            m = std::max(m, 1e-3);
            d = std::max(d, 1e-3);
            M.diagonal()(i) = m;
            D.diagonal()(i) = d;
        }

        // Forward Euler integration of M·ẍ + D·ẋ = W
        // ẋ_{k+1} = ẋ_k + dt * M^{-1} (W - D·ẋ_k)
        auto& vel = (side_index == 0) ? adm_state_left_ : adm_state_right_;
        Eigen::Matrix<double, 6, 1> accel = M.inverse() * (wrench_base - D * vel);
        vel = vel + dt * accel;
        // Velocity limiting: cap |ẋ| to keep integration stable
        for (int i = 0; i < 6; ++i)
        {
            double vmax = 0.5;  // m/s or rad/s
            if (std::abs(vel(i)) > vmax) vel(i) = std::copysign(vmax, vel(i));
        }
        // Cartesian displacement this step
        Eigen::Matrix<double, 6, 1> dx = vel * dt;
        // Cap per-DOF displacement
        for (int i = 0; i < 6; ++i)
        {
            if (std::abs(dx(i)) > admittance_max_displacement_)
                dx(i) = std::copysign(admittance_max_displacement_, dx(i));
        }

        // Map Cartesian offset to joints via damped least-squares inverse
        const double lambda = 0.05;
        Eigen::MatrixXd JJt = J * J.transpose() + lambda * lambda * Eigen::Matrix<double, 6, 6>::Identity();
        Eigen::VectorXd dq = J.transpose() * JJt.inverse() * dx;
        return dq;
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

        // Reset admittance state
        adm_state_left_.setZero();
        adm_state_right_.setZero();
        last_admittance_time_ = node_ ? node_->now() : rclcpp::Time();

        setupWrenchSubscriptions();

        RCLCPP_INFO(node_->get_logger(),
                    "COMPLIANCE state entered, holding %zu joints (kp=%.2f, kd=%.2f); "
                    "force_ff_scale=%.2f, admittance=%s, kinematics=%s",
                    hold_positions_.size(),
                    compliance_gains_.size() >= 1 ? compliance_gains_[0] : 0.0,
                    compliance_gains_.size() >= 2 ? compliance_gains_[1] : 0.0,
                    force_feedforward_scale_,
                    admittance_enabled_ ? "ON" : "off",
                    kinematicsAvailable() ? "available" : "unavailable");
    }

    void StateCompliance::run(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
    {
        const double dt = period.seconds();

        // Snapshot external wrenches
        std::array<double, 6> w_left{}, w_right{};
        {
            std::lock_guard<std::mutex> lk(wrench_mutex_);
            w_left = left_wrench_;
            w_right = right_wrench_;
        }

        // ---- 1) Compute admittance joint offsets (per side) ----
        Eigen::VectorXd adm_left, adm_right;
        if (admittance_enabled_ && kinematicsAvailable())
        {
            // Use the timestamp from `time` for the integration interval
            (void)last_admittance_time_;  // we use `period` directly
            auto w_base_l = wrenchToBase(0, w_left);
            auto w_base_r = wrenchToBase(1, w_right);
            adm_left = computeAdmittanceOffset(0, w_base_l, dt);
            adm_right = computeAdmittanceOffset(1, w_base_r, dt);
        }

        // ---- 2) Position baseline + admittance offset ----
        for (size_t i = 0;
             i < ctrl_interfaces_.joint_position_command_interface_.size() && i < hold_positions_.size();
             ++i)
        {
            double cmd = hold_positions_[i];
            if (admittance_enabled_ && kinematicsAvailable())
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

        // ---- 3) MIX mode: soft PD + gravity comp + force feed-forward ----
        if (ctrl_interfaces_.control_mode_ == ControlMode::MIX)
        {
            const double kp = compliance_gains_.size() >= 1 ? compliance_gains_[0] : 0.0;
            const double kd = compliance_gains_.size() >= 2 ? compliance_gains_[1] : 0.0;

            for (auto& iface : ctrl_interfaces_.joint_kp_command_interface_)
            {
                std::ignore = iface.get().set_value(kp);
            }
            for (auto& iface : ctrl_interfaces_.joint_kd_command_interface_)
            {
                std::ignore = iface.get().set_value(kd);
            }

            // Gravity compensation baseline torques
            std::vector<double> static_torques;
            if (gravity_compensation_)
            {
                std::vector<double> current_positions;
                current_positions.reserve(ctrl_interfaces_.joint_position_state_interface_.size());
                for (auto& iface : ctrl_interfaces_.joint_position_state_interface_)
                {
                    auto value = iface.get().get_optional();
                    current_positions.push_back(value.value_or(0.0));
                }
                static_torques = gravity_compensation_->calculateStaticTorques(current_positions);
            }

            // Force feed-forward torques (only when wrench feed-in is present)
            Eigen::VectorXd tau_ff_left, tau_ff_right;
            if (kinematicsAvailable() && force_feedforward_scale_ != 0.0)
            {
                auto w_base_l = wrenchToBase(0, w_left);
                auto w_base_r = wrenchToBase(1, w_right);
                tau_ff_left = computeForceFeedforward(0, w_base_l);
                tau_ff_right = computeForceFeedforward(1, w_base_r);
            }

            // Write effort: τ = τ_gravity + τ_ff
            for (size_t i = 0; i < ctrl_interfaces_.joint_force_command_interface_.size(); ++i)
            {
                double tau = 0.0;
                if (gravity_compensation_ && i < static_torques.size())
                {
                    tau += static_torques[i];
                }
                if (kinematicsAvailable() && force_feedforward_scale_ != 0.0)
                {
                    if (i < left_joint_count_)
                    {
                        tau += tau_ff_left.size() > static_cast<Eigen::Index>(i)
                                   ? tau_ff_left(static_cast<Eigen::Index>(i)) : 0.0;
                    }
                    else
                    {
                        size_t idx = i - left_joint_count_;
                        tau += tau_ff_right.size() > static_cast<Eigen::Index>(idx)
                                   ? tau_ff_right(static_cast<Eigen::Index>(idx)) : 0.0;
                    }
                }
                std::ignore = ctrl_interfaces_.joint_force_command_interface_[i].get().set_value(tau);
            }
        }
    }

    void StateCompliance::exit()
    {
        // Drop wrench subscriptions on exit to avoid stale callbacks
        left_wrench_sub_.reset();
        right_wrench_sub_.reset();
    }

    FSMStateName StateCompliance::checkChange()
    {
        // Common transition: any state -> HOLD on command 2
        switch (ctrl_interfaces_.fsm_command_)
        {
        case 2:
            return FSMStateName::HOLD;
        default:
            return FSMStateName::COMPLIANCE;
        }
    }
} // namespace arms_controller_common
