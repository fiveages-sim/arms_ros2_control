#include "ocs2_arm_controller/control/ExternalWrenchEstimator.h"

#include <Eigen/Core>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/data.hpp>
#include <cmath>
#include <algorithm>

namespace ocs2::mobile_manipulator
{
    namespace
    {
        void fillWrenchMsg(geometry_msgs::msg::WrenchStamped& msg,
                           const rclcpp::Time& time,
                           const std::string& frame_id,
                           const Eigen::Matrix<double, 6, 1>& w)
        {
            msg.header.stamp = time;
            msg.header.frame_id = frame_id;
            msg.wrench.force.x = w(0);
            msg.wrench.force.y = w(1);
            msg.wrench.force.z = w(2);
            msg.wrench.torque.x = w(3);
            msg.wrench.torque.y = w(4);
            msg.wrench.torque.z = w(5);
        }

        Eigen::Matrix<double, 6, 1> filterWrench(const Eigen::Matrix<double, 6, 1>& raw,
                                                 Eigen::VectorXd& storage,
                                                 double alpha)
        {
            if (storage.size() != 6)
            {
                storage.resize(6);
                storage.setZero();
            }
            if (alpha <= 0.0)
            {
                return raw;
            }
            Eigen::Matrix<double, 6, 1> out;
            for (int i = 0; i < 6; ++i)
            {
                storage(i) = (1.0 - alpha) * storage(i) + alpha * raw(i);
                out(i) = storage(i);
            }
            return out;
        }
    } // namespace

    ExternalWrenchEstimator::ExternalWrenchEstimator(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
                                                       const bool enabled,
                                                       const double damping,
                                                       const double filter_alpha,
                                                       std::string wrench_frame_id)
        : node_(std::move(node)),
          enabled_(enabled),
          damping_(damping),
          filter_alpha_(filter_alpha),
          wrench_frame_id_(std::move(wrench_frame_id))
    {
    }

    void ExternalWrenchEstimator::setupPublishers(const std::string& robot_name, const bool dual_arm_mode)
    {
        if (dual_arm_mode)
        {
            publisher_left_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
                robot_name + "/left_arm_external_wrench", 10);
            publisher_right_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
                robot_name + "/right_arm_external_wrench", 10);
            publisher_single_.reset();
        }
        else
        {
            publisher_single_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
                robot_name + "/external_wrench", 10);
            publisher_left_.reset();
            publisher_right_.reset();
        }
    }

    Eigen::Matrix<double, 6, 1> ExternalWrenchEstimator::dampedWrenchFromTorque(
        const Eigen::Ref<const Eigen::MatrixXd>& J,
        const Eigen::Ref<const Eigen::VectorXd>& tau_segment,
        const double lambda)
    {
        Eigen::MatrixXd JJt = J * J.transpose();
        const int dim = static_cast<int>(JJt.rows());
        for (int i = 0; i < dim; ++i)
        {
            JJt(i, i) += lambda * lambda;
        }
        const Eigen::VectorXd f = JJt.ldlt().solve(J * tau_segment);
        Eigen::Matrix<double, 6, 1> out;
        out << f(0), f(1), f(2), f(3), f(4), f(5);
        return out;
    }

    void ExternalWrenchEstimator::publish(const rclcpp::Time& time,
                                          const vector_t& joint_positions,
                                          arms_controller_common::CtrlInterfaces& ctrl,
                                          const std::shared_ptr<MobileManipulatorInterface>& interface,
                                          const std::vector<std::string>& joint_names) const
    {
        if (!enabled_ || !interface)
        {
            return;
        }
        if (ctrl.joint_force_state_interface_.empty() ||
            ctrl.joint_force_state_interface_.size() != joint_names.size())
        {
            if (!warned_no_effort_)
            {
                RCLCPP_WARN(node_->get_logger(),
                            "External wrench: add 'effort' to state_interfaces in the controller YAML "
                            "(same joint count as position) to enable τ-based external wrench.");
                warned_no_effort_ = true;
            }
            return;
        }
        warned_no_effort_ = false;

        const auto& pin_if = interface->getPinocchioInterface();
        const auto& model = pin_if.getModel();
        // Use a fresh Data object to avoid any stale/NaN cached fields.
        pinocchio::Data data(model);

        if (static_cast<size_t>(joint_positions.size()) != joint_names.size() ||
            static_cast<size_t>(model.nq) != joint_names.size())
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "External wrench: joint count (%zu) does not match Pinocchio nq (%u).",
                                 joint_names.size(), static_cast<unsigned>(model.nq));
            return;
        }

        Eigen::VectorXd tau_meas(static_cast<Eigen::Index>(joint_names.size()));
        for (size_t i = 0; i < joint_names.size(); ++i)
        {
            tau_meas(static_cast<Eigen::Index>(i)) =
                ctrl.joint_force_state_interface_[i].get().get_optional().value_or(0.0);
        }

        if (!joint_positions.allFinite())
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "External wrench: joint_positions contains NaN/Inf, skip publishing.");
            return;
        }
        if (!tau_meas.allFinite())
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "External wrench: tau_meas contains NaN/Inf, skip publishing.");
            return;
        }

        const Eigen::VectorXd tau_model =
            pinocchio::rnea(model, data, joint_positions, Eigen::VectorXd::Zero(model.nv),
                            Eigen::VectorXd::Zero(model.nv));
        if (!tau_model.allFinite())
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "External wrench: tau_model (RNEA) contains NaN/Inf, skip publishing.");
            return;
        }
        const Eigen::VectorXd tau_ext = tau_meas - tau_model;
        if (!tau_ext.allFinite())
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "External wrench: tau_ext contains NaN/Inf, skip publishing.");
            return;
        }

        // Diagnostics: if efforts are always zero or perfectly cancel gravity, wrench will be ~0.
        {
            const double n_meas = tau_meas.norm();
            const double n_model = tau_model.norm();
            const double n_ext = tau_ext.norm();
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "External wrench diag: ||tau_meas||=%.6f ||tau_model||=%.6f ||tau_ext||=%.6f",
                                 n_meas, n_model, n_ext);
            if (n_meas < 1e-6)
            {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                     "External wrench diag: measured efforts are ~0. "
                                     "Check that controller 'state_interfaces' includes 'effort' and hardware publishes effort.");
            }
        }

        // Keep pinocchio data consistent with current configuration.
        // (forwardKinematics is required before updating frame placements/Jacobians in many pinocchio setups.)
        pinocchio::forwardKinematics(model, data, joint_positions);
        pinocchio::computeJointJacobians(model, data, joint_positions);
        pinocchio::updateFramePlacements(model, data);
        const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;

        const auto& info = interface->getManipulatorModelInfo();
        const std::string frame_id = wrench_frame_id_.empty() ? info.baseFrame : wrench_frame_id_;
        const double lambda = damping_;
        const double alpha = filter_alpha_;

        const bool dual_arm = interface->dual_arm_;

        try
        {
            if (!dual_arm)
            {
                if (!publisher_single_)
                {
                    return;
                }
                const pinocchio::FrameIndex fid = model.getFrameId(info.eeFrame);
                const auto& oMf = data.oMf[fid];
                if (!oMf.translation().allFinite() || !oMf.rotation().allFinite())
                {
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                         "External wrench: frame placement for eeFrame contains NaN/Inf, fid=%d, frame=%s. skip.",
                                         static_cast<int>(fid), info.eeFrame.c_str());
                    return;
                }
                Eigen::MatrixXd J(6, model.nv);
                J.setZero();
                pinocchio::getFrameJacobian(model, data, fid, rf, J);
                if (!J.allFinite())
                {
                    Eigen::Index r = -1, c = -1;
                    for (Eigen::Index i = 0; i < J.rows() && r < 0; ++i)
                        for (Eigen::Index k = 0; k < J.cols(); ++k)
                            if (!std::isfinite(J(i, k))) { r = i; c = k; break; }
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                         "External wrench: Jacobian J contains NaN/Inf (frame=%s fid=%d at [%ld,%ld]), skip publishing.",
                                         info.eeFrame.c_str(), static_cast<int>(fid),
                                         static_cast<long>(r), static_cast<long>(c));
                    return;
                }
                Eigen::Matrix<double, 6, 1> w = dampedWrenchFromTorque(J, tau_ext, lambda);
                if (!w.allFinite())
                {
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                         "External wrench: solved wrench w contains NaN/Inf, skip publishing.");
                    return;
                }
                w = filterWrench(w, filtered_single_, alpha);
                geometry_msgs::msg::WrenchStamped msg;
                fillWrenchMsg(msg, time, frame_id, w);
                publisher_single_->publish(msg);
            }
            else
            {
                if (!publisher_left_ || !publisher_right_)
                {
                    return;
                }
                const size_t arm_dim = info.armDim;
                if (arm_dim % 2 != 0 || tau_ext.size() < static_cast<Eigen::Index>(arm_dim) ||
                    static_cast<size_t>(joint_positions.size()) < info.stateDim)
                {
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                         "External wrench: dual-arm armDim/state layout not supported.");
                    return;
                }
                const size_t arm_start = info.stateDim - arm_dim;
                const size_t half = arm_dim / 2;

                // Decide which eeFrame corresponds to left/right arm.
                // Convention in ManipulatorModelInfo: eeFrame=right arm, eeFrame1=left arm.
                // But some configs (e.g. m6_ccs) set eeFrame="left_eef", eeFrame1="right_eef".
                std::string left_frame = info.eeFrame1;
                std::string right_frame = info.eeFrame;
                auto lower = [](std::string s) {
                    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::tolower(c); });
                    return s;
                };
                const auto ee0 = lower(info.eeFrame);
                const auto ee1 = lower(info.eeFrame1);
                if (ee0.find("left") != std::string::npos && ee1.find("right") != std::string::npos)
                {
                    left_frame = info.eeFrame;
                    right_frame = info.eeFrame1;
                    static bool logged_swap = false;
                    if (!logged_swap)
                    {
                        RCLCPP_WARN(node_->get_logger(),
                                    "External wrench: detected swapped ee frames (eeFrame='%s', eeFrame1='%s'). "
                                    "Using left_frame='%s', right_frame='%s'.",
                                    info.eeFrame.c_str(), info.eeFrame1.c_str(),
                                    left_frame.c_str(), right_frame.c_str());
                        logged_swap = true;
                    }
                }

                const pinocchio::FrameIndex fid_left = model.getFrameId(left_frame);
                const auto& oMf_left = data.oMf[fid_left];
                if (!oMf_left.translation().allFinite() || !oMf_left.rotation().allFinite())
                {
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                         "External wrench: frame placement for left frame contains NaN/Inf, fid=%d, frame=%s. skip.",
                                         static_cast<int>(fid_left), left_frame.c_str());
                    return;
                }
                Eigen::MatrixXd Jl(6, model.nv);
                Jl.setZero();
                pinocchio::getFrameJacobian(model, data, fid_left, rf, Jl);
                Eigen::MatrixXd Jl_sub =
                    Jl.block(0, static_cast<Eigen::Index>(arm_start), 6, static_cast<Eigen::Index>(half));
                if (!Jl_sub.allFinite())
                {
                    Eigen::Index r = -1, c = -1;
                    for (Eigen::Index i = 0; i < Jl_sub.rows() && r < 0; ++i)
                        for (Eigen::Index k = 0; k < Jl_sub.cols(); ++k)
                            if (!std::isfinite(Jl_sub(i, k))) { r = i; c = k; break; }
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                         "External wrench: left Jacobian contains NaN/Inf (frame=%s fid=%d at [%ld,%ld]), skip publishing.",
                                         left_frame.c_str(), static_cast<int>(fid_left),
                                         static_cast<long>(r), static_cast<long>(c));
                    return;
                }
                Eigen::VectorXd tau_l =
                    tau_ext.segment(static_cast<Eigen::Index>(arm_start), static_cast<Eigen::Index>(half));
                Eigen::Matrix<double, 6, 1> w_l = dampedWrenchFromTorque(Jl_sub, tau_l, lambda);
                if (!w_l.allFinite())
                {
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                         "External wrench: left solved wrench w_l contains NaN/Inf, skip publishing.");
                    return;
                }
                w_l = filterWrench(w_l, filtered_left_, alpha);

                const pinocchio::FrameIndex fid_right = model.getFrameId(right_frame);
                const auto& oMf_right = data.oMf[fid_right];
                if (!oMf_right.translation().allFinite() || !oMf_right.rotation().allFinite())
                {
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                         "External wrench: frame placement for right frame contains NaN/Inf, fid=%d, frame=%s. skip.",
                                         static_cast<int>(fid_right), right_frame.c_str());
                    return;
                }
                Eigen::MatrixXd Jr(6, model.nv);
                Jr.setZero();
                pinocchio::getFrameJacobian(model, data, fid_right, rf, Jr);
                Eigen::MatrixXd Jr_sub = Jr.block(0, static_cast<Eigen::Index>(arm_start + half), 6,
                                                  static_cast<Eigen::Index>(half));
                if (!Jr_sub.allFinite())
                {
                    Eigen::Index r = -1, c = -1;
                    for (Eigen::Index i = 0; i < Jr_sub.rows() && r < 0; ++i)
                        for (Eigen::Index k = 0; k < Jr_sub.cols(); ++k)
                            if (!std::isfinite(Jr_sub(i, k))) { r = i; c = k; break; }
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                         "External wrench: right Jacobian contains NaN/Inf (frame=%s fid=%d at [%ld,%ld]), skip publishing.",
                                         right_frame.c_str(), static_cast<int>(fid_right),
                                         static_cast<long>(r), static_cast<long>(c));
                    return;
                }
                Eigen::VectorXd tau_r =
                    tau_ext.segment(static_cast<Eigen::Index>(arm_start + half), static_cast<Eigen::Index>(half));
                Eigen::Matrix<double, 6, 1> w_r = dampedWrenchFromTorque(Jr_sub, tau_r, lambda);
                if (!w_r.allFinite())
                {
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                         "External wrench: right solved wrench w_r contains NaN/Inf, skip publishing.");
                    return;
                }
                w_r = filterWrench(w_r, filtered_right_, alpha);

                geometry_msgs::msg::WrenchStamped msg_l;
                geometry_msgs::msg::WrenchStamped msg_r;
                fillWrenchMsg(msg_l, time, frame_id, w_l);
                fillWrenchMsg(msg_r, time, frame_id, w_r);
                publisher_left_->publish(msg_l);
                publisher_right_->publish(msg_r);
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                 "External wrench: %s", e.what());
        }
    }
} // namespace ocs2::mobile_manipulator
