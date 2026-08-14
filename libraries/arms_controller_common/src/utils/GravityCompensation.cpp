//
// Gravity Compensation Utility Implementation
//
#include "arms_controller_common/utils/GravityCompensation.h"
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

namespace arms_controller_common
{
    GravityCompensation::GravityCompensation(const std::string& urdf_path)
    {
        try
        {
            // Load model from URDF file
            pinocchio::urdf::buildModel(urdf_path, model_);
            data_ = pinocchio::Data(model_);
            
            RCLCPP_INFO(rclcpp::get_logger("GravityCompensation"),
                       "Loaded robot model from %s: %d joints",
                       urdf_path.c_str(), model_.nq);
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("GravityCompensation"),
                        "Failed to load URDF from %s: %s", urdf_path.c_str(), e.what());
            throw;
        }
    }

    GravityCompensation::GravityCompensation(const pinocchio::Model& model)
        : model_(model)
    {
        // Create data from the provided model
        data_ = pinocchio::Data(model_);
        
        RCLCPP_INFO(rclcpp::get_logger("GravityCompensation"),
                   "Initialized gravity compensation from existing Pinocchio model: %d joints",
                   model_.nq);
    }

    Eigen::VectorXd GravityCompensation::computeStaticTorques(
        const Eigen::VectorXd& joint_positions) const
    {
        const auto nq = static_cast<Eigen::Index>(model_.nq);
        const auto n_ctrl = joint_positions.size();

        if (n_ctrl <= 0 || nq <= 0)
        {
            return Eigen::VectorXd::Zero(std::max<Eigen::Index>(n_ctrl, 0));
        }

        // Exact match: fixed-base / URDF-only models.
        if (n_ctrl == nq)
        {
            return pinocchio::rnea(
                model_, data_, joint_positions,
                Eigen::VectorXd::Zero(model_.nv),
                Eigen::VectorXd::Zero(model_.nv));
        }

        // OCS2 WheelBasedMobileManipulator (and similar): virtual base DOFs are a
        // prefix of Pinocchio q; HW / controller joints are the suffix (armDim).
        // Pad base q with zeros (identity pose at origin) and return arm τ only.
        if (n_ctrl < nq)
        {
            if (!logged_base_pad_)
            {
                logged_base_pad_ = true;
                RCLCPP_INFO(rclcpp::get_logger("GravityCompensation"),
                            "Controlled joints (%ld) < Pinocchio nq (%ld): treating first %ld DOF as "
                            "virtual base (pad q=0), returning gravity τ for the last %ld joints",
                            static_cast<long>(n_ctrl), static_cast<long>(nq),
                            static_cast<long>(nq - n_ctrl), static_cast<long>(n_ctrl));
            }

            Eigen::VectorXd q = Eigen::VectorXd::Zero(nq);
            q.tail(n_ctrl) = joint_positions;

            const Eigen::VectorXd tau = pinocchio::rnea(
                model_, data_, q,
                Eigen::VectorXd::Zero(model_.nv),
                Eigen::VectorXd::Zero(model_.nv));
            return tau.tail(n_ctrl);
        }

        RCLCPP_WARN(rclcpp::get_logger("GravityCompensation"),
                    "Joint positions size (%ld) is larger than model nq (%ld); returning zeros",
                    static_cast<long>(n_ctrl), static_cast<long>(nq));
        return Eigen::VectorXd::Zero(n_ctrl);
    }

    std::vector<double> GravityCompensation::calculateStaticTorques(
        const std::vector<double>& joint_positions) const
    {
        const Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(
            joint_positions.data(), static_cast<Eigen::Index>(joint_positions.size()));
        const Eigen::VectorXd tau = computeStaticTorques(q);
        return std::vector<double>(tau.data(), tau.data() + tau.size());
    }

    Eigen::VectorXd GravityCompensation::calculateStaticTorquesEigen(
        const Eigen::VectorXd& joint_positions) const
    {
        return computeStaticTorques(joint_positions);
    }
} // namespace arms_controller_common
