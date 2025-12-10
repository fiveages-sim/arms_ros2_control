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
                       "Loaded robot model from %s: %zu joints", 
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
                   "Initialized gravity compensation from existing Pinocchio model: %zu joints", 
                   model_.nq);
    }

    std::vector<double> GravityCompensation::calculateStaticTorques(
        const std::vector<double>& joint_positions) const
    {
        if (joint_positions.size() != static_cast<size_t>(model_.nq))
        {
            RCLCPP_WARN(rclcpp::get_logger("GravityCompensation"),
                       "Joint positions size (%zu) doesn't match model size (%zu)",
                       joint_positions.size(), static_cast<size_t>(model_.nq));
            return std::vector<double>(model_.nq, 0.0);
        }

        // Convert to Eigen vector
        Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(
            joint_positions.data(), joint_positions.size());

        // Calculate static torques using RNEA with zero velocity and acceleration
        Eigen::VectorXd tau = pinocchio::rnea(
            model_, data_, q,
            Eigen::VectorXd::Zero(model_.nv),  // Zero velocity
            Eigen::VectorXd::Zero(model_.nv)  // Zero acceleration
        );

        // Convert back to std::vector
        std::vector<double> result(tau.data(), tau.data() + tau.size());
        return result;
    }

    Eigen::VectorXd GravityCompensation::calculateStaticTorquesEigen(
        const Eigen::VectorXd& joint_positions) const
    {
        if (joint_positions.size() != static_cast<size_t>(model_.nq))
        {
            RCLCPP_WARN(rclcpp::get_logger("GravityCompensation"),
                       "Joint positions size (%zu) doesn't match model size (%zu)",
                       joint_positions.size(), static_cast<size_t>(model_.nq));
            return Eigen::VectorXd::Zero(model_.nq);
        }

        // Calculate static torques using RNEA with zero velocity and acceleration
        return pinocchio::rnea(
            model_, data_, joint_positions,
            Eigen::VectorXd::Zero(model_.nv),  // Zero velocity
            Eigen::VectorXd::Zero(model_.nv)  // Zero acceleration
        );
    }
} // namespace arms_controller_common

