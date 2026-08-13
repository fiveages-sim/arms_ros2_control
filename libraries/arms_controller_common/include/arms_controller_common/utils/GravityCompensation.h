//
// Gravity Compensation Utility using Pinocchio
//
#pragma once

#include <string>
#include <vector>
#include <memory>
#include <Eigen/Dense>
// Disable maybe-uninitialized warning for Pinocchio library headers
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#pragma GCC diagnostic pop

namespace arms_controller_common
{
    /**
     * @brief Gravity compensation utility class
     * 
     * Calculates static torques (gravity compensation) for a robot arm
     * using Pinocchio library and URDF model.
     * 
     * Supports two construction modes:
     * 1. From URDF file path (for basic_joint_controller)
     * 2. From existing Pinocchio model (for ocs2_arm_controller, avoids duplicate loading)
     */
    class GravityCompensation
    {
    public:
        /**
         * @brief Constructor - loads robot model from URDF file
         * @param urdf_path Path to URDF file
         */
        explicit GravityCompensation(const std::string& urdf_path);

        /**
         * @brief Constructor - uses existing Pinocchio model (for OCS2 controllers)
         * @param model Reference to existing Pinocchio model (typically from OCS2 interface)
         */
        explicit GravityCompensation(const pinocchio::Model& model);

        /**
         * @brief Calculate static torques for given joint positions
         * @param joint_positions Controlled-joint positions (same order as HW / OCS2 arm segment).
         *        When the Pinocchio model has a virtual mobile base prefix (e.g. XY-yaw, nq = 3 + arm),
         *        pass only the arm joints; base q is padded with zeros and only arm τ is returned.
         * @return Static torques matching joint_positions size (not full model nq)
         */
        std::vector<double> calculateStaticTorques(const std::vector<double>& joint_positions) const;

        /**
         * @brief Calculate static torques using Eigen vector
         * @param joint_positions Controlled-joint positions (see calculateStaticTorques)
         * @return Static torques matching joint_positions size
         */
        Eigen::VectorXd calculateStaticTorquesEigen(const Eigen::VectorXd& joint_positions) const;

        /**
         * @brief Get number of configuration DOFs in the Pinocchio model (may include virtual base)
         */
        size_t getNumJoints() const { return model_.nq; }

        /**
         * @brief Check if model is loaded
         * @return True if model is valid
         */
        bool isValid() const { return model_.nq > 0; }

    private:
        /** Build full q, run RNEA, return τ for the controlled joints only. */
        Eigen::VectorXd computeStaticTorques(const Eigen::VectorXd& joint_positions) const;

        pinocchio::Model model_;
        mutable pinocchio::Data data_;  // Mutable for const methods
        mutable bool logged_base_pad_ = false;
    };
} // namespace arms_controller_common

