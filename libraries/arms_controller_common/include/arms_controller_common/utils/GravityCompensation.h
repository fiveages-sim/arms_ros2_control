//
// Gravity Compensation Utility using Pinocchio
//
#pragma once

#include <string>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace arms_controller_common
{
    /**
     * @brief Gravity compensation utility class
     * 
     * Calculates static torques (gravity compensation) for a robot arm
     * using Pinocchio library and URDF model.
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
         * @brief Calculate static torques for given joint positions
         * @param joint_positions Current joint positions (radians)
         * @return Static torques (gravity compensation) for each joint
         */
        std::vector<double> calculateStaticTorques(const std::vector<double>& joint_positions) const;

        /**
         * @brief Calculate static torques using Eigen vector
         * @param joint_positions Current joint positions as Eigen vector
         * @return Static torques as Eigen vector
         */
        Eigen::VectorXd calculateStaticTorquesEigen(const Eigen::VectorXd& joint_positions) const;

        /**
         * @brief Get number of joints in the model
         * @return Number of joints
         */
        size_t getNumJoints() const { return model_.nq; }

        /**
         * @brief Check if model is loaded
         * @return True if model is valid
         */
        bool isValid() const { return model_.nq > 0; }

    private:
        pinocchio::Model model_;
        mutable pinocchio::Data data_;  // Mutable for const methods
    };
} // namespace arms_controller_common

