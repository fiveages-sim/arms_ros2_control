#pragma once

#include <Eigen/Core>

#include <ompl/base/Constraint.h>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial.hpp>

#include <string>
#include <vector>

namespace arms_controller_common
{

/**
 * @brief Six-dimensional rigid grasp constraint for a pair of robot arms.
 *
 * The ambient state is ordered as [left arm joints, right arm joints]. All
 * joints named in the constructor must be scalar joints (nq == nv == 1).
 * Joints outside the two arms remain fixed at reference_configuration.
 *
 * The constraint is
 *
 *   f(q) = log6(T_lr_desired^{-1} * T_left(q)^{-1} * T_right(q)) = 0.
 *
 * It derives from OMPL's Constraint so the same implementation is used by
 * ProjectedStateSpace and, later, AtlasStateSpace.
 *
 * A constraint object owns its Pinocchio Data cache. Individual instances are
 * independent, but one instance must not be evaluated concurrently by several
 * threads because Pinocchio updates that cache during each evaluation.
 */
class BimanualClosedChainConstraint final : public ompl::base::Constraint
{
public:
    static constexpr unsigned int kConstraintDimension = 6;

    BimanualClosedChainConstraint(
        const pinocchio::Model& model,
        std::vector<std::string> left_joint_names,
        std::vector<std::string> right_joint_names,
        std::string left_end_effector_frame,
        std::string right_end_effector_frame,
        const Eigen::VectorXd& reference_configuration,
        const pinocchio::SE3& desired_left_to_right,
        double tolerance = 1e-4);

    /** Evaluate the six-dimensional SE(3) logarithm error. */
    void function(const Eigen::Ref<const Eigen::VectorXd>& x,
                  Eigen::Ref<Eigen::VectorXd> out) const override;

    /** Evaluate the analytic 6 x ambient_dimension constraint Jacobian. */
    void jacobian(const Eigen::Ref<const Eigen::VectorXd>& x,
                  Eigen::Ref<Eigen::MatrixXd> out) const override;

    /** Return T_left(q)^-1 * T_right(q). */
    pinocchio::SE3 relativePose(
        const Eigen::Ref<const Eigen::VectorXd>& x) const;

    /** Expand the ambient dual-arm state into the full Pinocchio q vector. */
    Eigen::VectorXd fullConfiguration(
        const Eigen::Ref<const Eigen::VectorXd>& x) const;

    const Eigen::VectorXd& lowerBounds() const noexcept { return lower_bounds_; }
    const Eigen::VectorXd& upperBounds() const noexcept { return upper_bounds_; }
    const pinocchio::SE3& desiredRelativePose() const noexcept
    {
        return desired_left_to_right_;
    }
    const pinocchio::Model& model() const noexcept { return model_; }

    const std::vector<std::string>& leftJointNames() const noexcept
    {
        return left_joint_names_;
    }
    const std::vector<std::string>& rightJointNames() const noexcept
    {
        return right_joint_names_;
    }

private:
    struct JointCoordinate
    {
        pinocchio::JointIndex joint_id{};
        Eigen::Index q_index{};
        Eigen::Index v_index{};
    };

    std::vector<JointCoordinate> resolveJointCoordinates(
        const std::vector<std::string>& joint_names) const;
    void validateAmbientState(const Eigen::Ref<const Eigen::VectorXd>& x) const;
    void updateKinematics(const Eigen::Ref<const Eigen::VectorXd>& x) const;
    Eigen::MatrixXd selectAmbientColumns(const Eigen::MatrixXd& full_jacobian) const;
    void extractBounds();

    pinocchio::Model model_;
    mutable pinocchio::Data data_;

    std::vector<std::string> left_joint_names_;
    std::vector<std::string> right_joint_names_;
    std::vector<JointCoordinate> left_coordinates_;
    std::vector<JointCoordinate> right_coordinates_;
    std::vector<JointCoordinate> ambient_coordinates_;

    pinocchio::FrameIndex left_frame_id_{};
    pinocchio::FrameIndex right_frame_id_{};
    Eigen::VectorXd reference_configuration_;
    pinocchio::SE3 desired_left_to_right_;
    Eigen::VectorXd lower_bounds_;
    Eigen::VectorXd upper_bounds_;
};

}  // namespace arms_controller_common
