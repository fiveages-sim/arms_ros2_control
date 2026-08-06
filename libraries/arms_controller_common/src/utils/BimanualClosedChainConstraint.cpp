#include "arms_controller_common/utils/BimanualClosedChainConstraint.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <unordered_set>
#include <utility>

namespace arms_controller_common
{

namespace
{

std::string dimensionMessage(const char* value_name, Eigen::Index expected,
                             Eigen::Index actual)
{
    std::ostringstream stream;
    stream << value_name << " dimension mismatch: expected " << expected
           << ", got " << actual;
    return stream.str();
}

}  // namespace

BimanualClosedChainConstraint::BimanualClosedChainConstraint(
    const pinocchio::Model& model,
    std::vector<std::string> left_joint_names,
    std::vector<std::string> right_joint_names,
    std::string left_end_effector_frame,
    std::string right_end_effector_frame,
    const Eigen::VectorXd& reference_configuration,
    const pinocchio::SE3& desired_left_to_right,
    double tolerance)
    : ompl::base::Constraint(
          static_cast<unsigned int>(left_joint_names.size() +
                                    right_joint_names.size()),
          kConstraintDimension, tolerance),
      model_(model),
      data_(model_),
      left_joint_names_(std::move(left_joint_names)),
      right_joint_names_(std::move(right_joint_names)),
      reference_configuration_(reference_configuration),
      desired_left_to_right_(desired_left_to_right)
{
    if (left_joint_names_.empty() || right_joint_names_.empty())
    {
        throw std::invalid_argument(
            "Both arms must contain at least one joint");
    }
    if (left_joint_names_.size() + right_joint_names_.size() <=
        kConstraintDimension)
    {
        throw std::invalid_argument(
            "Closed-chain ambient dimension must be greater than six");
    }
    if (reference_configuration_.size() != model_.nq)
    {
        throw std::invalid_argument(dimensionMessage(
            "Reference configuration", model_.nq,
            reference_configuration_.size()));
    }
    if (!reference_configuration_.allFinite())
    {
        throw std::invalid_argument(
            "Reference configuration contains non-finite values");
    }
    if (!desired_left_to_right_.isNormalized())
    {
        throw std::invalid_argument(
            "Desired relative pose does not contain a normalized rotation");
    }

    if (!model_.existFrame(left_end_effector_frame))
    {
        throw std::invalid_argument("Left end-effector frame not found: " +
                                    left_end_effector_frame);
    }
    if (!model_.existFrame(right_end_effector_frame))
    {
        throw std::invalid_argument("Right end-effector frame not found: " +
                                    right_end_effector_frame);
    }
    left_frame_id_ = model_.getFrameId(left_end_effector_frame);
    right_frame_id_ = model_.getFrameId(right_end_effector_frame);

    left_coordinates_ = resolveJointCoordinates(left_joint_names_);
    right_coordinates_ = resolveJointCoordinates(right_joint_names_);
    ambient_coordinates_.reserve(left_coordinates_.size() +
                                 right_coordinates_.size());
    ambient_coordinates_.insert(ambient_coordinates_.end(),
                                left_coordinates_.begin(),
                                left_coordinates_.end());
    ambient_coordinates_.insert(ambient_coordinates_.end(),
                                right_coordinates_.begin(),
                                right_coordinates_.end());

    std::unordered_set<pinocchio::JointIndex> unique_joint_ids;
    for (const auto& coordinate : ambient_coordinates_)
    {
        if (!unique_joint_ids.insert(coordinate.joint_id).second)
        {
            throw std::invalid_argument(
                "A joint is listed more than once in the dual-arm state: " +
                model_.names[coordinate.joint_id]);
        }
    }

    extractBounds();
}

void BimanualClosedChainConstraint::function(
    const Eigen::Ref<const Eigen::VectorXd>& x,
    Eigen::Ref<Eigen::VectorXd> out) const
{
    validateAmbientState(x);
    if (out.size() != static_cast<Eigen::Index>(kConstraintDimension))
    {
        throw std::invalid_argument(dimensionMessage(
            "Constraint output", kConstraintDimension, out.size()));
    }

    updateKinematics(x);
    const pinocchio::SE3 left_to_right =
        data_.oMf[left_frame_id_].actInv(data_.oMf[right_frame_id_]);
    const pinocchio::SE3 error_transform =
        desired_left_to_right_.actInv(left_to_right);
    out = pinocchio::log6(error_transform).toVector();
}

void BimanualClosedChainConstraint::jacobian(
    const Eigen::Ref<const Eigen::VectorXd>& x,
    Eigen::Ref<Eigen::MatrixXd> out) const
{
    validateAmbientState(x);
    if (out.rows() != static_cast<Eigen::Index>(kConstraintDimension) ||
        out.cols() != static_cast<Eigen::Index>(getAmbientDimension()))
    {
        std::ostringstream stream;
        stream << "Constraint Jacobian dimension mismatch: expected "
               << kConstraintDimension << "x" << getAmbientDimension()
               << ", got " << out.rows() << "x" << out.cols();
        throw std::invalid_argument(stream.str());
    }

    updateKinematics(x);

    Eigen::MatrixXd left_full_jacobian(6, model_.nv);
    Eigen::MatrixXd right_full_jacobian(6, model_.nv);
    left_full_jacobian.setZero();
    right_full_jacobian.setZero();
    pinocchio::getFrameJacobian(model_, data_, left_frame_id_,
                                pinocchio::LOCAL, left_full_jacobian);
    pinocchio::getFrameJacobian(model_, data_, right_frame_id_,
                                pinocchio::LOCAL, right_full_jacobian);

    const Eigen::MatrixXd left_jacobian =
        selectAmbientColumns(left_full_jacobian);
    const Eigen::MatrixXd right_jacobian =
        selectAmbientColumns(right_full_jacobian);

    const pinocchio::SE3 left_to_right =
        data_.oMf[left_frame_id_].actInv(data_.oMf[right_frame_id_]);
    const pinocchio::SE3 error_transform =
        desired_left_to_right_.actInv(left_to_right);

    // If V_l and V_r are local (body) frame velocities, the local velocity
    // of T_l^-1 T_r is V_r - Ad_(T_l^-1 T_r)^-1 V_l. Jlog6 maps this local
    // tangent into the derivative of log6(error_transform).
    const Eigen::MatrixXd relative_body_jacobian =
        right_jacobian - left_to_right.toActionMatrixInverse() * left_jacobian;
    out = pinocchio::Jlog6(error_transform) * relative_body_jacobian;
}

pinocchio::SE3 BimanualClosedChainConstraint::relativePose(
    const Eigen::Ref<const Eigen::VectorXd>& x) const
{
    validateAmbientState(x);
    updateKinematics(x);
    return data_.oMf[left_frame_id_].actInv(data_.oMf[right_frame_id_]);
}

Eigen::VectorXd BimanualClosedChainConstraint::fullConfiguration(
    const Eigen::Ref<const Eigen::VectorXd>& x) const
{
    validateAmbientState(x);
    Eigen::VectorXd q = reference_configuration_;
    for (Eigen::Index i = 0; i < x.size(); ++i)
    {
        q[ambient_coordinates_[static_cast<size_t>(i)].q_index] = x[i];
    }
    return q;
}

std::vector<BimanualClosedChainConstraint::JointCoordinate>
BimanualClosedChainConstraint::resolveJointCoordinates(
    const std::vector<std::string>& joint_names) const
{
    std::vector<JointCoordinate> coordinates;
    coordinates.reserve(joint_names.size());
    for (const auto& name : joint_names)
    {
        if (!model_.existJointName(name))
        {
            throw std::invalid_argument("Joint not found in Pinocchio model: " +
                                        name);
        }
        const pinocchio::JointIndex joint_id = model_.getJointId(name);
        const auto& joint = model_.joints[joint_id];
        if (joint.nq() != 1 || joint.nv() != 1)
        {
            std::ostringstream stream;
            stream << "Joint '" << name
                   << "' is not a scalar joint (nq=" << joint.nq()
                   << ", nv=" << joint.nv() << ")";
            throw std::invalid_argument(stream.str());
        }
        coordinates.push_back(
            {joint_id, static_cast<Eigen::Index>(joint.idx_q()),
             static_cast<Eigen::Index>(joint.idx_v())});
    }
    return coordinates;
}

void BimanualClosedChainConstraint::validateAmbientState(
    const Eigen::Ref<const Eigen::VectorXd>& x) const
{
    if (x.size() != static_cast<Eigen::Index>(getAmbientDimension()))
    {
        throw std::invalid_argument(dimensionMessage(
            "Ambient state", getAmbientDimension(), x.size()));
    }
    if (!x.allFinite())
    {
        throw std::invalid_argument("Ambient state contains non-finite values");
    }
}

void BimanualClosedChainConstraint::updateKinematics(
    const Eigen::Ref<const Eigen::VectorXd>& x) const
{
    const Eigen::VectorXd q = fullConfiguration(x);
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::computeJointJacobians(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
}

Eigen::MatrixXd BimanualClosedChainConstraint::selectAmbientColumns(
    const Eigen::MatrixXd& full_jacobian) const
{
    Eigen::MatrixXd selected(full_jacobian.rows(),
                             static_cast<Eigen::Index>(ambient_coordinates_.size()));
    for (size_t i = 0; i < ambient_coordinates_.size(); ++i)
    {
        selected.col(static_cast<Eigen::Index>(i)) =
            full_jacobian.col(ambient_coordinates_[i].v_index);
    }
    return selected;
}

void BimanualClosedChainConstraint::extractBounds()
{
    lower_bounds_.resize(static_cast<Eigen::Index>(ambient_coordinates_.size()));
    upper_bounds_.resize(static_cast<Eigen::Index>(ambient_coordinates_.size()));
    for (size_t i = 0; i < ambient_coordinates_.size(); ++i)
    {
        const Eigen::Index q_index = ambient_coordinates_[i].q_index;
        const double lower = model_.lowerPositionLimit[q_index];
        const double upper = model_.upperPositionLimit[q_index];
        if (!std::isfinite(lower) || !std::isfinite(upper) || lower >= upper)
        {
            throw std::invalid_argument(
                "Joint limits must be finite and ordered for joint: " +
                model_.names[ambient_coordinates_[i].joint_id]);
        }
        lower_bounds_[static_cast<Eigen::Index>(i)] = lower;
        upper_bounds_[static_cast<Eigen::Index>(i)] = upper;
    }
}

}  // namespace arms_controller_common
