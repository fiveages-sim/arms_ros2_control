#include "arms_controller_common/utils/BimanualSelfCollisionChecker.h"

#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/collision/distance.hpp>

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace arms_controller_common
{

BimanualSelfCollisionChecker::BimanualSelfCollisionChecker(
    std::shared_ptr<const BimanualClosedChainConstraint> constraint,
    pinocchio::GeometryModel geometry_model,
    double minimum_allowed_distance)
    : constraint_(std::move(constraint)),
      model_(constraint_ ? constraint_->model() : pinocchio::Model()),
      data_(model_),
      geometry_model_(std::move(geometry_model)),
      geometry_data_(geometry_model_),
      minimum_allowed_distance_(minimum_allowed_distance)
{
    if (!constraint_)
    {
        throw std::invalid_argument(
            "BimanualSelfCollisionChecker requires a non-null constraint");
    }
    if (!std::isfinite(minimum_allowed_distance_) ||
        minimum_allowed_distance_ < 0.0)
    {
        throw std::invalid_argument(
            "minimum_allowed_distance must be finite and non-negative");
    }
    if (geometry_model_.collisionPairs.empty())
    {
        throw std::invalid_argument(
            "GeometryModel contains no configured collision pairs");
    }

    for (const auto& object : geometry_model_.geometryObjects)
    {
        const bool has_parent_frame =
            object.parentFrame !=
            std::numeric_limits<pinocchio::FrameIndex>::max();
        if (object.parentJoint >=
                static_cast<pinocchio::JointIndex>(model_.njoints) ||
            (has_parent_frame &&
             object.parentFrame >=
                 static_cast<pinocchio::FrameIndex>(model_.nframes)))
        {
            throw std::invalid_argument(
                "GeometryModel is incompatible with the Pinocchio model");
        }
    }
}

BimanualSelfCollisionChecker::Result
BimanualSelfCollisionChecker::check(
    const Eigen::VectorXd& ambient_state) const
{
    Result result;
    try
    {
        const Eigen::VectorXd q =
            constraint_->fullConfiguration(ambient_state);
        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::updateGeometryPlacements(
            model_, data_, geometry_model_, geometry_data_);
        pinocchio::computeDistances(geometry_model_, geometry_data_);

        for (std::size_t i = 0;
             i < geometry_model_.collisionPairs.size(); ++i)
        {
            if (!geometry_data_.activeCollisionPairs[i])
            {
                continue;
            }
            const double distance = geometry_data_.distanceResults[i].min_distance;
            if (std::isfinite(distance) && distance < result.minimum_distance)
            {
                result.minimum_distance = distance;
                result.minimum_pair_index = i;
            }
        }

        if (result.minimum_pair_index ==
            std::numeric_limits<std::size_t>::max())
        {
            result.message =
                "No active collision pair produced a finite distance";
            return result;
        }

        const auto& pair =
            geometry_model_.collisionPairs[result.minimum_pair_index];
        result.first_object = geometry_model_.geometryObjects[pair.first].name;
        result.second_object = geometry_model_.geometryObjects[pair.second].name;
        result.query_valid = true;
        result.collision_free =
            result.minimum_distance > minimum_allowed_distance_;

        std::ostringstream stream;
        stream << "minimum distance " << result.minimum_distance
               << " m for pair '" << result.first_object << "' / '"
               << result.second_object << "' (required > "
               << minimum_allowed_distance_ << " m)";
        result.message = stream.str();
        return result;
    }
    catch (const std::exception& error)
    {
        result.message = std::string("Collision query failed: ") + error.what();
        return result;
    }
}

bool BimanualSelfCollisionChecker::isCollisionFree(
    const Eigen::VectorXd& ambient_state) const
{
    const Result result = check(ambient_state);
    return result.query_valid && result.collision_free;
}

}  // namespace arms_controller_common
