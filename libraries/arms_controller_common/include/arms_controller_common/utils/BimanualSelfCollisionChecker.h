#pragma once

#include "arms_controller_common/utils/BimanualClosedChainConstraint.h"

#include <Eigen/Core>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

#include <cstddef>
#include <limits>
#include <memory>
#include <string>

namespace arms_controller_common
{

/**
 * @brief Offline distance checker for configured robot self-collision pairs.
 *
 * The GeometryModel must have been built for the same Pinocchio model as the
 * closed-chain constraint and must already contain the desired collisionPairs.
 * In the controller this model is copied from OCS2's
 * PinocchioGeometryInterface, preserving the task-file collision-pair filter.
 *
 * This object owns independent Pinocchio Data and GeometryData caches, so a
 * query never changes the robot observation or visualization cache. One checker
 * instance must not be called concurrently from several threads.
 */
class BimanualSelfCollisionChecker final
{
public:
    struct Result
    {
        bool query_valid = false;
        bool collision_free = false;
        double minimum_distance = std::numeric_limits<double>::infinity();
        std::size_t minimum_pair_index =
            std::numeric_limits<std::size_t>::max();
        std::string first_object;
        std::string second_object;
        std::string message;
    };

    BimanualSelfCollisionChecker(
        std::shared_ptr<const BimanualClosedChainConstraint> constraint,
        pinocchio::GeometryModel geometry_model,
        double minimum_allowed_distance);

    Result check(const Eigen::VectorXd& ambient_state) const;
    bool isCollisionFree(const Eigen::VectorXd& ambient_state) const;

    std::size_t collisionPairCount() const noexcept
    {
        return geometry_model_.collisionPairs.size();
    }
    double minimumAllowedDistance() const noexcept
    {
        return minimum_allowed_distance_;
    }
    const BimanualClosedChainConstraint& constraint() const noexcept
    {
        return *constraint_;
    }

private:
    std::shared_ptr<const BimanualClosedChainConstraint> constraint_;
    pinocchio::Model model_;
    mutable pinocchio::Data data_;
    pinocchio::GeometryModel geometry_model_;
    mutable pinocchio::GeometryData geometry_data_;
    double minimum_allowed_distance_{};
};

}  // namespace arms_controller_common
