#pragma once

#include "arms_controller_common/utils/BimanualClosedChainConstraint.h"
#include "arms_controller_common/utils/BimanualSelfCollisionChecker.h"

#include <Eigen/Core>

#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace arms_controller_common
{

/**
 * @brief Closed-chain planner with optional configured-pair self-collision checks.
 *
 * When a collision checker is supplied, start states, goal candidates, sampled
 * OMPL states, and returned waypoints are all checked against its GeometryModel.
 * Without a checker the planner remains useful for offline constraint tests, but
 * reports collision_checked == false and its result must not be executed.
 */
class BimanualProjectedPlanner
{
public:
    enum class Status
    {
        SUCCESS,
        INVALID_INPUT,
        START_PROJECTION_FAILED,
        START_IN_COLLISION,
        COLLISION_CHECK_FAILED,
        NO_VALID_GOALS,
        TIMEOUT,
        PLANNING_FAILED,
        APPROXIMATE_SOLUTION,
        INVALID_SOLUTION
    };

    struct Options
    {
        double planning_time_seconds = 5.0;
        double delta = 0.05;
        double lambda = 2.0;
        unsigned int projection_max_iterations = 100;
        double planner_range = 0.0;
        double max_start_projection_distance = 0.25;
    };

    struct Result
    {
        Status status = Status::INVALID_INPUT;
        std::string message;
        double planning_time_seconds = 0.0;
        bool collision_checked = false;

        Eigen::VectorXd projected_start;
        std::vector<Eigen::VectorXd> path;
        // Indices in the dense path corresponding to the original RRTConnect
        // milestones. A time parameterizer may stop only at these indices.
        std::vector<std::size_t> milestone_indices;
        std::vector<std::size_t> accepted_goal_indices;
        std::vector<std::size_t> rejected_goal_indices;
        std::vector<std::size_t> collision_rejected_goal_indices;
        std::size_t collision_check_count = 0;
        double minimum_collision_distance =
            std::numeric_limits<double>::infinity();
        std::size_t selected_goal_index =
            std::numeric_limits<std::size_t>::max();

        bool success() const noexcept { return status == Status::SUCCESS; }
    };

    explicit BimanualProjectedPlanner(
        std::shared_ptr<BimanualClosedChainConstraint> constraint);

    BimanualProjectedPlanner(
        std::shared_ptr<BimanualClosedChainConstraint> constraint,
        std::shared_ptr<BimanualSelfCollisionChecker> collision_checker);

    Result plan(const Eigen::VectorXd& start,
                const std::vector<Eigen::VectorXd>& goal_candidates) const;

    Result plan(const Eigen::VectorXd& start,
                const std::vector<Eigen::VectorXd>& goal_candidates,
                const Options& options) const;

    static const char* statusName(Status status) noexcept;

private:
    bool isWithinBounds(const Eigen::VectorXd& state) const;
    bool validateOptions(const Options& options, std::string& error) const;

    std::shared_ptr<BimanualClosedChainConstraint> constraint_;
    std::shared_ptr<BimanualSelfCollisionChecker> collision_checker_;
};

}  // namespace arms_controller_common
