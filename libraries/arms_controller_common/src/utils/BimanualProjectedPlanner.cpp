#include "arms_controller_common/utils/BimanualProjectedPlanner.h"

#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace arms_controller_common
{

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace
{

struct CollisionDiagnostics
{
    bool query_failed = false;
    std::string failure_message;
    std::size_t check_count = 0;
    double minimum_distance = std::numeric_limits<double>::infinity();
};

void mergeCollisionDiagnostics(
    const CollisionDiagnostics& diagnostics,
    BimanualProjectedPlanner::Result& result)
{
    result.collision_check_count += diagnostics.check_count;
    result.minimum_collision_distance = std::min(
        result.minimum_collision_distance, diagnostics.minimum_distance);
}

}  // namespace

BimanualProjectedPlanner::BimanualProjectedPlanner(
    std::shared_ptr<BimanualClosedChainConstraint> constraint)
    : BimanualProjectedPlanner(std::move(constraint), nullptr)
{
}

BimanualProjectedPlanner::BimanualProjectedPlanner(
    std::shared_ptr<BimanualClosedChainConstraint> constraint,
    std::shared_ptr<BimanualSelfCollisionChecker> collision_checker)
    : constraint_(std::move(constraint)),
      collision_checker_(std::move(collision_checker))
{
    if (!constraint_)
    {
        throw std::invalid_argument(
            "BimanualProjectedPlanner requires a non-null constraint");
    }
    if (collision_checker_ &&
        &collision_checker_->constraint() != constraint_.get())
    {
        throw std::invalid_argument(
            "Planner and collision checker must share the same constraint instance");
    }
}

BimanualProjectedPlanner::Result BimanualProjectedPlanner::plan(
    const Eigen::VectorXd& start,
    const std::vector<Eigen::VectorXd>& goal_candidates) const
{
    return plan(start, goal_candidates, Options{});
}

BimanualProjectedPlanner::Result BimanualProjectedPlanner::plan(
    const Eigen::VectorXd& start,
    const std::vector<Eigen::VectorXd>& goal_candidates,
    const Options& options) const
{
    Result result;
    result.collision_checked = static_cast<bool>(collision_checker_);
    std::string option_error;
    if (!validateOptions(options, option_error))
    {
        result.message = option_error;
        return result;
    }

    const Eigen::Index dimension =
        static_cast<Eigen::Index>(constraint_->getAmbientDimension());
    if (start.size() != dimension || !start.allFinite())
    {
        std::ostringstream stream;
        stream << "Invalid start state: expected " << dimension
               << " finite values, got " << start.size();
        result.message = stream.str();
        return result;
    }
    if (!isWithinBounds(start))
    {
        result.message = "Start state violates joint bounds";
        return result;
    }
    if (goal_candidates.empty())
    {
        result.message = "At least one goal candidate is required";
        return result;
    }

    constraint_->setMaxIterations(options.projection_max_iterations);
    result.projected_start = start;
    if (!constraint_->project(result.projected_start))
    {
        result.status = Status::START_PROJECTION_FAILED;
        result.message = "OMPL failed to project the start state onto the closed-chain manifold";
        return result;
    }
    if (!isWithinBounds(result.projected_start))
    {
        result.status = Status::START_PROJECTION_FAILED;
        result.message = "Projected start state violates joint bounds";
        return result;
    }
    const double start_projection_distance =
        (result.projected_start - start).norm();
    if (start_projection_distance > options.max_start_projection_distance)
    {
        result.status = Status::START_PROJECTION_FAILED;
        std::ostringstream stream;
        stream << "Start projection moved " << start_projection_distance
               << " rad in joint-space norm, exceeding limit "
               << options.max_start_projection_distance;
        result.message = stream.str();
        return result;
    }

    if (collision_checker_)
    {
        const auto collision =
            collision_checker_->check(result.projected_start);
        ++result.collision_check_count;
        result.minimum_collision_distance = std::min(
            result.minimum_collision_distance, collision.minimum_distance);
        if (!collision.query_valid)
        {
            result.status = Status::COLLISION_CHECK_FAILED;
            result.message = "Start-state collision query failed: " +
                             collision.message;
            return result;
        }
        if (!collision.collision_free)
        {
            result.status = Status::START_IN_COLLISION;
            result.message = "Projected start state is in collision: " +
                             collision.message;
            return result;
        }
    }

    std::vector<Eigen::VectorXd> accepted_goals;
    accepted_goals.reserve(goal_candidates.size());
    for (std::size_t i = 0; i < goal_candidates.size(); ++i)
    {
        const Eigen::VectorXd& goal = goal_candidates[i];
        if (goal.size() != dimension || !goal.allFinite() ||
            !isWithinBounds(goal) || !constraint_->isSatisfied(goal))
        {
            result.rejected_goal_indices.push_back(i);
            continue;
        }
        if (collision_checker_)
        {
            const auto collision = collision_checker_->check(goal);
            ++result.collision_check_count;
            result.minimum_collision_distance = std::min(
                result.minimum_collision_distance, collision.minimum_distance);
            if (!collision.query_valid)
            {
                result.status = Status::COLLISION_CHECK_FAILED;
                result.message = "Goal-state collision query failed: " +
                                 collision.message;
                return result;
            }
            if (!collision.collision_free)
            {
                result.rejected_goal_indices.push_back(i);
                result.collision_rejected_goal_indices.push_back(i);
                continue;
            }
        }
        accepted_goals.push_back(goal);
        result.accepted_goal_indices.push_back(i);
    }
    if (accepted_goals.empty())
    {
        result.status = Status::NO_VALID_GOALS;
        result.message =
            "No goal candidate satisfies dimensions, joint bounds, the closed-chain constraint, and configured collision limits";
        return result;
    }

    constexpr double kNoOpGoalDistance = 1e-10;
    for (std::size_t i = 0; i < accepted_goals.size(); ++i)
    {
        if ((accepted_goals[i] - result.projected_start).norm() <=
            kNoOpGoalDistance)
        {
            result.path.push_back(result.projected_start);
            result.milestone_indices.push_back(0);
            result.selected_goal_index = result.accepted_goal_indices[i];
            result.status = Status::SUCCESS;
            result.message = collision_checker_
                                 ? "Projected start already matches a collision-checked closed-chain goal"
                                 : "Projected start already matches a closed-chain goal; collision checking was not configured";
            return result;
        }
    }

    auto ambient_space =
        std::make_shared<ob::RealVectorStateSpace>(
            static_cast<unsigned int>(dimension));
    ob::RealVectorBounds bounds(static_cast<unsigned int>(dimension));
    for (Eigen::Index i = 0; i < dimension; ++i)
    {
        bounds.setLow(static_cast<unsigned int>(i), constraint_->lowerBounds()[i]);
        bounds.setHigh(static_cast<unsigned int>(i), constraint_->upperBounds()[i]);
    }
    ambient_space->setBounds(bounds);

    auto constrained_space = std::make_shared<ob::ProjectedStateSpace>(
        ambient_space, constraint_);
    constrained_space->setDelta(options.delta);
    constrained_space->setLambda(options.lambda);
    auto space_information =
        std::make_shared<ob::ConstrainedSpaceInformation>(constrained_space);
    constrained_space->setup();

    og::SimpleSetup setup(space_information);
    auto collision_diagnostics = std::make_shared<CollisionDiagnostics>();
    setup.setStateValidityChecker(
        [space_information, constraint = constraint_,
         collision_checker = collision_checker_,
         collision_diagnostics](const ob::State* state)
        {
            if (!space_information->satisfiesBounds(state) ||
                !constraint->isSatisfied(state))
            {
                return false;
            }
            if (!collision_checker)
            {
                return true;
            }

            const auto* constrained_state =
                state->as<ob::ConstrainedStateSpace::StateType>();
            const Eigen::VectorXd ambient_state = *constrained_state;
            const auto collision = collision_checker->check(ambient_state);
            ++collision_diagnostics->check_count;
            collision_diagnostics->minimum_distance = std::min(
                collision_diagnostics->minimum_distance,
                collision.minimum_distance);
            if (!collision.query_valid)
            {
                collision_diagnostics->query_failed = true;
                collision_diagnostics->failure_message = collision.message;
                return false;
            }
            return collision.collision_free;
        });

    ob::ScopedState<> start_state(constrained_space);
    start_state->as<ob::ConstrainedStateSpace::StateType>()->copy(
        result.projected_start);
    setup.setStartState(start_state);

    auto goals = std::make_shared<ob::GoalStates>(space_information);
    for (const auto& accepted_goal : accepted_goals)
    {
        ob::ScopedState<> goal_state(constrained_space);
        goal_state->as<ob::ConstrainedStateSpace::StateType>()->copy(
            accepted_goal);
        goals->addState(goal_state);
    }
    setup.setGoal(goals);

    auto planner = std::make_shared<og::RRTConnect>(space_information);
    if (options.planner_range > 0.0)
    {
        planner->setRange(options.planner_range);
    }
    setup.setPlanner(planner);
    setup.setup();

    const auto planning_start = std::chrono::steady_clock::now();
    const ob::PlannerTerminationCondition time_limit =
        ob::timedPlannerTerminationCondition(options.planning_time_seconds);
    const ob::PlannerTerminationCondition collision_query_failure(
        [collision_diagnostics]()
        {
            return collision_diagnostics->query_failed;
        });
    const ob::PlannerTerminationCondition termination_condition =
        ob::plannerOrTerminationCondition(
            time_limit, collision_query_failure);
    const ob::PlannerStatus planner_status =
        setup.solve(termination_condition);
    result.planning_time_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - planning_start).count();
    mergeCollisionDiagnostics(*collision_diagnostics, result);
    collision_diagnostics->check_count = 0;
    collision_diagnostics->minimum_distance =
        std::numeric_limits<double>::infinity();

    if (collision_diagnostics->query_failed)
    {
        result.status = Status::COLLISION_CHECK_FAILED;
        result.message = "Collision query failed during OMPL planning: " +
                         collision_diagnostics->failure_message;
        return result;
    }

    if (planner_status == ob::PlannerStatus::APPROXIMATE_SOLUTION)
    {
        result.status = Status::APPROXIMATE_SOLUTION;
        result.message = "RRTConnect returned only an approximate solution";
        return result;
    }
    if (planner_status != ob::PlannerStatus::EXACT_SOLUTION)
    {
        result.status = planner_status == ob::PlannerStatus::TIMEOUT
                            ? Status::TIMEOUT
                            : Status::PLANNING_FAILED;
        result.message = "RRTConnect failed: " + planner_status.asString();
        return result;
    }

    const double simplification_time = std::min(
        0.25, options.planning_time_seconds * 0.1);
    setup.simplifySolution(simplification_time);
    result.planning_time_seconds += setup.getLastSimplificationTime();
    mergeCollisionDiagnostics(*collision_diagnostics, result);
    collision_diagnostics->check_count = 0;
    collision_diagnostics->minimum_distance =
        std::numeric_limits<double>::infinity();
    if (collision_diagnostics->query_failed)
    {
        result.status = Status::COLLISION_CHECK_FAILED;
        result.message = "Collision query failed while simplifying the OMPL path: " +
                         collision_diagnostics->failure_message;
        return result;
    }

    og::PathGeometric& solution_path = setup.getSolutionPath();
    std::vector<Eigen::VectorXd> original_milestones;
    original_milestones.reserve(solution_path.getStateCount());
    for (std::size_t i = 0; i < solution_path.getStateCount(); ++i)
    {
        const auto* state = solution_path.getState(
            static_cast<unsigned int>(i))
                                ->as<ob::ConstrainedStateSpace::StateType>();
        original_milestones.emplace_back(*state);
    }
    // Preserve the constrained geodesic used by OMPL. The time parameterizer
    // later interpolates only between these dense manifold states.
    solution_path.interpolate();
    const bool path_valid = solution_path.getStateCount() > 0 &&
                            solution_path.check();
    mergeCollisionDiagnostics(*collision_diagnostics, result);
    if (collision_diagnostics->query_failed)
    {
        result.status = Status::COLLISION_CHECK_FAILED;
        result.message = "Collision query failed while validating the OMPL path: " +
                         collision_diagnostics->failure_message;
        return result;
    }
    if (!path_valid)
    {
        result.status = Status::INVALID_SOLUTION;
        result.message = "OMPL returned a path that failed constrained-space validation";
        return result;
    }

    result.path.reserve(solution_path.getStateCount());
    for (std::size_t i = 0; i < solution_path.getStateCount(); ++i)
    {
        const auto* state = solution_path.getState(static_cast<unsigned int>(i))
                                ->as<ob::ConstrainedStateSpace::StateType>();
        Eigen::VectorXd waypoint = *state;
        if (!isWithinBounds(waypoint) || !constraint_->isSatisfied(waypoint))
        {
            result.path.clear();
            result.status = Status::INVALID_SOLUTION;
            result.message = "Solution contains an invalid closed-chain waypoint";
            return result;
        }
        if (collision_checker_)
        {
            const auto collision = collision_checker_->check(waypoint);
            ++result.collision_check_count;
            result.minimum_collision_distance = std::min(
                result.minimum_collision_distance,
                collision.minimum_distance);
            if (!collision.query_valid)
            {
                result.path.clear();
                result.status = Status::COLLISION_CHECK_FAILED;
                result.message = "Collision query failed for a solution waypoint: " +
                                 collision.message;
                return result;
            }
            if (!collision.collision_free)
            {
                result.path.clear();
                result.status = Status::INVALID_SOLUTION;
                result.message = "Solution contains a colliding waypoint: " +
                                 collision.message;
                return result;
            }
        }
        result.path.push_back(std::move(waypoint));
    }

    std::size_t search_start = 0;
    for (const auto& milestone : original_milestones)
    {
        bool found = false;
        for (std::size_t i = search_start; i < result.path.size(); ++i)
        {
            if (result.path[i].isApprox(milestone, 1.0e-12))
            {
                result.milestone_indices.push_back(i);
                search_start = i + 1;
                found = true;
                break;
            }
        }
        if (!found)
        {
            result.path.clear();
            result.milestone_indices.clear();
            result.status = Status::INVALID_SOLUTION;
            result.message = "Failed to preserve OMPL milestones while densifying the path";
            return result;
        }
    }

    const Eigen::VectorXd& path_end = result.path.back();
    double closest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < accepted_goals.size(); ++i)
    {
        const double distance = (path_end - accepted_goals[i]).norm();
        if (distance < closest_distance)
        {
            closest_distance = distance;
            result.selected_goal_index = result.accepted_goal_indices[i];
        }
    }

    result.status = Status::SUCCESS;
    std::ostringstream stream;
    stream << "Projected RRTConnect found a closed-chain path with "
           << result.path.size() << " waypoints";
    if (collision_checker_)
    {
        stream << "; checked configured collision pairs "
               << result.collision_check_count << " times";
    }
    else
    {
        stream << "; collision checking was not configured";
    }
    result.message = stream.str();
    return result;
}

const char* BimanualProjectedPlanner::statusName(Status status) noexcept
{
    switch (status)
    {
    case Status::SUCCESS:
        return "success";
    case Status::INVALID_INPUT:
        return "invalid_input";
    case Status::START_PROJECTION_FAILED:
        return "start_projection_failed";
    case Status::START_IN_COLLISION:
        return "start_in_collision";
    case Status::COLLISION_CHECK_FAILED:
        return "collision_check_failed";
    case Status::NO_VALID_GOALS:
        return "no_valid_goals";
    case Status::TIMEOUT:
        return "timeout";
    case Status::PLANNING_FAILED:
        return "planning_failed";
    case Status::APPROXIMATE_SOLUTION:
        return "approximate_solution";
    case Status::INVALID_SOLUTION:
        return "invalid_solution";
    }
    return "unknown";
}

bool BimanualProjectedPlanner::isWithinBounds(
    const Eigen::VectorXd& state) const
{
    return state.size() == constraint_->lowerBounds().size() &&
           (state.array() >= constraint_->lowerBounds().array()).all() &&
           (state.array() <= constraint_->upperBounds().array()).all();
}

bool BimanualProjectedPlanner::validateOptions(
    const Options& options, std::string& error) const
{
    if (!std::isfinite(options.planning_time_seconds) ||
        options.planning_time_seconds <= 0.0)
    {
        error = "planning_time_seconds must be finite and positive";
        return false;
    }
    if (!std::isfinite(options.delta) || options.delta <= 0.0)
    {
        error = "delta must be finite and positive";
        return false;
    }
    if (!std::isfinite(options.lambda) || options.lambda <= 1.0)
    {
        error = "lambda must be finite and greater than one";
        return false;
    }
    if (options.projection_max_iterations == 0)
    {
        error = "projection_max_iterations must be positive";
        return false;
    }
    if (!std::isfinite(options.planner_range) || options.planner_range < 0.0)
    {
        error = "planner_range must be finite and non-negative";
        return false;
    }
    if (!std::isfinite(options.max_start_projection_distance) ||
        options.max_start_projection_distance < 0.0)
    {
        error = "max_start_projection_distance must be finite and non-negative";
        return false;
    }
    return true;
}

}  // namespace arms_controller_common
