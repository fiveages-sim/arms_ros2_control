#include "arms_controller_common/utils/BimanualClosedChainConstraint.h"
#include "arms_controller_common/utils/BimanualProjectedPlanner.h"
#include "arms_controller_common/utils/BimanualSelfCollisionChecker.h"
#include "arms_controller_common/utils/CartesianTrajectoryManager.h"

#include <gtest/gtest.h>

#include <coal/shape/geometric_shapes.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <Eigen/LU>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

using arms_controller_common::BimanualClosedChainConstraint;
using arms_controller_common::BimanualProjectedPlanner;
using arms_controller_common::BimanualSelfCollisionChecker;
using arms_controller_common::CartesianTrajectoryManager;

struct TestRobot
{
    pinocchio::Model model;
    std::vector<std::string> left_joint_names;
    std::vector<std::string> right_joint_names;
    std::string left_frame = "left_tcp";
    std::string right_frame = "right_tcp";
};

template<typename JointModel>
pinocchio::JointIndex addRevoluteJoint(pinocchio::Model& model,
                                      pinocchio::JointIndex parent,
                                      const JointModel& joint_model,
                                      const pinocchio::SE3& placement,
                                      const std::string& name)
{
    const auto joint_id = model.addJoint(parent, joint_model, placement, name);
    model.appendBodyToJoint(joint_id, pinocchio::Inertia::Zero());
    model.addJointFrame(joint_id);
    return joint_id;
}

std::vector<std::string> addArm(pinocchio::Model& model,
                                const std::string& prefix,
                                double shoulder_y,
                                const std::string& tcp_frame)
{
    std::vector<std::string> names;
    names.reserve(7);
    pinocchio::JointIndex parent = 0;

    for (int i = 0; i < 7; ++i)
    {
        const std::string name = prefix + "_joint_" + std::to_string(i + 1);
        const Eigen::Vector3d translation =
            (i == 0) ? Eigen::Vector3d(0.0, shoulder_y, 0.25)
                     : Eigen::Vector3d(0.16, 0.0, 0.0);
        const pinocchio::SE3 placement(Eigen::Matrix3d::Identity(), translation);

        switch (i % 3)
        {
        case 0:
            parent = addRevoluteJoint(model, parent, pinocchio::JointModelRZ(),
                                      placement, name);
            break;
        case 1:
            parent = addRevoluteJoint(model, parent, pinocchio::JointModelRY(),
                                      placement, name);
            break;
        default:
            parent = addRevoluteJoint(model, parent, pinocchio::JointModelRX(),
                                      placement, name);
            break;
        }
        names.push_back(name);
    }

    model.addFrame(pinocchio::Frame(
        tcp_frame, parent,
        pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.14, 0.0, 0.0)),
        pinocchio::OP_FRAME));
    return names;
}

TestRobot makeTestRobot()
{
    TestRobot robot;
    robot.model.name = "test_dual_arm";
    robot.left_joint_names =
        addArm(robot.model, "left", 0.32, robot.left_frame);
    robot.right_joint_names =
        addArm(robot.model, "right", -0.32, robot.right_frame);

    robot.model.lowerPositionLimit.setConstant(-2.8);
    robot.model.upperPositionLimit.setConstant(2.8);
    robot.model.velocityLimit.setConstant(2.0);
    robot.model.effortLimit.setConstant(100.0);
    return robot;
}

Eigen::VectorXd referenceState()
{
    Eigen::VectorXd q(14);
    q << 0.20, -0.45, 0.35, 0.55, -0.30, 0.40, -0.15,
        -0.25, 0.50, -0.40, -0.45, 0.25, -0.35, 0.20;
    return q;
}

pinocchio::SE3 relativePose(const TestRobot& robot, const Eigen::VectorXd& q)
{
    pinocchio::Data data(robot.model);
    pinocchio::forwardKinematics(robot.model, data, q);
    pinocchio::updateFramePlacements(robot.model, data);
    const auto left_id = robot.model.getFrameId(robot.left_frame);
    const auto right_id = robot.model.getFrameId(robot.right_frame);
    return data.oMf[left_id].actInv(data.oMf[right_id]);
}

std::shared_ptr<BimanualClosedChainConstraint> makeConstraint(
    const TestRobot& robot, const Eigen::VectorXd& q_reference,
    double tolerance = 1e-9)
{
    return std::make_shared<BimanualClosedChainConstraint>(
        robot.model, robot.left_joint_names, robot.right_joint_names,
        robot.left_frame, robot.right_frame, q_reference,
        relativePose(robot, q_reference), tolerance);
}

Eigen::VectorXd projectOrThrow(BimanualClosedChainConstraint& constraint,
                               Eigen::VectorXd state)
{
    constraint.setMaxIterations(100);
    if (!constraint.project(state))
    {
        throw std::runtime_error("Test state projection failed");
    }
    return state;
}

Eigen::VectorXd evaluate(const BimanualClosedChainConstraint& constraint,
                         const Eigen::VectorXd& q)
{
    Eigen::VectorXd value(6);
    constraint.function(q, value);
    return value;
}

Eigen::Vector3d jointLocalPointInWorld(
    const pinocchio::Model& model, const Eigen::VectorXd& q,
    pinocchio::JointIndex joint_id, const Eigen::Vector3d& local_point)
{
    pinocchio::Data data(model);
    pinocchio::forwardKinematics(model, data, q);
    return data.oMi[joint_id].act(local_point);
}

pinocchio::GeometryModel makeTwoSphereGeometry(
    pinocchio::JointIndex moving_joint,
    const Eigen::Vector3d& moving_local_position,
    const Eigen::Vector3d& fixed_world_position,
    double radius)
{
    pinocchio::GeometryModel geometry_model;
    const auto moving_id = geometry_model.addGeometryObject(
        pinocchio::GeometryObject(
            "moving_sphere", moving_joint, 0,
            pinocchio::SE3(Eigen::Matrix3d::Identity(),
                           moving_local_position),
            std::make_shared<coal::Sphere>(radius)));
    const auto fixed_id = geometry_model.addGeometryObject(
        pinocchio::GeometryObject(
            "fixed_sphere", 0, 0,
            pinocchio::SE3(Eigen::Matrix3d::Identity(),
                           fixed_world_position),
            std::make_shared<coal::Sphere>(radius)));
    geometry_model.addCollisionPair(
        pinocchio::CollisionPair(moving_id, fixed_id));
    return geometry_model;
}

geometry_msgs::msg::Pose toPoseMessage(const pinocchio::SE3& transform)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();
    const Eigen::Quaterniond quaternion(transform.rotation());
    pose.orientation.x = quaternion.x();
    pose.orientation.y = quaternion.y();
    pose.orientation.z = quaternion.z();
    pose.orientation.w = quaternion.w();
    return pose;
}

std::pair<pinocchio::SE3, pinocchio::SE3> endEffectorPoses(
    const TestRobot& robot, const Eigen::VectorXd& q)
{
    pinocchio::Data data(robot.model);
    pinocchio::forwardKinematics(robot.model, data, q);
    pinocchio::updateFramePlacements(robot.model, data);
    return {
        data.oMf[robot.model.getFrameId(robot.left_frame)],
        data.oMf[robot.model.getFrameId(robot.right_frame)]};
}

TEST(BimanualClosedChainConstraintTest, ReferenceStateSatisfiesRigidGrasp)
{
    const TestRobot robot = makeTestRobot();
    const Eigen::VectorXd q_reference = referenceState();
    const auto constraint = makeConstraint(robot, q_reference);

    EXPECT_LT(evaluate(*constraint, q_reference).norm(), 1e-12);

    const pinocchio::SE3 actual = constraint->relativePose(q_reference);
    const pinocchio::SE3 error =
        constraint->desiredRelativePose().actInv(actual);
    EXPECT_LT(pinocchio::log6(error).toVector().norm(), 1e-12);
}

TEST(BimanualClosedChainConstraintTest, ExposesAmbientJointLimitsInInputOrder)
{
    const TestRobot robot = makeTestRobot();
    const Eigen::VectorXd q_reference = referenceState();
    const auto constraint = makeConstraint(robot, q_reference);

    ASSERT_EQ(constraint->lowerBounds().size(), 14);
    ASSERT_EQ(constraint->upperBounds().size(), 14);
    EXPECT_TRUE(constraint->lowerBounds().isApprox(
        Eigen::VectorXd::Constant(14, -2.8)));
    EXPECT_TRUE(constraint->upperBounds().isApprox(
        Eigen::VectorXd::Constant(14, 2.8)));
    EXPECT_TRUE(constraint->fullConfiguration(q_reference).isApprox(q_reference));
}

TEST(BimanualClosedChainConstraintTest, AnalyticJacobianMatchesFiniteDifference)
{
    const TestRobot robot = makeTestRobot();
    const Eigen::VectorXd q_reference = referenceState();
    const auto constraint = makeConstraint(robot, q_reference);

    Eigen::VectorXd q_evaluation = q_reference;
    Eigen::VectorXd offset(14);
    offset << 0.04, -0.03, 0.02, 0.01, -0.025, 0.035, -0.015,
        -0.02, 0.015, -0.035, 0.03, 0.01, -0.02, 0.025;
    q_evaluation += offset;
    ASSERT_GT(evaluate(*constraint, q_evaluation).norm(), 1e-3);

    Eigen::MatrixXd analytic(6, 14);
    constraint->jacobian(q_evaluation, analytic);

    constexpr double step = 1e-7;
    Eigen::MatrixXd numeric(6, 14);
    for (Eigen::Index column = 0; column < q_evaluation.size(); ++column)
    {
        Eigen::VectorXd plus = q_evaluation;
        Eigen::VectorXd minus = q_evaluation;
        plus[column] += step;
        minus[column] -= step;
        numeric.col(column) =
            (evaluate(*constraint, plus) - evaluate(*constraint, minus)) /
            (2.0 * step);
    }

    EXPECT_LT((analytic - numeric).cwiseAbs().maxCoeff(), 2e-6);
    EXPECT_LT((analytic - numeric).norm(), 5e-6);
}

TEST(BimanualClosedChainConstraintTest, GenericConfigurationHasFullConstraintRank)
{
    const TestRobot robot = makeTestRobot();
    const Eigen::VectorXd q_reference = referenceState();
    const auto constraint = makeConstraint(robot, q_reference);

    Eigen::MatrixXd jacobian(6, 14);
    constraint->jacobian(q_reference, jacobian);
    Eigen::FullPivLU<Eigen::MatrixXd> decomposition(jacobian);
    decomposition.setThreshold(1e-8);
    EXPECT_EQ(decomposition.rank(), 6);
}

TEST(BimanualClosedChainConstraintTest, OmplNewtonProjectionReturnsToManifold)
{
    const TestRobot robot = makeTestRobot();
    const Eigen::VectorXd q_reference = referenceState();
    auto constraint = makeConstraint(robot, q_reference, 1e-8);
    constraint->setMaxIterations(100);

    Eigen::VectorXd projected = q_reference;
    Eigen::VectorXd perturbation(14);
    perturbation << 0.03, -0.02, 0.01, 0.025, -0.015, 0.02, -0.01,
        -0.02, 0.025, -0.015, 0.01, 0.02, -0.025, 0.015;
    projected += perturbation;
    ASSERT_GT(evaluate(*constraint, projected).norm(), 1e-4);

    ASSERT_TRUE(constraint->project(projected));
    EXPECT_LT(evaluate(*constraint, projected).norm(), 1e-8);
    EXPECT_TRUE((projected.array() >= constraint->lowerBounds().array()).all());
    EXPECT_TRUE((projected.array() <= constraint->upperBounds().array()).all());
}

TEST(BimanualClosedChainConstraintTest, RejectsUnknownOrNonScalarJointMapping)
{
    const TestRobot robot = makeTestRobot();
    const Eigen::VectorXd q_reference = referenceState();
    auto invalid_left_names = robot.left_joint_names;
    invalid_left_names.front() = "missing_joint";

    EXPECT_THROW(
        BimanualClosedChainConstraint(
            robot.model, invalid_left_names, robot.right_joint_names,
            robot.left_frame, robot.right_frame, q_reference,
            relativePose(robot, q_reference)),
        std::invalid_argument);
}

TEST(BimanualSelfCollisionCheckerTest, ChecksArbitraryCandidateState)
{
    const TestRobot robot = makeTestRobot();
    const Eigen::VectorXd collision_state = referenceState();
    auto constraint = makeConstraint(robot, collision_state, 1e-7);
    const auto moving_joint =
        robot.model.getJointId(robot.left_joint_names.front());
    const Eigen::Vector3d local_point(0.50, 0.0, 0.0);
    const Eigen::Vector3d collision_center = jointLocalPointInWorld(
        robot.model, collision_state, moving_joint, local_point);
    pinocchio::GeometryModel geometry_model = makeTwoSphereGeometry(
        moving_joint, local_point, collision_center, 0.05);

    BimanualSelfCollisionChecker checker(
        constraint, std::move(geometry_model), 0.01);

    const auto collision_result = checker.check(collision_state);
    ASSERT_TRUE(collision_result.query_valid) << collision_result.message;
    EXPECT_FALSE(collision_result.collision_free);
    EXPECT_LE(collision_result.minimum_distance, 0.01);
    EXPECT_EQ(collision_result.first_object, "moving_sphere");
    EXPECT_EQ(collision_result.second_object, "fixed_sphere");

    Eigen::VectorXd safe_state = collision_state;
    safe_state[0] += 1.0;
    const auto safe_result = checker.check(safe_state);
    ASSERT_TRUE(safe_result.query_valid) << safe_result.message;
    EXPECT_TRUE(safe_result.collision_free) << safe_result.message;
    EXPECT_GT(safe_result.minimum_distance, 0.01);

    Eigen::VectorXd invalid_state(13);
    const auto invalid_result = checker.check(invalid_state);
    EXPECT_FALSE(invalid_result.query_valid);
    EXPECT_FALSE(invalid_result.collision_free);
}

TEST(BimanualSelfCollisionCheckerTest, RejectsGeometryWithoutConfiguredPairs)
{
    const TestRobot robot = makeTestRobot();
    const Eigen::VectorXd reference = referenceState();
    auto constraint = makeConstraint(robot, reference, 1e-7);

    EXPECT_THROW(
        BimanualSelfCollisionChecker(
            constraint, pinocchio::GeometryModel(), 0.01),
        std::invalid_argument);
}

TEST(BimanualProjectedPlannerTest, PlansToOneOfMultipleClosedChainGoals)
{
    const TestRobot robot = makeTestRobot();
    const Eigen::VectorXd start = referenceState();
    auto constraint = makeConstraint(robot, start, 1e-7);

    Eigen::VectorXd seed_one = start;
    seed_one[0] += 0.06;
    seed_one[2] -= 0.04;
    seed_one[8] += 0.05;
    seed_one[11] -= 0.03;
    Eigen::VectorXd seed_two = start;
    seed_two[1] -= 0.05;
    seed_two[4] += 0.04;
    seed_two[7] += 0.05;
    seed_two[12] -= 0.04;
    const Eigen::VectorXd goal_one = projectOrThrow(*constraint, seed_one);
    const Eigen::VectorXd goal_two = projectOrThrow(*constraint, seed_two);
    ASSERT_GT((goal_one - start).norm(), 0.01);
    ASSERT_GT((goal_two - start).norm(), 0.01);

    BimanualProjectedPlanner planner(constraint);
    BimanualProjectedPlanner::Options options;
    options.planning_time_seconds = 3.0;
    const auto result = planner.plan(start, {goal_one, goal_two}, options);

    ASSERT_TRUE(result.success()) << result.message;
    EXPECT_FALSE(result.collision_checked);
    EXPECT_EQ(result.accepted_goal_indices,
              (std::vector<std::size_t>{0, 1}));
    EXPECT_TRUE(result.rejected_goal_indices.empty());
    EXPECT_TRUE(result.selected_goal_index == 0 ||
                result.selected_goal_index == 1);
    ASSERT_GE(result.path.size(), 2u);
    ASSERT_GE(result.milestone_indices.size(), 2u);
    EXPECT_EQ(result.milestone_indices.front(), 0u);
    EXPECT_EQ(result.milestone_indices.back(), result.path.size() - 1);
    EXPECT_TRUE(std::is_sorted(result.milestone_indices.begin(),
                               result.milestone_indices.end()));
    EXPECT_TRUE(result.path.front().isApprox(result.projected_start, 1e-9));

    const double end_distance = std::min(
        (result.path.back() - goal_one).norm(),
        (result.path.back() - goal_two).norm());
    EXPECT_LT(end_distance, 1e-8);
    for (const auto& waypoint : result.path)
    {
        EXPECT_TRUE(constraint->isSatisfied(waypoint));
        EXPECT_TRUE((waypoint.array() >= constraint->lowerBounds().array()).all());
        EXPECT_TRUE((waypoint.array() <= constraint->upperBounds().array()).all());
    }
}

TEST(BimanualProjectedPlannerTest, ProjectsStartBeforePlanning)
{
    const TestRobot robot = makeTestRobot();
    const Eigen::VectorXd reference = referenceState();
    auto constraint = makeConstraint(robot, reference, 1e-7);

    Eigen::VectorXd start = reference;
    start[0] += 0.015;
    start[8] -= 0.012;
    ASSERT_FALSE(constraint->isSatisfied(start));

    Eigen::VectorXd goal_seed = reference;
    goal_seed[2] += 0.10;
    goal_seed[3] -= 0.08;
    goal_seed[9] += 0.07;
    goal_seed[11] -= 0.06;
    const Eigen::VectorXd goal = projectOrThrow(*constraint, goal_seed);

    BimanualProjectedPlanner planner(constraint);
    BimanualProjectedPlanner::Options options;
    options.planning_time_seconds = 3.0;
    options.max_start_projection_distance = 0.5;
    const auto result = planner.plan(start, {goal}, options);

    ASSERT_TRUE(result.success()) << result.message;
    EXPECT_TRUE(constraint->isSatisfied(result.projected_start));
    EXPECT_GT((result.projected_start - start).norm(), 1e-5);
    EXPECT_LT((result.path.back() - goal).norm(), 1e-8);
}

TEST(BimanualProjectedPlannerTest, RejectsInvalidGoalAndUsesRemainingGoal)
{
    const TestRobot robot = makeTestRobot();
    const Eigen::VectorXd start = referenceState();
    auto constraint = makeConstraint(robot, start, 1e-7);

    Eigen::VectorXd invalid_goal = start;
    invalid_goal[0] += 0.2;
    ASSERT_FALSE(constraint->isSatisfied(invalid_goal));

    Eigen::VectorXd valid_seed = start;
    valid_seed[1] -= 0.04;
    valid_seed[4] += 0.03;
    valid_seed[8] += 0.035;
    valid_seed[12] -= 0.025;
    const Eigen::VectorXd valid_goal = projectOrThrow(*constraint, valid_seed);

    BimanualProjectedPlanner planner(constraint);
    BimanualProjectedPlanner::Options options;
    options.planning_time_seconds = 3.0;
    const auto result = planner.plan(
        start, {invalid_goal, valid_goal}, options);

    ASSERT_TRUE(result.success()) << result.message;
    EXPECT_EQ(result.accepted_goal_indices,
              (std::vector<std::size_t>{1}));
    EXPECT_EQ(result.rejected_goal_indices,
              (std::vector<std::size_t>{0}));
    EXPECT_EQ(result.selected_goal_index, 1u);
    EXPECT_LT((result.path.back() - valid_goal).norm(), 1e-8);
}

TEST(BimanualProjectedPlannerTest, FailsBeforePlanningWhenAllGoalsAreInvalid)
{
    const TestRobot robot = makeTestRobot();
    const Eigen::VectorXd start = referenceState();
    auto constraint = makeConstraint(robot, start, 1e-7);
    Eigen::VectorXd invalid_goal = start;
    invalid_goal[7] += 0.25;
    ASSERT_FALSE(constraint->isSatisfied(invalid_goal));

    BimanualProjectedPlanner planner(constraint);
    const auto result = planner.plan(start, {invalid_goal});

    EXPECT_FALSE(result.success());
    EXPECT_EQ(result.status,
              BimanualProjectedPlanner::Status::NO_VALID_GOALS);
    EXPECT_TRUE(result.path.empty());
    EXPECT_EQ(result.rejected_goal_indices,
              (std::vector<std::size_t>{0}));
}

TEST(BimanualProjectedPlannerTest, AcceptsCurrentStateAsNoOpGoal)
{
    const TestRobot robot = makeTestRobot();
    const Eigen::VectorXd start = referenceState();
    auto constraint = makeConstraint(robot, start, 1e-7);
    BimanualProjectedPlanner planner(constraint);

    const auto result = planner.plan(start, {start});

    ASSERT_TRUE(result.success()) << result.message;
    ASSERT_EQ(result.path.size(), 1u);
    EXPECT_TRUE(result.path.front().isApprox(start, 1e-9));
    EXPECT_EQ(result.selected_goal_index, 0u);
    EXPECT_FALSE(result.collision_checked);
}

TEST(BimanualProjectedPlannerTest, RejectsCollidingGoalBeforePlanning)
{
    const TestRobot robot = makeTestRobot();
    const Eigen::VectorXd start = referenceState();
    auto constraint = makeConstraint(robot, start, 1e-7);

    Eigen::VectorXd goal_seed = start;
    goal_seed[0] += 0.55;
    goal_seed[2] -= 0.16;
    goal_seed[7] += 0.45;
    goal_seed[9] -= 0.12;
    const Eigen::VectorXd colliding_goal =
        projectOrThrow(*constraint, goal_seed);

    const auto moving_joint =
        robot.model.getJointId(robot.left_joint_names.front());
    const Eigen::Vector3d local_point(0.55, 0.0, 0.0);
    const Eigen::Vector3d start_center = jointLocalPointInWorld(
        robot.model, start, moving_joint, local_point);
    const Eigen::Vector3d goal_center = jointLocalPointInWorld(
        robot.model, colliding_goal, moving_joint, local_point);
    const double center_displacement = (goal_center - start_center).norm();
    ASSERT_GT(center_displacement, 0.05);

    const double radius = center_displacement * 0.10;
    const double safety_distance = center_displacement * 0.10;
    auto checker = std::make_shared<BimanualSelfCollisionChecker>(
        constraint,
        makeTwoSphereGeometry(
            moving_joint, local_point, goal_center, radius),
        safety_distance);
    BimanualProjectedPlanner planner(constraint, checker);

    const auto result = planner.plan(start, {colliding_goal, start});

    ASSERT_TRUE(result.success()) << result.message;
    EXPECT_TRUE(result.collision_checked);
    EXPECT_EQ(result.collision_rejected_goal_indices,
              (std::vector<std::size_t>{0}));
    EXPECT_EQ(result.rejected_goal_indices,
              (std::vector<std::size_t>{0}));
    EXPECT_EQ(result.accepted_goal_indices,
              (std::vector<std::size_t>{1}));
    EXPECT_EQ(result.selected_goal_index, 1u);
    EXPECT_GE(result.collision_check_count, 3u);
    EXPECT_TRUE(std::isfinite(result.minimum_collision_distance));
}

TEST(BimanualProjectedPlannerTest, RejectsCollidingProjectedStart)
{
    const TestRobot robot = makeTestRobot();
    const Eigen::VectorXd start = referenceState();
    auto constraint = makeConstraint(robot, start, 1e-7);
    const auto moving_joint =
        robot.model.getJointId(robot.left_joint_names.front());
    const Eigen::Vector3d local_point(0.50, 0.0, 0.0);
    const Eigen::Vector3d start_center = jointLocalPointInWorld(
        robot.model, start, moving_joint, local_point);
    auto checker = std::make_shared<BimanualSelfCollisionChecker>(
        constraint,
        makeTwoSphereGeometry(
            moving_joint, local_point, start_center, 0.05),
        0.01);
    BimanualProjectedPlanner planner(constraint, checker);

    const auto result = planner.plan(start, {start});

    EXPECT_FALSE(result.success());
    EXPECT_TRUE(result.collision_checked);
    EXPECT_EQ(result.status,
              BimanualProjectedPlanner::Status::START_IN_COLLISION);
    EXPECT_EQ(result.collision_check_count, 1u);
}

TEST(BimanualProjectedPlannerTest, ChecksOmplStatesAndReturnedPathForCollision)
{
    const TestRobot robot = makeTestRobot();
    const Eigen::VectorXd start = referenceState();
    auto constraint = makeConstraint(robot, start, 1e-7);

    Eigen::VectorXd goal_seed = start;
    goal_seed[0] += 0.06;
    goal_seed[2] -= 0.04;
    goal_seed[8] += 0.05;
    goal_seed[11] -= 0.03;
    const Eigen::VectorXd goal = projectOrThrow(*constraint, goal_seed);
    ASSERT_GT((goal - start).norm(), 0.01);

    const auto moving_joint =
        robot.model.getJointId(robot.left_joint_names.front());
    auto checker = std::make_shared<BimanualSelfCollisionChecker>(
        constraint,
        makeTwoSphereGeometry(
            moving_joint, Eigen::Vector3d(0.50, 0.0, 0.0),
            Eigen::Vector3d(10.0, 10.0, 10.0), 0.05),
        0.01);
    BimanualProjectedPlanner planner(constraint, checker);
    BimanualProjectedPlanner::Options options;
    options.planning_time_seconds = 3.0;

    const auto result = planner.plan(start, {goal}, options);

    ASSERT_TRUE(result.success()) << result.message;
    EXPECT_TRUE(result.collision_checked);
    ASSERT_GE(result.path.size(), 2u);
    EXPECT_GT(result.collision_check_count, result.path.size() + 2u);
    EXPECT_GT(result.minimum_collision_distance, 0.01);
    for (const auto& waypoint : result.path)
    {
        const auto collision = checker->check(waypoint);
        ASSERT_TRUE(collision.query_valid) << collision.message;
        EXPECT_TRUE(collision.collision_free) << collision.message;
    }
}

#ifdef HAS_LINA_PLANNING
TEST(CooperativeMotionPipelineTest, PlansAndTimeParameterizesClosedChainPath)
{
    const TestRobot robot = makeTestRobot();
    const Eigen::VectorXd start = referenceState();
    // Execution must stay on a numerically tight manifold even when the task
    // accepts millimetre-level relative-pose error.
    auto validation_constraint = makeConstraint(robot, start, 2.0e-6);

    Eigen::VectorXd goal_seed = start;
    goal_seed[0] += 0.08;
    goal_seed[2] -= 0.05;
    goal_seed[8] += 0.07;
    goal_seed[11] -= 0.04;
    const Eigen::VectorXd goal = projectOrThrow(
        *validation_constraint, goal_seed);
    ASSERT_GT((goal - start).norm(), 0.02);

    const auto [left_start, right_start] = endEffectorPoses(robot, start);
    const auto [left_goal, right_goal] = endEffectorPoses(robot, goal);
    auto kinematics = std::make_shared<arms_controller_common::ArmKinematics>(
        robot.model, "arm_base");
    std::vector<std::string> all_joint_names = robot.left_joint_names;
    all_joint_names.insert(all_joint_names.end(),
                           robot.right_joint_names.begin(),
                           robot.right_joint_names.end());
    kinematics->initializeFromParameters(
        all_joint_names, robot.left_frame, robot.right_frame);

    CartesianTrajectoryManager manager;
    manager.setKinematicsSolver(kinematics);
    const auto moving_joint =
        robot.model.getJointId(robot.left_joint_names.front());
    manager.setCooperativeCollisionGeometry(
        makeTwoSphereGeometry(
            moving_joint, Eigen::Vector3d(0.50, 0.0, 0.0),
            Eigen::Vector3d(10.0, 10.0, 10.0), 0.05),
        0.01);

    arms_ros2_control_msgs::msg::CooperativeMotion motion;
    motion.left_start = toPoseMessage(left_start);
    motion.right_start = toPoseMessage(right_start);
    motion.left_goal = toPoseMessage(left_goal);
    motion.right_goal = toPoseMessage(right_goal);
    motion.frame_id = "arm_base";
    motion.ik_type = "AUTO";
    motion.duration = 0.10;
    motion.auto_extend_duration = true;
    motion.max_translation_step = 0.01;
    motion.max_rotation_step = 0.02;
    motion.max_linear_velocity = 0.20;
    motion.max_linear_acceleration = 1.0;
    motion.max_angular_velocity = 0.40;
    motion.max_angular_acceleration = 1.0;
    motion.joint_velocity_scale = 0.5;
    motion.max_joint_acceleration = 1.0;
    motion.start_position_tolerance = 1.0e-6;
    motion.start_orientation_tolerance = 1.0e-6;
    motion.relative_position_tolerance = 3.0e-3;
    motion.relative_orientation_tolerance = 1.5e-2;
    motion.ik_max_iterations = 500;
    motion.ik_tolerance = 1.0e-5;

    constexpr double period = 0.01;
    std::vector<double> start_vector(start.data(), start.data() + start.size());
    ASSERT_TRUE(manager.planCooperativeMotion(start_vector, motion, period))
        << manager.getLastError();
    EXPECT_GT(manager.getPlanningTime(), motion.duration);

    const Eigen::VectorXd velocity_limits =
        kinematics->getJointVelocityLimits("both") *
        motion.joint_velocity_scale;
    Eigen::VectorXd previous = start;
    Eigen::VectorXd previous_velocity = Eigen::VectorXd::Zero(start.size());
    std::size_t sample_count = 0;
    while (!manager.isCompleted() && sample_count < 10000)
    {
        std::vector<double> sample;
        ASSERT_TRUE(manager.getNextJointPos(sample))
            << manager.getLastError();
        ASSERT_EQ(sample.size(), static_cast<std::size_t>(start.size()));
        const Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(
            sample.data(), static_cast<Eigen::Index>(sample.size()));
        EXPECT_TRUE(validation_constraint->isSatisfied(q));

        const Eigen::VectorXd velocity = (q - previous) / period;
        const Eigen::VectorXd acceleration =
            (velocity - previous_velocity) / period;
        EXPECT_LE(
            (velocity.cwiseAbs().array() /
             velocity_limits.array()).maxCoeff(),
            1.001);
        EXPECT_LE(acceleration.cwiseAbs().maxCoeff(),
                  motion.max_joint_acceleration * 1.001);
        previous = q;
        previous_velocity = velocity;
        ++sample_count;
    }
    ASSERT_TRUE(manager.isCompleted());
    EXPECT_GT(sample_count, 1u);
    const auto [executed_left_goal, executed_right_goal] =
        endEffectorPoses(robot, previous);
    EXPECT_LT((executed_left_goal.translation() -
               left_goal.translation()).norm(),
              motion.relative_position_tolerance);
    EXPECT_LT((executed_right_goal.translation() -
               right_goal.translation()).norm(),
              motion.relative_position_tolerance);
    EXPECT_LT(Eigen::AngleAxisd(
                  executed_left_goal.rotation().transpose() *
                  left_goal.rotation()).angle(),
              motion.relative_orientation_tolerance);
    EXPECT_LT(Eigen::AngleAxisd(
                  executed_right_goal.rotation().transpose() *
                  right_goal.rotation()).angle(),
              motion.relative_orientation_tolerance);
    EXPECT_LE(previous_velocity.cwiseAbs().maxCoeff() / period,
              motion.max_joint_acceleration * 1.001);
}
#endif

}  // namespace
