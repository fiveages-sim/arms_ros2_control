#include "arms_controller_common/FSM/StateMoveJ.h"
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace arms_controller_common;

static void near(double actual, double expected, const char* reason)
{
    if (!std::isfinite(actual) || std::abs(actual - expected) > 1e-10)
        throw std::runtime_error(std::string(reason) + ": " + std::to_string(actual) +
                                 " != " + std::to_string(expected));
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_movej_servo");
        node->declare_parameter("movej_duration", 3.0);
        node->declare_parameter("movej_interpolation_type", "servo");
        node->declare_parameter("movej_tanh_scale", 3.0);
        node->declare_parameter("movej_trajectory_duration", 3.0);
        node->declare_parameter("movej_trajectory_blend_ratio", 0.0);
        std::array<double, 2> measured{0.2, -0.1}, velocity{0.1, -0.05}, commands{};
        std::array<hardware_interface::StateInterface, 2> states{
            hardware_interface::StateInterface("left_joint", "position", &measured[0]),
            hardware_interface::StateInterface("right_joint", "position", &measured[1])};
        std::array<hardware_interface::StateInterface, 2> velocities{
            hardware_interface::StateInterface("left_joint", "velocity", &velocity[0]),
            hardware_interface::StateInterface("right_joint", "velocity", &velocity[1])};
        std::array<hardware_interface::CommandInterface, 2> outputs{
            hardware_interface::CommandInterface("left_joint", "position", &commands[0]),
            hardware_interface::CommandInterface("right_joint", "position", &commands[1])};
        hardware_interface::LoanedStateInterface s0(states[0]), s1(states[1]);
        hardware_interface::LoanedStateInterface v0(velocities[0]), v1(velocities[1]);
        hardware_interface::LoanedCommandInterface c0(outputs[0]), c1(outputs[1]);
        CtrlInterfaces ctrl;
        ctrl.joint_position_state_interface_ = {s0, s1};
        ctrl.joint_velocity_state_interface_ = {v0, v1};
        ctrl.joint_position_command_interface_ = {c0, c1};
        ctrl.initializeLastSentPositions();
        StateMoveJ state(ctrl, node, {"left_joint", "right_joint"});
        state.enter();
        std::array<JointServoFilter, 2> reference;
        auto reset_reference = [&] {
            for (size_t i = 0; i < 2; ++i)
            {
                reference[i].v_min = -2.0; reference[i].v_max = 2.0;
                reference[i].a_min = -4.0; reference[i].a_max = 4.0;
                reference[i].jerk_limit = 20.0;
                reference[i].reset(measured[i], velocity[i], 0.0);
            }
        };
        reset_reference();
        std::array<double, 2> target = measured;
        auto tick = [&](double dt) {
            state.run(rclcpp::Time(0), rclcpp::Duration::from_seconds(dt));
            const size_t steps = static_cast<size_t>(std::ceil(dt / .001));
            for (size_t i = 0; i < 2; ++i)
            {
                reference[i].dt = dt / steps;
                for (size_t j = 0; j < steps; ++j) reference[i].updateTo(target[i]);
                near(commands[i], reference[i].q, "servo must preserve streaming state");
            }
        };
        // Entry must preserve measured velocity even before the first target.
        tick(.001);
        // Changing targets every tick must never invoke stop-to-zero or restart the filter.
        for (int k = 0; k < 300; ++k)
        {
            target = {std::sin(k * .02), -std::sin(k * .03)};
            state.setTargetPosition({target[0], target[1]});
            tick(k % 2 ? .0025 : .001);
        }
        // Independent prefix streams must not hold/reset the other joint.
        target[0] = .8;
        state.setTargetPosition("left", {target[0]});
        tick(.005);
        target[1] = -.9;
        state.setTargetPosition("right", {target[1]});
        tick(.005);
        state.setTargetPosition("right", {target[1]}); // repeated target
        tick(.001);
        state.setTargetPosition({std::numeric_limits<double>::quiet_NaN(), 0.0});
        tick(.001); // invalid message leaves previous target intact
        state.setTargetPosition("missing", {0.0});
        tick(.001);
        state.setTargetPosition({0.0});
        tick(.001);
        // A timed trajectory must be rejected without changing the live target.
        trajectory_msgs::msg::JointTrajectory trajectory;
        trajectory.joint_names = {"left_joint", "right_joint"};
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {-1.0, 1.0};
        point.time_from_start.sec = 1;
        trajectory.points.push_back(point);
        state.setTrajectory(trajectory);
        tick(.001);
        // Leaving and re-entering initializes from a new measured state.
        state.exit();
        measured = {-.3, .4}; velocity = {0.0, 0.0};
        ctrl.initializeLastSentPositions();
        state.enter(); reset_reference(); target = measured;
        tick(.001);
        // Switching modes and returning must initialize from the latest command.
        node->set_parameter(rclcpp::Parameter("movej_interpolation_type", "none"));
        state.setTargetPosition({.12, -.23});
        state.run(rclcpp::Time(0), rclcpp::Duration::from_seconds(.001));
        near(commands[0], .12, "none mode"); near(commands[1], -.23, "none mode");
        node->set_parameter(rclcpp::Parameter("movej_interpolation_type", "servo"));
        for (size_t i = 0; i < 2; ++i) reference[i].reset(commands[i], 0.0, 0.0);
        target = {.5, -.5}; state.setTargetPosition({target[0], target[1]}); tick(.001);
        // A live linear point-to-point trajectory can switch into servo without
        // waiting for its old duration or resetting on subsequent targets.
        node->set_parameter(rclcpp::Parameter("movej_interpolation_type", "linear"));
        state.setTargetPosition({.7, -.7});
        state.run(rclcpp::Time(0), rclcpp::Duration::from_seconds(.001));
        node->set_parameter(rclcpp::Parameter("movej_interpolation_type", "servo"));
        for (size_t i = 0; i < 2; ++i) reference[i].reset(commands[i], 0.0, 0.0);
        target = {.3, -.3}; state.setTargetPosition({target[0], target[1]}); tick(.001);
        target = {.31, -.31}; state.setTargetPosition({target[0], target[1]}); tick(.001);
        state.exit();
        std::cout << "PASS: entry, continuous targets, prefixes, invalid inputs, re-entry, mode switching\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n'; rclcpp::shutdown(); return 1;
    }
    rclcpp::shutdown();
    return 0;
}
