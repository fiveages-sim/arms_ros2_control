#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "ocs2_arm_controller/ForceAdmittance.h"

namespace ocs2::mobile_manipulator
{
namespace
{
ForceAdmittanceAxisLimits conservativeLimits()
{
    ForceAdmittanceAxisLimits limits;
    limits.virtual_mass = 80.0;
    limits.virtual_damping = 2000.0;
    limits.max_velocity = 0.003;
    limits.max_acceleration = 0.01;
    limits.max_jerk = 0.05;
    limits.max_displacement = 0.05;
    limits.min_dt = 0.001;
    limits.max_dt = 0.02;
    return limits;
}

TEST(ForceAdmittance, EnforcesVelocityAccelerationAndJerkLimits)
{
    const auto limits = conservativeLimits();
    ForceAdmittanceAxisState state;
    double previous_acceleration = state.acceleration;
    constexpr double dt = 0.01;
    for (int i = 0; i < 1000; ++i)
    {
        state = stepForceAdmittanceAxis(state, 100.0, dt, limits);
        EXPECT_LE(std::abs(state.velocity), limits.max_velocity + 1e-12);
        EXPECT_LE(std::abs(state.acceleration), limits.max_acceleration + 1e-12);
        EXPECT_LE(std::abs(state.acceleration - previous_acceleration),
                  limits.max_jerk * dt + 1e-12);
        EXPECT_LE(std::abs(state.position), limits.max_displacement + 1e-12);
        previous_acceleration = state.acceleration;
    }
}

TEST(ForceAdmittance, ConvergesOnHardAndSoftSurfaces)
{
    const auto run_surface = [](double stiffness)
    {
        auto limits = conservativeLimits();
        limits.max_displacement = 0.10;
        ForceAdmittanceAxisState state;
        constexpr double desired_force = 1.0;
        constexpr double dt = 0.01;
        for (int i = 0; i < 30000; ++i)
        {
            const double measured_force = stiffness * std::max(0.0, state.position);
            state = stepForceAdmittanceAxis(
                state, desired_force - measured_force, dt, limits);
        }
        return stiffness * std::max(0.0, state.position);
    };

    EXPECT_NEAR(run_surface(100.0), 1.0, 0.03);    // soft pad
    EXPECT_NEAR(run_surface(10000.0), 1.0, 0.08);  // rigid wall
}

TEST(ForceAdmittance, HandlesTimingJitterAndInvalidInputs)
{
    const auto limits = conservativeLimits();
    ForceAdmittanceAxisState state;
    const double periods[] = {0.0001, 0.007, 0.010, 0.025,
                              std::numeric_limits<double>::quiet_NaN()};
    for (int i = 0; i < 1000; ++i)
    {
        const double error = (i == 500)
            ? std::numeric_limits<double>::infinity() : 1.0;
        state = stepForceAdmittanceAxis(state, error, periods[i % 5], limits);
        EXPECT_TRUE(std::isfinite(state.position));
        EXPECT_TRUE(std::isfinite(state.velocity));
        EXPECT_TRUE(std::isfinite(state.acceleration));
    }
}

TEST(ForceAdmittance, SmoothlyBrakesAtDisplacementBoundary)
{
    auto limits = conservativeLimits();
    limits.max_displacement = 0.002;
    ForceAdmittanceAxisState state;
    double previous_acceleration = state.acceleration;
    for (int i = 0; i < 2000; ++i)
    {
        state = stepForceAdmittanceAxis(state, 10.0, 0.01, limits);
        EXPECT_LE(state.position, limits.max_displacement + 1e-12);
        EXPECT_LE(std::abs(state.acceleration - previous_acceleration),
                  limits.max_jerk * 0.01 + 1e-12);
        previous_acceleration = state.acceleration;
    }
    EXPECT_LE(state.position, limits.max_displacement);
    EXPECT_GT(state.position, 0.001);
    EXPECT_NEAR(state.velocity, 0.0, 1e-9);
    EXPECT_EQ(state.saturation_direction, 1);
}

TEST(ForceAdmittance, StaleSensorCenteringReturnsOffsetSmoothly)
{
    auto limits = conservativeLimits();
    ForceAdmittanceAxisState state{0.02, 0.0, 0.0};
    double previous_acceleration = state.acceleration;
    for (int i = 0; i < 20000; ++i)
    {
        const double centering_error = -200.0 * state.position;
        state = stepForceAdmittanceAxis(state, centering_error, 0.01, limits);
        EXPECT_LE(std::abs(state.acceleration - previous_acceleration),
                  limits.max_jerk * 0.01 + 1e-12);
        previous_acceleration = state.acceleration;
    }
    EXPECT_NEAR(state.position, 0.0, 1e-4);
    EXPECT_NEAR(state.velocity, 0.0, 1e-4);
}

TEST(ForceAdmittance, EquilibriumComplianceSettlesAtFiniteOffset)
{
    auto limits = conservativeLimits();
    limits.virtual_damping = 800.0;
    limits.max_displacement = 0.05;
    ForceAdmittanceAxisState state;
    constexpr double applied_force = 1.0;
    constexpr double stiffness = 100.0;
    for (int i = 0; i < 30000; ++i)
    {
        const double equilibrium_input = applied_force - stiffness * state.position;
        state = stepForceAdmittanceAxis(state, equilibrium_input, 0.01, limits);
    }
    EXPECT_NEAR(state.position, applied_force / stiffness, 1e-4);
    EXPECT_NEAR(state.velocity, 0.0, 1e-5);
}

TEST(ForceAdmittance, WrongFeedbackSignDivergesTowardBound)
{
    auto limits = conservativeLimits();
    limits.max_displacement = 0.01;
    ForceAdmittanceAxisState correct;
    ForceAdmittanceAxisState wrong;
    constexpr double stiffness = 1000.0;
    for (int i = 0; i < 10000; ++i)
    {
        const double correct_force = stiffness * std::max(0.0, correct.position);
        const double wrong_sensor_value = -stiffness * std::max(0.0, wrong.position);
        correct = stepForceAdmittanceAxis(correct, 1.0 - correct_force, 0.01, limits);
        wrong = stepForceAdmittanceAxis(wrong, 1.0 - wrong_sensor_value, 0.01, limits);
    }
    EXPECT_NEAR(correct.position, 0.001, 2e-4);
    EXPECT_GT(wrong.position, 0.009);
}
} // namespace
} // namespace ocs2::mobile_manipulator
