#include "arms_controller_common/utils/OnlineThirdOrderInterpolator.h"

#include <gtest/gtest.h>

#include <cmath>

namespace
{
arms_controller_common::OnlineThirdOrderConfig makeConfig()
{
    arms_controller_common::OnlineThirdOrderConfig config;
    config.tracking_frequency = {4.0};
    config.max_velocity = {0.5};
    config.min_velocity = {-0.5};
    config.max_acceleration = {1.0};
    config.min_acceleration = {-1.0};
    config.max_jerk = {5.0};
    config.min_jerk = {-5.0};
    config.max_input_velocity = {0.6};
    config.spike_margin = {0.006};
    config.spike_cluster_tolerance = {0.01};
    config.deadband = {0.001};
    config.alpha = 0.65;
    config.beta = 0.08;
    config.input_timeout = 0.3;
    config.max_integrator_dt = 0.001;
    config.position_tolerance = {1.0e-5};
    config.velocity_tolerance = {1.0e-4};
    config.acceleration_tolerance = {1.0e-3};
    return config;
}
}  // namespace

TEST(OnlineThirdOrderInterpolator, HoldsLatestFramePendingForLookahead)
{
    arms_controller_common::OnlineThirdOrderInterpolator filter(makeConfig());
    filter.reset({0.0}, {}, {}, 0.0);
    for (int i = 1; i <= 5; ++i)
    {
        filter.pushTarget({0.0}, 0.05 * i);
    }

    filter.pushTarget({1.0}, 0.30);
    EXPECT_NEAR(filter.estimatedTargetPosition()[0], 0.0, 1.0e-12);
}

TEST(OnlineThirdOrderInterpolator, AcceptsSingleDistantInitialTarget)
{
    arms_controller_common::OnlineThirdOrderInterpolator filter(makeConfig());
    filter.reset({0.0}, {}, {}, 0.0);

    EXPECT_EQ(filter.pushTarget({1.0}, 0.01),
              arms_controller_common::OnlineThirdOrderInterpolator::TargetResult::PENDING_SPIKE);
    const double initial_position = filter.position()[0];
    for (int i = 1; i <= 100; ++i)
    {
        filter.update(0.001, 0.01 + 0.001 * i);
    }
    EXPECT_GT(filter.position()[0], initial_position);
}

TEST(OnlineThirdOrderInterpolator, RejectsMiddleSpikeAfterRecoveryFrame)
{
    auto config = makeConfig();
    arms_controller_common::OnlineThirdOrderInterpolator filter(config);
    filter.reset({0.0}, {}, {}, 0.0);

    filter.pushTarget({0.0}, 0.05);
    filter.pushTarget({1.0}, 0.10);
    EXPECT_EQ(filter.pushTarget({0.001}, 0.15),
              arms_controller_common::OnlineThirdOrderInterpolator::TargetResult::PENDING_SPIKE);
    EXPECT_NEAR(filter.estimatedTargetPosition()[0], 0.0, 1.0e-12);
}

TEST(OnlineThirdOrderInterpolator, AlphaBetaEstimatesMovingTargetVelocity)
{
    auto config = makeConfig();
    config.spike_margin = {0.1};
    arms_controller_common::OnlineThirdOrderInterpolator filter(config);
    filter.reset({0.0}, {}, {}, 0.0);

    for (int i = 1; i <= 20; ++i)
    {
        const double stamp = 0.05 * i;
        filter.pushTarget({0.2 * stamp}, stamp);
        filter.update(0.05, stamp);
    }
    EXPECT_GT(filter.estimatedTargetVelocity()[0], 0.05);
    EXPECT_LT(filter.estimatedTargetVelocity()[0], 0.35);
}

TEST(OnlineThirdOrderInterpolator, RespectsKinematicLimits)
{
    auto config = makeConfig();
    arms_controller_common::OnlineThirdOrderInterpolator filter(config);
    filter.reset({0.0}, {}, {}, 0.0);
    filter.pushTarget({1.0}, 0.01);

    double previous_acceleration = 0.0;
    for (int i = 1; i <= 3000; ++i)
    {
        filter.update(0.001, 0.01 + 0.001 * i);
        ASSERT_LE(std::abs(filter.velocity()[0]), 0.5 + 1.0e-9);
        ASSERT_LE(std::abs(filter.acceleration()[0]), 1.0 + 1.0e-9);
        const double jerk = (filter.acceleration()[0] - previous_acceleration) / 0.001;
        ASSERT_LE(std::abs(jerk), 5.0 + 1.0e-6);
        previous_acceleration = filter.acceleration()[0];
    }
}

TEST(OnlineThirdOrderInterpolator, RepeatedStationaryTargetRemovesEstimatedVelocity)
{
    auto config = makeConfig();
    config.spike_margin = {0.1};
    arms_controller_common::OnlineThirdOrderInterpolator filter(config);
    filter.reset({0.0}, {}, {}, 0.0);

    for (int i = 1; i <= 20; ++i)
    {
        const double stamp = 0.05 * i;
        filter.pushTarget({0.2 * stamp}, stamp);
        filter.update(0.05, stamp);
    }
    const double moving_velocity = std::abs(filter.estimatedTargetVelocity()[0]);

    for (int i = 21; i <= 60; ++i)
    {
        const double stamp = 0.05 * i;
        filter.pushTarget({0.2}, stamp);
        filter.update(0.05, stamp);
    }
    EXPECT_LT(std::abs(filter.estimatedTargetVelocity()[0]), moving_velocity * 0.25);
}

TEST(OnlineThirdOrderInterpolator, ValidInputRefreshesTimeout)
{
    auto config = makeConfig();
    arms_controller_common::OnlineThirdOrderInterpolator filter(config);
    filter.reset({0.0}, {}, {}, 0.0);
    filter.pushTarget({0.001}, 0.05);
    filter.pushTarget({2.0}, 0.20);
    EXPECT_FALSE(filter.targetTimedOut(0.36));
    EXPECT_TRUE(filter.targetTimedOut(0.51));
}
