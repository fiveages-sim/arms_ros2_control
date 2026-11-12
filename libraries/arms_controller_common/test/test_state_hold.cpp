#include <gtest/gtest.h>
#include <arms_controller_common/FSM/StateHold.h>
#include <hardware_interface/handle.hpp>
#include <array>
#include <limits>
#include <memory>

namespace
{
class HoldOutputTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        if (!rclcpp::ok()) rclcpp::init(0, nullptr);
        node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("hold_output_test");
        node->declare_parameter("hold_first_check_position_threshold", 0.1);
        node->declare_parameter("hold_position_threshold", 0.15);
        ctrl.last_sent_joint_positions_ = {0.1, -0.2, 0.3, -0.4, 0.5, -0.6};
        for (size_t i = 0; i < 6; ++i)
        {
            measured[i] = ctrl.last_sent_joint_positions_[i];
            position[i] = measured[i];
            velocity[i] = (i % 2 ? -1.0 : 1.0) * (i + 1) * 0.7;
            auto state = std::make_unique<hardware_interface::StateInterface>(
                "joint" + std::to_string(i), "position", &measured[i]);
            auto state_loan = std::make_unique<hardware_interface::LoanedStateInterface>(*state);
            ctrl.joint_position_state_interface_.push_back(std::ref(*state_loan));
            states.push_back(std::move(state)); state_loans.push_back(std::move(state_loan));
            addCommand("position", i, &position[i], ctrl.joint_position_command_interface_);
            addCommand("velocity", i, &velocity[i], ctrl.joint_velocity_command_interface_);
            addCommand("effort", i, &effort[i], ctrl.joint_force_command_interface_);
        }
        hold = std::make_unique<arms_controller_common::StateHold>(ctrl, node, nullptr);
    }
    template<class Interfaces>
    void addCommand(const std::string& type, size_t i, double* value, Interfaces& interfaces)
    {
        auto command = std::make_unique<hardware_interface::CommandInterface>(
            "joint" + std::to_string(i), type, value);
        auto loan = std::make_unique<hardware_interface::LoanedCommandInterface>(*command);
        interfaces.push_back(std::ref(*loan));
        commands.push_back(std::move(command)); command_loans.push_back(std::move(loan));
    }
    void tick(size_t count = 1)
    {
        for (size_t i = 0; i < count; ++i)
            hold->run(node->now(), rclcpp::Duration::from_seconds(0.002));
    }
    void expectZeroVelocity() { for (double v : velocity) EXPECT_DOUBLE_EQ(v, 0.0); }
    void expectPositions(const std::vector<double>& target)
    {
        for (size_t i = 0; i < 6; ++i) EXPECT_DOUBLE_EQ(position[i], target[i]) << "joint " << i;
    }
    arms_controller_common::CtrlInterfaces ctrl;
    std::array<double, 6> measured{}, position{}, velocity{}, effort{{1,2,3,4,5,6}};
    std::vector<std::unique_ptr<hardware_interface::StateInterface>> states;
    std::vector<std::unique_ptr<hardware_interface::LoanedStateInterface>> state_loans;
    std::vector<std::unique_ptr<hardware_interface::CommandInterface>> commands;
    std::vector<std::unique_ptr<hardware_interface::LoanedCommandInterface>> command_loans;
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
    std::unique_ptr<arms_controller_common::StateHold> hold;
};
TEST_F(HoldOutputTest, ClearsAllVelocityCommandsOnEntry)
{
    hold->enter(); expectZeroVelocity();
}
TEST_F(HoldOutputTest, ClearsNewStaleVelocityEveryCycle)
{
    hold->enter();
    for (size_t tick_id = 0; tick_id < 100; ++tick_id)
    {
        velocity.fill(3.482456); tick(); expectZeroVelocity();
    }
}
TEST_F(HoldOutputTest, KeepsCapturedTargetWhileFeedbackDrifts)
{
    const auto target = ctrl.last_sent_joint_positions_;
    hold->enter(); tick();
    double max_reference_drift = 0.0;
    for (size_t k = 0; k < 1000; ++k)
    {
        for (size_t i = 0; i < 6; ++i) measured[i] = target[i] + (i+1)*0.001*(k+1);
        tick();
        for (size_t i = 0; i < 6; ++i)
            max_reference_drift = std::max(max_reference_drift, std::abs(position[i] - target[i]));
    }
    EXPECT_DOUBLE_EQ(max_reference_drift, 0.0);
    expectPositions(target);
}
TEST_F(HoldOutputTest, StartupFallbackIsResolvedBeforeFirstOutputAndFrozen)
{
    measured[0] += 0.5;
    const std::vector<double> target(measured.begin(), measured.end());
    hold->enter(); expectPositions(target); expectZeroVelocity();
    measured.fill(2.0); tick(10); expectPositions(target);
}
TEST_F(HoldOutputTest, DisabledThresholdsNeverReanchor)
{
    node->set_parameter(rclcpp::Parameter("hold_first_check_position_threshold", 0.0));
    node->set_parameter(rclcpp::Parameter("hold_position_threshold", 0.0));
    const auto target = ctrl.last_sent_joint_positions_; measured.fill(2.0);
    hold->enter(); tick(10); expectPositions(target); expectZeroVelocity();
}
TEST_F(HoldOutputTest, ChangingThresholdsDuringHoldDoesNotReanchor)
{
    const auto target = ctrl.last_sent_joint_positions_; hold->enter(); tick();
    node->set_parameter(rclcpp::Parameter("hold_first_check_position_threshold", 0.001));
    node->set_parameter(rclcpp::Parameter("hold_position_threshold", 0.001));
    measured.fill(2.0); tick(10); expectPositions(target);
}
TEST_F(HoldOutputTest, ReentrySelectsNewTargetOnce)
{
    hold->enter(); tick(); hold->exit();
    measured.fill(1.0); velocity.fill(-2.0);
    const std::vector<double> target(6, 1.0); hold->enter();
    expectZeroVelocity(); expectPositions(target);
    measured.fill(1.5); tick(10); expectPositions(target);
}
TEST_F(HoldOutputTest, PositionOnlyHardwareNeedsNoVelocityInterfaces)
{
    ctrl.joint_velocity_command_interface_.clear();
    const auto target = ctrl.last_sent_joint_positions_;
    hold->enter(); tick(); measured.fill(2.0); tick(10); expectPositions(target);
}
TEST_F(HoldOutputTest, PartialVelocityInterfacesAreCleared)
{
    ctrl.joint_velocity_command_interface_.erase(
        ctrl.joint_velocity_command_interface_.begin() + 2,
        ctrl.joint_velocity_command_interface_.end());
    hold->enter(); EXPECT_DOUBLE_EQ(velocity[0], 0.0); EXPECT_DOUBLE_EQ(velocity[1], 0.0);
    velocity[0] = 2.0; velocity[1] = -3.0; tick();
    EXPECT_DOUBLE_EQ(velocity[0], 0.0); EXPECT_DOUBLE_EQ(velocity[1], 0.0);
}
TEST_F(HoldOutputTest, InvalidMeasuredJointDoesNotPoisonStartupFallback)
{
    const auto previous = ctrl.last_sent_joint_positions_;
    measured[0] += 0.5; measured[1] = std::numeric_limits<double>::quiet_NaN();
    hold->enter(); tick(); EXPECT_DOUBLE_EQ(position[0], measured[0]);
    EXPECT_DOUBLE_EQ(position[1], previous[1]);
}
TEST_F(HoldOutputTest, MixedModeStillClearsVelocityWithoutGravityObject)
{
    ctrl.control_mode_ = arms_controller_common::ControlMode::MIX;
    hold->enter(); tick(); expectZeroVelocity();
    for (size_t i = 0; i < 6; ++i) EXPECT_DOUBLE_EQ(effort[i], i + 1.0);
}
TEST_F(HoldOutputTest, PositionModeClearsVelocityInterfacesIfPresent)
{
    ctrl.control_mode_ = arms_controller_common::ControlMode::POSITION;
    hold->enter(); tick(); expectZeroVelocity();
}
}  // namespace
