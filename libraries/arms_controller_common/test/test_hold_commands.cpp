#include <gtest/gtest.h>
#include <arms_controller_common/CtrlInterfaces.h>
#include <hardware_interface/handle.hpp>
#include <memory>

TEST(HoldCommands, WaitingForPolicyClearsVelocityAndDerivativeSeed)
{
    arms_controller_common::CtrlInterfaces ctrl;
    double q_cmd = -4.0, dq_cmd = 3.482456;
    hardware_interface::CommandInterface q("joint", "position", &q_cmd);
    hardware_interface::CommandInterface dq("joint", "velocity", &dq_cmd);
    hardware_interface::LoanedCommandInterface q_loan(q), dq_loan(dq);
    ctrl.joint_position_command_interface_.push_back(std::ref(q_loan));
    ctrl.joint_velocity_command_interface_.push_back(std::ref(dq_loan));
    ctrl.last_sent_joint_positions_ = {0.7};
    ctrl.recordCommandMotion(0.002);
    ctrl.setCommandDerivatives(0, 3.482456, 10.0);
    // Exact shared output primitive used by CtrlComponent::holdLastSentPositions().
    ctrl.setJointHoldCommands(ctrl.last_sent_joint_positions_);
    ctrl.recordCommandMotion(0.002);
    EXPECT_DOUBLE_EQ(q_cmd, 0.7); EXPECT_DOUBLE_EQ(dq_cmd, 0.0);
    const auto motion = ctrl.getCommandMotionState();
    ASSERT_EQ(motion.velocities.size(), 1u);
    EXPECT_DOUBLE_EQ(motion.velocities[0], 0.0);
    EXPECT_DOUBLE_EQ(motion.accelerations[0], 0.0);
}
TEST(HoldCommands, ClearsEveryVelocityIncludingUnmatchedOptionalChannels)
{
    arms_controller_common::CtrlInterfaces ctrl;
    double v0 = 1.0, v1 = -2.0;
    hardware_interface::CommandInterface a("a", "velocity", &v0), b("b", "velocity", &v1);
    hardware_interface::LoanedCommandInterface la(a), lb(b);
    ctrl.joint_velocity_command_interface_ = {std::ref(la), std::ref(lb)};
    ctrl.setJointHoldCommands({});
    EXPECT_DOUBLE_EQ(v0, 0.0); EXPECT_DOUBLE_EQ(v1, 0.0);
}
