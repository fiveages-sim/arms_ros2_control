#include <gtest/gtest.h>
#include "arms_controller_common/FSM/ComplianceSupport.h"

using namespace arms_controller_common::compliance_detail;

TEST(ComplianceWrench, WorldOffsetAndRotationDoNotChangeControlWrench)
{
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer(clock);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "world";
    tf.child_frame_id = "control_base";
    tf.transform.translation.x = 3.0;
    tf.transform.translation.y = -2.0;
    tf.transform.rotation.z = std::sin(0.7);
    tf.transform.rotation.w = std::cos(0.7);
    ASSERT_TRUE(buffer.setTransform(tf, "test", true));
    tf.header.frame_id = "control_base";
    tf.child_frame_id = "sensor";
    tf.transform.translation.x = 0.2;
    tf.transform.translation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;
    ASSERT_TRUE(buffer.setTransform(tf, "test", true));
    Eigen::Matrix<double, 6, 1> wrench;
    wrench << 0, 0, 10, 0, 0, 0;
    const auto result = transformWrench(buffer, "control_base", "sensor",
                                       Eigen::Vector3d(0.1, 0, 0), wrench);
    Eigen::Matrix<double, 6, 1> expected;
    expected << 0, 0, 10, 0, -1, 0;
    EXPECT_TRUE(result.isApprox(expected, 1e-12));
    // Missing control transform must not silently fall back to world/base_link.
    EXPECT_THROW(transformWrench(buffer, "missing", "sensor",
                                 Eigen::Vector3d::Zero(), wrench), tf2::TransformException);
}

TEST(ComplianceWrench, RotatesSensorForceAndTorque)
{
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer(clock);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = "base";
    tf.child_frame_id = "sensor";
    tf.transform.rotation.z = std::sqrt(0.5);
    tf.transform.rotation.w = std::sqrt(0.5);
    ASSERT_TRUE(buffer.setTransform(tf, "test", true));
    Eigen::Matrix<double, 6, 1> wrench, expected;
    wrench << 10, 0, 0, 2, 0, 0;
    expected << 0, 10, 0, 0, 2, 0;
    EXPECT_TRUE(transformWrench(buffer, "base", "sensor", Eigen::Vector3d::Zero(),
                               wrench).isApprox(expected, 1e-12));
}

TEST(ComplianceAlignment, ContactLossClearsIntegralAndSmoothlyReleasesBias)
{
    AlignmentState state;
    state.integral << 5, -3, 2;
    state.bias << 0.2, 0, 0;
    state.release(1.0, 0.002);
    EXPECT_TRUE(state.integral.isZero());
    EXPECT_NEAR(state.bias.x(), 0.1996, 1e-12);
    for (int i = 0; i < 2500; ++i) state.release(1.0, 0.002);
    EXPECT_LT(state.bias.norm(), 0.002);
    EXPECT_TRUE(state.integral.isZero()); // no old torque drive on re-contact
}

TEST(ComplianceAlignment, ToggleResetRemovesPreviousSessionState)
{
    AlignmentState state;
    state.integral.setConstant(5);
    state.bias.setConstant(0.1);
    state.reset();
    EXPECT_TRUE(state.integral.isZero());
    EXPECT_TRUE(state.bias.isZero());
}
