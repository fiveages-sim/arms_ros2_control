#pragma once

#include <algorithm>
#include <Eigen/Geometry>
#include <tf2_ros/buffer.h>

namespace arms_controller_common::compliance_detail
{
// Both origins and the returned wrench are expressed in the control frame.
inline Eigen::Matrix<double, 6, 1> transformWrench(
    tf2_ros::Buffer& buffer, const std::string& control_frame,
    const std::string& sensor_frame, const Eigen::Vector3d& tcp_position,
    const Eigen::Matrix<double, 6, 1>& sensor_wrench)
{
    const auto tf = buffer.lookupTransform(control_frame, sensor_frame, tf2::TimePointZero);
    const auto& q = tf.transform.rotation;
    const auto& p = tf.transform.translation;
    const Eigen::Matrix3d rotation = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
    const Eigen::Vector3d force = rotation * sensor_wrench.head<3>();
    Eigen::Matrix<double, 6, 1> result;
    result << force, rotation * sensor_wrench.tail<3>() +
        (Eigen::Vector3d(p.x, p.y, p.z) - tcp_position).cross(force);
    return result;
}

// Alignment owns its integral; it must never borrow the force-axis integrator.
struct AlignmentState
{
    Eigen::Vector3d bias{Eigen::Vector3d::Zero()};
    Eigen::Vector3d integral{Eigen::Vector3d::Zero()};

    void reset() { bias.setZero(); integral.setZero(); }
    void release(double rate, double dt)
    {
        integral.setZero();
        bias *= std::max(0.0, 1.0 - rate * dt);
    }
};
}
