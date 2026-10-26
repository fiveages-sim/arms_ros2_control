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

// A unique translational force axis defines the alignment normal in the control
// frame. Multiple selected axes do not define a unique surface normal.
inline int alignmentNormalAxis(const Eigen::Vector3d& selection)
{
    int axis = -1;
    for (int k = 0; k < 3; ++k)
    {
        if (selection(k) <= 0.5) continue;
        if (axis >= 0) return -1;
        axis = k;
    }
    return axis;
}

inline void constrainAlignment(Eigen::Vector3d& value, int normal_axis)
{
    if (normal_axis >= 0 && normal_axis < 3) value(normal_axis) = 0.0;
    else value.setZero();
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

// 阻尼最小二乘（DLS）：qdot = Jᵀ(JJᵀ + λ²I)⁻¹ v。
//
// 这是**无约束**解：只提供奇异方向衰减（σ²/(σ²+λ²) 平滑下降），不处理速度盒
// ——关节速度上限、URDF 限位软减速、6/7 轴耦合边界都不在它的输入里。若拿它当
// 唯一求解器，速度盒只能由调用者事后 std::clamp 截断，而截断会改变末端速度
// 方向（顶到限位后表现为 "v 非零、vach≈0"、sat 常驻、手腕顿挫）。
//
// 因此它只保留作对照/调试用。需要限位的单臂求解请用 solveBoxQp()：变量只有该臂
// 关节（n=nq），不需要联合求解。注意"分体模式"缺的是腰（不需要联合），不是盒。
inline Eigen::VectorXd solveDls(const Eigen::MatrixXd& J, const Eigen::VectorXd& v,
                               double lambda)
{
    if (J.rows() <= 0 || J.cols() <= 0) return Eigen::VectorXd::Zero(std::max<Eigen::Index>(0, J.cols()));
    const Eigen::MatrixXd JJt = J * J.transpose();
    const Eigen::MatrixXd reg = JJt + lambda * lambda *
        Eigen::MatrixXd::Identity(JJt.rows(), JJt.cols());
    return J.transpose() * reg.ldlt().solve(v);
}

// 盒约束加权最小二乘（坐标下降，精确到 1e-9）：
//   min 0.5·xᵀ(JᵀJ + diag(λ)²)x − (Jᵀv)ᵀx
//   s.t. −vmax_dn ≤ x ≤ vmax_up
// 两种用法共用同一个求解器：
//   分体模式：x 只含单臂关节（n=nq），盒即该臂的关节速度/URDF 限位软减速；
//   全身模式：x 含腰部与双臂，共享的腰部列让两臂的任务在同一个问题里竞争，
//             分配由 λ（正则强度）与速度盒共同决定。
// 对 H 的条件数不敏感（坐标下降逐坐标精确投影，不依赖整体矩阵求逆）。
inline Eigen::VectorXd solveBoxQp(const Eigen::MatrixXd& J, const Eigen::VectorXd& v,
                                  const Eigen::VectorXd& lambda,
                                  const Eigen::VectorXd& vmax_up,
                                  const Eigen::VectorXd& vmax_dn)
{
    const Eigen::Index n = J.cols();
    if (n <= 0 || J.rows() <= 0) return Eigen::VectorXd::Zero(std::max<Eigen::Index>(0, n));

    Eigen::MatrixXd H = J.transpose() * J;
    for (Eigen::Index i = 0; i < n && i < lambda.size(); ++i)
        H(i, i) += lambda(i) * lambda(i);
    const Eigen::VectorXd c = -J.transpose() * v;
    const Eigen::VectorXd lo = -vmax_dn;
    const Eigen::VectorXd hi = vmax_up;

    Eigen::VectorXd x = H.ldlt().solve(-c);
    if (!x.allFinite()) x.setZero(n);
    x = x.cwiseMax(lo).cwiseMin(hi);

    for (int sweep = 0; sweep < 60; ++sweep)
    {
        double delta = 0.0;
        for (Eigen::Index i = 0; i < n; ++i)
        {
            double r = -c(i);
            for (Eigen::Index j = 0; j < n; ++j)
                if (j != i) r -= H(i, j) * x(j);
            const double xi = std::clamp(r / H(i, i), lo(i), hi(i));
            delta = std::max(delta, std::abs(xi - x(i)));
            x(i) = xi;
        }
        if (delta < 1e-9) break;
    }
    return x;
}
}
