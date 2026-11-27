// CompliancePath — COMPLIANCE 内的连续路径：折线 + 圆弧圆角 + 弧长速度曲线。
//
// 一次规划（订阅回调里），回放时每周期只调 sample(t)：无分配、O(段数)。
// 采样同时给出**目标速度**（base 系 6D twist），供位控轴做速度前馈：
//
//     v_pos = v_ff + K·Δx + I
//
// v_ff 是本类解析算出来的（切向 × 弧长速度），不是差分、不依赖消息流。
#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace arms_controller_common::compliance_detail
{
/// 折线上的一个角点（base 系）。
struct PathWaypoint
{
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

struct PathPlanLimits
{
    double blend_radius{0.0};   ///< [m] 圆角半径；0 = 每段独立加减速（角点停到 0）
    double speed{0.05};         ///< [m/s] 轨迹速度上限；duration>0 时可省略
    double accel{0.2};          ///< [m/s²] 切向加减速；duration>0 时可省略
    double lat_accel{0.0};      ///< [m/s²] 圆角向心加速度上限；0 = 不限；仅圆角有效
    double duration{0.0};       ///< [s] 总运行时间（含 loops）；>0 时反解 speed/accel
    /// [m/s] 硬上限：duration 反解后仍然适用，超过则该时长不可实现（直接拒绝）。
    /// 前馈是开环量，规划速度必须与 QP 实际执行速度一致，所以这个上限不能事后夹。
    double max_speed{0.0};      ///< <=0 = 不限
    int loops{1};               ///< 循环趟数；开链路径按**往返**执行（奇数次停在末端）
    bool closed{false};         ///< 闭合（走回第 1 点）；闭合时 loops 为纯循环
};

struct PathSample
{
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    /// 目标速度：线速度 [m/s] + 角速度 [rad/s]，base 系 —— 位控轴前馈用。
    Eigen::Matrix<double, 6, 1> target_twist{Eigen::Matrix<double, 6, 1>::Zero()};
    int loop{0};
};

/// 圆弧圆角 + 整条路径单条梯形速度曲线的回放器。
///
/// 角点 k 的过渡：切线长 t = r·tan(θ/2)，圆弧与相邻直线相切（G1）。
/// 某条边上两个圆角的 t 之和超过该边长的 90% 时，所有角点的 t 同比缩小
/// （等价于整体缩小半径），因此直线段一定保留、对称输入得到对称结果。
/// 共线角（θ≈0）与近反向尖角（θ>170°）不做过渡，保留原折线形状。
class PathPlan
{
public:
    static constexpr std::size_t kMaxSegments = 256;
    static constexpr std::size_t kMaxWaypoints = 128;
    /// 圆角半径小于这个值就不插圆弧（1 µm）。
    static constexpr double kMinRadius = 1.0e-6;
    /// 相邻两个角点的切线长最多占用段长的这个比例。
    static constexpr double kEdgeUse = 0.9;
    /// duration>0 且消息里 speed/accel 缺省（<=0）时的名义值。
    /// accel=0.05 m/s² 对应角点冲过头 ≈ a·τe/K ≈ 0.4 mm
    /// （辨识结果 τe=38.3 ms、外环 K_eff=4.71 1/s）。
    static constexpr double kDefaultSpeed = 0.05;   // [m/s]
    static constexpr double kDefaultAccel = 0.05;   // [m/s²]

    /// 规划路径。返回空字符串表示成功，否则是拒绝原因（直接进日志）。
    std::string build(const std::vector<PathWaypoint>& waypoints,
                      const PathPlanLimits& limits);

    bool valid() const { return segment_count_ > 0 && length_ > 0.0; }
    double length() const { return length_; }                              ///< 单趟 [m]
    double total_length() const { return length_ * static_cast<double>(loops_); }
    double duration() const { return duration_; }                          ///< 含加减速 [s]
    double speed() const { return speed_; }                                ///< 实际速度 [m/s]
    double accel() const { return accel_; }
    double loop_duration() const { return loop_duration_; }              ///< 单趟时长 [s]
    /// true = 每段独立梯形（角点停到 0，无圆角）；false = 整条一条梯形 + 圆角过渡。
    bool stops_at_waypoints() const { return stop_at_waypoints_; }
    double min_corner_radius() const { return min_corner_radius_; }
    std::size_t segment_count() const { return segment_count_; }
    std::size_t corner_count() const { return corner_count_; }
    std::size_t blended_corner_count() const { return blended_corners_; }
    int loops() const { return loops_; }
    bool closed() const { return closed_; }

    /// t ∈ [0, duration]，越界钳到端点。位置/姿态/目标速度都是 base 系。
    void sample(double t, PathSample& out) const;

    Eigen::Vector3d start_position() const;
    Eigen::Quaterniond start_orientation() const;
    Eigen::Vector3d end_position() const;

    /// 梯形速度曲线（与 run_force_xy_path.py 的实现逐步对应）。
    static double trapezoid_duration(double distance, double speed, double accel);
    static double trapezoid_distance(double t, double distance, double speed, double accel);
    static double trapezoid_speed(double t, double distance, double speed, double accel);

private:
    struct Segment
    {
        double length{0.0};
        int kind{0};                       ///< 0 = 直线, 1 = 圆弧

        Eigen::Vector3d p0{Eigen::Vector3d::Zero()};      ///< 直线
        Eigen::Vector3d p1{Eigen::Vector3d::Zero()};

        Eigen::Vector3d center{Eigen::Vector3d::Zero()};  ///< 圆弧
        Eigen::Vector3d a0{Eigen::Vector3d::UnitX()};
        Eigen::Vector3d axis{Eigen::Vector3d::UnitZ()};
        double radius{0.0};
        double sweep{0.0};

        Eigen::Quaterniond q0{Eigen::Quaterniond::Identity()};
        Eigen::Quaterniond q1{Eigen::Quaterniond::Identity()};
        /// 段内角速度（rad/m）：直线段按 slerp 均分，圆弧段恒为 0。
        Eigen::Vector3d omega_per_meter{Eigen::Vector3d::Zero()};
    };

    /// 弧长 s[0, length] 定位到 (段, 段内比例)。
    void locate(double s, std::size_t& index, double& fraction) const;

    std::array<Segment, kMaxSegments> segments_{};
    std::size_t segment_count_{0};

    double length_{0.0};
    double speed_{0.0};
    double accel_{0.0};
    double duration_{0.0};
    /// true = 每段各自一条梯形曲线（段末速度归零）；false = 整条路径一条梯形。
    bool stop_at_waypoints_{false};
    /// 每段起始时刻 [s]（单趟）。尾哨兵 seg_t0_[segment_count_] = loop_duration_。
    std::array<double, kMaxSegments + 1> seg_t0_{};
    /// 单趟时长 [s]（含段间停到 0 的全部加速/减速）。
    double loop_duration_{0.0};
    double min_corner_radius_{0.0};
    std::size_t corner_count_{0};
    std::size_t blended_corners_{0};
    int loops_{1};
    bool closed_{false};
};
}  // namespace arms_controller_common::compliance_detail
