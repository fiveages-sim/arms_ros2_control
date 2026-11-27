#include "arms_controller_common/FSM/CompliancePath.h"

#include <algorithm>
#include <cmath>

namespace arms_controller_common::compliance_detail
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

Eigen::Vector3d unitOrZero(const Eigen::Vector3d& v)
{
    const double n = v.norm();
    return n > 1e-12 ? Eigen::Vector3d(v / n) : Eigen::Vector3d::Zero();
}

const char* kTooManySegments = "路径段数超过上限（角点太多或边太短）";
}  // namespace

double PathPlan::trapezoid_duration(double distance, double speed, double accel)
{
    if (distance <= 0.0 || speed <= 0.0) return 0.0;
    accel = std::max(accel, 1e-6);
    if (distance >= speed * speed / accel) return distance / speed + speed / accel;
    return 2.0 * std::sqrt(distance / accel);
}

double PathPlan::trapezoid_distance(double t, double distance, double speed, double accel)
{
    if (distance <= 0.0 || speed <= 0.0 || t <= 0.0) return 0.0;
    accel = std::max(accel, 1e-6);
    if (distance >= speed * speed / accel)
    {
        const double t_acc = speed / accel;
        const double d_acc = 0.5 * accel * t_acc * t_acc;
        if (t <= t_acc) return 0.5 * accel * t * t;
        const double t_cruise_end = t_acc + (distance - 2.0 * d_acc) / speed;
        if (t <= t_cruise_end) return d_acc + speed * (t - t_acc);
        const double t_dec = t - t_cruise_end;
        if (t_dec >= t_acc) return distance;
        return distance - d_acc + speed * t_dec - 0.5 * accel * t_dec * t_dec;
    }
    const double t_acc = std::sqrt(distance / accel);  // 加速到一半就减速
    if (t <= t_acc) return 0.5 * accel * t * t;
    if (t >= 2.0 * t_acc) return distance;
    const double t_dec = t - t_acc;
    return 0.5 * accel * t_acc * t_acc + accel * t_acc * t_dec - 0.5 * accel * t_dec * t_dec;
}

double PathPlan::trapezoid_speed(double t, double distance, double speed, double accel)
{
    if (distance <= 0.0 || speed <= 0.0 || t <= 0.0) return 0.0;
    accel = std::max(accel, 1e-6);
    if (distance >= speed * speed / accel)
    {
        const double t_acc = speed / accel;
        if (t <= t_acc) return accel * t;
        const double d_acc = 0.5 * accel * t_acc * t_acc;
        const double t_cruise_end = t_acc + (distance - 2.0 * d_acc) / speed;
        if (t <= t_cruise_end) return speed;
        const double t_dec = t - t_cruise_end;
        if (t_dec >= t_acc) return 0.0;
        return speed - accel * t_dec;
    }
    const double t_acc = std::sqrt(distance / accel);
    if (t <= t_acc) return accel * t;
    if (t >= 2.0 * t_acc) return 0.0;
    return accel * (2.0 * t_acc - t);
}

std::string PathPlan::build(const std::vector<PathWaypoint>& waypoints,
                            const PathPlanLimits& limits)
{
    *this = PathPlan{};

    if (!std::isfinite(limits.speed) || limits.speed < 0.0) return "speed 不能为负";
    if (!std::isfinite(limits.accel) || limits.accel < 0.0) return "accel 不能为负";
    if (!std::isfinite(limits.blend_radius) || limits.blend_radius < 0.0)
        return "blend_radius 必须 >= 0";
    if (!std::isfinite(limits.lat_accel) || limits.lat_accel < 0.0)
        return "lat_accel 必须 >= 0";
    if (!std::isfinite(limits.duration) || limits.duration < 0.0)
        return "duration 必须 >= 0";

    closed_ = limits.closed;
    loops_ = std::max(1, limits.loops);

    // 去重：相邻重合点只保留一个；闭合时若首点被写了两遍则折掉尾部
    std::vector<PathWaypoint> pts;
    pts.reserve(waypoints.size());
    for (const auto& wp : waypoints)
    {
        if (!wp.position.allFinite() || !wp.orientation.coeffs().allFinite())
            return "角点含非有限值";
        // 零四元数 normalize() 之后是 NaN，会一路污染到下发指令 —— 提前拦掉。
        if (wp.orientation.coeffs().norm() < 1e-6)
            return "角点四元数长度为 0（应给单位四元数 0,0,0,1）";
        PathWaypoint item = wp;
        item.orientation.normalize();
        if (!pts.empty() && (item.position - pts.back().position).norm() < 1e-6)
        {
            pts.back().orientation = item.orientation;
            continue;
        }
        pts.push_back(item);
    }
    if (closed_ && pts.size() >= 2 &&
        (pts.front().position - pts.back().position).norm() < 1e-6)
        pts.pop_back();

    if (pts.size() < 2) return "角点去重后不足 2 个";
    if (closed_ && pts.size() < 3) return "闭合路径至少需要 3 个角点";
    if (pts.size() > kMaxWaypoints) return "角点太多（上限 128）";

    const std::size_t n = pts.size();
    std::vector<Eigen::Vector3d> dir_in(n, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> dir_out(n, Eigen::Vector3d::Zero());
    std::vector<double> turn(n, 0.0);
    std::vector<double> tangent(n, 0.0);
    std::vector<double> radii(n, 0.0);

    if (limits.blend_radius > 0.0)
    {
        for (std::size_t k = 0; k < n; ++k)
        {
            if (!closed_ && (k == 0 || k + 1 == n)) continue;  // 开链两端不圆角
            const Eigen::Vector3d u_in =
                unitOrZero(pts[k].position - pts[(k + n - 1) % n].position);
            const Eigen::Vector3d u_out =
                unitOrZero(pts[(k + 1) % n].position - pts[k].position);
            if (u_in.isZero(1e-9) || u_out.isZero(1e-9)) continue;
            const double theta = std::acos(std::clamp(u_in.dot(u_out), -1.0, 1.0));
            if (theta < 1e-3 || theta > 170.0 * kPi / 180.0) continue;  // 共线 / 近反向
            dir_in[k] = u_in;
            dir_out[k] = u_out;
            turn[k] = theta;
            tangent[k] = limits.blend_radius * std::tan(0.5 * theta);
        }

        // 段长约束：两个角点的切线长之和不得超过边长的 kEdgeUse。
        // 取一个全局比例 —— 对称输入得到对称结果，且直线段一定保留。
        double scale = 1.0;
        const std::size_t edges = closed_ ? n : n - 1;
        for (std::size_t k = 0; k < edges; ++k)
        {
            const std::size_t nxt = (k + 1) % n;
            const double total = tangent[k] + tangent[nxt];
            if (total <= 0.0) continue;
            const double edge = (pts[nxt].position - pts[k].position).norm();
            scale = std::min(scale, kEdgeUse * edge / total);
        }
        scale = std::clamp(scale, 0.0, 1.0);
        for (std::size_t k = 0; k < n; ++k)
        {
            tangent[k] *= scale;
            if (tangent[k] <= 1e-12)
            {
                tangent[k] = 0.0;
                dir_in[k].setZero();
                dir_out[k].setZero();
                turn[k] = 0.0;
                continue;
            }
            const double r = tangent[k] / std::tan(0.5 * turn[k]);
            if (r < kMinRadius)
            {
                tangent[k] = 0.0;
                dir_in[k].setZero();
                dir_out[k].setZero();
                turn[k] = 0.0;
            }
            else
            {
                radii[k] = r;
            }
        }
    }

    std::vector<Eigen::Vector3d> entry(n, Eigen::Vector3d::Zero());
    std::vector<Eigen::Vector3d> exit(n, Eigen::Vector3d::Zero());
    for (std::size_t k = 0; k < n; ++k)
    {
        entry[k] = tangent[k] > 0.0 ? pts[k].position - dir_in[k] * tangent[k]
                                    : pts[k].position;
        exit[k] = tangent[k] > 0.0 ? pts[k].position + dir_out[k] * tangent[k]
                                   : pts[k].position;
    }

    auto add_line = [&](const Eigen::Vector3d& p0, const Eigen::Vector3d& p1,
                        const Eigen::Quaterniond& q0, const Eigen::Quaterniond& q1) -> bool {
        const double len = (p1 - p0).norm();
        if (len <= 1e-9) return true;  // 两段圆角贴在一起：直接跳过零长直线
        if (segment_count_ >= kMaxSegments) return false;
        Segment& seg = segments_[segment_count_++];
        seg.length = len;
        seg.kind = 0;
        seg.p0 = p0;
        seg.p1 = p1;
        seg.q0 = q0;
        seg.q1 = q1;
        // 段内姿态按 slerp 均分 → 角速度 = 轴 × (转角 / 段长)，与采样严格一致
        Eigen::Quaterniond rel = q1 * q0.conjugate();
        if (rel.w() < 0.0) rel.coeffs() *= -1.0;
        const double vec_norm = rel.vec().norm();
        const double angle = 2.0 * std::atan2(vec_norm, rel.w());
        if (angle > 1e-9 && vec_norm > 1e-12)
            seg.omega_per_meter = rel.vec() / vec_norm * (angle / len);
        return true;
    };

    auto add_arc = [&](std::size_t k) -> bool {
        if (segment_count_ >= kMaxSegments) return false;
        const Eigen::Vector3d u_in = dir_in[k];
        const Eigen::Vector3d u_out = dir_out[k];
        const double theta = turn[k];
        const double r = radii[k];
        const Eigen::Vector3d bisector = unitOrZero(u_out - u_in);
        const Eigen::Vector3d center =
            pts[k].position + bisector * (r / std::sin(0.5 * (kPi - theta)));
        Segment& seg = segments_[segment_count_++];
        seg.length = r * theta;
        seg.kind = 1;
        seg.center = center;
        seg.radius = r;
        seg.sweep = theta;
        seg.a0 = unitOrZero(entry[k] - center);
        seg.axis = unitOrZero(u_in.cross(u_out));
        seg.q0 = pts[k].orientation;
        seg.q1 = pts[k].orientation;
        return true;
    };

    if (closed_)
    {
        if (tangent[0] > 0.0 && !add_arc(0)) return kTooManySegments;
        Eigen::Vector3d cur = exit[0];
        for (std::size_t k = 1; k < n; ++k)
        {
            if (!add_line(cur, entry[k], pts[k - 1].orientation, pts[k].orientation))
                return kTooManySegments;
            if (tangent[k] > 0.0 && !add_arc(k)) return kTooManySegments;
            cur = exit[k];
        }
        if (!add_line(cur, entry[0], pts[n - 1].orientation, pts[0].orientation))
            return kTooManySegments;
    }
    else
    {
        Eigen::Vector3d cur = pts[0].position;
        for (std::size_t k = 1; k < n; ++k)
        {
            if (!add_line(cur, entry[k], pts[k - 1].orientation, pts[k].orientation))
                return kTooManySegments;
            if (tangent[k] > 0.0 && !add_arc(k)) return kTooManySegments;
            cur = exit[k];
        }
    }

    for (std::size_t i = 0; i < segment_count_; ++i) length_ += segments_[i].length;
    if (!(length_ > 1e-9)) return "路径长度为 0";

    for (std::size_t k = 0; k < n; ++k)
    {
        if (!closed_ && (k == 0 || k + 1 == n)) continue;
        ++corner_count_;
        if (radii[k] > 0.0)
        {
            ++blended_corners_;
            min_corner_radius_ = min_corner_radius_ > 0.0
                                     ? std::min(min_corner_radius_, radii[k])
                                     : radii[k];
        }
    }

    // ── 速度规划 ──────────────────────────────────────────────────────
    // blend_radius <= 0：每段各自一条梯形曲线，段末速度归零 —— 角点停到 0
    //   再转向，速度方向不突变、法向加速度恒为 0。走折线用这个。
    // blend_radius >  0：保持原行为，整条路径一条梯形 + 圆弧过渡。
    stop_at_waypoints_ = !(limits.blend_radius > 0.0);

    const bool by_duration = std::isfinite(limits.duration) && limits.duration > 0.0;
    double v_nom = limits.speed;
    double a_nom = limits.accel;
    if (by_duration)
    {
        // 给总时长时可以省略 speed/accel：用名义值起步，下面的等比缩放再调
        if (!(v_nom > 0.0)) v_nom = kDefaultSpeed;
        if (!(a_nom > 0.0)) a_nom = kDefaultAccel;
    }
    else
    {
        if (!(v_nom > 0.0)) return "speed 必须 > 0（或改给 duration）";
        if (!(a_nom > 0.0)) return "accel 必须 > 0（或改给 duration）";
    }

    // lat_accel 只约束圆角处的向心加速度，无圆角时无意义
    if (limits.lat_accel > 0.0 && min_corner_radius_ > 0.0)
        v_nom = std::min(v_nom, std::sqrt(limits.lat_accel * min_corner_radius_));
    if (!(v_nom > 1e-9)) return "速度上限为 0（lat_accel 与圆角半径不匹配）";

    // 单趟时长（给定 v/a 时）
    auto loop_time = [&](double v, double a) {
        if (!stop_at_waypoints_) return trapezoid_duration(length_, v, a);
        double t = 0.0;
        for (std::size_t i = 0; i < segment_count_; ++i)
            t += trapezoid_duration(segments_[i].length, v, a);
        return t;
    };

    // 时长控制：v→v/k、a→a/k² 时距离阈值 speed²/accel 不变，梯形形状（是否
    // 进入三角区）不变，每段时长严格变成 k 倍 —— 所以总时长也严格 k 倍。
    // 一次除法即可命中，不需要迭代。
    if (by_duration)
    {
        const double t_nom = loop_time(v_nom, a_nom);
        if (!(t_nom > 1e-12)) return "路径时长为 0（路径太短）";
        const double k = limits.duration / static_cast<double>(loops_) / t_nom;
        if (!(k > 1e-9)) return "duration 过小";
        v_nom /= k;
        a_nom /= (k * k);

        // 时长反解会把速度抬到任意值，必须再检查一次硬上限：
        // 前馈是开环量，规划速度必须与 QP 实际执行速度一致，不能事后夹。
        if (limits.max_speed > 0.0 && v_nom > limits.max_speed * (1.0 + 1e-9))
        {
            const double k_min = v_nom / limits.max_speed;
            const double t_min =
                loop_time(v_nom / k_min, a_nom / (k_min * k_min)) *
                static_cast<double>(loops_);
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                          "duration %.2f s 太短：这条路径最短需要 %.2f s"
                          "（速度上限 %.0f mm/s，按此时长反解需 %.0f mm/s）",
                          limits.duration, t_min, limits.max_speed * 1000.0,
                          v_nom * 1000.0);
            return std::string(buf);
        }
    }

    if (!stop_at_waypoints_ && !closed_ && loops_ > 1)
    {
        // 整条路径一条梯形 → 圈边界处速度不为 0，无法掉头；旧实现会瞬移回起点。
        return "整条梯形（blend_radius>0）不支持开链多圈：会瞬移回起点；"
               "请用 blend_radius=0（每段停车，按往返执行）或设为 closed";
    }

    speed_ = v_nom;
    accel_ = a_nom;

    if (stop_at_waypoints_)
    {
        double t = 0.0;
        for (std::size_t i = 0; i < segment_count_; ++i)
        {
            seg_t0_[i] = t;
            t += trapezoid_duration(segments_[i].length, speed_, accel_);
        }
        seg_t0_[segment_count_] = t;   // 尾哨兵：单趟结束
        loop_duration_ = t;
        duration_ = loop_duration_ * static_cast<double>(loops_);
    }
    else
    {
        loop_duration_ = trapezoid_duration(length_, speed_, accel_);
        duration_ = trapezoid_duration(total_length(), speed_, accel_);
    }
    if (!(duration_ > 0.0)) return "路径时长为 0";
    return {};
}

void PathPlan::locate(double s, std::size_t& index, double& fraction) const
{
    double travelled = 0.0;
    for (std::size_t i = 0; i < segment_count_; ++i)
    {
        const double len = segments_[i].length;
        if (s <= travelled + len + 1e-12 || i + 1 == segment_count_)
        {
            index = i;
            const double local = std::clamp(s - travelled, 0.0, len);
            fraction = len > 1e-12 ? local / len : 0.0;
            return;
        }
        travelled += len;
    }
    index = segment_count_ > 0 ? segment_count_ - 1 : 0;
    fraction = 1.0;
}

void PathPlan::sample(double t, PathSample& out) const
{
    out = PathSample{};
    if (!valid()) return;

    std::size_t index = 0;
    double fraction = 0.0;
    double speed_now = 0.0;
    int loop = loops_ - 1;

    if (stop_at_waypoints_)
    {
        // 每段独立梯形：先按时刻定位段，再在段内解梯形。
        const double tc = std::clamp(t, 0.0, duration_);
        loop = loop_duration_ > 1e-12
                   ? std::clamp(static_cast<int>(tc / loop_duration_), 0, loops_ - 1)
                   : 0;
        double t_local = tc - static_cast<double>(loop) * loop_duration_;

        // 开链路径多圈 = 往返：奇数圈反向走。每段末速度都是 0（本轮就是
        // 按段独立梯形规划的），所以换向点两侧都静止，位置与速度都连续。
        // 旧实现每圈从 t_local=0 重来，等于在终点瞬移回起点。
        const bool reverse = !closed_ && (loop % 2 == 1);
        if (reverse) t_local = std::max(loop_duration_ - t_local, 0.0);

        for (std::size_t i = 0; i < segment_count_ - 1; ++i)
        {
            if (t_local < seg_t0_[i + 1]) { index = i; break; }
            index = i + 1;
        }
        const double tau = std::max(t_local - seg_t0_[index], 0.0);
        const double len = segments_[index].length;
        fraction = len > 1e-12
                       ? std::clamp(trapezoid_distance(tau, len, speed_, accel_) / len, 0.0, 1.0)
                       : 1.0;
        speed_now = trapezoid_speed(tau, len, speed_, accel_);
        if (reverse) speed_now = -speed_now;
    }
    else
    {
        const double total = total_length();
        const double s = trapezoid_distance(t, total, speed_, accel_);
        speed_now = trapezoid_speed(t, total, speed_, accel_);

        double s_local = length_;
        if (s < total - 1e-9)
        {
            s_local = std::fmod(std::max(s, 0.0), length_);
            loop = static_cast<int>(std::max(s, 0.0) / length_);
            loop = std::clamp(loop, 0, loops_ - 1);
        }

        locate(std::clamp(s_local, 0.0, length_), index, fraction);
    }
    if (segment_count_ == 0) return;

    const Segment& seg = segments_[index];
    if (seg.kind == 0)
    {
        out.position = seg.p0 + (seg.p1 - seg.p0) * fraction;
        out.orientation = seg.q0.slerp(fraction, seg.q1).normalized();
        const Eigen::Vector3d tangent = unitOrZero(seg.p1 - seg.p0);
        out.target_twist.head<3>() = tangent * speed_now;
        out.target_twist.tail<3>() = seg.omega_per_meter * speed_now;
    }
    else
    {
        const double angle = seg.sweep * fraction;
        const Eigen::Vector3d perp = seg.axis.cross(seg.a0);
        out.position =
            seg.center + (seg.a0 * std::cos(angle) + perp * std::sin(angle)) * seg.radius;
        out.orientation = seg.q0;
        const Eigen::Vector3d tangent =
            unitOrZero(-seg.a0 * std::sin(angle) + perp * std::cos(angle));
        out.target_twist.head<3>() = tangent * speed_now;
        out.target_twist.tail<3>().setZero();
    }
    out.loop = loop;
}

Eigen::Vector3d PathPlan::start_position() const
{
    PathSample s;
    sample(0.0, s);
    return s.position;
}

Eigen::Quaterniond PathPlan::start_orientation() const
{
    PathSample s;
    sample(0.0, s);
    return s.orientation;
}

Eigen::Vector3d PathPlan::end_position() const
{
    PathSample s;
    sample(duration_, s);
    return s.position;
}
}  // namespace arms_controller_common::compliance_detail
