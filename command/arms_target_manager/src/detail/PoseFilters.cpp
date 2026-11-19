//
// PoseFilters 实现：冻结追赶、一阶低通、姿态分解、RPY 转旋转矩阵。
//
// 关键点：
//   * StaleCatchUpFilter 把“冻结恢复后的那一次跳变”分成 N-1 帧摊平，
//     输入每帧都有变化时完全不干预；
//   * PoseLowPass 第一帧直接取输入作初值，不会从零缓慢收敛；
//   * dropRoll 只保留偏航与俯仰。
//
// 注意：
//   纯计算，不访问 ROS；可以直接写单元测试。
//
#include "arms_target_manager/detail/PoseFilters.h"

#include <algorithm>
#include <cmath>

namespace arms_ros2_control::command::detail
{
    /**
     * @brief 冻结追赶滤波
     *
     * 上游链路冻结时会逐位重发上一帧，恢复后一帧补齐整段位移（实测可达上百毫米）。
     * 这里识别“连续 N 帧输入逐位相同”，把恢复时那一次跳变量分成 N-1 帧摊平。
     * 正常运动（输入有变化）不受影响。
     *
     * @param position 输入/输出位置，原地改写
     * @param orientation 输入/输出姿态，原地改写
     * @return true 表示当前仍处于摊平阶段（输出与输入不相等）
     */
    bool StaleCatchUpFilter::apply(Eigen::Vector3d& position, Eigen::Quaterniond& orientation)
    {
        State& s = state_;
        const Eigen::Vector3d input_position = position;
        const Eigen::Quaterniond input_orientation = orientation;

        if (!s.has_previous_input)
        {
            s.has_previous_input = true;
            s.previous_input_position = input_position;
            s.previous_input_orientation = input_orientation;
            s.output_position = input_position;
            s.output_orientation = input_orientation;
            return false;
        }

        const bool identical =
            input_position == s.previous_input_position &&
            input_orientation.coeffs() == s.previous_input_orientation.coeffs();

        s.previous_input_position = input_position;
        s.previous_input_orientation = input_orientation;

        bool armed_this_frame = false;
        if (identical)
        {
            ++s.frozen_frames;
        }
        else
        {
            if (s.frozen_frames >= min_frozen_frames_)
            {
                const int spread = s.frozen_frames;
                const double keep =
                    static_cast<double>(spread - 1) / static_cast<double>(spread);

                Eigen::Quaterniond aligned = input_orientation;
                if (aligned.dot(s.output_orientation) < 0.0)
                {
                    aligned.coeffs() = -aligned.coeffs();
                }

                const Eigen::Vector3d gap = input_position - s.output_position;
                s.residual_position = gap * keep;
                s.residual_orientation = Eigen::Quaterniond::Identity().slerp(
                    keep, (s.output_orientation.conjugate() * aligned).normalized());
                s.remaining_frames = spread - 1;
                armed_this_frame = true;
            }
            s.frozen_frames = 0;
        }

        if (!armed_this_frame && s.remaining_frames > 0)
        {
            const double keep = static_cast<double>(s.remaining_frames - 1) /
                static_cast<double>(s.remaining_frames);
            s.residual_position *= keep;
            s.residual_orientation =
                Eigen::Quaterniond::Identity().slerp(keep, s.residual_orientation);
            --s.remaining_frames;
            if (s.remaining_frames == 0)
            {
                s.residual_position.setZero();
                s.residual_orientation.setIdentity();
            }
        }

        if (s.remaining_frames > 0)
        {
            Eigen::Quaterniond aligned = input_orientation;
            if (aligned.dot(s.output_orientation) < 0.0)
            {
                aligned.coeffs() = -aligned.coeffs();
            }
            position = input_position - s.residual_position;
            orientation = (aligned * s.residual_orientation.conjugate()).normalized();
        }

        s.output_position = position;
        s.output_orientation = orientation;
        return s.remaining_frames > 0;
    }

    /**
     * @brief 一阶低通滤波
     *
     * 第一帧直接作为初值（不产生向零点的收敛过程），之后按 alpha 系数跟随。
     * alpha 越大越跟随，越小越平滑。
     *
     * @param position 输入/输出位置，原地改写
     * @param orientation 输入/输出姿态，原地改写（走球面插值）
     */
    void PoseLowPass::apply(Eigen::Vector3d& position, Eigen::Quaterniond& orientation)
    {
        if (!primed_)
        {
            position_ = position;
            orientation_ = orientation.normalized();
            primed_ = true;
            return;
        }
        position_ += alpha_ * (position - position_);
        orientation_ = orientation_.slerp(alpha_, orientation.normalized()).normalized();
        position = position_;
        orientation = orientation_;
    }

    /**
     * @brief 去掉姿态里的横滚分量，只保留偏航与俯仰
     * @param q 原始姿态
     * @return 横滚被置零后的姿态
     */
    Eigen::Quaterniond dropRoll(const Eigen::Quaterniond& q)
    {
        const Eigen::Matrix3d m = q.normalized().toRotationMatrix();
        const double yaw = std::atan2(m(1, 0), m(0, 0));
        const double pitch = std::atan2(-m(2, 0), std::hypot(m(2, 1), m(2, 2)));
        return Eigen::Quaterniond(
                   Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                   Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()))
            .normalized();
    }

    /**
     * @brief 固定轴 RPY（度）转旋转矩阵，与 URDF 的 rpy 约定一致：R = Rz·Ry·Rx
     * @param rpy_deg 依次为 roll / pitch / yaw，单位度
     * @return 对应的旋转矩阵
     */
    Eigen::Matrix3d rpyDegToMatrix(const double rpy_deg[3])
    {
        return (Eigen::AngleAxisd(rpy_deg[2] * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(rpy_deg[1] * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(rpy_deg[0] * M_PI / 180.0, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    }

} // namespace arms_ros2_control::command::detail
