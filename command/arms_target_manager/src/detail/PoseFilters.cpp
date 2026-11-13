//
// PoseFilters 实现：冻结追赶、一阶低通、姿态分解、RPY 转旋转矩阵。
//
// 纯计算，不访问 ROS；可以直接写单元测试。
//
#include "arms_target_manager/detail/PoseFilters.h"

#include <algorithm>
#include <cmath>

namespace arms_ros2_control::command::detail
{
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

    Eigen::Matrix3d rpyDegToMatrix(const double rpy_deg[3])
    {
        return (Eigen::AngleAxisd(rpy_deg[2] * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(rpy_deg[1] * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(rpy_deg[0] * M_PI / 180.0, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    }

} // namespace arms_ros2_control::command::detail
