//
// 与 ROS 无关的纯计算：冻结追赶、低通、姿态分解。
//
// 放在 detail 里只为一件事：可以单独写测试，不必启动节点。
//
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace arms_ros2_control::command::detail
{
    /**
     * 上游 XR 数据在链路没更新时会重复发送同一帧，恢复后一帧补齐整段位移。
     * 这里检测“逐位相同”的冻结，并把解冻时的缺口按冻结帧数摊到随后若干帧，
     * 正常运动逐位不受影响。
     */
    class StaleCatchUpFilter
    {
    public:
        struct State
        {
            bool has_previous_input{false};
            Eigen::Vector3d previous_input_position{Eigen::Vector3d::Zero()};
            Eigen::Quaterniond previous_input_orientation{Eigen::Quaterniond::Identity()};
            int frozen_frames{0};
            Eigen::Vector3d output_position{Eigen::Vector3d::Zero()};
            Eigen::Quaterniond output_orientation{Eigen::Quaterniond::Identity()};
            Eigen::Vector3d residual_position{Eigen::Vector3d::Zero()};
            Eigen::Quaterniond residual_orientation{Eigen::Quaterniond::Identity()};
            int remaining_frames{0};
        };

        explicit StaleCatchUpFilter(int min_frozen_frames = 2)
            : min_frozen_frames_(min_frozen_frames)
        {
        }

        /** 原地平滑。返回 true 表示本帧处于摊平过程中。 */
        bool apply(Eigen::Vector3d& position, Eigen::Quaterniond& orientation);

        void reset() { state_ = State{}; }

        [[nodiscard]] int frozenFrames() const { return state_.frozen_frames; }

    private:
        State state_;
        int min_frozen_frames_{2};
    };

    /** 一阶低通，位置线性插值、姿态球面插值。 */
    class PoseLowPass
    {
    public:
        void setAlpha(double alpha)
        {
            alpha_ = alpha < 0.0 ? 0.0 : (alpha > 1.0 ? 1.0 : alpha);
        }

        void reset() { primed_ = false; }

        void apply(Eigen::Vector3d& position, Eigen::Quaterniond& orientation);

    private:
        double alpha_{0.3};
        bool primed_{false};
        Eigen::Vector3d position_{Eigen::Vector3d::Zero()};
        Eigen::Quaterniond orientation_{Eigen::Quaterniond::Identity()};
    };

    /** ZYX 分解后丢弃 roll，用于只有 yaw/pitch 自由度的任务。 */
    Eigen::Quaterniond dropRoll(const Eigen::Quaterniond& q);

    /** 固定轴 RPY（度）转旋转矩阵，与 URDF 的 rpy 约定一致。 */
    Eigen::Matrix3d rpyDegToMatrix(const double rpy_deg[3]);

} // namespace arms_ros2_control::command::detail
