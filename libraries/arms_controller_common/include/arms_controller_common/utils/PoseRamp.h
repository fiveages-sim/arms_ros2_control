#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace arms_controller_common
{
    /**
     * 接管渐入参数。
     *
     * 语义：把一个笛卡尔位姿目标从「当前保持的位姿」平滑过渡到「实时期望位姿」，
     * 过渡期间按线速度 / 角速度上限自动计算最短时长；过渡结束后输出严格等于期望位姿。
     */
    struct PoseRampConfig
    {
        // false = 完全直通（输出恒等于期望位姿），用于一键回退到无渐入行为
        bool enable{true};
        // 过渡段线速度上限 [m/s]
        double max_linear_velocity{0.15};
        // 过渡段角速度上限 [rad/s]
        double max_angular_velocity{0.4};
        // 过渡段最短时长 [s]（目标几乎重合时也不瞬变）
        double min_duration{0.2};
        // 过渡段最长时长 [s]；触顶时实际峰值速度会超过上限，通过 lastPlanWasCapped() 暴露
        double max_duration{3.0};
        // 输出中断超过该时长后再恢复时，自动以「上次输出」为锚点重新渐入 [s]
        // 0 = 关闭该行为（适用于目标通道本身低频的场景请设 0）
        double resume_gap{0.3};
    };

    /**
     * 笛卡尔位姿接管渐入器（纯 Eigen，无外部依赖，header-only）。
     *
     * 设计要点：
     *   1. 进度采用五次多项式 s(t) = 6t^5 - 15t^4 + 10t^3，两端速度与加速度均为 0，
     *      因此起手不产生速度阶跃、收尾不与「直通跟踪」产生交接跳变。
     *   2. 过渡时长由「锚点 → 首帧期望位姿」的距离反算：
     *          T = 1.875 * max(|Δp| / v_max, Δθ / ω_max)
     *      1.875 是五次多项式导数的峰值（30t^2(1-t)^2 的最大值），
     *      不乘该系数则实际峰值速度会超标 87.5%。
     *   3. 每帧输出 anchor 与**实时**期望位姿之间的 s 插值，因此过渡期间人继续移动
     *      不会让目标「追一个冻结的旧终点」，s = 1 时输出严格等于当时的期望位姿。
     *   4. 过渡结束后进入 DONE 直通状态；若输出中断超过 resume_gap（骨架超时冻结、
     *      上游话题暂停等），下一帧自动以最后输出为锚点重新渐入，不会把阶跃补回去。
     *
     * 线程安全：不加锁。调用方需保证同一实例只在同一回调线程/互斥区内使用。
     */
    class PoseRamp
    {
    public:
        enum class Phase
        {
            IDLE,     // 未启用：输出 = 期望位姿
            RAMPING,  // 渐入中：输出 = 锚点与期望位姿之间的插值
            DONE      // 渐入完成：输出 = 期望位姿
        };

        PoseRamp() = default;
        explicit PoseRamp(const PoseRampConfig& config) { configure(config); }

        /** 更新参数。会做合法性收敛，避免除零与时长区间自相矛盾。 */
        void configure(const PoseRampConfig& config)
        {
            config_ = config;
            config_.max_linear_velocity = std::max(1e-6, config_.max_linear_velocity);
            config_.max_angular_velocity = std::max(1e-6, config_.max_angular_velocity);
            config_.min_duration = std::max(0.0, config_.min_duration);
            config_.max_duration = std::max(config_.min_duration, config_.max_duration);
            config_.resume_gap = std::max(0.0, config_.resume_gap);
        }

        const PoseRampConfig& config() const { return config_; }

        /** 回到直通状态（停跟踪 / 失能 / 拓扑变化时调用）。 */
        void reset()
        {
            phase_ = Phase::IDLE;
            clock_started_ = false;
            duration_ = 0.0;
            duration_capped_ = false;
            has_anchor_ = false;
        }

        /**
         * 以给定位姿为起点开启一次渐入。
         * 调用方应在「开启跟踪」时用机器人当前保持的位姿（最后下发目标 / 实测位姿）
         * 作为锚点，这样第一帧输出不会跳变。
         */
        void armFrom(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation)
        {
            anchor_position_ = position;
            anchor_orientation_ = normalizedOrIdentity(orientation);
            has_anchor_ = true;
            clock_started_ = false;
            start_time_ = std::chrono::steady_clock::time_point{};
            duration_ = 0.0;
            duration_capped_ = false;
            phase_ = Phase::RAMPING;
        }

        bool active() const { return phase_ != Phase::IDLE; }
        bool ramping() const { return phase_ == Phase::RAMPING; }
        Phase phase() const { return phase_; }

        /** 上一次规划是否因 max_duration 触顶而不得不超过速度上限。 */
        bool lastPlanWasCapped() const { return duration_capped_; }

        /** 上一次规划出的过渡时长 [s]（未开始时为 0）。 */
        double lastDuration() const { return duration_; }

        /**
         * 计算本帧应下发的位姿。
         *
         * @param desired_position   实时期望位置（必须有限）
         * @param desired_orientation 实时期望姿态（会归一化；非有限时退回单位四元数）
         * @param now                当前时刻（steady_clock）
         * @param output_position    本帧输出位置
         * @param output_orientation 本帧输出姿态
         * @return true 表示本帧仍处于渐入阶段（输出被插值），false 表示直通
         */
        bool update(const Eigen::Vector3d& desired_position,
                    const Eigen::Quaterniond& desired_orientation,
                    const std::chrono::steady_clock::time_point& now,
                    Eigen::Vector3d& output_position,
                    Eigen::Quaterniond& output_orientation)
        {
            const Eigen::Vector3d target_position =
                desired_position.allFinite() ? desired_position : Eigen::Vector3d::Zero();
            const Eigen::Quaterniond target_orientation =
                normalizedOrIdentity(desired_orientation);

            if (!config_.enable)
            {
                phase_ = Phase::IDLE;
                output_position = target_position;
                output_orientation = target_orientation;
                return false;
            }

            // 输出中断后再恢复：以最后输出为锚点重新渐入，避免把中断期间的位移
            // 一次性补成阶跃（骨架超时冻结 / 上游话题暂停）。
            if (phase_ == Phase::DONE && config_.resume_gap > 0.0 && has_last_emit_ &&
                elapsedSeconds(now, last_emit_time_) > config_.resume_gap)
            {
                anchor_position_ = last_output_position_;
                anchor_orientation_ = last_output_orientation_;
                has_anchor_ = true;
                clock_started_ = false;
                duration_ = 0.0;
                duration_capped_ = false;
                phase_ = Phase::RAMPING;
            }

            if (phase_ == Phase::RAMPING)
            {
                if (!has_anchor_)
                {
                    // 没给锚点：直接以当前期望位姿起手，等价于不渐入。
                    anchor_position_ = target_position;
                    anchor_orientation_ = target_orientation;
                    has_anchor_ = true;
                }

                if (!clock_started_)
                {
                    planDuration(now, target_position, target_orientation);
                }

                const double raw_progress = duration_ > 0.0
                    ? elapsedSeconds(now, start_time_) / duration_
                    : 1.0;
                const double progress = std::clamp(raw_progress, 0.0, 1.0);

                if (progress >= 1.0)
                {
                    output_position = target_position;
                    output_orientation = target_orientation;
                    phase_ = Phase::DONE;
                }
                else
                {
                    const double s = quinticProgress(progress);
                    output_position =
                        anchor_position_ + s * (target_position - anchor_position_);
                    output_orientation = anchor_orientation_.slerp(s, target_orientation);
                    output_orientation.normalize();
                }

                rememberOutput(now, output_position, output_orientation);
                // 进度刚好到 1 的同一帧已经输出目标位姿，按直通语义返回 false。
                return phase_ == Phase::RAMPING;
            }

            // IDLE / DONE：直通
            output_position = target_position;
            output_orientation = target_orientation;
            rememberOutput(now, output_position, output_orientation);
            return false;
        }

    private:
        static Eigen::Quaterniond normalizedOrIdentity(const Eigen::Quaterniond& q)
        {
            if (!q.coeffs().allFinite() || q.norm() < 1e-9)
            {
                return Eigen::Quaterniond::Identity();
            }
            Eigen::Quaterniond normalized = q;
            normalized.normalize();
            return normalized;
        }

        static double elapsedSeconds(const std::chrono::steady_clock::time_point& now,
                                     const std::chrono::steady_clock::time_point& from)
        {
            if (now <= from)
            {
                return 0.0;
            }
            return std::chrono::duration<double>(now - from).count();
        }

        /** 五次多项式：s(0)=0, s(1)=1, s'(0)=s'(1)=0, s''(0)=s''(1)=0。 */
        static double quinticProgress(double t)
        {
            const double t2 = t * t;
            const double t3 = t2 * t;
            return t3 * (10.0 - 15.0 * t + 6.0 * t2);
        }

        void planDuration(const std::chrono::steady_clock::time_point& now,
                          const Eigen::Vector3d& target_position,
                          const Eigen::Quaterniond& target_orientation)
        {
            const double linear_distance = (target_position - anchor_position_).norm();
            const double angular_distance =
                std::abs(anchor_orientation_.angularDistance(target_orientation));

            // 五次多项式的峰值速度系数（30t^2(1-t)^2 在 t=0.5 处取 1.875）。
            constexpr double kPeakFactor = 1.875;
            double duration = 0.0;
            duration = std::max(duration,
                                kPeakFactor * linear_distance / config_.max_linear_velocity);
            duration = std::max(duration,
                                kPeakFactor * angular_distance / config_.max_angular_velocity);

            duration_capped_ = false;
            if (duration > config_.max_duration)
            {
                duration = config_.max_duration;
                duration_capped_ = true;
            }
            duration = std::max(duration, config_.min_duration);
            duration_ = std::max(0.0, duration);

            start_time_ = now;
            clock_started_ = true;
        }

        void rememberOutput(const std::chrono::steady_clock::time_point& now,
                            const Eigen::Vector3d& position,
                            const Eigen::Quaterniond& orientation)
        {
            last_emit_time_ = now;
            has_last_emit_ = true;
            last_output_position_ = position;
            last_output_orientation_ = orientation;
        }

        PoseRampConfig config_{};

        Phase phase_{Phase::IDLE};
        bool has_anchor_{false};
        bool clock_started_{false};
        bool duration_capped_{false};
        double duration_{0.0};
        std::chrono::steady_clock::time_point start_time_{};

        Eigen::Vector3d anchor_position_{Eigen::Vector3d::Zero()};
        Eigen::Quaterniond anchor_orientation_{Eigen::Quaterniond::Identity()};

        bool has_last_emit_{false};
        std::chrono::steady_clock::time_point last_emit_time_{};
        Eigen::Vector3d last_output_position_{Eigen::Vector3d::Zero()};
        Eigen::Quaterniond last_output_orientation_{Eigen::Quaterniond::Identity()};
    };

}  // namespace arms_controller_common
