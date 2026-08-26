#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace arms_controller_common
{

struct OnlineThirdOrderConfig
{
    std::vector<double> tracking_frequency;
    std::vector<double> max_velocity;
    std::vector<double> min_velocity;
    std::vector<double> max_acceleration;
    std::vector<double> min_acceleration;
    std::vector<double> max_jerk;
    std::vector<double> min_jerk;

    std::vector<double> max_input_velocity;
    std::vector<double> spike_margin;
    std::vector<double> spike_cluster_tolerance;
    std::vector<double> deadband;

    double alpha{0.65};
    double beta{0.08};
    double input_timeout{0.3};
    double decision_timeout{0.075};
    double max_integrator_dt{0.005};

    std::vector<double> position_tolerance;
    std::vector<double> velocity_tolerance;
    std::vector<double> acceleration_tolerance;
};

/**
 * Robust position-only streaming reference conditioner and jerk-limited
 * third-order reference governor.
 *
 * Raw targets are processed at their arrival rate. Motion state is advanced at
 * the controller update rate, so receiving a new target never resets q/v/a.
 */
class OnlineThirdOrderInterpolator
{
public:
    enum class TargetResult
    {
        ACCEPTED,
        HELD_DEADBAND,
        PENDING_SPIKE,
        INVALID
    };

    explicit OnlineThirdOrderInterpolator(OnlineThirdOrderConfig config);

    void reset(const std::vector<double>& position,
               const std::vector<double>& velocity = {},
               const std::vector<double>& acceleration = {},
               double stamp_seconds = 0.0);

    TargetResult pushTarget(const std::vector<double>& raw_target,
                            double stamp_seconds);

    std::vector<double> update(double dt, double now_seconds);

    const std::vector<double>& position() const { return position_; }
    const std::vector<double>& velocity() const { return velocity_; }
    const std::vector<double>& acceleration() const { return acceleration_; }
    const std::vector<double>& estimatedTargetPosition() const { return estimate_position_; }
    const std::vector<double>& estimatedTargetVelocity() const { return estimate_velocity_; }

    bool initialized() const { return initialized_; }
    bool hasTarget() const { return has_target_; }
    bool targetTimedOut(double now_seconds) const;
    std::size_t dof() const { return position_.size(); }

private:
    struct TargetFrame
    {
        std::vector<double> position;
        double stamp{0.0};
    };

    void validateConfig() const;
    bool vectorNear(const std::vector<double>& lhs,
                    const std::vector<double>& rhs,
                    const std::vector<double>& tolerance) const;
    void acceptMeasurement(const std::vector<double>& measurement,
                           double stamp_seconds);
    void commitFrame(const TargetFrame& frame, bool stationary);
    bool isMiddleFrameSpike(const TargetFrame& previous,
                            const TargetFrame& middle,
                            const TargetFrame& next) const;
    void flushPendingTarget(double now_seconds);
    void integrateStep(double dt, double now_seconds);

    OnlineThirdOrderConfig config_;
    std::vector<double> position_;
    std::vector<double> velocity_;
    std::vector<double> acceleration_;

    std::vector<double> accepted_target_;

    std::vector<double> estimate_position_;
    std::vector<double> estimate_velocity_;
    TargetFrame committed_frame_;
    std::optional<TargetFrame> pending_frame_;
    double last_measurement_stamp_{0.0};
    double last_accepted_stamp_{0.0};
    double last_valid_input_stamp_{0.0};
    bool initialized_{false};
    bool has_target_{false};
};

}  // namespace arms_controller_common
