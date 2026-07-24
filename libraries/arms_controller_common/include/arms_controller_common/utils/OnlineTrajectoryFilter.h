#pragma once

#include <cstddef>

namespace arms_controller_common
{
    /**
     * @brief Stateful one-DoF Ruckig online trajectory generator.
     *
     * The public interface remains compatible with the previous third-order
     * filter, while each update replans from the retained position, velocity,
     * and acceleration state under velocity, acceleration, and jerk limits.
     */
    class OnlineTrajectoryFilter
    {
    public:
        struct Limits
        {
            double min_velocity{-1.0};
            double max_velocity{1.0};
            double min_acceleration{-2.0};
            double max_acceleration{2.0};
            double max_jerk{10.0};
            // Kept for ROS parameter/API compatibility; Ruckig does not use a
            // feedback tracking frequency.
            double tracking_frequency{5.0};
            double target_filter_alpha{0.2};
        };

        OnlineTrajectoryFilter() = default;
        explicit OnlineTrajectoryFilter(const Limits& limits);

        bool configure(const Limits& limits);
        void reset(double position, double velocity = 0.0, double acceleration = 0.0);
        void setTarget(double position, double velocity = 0.0, double acceleration = 0.0);

        /** Advance by dt seconds and return the new position command. */
        double update(double dt);

        bool isSettled(double position_tolerance,
                       double velocity_tolerance,
                       double acceleration_tolerance) const;

        double position() const { return position_; }
        double velocity() const { return velocity_; }
        double acceleration() const { return acceleration_; }
        double targetPosition() const { return target_position_; }
        double lastJerk() const { return last_jerk_; }

    private:
        Limits limits_{};
        bool configured_{false};

        double position_{0.0};
        double velocity_{0.0};
        double acceleration_{0.0};
        double target_position_{0.0};
        double target_velocity_{0.0};
        double target_acceleration_{0.0};
        double requested_target_position_{0.0};
        double requested_target_velocity_{0.0};
        double requested_target_acceleration_{0.0};
        double last_jerk_{0.0};
    };
}  // namespace arms_controller_common
