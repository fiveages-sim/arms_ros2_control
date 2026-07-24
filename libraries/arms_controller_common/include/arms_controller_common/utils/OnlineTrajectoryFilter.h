#pragma once

#include <cstddef>

namespace arms_controller_common
{
    /**
     * @brief Stateful third-order online trajectory filter for one joint.
     *
     * The filter keeps position, velocity and acceleration continuous while a
     * streaming position target changes. It follows the third-order state and
     * constraint model from trajectory_online_planning/main.py, with a
     * critically damped jerk command and target low-pass filtering to avoid the
     * discrete bang-bang limit cycle at 100 Hz.
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
        double calculateDesiredJerk() const;
        double enforceDiscreteLimits(double desired_jerk, double dt) const;

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
