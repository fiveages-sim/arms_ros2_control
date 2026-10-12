#pragma once

#include <algorithm>
#include <cmath>

namespace arms_controller_common
{
// Stateful third-order tracking filter ported from rokae_ros2_control.
// Retains the original discrete switching law; v/a limits can overshoot by a step.
    struct JointServoFilter {
        double v_min{-3.0};
        double v_max{3.0};
        double a_min{-10.0};
        double a_max{10.0};
        double jerk_limit{100.0};
        double dt{0.001};
        double q{0.0};
        double qd{0.0};
        double qdd{0.0};

        void reset(double position, double velocity = 0.0, double acceleration = 0.0);
        double updateTo(double target_position, double target_velocity = 0.0, double target_acceleration = 0.0);

    private:
        static constexpr double kEpsilon = 1.0e-9;
        static int sign(double value);
        double getJerk(double target_position, double target_velocity, double target_acceleration) const;
        double accelControl(double error_acceleration, double acceleration_limit) const;
        double velocityBoundary(double error_acceleration, double error_velocity, double velocity_limit) const;
        double velocityControl(
            double min_error_acceleration,
            double max_error_acceleration,
            double error_acceleration,
            double error_velocity,
            double velocity_limit) const;
    };

    inline int JointServoFilter::sign(double value) {
        if (value > kEpsilon) {
            return 1;
        }
        if (value < -kEpsilon) {
            return -1;
        }
        return 0;
    }

    inline void JointServoFilter::reset(
        double position, double velocity, double acceleration) {
        q = position;
        qd = velocity;
        qdd = acceleration;
    }

    inline double JointServoFilter::updateTo(
        double target_position, double target_velocity, double target_acceleration) {
        const double jerk = getJerk(target_position, target_velocity, target_acceleration);
        const double next_acceleration = qdd + dt * jerk;
        const double next_velocity = qd + dt * 0.5 * (next_acceleration + qdd);
        q += dt * 0.5 * (next_velocity + qd);
        qd = next_velocity;
        qdd = next_acceleration;
        return q;
    }

    inline double JointServoFilter::getJerk(
        double target_position, double target_velocity, double target_acceleration) const {
        const double jerk = std::max(std::abs(jerk_limit), kEpsilon);
        const double ek = (q - target_position) / jerk;
        const double ek_1 = (qd - target_velocity) / jerk;
        const double ek_2 = (qdd - target_acceleration) / jerk;
        const double e_min_1 = (v_min - target_velocity) / jerk;
        const double e_max_1 = (v_max - target_velocity) / jerk;
        const double e_min_2 = (a_min - target_acceleration) / jerk;
        const double e_max_2 = (a_max - target_acceleration) / jerk;

        const double delta = ek_1 + (ek_2 * std::abs(ek_2)) / 2.0;
        const int s_delta = sign(delta);
        const double sqrt_arg = std::max(
            0.0,
            2.0 * std::pow((ek_2 * ek_2) + 2.0 * ek_1 * static_cast<double>(s_delta), 3.0));
        const double sigma =
            ek + ek_1 * ek_2 * static_cast<double>(s_delta) -
            (std::pow(ek_2, 3.0) * (1.0 - 3.0 * std::abs(s_delta))) / 6.0 +
            static_cast<double>(s_delta) * std::sqrt(sqrt_arg) / 4.0;

        const double max_accel_error = std::abs(e_max_2) > kEpsilon ? e_max_2 : kEpsilon;
        const double min_accel_error = std::abs(e_min_2) > kEpsilon ? e_min_2 : -kEpsilon;
        const double positive_brake = (ek_2 * ek_2) - 2.0 * ek_1;
        const double negative_brake = (ek_2 * ek_2) + 2.0 * ek_1;
        const double nu_p =
            ek - max_accel_error * positive_brake / 4.0 -
            (positive_brake * positive_brake) / (8.0 * max_accel_error) -
            ek_2 * (3.0 * ek_1 - (ek_2 * ek_2)) / 3.0;
        const double nu_n =
            ek - min_accel_error * negative_brake / 4.0 -
            (negative_brake * negative_brake) / (8.0 * min_accel_error) +
            ek_2 * (3.0 * ek_1 + (ek_2 * ek_2)) / 3.0;

        double sigma_limited = sigma;
        if (ek_2 <= e_max_2 && ek_1 <= (ek_2 * ek_2) / 2.0 - (e_max_2 * e_max_2)) {
            sigma_limited = nu_p;
        } else if (ek_2 >= e_min_2 && ek_1 >= (e_min_2 * e_min_2) - (ek_2 * ek_2) / 2.0) {
            sigma_limited = nu_n;
        }

        const int sigma_sign = sign(sigma_limited);
        const double control =
            -jerk * sign(
                sigma_limited +
                (1.0 - std::abs(sigma_sign)) *
                    (delta + (1.0 - std::abs(s_delta)) * ek_2));

        return std::max(
            velocityControl(e_min_2, e_max_2, ek_2, ek_1, e_min_1),
            std::min(control, velocityControl(e_min_2, e_max_2, ek_2, ek_1, e_max_1)));
    }

    inline double JointServoFilter::accelControl(
        double error_acceleration, double acceleration_limit) const {
        return -std::abs(jerk_limit) * sign(error_acceleration - acceleration_limit);
    }

    inline double JointServoFilter::velocityBoundary(
        double error_acceleration, double error_velocity, double velocity_limit) const {
        const double delta_v =
            error_acceleration * std::abs(error_acceleration) +
            2.0 * (error_velocity - velocity_limit);
        return -std::abs(jerk_limit) *
               sign(delta_v + (1.0 - std::abs(sign(delta_v))) * error_acceleration);
    }

    inline double JointServoFilter::velocityControl(
        double min_error_acceleration,
        double max_error_acceleration,
        double error_acceleration,
        double error_velocity,
        double velocity_limit) const {
        return std::max(
            accelControl(error_acceleration, min_error_acceleration),
            std::min(
                velocityBoundary(error_acceleration, error_velocity, velocity_limit),
                accelControl(error_acceleration, max_error_acceleration)));
    }

}
