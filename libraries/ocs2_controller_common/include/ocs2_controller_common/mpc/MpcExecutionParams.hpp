/******************************************************************************
 * MPC timing derived from controller rate and ros__parameters (in-process MPC).
 ******************************************************************************/

#pragma once

#include <cmath>

#include <rclcpp/logger.hpp>

namespace ocs2::controller_common {

struct MpcExecutionParams {
    double mpc_period_sec{0.05};
    int thread_sleep_ms{2};
};

/** Mirrors previous StateOCS2 logic: mpc_frequency from param, clamp, min 10 Hz, thread sleep 2x controller period. */
inline MpcExecutionParams computeMpcExecutionParams(const double controller_frequency_hz,
                                                    const int mpc_frequency_param,
                                                    const rclcpp::Logger& logger) {
    MpcExecutionParams out;
    double mpc_frequency = static_cast<double>(mpc_frequency_param);

    if (mpc_frequency > 0.0) {
        RCLCPP_INFO(logger, "MPC frequency from parameter: %.1f Hz", mpc_frequency);
    }

    if (!(mpc_frequency > 0.0 && mpc_frequency <= controller_frequency_hz)) {
        const double original_frequency = mpc_frequency;
        mpc_frequency = controller_frequency_hz / 4.0;

        if (original_frequency <= 0.0) {
            RCLCPP_WARN(logger,
                        "Invalid MPC frequency (%.1f Hz), using 1/4 of controller frequency: %.1f Hz (controller: %.1f Hz)",
                        original_frequency, mpc_frequency, controller_frequency_hz);
        } else if (original_frequency > controller_frequency_hz) {
            RCLCPP_WARN(logger,
                        "MPC frequency (%.1f Hz) exceeds controller frequency (%.1f Hz), using 1/4 of controller frequency: %.1f Hz",
                        original_frequency, controller_frequency_hz, mpc_frequency);
        } else {
            RCLCPP_INFO(logger,
                        "MPC frequency not set, using 1/4 of controller frequency: %.1f Hz (controller: %.1f Hz)",
                        mpc_frequency, controller_frequency_hz);
        }
    }

    if (mpc_frequency < 10.0) {
        RCLCPP_WARN(logger, "MPC frequency (%.1f Hz) is too low, setting minimum frequency to 10.0 Hz", mpc_frequency);
        mpc_frequency = 10.0;
    }

    out.mpc_period_sec = 1.0 / mpc_frequency;
    out.thread_sleep_ms = static_cast<int>(2.0 / controller_frequency_hz * 1000.0);

    RCLCPP_INFO(logger, "Thread sleep duration: %d ms (based on controller frequency: %.1f Hz)", out.thread_sleep_ms,
                controller_frequency_hz);

    return out;
}

} // namespace ocs2::controller_common
