#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

namespace ocs2::controller_common {

/** 1 Hz window of per-cycle microseconds for p50 / p99 / max. Not for the RT hot path when disabled. */
struct RtSampleWindow {
    static constexpr size_t kCap = 1024;
    std::array<uint32_t, kCap> us{};
    size_t n{0};

    void add(const uint32_t sample_us)
    {
        if (n < kCap)
        {
            us[n++] = sample_us;
        }
    }

    void reset() { n = 0; }

    uint32_t percentile(const double p) const
    {
        if (n == 0)
        {
            return 0;
        }
        std::array<uint32_t, kCap> tmp{};
        std::copy(us.begin(), us.begin() + static_cast<std::ptrdiff_t>(n), tmp.begin());
        std::sort(tmp.begin(), tmp.begin() + static_cast<std::ptrdiff_t>(n));
        auto i = static_cast<size_t>(p * static_cast<double>(n - 1) + 0.5);
        if (i >= n)
        {
            i = n - 1;
        }
        return tmp[i];
    }

    uint32_t maxv() const
    {
        if (n == 0)
        {
            return 0;
        }
        return *std::max_element(us.begin(), us.begin() + static_cast<std::ptrdiff_t>(n));
    }
};

class RtCycleTiming {
public:
    using Clock = std::chrono::steady_clock;

    std::atomic<bool> enabled{false};

    struct Scope {
        uint32_t* dst{nullptr};
        Clock::time_point t0{};
        explicit Scope(uint32_t* d) : dst(d)
        {
            if (dst)
            {
                t0 = Clock::now();
            }
        }
        ~Scope()
        {
            if (dst)
            {
                *dst = static_cast<uint32_t>(
                    std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - t0).count());
            }
        }
        Scope(const Scope&) = delete;
        Scope& operator=(const Scope&) = delete;
        Scope(Scope&& other) noexcept : dst(other.dst), t0(other.t0) { other.dst = nullptr; }
        Scope& operator=(Scope&&) = delete;
    };

    Scope scope(uint32_t* dst)
    {
        return enabled.load(std::memory_order_relaxed) ? Scope(dst) : Scope(nullptr);
    }

    void beginCycle()
    {
        if (!enabled.load(std::memory_order_relaxed))
        {
            return;
        }
        cycle_t0_ = Clock::now();
        obs_us = ee_fk_us = ee_pub_us = viz_us = eval_us = 0;
        eval_upd_us = eval_obs_pub_us = eval_mrt_us = eval_traj_us = eval_cmd_us = 0;
    }

    void endCycle(const std::string& fsm, const rclcpp::Logger& logger)
    {
        if (!enabled.load(std::memory_order_relaxed))
        {
            return;
        }
        const auto total = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - cycle_t0_).count());
        total_w_.add(total);
        obs_w_.add(obs_us);
        ee_fk_w_.add(ee_fk_us);
        ee_pub_w_.add(ee_pub_us);
        viz_w_.add(viz_us);
        eval_w_.add(eval_us);
        eval_upd_w_.add(eval_upd_us);
        eval_obs_pub_w_.add(eval_obs_pub_us);
        eval_mrt_w_.add(eval_mrt_us);
        eval_traj_w_.add(eval_traj_us);
        eval_cmd_w_.add(eval_cmd_us);

        const auto now = Clock::now();
        if (last_log_.time_since_epoch().count() != 0 &&
            now - last_log_ < std::chrono::seconds(1))
        {
            return;
        }
        last_log_ = now;
        last_fsm_ = fsm;

        RCLCPP_INFO(logger,
                    "[rt_timing] fsm=%s n=%zu update_us p50=%u p99=%u max=%u  "
                    "obs_p50=%u ee_fk_p50=%u ee_pub_p50=%u viz_p50=%u  "
                    "eval_p50=%u upd_p50=%u obs_pub_p50=%u mrt_p50=%u traj_p50=%u cmd_p50=%u",
                    fsm.c_str(), total_w_.n,
                    total_w_.percentile(0.50), total_w_.percentile(0.99), total_w_.maxv(),
                    obs_w_.percentile(0.50), ee_fk_w_.percentile(0.50), ee_pub_w_.percentile(0.50),
                    viz_w_.percentile(0.50), eval_w_.percentile(0.50), eval_upd_w_.percentile(0.50),
                    eval_obs_pub_w_.percentile(0.50), eval_mrt_w_.percentile(0.50),
                    eval_traj_w_.percentile(0.50), eval_cmd_w_.percentile(0.50));

        total_w_.reset();
        obs_w_.reset();
        ee_fk_w_.reset();
        ee_pub_w_.reset();
        viz_w_.reset();
        eval_w_.reset();
        eval_upd_w_.reset();
        eval_obs_pub_w_.reset();
        eval_mrt_w_.reset();
        eval_traj_w_.reset();
        eval_cmd_w_.reset();
    }

    uint32_t obs_us{0};
    uint32_t ee_fk_us{0};
    uint32_t ee_pub_us{0};
    uint32_t viz_us{0};
    uint32_t eval_us{0};
    uint32_t eval_upd_us{0};
    uint32_t eval_obs_pub_us{0};
    uint32_t eval_mrt_us{0};
    uint32_t eval_traj_us{0};
    uint32_t eval_cmd_us{0};

private:
    Clock::time_point cycle_t0_{};
    Clock::time_point last_log_{};
    std::string last_fsm_;
    RtSampleWindow total_w_;
    RtSampleWindow obs_w_;
    RtSampleWindow ee_fk_w_;
    RtSampleWindow ee_pub_w_;
    RtSampleWindow viz_w_;
    RtSampleWindow eval_w_;
    RtSampleWindow eval_upd_w_;
    RtSampleWindow eval_obs_pub_w_;
    RtSampleWindow eval_mrt_w_;
    RtSampleWindow eval_traj_w_;
    RtSampleWindow eval_cmd_w_;
};

} // namespace ocs2::controller_common
