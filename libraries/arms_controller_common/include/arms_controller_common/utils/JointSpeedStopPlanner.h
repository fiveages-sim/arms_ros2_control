#pragma once
#include <vector>
#include <string>
#include <cstddef>
#include <memory>

namespace arms_controller_common
{
    // Common progress preserves joint velocity ratios during braking. If the
    // initial accelerations are incompatible, first release them over a shared
    // interval. A position-bound truncation stops the entire group together.
    class JointSpeedStopPlanner
    {
    public:
        bool init(const std::vector<double>& positions,
                  const std::vector<double>& velocities,
                  const std::vector<double>& accelerations,
                  double max_velocity, double max_acceleration, double max_jerk,
                  const std::vector<double>& lower, const std::vector<double>& upper,
                  bool truncate_at_limits);
        std::vector<double> run(double dt);
        const std::vector<double>& velocities() const { return velocities_; }
        const std::vector<double>& accelerations() const { return accelerations_; }
        bool isMotionOver() const;
        bool isActive() const { return active_; }
        bool wasTruncated() const { return truncated_; }
        const std::string& error() const { return error_; }
        void reset();

    private:
        bool initScalar(const std::vector<double>& positions,
                        const std::vector<double>& velocities,
                        const std::vector<double>& accelerations,
                        double max_velocity, double max_acceleration, double max_jerk,
                        const std::vector<double>& lower, const std::vector<double>& upper,
                        bool truncate_at_limits);
        bool synchronized_{false};
        std::unique_ptr<JointSpeedStopPlanner> scalar_stop_;
        std::vector<double> initial_positions_, initial_velocities_, initial_accelerations_;
        std::vector<double> base_positions_, directions_;
        double transition_duration_{0}, transition_elapsed_{0}, release_duration_{0};
        struct Segment { double q, v, a, jerk, duration; };
        struct Joint
        {
            std::vector<Segment> segments;
            size_t index{0};
            double elapsed{0}, final_position{0};
        };
        std::vector<Joint> joints_;
        std::vector<double> positions_, velocities_, accelerations_, lower_, upper_;
        bool active_{false}, truncated_{false};
        std::string error_;
    };
}
