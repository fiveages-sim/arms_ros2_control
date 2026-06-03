#pragma once

#include <memory>
#include <vector>

#ifdef HAS_LINA_PLANNING
#include "lina_planning/planning/path_planner/speedj.h"
#endif

namespace arms_controller_common
{
    /**
     * @brief Full-joint SpeedJ planner that decelerates all joints to zero velocity.
     */
    class JointSpeedStopPlanner
    {
    public:
        JointSpeedStopPlanner() = default;

        bool init(
            const std::vector<double>& joint_pos,
            const std::vector<double>& joint_vel,
            double period,
            double max_acc = 2.0,
            double max_jerk = 10.0);

        std::vector<double> run();

        bool isMotionOver() const;

        bool isActive() const { return active_; }

        void reset();

    private:
        bool active_{false};
        double period_{0.002};
        double max_acc_{2.0};
        double max_jerk_{10.0};
        std::vector<double> joint_pos_cache_;

#ifdef HAS_LINA_PLANNING
        std::unique_ptr<planning::SpeedJ> speedj_planner_;
#else
        struct FallbackState
        {
            std::vector<double> joint_vel;
            bool motion_over{true};
        };
        FallbackState fallback_state_;
#endif
    };
} // namespace arms_controller_common
