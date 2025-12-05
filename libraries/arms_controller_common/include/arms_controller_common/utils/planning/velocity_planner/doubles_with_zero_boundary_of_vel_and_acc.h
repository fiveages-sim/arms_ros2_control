#pragma once
#include "arms_controller_common/utils/planning/common/normalization_point.h"

namespace planning {
    /**
     * @brief 使用doubles规划一段初始速度和中止速度均为0的速度轨迹
       @param
     */
    class DoublesWithZeroBoundaryOfVelAndAcc {
    private:
        double start_pos;
        double end_pos;
        double direction = 1.0;
        double max_vel;
        double max_acc;
        double max_jerk;
        double period;
        double tj1;
        double tj2;
        double ta;
        double td;
        double tv;
        double totaltime;

    public:
        DoublesWithZeroBoundaryOfVelAndAcc(){};
        bool init(double start_pos = 0.0, double end_pos = 1.0, double max_vel = 0.0,
                  double max_acc = 0.0, double max_jerk = 0.0, double period = 0.001);
        NormalizationPoint calculatePointAtT(double t);
        double getTotalTime() { return totaltime; };
        ~DoublesWithZeroBoundaryOfVelAndAcc() = default;
    };

} // namespace planning