#pragma once

namespace planning {
    class NormalizationPoint {
    private:

    public:
        NormalizationPoint(double time = 0.0, double pos = 0.0, double vel = 0.0,
                           double acc = 0.0, double jerk = 0.0);
        NormalizationPoint operator*(double arg) const {
            return NormalizationPoint(time, arg * pos, arg * vel, arg * acc,
                                      arg * jerk);
        }
        ~NormalizationPoint()=default;
        double time;
        double pos;
        double vel;
        double acc;
        double jerk;
    };

} // namespace planning
