#include "joint_vector.h"
namespace planning {
    class TrajectPoint {
    public:
        TrajectPoint(){};
        explicit TrajectPoint(size_t n);
        TrajectPoint(const JointVector &joint_pos, const JointVector &joint_vel,
                     const JointVector &joint_acc, const JointVector &joint_jerk);
        TrajectPoint(const JointVector &joint_pos, const JointVector &joint_vel,
                     const JointVector &joint_acc);
        TrajectPoint(const JointVector &joint_pos, const JointVector &joint_vel);
        TrajectPoint(const JointVector &joint_pos);
        ~TrajectPoint() = default;

    public:
        JointVector joint_pos;
        JointVector joint_vel;
        JointVector joint_acc;
        JointVector joint_jerk;
        double time;
    };

    class TrajectoryParameter {
    public:
        TrajectoryParameter(){};
        TrajectoryParameter(const TrajectoryParameter &arg);
        TrajectoryParameter(size_t n);
        TrajectoryParameter(const JointVector &max_vel, const JointVector &max_acc,
                            const JointVector &max_jerk);
        ~TrajectoryParameter() = default;
        JointVector joint_max_vel;
        JointVector joint_max_acc;
        JointVector joint_max_jerk;
    };

    class TrajectoryInitParameters {
    public:
        TrajectoryInitParameters(){};
        TrajectoryInitParameters(size_t n);
        TrajectoryInitParameters(const TrajectPoint &start_point,
                                 const TrajectPoint &end_point,
                                 TrajectoryParameter &joint_para, double period=0.001);
        ~TrajectoryInitParameters() = default;
        TrajectPoint start_point;
        TrajectPoint end_point;
        TrajectoryParameter parameter;
        double period=0.001;
    };

} // namespace planning
