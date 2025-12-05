#include "arms_controller_common/utils/planning/common/trajectory_init_parameters.h"

namespace planning {
TrajectPoint::TrajectPoint(size_t n)
    : joint_pos(n), joint_vel(n), joint_acc(n), joint_jerk(n){};
TrajectPoint::TrajectPoint(const JointVector &_joint_pos,
                           const JointVector &_joint_vel,
                           const JointVector &_joint_acc,
                           const JointVector &_joint_jerk)
    : joint_pos(_joint_pos), joint_vel(_joint_vel), joint_acc(_joint_acc),
      joint_jerk(_joint_jerk){};
TrajectPoint::TrajectPoint(const JointVector &_joint_pos,
                           const JointVector &_joint_vel,
                           const JointVector &_joint_acc)
    : joint_pos(_joint_pos), joint_vel(_joint_vel), joint_acc(_joint_acc),
      joint_jerk(joint_pos.getJointSize()){};
TrajectPoint::TrajectPoint(const JointVector &_joint_pos,
                           const JointVector &_joint_vel)
    : joint_pos(_joint_pos), joint_vel(_joint_vel),
      joint_acc(joint_pos.getJointSize()),
      joint_jerk(joint_pos.getJointSize()){};
TrajectPoint::TrajectPoint(const JointVector &_joint_pos)
    : joint_pos(_joint_pos), joint_vel(joint_pos.getJointSize()),
      joint_acc(joint_pos.getJointSize()),
      joint_jerk(joint_pos.getJointSize()){};

TrajectoryParameter::TrajectoryParameter(const JointVector &_max_vel,
                                         const JointVector &_max_acc,
                                         const JointVector &_max_jerk)
    : joint_max_vel(_max_vel), joint_max_acc(_max_acc),
      joint_max_jerk(_max_jerk){};
TrajectoryParameter::TrajectoryParameter(const TrajectoryParameter &arg)
    : joint_max_vel(arg.joint_max_vel), joint_max_acc(arg.joint_max_acc),
      joint_max_jerk(arg.joint_max_jerk){};
TrajectoryParameter::TrajectoryParameter(size_t n)
    : joint_max_vel(n), joint_max_acc(n), joint_max_jerk(n){};

TrajectoryInitParameters::TrajectoryInitParameters(size_t n)
    : start_point(n), end_point(n), parameter(n){};
TrajectoryInitParameters::TrajectoryInitParameters(
    const TrajectPoint &_start_point, const TrajectPoint &_end_point,
    TrajectoryParameter &para, double _period)
    : start_point(start_point), end_point(end_point), parameter(para),
      period(_period){};
} // namespace planning
