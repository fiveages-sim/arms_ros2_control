#include "arms_controller_common/utils/planning/path_planner/movej.h"
#include <iostream>

namespace planning {
bool moveJ::init(const TrajectoryInitParameters &traj_para) {
  const double epislon = 1.0e-10;
  start_pos = traj_para.start_point.joint_pos;
  delta_pos = traj_para.end_point.joint_pos - start_pos;
  size_t nr_of_joints = start_pos.getJointSize();
  for (size_t i = 0; i < nr_of_joints; i++) {
    if (fabs(traj_para.start_point.joint_vel(i)) > epislon) {
      std::cerr << "Initial condition error! starting velocity of joint " << i
                << "is not zero" << std::endl;
      return false;
    }

    if (fabs(traj_para.start_point.joint_acc(i)) > epislon) {
      std::cerr << "Initial condition error! starting acceleration of joint "
                << i << "is not zero" << std::endl;
      return false;
    }

    if (fabs(traj_para.start_point.joint_jerk(i)) > epislon) {
      std::cerr << "Initial condition error! starting jerk of joint " << i
                << "is not zero" << std::endl;
      return false;
    }

    if (fabs(traj_para.end_point.joint_vel(i)) > epislon) {
      std::cerr << "Initial condition error! termination velocity of joint "
                << i << "is not zero" << std::endl;
      return false;
    }

    if (fabs(traj_para.end_point.joint_acc(i)) > epislon) {
      std::cerr << "Initial condition error! termination acceleration of joint "
                << i << "is not zero" << std::endl;
      return false;
    }

    if (fabs(traj_para.end_point.joint_jerk(i)) > epislon) {
      std::cerr << "Initial condition error! termination jerk of joint " << i
                << "is not zero" << std::endl;
      return false;
    }
  }

  double norm_vel = 1.0e10, norm_acc = 1.0e10, norm_jerk = 1.0e10;
  double tmp_vel, tmp_acc, tmp_jerk;
  bool delta_pos_is_zero = true;
  for (size_t i = 0; i < nr_of_joints; i++) {
    if (fabs(delta_pos(i)) < epislon) {
      continue;
    }
    delta_pos_is_zero = false;
    if (fabs(traj_para.parameter.joint_max_vel(i)) < epislon) {
      std::cerr << "Initial condition error! max velocity of joint " << i
                << "can't be zero" << std::endl;
      return false;
    }

    if (fabs(traj_para.parameter.joint_max_acc(i)) < epislon) {
      std::cerr << "Initial condition error! max acceleration of joint " << i
                << "can't be zero" << std::endl;
      return false;
    }

    if (fabs(traj_para.parameter.joint_max_vel(i)) < epislon) {
      std::cerr << "Initial condition error! max jerk of joint " << i
                << "can't be zero" << std::endl;
      return false;
    }

    tmp_vel = fabs(traj_para.parameter.joint_max_vel(i) / delta_pos(i));

    tmp_acc = fabs(traj_para.parameter.joint_max_acc(i) / delta_pos(i));

    tmp_jerk = fabs(traj_para.parameter.joint_max_jerk(i) / delta_pos(i));

    if (tmp_vel < norm_vel) {
      norm_vel = tmp_vel;
    }
    if (tmp_acc < norm_acc) {
      norm_acc = tmp_acc;
    }
    if (tmp_jerk < norm_jerk) {
      norm_jerk = tmp_jerk;
    }
  }
  if (delta_pos_is_zero) {
    std::cerr << "Initial condition error!The starting and ending joints are "
                 "the same point."
              << std::endl;
    return false;
  }
  if (velocity_curve.init(0.0, 1.0, norm_vel, norm_acc, norm_jerk,
                          traj_para.period)) {
    total_time = velocity_curve.getTotalTime();
    return true;
  } else {
    return false;
  }
};

TrajectPoint moveJ::calculatePointAtT(double t) {
  TrajectPoint res;
  res.time = t;
  NormalizationPoint norm_point = velocity_curve.calculatePointAtT(t);
  res.joint_jerk = delta_pos * norm_point.jerk;
  res.joint_acc = delta_pos * norm_point.acc;
  res.joint_vel = delta_pos * norm_point.vel;
  res.joint_pos = delta_pos * norm_point.pos + start_pos;
  return res;
}

} // namespace planning
