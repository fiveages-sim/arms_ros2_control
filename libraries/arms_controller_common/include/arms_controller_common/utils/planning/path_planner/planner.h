#pragma once
#include "arms_controller_common/utils/planning/common/trajectory_init_parameters.h"
namespace planning {
class Planner {

public:
  Planner(){};
  virtual bool init(const TrajectoryInitParameters &traj_para) = 0;
  virtual TrajectPoint calculatePointAtT(double t) = 0;
  double getTotalTime() { return total_time; };

  ~Planner() = default;

  double total_time = 0.0;
};

} // namespace planning
