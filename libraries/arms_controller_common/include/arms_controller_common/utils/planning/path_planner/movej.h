#pragma once
#include "doubles_with_zero_boundary_of_vel_and_acc.h"
#include "planner.h"

namespace planning {
class moveJ : public Planner {
public:
  moveJ(){};
  bool init(const TrajectoryInitParameters &traj_para) override final;
  TrajectPoint calculatePointAtT(double t) override final;
  ~moveJ() = default;
  JointVector start_pos;
  JointVector delta_pos;
  DoublesWithZeroBoundaryOfVelAndAcc velocity_curve;
};

} // namespace planning
