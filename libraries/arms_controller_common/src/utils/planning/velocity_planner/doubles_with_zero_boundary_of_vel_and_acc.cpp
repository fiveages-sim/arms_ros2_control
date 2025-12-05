#include "arms_controller_common/utils/planning/velocity_planner/doubles_with_zero_boundary_of_vel_and_acc.h"
#include <iostream>
#include <math.h>
namespace planning {

bool DoublesWithZeroBoundaryOfVelAndAcc::init(double _start_pos,
                                              double _end_pos, double _max_vel,
                                              double _max_acc, double _max_jerk,
                                              double _period) {
  const double min_value = 1.0e-6;
  start_pos = _start_pos;
  end_pos = _end_pos;
  max_vel = fabs(_max_vel);
  max_acc = fabs(_max_acc);
  max_jerk = fabs(_max_jerk);
  period = fabs(_period);
  if (max_jerk < min_value || max_acc < min_value || max_vel < min_value) {
    return false;
  }

  double delta_pos = end_pos - start_pos;
  if (delta_pos < 0) {
    delta_pos = fabs(delta_pos);
    direction = -1.0;
    start_pos *= -1.0;
    end_pos *= -1.0;
  }
  if (delta_pos < min_value) {
    tj1 = 0.0;
    ta = 0.0;
    tj2 = 0.0;
    td = 0.0;
    tv = 0.0;
    totaltime = 0.0;
    return true;
  }

  if (max_vel * max_jerk < pow(max_acc, 2.0)) {
    tj1 = sqrt(max_vel / max_jerk);
    ta = 2.0 * tj1;

  } else {
    tj1 = max_acc / max_jerk;
    ta = tj1 + max_vel / max_acc;
  }
  tv = delta_pos / max_vel - ta;
  if (tv > 0) {
    tj2 = tj1;
    td = ta;
    totaltime = ta + tv + td;
  } else {
    if (delta_pos < 2.0 * pow(max_acc, 3.0) / pow(max_jerk, 2.0)) {
      tj1 = cbrt(0.5 * delta_pos / max_jerk);
      ta = 2.0 * tj1;
      tv = 0.0;
      tj2 = tj1;
      td = ta;
      totaltime = ta + tv + td;
    } else {
      tj1 = max_acc / max_jerk;
      ta = 0.5 * tj1 + sqrt(pow(0.5 * tj1, 2.0) + delta_pos / max_acc);
      tv = 0.0;
      tj2 = tj1;
      td = ta;
      totaltime = ta + tv + td;
    }
  }
  return true;
};

NormalizationPoint
DoublesWithZeroBoundaryOfVelAndAcc::calculatePointAtT(double t) {
  NormalizationPoint point;
  double alima = max_jerk * tj1;
  double alimd = -max_jerk * tj2;
  double v0 = 0.0;
  double v1 = 0.0;
  double vlim = v0 + (ta - tj1) * alima;
  if (t >= 0 && t <= tj1) {
    point.jerk = max_jerk;
    point.acc = max_jerk * t;
    point.vel = v0 + 0.5 * max_jerk * t * t;
    point.pos = start_pos + v0 * t + max_jerk * t * t * t / 6.0;
  } else if (t > tj1 && t <= ta - tj1) {
    point.jerk = 0.0;
    point.acc = alima;
    point.vel = v0 + alima * (t - 0.5 * tj1);
    point.pos = start_pos + v0 * t +
                alima * (3.0 * t * t - 3.0 * tj1 * t + tj1 * tj1) / 6.0;
  } else if (t > ta - tj1 && t <= ta) {
    point.jerk = -max_jerk;
    point.acc = max_jerk * (ta - t);
    point.vel = vlim - 0.5 * max_jerk * pow(ta - t, 2.0);
    point.pos = start_pos + 0.5 * (vlim + v0) * ta - vlim * (ta - t) +
                max_jerk * pow(ta - t, 3.0) / 6.0;
  } else if (t > ta && t <= ta + tv) {
    point.jerk = 0.0;
    point.acc = 0.0;
    point.vel = vlim;
    point.pos = start_pos + 0.5 * (vlim + v0) * ta + vlim * (t - ta);
  } else if (t > ta + tv && t <= ta + tv + tj2) {
    point.jerk = -max_jerk;
    point.acc = -max_jerk * (t - totaltime + td);
    point.vel = vlim - 0.5 * max_jerk * pow(t - totaltime + td, 2.0);
    point.pos = end_pos - 0.5 * (vlim + v1) * td + vlim * (t - totaltime + td) -
                max_jerk * pow(t - totaltime + td, 3.0) / 6;
  } else if (t > ta + tv + tj2 && t <= totaltime - tj2) {
    point.jerk = 0.0;
    point.acc = alimd;
    point.vel = vlim + alimd * (t - totaltime + td - 0.5 * tj2);
    point.pos = end_pos - 0.5 * (vlim + v1) * td + vlim * (t - totaltime + td) -
                max_jerk * pow(t - totaltime + td, 3.0) / 6.0;
  } else if (t > totaltime - tj2 && t <= totaltime) {
    point.jerk = max_jerk;
    point.acc = -max_jerk * (totaltime - t);
    point.vel = v1 + 0.5 * max_jerk * pow(totaltime - t, 2.0);
    point.pos = end_pos - v1 * (totaltime - t) -
                max_jerk * pow(totaltime - t, 3.0) / 6.0;
  } else if (t >= totaltime) {
    point.jerk = 0.0;
    point.acc = 0.0;
    point.vel = 0.0;
    point.pos = end_pos;
  }
  point.time = t;
  return point * direction;
};

} // namespace planning