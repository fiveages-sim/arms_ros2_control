#pragma once

#include "arx_hardware_interface/canbase/CanBaseDef.hpp"

namespace arx {
namespace hw_interface {
class ChassisType1 {
public:
  CanFrame packChassisCmd1(int v_x, int v_y, int v_z, int mode);
  CanFrame packChassisCmd2(double wheel1_vel, double wheel2_vel,
                           double wheel3_vel, double wheel4_vel);
  void unpackChassisFeedback(CanFrame &frame);
  void unpackOrientation(CanFrame &frame);
  void unpackAngularVel(CanFrame &frame);
  void unpackAccel(CanFrame &frame);
  void ExchangeData();
  void getWheelVel(double *vel);
  void getOrientation(double *orientation);
  void getAngularVel(double *angular_vel);
  void getAccel(double *accel);

private:
  float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
  }
  double wheel1_vel_, wheel2_vel_, wheel3_vel_, wheel4_vel_;
  double wheel1_vel_exchange_, wheel2_vel_exchange_, wheel3_vel_exchange_,
      wheel4_vel_exchange_;
  double roll_ = 0, pitch_ = 0, yaw_ = 0;
  double roll_exchange_ = 0, pitch_exchange_ = 0, yaw_exchange_ = 0;
  double angular_vel_[3] = {0, 0, 0};
  double angular_vel_exchange_[3] = {0, 0, 0};
  double accel_[3] = {0, 0, 0};
  double accel_exchange_[3] = {0, 0, 0};
};
} // namespace hw_interface
} // namespace arx
