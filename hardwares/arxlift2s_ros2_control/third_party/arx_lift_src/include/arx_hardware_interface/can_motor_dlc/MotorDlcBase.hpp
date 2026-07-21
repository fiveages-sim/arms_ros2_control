#pragma once

#include "arx_hardware_interface/canbase/CanBaseDef.hpp"
#include "arx_hardware_interface/typedef/HybridJointTypeDef.hpp"

namespace arx {
namespace hw_interface {
class MotorDlcBase {
public:
  virtual ~MotorDlcBase() {}

  virtual CanFrame packMotorMsg(HybridJointCmd *command) = 0;
  virtual CanFrame packMotorMsg(double k_p, double k_d, double position,
                                double velocity, double torque) = 0;

  virtual void CanAnalyze(CanFrame *frame) = 0;

  virtual CanFrame packEnableMotor() = 0;
  virtual CanFrame packDisableMotor() = 0;
  virtual CanFrame packSetZero() { return CanFrame{}; }
  virtual CanFrame packClearError() { return {}; };

  virtual HybridJointStatus GetMotorMsg() = 0;
  virtual bool online() = 0;
  virtual void resetCircle() {}
  virtual void ExchangeMotorMsg() = 0;
  int getErrorCode() { return error_; }

protected:
  double restrictBound(double num, double max, double min) {
    if (num > max)
      return max;
    else if (num < min)
      return min;
    else
      return num;
  }
  int error_;
};
} // namespace hw_interface
} // namespace arx