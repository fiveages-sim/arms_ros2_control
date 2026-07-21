#pragma once

#include "arx_hardware_interface/canbase/CanBaseDef.hpp"

#include "arx_hardware_interface/can_motor_dlc/MotorDlcBase.hpp"

#include "arx_hardware_interface/typedef/HybridJointTypeDef.hpp"

#include <stdio.h>

#include <chrono>
#include <cmath>

namespace arx {
namespace hw_interface {
class MotorType3 : public MotorDlcBase {
public:
  MotorType3(int motor_id) : motor_id_(motor_id) {};

  CanFrame packMotorMsg(HybridJointCmd *command) override;
  CanFrame packMotorMsg(double k_p, double k_d, double position,
                        double velocity, double torque) override;

  CanFrame packEnableMotor() override;
  CanFrame packSetZero() override;
  CanFrame packDisableMotor() override;
  CanFrame packClearError() override;

  void CanAnalyze(CanFrame *frame) override; // 尝试接收电机数据
  void resetCircle() override;

  HybridJointStatus GetMotorMsg() override;
  bool online() override {
    std::chrono::time_point<std::chrono::system_clock> now =
        std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        now - last_update_time_);
    if (duration.count() > 100000)
      online_ = false;
    return online_;
  }
  void ExchangeMotorMsg() override;

private:
  double position_;
  double velocity_;
  double current_;
  bool online_{true};
  std::chrono::system_clock::time_point last_update_time_;

  double position_exchange_ = 0;
  double last_q_raw_ = 0;
  int circle_ = 0;
  double velocity_exchange_ = 0;
  double current_exchange_ = 0;
  std::chrono::system_clock::time_point last_update_time_exchange_;

  CanFrame *frame_;
  int motor_id_;
};
} // namespace hw_interface
} // namespace arx
