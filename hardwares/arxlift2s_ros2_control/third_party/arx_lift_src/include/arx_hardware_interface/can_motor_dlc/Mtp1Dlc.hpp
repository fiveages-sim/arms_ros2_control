#pragma once

#include "arx_hardware_interface/canbase/CanBaseDef.hpp"

#include "arx_hardware_interface/can_motor_dlc/MotorDlcBase.hpp"

#include "arx_hardware_interface/typedef/HybridJointTypeDef.hpp"

#include <stdio.h>

#include <cmath>

#include <chrono>

namespace arx {
namespace hw_interface {
class MotorType1 : public MotorDlcBase {
public:
  MotorType1(int motor_id) : motor_id_(motor_id) {
    last_update_time_exchange_ = std::chrono::system_clock::now();
  };

  CanFrame packMotorMsg(HybridJointCmd *command) override;
  CanFrame packMotorMsg(double k_p, double k_d, double position,
                        double velocity, double torque) override;

  CanFrame packEnableMotor() override;
  CanFrame packDisableMotor() override;
  CanFrame packClearError() override;

  void CanAnalyze(CanFrame *frame) override; // 尝试接收电机数据

  HybridJointStatus GetMotorMsg() override;
  bool online() override {
    std::chrono::time_point<std::chrono::system_clock> now =
        std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        now - last_update_time_);
    if (duration.count() > 200000 && motor_id_ < 5)
      online_ = false;
    if (duration.count() > 800000)
      online_ = false;
    return online_;
  }
  void ExchangeMotorMsg() override;

private:
  double position_ = 0;
  double velocity_ = 0;
  double current_ = 0;
  bool online_{true};
  std::chrono::system_clock::time_point last_update_time_;

  double position_exchange_ = 0;
  double velocity_exchange_ = 0;
  double current_exchange_ = 0;
  int error_exchange_ = 1;
  std::chrono::system_clock::time_point last_update_time_exchange_;

  CanFrame *frame_;
  int motor_id_;
};
} // namespace hw_interface
} // namespace arx
