//
// Created by arx4060 on 24-12-4.
//

#pragma once

#include "arx_hardware_interface/ARXJoy.hpp"
#include "arx_hardware_interface/can_motor_dlc/Mtp2Dlc.hpp"
#include "arx_hardware_interface/can_motor_dlc/Mtp3Dlc.hpp"
#include "arx_hardware_interface/canbase/SocketCan.hpp"
#include "arx_hardware_interface/chassis_dlc/Chassis1Dlc.hpp"
#include <chrono>

using namespace arx;
using namespace hw_interface;

namespace arx {
class LiftHeadControlLoop {
 public:
  enum class RobotType {
    LIFT,
    X7S,
    LIFTS
  };
  struct config {
    double gravity_compensation_torque = 0;
    double lift_max_vel = 0;
    double lift_kp = 0;
    double lift_calibrate_vel = 3;
    double lift_max_torque = 15;
  } config_;
  LiftHeadControlLoop(const char *bus_name, RobotType robot_type);
  ~LiftHeadControlLoop();
  void setRobotType(RobotType robot_type);
  void setHeight(double height);
  void setWaistPos(double pos);
  void setHeadYaw(double yaw_des);
  void setHeadPitch(double pitch_des);
  void setChassisCmd(double v_x, double v_y, double w_z, int mode);
  void setWheelVel(double wheel1_vel, double wheel2_vel, double wheel3_vel, double wheel4_vel);
  double getHeight();
  double getWaistPos();
  double getHeadYaw();
  double getHeadPitch();
  void getWheelVel(double *vel);
  void getOrientation(double *orientation);
  void getAngularVel(double *angular_vel);
  void getAccel(double *accel);
  void read();
  void update();
  void write();
  void protect();
  void loop() {
    read();
    if (!protect_) {
      update();
      write();
    } else {
      protect();
    }
  }

  // Header-only accessors (inline; no new .so symbols).
  HybridJointStatus getLiftMotorStatus()
  {
    return lift_motor_ ? lift_motor_->GetMotorMsg() : HybridJointStatus{};
  }
  bool isLiftMotorOnline()
  {
    return lift_motor_ && lift_motor_->online();
  }
  int getLiftMotorErrorCode()
  {
    return lift_motor_ ? lift_motor_->getErrorCode() : -1;
  }

  /** Swap double-buffered motor status (Soft-P update() does this). */
  void exchangeLiftMotorMsg()
  {
    if (lift_motor_) {
      lift_motor_->ExchangeMotorMsg();
    }
  }

  /**
   * Bypass Soft-P: pack Type3 Hybrid (kp,kd,p,v,t) and send on can.
   * Position/velocity must be in **motor** frame (getHeight = -motor.position).
   */
  bool sendLiftHybrid(
    double k_p, double k_d, double position, double velocity, double torque)
  {
    if (!lift_motor_) {
      return false;
    }
    CanFrame frame =
      lift_motor_->packMotorMsg(k_p, k_d, position, velocity, torque);
    return socket_can_.ExchangeData(&frame);
  }

 protected:
  double max_height_ = 20;
  double lift_motor_pos_des_{}, waist_motor_pos_des_{}, head_yaw_motor_pos_des_{},
      head_pitch_motor_pos_des_{};
  double lift_pos_current_des_{};
  double waist_pos_current_des_{};
  double lift_motor_torque_des_{-1}, lift_motor_vel_des_{};
  bool lift_is_calibrated_ = false, waist_is_calibrated_ = false;
  bool head_initial_pos_flag_ = false;
  bool head_init_complete_ = false;
  int head_init_count_ = 0;
  std::chrono::system_clock::time_point calibrated_start_time_;
  int calibrated_count_ = 0;
  int waist_calibrated_count_ = 0;
  // chassis
  int mode_ = 2;
  int p_x_ = 0, p_y_ = 0, p_z_ = 0;
  double wheel1_vel_, wheel2_vel_, wheel3_vel_, wheel4_vel_;
  double max_vel_x_, max_vel_y_, max_vel_z_;

 private:
  SocketCan socket_can_;
  std::shared_ptr<MotorDlcBase> lift_motor_;
  MotorType3 waist_motor_, head_yaw_motor_, head_pitch_motor_;
  ChassisType1 chassis_;
  ARXJoy switch_;
  RobotType robot_type_;
  bool protect_ = false;
  void recieveFrameCallback(CanFrame *frame);
  void readFromBuffer();
};
}// namespace arx
