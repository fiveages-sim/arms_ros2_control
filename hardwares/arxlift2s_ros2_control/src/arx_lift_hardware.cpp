// Copyright 2026 FiveAges Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file arx_lift_hardware.cpp
 * @brief Lift2S 升降柱 SystemInterface 实现。
 *
 * 控制路径概要：
 * - 后台线程 ~400 Hz 跟踪 position 指令（斜坡限速），按 soft_p / hybrid 下发
 * - soft_p（position）：只用 position → setHeight/loop；忽略 vel/effort/kp/kd
 * - hybrid：position 斜坡 + HI 重力前馈；忽略控制器 effort/vel/kp/kd（防 OCS2 RNEA 双重前馈）
 * - ros2_control 的 read()/write() 不做电机闭环；write 仅调试日志与状态话题
 */

#include "arxlift2s_ros2_control/arx_lift_hardware.h"

#include <chrono>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <stdexcept>
#include <thread>

namespace arxlift2s_ros2_control
{
namespace
{
/** @brief 热调参数名：电机模式 soft_p | hybrid。 */
constexpr const char * kMotorModeParam = "arx_lift.motor_mode";
/** @brief 热调参数名：Soft-P 位置增益。 */
constexpr const char * kSoftPKpParam = "arx_lift.soft_p_kp";
/** @brief 热调参数名：Hybrid MIT kp。 */
constexpr const char * kHybridKpParam = "arx_lift.hybrid_kp";
/** @brief 热调参数名：Hybrid MIT kd。 */
constexpr const char * kHybridKdParam = "arx_lift.hybrid_kd";
/** @brief 热调参数名：HI 重力前馈（Hybrid τ_ff；Soft-P 写入 SDK config）。 */
constexpr const char * kGravityCompParam = "arx_lift.gravity_compensation_torque";
/** @brief Hybrid Type3 打包 kd 上限（见 libarx_lift_src.so）。 */
constexpr double kHybridKdMax = 5.0;

/** @brief 读取 URDF/xacro hardware_parameters 字符串，缺省返回 default_value。 */
std::string get_hw_param(
  const hardware_interface::HardwareInfo & info, const std::string & key,
  const std::string & default_value = "")
{
  const auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) {
    return default_value;
  }
  return it->second;
}

/** @brief 解析布尔型 hardware 参数。 */
bool parse_bool_param(
  const hardware_interface::HardwareInfo & info, const std::string & key,
  bool default_value, bool & out)
{
  const std::string raw = get_hw_param(info, key, "");
  if (raw.empty()) {
    out = default_value;
    return true;
  }
  std::string s = raw;
  for (char & c : s) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  out = (s == "true" || s == "1" || s == "yes" || s == "on");
  return true;
}

/** @brief 解析浮点型 hardware 参数；非法字符串返回 false。 */
bool parse_double_param(
  const hardware_interface::HardwareInfo & info, const std::string & key,
  double default_value, double & out, const rclcpp::Logger & logger)
{
  const std::string raw = get_hw_param(info, key, "");
  if (raw.empty()) {
    out = default_value;
    return true;
  }
  try {
    out = std::stod(raw);
    return true;
  } catch (const std::exception &) {
    RCLCPP_ERROR(
      logger, "Invalid %s parameter: '%s'", key.c_str(), raw.c_str());
    return false;
  }
}

}  // namespace

/** @brief 将 "soft_p"/"hybrid"（及别名）解析为 MotorMode。 */
bool ArxLiftHardware::parseMotorMode(const std::string & raw, MotorMode & out)
{
  std::string s = raw;
  for (char & c : s) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  if (s == "soft_p" || s == "softp" || s == "soft-p" || s == "position") {
    out = MotorMode::SoftP;
    return true;
  }
  if (s == "hybrid") {
    out = MotorMode::Hybrid;
    return true;
  }
  return false;
}

/** @brief MotorMode → 参数字符串。 */
const char * ArxLiftHardware::motorModeName(MotorMode mode)
{
  return mode == MotorMode::SoftP ? "soft_p" : "hybrid";
}

/**
 * @brief 注册升降热调参数（模式 / soft_p_kp / hybrid_kp|kd / gravity / status_debug）。
 * @note 挂在 controller_manager 节点上，可用 ros2 param set 在线修改。
 */
void ArxLiftHardware::setupDynamicParameters(
  const std::string & initial_mode, double soft_p_kp, double hybrid_kp,
  double hybrid_kd, double gravity_comp, bool status_debug)
{
  auto node = get_node();
  if (!node) {
    RCLCPP_WARN(
      get_logger(),
      "No ROS node; arx_lift.* params will not be dynamically settable");
    return;
  }

  auto declare_or_set_string =
    [&](const char * name, const std::string & value) {
      if (!node->has_parameter(name)) {
        node->declare_parameter<std::string>(name, value);
      } else {
        node->set_parameter(rclcpp::Parameter(name, value));
      }
    };
  auto declare_or_set_double = [&](const char * name, double value) {
    if (!node->has_parameter(name)) {
      node->declare_parameter<double>(name, value);
    } else {
      node->set_parameter(rclcpp::Parameter(name, value));
    }
  };

  auto declare_or_set_bool = [&](const char * name, bool value) {
    if (!node->has_parameter(name)) {
      node->declare_parameter<bool>(name, value);
    } else {
      node->set_parameter(rclcpp::Parameter(name, value));
    }
  };

  declare_or_set_string(kMotorModeParam, initial_mode);
  declare_or_set_double(kSoftPKpParam, soft_p_kp);
  declare_or_set_double(kHybridKpParam, hybrid_kp);
  declare_or_set_double(kHybridKdParam, hybrid_kd);
  declare_or_set_double(kGravityCompParam, gravity_comp);
  declare_or_set_bool("status_debug", status_debug);
  status_debug_.store(status_debug);

  auto parse_nonneg = [](const rclcpp::Parameter & p, double & out,
                         std::string & reason, const char * label) -> bool {
    if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
      p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
    {
      reason = std::string(label) + " must be a number";
      return false;
    }
    out = p.as_double();
    if (!(out >= 0.0) || !std::isfinite(out)) {
      reason = std::string(label) + " must be finite and >= 0";
      return false;
    }
    return true;
  };

  auto parse_finite = [](const rclcpp::Parameter & p, double & out,
                         std::string & reason, const char * label) -> bool {
    if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE &&
      p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
    {
      reason = std::string(label) + " must be a number";
      return false;
    }
    out = p.as_double();
    if (!std::isfinite(out)) {
      reason = std::string(label) + " must be finite";
      return false;
    }
    return true;
  };

  param_cb_ = node->add_on_set_parameters_callback(
    [this, parse_nonneg, parse_finite](const std::vector<rclcpp::Parameter> & params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : params) {
        const auto & name = p.get_name();
        if (name == kMotorModeParam) {
          if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
            result.successful = false;
            result.reason = "arx_lift.motor_mode must be a string";
            return result;
          }
          MotorMode mode;
          if (!parseMotorMode(p.as_string(), mode)) {
            result.successful = false;
            result.reason = "arx_lift.motor_mode must be 'soft_p'|'position' or 'hybrid'";
            return result;
          }
          motor_mode_.store(static_cast<int>(mode));
          RCLCPP_INFO(
            get_logger(), "%s -> %s", kMotorModeParam, motorModeName(mode));
        } else if (name == kSoftPKpParam) {
          double v = 0.0;
          if (!parse_nonneg(p, v, result.reason, kSoftPKpParam)) {
            result.successful = false;
            return result;
          }
          soft_p_kp_.store(v);
          RCLCPP_INFO(get_logger(), "%s -> %.3f", kSoftPKpParam, v);
        } else if (name == kHybridKpParam) {
          double v = 0.0;
          if (!parse_nonneg(p, v, result.reason, kHybridKpParam)) {
            result.successful = false;
            return result;
          }
          hybrid_kp_.store(v);
          RCLCPP_INFO(get_logger(), "%s -> %.3f", kHybridKpParam, v);
        } else if (name == kHybridKdParam) {
          double v = 0.0;
          if (!parse_nonneg(p, v, result.reason, kHybridKdParam)) {
            result.successful = false;
            return result;
          }
          if (v > kHybridKdMax) {
            RCLCPP_WARN(
              get_logger(),
              "%s=%.3f exceeds Hybrid MIT pack max %.1f; will clamp when sending",
              kHybridKdParam, v, kHybridKdMax);
          }
          hybrid_kd_.store(v);
          RCLCPP_INFO(get_logger(), "%s -> %.3f", kHybridKdParam, v);
        } else if (name == kGravityCompParam) {
          double v = 0.0;
          if (!parse_finite(p, v, result.reason, kGravityCompParam)) {
            result.successful = false;
            return result;
          }
          gravity_compensation_torque_.store(v);
          RCLCPP_INFO(get_logger(), "%s -> %.3f", kGravityCompParam, v);
        } else if (name == "status_debug") {
          if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
            result.successful = false;
            result.reason = "status_debug must be a bool";
            return result;
          }
          status_debug_.store(p.as_bool());
          RCLCPP_INFO(
            get_logger(), "status_debug -> %s",
            status_debug_.load() ? "true" : "false");
        }
      }
      return result;
    });

  RCLCPP_INFO(
    get_logger(),
    "Dynamic params: %s=%s %s=%.3f %s=%.3f %s=%.3f %s=%.3f status_debug=%s",
    kMotorModeParam, initial_mode.c_str(), kSoftPKpParam, soft_p_kp,
    kHybridKpParam, hybrid_kp, kHybridKdParam, hybrid_kd,
    kGravityCompParam, gravity_comp,
    status_debug ? "true" : "false");
}

/**
 * @brief Hybrid 路径：斜坡逼近目标后 sendLiftHybrid。
 *
 * τ_ff = gravity_compensation_torque_（仅 HI 重力），再限幅。
 * **故意忽略** 上层 effort（全身 OCS2 RNEA），避免与 HI 重力双重前馈过流。
 * 位置/速度按电机坐标系取负（与 getHeight = -motor.position 一致）。
 * 忽略控制器 velocity/effort/kp/kd 指令接口。
 */
void ArxLiftHardware::sendHybridHoldOrTrack(double q_target_sdk, double dt_s)
{
  if (!lift_) {
    return;
  }

  if (!ramp_initialized_) {
    ramp_q_sdk_ = sdk_get_height_.load();
    ramp_initialized_ = true;
  }

  // 按 cmd_ramp_vel 限速逼近目标（单位：SDK 弧度）
  const double v_ramp_sdk = cmd_ramp_vel_mps_ * height_rad_per_meter_;
  const double err = q_target_sdk - ramp_q_sdk_;
  double v_d = 0.0;
  constexpr double kPosEps = 1e-4;
  if (std::abs(err) <= kPosEps) {
    ramp_q_sdk_ = q_target_sdk;
    v_d = 0.0;
  } else {
    const double max_step = v_ramp_sdk * dt_s;
    const double step = std::copysign(std::min(std::abs(err), max_step), err);
    ramp_q_sdk_ += step;
    v_d = step / dt_s;
  }
  ramp_q_sdk_ = std::clamp(ramp_q_sdk_, 0.0, sdk_max_rad_);

  const double hy_kp = hybrid_kp_.load();
  const double hy_kd = hybrid_kd_.load();

  // 仅 HI 重力前馈；不叠加 lift_effort_command_（OCS2 MIX 仍可写，但不下发电机）
  const double t_ff = std::clamp(
    gravity_compensation_torque_.load(), -lift_max_torque_, lift_max_torque_);
  (void)lift_effort_command_;

  lift_->read();
  lift_->exchangeLiftMotorMsg();
  const double p_motor = -ramp_q_sdk_;
  const double v_motor = -v_d;
  const double kd_send = std::min(hy_kd, kHybridKdMax);
  lift_->sendLiftHybrid(hy_kp, kd_send, p_motor, v_motor, t_ff);

  last_written_height_.store(ramp_q_sdk_);
  last_written_vel_.store(v_d);
}

/** @brief 关闭指令门控并 join 后台循环线程。 */
void ArxLiftHardware::stop_loop_thread()
{
  command_enabled_ = false;
  loop_running_ = false;
  if (loop_thread_.joinable()) {
    loop_thread_.join();
  }
}

/**
 * @brief 析构：停线程；故意不销毁 lift_，避免厂商 SocketCAN 线程纯虚崩溃。
 */
ArxLiftHardware::~ArxLiftHardware()
{
  stop_loop_thread();
  param_cb_.reset();
  motor_pub_.reset();
  // 故意泄漏 lift_：厂商 SocketCAN 收包线程仍在跑时销毁 LiftHeadControlLoop
  // 会触发 "pure virtual method called"。
}

/**
 * @brief 从 URDF hardware 参数加载 CAN、单位换算、增益与电机模式。
 * @return SUCCESS / ERROR
 */
hardware_interface::CallbackReturn ArxLiftHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 1) {
    RCLCPP_ERROR(
      get_logger(),
      "ArxLiftHardware expects exactly 1 joint, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  lift_joint_name_ = info_.joints.front().name;
  can_name_ = get_hw_param(info_, "can_name", "can5");
  const std::string robot_type_str = get_hw_param(info_, "robot_type", "2");
  try {
    robot_type_ = std::stoi(robot_type_str);
  } catch (const std::exception &) {
    RCLCPP_ERROR(
      get_logger(), "Invalid robot_type parameter: '%s'",
      robot_type_str.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (robot_type_ < 0 || robot_type_ > 2) {
    RCLCPP_ERROR(
      get_logger(), "robot_type must be 0(LIFT)/1(X7S)/2(LIFTS), got %d",
      robot_type_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  double gravity = -1.8;
  double max_vel = 0.20;
  double max_torque = 15.0;
  double soft_p_kp = 8.0;
  double hybrid_kp = 50.0;
  double hybrid_kd = 1.0;
  double height_scale = 41.54;
  double span_m = 0.48;
  double sdk_max = 20.0;
  double ramp_vel = 0.04;

  if (!parse_double_param(
      info_, "gravity_compensation_torque", -1.8, gravity, get_logger()) ||
    !parse_double_param(info_, "lift_max_vel", 0.20, max_vel, get_logger()) ||
    !parse_double_param(
      info_, "lift_max_torque", 15.0, max_torque, get_logger()) ||
    !parse_double_param(info_, "sdk_max_rad", 20.0, sdk_max, get_logger()) ||
    !parse_double_param(info_, "cmd_ramp_vel", 0.04, ramp_vel, get_logger()) ||
    !parse_double_param(info_, "hybrid_kp", 5.0, hybrid_kp, get_logger()))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 米 ↔ 电机弧度（优先 height_rad_per_meter；别名 height_to_motor）
  if (!get_hw_param(info_, "height_rad_per_meter", "").empty()) {
    if (!parse_double_param(
        info_, "height_rad_per_meter", 41.54, height_scale, get_logger()))
    {
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else if (!parse_double_param(
      info_, "height_to_motor", 41.54, height_scale, get_logger()))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!get_hw_param(info_, "height_span_m", "").empty()) {
    if (!parse_double_param(info_, "height_span_m", 0.48, span_m, get_logger())) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else if (!parse_double_param(info_, "max_height_m", 0.48, span_m, get_logger())) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // soft_p_kp（别名：lift_kp / kp）
  if (!get_hw_param(info_, "soft_p_kp", "").empty()) {
    if (!parse_double_param(info_, "soft_p_kp", 8.0, soft_p_kp, get_logger())) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else if (!get_hw_param(info_, "lift_kp", "").empty()) {
    if (!parse_double_param(info_, "lift_kp", 8.0, soft_p_kp, get_logger())) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else if (!get_hw_param(info_, "kp", "").empty()) {
    if (!parse_double_param(info_, "kp", 8.0, soft_p_kp, get_logger())) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // 优先 hybrid_kd；别名 kd
  if (!get_hw_param(info_, "hybrid_kd", "").empty()) {
    if (!parse_double_param(info_, "hybrid_kd", 2.0, hybrid_kd, get_logger())) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else if (!get_hw_param(info_, "kd", "").empty()) {
    if (!parse_double_param(info_, "kd", 2.0, hybrid_kd, get_logger())) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  auto ok_gain = [](double v) { return std::isfinite(v) && v >= 0.0; };
  if (!ok_gain(soft_p_kp) || !ok_gain(hybrid_kp) || !ok_gain(hybrid_kd)) {
    RCLCPP_ERROR(
      get_logger(),
      "soft_p_kp/hybrid_kp/hybrid_kd must be finite and >= 0 (got %.3f / %.3f / %.3f)",
      soft_p_kp, hybrid_kp, hybrid_kd);
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (height_scale <= 0.0 || span_m <= 0.0 || sdk_max <= 0.0 || ramp_vel <= 0.0) {
    RCLCPP_ERROR(
      get_logger(),
      "height scale / span / sdk_max / cmd_ramp_vel must be > 0");
    return hardware_interface::CallbackReturn::ERROR;
  }

  gravity_compensation_torque_.store(gravity);
  lift_max_vel_ = max_vel;
  lift_max_torque_ = max_torque;
  height_rad_per_meter_ = height_scale;
  height_span_m_ = span_m;
  sdk_max_rad_ = sdk_max;
  cmd_ramp_vel_mps_ = ramp_vel;
  soft_p_kp_.store(soft_p_kp);
  hybrid_kp_.store(hybrid_kp);
  hybrid_kd_.store(hybrid_kd);

  motor_mode_param_ = get_hw_param(info_, "lift_motor_mode", "hybrid");
  MotorMode mode;
  if (!parseMotorMode(motor_mode_param_, mode)) {
    RCLCPP_ERROR(
      get_logger(),
      "Invalid lift_motor_mode '%s' (use soft_p|position or hybrid)",
      motor_mode_param_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  motor_mode_.store(static_cast<int>(mode));
  motor_mode_param_ = motorModeName(mode);

  {
    bool dbg = false;
    parse_bool_param(info_, "status_debug", false, dbg);
    status_debug_.store(dbg);
  }

  parse_bool_param(info_, "shutdown_return_home", false, shutdown_return_home_);
  if (!parse_double_param(
      info_, "shutdown_height_m", 0.0, shutdown_height_m_, get_logger()) ||
    !parse_double_param(
      info_, "shutdown_home_velocity", 0.10, shutdown_home_velocity_,
      get_logger()) ||
    !parse_double_param(
      info_, "shutdown_home_timeout", 2.0, shutdown_home_timeout_sec_,
      get_logger()))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!std::isfinite(shutdown_home_velocity_) || shutdown_home_velocity_ <= 0.0) {
    shutdown_home_velocity_ = 0.10;
  }
  if (!std::isfinite(shutdown_home_timeout_sec_) ||
    shutdown_home_timeout_sec_ <= 0.0)
  {
    shutdown_home_timeout_sec_ = 2.0;
  }
  shutdown_height_m_ = std::clamp(shutdown_height_m_, 0.0, span_m);
  safe_exit_done_ = false;
  soft_stop_active_ = false;

  lift_position_ = 0.0;
  lift_velocity_ = 0.0;
  lift_effort_ = 0.0;
  lift_position_command_ = 0.0;
  lift_velocity_command_ = 0.0;
  lift_effort_command_ = 0.0;
  lift_kp_command_ = 0.0;
  lift_kd_command_ = 0.0;
  command_enabled_ = false;
  last_written_height_ = 0.0;
  last_written_vel_ = 0.0;
  last_applied_mode_ = -1;
  lift_.reset();

  RCLCPP_INFO(
    get_logger(),
    "ArxLiftHardware init: joint=%s can=%s robot_type=%d "
    "motor_mode=%s soft_p_kp=%.3f hybrid_kp=%.3f hybrid_kd=%.3f "
    "gravity=%.3f ramp=%.3f m/s shutdown_return_home=%s height=%.3f",
    lift_joint_name_.c_str(), can_name_.c_str(), robot_type_,
    motor_mode_param_.c_str(), soft_p_kp_.load(), hybrid_kp_.load(),
    hybrid_kd_.load(), gravity_compensation_torque_.load(), cmd_ramp_vel_mps_,
    shutdown_return_home_ ? "true" : "false", shutdown_height_m_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

/** @brief 导出 position / velocity / effort 状态接口。 */
std::vector<hardware_interface::StateInterface::ConstSharedPtr>
ArxLiftHardware::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
  state_interfaces.push_back(
    std::make_shared<hardware_interface::StateInterface>(
      lift_joint_name_, hardware_interface::HW_IF_POSITION, &lift_position_));
  state_interfaces.push_back(
    std::make_shared<hardware_interface::StateInterface>(
      lift_joint_name_, hardware_interface::HW_IF_VELOCITY, &lift_velocity_));
  state_interfaces.push_back(
    std::make_shared<hardware_interface::StateInterface>(
      lift_joint_name_, hardware_interface::HW_IF_EFFORT, &lift_effort_));
  return state_interfaces;
}

/**
 * @brief 导出 MIX 指令接口，供 ocs2_wbc claim。
 * @note soft_p（WBC 默认）：只用 position；velocity/effort/kp/kd 可写但不下发电机。
 *       hybrid：position + 可选 effort 前馈；kp/kd 仍由 arx_lift.* 参数决定。
 */
std::vector<hardware_interface::CommandInterface::SharedPtr>
ArxLiftHardware::on_export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
  command_interfaces.push_back(
    std::make_shared<hardware_interface::CommandInterface>(
      lift_joint_name_, hardware_interface::HW_IF_POSITION,
      &lift_position_command_));
  command_interfaces.push_back(
    std::make_shared<hardware_interface::CommandInterface>(
      lift_joint_name_, hardware_interface::HW_IF_VELOCITY,
      &lift_velocity_command_));
  command_interfaces.push_back(
    std::make_shared<hardware_interface::CommandInterface>(
      lift_joint_name_, hardware_interface::HW_IF_EFFORT,
      &lift_effort_command_));
  command_interfaces.push_back(
    std::make_shared<hardware_interface::CommandInterface>(
      lift_joint_name_, "kp", &lift_kp_command_));
  command_interfaces.push_back(
    std::make_shared<hardware_interface::CommandInterface>(
      lift_joint_name_, "kd", &lift_kd_command_));
  return command_interfaces;
}

/**
 * @brief 创建 SDK、启后台循环、等待校准后打开指令门控。
 *
 * 流程：构造 LiftHeadControlLoop → 底盘停车 → 注册热调参数 →
 * 循环线程（校准期 Soft-P）→ 等待约 2.5s → 指令对齐当前高度 → command_enabled。
 */
hardware_interface::CallbackReturn ArxLiftHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop_loop_thread();
  param_cb_.reset();

  try {
    const auto type =
      static_cast<arx::LiftHeadControlLoop::RobotType>(robot_type_);
    lift_ = std::make_shared<arx::LiftHeadControlLoop>(can_name_.c_str(), type);

    lift_->config_.lift_kp = soft_p_kp_.load();
    lift_->config_.gravity_compensation_torque =
      gravity_compensation_torque_.load();
    lift_->config_.lift_max_vel = lift_max_vel_;
    lift_->config_.lift_max_torque = lift_max_torque_;

    // 底盘停车模式（mode=2）
    lift_->setChassisCmd(0.0, 0.0, 0.0, 2);

    setupDynamicParameters(
      motor_mode_param_, soft_p_kp_.load(), hybrid_kp_.load(),
      hybrid_kd_.load(), gravity_compensation_torque_.load(),
      status_debug_.load());

    if (auto node = get_node()) {
      motor_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/arx_lift/motor_status", rclcpp::SystemDefaultsQoS());
      RCLCPP_INFO(
        get_logger(),
        "Publishing raw lift motor status on /arx_lift/motor_status");
    }

    loop_running_ = true;
    command_enabled_ = false;  // 校准完成前不跟踪外部指令
    last_applied_mode_ = -1;
    ramp_initialized_ = false;
    safe_exit_done_ = false;
    soft_stop_active_ = false;
    using namespace std::chrono_literals;
    // 后台控制循环：斜坡跟踪 + soft_p/hybrid 下发 + 刷新状态缓冲
    loop_thread_ = std::thread([this]() {
      auto last = std::chrono::steady_clock::now();
      while (loop_running_.load()) {
        try {
          if (!lift_) {
            std::this_thread::sleep_for(5ms);
            continue;
          }

          const auto now = std::chrono::steady_clock::now();
          double dt_s = std::chrono::duration<double>(now - last).count();
          last = now;
          // 限制 dt，避免长时间阻塞后一步跳过大
          if (dt_s < 1e-4) {
            dt_s = 1e-4;
          } else if (dt_s > 0.05) {
            dt_s = 0.05;
          }

          if (soft_stop_active_.load()) {
            // Soft stop at height: keep gravity τ_ff (τ=0 while elevated buzzes/drops).
            // hybrid: kp≈0 + kd + gravity; soft_p: hold current height.
            const double q_hold = sdk_get_height_.load();
            ramp_q_sdk_ = q_hold;
            const int mode_i = motor_mode_.load();
            if (mode_i == static_cast<int>(MotorMode::SoftP)) {
              lift_->config_.lift_kp = soft_p_kp_.load();
              lift_->config_.gravity_compensation_torque =
                gravity_compensation_torque_.load();
              lift_->setHeight(q_hold);
              lift_->loop();
            } else {
              const double hy_kd =
                std::min(std::max(hybrid_kd_.load(), 0.5), kHybridKdMax);
              const double t_ff = std::clamp(
                gravity_compensation_torque_.load(), -lift_max_torque_,
                lift_max_torque_);
              lift_->read();
              lift_->exchangeLiftMotorMsg();
              lift_->sendLiftHybrid(0.0, hy_kd, -q_hold, 0.0, t_ff);
            }
            last_written_height_.store(q_hold);
            last_written_vel_.store(0.0);
          } else if (!command_enabled_.load()) {
            // 校准 / 停车：始终 Soft-P loop()
            lift_->config_.lift_kp = soft_p_kp_.load();
            lift_->loop();
            ramp_initialized_ = false;
            last_applied_mode_ = -1;
          } else {
            const double q_target = rosToSdk(lift_position_command_);
            if (!ramp_initialized_) {
              ramp_q_sdk_ = sdk_get_height_.load();
              ramp_initialized_ = true;
            }

            const int mode_i = motor_mode_.load();
            const double soft_kp = soft_p_kp_.load();
            const double hy_kp = hybrid_kp_.load();
            const double hy_kd = hybrid_kd_.load();
            if (mode_i != last_applied_mode_) {
              // soft_p ↔ hybrid 切换时对齐斜坡，避免位置跳变
              ramp_q_sdk_ = sdk_get_height_.load();
              last_applied_mode_ = mode_i;
              RCLCPP_INFO(
                get_logger(),
                "Lift motor mode applied: %s (soft_p_kp=%.3f hybrid_kp=%.3f kd=%.3f)",
                motorModeName(static_cast<MotorMode>(mode_i)), soft_kp, hy_kp,
                hy_kd);
            }

            if (mode_i == static_cast<int>(MotorMode::SoftP)) {
              // Soft-P / position：只用上层 position，忽略 vel/effort/kp/kd
              const double v_ramp_sdk =
                cmd_ramp_vel_mps_ * height_rad_per_meter_;
              const double err = q_target - ramp_q_sdk_;
              double v_d = 0.0;
              constexpr double kPosEps = 1e-4;
              if (std::abs(err) <= kPosEps) {
                ramp_q_sdk_ = q_target;
                v_d = 0.0;
              } else {
                const double max_step = v_ramp_sdk * dt_s;
                const double step =
                  std::copysign(std::min(std::abs(err), max_step), err);
                ramp_q_sdk_ += step;
                v_d = step / dt_s;
              }
              ramp_q_sdk_ = std::clamp(ramp_q_sdk_, 0.0, sdk_max_rad_);

              // 故意不读 lift_velocity/effort/kp/kd_command_
              lift_->config_.lift_kp = soft_kp;
              lift_->config_.gravity_compensation_torque =
                gravity_compensation_torque_.load();
              lift_->setHeight(ramp_q_sdk_);
              lift_->loop();
              last_written_height_.store(ramp_q_sdk_);
              last_written_vel_.store(v_d);
            } else {
              // Hybrid：HI 重力前馈；忽略上层 effort（见 sendHybridHoldOrTrack）
              sendHybridHoldOrTrack(q_target, dt_s);
            }
          }

          // 刷新状态供 read()/话题使用
          const double sdk_h = lift_->getHeight();
          const auto motor = lift_->getLiftMotorStatus();
          sdk_get_height_.store(sdk_h);
          motor_position_.store(motor.position);
          motor_velocity_.store(motor.velocity);
          motor_torque_.store(motor.torque);
          motor_current_.store(motor.current);
          motor_online_.store(lift_->isLiftMotorOnline() ? 1 : 0);
          motor_error_.store(lift_->getLiftMotorErrorCode());

          lift_position_ = sdkToRos(sdk_h);
          lift_velocity_ = sdkToRos(-motor.velocity);
          lift_effort_ = motor.torque;
        } catch (const std::exception & e) {
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "ArxLiftHardware loop thread failed: %s", e.what());
        }
        std::this_thread::sleep_for(2500us);  // 约 400 Hz
      }
    });

    // 等待 SDK 内部升降校准
    constexpr auto kCalibWait = std::chrono::milliseconds(2500);
    std::this_thread::sleep_for(kCalibWait);

    // 指令对齐当前反馈，再打开外部跟踪
    lift_position_command_ = lift_position_;
    ramp_q_sdk_ = rosToSdk(lift_position_command_);
    ramp_initialized_ = true;
    last_written_height_.store(ramp_q_sdk_);
    last_written_vel_.store(0.0);
    command_enabled_ = true;

    RCLCPP_INFO(
      get_logger(),
      "ArxLiftHardware activated on %s (ros_height=%.3f m, sdk_rad=%.3f, "
      "motor_mode=%s soft_p_kp=%.3f hybrid_kp=%.3f hybrid_kd=%.3f gravity=%.3f)",
      can_name_.c_str(), lift_position_, sdk_get_height_.load(),
      motorModeName(static_cast<MotorMode>(motor_mode_.load())),
      soft_p_kp_.load(), hybrid_kp_.load(), hybrid_kd_.load(),
      gravity_compensation_torque_.load());
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to activate ArxLiftHardware: %s", e.what());
    stop_loop_thread();
    param_cb_.reset();
    lift_.reset();
    return hardware_interface::CallbackReturn::ERROR;
  }
}

/**
 * @brief 可选降高 + soft stop，再停循环、底盘停车；不销毁 lift_。
 */
hardware_interface::CallbackReturn ArxLiftHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "ArxLiftHardware on_deactivate: enterSafeExit");
  enterSafeExit(/*allow_return_home=*/true);
  param_cb_.reset();
  motor_pub_.reset();
  RCLCPP_INFO(get_logger(), "ArxLiftHardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 有序退出（幂等）；同样保留 lift_ 存活，避免 CAN 线程崩溃。
 */
hardware_interface::CallbackReturn ArxLiftHardware::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "ArxLiftHardware on_shutdown: enterSafeExit");
  enterSafeExit(/*allow_return_home=*/true);
  param_cb_.reset();
  motor_pub_.reset();
  RCLCPP_INFO(
    get_logger(),
    "ArxLiftHardware shutdown (lift SDK left alive to avoid CAN thread crash)");
  return hardware_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 故障：仅 soft stop + 停线程，不降高度。
 */
hardware_interface::CallbackReturn ArxLiftHardware::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(get_logger(), "ArxLiftHardware on_error: soft stop only");
  enterSafeExit(/*allow_return_home=*/false);
  param_cb_.reset();
  motor_pub_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

void ArxLiftHardware::enterSafeExit(bool allow_return_home)
{
  if (safe_exit_done_.exchange(true)) {
    return;
  }

  try {
    if (allow_return_home && shutdown_return_home_ && loop_running_.load() &&
      lift_)
    {
      interpolateLiftToShutdownHeight();
    }
    softStopLift();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "enterSafeExit failed: %s; forcing soft stop + stop thread",
      e.what());
    try {
      softStopLift();
    } catch (...) {
    }
  }

  stop_loop_thread();
  soft_stop_active_ = false;

  if (lift_) {
    try {
      lift_->setChassisCmd(0.0, 0.0, 0.0, 2);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Chassis park on safe exit failed: %s", e.what());
    }
  }
}

void ArxLiftHardware::interpolateLiftToShutdownHeight()
{
  if (!lift_ || !loop_running_.load()) {
    return;
  }

  const double goal =
    std::clamp(shutdown_height_m_, 0.0, height_span_m_);
  const double start = lift_position_;
  const double travel = std::abs(goal - start);
  constexpr double kMinDurationSec = 0.5;
  const double speed = std::max(0.02, std::abs(shutdown_home_velocity_));
  const double duration_sec = std::clamp(
    std::max(travel / speed, kMinDurationSec),
    kMinDurationSec,
    std::max(kMinDurationSec, shutdown_home_timeout_sec_));

  RCLCPP_WARN(
    get_logger(),
    "Shutdown interpolate lift: %.3f -> %.3f m, travel=%.3f, duration=%.2fs "
    "(ramp<=%.2f m/s)",
    start, goal, travel, duration_sec, speed);

  if (travel <= 0.01) {
    RCLCPP_INFO(get_logger(), "Lift already near shutdown height");
    lift_position_command_ = goal;
    return;
  }

  // Temporarily use shutdown velocity for the ramp while loop_thread_ tracks.
  const double old_ramp = cmd_ramp_vel_mps_;
  cmd_ramp_vel_mps_ = std::min(speed, lift_max_vel_);
  lift_position_command_ = goal;
  command_enabled_ = true;

  const auto t0 = std::chrono::steady_clock::now();
  constexpr auto kPoll = std::chrono::milliseconds(20);
  constexpr double kTolM = 0.015;
  while (loop_running_.load()) {
    const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
        .count();
    if (elapsed >= duration_sec) {
      break;
    }
    if (std::abs(lift_position_ - goal) <= kTolM) {
      break;
    }
    std::this_thread::sleep_for(kPoll);
  }

  cmd_ramp_vel_mps_ = old_ramp;
  RCLCPP_INFO(
    get_logger(),
    "Lift shutdown interpolate done (feedback=%.3f m, goal=%.3f m)",
    lift_position_, goal);
}

void ArxLiftHardware::softStopLift()
{
  if (!lift_ || !loop_running_.load()) {
    return;
  }

  // Hold current height; hybrid path uses kp≈0 / τ=0 via soft_stop_active_.
  lift_position_command_ = lift_position_;
  RCLCPP_INFO(
    get_logger(),
    "Lift soft stop (mode=%s, hold=%.3f m)",
    motorModeName(static_cast<MotorMode>(motor_mode_.load())),
    lift_position_command_);
  soft_stop_active_ = true;
  std::this_thread::sleep_for(std::chrono::milliseconds(250));
}

/**
 * @brief 状态已由后台线程写入缓冲；此处仅做存活检查。
 */
hardware_interface::return_type ArxLiftHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!lift_) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

/**
 * @brief 不向电机发指令；可选 status_debug 日志，并发布 /arx_lift/motor_status。
 */
hardware_interface::return_type ArxLiftHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!lift_) {
    return hardware_interface::return_type::ERROR;
  }

  const double err = lift_position_command_ - lift_position_;
  const double motor_pos = motor_position_.load();
  const double sdk_h = sdk_get_height_.load();
  const int err_code = motor_error_.load();
  const std::string err_code_str =
    (err_code >= 0 && err_code < 256) ? std::to_string(err_code) : "n/a";
  const auto mode = static_cast<MotorMode>(motor_mode_.load());
  if (status_debug_.load()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "ArxLiftHardware mode=%s cmd=%.4f m feedback=%.4f m err=%.4f m | "
      "sdk_rad=%.4f motor_pos=%.4f "
      "vel_rad=%.4f torq=%.4f curr=%.4f online=%d err_code=%s "
      "(last q=%.4f v=%.4f rad/s, soft_p_kp=%.3f hybrid_kp=%.3f hybrid_kd=%.3f)",
      motorModeName(mode), lift_position_command_, lift_position_, err, sdk_h,
      motor_pos, motor_velocity_.load(), motor_torque_.load(),
      motor_current_.load(), motor_online_.load(), err_code_str.c_str(),
      last_written_height_.load(), last_written_vel_.load(), soft_p_kp_.load(),
      hybrid_kp_.load(), hybrid_kd_.load());
  }

  // data: pos,vel,torq,curr,sdk_h,online,err, -pos, mode, soft_p_kp, hybrid_kp, hybrid_kd
  if (motor_pub_) {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
      motor_pos,
      motor_velocity_.load(),
      motor_torque_.load(),
      motor_current_.load(),
      sdk_h,
      static_cast<double>(motor_online_.load()),
      static_cast<double>(motor_error_.load()),
      -motor_pos,
      static_cast<double>(motor_mode_.load()),
      soft_p_kp_.load(),
      hybrid_kp_.load(),
      hybrid_kd_.load()};
    motor_pub_->publish(msg);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace arxlift2s_ros2_control

PLUGINLIB_EXPORT_CLASS(
  arxlift2s_ros2_control::ArxLiftHardware, hardware_interface::SystemInterface)
