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

#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "arx_lift_src/lift_head_control_loop.h"

#include <algorithm>
#include <atomic>
#include <memory>
#include <string>
#include <thread>

namespace arxlift2s_ros2_control
{

/**
 * @brief Lift2S 升降柱 ros2_control 硬件接口。
 *
 * 封装官方 arx::LiftHeadControlLoop（libarx_lift_src.so，默认 can5）。
 *
 * 单位约定：
 * - ros2_control / URDF：米（约 0–0.48）
 * - SDK getHeight / Hybrid 打包：电机弧度（约 0–20）
 * - 换算：sdk_rad = meters * height_rad_per_meter（默认 41.54）
 *
 * 电机模式（URDF ``lift_motor_mode``，运行时可改 ``arx_lift.motor_mode``）：
 * - ``soft_p`` / ``position`` — Soft-P：``setHeight`` / ``loop()``，**只用上层 position**
 *   （忽略 velocity / effort / kp / kd）；增益 ``arx_lift.soft_p_kp``。
 * - ``hybrid`` — Hybrid MIT：``sendLiftHybrid``；position 斜坡 + **仅 HI**
 *   ``gravity_compensation_torque`` 前馈（**忽略上层 effort**，防 OCS2 RNEA 双重前馈）；
 *   增益 ``arx_lift.hybrid_kp/kd``（kd 打包上限 ≤5）。全身/分体均可选。
 *
 * 导出 MIX 指令接口供 ocs2_wbc claim；soft_p/hybrid 下 vel/effort 均可被写入但 HI 不驱动电机。
 */
class ArxLiftHardware : public hardware_interface::SystemInterface
{
public:
  enum class MotorMode : int
  {
    SoftP = 0,
    Hybrid = 1,
  };

  ArxLiftHardware() = default;
  ~ArxLiftHardware() override;

  /** @brief 解析 hardware 参数并初始化状态/指令缓冲。 */
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  /** @brief 启动 SDK 与后台控制循环，校准后打开指令跟踪。 */
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /** @brief 可选降高 + soft stop 后停循环；保留 lift_ 避免 CAN 线程崩溃。 */
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /** @brief 有序退出（幂等）；同样不销毁 lift_。 */
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  /** @brief 故障：仅 soft stop，不降高度。 */
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  /** @brief 导出升降 position/velocity/effort 状态接口。 */
  std::vector<hardware_interface::StateInterface::ConstSharedPtr>
  on_export_state_interfaces() override;

  /** @brief 导出 MIX 指令接口（供全身 OCS2 claim）。 */
  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_command_interfaces() override;

  /** @brief 状态由后台线程刷新；此处仅检查 SDK 存活。 */
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /** @brief 调试日志与 /arx_lift/motor_status；不直接驱动电机。 */
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /** @brief 停止后台 CAN 循环线程。 */
  void stop_loop_thread();

  /** @brief 解析 soft_p / hybrid 电机模式字符串。 */
  static bool parseMotorMode(const std::string & raw, MotorMode & out);

  /** @brief 电机模式枚举转参数名。 */
  static const char * motorModeName(MotorMode mode);

  /** @brief 声明并注册 arx_lift.* / status_debug 热调参数回调。 */
  void setupDynamicParameters(
    const std::string & initial_mode, double soft_p_kp, double hybrid_kp,
    double hybrid_kd, double gravity_comp, bool status_debug);

  /**
   * @brief Hybrid 斜坡跟踪并下发 sendLiftHybrid。
   * @param q_target_sdk 目标高度（SDK 弧度）
   * @param dt_s 本周期时间步（秒）
   */
  void sendHybridHoldOrTrack(double q_target_sdk, double dt_s);

  /** @brief 有序退出：可选降高 + soft stop，再停线程（幂等）。 */
  void enterSafeExit(bool allow_return_home);
  void softStopLift();
  void interpolateLiftToShutdownHeight();

  /** @brief ROS 米 → SDK 电机弧度。 */
  double rosToSdk(double ros_m) const
  {
    const double m = std::clamp(ros_m, 0.0, height_span_m_);
    const double sdk = m * height_rad_per_meter_;
    return std::clamp(sdk, 0.0, sdk_max_rad_);
  }

  /** @brief SDK 电机弧度 → ROS 米。 */
  double sdkToRos(double sdk_rad) const
  {
    if (height_rad_per_meter_ <= 0.0) {
      return 0.0;
    }
    return sdk_rad / height_rad_per_meter_;
  }

  std::string lift_joint_name_;
  double lift_position_{0.0};   ///< 状态：位置 [m]
  double lift_velocity_{0.0};   ///< 状态：速度 [m/s]
  double lift_effort_{0.0};     ///< 状态：力矩反馈
  double lift_position_command_{0.0};  ///< 指令：位置 [m]（实际跟踪用）
  double lift_velocity_command_{0.0};  ///< 指令：速度（HI 忽略，仅供 MIX claim）
  double lift_effort_command_{0.0};    ///< 指令：力矩（Hybrid 下叠加到 τ_ff）
  double lift_kp_command_{0.0};        ///< 指令：kp（HI 忽略，用 arx_lift.*）
  double lift_kd_command_{0.0};        ///< 指令：kd（HI 忽略，用 arx_lift.*）

  std::string can_name_{"can5"};
  int robot_type_{0};
  double lift_max_vel_{0.20};
  double lift_max_torque_{15.0};              ///< Hybrid τ_ff 限幅
  double cmd_ramp_vel_mps_{0.04};             ///< 位置斜坡限速 [m/s]

  // Soft-P / Hybrid 增益与重力（运行时可改 arx_lift.*）
  std::atomic<double> soft_p_kp_{8.0};
  std::atomic<double> hybrid_kp_{5.0};
  std::atomic<double> hybrid_kd_{2.0};
  std::atomic<double> gravity_compensation_torque_{-1.8};  ///< HI 重力前馈

  std::string motor_mode_param_{"hybrid"};
  std::atomic<int> motor_mode_{static_cast<int>(MotorMode::Hybrid)};
  int last_applied_mode_{-1};  ///< 上次已应用模式；切模式时重对齐斜坡

  double height_rad_per_meter_{41.54};
  double height_span_m_{0.48};
  double sdk_max_rad_{20.0};

  double ramp_q_sdk_{0.0};     ///< 斜坡当前高度（SDK 弧度）
  bool ramp_initialized_{false};

  std::shared_ptr<arx::LiftHeadControlLoop> lift_;

  std::thread loop_thread_;
  std::atomic<bool> loop_running_{false};
  std::atomic<bool> command_enabled_{false};  ///< false：仅校准/停车 Soft-P
  std::atomic<double> last_written_height_{0.0};
  std::atomic<double> last_written_vel_{0.0};

  std::atomic<double> motor_position_{0.0};
  std::atomic<double> motor_velocity_{0.0};
  std::atomic<double> motor_torque_{0.0};
  std::atomic<double> motor_current_{0.0};
  std::atomic<int> motor_online_{0};
  std::atomic<int> motor_error_{0};
  std::atomic<double> sdk_get_height_{0.0};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr motor_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  /** @brief write() 周期状态日志开关（节点参数 ``status_debug``）。 */
  std::atomic<bool> status_debug_{false};

  // Deactivate: optional ramp to height then soft stop (default: soft stop only).
  bool shutdown_return_home_{false};
  double shutdown_height_m_{0.0};
  double shutdown_home_velocity_{0.10};
  double shutdown_home_timeout_sec_{2.0};
  std::atomic<bool> safe_exit_done_{false};
  std::atomic<bool> soft_stop_active_{false};  ///< loop 发阻尼/零刚度包
};

}  // namespace arxlift2s_ros2_control
