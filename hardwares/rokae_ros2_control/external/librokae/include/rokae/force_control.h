/**
 * @file force_control.h
 * @brief 非实时模式力控指令
 * @copyright Copyright (C) 2024 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef XCORESDK_INCLUDE_ROKAE_FORCE_CONTROL_H_
#define XCORESDK_INCLUDE_ROKAE_FORCE_CONTROL_H_

#include "base.h"
#include "data_types.h"

namespace rokae {

 // forward declarations
 class XService;

 /**
  * @class BaseForceControl
  * @brief 力控指令类
  */
 class XCORE_API BaseForceControl {

  public:

   /**
    * @brief 力控初始化
    * @param[in] frame_type 力控坐标系，支持world/wobj/tool/base/flange。工具工件坐标系使用setToolset()设置的坐标系
    * @param[out] ec 错误码
    */
   void fcInit(FrameType frame_type, error_code &ec) noexcept;

   /**
    * @brief 开始力控，fcInit()之后调用。
    * 如需在力控模式下执行运动指令，fcStart()之后可执行。
    * 注意，如果在fcStart()之前通过moveAppend()下发了运动指令但未开始运动，fcStart之后就会执行这些运动指令。
    * @param[out] ec 错误码
    */
   void fcStart(error_code &ec) noexcept;

   /**
    * @brief 停止力控
    * @param[out] ec 错误码
    */
   void fcStop(error_code &ec) noexcept;

   /**
    * @brief 设置阻抗控制类型
    * @param[in] type 0 - 关节阻抗 | 1 - 笛卡尔阻抗
    * @param[out] ec 错误码
    */
   void setControlType(int type, error_code &ec) noexcept;

   /**
    * @brief 设置力控模块使用的负载信息，fcStart()之后可调用。
    * @param[in] load 负载
    * @param[out] ec 错误码
    */
   void setLoad(const Load &load, error_code &ec) noexcept;

   /**
    * @brief 设置笛卡尔阻抗刚度。fcInit()之后调用生效
    * 各机型的最大刚度不同，请参考《xCore控制系统手册》 SetCartCtrlStiffVec指令的说明
    * @param[in] stiffness 依次为：X Y Z方向阻抗力刚度[N/m], X Y Z方向阻抗力矩刚度[Nm/rad]
    * @param[out] ec 错误码
    */
   void setCartesianStiffness(const std::array<double, 6> &stiffness, error_code &ec) noexcept;

   /**
    * @brief 设置笛卡尔零空间阻抗刚度。fcInit()之后调用生效
    * @param[in] stiffness 范围[0,4], 大于4会默认设置为4, 单位Nm/rad
    * @param[out] ec 错误码
    */
   void setCartesianNullspaceStiffness(double stiffness, error_code &ec) noexcept;

   /**
    * @brief 设置笛卡尔期望力/力矩。fcStart()之后可调用
    * @param[in] value 依次为: X Y Z方向笛卡尔期望力, 范围[-60,60], 单位N; X Y Z方向笛卡尔期望力矩, 范围[-10,10], 单位Nm
    * @param[out] ec 错误码
    */
   void setCartesianDesiredForce(const std::array<double, 6> &value, error_code &ec) noexcept;

   /**
    * @brief 设置绕单轴旋转的正弦搜索运动。
    * 设置阻抗控制类型为笛卡尔阻抗(即setControlType(1))之后, startOverlay()之前调用生效。
    * 各机型的搜索运动幅值上限和搜索运动频率上限不同，请参考《xCore控制系统手册》 SetSineOverlay指令的说明。
    * @param[in] line_dir 搜索运动参考轴: 0 - X | 1 - Y | 2 - Z
    * @param[in] amplify 搜索运动幅值, 单位Nm
    * @param[in] frequency 搜索运动频率, 单位Hz
    * @param[in] phase 搜索运动相位, 范围[0, PI], 单位弧度
    * @param[in] bias 搜索运动偏置, 范围[0, 10], 单位Nm
    * @param[out] ec 错误码
    */
   void setSineOverlay(int line_dir, double amplify, double frequency, double phase,
                       double bias, error_code &ec) noexcept;

   /**
    * @brief 设置平面内的莉萨如搜索运动
    * 设置阻抗控制类型为笛卡尔阻抗(即setControlType(1))之后, startOverlay()之前调用生效。
    * @param[in] plane 搜索运动参考平面: 0 - XY | 1 - XZ | 2 - YZ
    * @param[in] amplify_one 搜索运动一方向幅值, 范围[0, 20], 单位Nm
    * @param[in] frequency_one 搜索运动一方向频率, 范围[0, 5], 单位Hz
    * @param[in] amplify_two 搜索运动二方向幅值, 范围[0, 20]单位Nm
    * @param[in] frequency_two 搜索运动二方向频率, 范围[0, 5], 单位Hz
    * @param[in] phase_diff 搜索运动两个方向相位偏差, 范围[0, PI], 单位弧度
    * @param[out] ec 错误码
    */
   void setLissajousOverlay(int plane, double amplify_one, double frequency_one, double amplify_two,
                            double frequency_two, double phase_diff, error_code &ec) noexcept;

   /**
    * @brief 开启搜索运动。fcStart()之后调用生效
    * 搜索运动为前序设置的 setSineOverlay()或 setLissajousOverlay()的叠加
    * @param[out] ec 错误码
    */
   void startOverlay(error_code &ec) noexcept;

   /**
    * @brief 停止搜索运动
    * @param[out] ec 错误码
    */
   void stopOverlay(error_code &ec) noexcept;

   /**
    * @brief 暂停搜索运动。startOverlay()之后调用生效
    * @param[out] ec 错误码
    */
   void pauseOverlay(error_code &ec) noexcept;

   /**
    * @brief 重新开启暂停的搜索运动。pauseOverlay()之后调用生效。
    * @param[out] ec 错误码
    */
   void restartOverlay(error_code &ec) noexcept;

   /**
    * @brief 设置与接触力有关的终止条件
    * @param[in] range 各方向上的力限制 { X_min, X_max, Y_min, Y_max, Z_min, Z_max }, 单位N。
    * 设置下限时, 负值表示负方向上的最大值; 设置上限时, 负值表示负方向上的最小值。
    * @param[in] isInside true - 超出限制条件时停止等待; false - 符合限制条件时停止等待
    * @param[in] timeout 超时时间, 范围[1, 600], 单位秒
    * @param[out] ec 错误码
    */
   void setForceCondition(const std::array<double, 6> &range, bool isInside, double timeout, error_code &ec) noexcept;

   /**
    * @brief 设置与接触力矩有关的终止条件
    * @param[in] range 各方向上的力矩限制 { X_min, X_max, Y_min, Y_max, Z_min, Z_max }, 单位Nm。
    * 设置下限时, 负值表示负方向上的最大值; 设置上限时, 负值表示负方向上的最小值。
    * @param[in] isInside true - 超出限制条件时停止等待; false - 符合限制条件时停止等待
    * @param[in] timeout 超时时间, 范围[1, 600], 单位秒
    * @param[out] ec 错误码
    */
   void setTorqueCondition(const std::array<double, 6> &range, bool isInside, double timeout, error_code &ec) noexcept;

   /**
    * @brief 设置与接触位置有关的终止条件
    * @param[in] supervising_frame 长方体所在的参考坐标系, 相对于外部工件坐标系。
    * 外部工件坐标系是通过setToolset()设置的 (Toolset::ref)
    * @param[in] box 定义一个长方体 { X_start, X_end, Y_start, Y_end, Z_start, Z_end }, 单位米
    * @param[in] isInside true - 超出限制条件时停止等待; false - 符合限制条件时停止等待
    * @param[in] timeout 超时时间, 范围[1, 600], 单位秒
    * @param[out] ec 错误码
    */
   void setPoseBoxCondition(const Frame &supervising_frame, const std::array<double, 6> &box, bool isInside,
                           double timeout, error_code &ec) noexcept;

   /**
    * @brief 激活前序设置的终止条件并等待，直到满足这些条件或者超时
    * @param[out] ec 错误码
    */
   void waitCondition(error_code &ec) noexcept;

   /**
    * @brief 启动/关闭力控模块保护监控。
    * 设置监控参数后，不立即生效，调用fcMonitor(true)后开始生效，并且一直保持，直到调用fcMotion(false)后结束。
    * 结束后保护阈值恢复成默认值，即仍然会有保护效果，关闭监控后不再是用户设置的参数。
    * @param[in] enable true - 打开 | false - 关闭
    * @param[out] ec 错误码
    * @see setCartesianMaxVel() setJointMaxVel() setJointMaxMomentum() setJointMaxEnergy()
    */
   void fcMonitor(bool enable, error_code &ec) noexcept;

   /**
    * @brief 设置力控模式下, 机械臂末端相对基坐标系的最大速度
    * @param[in] velocity 依次为：X Y Z [m/s], A B C [rad/s], 范围 >=0
    * @param[out] ec 错误码
    */
   void setCartesianMaxVel(const std::array<double, 6> &velocity, error_code &ec) noexcept;

   /// @cond DO_NOT_DOCUMENT
   explicit BaseForceControl(std::shared_ptr<XService> rpc);
   virtual ~BaseForceControl();

  XCORESDK_DECLARE_IMPL
   /// @endcond
 };

 /**
  * @brief 力控指令类
  * @tparam DoF 轴数
  */
 template<unsigned short DoF>
 class XCORE_API ForceControl_T : public BaseForceControl {
  public:

   using BaseForceControl::BaseForceControl;

   /**
    * @brief 获取当前力矩信息
    * @param[in] ref_type 力矩相对的参考系：
    *     1) FrameType::world - 末端相对世界坐标系的力矩信息
    *     2) FrameType::flange - 末端相对于法兰盘的力矩信息
    *     3) FrameType::tool - 末端相对于TCP点的力矩信息
    * @param[out] joint_torque_measured 轴空间测量力信息,力传感器测量到的各轴所受力矩, 单位Nm
    * @param[out] external_torque_measured 轴空间外部力信息，控制器根据机器人模型和测量力计算出的各轴所受力矩信息, 单位Nm
    * @param[out] cart_torque 笛卡尔空间各个方向[X, Y, Z]受到的力矩, 单位Nm
    * @param[out] cart_force 笛卡尔空间各个方向[X, Y, Z]受到的力, 单位N
    * @param[out] ec 错误码
    */
   void getEndTorque(FrameType ref_type, std::array<double, DoF> &joint_torque_measured, std::array<double, DoF> &external_torque_measured,
                     std::array<double, 3> &cart_torque, std::array<double, 3> &cart_force, error_code &ec) noexcept;

   /**
    * @brief 设置关节阻抗刚度。fcInit()之后调用生效
    * 各机型的最大刚度不同，请参考《xCore控制系统手册》 SetJntCtrlStiffVec指令的说明
    * @param[in] stiffness 各轴刚度
    * @param[out] ec 错误码
    */
   void setJointStiffness(const std::array<double, DoF> &stiffness, error_code &ec) noexcept;

   /**
    * @brief 设置关节期望力矩。fcStart()之后可调用
    * @param[in] torque 力矩值, 范围[-30,30], 单位Nm
    * @param[out] ec 错误码
    */
   void setJointDesiredTorque(const std::array<double, DoF> &torque, error_code &ec) noexcept;

   /**
    * @brief 设置力控模式下的轴最大速度
    * @param[in] velocity 轴速度 [rad/s]，范围 >=0
    * @param[out] ec 错误码
    */
   void setJointMaxVel(const std::array<double, DoF> &velocity, error_code &ec) noexcept;

   /**
    * @brief 设置力控模式下轴最大动量。
    * 计算方式：F*t，可以理解为冲量，F为力矩传感器读数，t为控制周期，如果超过30个周期都超过动量阈值则触发保护
    * @param[in] momentum 动量 [N·s]，范围 >=0
    * @param[out] ec 错误码
    */
   void setJointMaxMomentum(const std::array<double, DoF> &momentum, error_code &ec) noexcept;

   /**
    * @brief 设置力控模式下轴最大动能。
    * 计算方式：F*v，可以理解为功率，F为力矩传感器读数，v为关节速度, 如果超过30个周期都超过动能阈值则触发保护
    * @param[in] energy 动能 [N·rad/s]，范围 >=0
    * @param[out] ec 错误码
    */
   void setJointMaxEnergy(const std::array<double, DoF> &energy, error_code &ec) noexcept;

 };

} // namespace rokae

#endif //XCORESDK_INCLUDE_ROKAE_FORCE_CONTROL_H_
