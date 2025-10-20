/**
 * @file motion_control_rt.h
 * @brief 实时模式运动控制
 * @copyright Copyright (C) 2024 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef ROKAEAPI_ROKAE_MOTION_CONTROL_RT_H_
#define ROKAEAPI_ROKAE_MOTION_CONTROL_RT_H_

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include <memory>
#include "base.h"
#include "data_types.h"

namespace rokae {

 /// @cond DO_NOT_DOCUMENT
 // forward declaration
 class XService;
 class DataReceiver;
 template <MotionControlMode ControlMode>
 class MotionControl;
 template <unsigned short DoF>
 class xMateModel;
 /// @endcond

/**
 * @class BaseMotionControl
 * @brief 运动控制通用类
 */
 class XCORE_API BaseMotionControl : public Base<BaseMotionControl> {
  public:
   /// @cond DO_NOT_DOCUMENT
   explicit BaseMotionControl(std::shared_ptr<XService> comm) noexcept;
   virtual ~BaseMotionControl() noexcept;
   BaseMotionControl(const BaseMotionControl&) = delete;
   BaseMotionControl &operator=(const BaseMotionControl &) = delete;

  XCORESDK_DECLARE_IMPL
   /// @endcond
 };

 /**
  * @class MotionControl<MotionControlMode::RtCommand>
  * @brief 实时模式运动控制通用类
  */
 template <>
 class XCORE_API MotionControl<MotionControlMode::RtCommand> : public BaseMotionControl {
  public:
   /**
    * @brief constructor
    * @throw NetworkException 网络连接错误
    */
   MotionControl(std::shared_ptr<XService> sdkRpc, XService *_rtRpc, DataReceiver *recv, const std::string &localIp);
   virtual ~MotionControl() noexcept;

   // *********************************************************************
   // *******************           网络连接           **********************

   /**
    * @brief 断开与实时控制服务器的连接, 关闭数据接收和指令发送端口。不会断开和机器人的连接。
    * 若机器人在运动，断开后会立即停止运动
    */
   void disconnectNetwork() noexcept;

   /**
    * @brief 重新连接到实时控制服务器
    * @param[out] ec 错误码
    */
   void reconnectNetwork(error_code &ec) noexcept;

   // *********************************************************************
   // *******************           控制方式           **********************

   /**
    * @brief 使用周期调度，设置回调函数。
    * @note 1) 回调函数应按照1毫秒为周期规划运动命令，规划结果为函数的返回值。SDK对返回值进行滤波处理后发送给控制器。
    *       2) JointPosition的关节角度数组长度，和Torque的关节力矩值数组长度应和机器人轴数相同。若不同不会报错，但有可能造成不合理的命令
    *       3) 一次运动循环结束时，可以通过返回的Command.setFinish()的方式来标识，SDK内部会负责停止运动以及停止调用回调函数
    * @tparam Command JointPosition | CartesianPosition | Torque
    * @param[in] callback 回调函数。根据控制模式(RtControllerMode)不同，函数返回值有3种: 关节角度/笛卡尔位姿/力矩。
    * 其中笛卡尔位姿使用旋转矩阵表示旋转量，pos为末端相对于基坐标系的位姿。
    * @param[in] priority 任务优先级, 0为不指定。此参数仅当使用实时操作系统时生效，若无法设置会打印控制台错误信息。
    * @param[in] useStateDataInLoop 是否需要在周期内读取状态数据。当设置为true时：
    *       1) xCore-SDK会在回调函数之前更新实时状态数据(updateStateData()),在回调函数内直接getStateData()即可;
    *       2) 状态数据的发送周期应和控制周期一致, 为1ms: startReceiveRobotState(interval = milliseconds(1));
    */
   template<class Command>
   void setControlLoop(const std::function<Command(void)>& callback, int priority = 0, bool useStateDataInLoop = false) noexcept;

   /**
    * @brief 开始执行回调函数。
    * @param[in] blocking 是否阻塞调用此函数的线程。若为非阻塞线程，需调用stopLoop()停止调度任务，否则无法开始下一次循环周期。
    * @throw RealtimeControlException 命令发送网络异常; 或命令类型与控制模式不匹配; 或控制器执行已发送命令时发生错误
    */
   void startLoop(bool blocking = true);

   /**
    * @brief 停止执行周期性调度任务
    * @throw RealtimeControlException 执行过程中发生异常
    */
   void stopLoop();

   /**
    * @brief 机器人停止运动，停止接收客户端发送的运动指令。
    * @note 另外，JointPosition/CartesianPosition/Torque指令可通过setFinished()标识一次运动循环结束，标识后会机器人停止运动，
    * 呈现的效果和调用stopMove()一样。
    * 此函数仅用于实时控制，不可以用于停止非实时运动指令。
    * @throw RealtimeMotionException 停止运动失败
    */
   void stopMove();

   // *********************************************************************
   // *****************        获取机器人实时状态数据        ******************

   /**
    * @brief 让机器人开始发送实时状态数据。阻塞等待收到第一帧消息，超时时间为3秒
    * @param[in] fields 接收的机器人状态数据, 最大总长度为1024个字节
    * @throw RealtimeControlException 设置了不支持的状态数据；或机器人无法开始发送数据；或总长度超过1024
    * @throw RealtimeStateException 已经开始发送数据；或超时后仍未收到第一帧数据
    */
   [[deprecated("Use Robot_T::startReceiveRobotState(interval, fields) instead")]]
   void startReceiveRobotState(const std::vector<std::string>& fields);

   /**
    * @brief 停止接收实时状态数据，同时控制器停止发送。可用于重新设置要接收的状态数据。
    * 调用此函数后，实时运动的错误信息也会停止接收，建议在运动停止时调用。
    */
   [[deprecated("Use BaseRobot::stopReceiveRobotState() instead")]]
   void stopReceiveRobotState() noexcept;

   /**
    * @brief 更新机器人状态数据到当前最新
    * @note 通过setControlLoop()设置的回调在每次执行时会更新一次状态数据，因此不需要在回调函数中调用此接口
    * @throw RealtimeStateException 控制器处理运动命令时出错，错误位被置位
    * @throw RealtimeControlException 无法收到数据；或收到的数据有错误导致无法解析
    */
   [[deprecated("Use BaseRobot::updateRobotState() instead")]]
   void updateRobotState();

   /**
    * @brief 读取机器人状态数据
    * @note 注意传入的data类型要和数据类型一致。
    * @tparam R 数据类型
    * @param[in] fieldName 数据名
    * @param[out] data 数值
    * @return 若无该数据名；或未通过startReceiveRobotState()设置为要接收的数据；或该数据类型和R不符，返回-1。
    * 成功读取返回0。
    * @throw RealtimeStateException 网络错误
    */
   template<typename R>
   [[deprecated("Use BaseRobot::getStateData(fieldName, data) instead")]]
   int getStateData(const std::string &fieldName, R &data);

   // *********************************************************************
   // ********************          其他操作            *********************
   /**
    * @brief 当错误发生后，自动恢复机器人。
    * @param[out] ec 错误码
    */
   void automaticErrorRecovery(error_code &ec) noexcept;

   /// @cond DO_NOT_DOCUMENT
  XCORESDK_DECLARE_IMPLD
  /// @endcond
 };

 /**
  * @class RtMotionControl
  * @brief 实时模式运动控制
  * @tparam DoF 轴数
  */
 template <WorkType Wt, unsigned short DoF>
 class XCORE_API RtMotionControl : public MotionControl<MotionControlMode::RtCommand> {
  public:
   /**
    * @brief constructor
    * @throw NetworkException 网络连接错误
    */
   RtMotionControl(std::shared_ptr<XService> sdkRpc, XService *rtRpc, DataReceiver *recv, const std::string &localIp);

   // **********************************************************************
   // ***************           控制模式 & 发送运动指令          ***************

   /**
    * @brief 指定控制模式，机器人准备开始运动，在每段回调执行前需要先调用此接口。
    * 调用此接口机器人不会立即开始运动, 而是有运动命令发送后才会开始。
    * @param[in] rtMode 控制模式
    * @note 1) 在startMove之前应将参数依次设置好，例如滤波阻抗参数等等，设置完成后再调用startMove()。
    * 在调用startMove后执行其他指令可能会失败，例如下电等操作。正确停止方法是调用stopMove。
    * @throw RealtimeStateException 已经开始运动运动后重复调用
    * @throw RealtimeParameterException 指定了不支持的控制模式
    * @throw RealtimeControlException 控制器无法切换到该控制模式，多出现于切换到力控模式时
    */
   void startMove(RtControllerMode rtMode);

   // *********************************************************************
   // *******************           参数设置           **********************

   /**
    * @brief 设置限幅滤波参数.
    * @param[in] limit_rate true - 限幅开启
    * @param[in] cutoff_frequency 截止频率。范围是0 ~ 1000Hz，建议10~100Hz.
    * @return true - 设定成功
    */
   bool setFilterLimit(bool limit_rate, double cutoff_frequency) noexcept;

   /**
    * @brief 设置笛卡尔空间运动区域，超过设置区域运动会停止。
    * 非力控虚拟墙。若机器人末端或TCP末端超过安全区域，电机同样会做下电处理。
    * @param[in] lengths 安全区域长方体长宽高，对应XYZ, 单位: 米
    * @param[in] frame 安全区域长方体中心相对于基坐标系位姿
    * @param[out] ec 错误码
    */
   void setCartesianLimit(const std::array<double, 3> &lengths, const std::array<double, 16> &frame, error_code &ec) noexcept;

   /**
    * @brief 设置末端执行器相对于机器人法兰的位姿，设置TCP后控制器会保存配置，机器人重启后恢复默认设置。
    * @param[in] frame 末端执行器坐标系相对于法兰坐标系的齐次矩阵，单位: rad, m
    * @param[out] ec 错误码
    */
   void setEndEffectorFrame(const std::array<double, 16> &frame, error_code &ec) noexcept;

   /**
    * @brief 设置工具和负载的质量、质心和惯性矩阵。设置负载后控制器会保存负载配置，机器人重启后恢复默认设置。
    * @param[in] load 负载信息
    * @param[out] ec 错误码
    */
   void setLoad(const Load &load, error_code &ec) noexcept;

   // *************************************************************************
   // *****************            Move指令 (上位机规划)         *****************

   /**
    * @brief MoveJ指令，上位机规划路径，在到达target之前处于处于阻塞状态。如果运动中发生错误将停止阻塞状态并返回。
    * @param[in] speed 速度比例系数
    * @param[in] start 起始关节角度，需要是机器人当前关节角度，否则可能造成下电。
    * @param[in] target 机器人目标关节角度
    * @throw RealtimeMotionException 机器人运动过程中发生错误
    */
   void MoveJ(double speed, const std::array<double, DoF>& start, const std::array<double, DoF>& target);

   /**
    * @brief MoveL指令，上位机规划路径，在到达target之前处于处于阻塞状态。如果运动中发生错误将停止阻塞状态并返回。
    * @param[in] speed 速度比例系数, 范围 0 - 1
    * @param[in] start 起始位姿, 需要是机器人当前位姿，否则可能造成下电。如果设置了TCP，那么应该是工具相对于基坐标系的位姿。
    * @param[in] target 机器人目标位姿。同理如果设置了TCP，应是TCP相对于基坐标系的位姿
    * @throw RealtimeParameterException 起始或目标位姿参数错误
    * @throw RealtimeMotionException 机器人运动过程中发生错误
    */
   void MoveL(double speed, CartesianPosition& start, CartesianPosition& target);

   /**
    * @brief MoveC指令，在到达target之前处于阻塞状态。如果运动中发生错误将停止阻塞状态并返回。
    * @param[in] speed 速度比例系数
    * @param[in] start 机器人起始位姿, 需要是机器人当前位姿。如果设置了TCP，那么应该是工具相对于基坐标系的位姿。
    * @param[in] aux 机器人辅助点位姿。同理如果设置了TCP，应是TCP相对于基坐标系的位姿
    * @param[in] target 机器人目标位姿。同理如果设置了TCP，应是TCP相对于基坐标系的位姿
    * @throw RealtimeParameterException 点位错误, 无法计算出圆弧路径
    * @throw RealtimeMotionException 机器人运动过程中发生错误
    */
   void MoveC(double speed, CartesianPosition& start, CartesianPosition& aux, CartesianPosition& target);

 };

 /**
  * @class RtMotionControlCobot
  * @brief 协作机型实时运动控制类
  * @tparam DoF 轴数
  */
 template <unsigned short DoF>
 class XCORE_API RtMotionControlCobot: public RtMotionControl<WorkType::collaborative, DoF> {
  public:
   using RtMotionControl<WorkType::collaborative, DoF>::RtMotionControl;

   // *********************************************************************
   // *******************           参数设置           **********************

   /**
    * @brief 设置轴空间阻抗控制系数，轴空间阻抗运动时生效
    * @param[in] factor 轴空间阻抗系数，单位: Nm/rad
    * xMateErPro机型最大刚度为 { 3000, 3000, 3000, 3000, 300, 300, 300 }
    * 六轴机型最大刚度为 { 3000, 3000, 3000, 300, 300, 300 }
    * 五轴机型最大刚度为 { 3000, 3000, 3000, 300, 300 }
    * 实际有效的最大值和传感器等硬件状态有关系，如发生抖动等现象，请尝试减小阻抗系数。
    * @param[out] ec 错误码
    */
   void setJointImpedance(const std::array<double, DoF> &factor, error_code &ec) noexcept;

   /**
    * @brief 设置笛卡尔空间阻抗控制系数, 笛卡尔阻抗运动时生效
    * @param[in] factor 阻抗系数[ X, Y, Z, Rx, Ry, Rz], 最大值为 { 3000, 3000, 3000, 300, 300, 300 }, 单位: N/m, Nm/rad
    * 实际有效的最大值和传感器等硬件状态有关系，如发生抖动等现象，请尝试减小阻抗系数。
    * @param[out] ec 错误码
    */
   void setCartesianImpedance(const std::array<double, 6> &factor, error_code &ec) noexcept;

   /**
    * @brief 设置机器人控制器的滤波截止频率，用来平滑指令。允许的范围: 1 ~ 1000Hz, 建议设置为10 ~ 100Hz。
    * @param[in] jointFrequency 关节位置的滤波截止频率，单位: Hz
    * @param[in] cartesianFrequency 笛卡尔空间位置的滤波截止频率，单位: Hz
    * @param[in] torqueFrequency 关节力矩的滤波截止频率，单位: Hz
    * @param[out] ec 错误码
    */
   void setFilterFrequency(double jointFrequency, double cartesianFrequency, double torqueFrequency, error_code &ec) noexcept;

   /**
    * @brief 设置末端期望力, 在笛卡尔空间阻抗运动时生效
    * @param[in] torque 笛卡尔空间末端期望力, 允许的范围为 { ±60, ±60, ±60, ±30, ±30, ±30 }, 单位: N, N·m
    * @param[out] ec 错误码
    */
   void setCartesianImpedanceDesiredTorque(const std::array<double, 6> &torque, error_code &ec) noexcept;

   /**
    * @brief 设置滤波参数
    * @param[in] frequency 允许的范围 1 ~ 1000Hz
    * @param[out] ec 错误码
    */
   void setTorqueFilterCutOffFrequency(double frequency, error_code &ec) noexcept;

   /**
    * @brief 设置机器人力控坐标系
    * @param[in] frame 力控坐标系相对于法兰坐标系的变换矩阵
    * @param[in] type 类别, 指定哪个坐标系为力控任务坐标系, 支持:
    *     1) 世界坐标系 FrameType::world;
    *     2) 工具坐标系 FrameType::tool;
    *     3) 路径坐标系 FrameType::path (力控任务坐标系需要跟踪轨迹变化的过程)
    * @param[out] ec 错误码
    */
   void setFcCoor(const std::array<double, 16> &frame, FrameType type, error_code &ec) noexcept;

   /**
    * @brief 设置碰撞检测阈值。
    * 碰撞检测只在位置控制时生效，力控时不生效。若检测到碰撞，控制器会下发下电指令，电机抱闸吸合下使能。
    * @param[in] torqueThresholds 关节碰撞检测阈值, 单位N。
    * xMateErPro机型最大值为 { 75, 75, 60, 45, 30, 30, 20 }，
    * 其他机型最大值为{ 75, 75, 45, 30, 30, 20 }
    * 5轴机型最大值为{ 75, 75, 45, 30, 20 }
    * @param[out] ec 错误码
    */
   void setCollisionBehaviour(const std::array<double, DoF> &torqueThresholds, error_code &ec) noexcept;

 };

 /**
  * @class RtMotionControlIndustrial
  * @brief 工业机型实时模式运动控制类
  * @tparam DoF 轴数
  */
 template <unsigned short DoF>
 class XCORE_API RtMotionControlIndustrial: public RtMotionControl<WorkType::industrial, DoF> {
  public:
   using RtMotionControl<WorkType::industrial, DoF>::RtMotionControl;
 };

}
#endif //ROKAEAPI_ROKAE_MOTION_CONTROL_RT_H_
