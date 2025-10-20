/**
 * @file data_types.h
 * @brief 定义数据结构和枚举类
 * @copyright Copyright (C) 2024 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef ROKAEAPI_INCLUDE_ROKAE_DATA_TYPES_H_
#define ROKAEAPI_INCLUDE_ROKAE_DATA_TYPES_H_

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include <array>
#include <vector>
#include <string>
#include <any>
#include <unordered_map>
#include "base.h"

namespace rokae {

 /// @cond DO_NOT_DOCUMENT
 const int USE_DEFAULT = -1;
 const int Unknown = -1;
 /// @endcond

// *********************         Enum class          **********************
// *********************           枚举类             **********************

 /**
  * @enum OperationState
  * @brief 机器人工作状态
  */
  enum class OperationState {
    idle             = 0, ///< 机器人静止
    jog              = 1, ///< jog状态(未运动)
    rtControlling    = 2, ///< 实时模式控制中
    drag             = 3, ///< 拖动已开启
    rlProgram        = 4, ///< RL工程运行中
    demo             = 5, ///< Demo演示中
    dynamicIdentify  = 6, ///< 动力学辨识中
    frictionIdentify = 7, ///< 摩擦力辨识中
    loadIdentify     = 8, ///< 负载辨识中
    moving           = 9, ///< 机器人运动中
    jogging          = 10, ///< Jog运动中
    unknown          = Unknown ///< 未知
  };

 /**
  * @enum WorkType
  * @brief 机型类别
  */
 enum class WorkType {
   industrial,   ///< 工业机器人
   collaborative ///< 协作机器人
 };

 /**
  * @enum OperateMode
  * @brief 机器人操作模式
  */
 enum class OperateMode {
   manual    = 0,      ///< 手动
   automatic = 1,      ///< 自动
   unknown   = Unknown ///< 未知(发生异常)
 };

 /**
  * @enum PowerState
  * @brief 机器人上下电及急停状态
  */
 enum class PowerState {
   on      = 0, ///< 上电
   off     = 1, ///< 下电
   estop   = 2, ///< 急停被按下
   gstop   = 3, ///< 安全门打开
   unknown = Unknown ///< 未知(发生异常)
 };

 /**
  * @brief 位姿坐标系类型
  */
 enum class CoordinateType {
   flangeInBase, ///< 法兰相对于基坐标系
   endInRef      ///< 末端相对于外部坐标系
 };

 /**
  * @enum MotionControlMode
  * @brief SDK运动控制模式
  */
 enum class MotionControlMode : unsigned {
   Idle,       ///< 空闲
   NrtCommand, ///< 非实时模式执行运动指令
   NrtRLTask,  ///< 非实时模式运行RL工程
   RtCommand,  ///< 实时模式控制
 };

 /**
  * @enum RtControllerMode
  * @brief 控制器实时控制模式
  */
 enum class RtControllerMode : unsigned {
   jointPosition,      ///< 实时轴空间位置控制
   cartesianPosition,  ///< 实时笛卡尔空间位置控制
   jointImpedance,     ///< 实时轴空间阻抗控制
   cartesianImpedance, ///< 实时笛卡尔空间阻抗控制
   torque              ///< 实时力矩控制
 };

 namespace RtSupportedFields {
  /// 说明：数据名后为数据类型
  /// ArrayXD = std::array<double, DoF> , DoF为轴数
  /// Array6D = std::array<double, 6>, 以此类型
  constexpr const char *jointPos_m = "q_m";   ///< 关节角度 [rad] - ArrayXD
  constexpr const char *jointPos_c = "q_c";   ///< 指令关节角度 [rad] - ArrayXD。仅在打开实时模式控制之后数据有效。
  constexpr const char *jointVel_m = "dq_m";  ///< 关节速度 [rad/s]- ArrayXD
  constexpr const char *jointVel_c = "dq_c";  ///< 指令关节速度 [rad/s] - ArrayXD。仅在打开实时模式控制之后数据有效。
  constexpr const char *jointAcc_m = "ddq_m"; ///< 关节加速度 [rad/s^2] - ArrayXD。仅在打开实时模式控制之后数据有效。
  constexpr const char *jointAcc_c = "ddq_c"; ///< 指令关节加速度 [rad/s^2] - ArrayXD。仅在打开实时模式控制之后数据有效。
  constexpr const char *tcpPose_m  = "pos_m"; ///< 末端位姿, 相对于基坐标系, 行优先齐次变换矩阵 - Array16D
  constexpr const char *tcpPoseAbc_m = "pos_abc_m"; ///< 末端位姿, 相对于基坐标系 [X,Y,Z,Rx,Ry,Rz] - Array6D
  constexpr const char *tcpPose_c  = "pos_c"; ///< 发送的末端位姿指令, 相对于基坐标系, 行优先齐次变换矩阵 - Array16D。仅在打开实时模式控制之后数据有效。
  constexpr const char *tcpVel_m   = "pos_vel_m"; ///< 机器人末端速度 - Array6D。仅在打开实时模式控制之后数据有效。
  constexpr const char *tcpVel_c   = "pos_vel_c"; ///< 指令机器人末端速度 - Array6D。仅在打开实时模式控制之后数据有效。
  constexpr const char *tcpAcc_m   = "pos_acc_m"; ///< 机器人末端加速度 - Array6D。仅在打开实时模式控制之后数据有效。
  constexpr const char *tcpAcc_c   = "pos_acc_c"; ///< 指令机器人末端加速度 - Array6D。仅在打开实时模式控制之后数据有效。
  constexpr const char *exJointPos_m = "ex_q_m"; ///< 外部轴数值 [rad] 导轨[m] - Array6D 实际有效数据个数为外部轴数
  constexpr const char *exJointVel_m = "ex_dq_m"; ///< 外部轴速度 [rad/s] 导轨[m/s] - Array6D 实际有效数据个数为外部轴数
  constexpr const char *exMotor_m = "ex_motor_m"; ///< 外部轴电机位置 - Array6D 实际有效数据个数为外部轴数
  constexpr const char *elbow_m    = "psi_m";     ///< 臂角 [rad] - double
  constexpr const char *elbow_c    = "psi_c";     ///< 指令臂角 [rad] - double。仅在打开实时模式控制之后数据有效。
  constexpr const char *elbowVel_c = "psi_vel_c"; ///< 指令臂角速度 [rad/s] - double。仅在打开实时模式控制之后数据有效。
  constexpr const char *elbowAcc_c = "psi_acc_c"; ///< 指令臂角加速度 [rad/s] - double。仅在打开实时模式控制之后数据有效。
  constexpr const char *tau_m      = "tau_m";     ///< 关节力矩 [Nm] - ArrayXD
  constexpr const char *tau_c      = "tau_c";     ///< 指令关节力矩 [Nm] - ArrayXD。仅在打开实时模式控制之后数据有效。
  constexpr const char *tauFiltered_m    = "tau_filtered_m"; ///< 滤波后关节力矩 [Nm] - ArrayXD。仅在打开实时模式控制之后数据有效。
  constexpr const char *tauVel_c         = "tau_vel_c";      ///< 指令力矩微分 [Nm/s] - ArrayXD。仅在打开实时模式控制之后数据有效。
  constexpr const char *tauExt_inBase    = "tau_ext_base";   ///< 基坐标系中外部力矩 [Nm] - Array6D。仅在打开实时模式控制之后数据有效。
  constexpr const char *tauExt_inStiff   = "tau_ext_stiff";  ///< 力控坐标系中外部力矩 [Nm] - Array6D。仅在打开实时模式控制之后数据有效。
  constexpr const char *theta_m          = "theta_m";        ///< 电机位置 - ArrayXD
  constexpr const char *thetaVel_m       = "theta_vel_m";        ///< 电机位置微分 - ArrayXD
  constexpr const char *motorTau         = "motor_tau";          ///< 电机转矩 - ArrayXD
  constexpr const char *motorTauFiltered = "motor_tau_filtered"; ///< 滤波后电机转矩 - ArrayXD。仅在打开实时模式控制之后数据有效。
  constexpr const char *keypads  = "io_keypad";    ///< 末端按键状态 - ArrayXD
 }

 /**
  * @enum StopLevel
  * @brief 机器人停止运动等级
  */
 enum class StopLevel {
   stop0, ///< 快速停止机器人运动后断电
   stop1, ///< 规划停止机器人运动后断电, 停在原始路径上
   stop2,  ///< 规划停止机器人运动后不断电, 停在原始路径上
   suppleStop ///< 柔顺停止，仅适用于协作机型
 };

 /**
  * @struct DragParameter
  * @brief 机器人拖动模式参数, 包括拖动类型和空间
  */
 struct DragParameter {
   /**
    * @brief 拖动空间
    */
   enum Space {
     jointSpace     = 0, ///< 轴空间
     cartesianSpace = 1  ///< 笛卡尔空间
   };
   /**
    * @brief 拖动类型
    */
   enum Type {
     translationOnly = 0, ///< 仅平移
     rotationOnly    = 1, ///< 仅旋转
     freely          = 2  ///< 自由拖拽
   };
 };

 /**
  * @enum FrameType
  * @brief 坐标系类型
  */
 enum class FrameType {
   world  = 0, ///< 世界坐标系
   base   = 1, ///< 基坐标系
   flange = 2, ///< 法兰坐标系
   tool   = 3, ///< 工具坐标系
   wobj   = 4, ///< 工件坐标系
   path   = 5, ///< 路径坐标系
   rail   = 6  ///< 导轨基坐标系
 };

 /**
  * @struct JogOpt
  * @brief Jog选项: 坐标系
  */
 struct JogOpt {
   /**
    * @brief Jog坐标系
    */
   enum Space {
     world = 0, ///< 世界坐标系
     flange, ///< 法兰坐标系
     baseFrame, ///< 基坐标系
     toolFrame, ///< 工具坐标系
     wobjFrame, ///< 工件坐标系
     jointSpace, ///< 轴空间
     singularityAvoidMode, ///< 奇异规避模式，适用于工业六轴, xMateCR和xMateSR机型，规避方法是锁定4轴
     baseParallelMode ///< 平行基座模式，仅适用于xMateCR和xMateSR机型
   };
 };

 /**
  * @struct xPanelOpt
  * @brief xPanel配置: 对外供电模式
  */
 struct xPanelOpt {
   /**
    * @brief 供电模式
    */
   enum Vout {
     off,       ///< 不输出
     reserve,   ///< 保留
     supply12v, ///< 输出12V
     supply24v, ///< 输出24V
   };
 };

 /**
  * @brief 奇异规避方式
  */
 enum class AvoidSingularityMethod {
   lockAxis4, ///< 四轴锁定
   wrist,     ///< 牺牲姿态
   jointWay   ///< 轴空间短轨迹插补
 };

 /**
  * @brief 事件信息 - map类型
  */
 typedef std::unordered_map<std::string, std::any> EventInfo;
 /**
  * @brief 事件回调函数类型
  */
 typedef std::function<void(const EventInfo &)> EventCallback;

 /**
  * @brief 事件类型
  */
 enum class Event {
   moveExecution, ///< 非实时运动指令执行信息
   safety,        ///< 安全 (是否碰撞)
   rlExecution    ///< RL执行状态
 };

 /**
  * @brief 事件信息字段
  */
 namespace EventInfoKey {
  /**
   * 非实时运动指令执行信息
   */
  namespace MoveExecution {
   constexpr const char *ID = "cmdID";     ///< 路径ID, 对应调用moveAppend()时第二个参数; 类型string
   constexpr const char *ReachTarget = "reachTarget"; ///< 轨迹是否到达目标点; 类型bool
   constexpr const char *WaypointIndex = "wayPointIndex"; ///< 当前正在执行的轨迹目标点下标, 从0开始; 类型int
   constexpr const char *Error = "error"; ///< 错误码, 运动指令执行前或执行中的错误; 类型error_code
   constexpr const char *Remark = "remark"; ///< 其它执行信息，目前包括目标点距离过近的告警信息; 类型string
   constexpr const char *CustomInfo = "customInfo"; ///< 用户自定义信息, 对应NrtCommand::customInfo; 类型string
  }
  /**
   * @brief 安全相关
   */
  namespace Safety {
   constexpr const char *Collided = "collided"; ///< 是否碰撞; 类型bool, true-发生碰撞 | false-未发生或已恢复
  }

  /**
   * @brief RL程序执行状态
   */
  namespace RlExecution {
   constexpr const char *TaskName = "taskName"; ///< 执行的任务名称; 类型string
   constexpr const char *LookaheadLine = "lookaheadLine"; ///< 前瞻行号; 类型int
   constexpr const char *LookaheadFile = "lookaheadFile"; ///< 前瞻到的文件名; 类型string
   constexpr const char *ExecuteLine = "executeLine"; ///< 执行行号; 类型int
   constexpr const char *ExecuteFile = "executeFile"; ///< 正在执行的文件名; 类型string
  }
 }

// *******************          Data types            ********************
// *******************           数据结构              ********************
#if defined(XCORESDK_SUPPRESS_DLL_WARNING)
#pragma warning(push)
#pragma warning(disable : 4251)
#endif

 /**
  * @class Frame
  * @brief 坐标系
  */
 class XCORE_API Frame {
  public:
   /**
    * @brief default constructor
    */
   Frame() = default;

   /**
    * @brief 初始化trans & rpy
    * @param trans 平移量
    * @param rpy 欧拉角XYZ
    */
   Frame(const std::array<double, 3> &trans, const std::array<double, 3> &rpy);

   /**
    * @brief 初始化trans & rpy
    * @param frame [X, Y, Z, Rx, Ry, Rz]
    */
   Frame(const std::array<double, 6> &frame);

   /**
    * @brief 初始化pos
    * @param matrix 4*4变换矩阵
    */
   Frame(const std::array<double,16> &matrix);

   /**
    * @brief 初始化
    * @param values 长度为6时初始化trans & rot = [X, Y, Z, Rx, Ry, Rz];
    *               长度为16时初始化pos
    * @throw ArgumentException 初始化列表长度错误
    */
   Frame(std::initializer_list<double> values);

   std::array<double, 3> trans {}; ///< 平移量 [X, Y, Z], 单位:米
   std::array<double, 3> rpy {};   ///< 欧拉角 [Rx, Ry, Rz], 单位:弧度
   std::array<double, 16> pos {};  ///< 行优先齐次变换矩阵。只用于实时模式笛卡尔位置/阻抗控制。
 };

 /**
  * @class Finishable
  * @brief 一次运动循环是否结束
  */
 class XCORE_API Finishable {
  public:
   /**
    * @brief 是否已设置运动循环结束
    */
   uint8_t isFinished() const;

   /**
    * @brief 标识运动循环已结束
    */
   void setFinished();

  protected:
   uint8_t finished { 0 }; ///< 用于判断是否结束一个运动循环
 };

 /**
  * @class CartesianPosition
  * @brief 笛卡尔点位
  */
 class XCORE_API CartesianPosition : public Frame, public Finishable {
  public:
   using Frame::Frame;
   /**
    * @brief 偏移
    */
   struct Offset {
     /**
      * @brief 偏移类型
      */
     enum Type {
       none,   ///< 无偏移
       offs,   ///< 相对工件坐标系偏移
       relTool ///< 相对工具坐标系偏移
     };

     /**
      * @brief default constructor
      */
     Offset() = default;

     /**
      * @brief constructor
      */
     Offset(Type type, const Frame &frame);

     Type type { none }; ///< 偏移类型
     Frame frame { };    ///< 相对于指定工具/工件坐标系的偏移
   };

   double elbow { 0 };      ///< 臂角, 适用于7轴机器人, 单位：弧度
   bool hasElbow { false }; ///< 是否有臂角
   std::vector<int> confData; ///< 轴配置数据，长度为8: [cf1, cf2, cf3, cf4, cf5, cf6, cf7, cfx]
   std::vector<double> external; ///< 外部关节数值 单位:弧度|米。导轨单位米
 };

 /**
  * @class JointPosition
  * @brief 关节点位
  */
 class XCORE_API JointPosition : public Finishable {
  public:
   /**
    * @brief default constructor
    */
   JointPosition() = default;
   /**
    * @param joints 长度应与机器人轴数一致. 外部关节可缺省
    */
   JointPosition(std::initializer_list<double> joints);

   /**
    * @brief constructor
    * @param joints 轴角度
    */
   JointPosition(std::vector<double> joints);

   /**
    * @brief 初始化joints
    * @param n 长度, 应和机型轴数匹配
    * @param v 初始值
    */
   JointPosition(size_t n, double v = 0);

   std::vector<double> joints; ///< 关节角度值, 单位:弧度
   std::vector<double> external; ///< 外部关节数值, 单位:弧度|米。导轨单位米
 };

 /**
  * @class Torque
  * @brief 关节扭矩，不包含重力和摩擦力
  */
 class XCORE_API Torque : public Finishable {
  public:
   Torque() = default;

   /**
    * @brief constructor
    * @param tau 力矩指令值
    */
   Torque(std::vector<double> tau);

   /**
    * @brief constructor
    * @param tau 力矩指令值
    */
   Torque(std::initializer_list<double> tau);

   /**
    * @brief 初始化tau
    * @param n 长度, 应和机型轴数匹配
    * @param v 初始值
    */
   Torque(size_t n, double v = 0);

   std::vector<double> tau; ///< 期望关节扭矩，单位: Nm
 };

 /**
   * @class Load
   * @brief 负载信息
   */
 class XCORE_API Load {
  public:
   Load() = default;
   /**
    * @param m 质量
    * @param cog 质心
    * @param inertia 惯量
    */
   Load(double m, const std::array<double, 3> &cog, const std::array<double, 3> &inertia);

   double mass { 0 };  ///< 负载质量, 单位:千克
   std::array<double, 3> cog {};     ///< 质心 [x, y, z], 单位:米
   std::array<double, 3> inertia {}; ///< 惯量 [ix, iy, iz], 单位:千克·平方米
 };

 /**
  * @class Toolset
  * @brief 工具工件组信息, 根据一对工具工件的坐标、负载、机器人手持设置计算得出
  * @note 并不显式区分手持/外部. 该类可这样理解: 如手持工具, 则负载和机器人末端坐标系是工具的, 参考坐标系则是工件的；
  *       反之, 如果手持工件, 则负载和末端坐标系来自工件, 参考坐标系来自工具
  */
 class XCORE_API Toolset {
  public:
   Toolset() = default;
   /**
    * @param load 负载信息
    * @param end 末端坐标系
    * @param ref 参考坐标系
    */
   Toolset(const Load &load, const Frame &end, const Frame &ref);

   Load load {}; ///< 机器人末端手持负载
   Frame end {}; ///< 机器人末端坐标系相对法兰坐标系转换
   Frame ref {}; ///< 机器人参考坐标系相对世界坐标系转换
 };

 /**
  * @class FrameCalibrationResult
  * @brief 坐标系标定结果
  */
 class XCORE_API FrameCalibrationResult {
  public:
   FrameCalibrationResult() = default;
   Frame frame {};  ///< 标定结果
   std::array<double, 3> errors {}; ///< 样本点与TCP标定值的偏差, 依次为最小值,平均值,最大值, 单位m
 };

 /**
  * @class RLProjectInfo
  * @brief RL工程信息
  */
 class XCORE_API RLProjectInfo {
  public:
   /**
    * @brief constructor
    * @param name RL工程名
    */
   explicit RLProjectInfo(std::string name);

   std::string name; ///< 工程名称
   std::vector<std::string> taskList; ///< 任务名称列表
 };

 /**
  * @class WorkToolInfo
  * @brief 工具/工件信息。工件的坐标系已相对其用户坐标系变换
  */
 class XCORE_API WorkToolInfo {
  public:
   WorkToolInfo() = default;

   /**
    * @brief constructor
    * @param name 名称
    * @param isHeld 是否机器人手持
    * @param posture 位姿
    * @param load 负载
    */
   WorkToolInfo(std::string name, bool isHeld, const Frame &posture, const Load &load);

   std::string name {};  ///< 名称
   std::string alias {}; ///< 别名, 暂未使用
   bool robotHeld {};    ///< 是否机器人手持
   Frame pos {};         ///< 位姿
   Load load {};         ///< 负载
 };

 /**
  * @class NrtCommand
  * @brief 非实时运动指令
  */
 class XCORE_API NrtCommand {
  public:

   /**
    * @brief 机器人末端最大线速度, 单位mm/s
    * @see setDefaultSpeed()
    */
   double speed { USE_DEFAULT };

   /**
    * @brief 转弯区半径大小，单位mm
    * @see setDefaultZone()
    */
   double zone { USE_DEFAULT };

   /**
    * @brief 自定义信息，可在运动信息反馈中返回出来
    */
   std::string customInfo {};

   NrtCommand() = default;

   /**
    * @brief constructor
    * @param speed 本条指令的速度, 单位mm/s
    * @param zone 本条指令的转弯区, 单位mm
    */
   NrtCommand(double speed, double zone);

   virtual ~NrtCommand() = default;
 };

 /**
  * @class MoveAbsJCommand
  * @brief 运动指令 - 轴运动MoveAbsJ
  */
 class XCORE_API MoveAbsJCommand : public NrtCommand{
  public:
   /**
    * @param target 目标轴角度
    * @param speed 末端线速度, 单位mm/s, 关节速度根据末端线速度大小划分几个区间，详见setDefaultSpeed()
    * @param zone 转弯区, 单位mm
    */
   MoveAbsJCommand(JointPosition target, double speed = USE_DEFAULT, double zone = USE_DEFAULT);

   JointPosition target; ///< 目标关节点位

   double jointSpeed { USE_DEFAULT }; ///< 关节速度百分比，范围[0, 1]。大于等于0时生效；小于0时仍使用speed计算出的关节速度
 };

 /**
  * @brief 运动停留指令。可插在两条运动指令之间，前一条运动到位后，等待一段时间，再执行下一条。
  * 该指令执行完不会有信息反馈
  */
 class XCORE_API MoveWaitCommand : public NrtCommand {
  public:

   /**
    * @brief Constructor
    * @param duration 时长
    */
   MoveWaitCommand(std::chrono::steady_clock::duration duration);

   std::chrono::steady_clock::duration duration_; ///< 停留时长, 最小有效时长1ms
 };

 /**
  * @class MoveJCommand
  * @brief 运动指令 - 轴运动MoveJ
  */
 class XCORE_API MoveJCommand : public NrtCommand {
  public:
   /**
    * @param target 目标笛卡尔点位
    * @param speed 末端线速度, 单位mm/s, 关节速度根据末端线速度大小划分几个区间，详见setDefaultSpeed()
    * @param zone 转弯区, 单位mm
    */
   MoveJCommand(CartesianPosition target, double speed = USE_DEFAULT, double zone = USE_DEFAULT);

   CartesianPosition target; ///< 目标笛卡尔点位
   CartesianPosition::Offset offset; ///< 偏移选项

   double jointSpeed { USE_DEFAULT }; ///< 关节速度百分比，范围[0, 1]。大于等于0时生效；小于0时仍使用speed计算出的关节速度
 };

 /**
  * @class MoveLCommand
  * @brief 运动指令 - 末端直线轨迹MoveL
  */
 class XCORE_API MoveLCommand : public NrtCommand {
  public:
   /**
    * @param target 目标笛卡尔点位
    * @param speed 末端线速度, 单位mm/s
    * @param zone 转弯区, 单位mm
    */
   MoveLCommand(CartesianPosition target, double speed = USE_DEFAULT, double zone = USE_DEFAULT);

   CartesianPosition target; ///< 目标笛卡尔点位
   CartesianPosition::Offset offset; ///< 偏移选项

   double rotSpeed { USE_DEFAULT }; ///< 空间旋转速度，单位rad/s。大于等于0时生效；小于0时旋转速度默认为200°/s
 };

 /**
  * @class MoveCCommand
  * @brief 运动指令 - 圆弧轨迹MoveC
  */
 class XCORE_API MoveCCommand : public NrtCommand {
  public:
   /**
    * @param target 目标点
    * @param aux 辅助点
    * @param speed 末端线速度, 单位mm/s, 关节速度根据末端线速度大小划分几个区间，详见setDefaultSpeed()
    * @param zone 转弯区, 单位mm
    */
   MoveCCommand(CartesianPosition target, CartesianPosition aux, double speed = USE_DEFAULT, double zone = USE_DEFAULT);

   CartesianPosition target; ///< 目标笛卡尔点位
   CartesianPosition::Offset targetOffset; ///< 偏移选项
   CartesianPosition aux;    ///< 辅助点位
   CartesianPosition::Offset auxOffset; ///< 偏移选项

   double rotSpeed { USE_DEFAULT }; ///< 空间旋转速度，单位rad/s。大于等于0时生效；小于0时旋转速度默认为200°/s
 };

 /**
  * @brief 运动指令 - 全圆轨迹MoveCF
  */
 class XCORE_API MoveCFCommand : public MoveCCommand{
  public:
   /**
    * @brief 全圆姿态旋转类型
    */
   enum RotType {
     constPose, ///< 不变姿态
     rotAxis,   ///< 动轴旋转
     fixedAxis  ///< 定轴旋转
   };

   using MoveCCommand::MoveCCommand;
   /**
    * @param target 目标点
    * @param aux 辅助点
    * @param speed 末端线速度, 单位mm/s
    * @param zone 转弯区, 单位mm
    * @param angle 执行角度, 单位弧度
    */
   MoveCFCommand(const CartesianPosition &target, const CartesianPosition &aux, double angle, double speed = USE_DEFAULT, double zone = USE_DEFAULT);

   double angle { 0 }; ///< 全圆执行角度, 单位: 弧度
   RotType rotType { constPose }; ///< 全圆姿态旋转模式
 };

 /**
  * @brief 运动指令 - 螺旋线轨迹MoveSP
  */
 class XCORE_API MoveSPCommand : public NrtCommand {
  public:
   /**
    * @param target 终点姿态
    * @param r0 初始半径 [m]
    * @param rStep 每旋转单位角度，半径的变化 [m/rad]
    * @param angle 合计旋转角度 [rad]
    * @param dir 旋转方向, true - clockwise | false - anticlockwise
    * @param speed 末端线速度, 单位mm/s
    */
   MoveSPCommand(const CartesianPosition &target, double r0, double rStep, double angle, bool dir, double speed = USE_DEFAULT);

   CartesianPosition target; ///< 终点笛卡尔点位, 只使用点位的rpy来指定终点的姿态
   CartesianPosition::Offset targetOffset; ///< 偏移选项
   double radius { 0 };      ///< 初始半径, 单位: 米
   double radius_step { 0 }; ///< 每旋转单位角度，半径的变化，单位: 米/弧度
   double angle { 0 };       ///< 合计旋转角度, 单位: 弧度
   bool direction;           ///< 旋转方向, true - 顺时针 | false - 逆时针

   double rotSpeed { USE_DEFAULT }; ///< 空间旋转速度，单位rad/s。大于等于0时生效；小于0时旋转速度默认为200°/s
 };

 /**
  * @class LogInfo
  * @brief 控制器日志信息
  */
 class XCORE_API LogInfo {
  public:
   /**
    * @brief 日志等级
    */
   enum Level {
     info,    ///< 通知
     warning, ///< 警告
     error    ///< 错误
   };

   /**
    * @brief constructor
    * @param id 日志ID号
    * @param ts 日期时间
    * @param ct 内容
    * @param r 修复办法
    */
   LogInfo(int id, std::string ts, std::string ct, std::string r);

   const int id;                ///< 日志ID号
   const std::string timestamp; ///< 日期及时间
   const std::string content;   ///< 日志内容
   const std::string repair;    ///< 修复办法
 };

 /**
  * @brief 末端按键状态
  */
 struct KeyPadState {
   bool key1_state = false; ///< CR1号
   bool key2_state = false; ///< CR2号
   bool key3_state = false; ///< CR3号
   bool key4_state = false; ///< CR4号
   bool key5_state = false; ///< CR5号
   bool key6_state = false; ///< CR6号
   bool key7_state = false; ///< CR7号
 };

#if defined(XCORESDK_SUPPRESS_DLL_WARNING)
#pragma warning(pop)
#endif // if defined(XCORESDK_SUPPRESS_DLL_WARNING)

}  // namespace rokae

#endif //ROKAEAPI_INCLUDE_ROKAE_DATA_TYPES_H_
