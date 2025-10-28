/**
 * @file robot.h
 * @brief 机器人交互接口
 * @copyright Copyright (C) 2024 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef ROKAEAPI_ROBOT_H_
#define ROKAEAPI_ROBOT_H_

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include <cfloat>
#include "base.h"
#include "model.h"
#include "exception.h"
#include "motion_control_rt.h"
#include "planner.h"
#include "force_control.h"

namespace rokae {

 // forward declarations
 class BaseModel;

 /**
  * @struct Info
  * @brief 机器人基本信息，在与建立机器人连接后加载
  */
 struct Info {
   std::string id;      ///< 机器人uid, 可用于区分连接的机器人
   std::string version; ///< 控制器版本
   std::string type;    ///< 机器人机型名称
   int joint_num;       ///< 轴数
 };

 /**
  * @struct StateList
  * @brief 状态列表
  */
 struct StateList {
   std::vector<double> joint_pos; ///< 轴角，单位弧度
   CartesianPosition cart_pos {}; ///< 笛卡尔位姿
   std::vector<std::pair<std::string, bool>> digital_signals; ///< 数字量IO, 信号名称及数值
   std::vector<std::pair<std::string, double>> analog_signals; ///< 模拟量IO, 信号名称及数值
   OperateMode operation_mode; ///< 操作模式
   double speed_override; ///< 速度覆盖比例
 };

 /**
  * @class BaseRobot
  * @brief 机器人通用接口
  */
 class XCORE_API BaseRobot : public Base<BaseRobot> {

  public:

   // **********************************************************************
   // *************       Network communication methods      ***************
   // *************                网络通信接口                ***************

   /**
    * @brief 断开与机器人连接。断开前会停止机器人运动, 请注意安全
    * @param[out] ec 错误码
    */
   void disconnectFromRobot(error_code &ec) noexcept;

   // **********************************************************************
   // ***************      Motors power on/off methods      ****************
   // ***************              电机上下电接口             *****************

   /**
    * @brief 机器人上下电以及急停状态
    * @param[out] ec 错误码
    * @return on-上电 | off-下电 | estop-急停 | gstop-安全门打开
    */
   PowerState powerState(error_code &ec) const noexcept;

   /**
    * @brief 机器人上下电。注: 只有无外接使能开关或示教器的机器人才能手动模式上电。
    * @param[in] on true-上电 | false-下电
    * @param[out] ec 错误码
    */
   void setPowerState(bool on, error_code &ec) noexcept;


   // **********************************************************************
   // ***************     Robot operation mode methods      ****************
   // ***************            机器人操作模式接口            ****************

   /**
    * @brief 查询机器人当前操作模式
    * @param[out] ec 错误码
    * @return 手动 | 自动
    */
   OperateMode operateMode(error_code &ec) const noexcept;

   /**
    * @brief 切换手自动模式
    * @param[in] mode 手动/自动
    * @param[out] ec 错误码
    */
   void setOperateMode(OperateMode mode, error_code &ec) noexcept;

   // **********************************************************************
   // **************       Get robot information methods      **************
   // **************           查询机器人信息及状态接口           **************

   /**
    * @brief 查询机器人基本信息
    * @param[out] ec 错误码
    * @return 机器人基本信息
    */
   Info robotInfo(error_code &ec) const noexcept;

   /**
    * @brief 查询机器人当前运行状态 (空闲,运动中, 拖动开启等)
    * @param[out] ec 错误码
    * @return 运行状态枚举类
    */
   OperationState operationState(error_code &ec) const noexcept;

   // **********************************************************************
   // ******************      Get robot posture methods      ***************
   // ******************           获取机器人位姿接口           ***************

   /**
    * @brief 机器人当前轴角度, 机器人本体+外部轴, 单位: \f$[rad]\f$
    * 外部轴导轨单位\f$[m]\f$
    * @param[out] ec 错误码
    * @return 长度: \f$ \mathbb{R}^{DoF+ExJnt \times 1} \f$.
    */
   std::vector<double> jointPos(error_code &ec) noexcept;

   /**
    * @brief 机器人当前关节速度, 机器人本体+外部轴, 单位: \f$[\frac{rad}{s}]\f$
    * 外部轴导轨，单位\f$[\frac{m}{s}]\f$
    * @param[out] ec 错误码
    * @return 长度: \f$ \mathbb{R}^{DoF+ExJnt \times 1} \f$.
    */
   std::vector<double> jointVel(error_code &ec) noexcept;

   /**
    * @brief 获取机器人法兰或末端的当前位姿 \f$^{O}T_{F}~[m][rad]\f$.
    * @param[in] ct 坐标系类型
    * 1) flangeInBase: 法兰相对于基坐标系;
    * 2) endInRef: 末端相对于外部参考坐标系。例如,当设置了手持工具及外部工件后，该坐标系类型返回的是工具相对于工件坐标系的坐标。
    *              再例如，若外部参考坐标系与基坐标系重合，那么返回的结果等同于末端相对于基坐标系的位姿。
    * @param[out] ec 错误码
    * @return double数组, 长度: \f$ \mathbb{R}^{6 \times 1} \f$ = \f$ \mathbb{R}^{3 \times 1} \f$
    * transformation and \f$ \mathbb{R}^{3 \times 1} \f$ rotation \f$ [x, y, z, rx, ry, rz]^T \f$.
    */
   std::array<double, 6> posture(CoordinateType ct, error_code &ec) noexcept;

   /**
    * @brief 获取机器人法兰或末端的当前位姿
    * @param[in] ct 坐标系类型
    * @param[out] ec 错误码
    * @return 当前笛卡尔位置
    */
   CartesianPosition cartPosture(CoordinateType ct, error_code &ec) noexcept;

   // **********************************************************************
   // ***************      Get and set coordinate methods      *************
   // *****************           获取与设置坐标系接口           ***************

   /**
    * @brief 读取基坐标系, 相对于世界坐标系
    * @param[out] ec 错误码
    * @return 数组, 长度: \f$ \mathbb{R}^{6 \times 1} \f$ = \f$ \mathbb{R}^{3 \times 1} \f$
    * transformation and \f$ \mathbb{R}^{3 \times 1} \f$ rotation \f$ [x, y, z, rx, ry, rz]^T \f$.
    */
   std::array<double, 6> baseFrame(error_code &ec) const noexcept;

   /**
    * @brief 设置基坐标系, 设置后仅保存数值，重启控制器后生效
    * @param[in] frame 坐标系，默认使用自定义安装方式
    * @param[out] ec 错误码
    */
   void setBaseFrame(const Frame &frame, error_code &ec) noexcept;

   /**
    * @brief 查询当前工具工件组信息
    * @note 此工具工件组仅为SDK运动控制使用, 不与RL工程相关.
    * @param[out] ec 错误码
    */
   Toolset toolset(std::error_code &ec) const noexcept;

   /**
    * @brief 设置工具工件组信息
    * @note 此工具工件组仅为SDK使用, 不与RL工程相关。
    * 设置后RobotAssist右上角会显示“toolx", "wobjx", 状态监控显示的末端坐标也会变化。
    * 除此接口外, 如果通过RobotAssist更改默认工具工件(右上角的选项), 该工具工件组也会相应更改。
    * @param[in] toolset 工具工件组信息
    * @param[out] ec 错误码
    */
   void setToolset(const Toolset& toolset, error_code &ec) noexcept;

   /**
    * @brief 使用已创建的工具和工件，设置工具工件组信息
    * @note 设置前提:
    *   1) 使用RL工程中创建的工具工件: 需要先加载对应RL工程;
    *   2) 全局工具工件: 无需加载工程，直接调用即可。e.g. setToolset("g_tool_0", "g_wobj_0")
    * 一组工具工件无法同时为手持或外部；如果有冲突，以工具的位置为准，例如工具工件同时为手持，不会返回错误，但是工件的坐标系变成了外部
    * @param[in] toolName 工具名称
    * @param[in] wobjName 工件名称
    * @param[out] ec 错误码
    * @return 设置后的工具工件组信息。当发生错误设置失败时，返回Toolset类型初始化默认值0
    */
   Toolset setToolset(const std::string &toolName, const std::string &wobjName, error_code &ec) noexcept;

   // **********************************************************************
   // ********************    Input/Output devices     *********************
   // ********************         输入/输出设备         *********************
   /**
    * @brief 查询数字量输入信号值
    * @param[in] board IO板序号
    * @param[in] port 信号端口号
    * @param[out] ec 错误码
    * @return true-开 | false-关
    */
   bool getDI(unsigned int board, unsigned int port, error_code &ec) noexcept;

   /**
    * @brief 设置数字量输入信号，仅当输入仿真模式打开时可以设置(见setSimulationMode())
    * @param[in] board IO板序号
    * @param[in] port 信号端口号
    * @param[in] state true-开 | false-关
    * @param[out] ec 错误码
    */
   void setDI(unsigned board, unsigned port, bool state, error_code &ec) noexcept;

   /**
    * @brief 查询数字输出量信号值
    * @param[in] board IO板序号
    * @param[in] port 信号端口号
    * @param[out] ec 错误码
    * @return true-开 | false-关
    */
   bool getDO(unsigned int board, unsigned int port, error_code &ec) noexcept;

   /**
    * @brief 设置数字量输出信号值
    * @param[in] board IO板序号
    * @param[in] port 信号端口号
    * @param[in] state true-开 | false-关
    * @param[out] ec 错误码
    */
   void setDO(unsigned int board, unsigned int port, bool state, error_code &ec) noexcept;

   /**
    * @brief 读取模拟量输入信号值
    * @param[in] board IO板序号
    * @param[in] port 信号端口号
    * @param[out] ec 错误码
    * @return 信号值
    */
   double getAI(unsigned board, unsigned port, error_code &ec) noexcept;

   /**
    * @brief 设置模拟量输出信号
    * @param[in] board IO板序号
    * @param[in] port 信号端口号
    * @param[in] value 输出值
    * @param[out] ec 错误码
    */
   void setAO(unsigned board, unsigned port, double value, error_code &ec) noexcept;

   /**
    * @brief 设置输入仿真模式
    * @param[in] state true - 打开 | false - 关闭
    * @param[out] ec 错误码
    */
   void setSimulationMode(bool state, error_code &ec) noexcept;

   /**
    * @brief 读取寄存器值。可读取单个寄存器，寄存器数组，或按索引读取寄存器数组。
    * 如果要读取整个寄存器数组，value传入对应类型的vector，index值被忽略。
    * @tparam T 读取数值类型
    * @param[in] name 寄存器名称
    * @param[in] index 按索引读取寄存器数组中元素，从0开始。
    *           下列两种情况会报错：1) 索引超出数组长度; 2) 寄存器不是数组但index大于0
    * @param[out] value 寄存器数值，允许的类型有bool/int/float
    * @param[out] ec 错误码
    */
   template<typename T>
   void readRegister(const std::string &name, unsigned index, T &value, error_code &ec) noexcept;

   /**
    * @brief 写寄存器值。可写入单个寄存器，寄存器数组，或按索引写入寄存器数组中某一元素。
    * 如果要写整个寄存器数组，value传入对应类型的vector，index值被忽略。
    * @tparam T 写入数值类型
    * @param[in] name 寄存器名称
    * @param[in] index 数组索引，从0开始。
    *           下列两种情况会报错：1) 索引超出数组长度; 2) 寄存器不是数组但index大于0
    * @param[in] value 写入的数值
    * @param[out] ec 错误码
    */
   template<typename T>
   void writeRegister(const std::string &name, unsigned index, T value, error_code &ec) noexcept;


   // **********************************************************************
   // ******************        Other operations      **********************
   // ******************             其他操作           **********************
   /**
    * @brief 清除伺服报警
    * @param[out] ec 错误码，当有伺服报警且清除失败的情况下错误码置为-1
    */
   void clearServoAlarm(error_code &ec) noexcept;

   // **********************************************************************
   // ***************        Recover state methods         *****************
   // ***************              恢复状态接口             *****************

   /**
    * @brief 根据选项恢复机器人状态
    * @param[in] item 恢复选项，1：急停恢复
    * @param[out] ec 错误码
    */
   void recoverState(int item, error_code &ec) noexcept;

   /**
    * @brief 查询xCore-SDK版本
    * @return 版本号
    */
   static std::string sdkVersion() noexcept;

   /**
    * @brief 查询控制器最新的日志
    * @param[in] count 查询个数，上限是10条
    * @param[in] level 指定日志等级，空集合代表不指定
    * @param[out] ec 错误码
    * @return 日志信息
    */
   std::vector<LogInfo> queryControllerLog(unsigned count, const std::set<LogInfo::Level>& level, error_code &ec) noexcept;

   // **********************************************************************
   // ******************      Motion Controller        *********************
   // ******************        运动控制相关接口          *********************

   /**
    * @brief 设置运动控制模式
    * @note 在调用各运动控制接口之前, 须设置对应的控制模式。
    * @param[in] mode 模式
    * @param[out] ec 错误码
    */
   void setMotionControlMode(MotionControlMode mode, error_code &ec) noexcept;

   //  --------------    MotionControlMode::NrtCommand   ------------------
   //  ----------------         非实时运动指令              ------------------
   /**
    * @brief 运动重置, 清空已发送的运动指令, 清除执行信息
    * @note Robot类在初始化时会调用一次运动重置。RL程序和SDK运动指令切换控制，需要先运动重置。
    * @param[out] ec 错误码
    */
   void moveReset(error_code &ec) noexcept;

   /**
    * @brief 开始/继续运动
    * @param[out] ec 错误码
    */
   void moveStart(error_code &ec) noexcept;

   /**
    * @brief 暂停机器人运动; 暂停后可调用moveStart()继续运动。若需要完全停止，不再执行已添加的指令，可调用moveReset()
    * @note 目前支持stop2停止类型, 规划停止不断电, 参见StopLevel。
    * @param[out] ec 错误码
    */
   void stop(error_code &ec) noexcept;

   /**
    * @brief 添加单条或多条运动指令, 添加后调用moveStart()开始运动
    * @tparam Command 运动指令类: MoveJCommand | MoveAbsJCommand | MoveLCommand | MoveCCommand | MoveCFCommand |
    * MoveSPCommand;
    * 笛卡尔空间运动使用欧拉角XYZ表示旋转量，即使用trans&rpy的值
    * @param[in] cmds 指令列表, 允许的个数为1-100, 须为同类型的指令
    * @param[out] cmdID 本条指令的ID, 可用于查询指令执行信息
    * @param[out] ec 错误码, 仅反馈指令发送前的错误, 包括:
    *                1) 网络连接问题; 2) 指令个数不符;
    */
   template<class Command>
   void moveAppend(const std::vector<Command> &cmds, std::string &cmdID, error_code &ec) noexcept;

   /**
   * @brief 添加单条或多条运动指令, 添加后调用moveStart()开始运动
   * @tparam Command 运动指令类: MoveJCommand | MoveAbsJCommand | MoveLCommand | MoveCCommand | MoveCFCommand |
   * MoveSPCommand
   * @param[in] cmds 指令列表, 允许的个数为1-100, 须为同类型的指令
   * @param[out] cmdID 本条指令的ID, 可用于查询指令执行信息
   * @param[out] ec 错误码, 仅反馈指令发送前的错误, 包括:
   *                1) 网络连接问题; 2) 指令个数不符;
   */
   template<class Command>
   void moveAppend(std::initializer_list<Command> cmds, std::string &cmdID, error_code &ec) noexcept;

   /**
    * @brief 添加单条运动指令, 添加后调用moveStart()开始运动
    * @tparam Command 运动指令类: MoveJCommand | MoveAbsJCommand | MoveLCommand | MoveCCommand |
    * MoveCFCommand | MoveSPCommand | MoveWaitCommand
    * @param[in] cmd 指令
    * @param[out] cmdID 本条指令的ID, 可用于查询指令执行信息
    * @param[out] ec 错误码
    */
   template<class Command>
   void moveAppend(const Command &cmd, std::string &cmdID, error_code &ec) noexcept;

   /**
    * @brief 设定默认运动速度，单位mm/s, 初始值为100
    * @note 该数值表示末端最大线速度, 自动计算对应关节速度。
    *   关节速度百分比根据speed划分为5个的范围:
    *         < 100 : 10%
    *     100 ~ 200 : 30%
    *     200 ~ 500 : 50%
    *     500 ~ 800 : 80%
    *         > 800 : 100%
    *   空间旋转速度为200°/s
    * @param[in] speed 该接口不对参数进行范围限制。末端线速度的实际有效范围分别是(0, 4000](协作), (0, 7000](工业)。
    * @param[out] ec 错误码
    */
   void setDefaultSpeed(double speed, error_code &ec) noexcept;

   /**
    * @brief 设定默认转弯区, 单位:mm。初始值为为0 (fine, 无转弯区)
    * @note 该数值表示运动最大转弯区半径, 自动计算转弯百分比
    *   转弯百分比划分4个范围:
    *         < 1 : 0 (fine)
    *      1 ~ 20 : 10%
    *     20 ~ 60 : 30%
    *        > 60 : 100%
    * @param[in] zone 该接口不对参数进行范围限制。转弯区半径大小实际有效范围是[0, 200]。
    * @param[out] ec 错误码
    */
   void setDefaultZone(int zone, error_code &ec) noexcept;

   /**
    * @brief 设置是否使用轴配置数据(confData)计算逆解。初始值为false
    * @param[in] forced true - 使用运动指令的confData计算笛卡尔点位逆解, 如计算失败则返回错误;
    * false - 不使用，逆解时会选取机械臂当前轴角度的最近解
    * @param[out] ec 错误码
    */
   void setDefaultConfOpt(bool forced, error_code &ec) noexcept;

   /**
    * @brief 设置最大缓存指令个数，指发送到控制器待规划的路径点个数，允许的范围[1,300]，初始值为30。
    * @note 如果轨迹多为短轨迹，可以调大这个数值，避免因指令发送不及时导致机器人停止运动(停止后如果有未执行的指令，可moveStart()继续;
    * @param[in] number 个数
    * @param[out] ec 错误码
    */
   void setMaxCacheSize(int number, error_code &ec) noexcept;

   /**
    * @brief 动态调整机器人运动速率，非实时模式时生效。
    * @param[in] scale 运动指令的速度的比例，范围 0.01 - 1。当设置scale为1时，机器人将以路径原本速度运动。
    * @param[out] ec 错误码
    */
   void adjustSpeedOnline(double scale, error_code &ec) noexcept;

   /**
    * @brief 读取当前加/减速度和加加速度
    * @param[out] acc 系统预设加速度的百分比
    * @param[out] jerk 系统预设的加加速度的百分比
    * @param[out] ec 错误码
    */
   void getAcceleration(double &acc, double &jerk, error_code &ec) noexcept;

   /**
    * @brief 调节运动加/减速度和加加速度。如果在机器人运动中调用，当前正在执行的指令不生效，下一条指令生效
    * @param[in] acc 系统预设加速度的百分比，范围[0.2, 1.5], 超出范围不会报错，自动改为上限或下限值
    * @param[in] jerk 系统预设的加加速度的百分比，范围[0.1, 2], 超出范围不会报错，自动改为上限或下限值
    * @param[out] ec 错误码
    */
   void adjustAcceleration(double acc, double jerk, error_code &ec) noexcept;

   /**
    * @brief 开始jog机器人，需要切换到手动操作模式。
    * @note 调用此接口并且机器人开始运动后，无论机器人是否已经自行停止，都必须调用stop()来结束jog操作，否则机器人会一直处于jog的运行状态。
    * @param[in] space jog参考坐标系。
    *     1) 工具/工件坐标系使用原则同setToolset();
    *     2) 工业六轴机型和xMateCR/SR六轴机型支持两种奇异规避方式: JogOpt::singularityAvoidMode JogOpt::baseParallelMode
    *     3) CR5轴机型支持平行基座模式Jog: JogOpt::baseParallelMode
    * @param[in] rate 速率, 范围 0.01 - 1
    * @param[in] step 步长。单位: 笛卡尔空间-毫米 | 轴空间-度。步长大于0即可，不设置上限，
    *             如果机器人无法继续jog会自行停止运动。
    * @param[in] index 根据不同的space，该参数含义如下：
    *     1) 世界坐标系,基坐标系,法兰坐标系,工具工件坐标系:
    *       a) 6轴机型: 0~5分别对应X, Y, Z, Rx, Ry, Rz。>5代表外部轴(若有)
    *       b) 7轴机型6代表肘关节, >6代表外部轴(若有)
    *     2) 轴空间: 关节序号，从0开始计数
    *     3) 奇异规避模式,平行基座模式:
    *       a) 6轴机型：0~5分别对应X, Y, Z, J4(4轴), Ry, J6(6轴);
    *       b) 5轴机型：0~4分别对应X, Y, Z, Ry, J5(5轴)
    * @param[in] direction 根据不同的space和index，该参数含义如下：
    *     1) 奇异规避模式 J4: true - ±180° | false - 0°;
    *     2) 平行基座模式 J4 & Ry: true - ±180° | false - 0°
    *     3) 其它，true - 正向 | false - 负向
    * @param[out] ec 错误码
    */
   void startJog(JogOpt::Space space, double rate, double step, unsigned index, bool direction, error_code &ec) noexcept;

   /**
    * @brief 执行单条或多条运动指令，调用后机器人立刻开始运动
    * @tparam Command 运动指令类: MoveJCommand | MoveAbsJCommand | MoveLCommand | MoveCCommand | MoveCFCommand |
    * MoveSPCommand;
    * 笛卡尔空间运动使用欧拉角XYZ表示旋转量，即使用trans&rpy的值
    * @param[in] cmds 指令列表, 允许的个数为1-100,须为同类型的指令
    * @param[out] ec 错误码, 仅反馈执行前的错误, 包括:
    *                1) 网络连接问题; 2) 指令个数不符
    */
   template<class Command>
   void executeCommand(const std::vector<Command> &cmds, error_code &ec) noexcept;

   /**
    * @brief 执行单条或多条运动指令，调用后机器人立刻开始运动
    * @tparam Command 运动指令类: MoveJCommand | MoveAbsJCommand | MoveLCommand | MoveCCommand;
    * 笛卡尔空间运动使用欧拉角XYZ表示旋转量，即使用trans&rpy的值
    * @param[in] cmds 指令列表, 允许的个数为1-100,须为同类型的指令
    * @param[out] ec 错误码, 仅反馈执行前的错误, 包括:
    *                1) 网络连接问题; 2) 指令个数不符;
    */
   template<class Command>
   void executeCommand(std::initializer_list<Command> cmds, error_code &ec) noexcept;

   // *********************************************************************
   // *****************         事件/状态监听              ******************

   /**
    * @brief 设置接收事件的回调函数
    * @param[in] eventType 事件类型
    * @param[in] callback 处理事件的回调函数。说明:
    *   1) 对于Event::moveExecution, 回调函数在同一个线程执行, 请避免函数中有执行时间较长的操作;
    *   2) Event::safety则每次独立线程回调, 没有执行时间的限制
    * @param[out] ec 错误码
    */
   void setEventWatcher(Event eventType, const EventCallback &callback, error_code &ec) noexcept;

   /**
    * @brief 查询事件信息。与setEventWatcher()回调时的提供的信息相同，区别是这个接口是主动查询的方式
    * @param[in] eventType 事件类型
    * @param[out] ec 错误码
    * @return 事件信息
    */
   EventInfo queryEventInfo(Event eventType, error_code &ec) noexcept;

   // *********************************************************************
   // *****************        获取机器人实时状态数据        ******************

   /**
    * @brief 停止接收实时状态数据，同时控制器停止发送。可用于重新设置要接收的状态数据。
    */
   void stopReceiveRobotState() noexcept;

   /**
    * @brief 接收一次机器人状态数据，在每周期读取数据前，需调用此函数。建议按照设定的发送频率来调用，以获取最新的数据
    * @param[in] timeout 超时时间
    * @return 接收到的数据长度。如果超时前没有收到数据，那么返回0。
    * @throw RealtimeControlException 无法收到数据；或收到的数据有错误导致无法解析
    */
   unsigned updateRobotState(std::chrono::steady_clock::duration timeout);

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
   int getStateData(const std::string &fieldName, R &data);

   //  --------------    MotionControlMode::NrtRLTask   ------------------
   //  ----------------         RL工程相关指令            -------------------

   /**
    * @brief 查询工控机中RL工程名称及任务
    * @param[out] ec 错误码
    * @return 工程信息列表，若没有创建工程则返回空列表
    */
   std::vector<RLProjectInfo> projectsInfo(error_code &ec) noexcept;

   /**
    * @brief 加载工程
    * @param[in] name 工程名称
    * @param[in] tasks 要运行的任务。该参数必须指定，不能为空，否则无法执行工程。
    * @param[out] ec 错误码
    */
   void loadProject(const std::string &name, const std::vector<std::string> &tasks, error_code &ec) noexcept;

   /**
    * @brief 程序指针跳转到main。调用后，等待控制器解析完工程后返回，阻塞时间视工程大小而定，超时时间设定为10秒。
    * @param[out] ec 错误码。错误码能提供的信息有限，不能反馈如RL语法错误、变量不存在等错误。可通过queryControllerLog()查询错误日志。
    */
   void ppToMain(error_code &ec) noexcept;

   /**
    * @brief 开始运行当前加载的工程
    * @param[out] ec 错误码
    */
   void runProject(error_code &ec) noexcept;

   /**
    * @brief 暂停运行工程
    * @param[out] ec 错误码
    */
   void pauseProject(error_code &ec) noexcept;

   /**
    * @brief 更改工程的运行速度和循环模式
    * @param[in] rate 运行速率，范围 0.01 - 1
    * @param[in] loop true - 循环执行 | false - 单次执行
    * @param[out] ec 错误码
    */
   void setProjectRunningOpt(double rate, bool loop, error_code &ec) noexcept;

   /**
    * @brief 查询当前加载工程的工具信息
    * @param[out] ec 错误码
    * @return 工具信息列表, 若未加载任何工程或没有创建工具, 则返回默认工具tool0的信息
    */
   std::vector<WorkToolInfo> toolsInfo(std::error_code &ec) noexcept;

   /**
    * @brief 查询当前加载工程的工件信息
    * @param[out] ec 错误码
    * @return 工件信息列表, 若未加载任何工程或没有创建工件, 则返回空vector
    */
   std::vector<WorkToolInfo> wobjsInfo(std::error_code &ec) noexcept;

   // ---------------------------------------------------------------------
   // ---------------------           外部轴          ----------------------

   /**
    * @brief 设置导轨参数
    * @tparam R 参数类型
    * @param[in] name 参数名，见value说明
    * @param[in] value
    *   参数             |    参数名         |   数据类型
    *   开关             | enable           | bool
    *   基坐标系         | baseFrame         | Frame
    *   导轨名称         | name              | std::string
    *   编码器分辨率      | encoderResolution | int
    *   减速比           | reductionRatio    | double
    *   电机最大转速(rpm) | motorSpeed       | int
    *   软限位(m), [下限,上限]   | softLimit  | std::vector<double>
    *   运动范围(m), [下限,上限] | range      | std::vector<double>
    *   最大速度(m/s)      | maxSpeed | double
    *   最大加速度（m/s^2)  | maxAcc  | double
    *   最大加加速度(m/s^3) | maxJerk | double
    * @param[out] ec 错误码。参数名不存在或数据类型不匹配返回错误码
    */
   template<typename R>
   void setRailParameter(const std::string &name, R value, error_code &ec) noexcept;

   /**
    * @brief 读取导轨参数
    * @tparam R 参数类型
    * @param[in] name 参数名，见setRailParameter()
    * @param[out] value 参数数值，见setRailParameter()
    * @param[out] ec 错误码 参数名不存在或数据类型不匹配返回错误码
    */
   template<typename R>
   void getRailParameter(const std::string &name, R &value, error_code &ec) noexcept;

   // *********************************************************************
   // ***********           NTP相关 (非标配功能，需要额外安装)         **********

   /**
    * @brief 配置NTP。非标配功能，需要额外安装。
    * @param[in] server_ip NTP服务端IP
    * @param[out] ec 错误码，NTP服务未正确安装，或地址非法
    */
   void configNtp(const std::string &server_ip, error_code &ec) noexcept;

   /**
    * @brief 手动同步一次时间，远端IP是通过configNtp配置的。耗时几秒钟，阻塞等待同步完成，接口预设的超时时间是12秒
    * @param[out] ec 错误码，NTP服务未正确安装，或无法和服务端同步
    */
   void syncTimeWithServer(error_code &ec) noexcept;

   /**
    * @brief 检验笛卡尔轨迹是否可达，直线轨迹
    * @param[in] start 起始点
    * @param[in] start_joint 起始轴角 [弧度]
    * @param[in] target 目标点
    * @param[out] ec 含不可达的错误原因
    * @return 计算出的目标轴角，仅当无错误码时有效
    * @note 支持导轨，返回的目标轴角为轴数+外部轴数
    */
   std::vector<double> checkPath(const CartesianPosition &start,
                                const std::vector<double> &start_joint,
                                const CartesianPosition &target,
                                error_code &ec) noexcept;

   /**
    * @brief 校验多个直线轨迹
    * @param[in] start_joint 起始轴角，单位[弧度]
    * @param[in] points 笛卡尔点位，至少需要2个点，第一个点是起始点
    * @param[out] target_joint_calculated 若校验通过。返回计算出的目标轴角
    * @param[out] ec 校验失败的原因
    * @return 若校验失败，返回points中出错目标点的下标。其它情况返回0
    */
   int checkPath(const std::vector<double> &start_joint,
                 const std::vector<CartesianPosition> &points,
                 std::vector<double> &target_joint_calculated,
                                 error_code &ec) noexcept;

   /**
    * @brief 检验笛卡尔轨迹是否可达，包括圆弧，全圆
    * @param[in] start 起始点
    * @param[in] start_joint 起始轴角 [弧度]
    * @param[in] aux 辅助点
    * @param[in] target 目标点
    * @param[out] ec 含不可达的错误原因
    * @param[in] angle 全圆执行角度，不等于零时代表校验全圆轨迹
    * @param[in] rot_type 全圆旋转类型
    * @return 计算出的目标轴角，仅当无错误码时有效
    * @note 支持导轨，返回的目标轴角为轴数+外部轴数
    */
   std::vector<double> checkPath(const CartesianPosition &start,
                                const std::vector<double> &start_joint,
                                const CartesianPosition &aux,
                                const CartesianPosition &target,
                                error_code &ec, double angle =0.0,
                                MoveCFCommand::RotType rot_type = MoveCFCommand::constPose) noexcept;

   /**
    * @brief 析构Robot对象时会让机器人停止运动
    */
   virtual ~BaseRobot() noexcept;

  protected:

   /// @cond DO_NOT_DOCUMENT
   BaseRobot();
   /// @endcond

  public:
   /// @cond DO_NOT_DOCUMENT
   BaseRobot(const BaseRobot&) = delete;
   BaseRobot& operator= (BaseRobot&) = delete;
   /// @endcond

  XCORESDK_DECLARE_IMPL
 };

 /**
  * @class Robot_T
  * @brief 机器人模板类
  * @tparam Wt 协作/工业类型
  * @tparam DoF 轴数
  */
 template<WorkType Wt, unsigned short DoF>
 class XCORE_API Robot_T : virtual public BaseRobot {

  public:
   /**
    * @brief Default constructor, call connectToRobot(remoteIp) afterwards
    */
   Robot_T() = default;

   /**
    * @brief 创建机器人实例, 并连接机器人
    * @param[in] remoteIP 机器人IP地址
    * @param[in] localIP 本机地址。实时模式下收发交互数据用，可不设置；PCB3/4轴机型不支持
    * @throw NetworkException 网络连接错误
    * @throw ExecutionException 机器人实例与连接机型不符，或未授权SDK
    */
   explicit Robot_T(const std::string &remoteIP, const std::string &localIP = "");

   /**
    * @brief 连接到机器人。机器人地址为创建robot实例时传入的
    * @param[out] ec 错误码
    */
   void connectToRobot(error_code &ec) noexcept;

   /**
    * @brief 连接到机器人
    * @param remoteIP 机器人IP地址
    * @param localIP 本机地址。实时模式下收发交互数据用，可不设置；PCB3/4轴机型不支持
    * @throw NetworkException 网络连接错误
    * @throw ExecutionException 机器人实例与连接机型不符，或未授权SDK
    */
   void connectToRobot(const std::string &remoteIP, const std::string &localIP = "");

   // **********************************************************************
   // ******************             状态查询              ******************

   /**
    * @brief 查询当前位置, IO信号, 操作模式, 速度覆盖值
    * @param ec 错误码
    * @return 查询结果
    */
   StateList getStateList(error_code &ec) noexcept;

   // **********************************************************************
   // ******************    Get robot joint state       ********************
   // ******************        获取机器人关节状态         *********************

   /**
    * @brief 机器人当前轴角度, 单位: \f$[rad]\f$
    * @param[out] ec 错误码
    * @return 长度: \f$ \mathbb{R}^{DoF \times 1} \f$.
    */
   std::array<double, DoF> jointPos(error_code &ec) noexcept;

   /**
    * @brief 机器人当前关节速度, 单位: \f$[\frac{rad}{s}]\f$
    * @param[out] ec 错误码
    * @return 长度: \f$ \mathbb{R}^{DoF \times 1} \f$.
    */
   std::array<double, DoF> jointVel(error_code &ec) noexcept;

   /**
    * @brief 关节力传感器数值, 单位: \f$[Nm]\f$
    * @param[out] ec 错误码
    * @return 长度: \f$ \mathbb{R}^{DoF \times 1} \f$.
    */
   std::array<double, DoF> jointTorque(error_code &ec) noexcept;

   // **********************************************************************
   // *********************             标定           **********************

   /**
    * @brief 坐标系标定 (N点标定)
    * @param[in] type 坐标系类型，支持工具(FrameType::tool), 工件(FrameType::wobj), 基坐标系(FrameType::base)
    * @note  各坐标系类型支持的标定方法及注意事项：
    *   1) 工具坐标系: 三点/四点/六点标定法
    *   2) 工件坐标系: 三点标定。标定结果不会相对用户坐标系做变换，即，若为外部工件，返回的结果是相对于基坐标系的。
    *   3) 基坐标系: 六点标定。标定前请确保动力学约束和前馈已关闭。
    *              若标定成功(无错误码)，控制器会自动保存标定结果，重启控制器后生效。
    *   4) 导轨基坐标系: 三点标定。若标定成功(无错误码)，控制器会自动保存标定结果，重启控制器后生效。
    * @param[in] points 轴角度列表，列表长度为N。例如，使用三点法标定工具坐标系，应传入3组轴角度。轴角度的单位是弧度。
    * @param[in] is_held true - 机器人手持 | false - 外部。仅影响工具/工件的标定
    * @param[out] ec 错误码
    * @param[in] base_aux 基坐标系标定时用到的辅助点, 单位[米]
    * @return 标定结果，当错误码没有被置位时，标定结果有效。
    */
   FrameCalibrationResult calibrateFrame(FrameType type, const std::vector<std::array<double, DoF>> &points, bool is_held,
                                         error_code &ec, const std::array<double, 3> &base_aux = {}) noexcept;

   // **********************************************************************
   // ********     Robot model for dynamic/kinematic calculation    ********
   // ********         获取机器人模型类, 用于运动学/动力学计算             ********

   /**
    * @brief 获取模型类
    * @return Model类
    */
   Model_T<DoF> model() noexcept;

   // *********************************************************************
   // *****************        获取机器人实时状态数据        ******************

   /**
    * @brief 让机器人控制器开始发送实时状态数据。阻塞等待收到第一帧消息，超时时间为3秒
    * @param[in] interval 控制器发送状态数据的间隔，允许的时长：1ms/2ms/4ms/8ms/1s
    * @param[in] fields 接收的机器人状态数据, 最大总长度为1024个字节。支持的数据及名称见data_types.h, RtSupportedFields
    * @throw RealtimeControlException 设置了不支持的状态数据；或机器人无法开始发送数据；或总长度超过1024
    * @throw RealtimeStateException 已经开始发送数据；或超时后仍未收到第一帧数据
    */
   void startReceiveRobotState(std::chrono::steady_clock::duration interval, const std::vector<std::string>& fields);

   /**
    * @brief 获取当前软限位数值
    * @param[out] limits 各轴软限位 [下限, 上限]，单位: 弧度
    * @param[out] ec 错误码
    * @return true - 已打开 | false - 已关闭
    */
   bool getSoftLimit(std::array<double[2], DoF> &limits, error_code &ec) noexcept;

   /**
    * @brief 设置软限位。软限位设定要求：
    *     1) 打开软限位时，机械臂应下电且处于手动模式;
    *     2) 软限位不能超过机械硬限位
    *     3) 机械臂当前各轴角度应在设定的限位范围内
    * @param[in] enable true - 打开 | false - 关闭。
    * @param[out] ec 错误码
    * @param[in] limits 各轴[下限, 上限]，单位：弧度。
    *     1) 当limits为默认值时，视为仅打开软限位不修改数值; 不为默认值时，先修改软限位再打开
    *     2) 关闭软限位时不会修改限位数值
    */
   void setSoftLimit(bool enable, error_code &ec, const std::array<double[2], DoF> &limits = {{DBL_MAX, DBL_MAX}}) noexcept;

 };

 /**
  * @brief 协作机器人通用类
  */
 class XCORE_API BaseCobot: virtual public BaseRobot {
  public:
   using BaseRobot::BaseRobot;

   /**
    * @brief 打开拖动
    * @param[in] space 拖动空间. 轴空间拖动仅支持自由拖拽类型
    * @param[in] type 拖动类型
    * @param[out] ec 错误码
    * @param[in] enable_drag_button true - 打开拖动功能之后可以直接拖动机器人，不需要按住末端按键
    */
   void enableDrag(DragParameter::Space space, DragParameter::Type type, error_code& ec, bool enable_drag_button = false) noexcept;

   /**
    * @brief 关闭拖动
    * @param[out] ec 错误码
    */
   void disableDrag(error_code& ec) noexcept;

   /**
    * @brief 开始录制路径
    * @param[in] duration 路径的时长，单位:秒，范围1~1800.此时长只做范围检查用，到时后控制器不会停止录制，需要调用stopRecordPath()来停止
    * @param[out] ec 错误码
    */
   void startRecordPath(int duration, error_code& ec) noexcept;

   /**
    * @brief 停止录制路径, 若录制成功(无错误码)则路径数据保存在缓存中
    * @param[out] ec 错误码
    */
   void stopRecordPath(error_code& ec) noexcept;

   /**
    * @brief 取消录制, 缓存的路径数据将被删除
    * @param[out] ec 错误码
    */
   void cancelRecordPath(error_code& ec) noexcept;

   /**
    * @brief 保存录制好的路径
    * @param[in] name 路径名称
    * @param[out] ec 错误码
    * @param[in] saveAs 重命名，可选参数。
    *               如果已录制好一条路径但没有保存，则用该名字保存路径。如果没有未保存的路径，则将已保存的名为"name"的路径重命名为"saveAs"
    */
   void saveRecordPath(const std::string& name, error_code& ec, const std::string& saveAs = "") noexcept;

   /**
    * @brief 运动指令-路径回放。
    * 和其它运动指令类似，调用replayPath之后，需调用moveStart才会开始运动。
    * @param[in] name 要回放的路径名称
    * @param[in] rate 回放速率, 应小于3.0, 1为路径原始速率。注意当速率大于1时，可能产生驱动器无法跟随错误
    * @param[out] ec 错误码
    */
   void replayPath(const std::string& name, double rate, error_code& ec) noexcept;

   /**
    * @brief 删除已保存的路径
    * @param[in] name 要删除的路径名称
    * @param[out] ec 错误码。若路径不存在，错误码不会被置位
    * @param[in] removeAll 是否删除所有路径, 可选参数, 默认为否
    */
   void removePath(const std::string& name, error_code& ec, bool removeAll = false) noexcept;

   /**
    * @brief 查询已保存的所有路径名称
    * @param[out] ec 错误码
    * @return 名称列表, 若没有路径则返回空列表
    */
   std::vector<std::string> queryPathLists(error_code& ec) noexcept;

   /**
    * @brief 设置xPanel对外供电模式。注：仅部分机型支持xPanel功能，不支持的机型会返回错误码
    * @param[in] opt 模式
    * @param[out] ec 错误码
    */
   void setxPanelVout(xPanelOpt::Vout opt, error_code& ec) noexcept;

   /**
    * @brief 使用CR和SR末端的485通信功能，需要修改末端的参数配置，可通过此接口进行参数配置
    * @param[in] opt 对外供电模式，0：不输出，1：保留，2：12v，3：24v
    * @param[in] if_rs485 接口工作模式，是否打开末端485通信
    * @param[out] ec 错误码
    */
   void setxPanelRS485(xPanelOpt::Vout opt, bool if_rs485, error_code& ec) noexcept;


   /**
    * @brief 通过xPanel末端读写modbus寄存器
    * @param[in] slave_addr 设备地址 0-65535
    * @param[in] fun_cmd 功能码 0x03 0x04 0x06 0x10
    * @param[in] reg_addr 寄存器地址 0-65535
    * @param[in] data_type 支持的数据类型  int32、int16、uint32、uint16
    * @param[in] num 一次连续操作寄存器的个数 0-3，类型为int16/uint16时，最大为3；类型为int32/uint32、float时，最大为1，功能码为0x06时，此参数无效
    * @param[in/out] data_array 发送或接收数据的数组，非const，功能码为0x06时，只使用此数组的数据[0],此时num的值无效，除了0x06功能码，大小需要与num匹配
    * @param[in] if_crc_reverse 是否改变CRC校验高低位，默认false，少数厂家末端工具需要反转
    * @param[out] ec 错误码
    */
   void XPRWModbusRTUReg(int slave_addr, int fun_cmd, int reg_addr, std::string data_type, int num, std::vector<int>& data_array, bool if_crc_reverse, error_code& ec) noexcept;

   /**
    * @brief 通过xPanel末端读写modbus线圈或离散输入
    * @param[in] slave_addr 设备地址 0-65535
    * @param[in] fun_cmd 功能码 0x01 0x02 0x05 0x0F
    * @param[in] coil_addr 线圈或离散输入寄存器地址 0-65535
    * @param[in] num 一次连续读写线圈离散输入的个数（0-48），功能码0x05时，此值无效
    * @param[in/out] data_array 发送或接收数据的数组，非const，功能码为0x05时，只使用此数组的数据[0],此时num的值无效，除了0x05功能码，大小需要与num匹配
    * @param[in] if_crc_reverse 是否改变CRC校验高低位，默认false，少数厂家末端工具需要反转
    * @param[out] ec 错误码
    */
   void XPRWModbusRTUCoil(int slave_addr, int fun_cmd, int coil_addr, int num, std::vector<bool>& data_array, bool if_crc_reverse, error_code& ec) noexcept;

   /**
    * @brief 通过xPanel末端直接传输RTU协议裸数据
    * @param[in] send_byte 发送字节长度  0-16
    * @param[in] rev_byte 接收字节长度 0-16
    * @param[in] send_data 发送字节数据 数组长度需要和send_byte 参数一致
    * @param[out] rev_data 接收字节数据 数组长度需要和rev_byte 参数一致
    * @param[out] ec 错误码
    */
   void XPRS485SendData(int send_byte, int rev_byte, const std::vector<uint8_t>& send_data, std::vector<uint8_t>& rev_data, error_code& ec) noexcept;

   /**
    * @brief 获取末端按键状态，不支持的机型会返回错误码
    * @param[out] ec 错误码
    * @return 末端按键的状态。末端按键编号见《xCore机器人控制系统使用手册》末端把手的图示。
    */
   KeyPadState getKeypadState(error_code& ec) noexcept;

   // **********************************************************
   // *****************    实时接口    ***************************

   /**
    * @brief 设置发送实时运动指令网络延迟阈值，即RobotAssist - RCI设置界面中的”包丢失阈值“。
    * 请在切换到RtCommand模式前进行设置，否则不生效。
    * @param[in] percent 允许的范围0 - 100
    * @param[out] ec 错误码
    */
   void setRtNetworkTolerance(unsigned percent, error_code& ec) noexcept;

   /**
    * @brief 兼容RCI客户端设置的接口。通过SDK设置运动控制模式为实时模式之后，无法再使用原RCI客户端控制机器人。
    * 若有使用原版的需求，可在切换到非实时模式后，调用此接口。然后再在RobotAssist上打开RCI功能，即可使用RCI客户端。
    * @param[in] use true - 切换到使用第一代
    * @param[out] ec 错误码
    */
   void useRciClient(bool use, error_code& ec) noexcept;

   // *********************************************************************
   // ******************            碰撞检测            *********************

   /**
    * @brief 关闭碰撞检测功能
    * @param[out] ec 错误码
    */
   void disableCollisionDetection(error_code& ec) noexcept;

 };

 /**
  * @class Cobot
  * @brief 协作机器人模板类, 提供协作机器人支持功能的接口
  * @tparam DoF 轴个数
  */
 template<unsigned short DoF>
 class XCORE_API Cobot : public Robot_T<WorkType::collaborative, DoF>, public BaseCobot {
  public:
   /**
    * @brief default constructor
    */
   Cobot();

   /**
     * @brief 协作机器人
     * @param[in] remoteIP 机器人IP地址
     * @param[in] localIP 本机地址。实时模式下收发交互数据用，可不设置。
     * @throw NetworkException 网络连接错误
     * @throw ExecutionException 机器人实例与连接机型不符，或未授权SDK
     */
   explicit Cobot(const std::string &remoteIP, const std::string& localIP = "");

   /**
    * @brief 析构，若此时机器人在运动则将停止运动
    */
   virtual ~Cobot();


   // **********************************************************
   // *****************    实时接口    ***************************

   /**
    * @brief 创建实时运动控制类(RtMotionControlCobot)实例，通过此实例指针进行实时模式相关的操作。
    * @note 除非重复调用此接口，客户端内部逻辑不会主动析构返回的对象，
    * 包括但不限于断开和机器人连接disconnectFromRobot()，切换到非实时运动控制模式等，但做上述操作之后再进行实时模式控制会产生异常。
    * @return 控制器对象
    * @throw RealtimeControlException 创建RtMotionControl实例失败，由于网络问题
    * @throw ExecutionException 没有切换到实时运动控制模式
    */
   std::weak_ptr<RtMotionControlCobot<DoF>> getRtMotionController();


   // *********************************************************************
   // ******************            碰撞检测            *********************

   /**
    * @brief 设置碰撞检测相关参数, 打开碰撞检测功能。
    * @param[in] sensitivity 碰撞检测灵敏度，范围0.01-2.0
    * @param[in] behaviour 碰撞后机器人行为, 支持stop1(安全停止, stop0和stop1处理方式相同), stop2(触发暂停）, suppleStop(柔顺停止)
    * @param[in] fallback_compliance
    *   1) 碰撞后行为是安全停止或触发暂停时，该参数含义是碰撞后回退距离，单位: 米
    *   2) 碰撞后行为是柔顺停止时，该参数含义是柔顺度，范围 [0.0, 1.0]
    * @param[out] ec 错误码
    */
   void enableCollisionDetection(const std::array<double, DoF> &sensitivity, StopLevel behaviour,
                                 double fallback_compliance, error_code &ec) noexcept;

   /**
    * @brief 力传感器标定。标定过程需要约100ms, 该函数不会阻塞等待标定完成。
    * 标定前需要通过setToolset()设置正确的负载(Toolset::load), 否则会影响标定结果准确性。
    * @param[in] all_axes true - 标定所有轴 | false - 单轴标定
    * @param[in] axis_index 轴下标, 范围[0, DoF), 仅当单轴标定时生效
    * @param[out] ec 错误码
    */
   void calibrateForceSensor(bool all_axes, int axis_index, error_code &ec) noexcept;

   // *********************************************************************
   // *********************          力控指令            ********************

   /**
    * @brief 力控指令类
    * @return ForceControl_T
    */
   ForceControl_T<DoF> forceControl() noexcept;


#ifdef XMATEMODEL_LIB_SUPPORTED
   /**
    * @brief 获取xMate模型类
    * @throw ExecutionException 从控制器读取模型参数失败
    */
   xMateModel<DoF> model();
#endif

  XCORESDK_DECLARE_IMPLD
 };

 /**
  * @class IndustrialRobot
  * @brief 工业机器人模板类
  * @tparam DoF 轴数
  */
 template<unsigned short DoF>
 class XCORE_API IndustrialRobot: public Robot_T<WorkType::industrial,DoF> {
  public:
   using Robot_T<WorkType::industrial,DoF>::Robot_T;
 };

 // ***********************************************************************
 // ***************      Robot classes for instantiate      ***************
 // ***************           可实例化的机器人类             ****************

 /**
  * @class xMateRobot
  * @brief 6轴协作机器人, 包括 xMateCR7/12, xMateSR3/4, xMateER3/7
  */
 class XCORE_API xMateRobot : public Cobot<6> {
  public:
   /**
    * @brief default constructor
    */
   xMateRobot() = default;

   /**
    * @brief 创建机器人实例并连接
    * @param remoteIP 机器人IP地址
    * @param localIP 本机地址，实时收发数据时需要设置
    * @throw NetworkException 网络连接错误
    * @throw ExecutionException 机器人实例与连接机型不符，或未授权SDK
    */
   explicit xMateRobot(const std::string &remoteIP, const std::string& localIP = "");

   /**
    * @brief 打开/关闭奇异点规避功能。只适用于部分机型:
    *   1) 四轴锁定: 支持xMateCR和xMateSR机型；
    *   2) 牺牲姿态: 支持所有协作六轴机型；
    *   3) 轴空间插补: 不支持
    * @param[in] method 奇异规避方式
    * @param[in] enable true - 打开功能 | false - 关闭;
    * 对于四轴锁定方式, 打开之前要确保4轴处于零位。
    * @param[in] limit 不同的规避方式，该参数含义分别为:
    *   1) 牺牲姿态: 允许的姿态误差, 范围 (0, PI*2], 单位弧度
    *   2) 四轴锁定: 无参数
    * @param[out] ec 错误码
    */
   void setAvoidSingularity(AvoidSingularityMethod method, bool enable, double limit, error_code &ec) noexcept;

   /**
    * @brief 查询是否处于规避奇异点的状态
    * @param[in] method 奇异规避的方式
    * @param[out] ec 错误码
    * @return true - 已打开
    */
   bool getAvoidSingularity(AvoidSingularityMethod method, error_code &ec) noexcept;
 };

 /**
  * @class xMateCr5Robot
  * @brief 5轴协作机器人, 包括 XMC17_5/XMC25_5
  */
 class XCORE_API xMateCr5Robot : public Cobot<5> {
  public:
   /**
	* @brief default constructor
	*/
   xMateCr5Robot() = default;

   /**
	* @brief 创建机器人实例并连接
	* @param remoteIP 机器人IP地址
	* @param localIP 本机地址，实时收发数据时需要设置
	* @throw NetworkException 网络连接错误
	* @throw ExecutionException 机器人实例与连接机型不符，或未授权SDK
	*/
   explicit xMateCr5Robot(const std::string &remoteIP, const std::string& localIP = "");

 };

 /**
  * @class xMateErProRobot
  * @brief 7轴协作机器人, 包括 xMateER3 Pro / xMateER7 Pro
  */
 class XCORE_API xMateErProRobot : public Cobot<7> {
  public:
   /**
    * @brief default constructor
    */
   xMateErProRobot() = default;

   /**
    * @brief 创建协作7轴机器人实例并连接
    * @param remoteIP 机器人IP地址
    * @param localIP 本机地址，实时收发数据时需要设置
    * @throw NetworkException 网络连接错误
    * @throw ExecutionException 机器人实例与连接机型不符，或未授权SDK
    */
   explicit xMateErProRobot(const std::string &remoteIP, const std::string& localIP = "");
 };

 /**
  * @class StandardRobot
  * @brief 标准工业6轴机型
  */
 class XCORE_API StandardRobot: public IndustrialRobot<6> {
  public:

   /**
    * @brief Default constructor
    */
   StandardRobot();

   /**
     * @brief 工业6轴机器人
     * @param[in] remoteIP 机器人IP地址
     * @param[in] localIP 本机地址。实时模式下收发交互数据用，使用实时运动控制模式时需要设置。
     * @throw NetworkException 网络连接错误
     * @throw ExecutionException 机器人实例与连接机型不符，或未授权SDK
     */
   explicit StandardRobot(const std::string& remoteIP, const std::string& localIP = "");

   // *********************************************************************
   // ******************            碰撞检测            *********************

   /**
    * @brief 设置碰撞检测相关参数, 打开碰撞检测功能。工业机型只支持stop1（安全停止）
    * @param[in] sensitivity 碰撞检测灵敏度，范围0.01-2.0
    * @param[in] fallback 碰撞后回退距离，单位: 米
    * @param[out] ec 错误码
    */
   void enableCollisionDetection(const std::array<double, 6> &sensitivity, double fallback, error_code &ec) noexcept;

   /**
    * @brief 关闭碰撞检测功能
    * @param[out] ec 错误码
    */
   void disableCollisionDetection(error_code &ec) noexcept;

   /**
    * @brief 创建实时运动控制类(RtMotionControlIndustrial)实例，通过此实例指针进行实时模式相关的操作。
    * @note 除非重复调用此接口，客户端内部逻辑不会主动析构返回的对象，
    * 包括但不限于断开和机器人连接disconnectFromRobot()，切换到非实时运动控制模式等，但做上述操作之后再进行实时模式控制会产生异常。
    * @return 控制器对象
    * @throw RealtimeControlException 创建RtMotionControl实例失败，由于网络问题
    * @throw ExecutionException 没有切换到实时运动控制模式
    */
   std::weak_ptr<RtMotionControlIndustrial<6>> getRtMotionController();

   /**
    * @brief 设置发送实时运动指令网络延迟阈值。
    * 请在切换到RtCommand模式前进行设置，否则不生效。
    * @param[in] percent 允许的范围0 - 100
    * @param[out] ec 错误码
    */
   void setRtNetworkTolerance(unsigned percent, error_code &ec) noexcept;

   /**
    * @brief 打开/关闭奇异点规避功能
    * @param[in] method 奇异规避方式，三种方式都支持
    * @param[in] enable true - 打开功能 | false - 关闭;
    * 对于四轴锁定方式, 打开之前要确保4轴处于零位
    * @param[in] threshold 不同的规避方式，该参数含义分别为:
    *   1) 牺牲姿态: 允许的姿态误差, 范围 (0, PI*2], 单位弧度
    *   2) 轴空间插补: 规避半径, 范围[0.005, 10], 单位米
    *   3) 四轴锁定: 无参数
    * @param[out] ec 错误码
    */
   void setAvoidSingularity(AvoidSingularityMethod method, bool enable, double threshold, error_code &ec) noexcept;

   /**
    * @brief 查询是否处于规避奇异点的状态
    * @param[in] method 奇异规避的方式
    * @param[out] ec 错误码
    * @return true - 已打开
    */
   bool getAvoidSingularity(AvoidSingularityMethod method, error_code &ec) noexcept;

  XCORESDK_DECLARE_IMPLD
 };

 /**
  * @class PCB3Robot
  * @brief PCB3轴机型
  */
 class XCORE_API PCB3Robot: public IndustrialRobot<3> {
  public:
   /**
    * @brief Default constructor
    */
   PCB3Robot() = default;

   /**
    * @brief 创建PCB3机器人实例并连接
    * @param remoteIP 机器人IP地址
    * @throw NetworkException 网络连接异常
    */
   explicit PCB3Robot(const std::string &remoteIP);
 };

 /**
  * @class PCB4Robot
  * @brief PCB4轴机型
  */
 class XCORE_API PCB4Robot: public IndustrialRobot<4> {
  public:
   /**
    * @brief Default constructor
    */
   PCB4Robot() = default;

   /**
    * @brief 创建PCB4机器人实例并连接
    * @param remoteIP 机器人IP地址
    */
   explicit PCB4Robot(const std::string &remoteIP);
 };

 /// @cond DO_NOT_DOCUMENT
 // backward-compatible, using alias
 using XMateRobot = xMateRobot;
 using XMateErProRobot = xMateErProRobot;
 /// @endcond

}  // namespace rokae

#endif // ROKAEAPI_ROBOT_H_
