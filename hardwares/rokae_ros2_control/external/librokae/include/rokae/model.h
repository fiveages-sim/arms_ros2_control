/**
 * @file model.h
 * @brief xMateModel模型库
 * @copyright Copyright (C) 2024 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef ROKAEAPI_MODEL_H
#define ROKAEAPI_MODEL_H

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include "base.h"
#include "data_types.h"

namespace rokae {

 // forward declarations
 class BaseRobot;
 class XService;
 struct Info;

 /**
  * @class BaseModel
  * @brief 模型通用类
  */
 class XCORE_API BaseModel : public Base<BaseModel>{
  public:

   /// @cond DO_NOT_DOCUMENT
   explicit BaseModel(std::shared_ptr<XService> rpc);
   virtual ~BaseModel();
   /// @endcond

  XCORESDK_DECLARE_IMPL
 };

/**
 * @class Model_T
 * @brief 模型模板类
 * @tparam DoF 轴数
 */
 template<unsigned short DoF>
 class XCORE_API Model_T : public BaseModel {

  public:
   using BaseModel::BaseModel;

   /**
    * @brief 根据位姿计算逆解。逆解选解策略:
    *   1) 默认Conf关闭时，选择离当前位置最近的解
    *   2) 默认Conf打开时，用confData计算
    * @param[in] posture 机器人末端位姿，相对于外部参考坐标系。参考的坐标系是通过setToolset()设置的
    * @param[out] ec 错误码
    * @return 轴角度, 单位:弧度
    */
   std::array<double, DoF> calcIk(CartesianPosition posture, error_code &ec) noexcept;

   /**
    * @brief 根据位姿计算给定工具工件坐标系下逆解。逆解选解策略:
    *   1) 默认Conf关闭时，选择离当前位置最近的解
    *   2) 默认Conf打开时，用confData计算
    * @param[in] posture 机器人末端位姿，相对于外部参考坐标系
    * @param[in] tool_set 工具工件坐标系
    * @param[out] ec 错误码
    * @return 轴角度, 单位:弧度
    */
   std::array<double,DoF> calcIk(CartesianPosition posture,const Toolset &tool_set, error_code& ec) noexcept;


   /**
    * @brief 根据位姿计算逆解。并自动遍历臂角
    * @param[in] posture 机器人末端位姿，相对于外部参考坐标系。参考的坐标系是通过setToolset()设置的
    * @param[in] elb_accuracy 臂角精度
    * @param[out] ec 错误码
    * @return 轴角度, 单位:弧度
    */
   std::array<double, DoF> calcIk_SearchElbow(CartesianPosition posture, double elb_accuracy, error_code &ec) noexcept;

   /**
    * @brief 根据位姿计算给定工具工件坐标系下逆解。并自动遍历臂角
    * @param[in] posture 机器人末端位姿，相对于外部参考坐标系
    * @param[in] tool_set 工具工件坐标系
    * @param[in] elb_accuracy 臂角精度
    * @param[out] ec 错误码
    * @return 轴角度, 单位:弧度
    */
   std::array<double, DoF> calcIk_SearchElbow(CartesianPosition posture,const Toolset &tool_set, double elb_accuracy, error_code& ec) noexcept;

   /**
    * @brief 根据轴角度计算正解。
    * @param[in] joints 轴角度, 单位: 弧度
    * @param[out] ec 错误码
    * @return 机器人末端位姿，相对于外部参考坐标系。参考的坐标系是通过setToolset()设置的
    */
   CartesianPosition calcFk(const std::array<double, DoF> &joints, error_code &ec) noexcept;

   /**
    * @brief 根据轴角度计算给定工具工件坐标系下正解
    * @param joints 轴角度, 单位: 弧度
    * @param tool_set 工具工件坐标系
    * @param ec 错误码
    * @return 机器人末端位姿，相对于外部参考坐标系
    */
   CartesianPosition calcFk(const std::array<double,DoF> &joints,const Toolset &tool_set, error_code& ec) noexcept;

 };

 /**
  * @enum SegmentFrame
  * @brief 连杆标号
  */
 enum class SegmentFrame : unsigned {
   joint1 = 1, joint2 = 2, joint3 = 3, joint4 = 4, joint5 = 5,
   joint6 = 6, joint7 = 7, flange = 8, endEffector = 9, stiffness = 10 };

 /**
  * @enum TorqueType
  * @brief 力矩类型
  */
 enum class TorqueType {
   full,     ///< 关节力矩，由动力学模型计算得到
   inertia,  ///< 惯性力
   coriolis, ///< 科氏力
   friction, ///< 摩擦力
   gravity   ///< 重力
 };

#ifdef XMATEMODEL_LIB_SUPPORTED

 /**
  * @class xMateModel
  * @brief xMate模型库。支持的机型: xMateER系列, XMC7/12/18/20, XMS3/4
  * @tparam DoF 轴数
  */
 template <unsigned short DoF>
 class XCORE_API xMateModel : public Model_T<DoF> {

  public:
   /**
    * @brief Create an xMateModel instance
    * @throw ExecutionException 加载模型失败
    */
   explicit xMateModel(std::shared_ptr<XService> rpc, const Info& info);
   ~xMateModel();

   /**
    * @brief 设置负载参数，只在计算时使用，并不将参数传给机器人控制器，设置后动力学计算结果相应改变
    * @param[in] mass 质量
    * @param[in] cog 质心, 单位: m
    * @param[in] inertia 惯量
    * @see Load
    */
   void setLoad(double mass, const std::array<double, 3> &cog, const std::array<double, 3> &inertia);

   /**
    * @brief 设置TCP工具，只在计算时使用，并不将参数传给机器人控制器，设置TCP后，正逆解结果和输入参数相应改变
    * @param[in] f_t_ee 末端执行器相对于法兰的位姿
    * @param[in] ee_t_k 刚度坐标系相对于末端执行器的位姿
    */
   void setTcpCoor(const std::array<double, 16> &f_t_ee, const std::array<double, 16> &ee_t_k);

   /**
    * @brief 获取笛卡尔空间位置
    * @param[in] jntPos 需要计算笛卡尔位姿的关节角度
    * @param[in] nr 指定坐标系, 缺省值为flange
    * @return 向量化4x4位姿矩阵，行优先.
    */
   std::array<double, 16> getCartPose(const std::array<double, DoF> &jntPos, SegmentFrame nr = SegmentFrame::flange);

   /**
    * @brief 获取笛卡尔空间速度
    * @param[in] jntPos 需要计算笛卡尔空间速度的关节角度
    * @param[in] jntVel 需要计算笛卡尔空间速度的关节角速度
    * @param[in] nr 指定坐标系, 缺省值为flange
    * @return 计算结果
    */
   std::array<double, 6> getCartVel(const std::array<double, DoF> &jntPos, const std::array<double, DoF> &jntVel,
                                    SegmentFrame nr = SegmentFrame::flange);

   /**
    * @brief 获取笛卡尔空间加速度
    * @param[in] jntPos 需要计算笛卡尔空间速度的关节角度
    * @param[in] jntVel 需要计算笛卡尔空间速度的关节角速度
    * @param[in] jntAcc 需要计算笛卡尔空间速度的关节角加速度
    * @param[in] nr 指定坐标系
    * @return 计算结果
    */
   std::array<double, 6> getCartAcc(const std::array<double, DoF> &jntPos,
                                    const std::array<double, DoF> &jntVel,
                                    const std::array<double, DoF> &jntAcc,
                                    SegmentFrame nr = SegmentFrame::flange );

   /**
    * @brief 逆解获得关节空间位置。一个位姿可能对应多个关节角度，jntPos的选取原则是选取一个与jntInit最近的解。
    * @param[in] cartPos 法兰笛卡尔空间位姿
    * @param[in] elbow 臂角
    * @param[in] jntInit 初始关节角度
    * @param[out] jntPos 关节空间位置
    * @return 计算逆解结果 -
    *    1) -1, -2, -3: 无解，原因是cartPos超出机器人工作空间;
    *    2) -4, -5: jntPos与jntInit相差较大，一般认为jntInit代表机器人当前位置，jntPos与jntInit之差可以等效为电机转速。
    *               若超过机器人轴额定转速，则返回-4或-5;
    *    3) -6, -7: jntPos超过软限位;
    *    4)	-8: 机器人奇异；
    */
    int getJointPos(const std::array<double, 16> &cartPos,
                    double elbow,
                    const std::array<double, DoF> &jntInit,
                    std::array<double, DoF> &jntPos);

   /**
    * @brief 目标比较从[-PI,PI]之间进行遍历，得到其中最近的逆解
    * @param[in] cartPos 法兰笛卡尔空间位姿
    * @param[in] psi_accuracy 臂角求解精度，单位度(例如1、10、20)，精度越高，求解时间越长
    * @param[in] elbow 臂角
    * @param[in] jntInit 初始关节角度
    * @param[out] jntPos 关节空间位置
    * @return 计算逆解结果 -
    *    1) -1, -2, -3: 无解，原因是cartPos超出机器人工作空间或无解
    *    2) -4, -5: jntPos与jntInit相差较大，一般认为jntInit代表机器人当前位置，jntPos与jntInit之差可以等效为电机转速。
    *               若超过机器人轴额定转速，则返回-4或-5;
    *    3) -6, -7: jntPos超过软限位;
    *    4)	-8: 机器人奇异；
    */
    int getJointPosSearchPsi(const std::array<double, 16> &cartPos, const double& psi_accuracy, double elbow,
                                  const std::array<double, DoF> &q_init, std::array<double, DoF> &q);
   /**
    * @brief 逆解获得关节空间速度
    * @param[in] cartVel 法兰笛卡尔空间速度
    * @param[in] jntPos 此时关节角度
    * @return 计算结果
    */
   std::array<double, DoF> getJointVel(const std::array<double, 6> &cartVel, const std::array<double, DoF> &jntPos);

   /**
    * @brief 逆解获得关节空间加速度
    * @param[in] cartAcc 法兰笛卡尔空间加速度
    * @param[in] jntPos 此时关节角度
    * @param[in] jntVel 此时关节角速度
    * @return 计算结果
    */
   std::array<double, DoF> getJointAcc(const std::array<double, 6> &cartAcc,
                                       const std::array<double, DoF> &jntPos,
                                       const std::array<double, DoF> &jntVel);

   /**
    * @brief 获取指定坐标系相对于基坐标系的雅克比矩阵, 行优先
    * @param[in] jntPos 关节角度.
    * @param[in] nr 指定坐标系
    * @return 计算结果, 长度 \f$ \mathbb{R}^{6 \times DoF} \f$
    */
   std::array<double, DoF*6> jacobian(const std::array<double, DoF> &jntPos, SegmentFrame nr = SegmentFrame::flange);

   /**
    * @brief 获取指定坐标系相对于基坐标系的雅克比矩阵, 行优先
    * @param[in] jntPos 关节角度.
    * @param[in] f_t_ee 末端执行器相对于法兰坐标系的位姿.
    * @param[in] ee_t_k 刚度坐标系相对于末端执行器的位姿.
    * @param[in] nr 指定坐标系
    * @return 计算结果, 长度 \f$ \mathbb{R}^{6 \times DoF} \f$
    */
   std::array<double, DoF*6> jacobian(const std::array<double, DoF> &jntPos,
                                      const std::array<double, 16> &f_t_ee,
                                      const std::array<double, 16> &ee_t_k,
                                      SegmentFrame nr = SegmentFrame::flange);

   /**
    * @brief 由模型计算关节力矩
    * @param[in] jntPos 关节角度
    * @param[in] jntVel 关节角速度
    * @param[in] jntAcc 关节角加速度
    * @param[in] torque_type 指定力矩类型
    * @return 计算结果，单位: Nm
    */
   std::array<double, DoF> getTorque(const std::array<double, DoF> &jntPos,
                                     const std::array<double, DoF> &jntVel,
                                     const std::array<double, DoF> &jntAcc,
                                     TorqueType torque_type);

   /**
    * @brief 由模型计算关节力矩，计算结果单位：Nm。如有负载，先通过setLoad()设置负载参数。
    * @param[in] jntPos 关节角度
    * @param[in] jntVel 关节角速度
    * @param[in] jntAcc 关节角加速度
    * @param[out] trq_full 总关节力矩
    * @param[out] trq_inertia 离心力
    * @param[out] trq_coriolis 科氏力
    * @param[out] trq_friction 关节摩擦力
    * @param[out] trq_gravity 重力矩
    */
   void getTorqueWithFriction(const std::array<double, DoF> &jntPos,
                              const std::array<double, DoF> &jntVel,
                              const std::array<double, DoF> &jntAcc,
                              std::array<double, DoF> &trq_full,
                              std::array<double, DoF> &trq_inertia,
                              std::array<double, DoF> &trq_coriolis,
                              std::array<double, DoF> &trq_friction,
                              std::array<double, DoF> &trq_gravity);

   /**
    * @brief 由模型计算无摩擦力的关节力矩, 计算结果单位: Nm。如有负载，先通过setLoad()设置负载参数。
    * @param[in] jntPos 关节角度
    * @param[in] jntVel 关节角速度
    * @param[in] jntAcc 关节角加速度
    * @param[out] trq_full 总关节力矩
    * @param[out] trq_inertia 离心力
    * @param[out] trq_coriolis 科氏力
    * @param[out] trq_gravity 重力矩
    */
   void getTorqueNoFriction(const std::array<double, DoF> &jntPos,
                            const std::array<double, DoF> &jntVel,
                            const std::array<double, DoF> &jntAcc,
                            std::array<double, DoF> &trq_full,
                            std::array<double, DoF> &trq_inertia,
                            std::array<double, DoF> &trq_coriolis,
                            std::array<double, DoF> &trq_gravity);

  XCORESDK_DECLARE_IMPLD
 };

 template <unsigned short DoF>
 using XMateModel = xMateModel<DoF>;

#endif // #ifdef XMATEMODEL_LIB_SUPPORTED

}  // namespace rokae

#endif // ROKAEAPI_MODEL_H
