/**
 * @file planner.h
 * @brief 路径规划相关功能
 * @copyright Copyright (C) 2024 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef ROKAEAPI_PLANNER_H
#define ROKAEAPI_PLANNER_H

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include "Eigen/Core"
#include "Eigen/Dense"
#include "base.h"
#include "data_types.h"

namespace rokae {

 /// @cond DO_NOT_DOCUMENT
 // forward-declaration
 template <unsigned short DoF>
 class Cobot;
 template <unsigned short DoF>
 class xMateModel;
 /// @endcond

/**
 * @class CartMotionGenerator
 * @brief S速度规划的笛卡尔空间运动。
 * 参考文献: Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
 * (Kogan Page Science Paper edition).
 */
 class XCORE_API CartMotionGenerator {
  public:
   /**
    * @brief 根据关节目标位置和速度系数生成一条轴空间轨迹，可用来回零或到达指定位置。
    * @param[in] speed_factor 速度系数，范围[0, 1].
    * @param[in] s_goal 目标关节角度
    */
   CartMotionGenerator(double speed_factor, double s_goal);
   ~CartMotionGenerator();

   /**
    * @brief 设置笛卡尔空间运动参数
    * @param[in] ds_max 最大速度
    * @param[in] dds_max_start 最大开始加速度
    * @param[in] dds_max_end 最大结束加速度
    */
   void setMax(double ds_max, double dds_max_start, double dds_max_end);

   /**
    * @brief 获得总运动时间
    * @return 运动时间，单位：秒
    */
   double getTime();

   /**
    * @brief 计算时间t时的弧长s
    * @param[in] t 距开始规划的时间间隔，单位：秒
    * @param[out] delta_s_d 计算结果
    * @return false: 运动规划没有结束 | true: 运动规划结束
    */
   bool calculateDesiredValues(double t, double *delta_s_d) const;

   /**
    * @brief 同步当前弧长
    * @param[in] s_init 初始弧长
    */
   void calculateSynchronizedValues(double s_init);

  XCORESDK_DECLARE_IMPL
 };

/**
 * @class JointMotionGenerator
 * @brief S速度规划的轴空间运动。参考文献:
 * Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
 * (Kogan Page Science Paper edition).
 */
 class XCORE_API JointMotionGenerator {
   using VectorNd = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
   using VectorNi = Eigen::Matrix<int, 7, 1, Eigen::ColMajor>;

  public:
   /**
    * @brief 根据关节目标位置和速度系数生成一条轴空间轨迹，可用来回零或到达指定位置。
    * @param[in] speed_factor 速度系数，范围[0, 1]
    * @param[in] q_goal 目标关节角度
    */
   JointMotionGenerator(double speed_factor, std::array<double, 7> q_goal);
   virtual ~JointMotionGenerator();

   /**
    * @brief 设置轴空间运动参数
    * @param[in] dq_max 最大速度
    * @param[in] ddq_max_start 最大开始加速度
    * @param[in] ddq_max_end 最大结束加速度
    */
   void setMax(const std::array<double, 7> &dq_max,
               const std::array<double, 7> &ddq_max_start,
               const std::array<double, 7> &ddq_max_end);

   /**
    * @brief 获得总运动时间
    */
   double getTime();

   /**
    * @brief 计算时间t时的关节角度增量
    * @param[in] t 时间点
    * @param[out] delta_q_d 计算结果
    * @return false: 运动规划没有结束 | true: 运动规划结束
    */
   bool calculateDesiredValues(double t, std::array<double, 7> &delta_q_d) const;

   /**
    * @brief 同步当前轴角度值
    * @param[in] q_init 初始轴角度
    */
   void calculateSynchronizedValues(const std::array<double, 7> &q_init);

  XCORESDK_DECLARE_IMPL
 };

#if defined(XMATEMODEL_LIB_SUPPORTED)
 /**
  * @brief 点位跟随, 点位可以是笛卡尔位姿或轴角度
  * @tparam DoF 轴数
  */
 template <unsigned short DoF>
 class XCORE_API FollowPosition {

  public:
   /**
    * Default constructor. Call init() to setup Robot instance
    */
   FollowPosition();

   /**
    * @param robot rokae::Robot实例
    * @param model rokae::xMateModel实例, 通过robot.model()获取
    * @param[in] endInFlange 末端相对于法兰的位姿
    */
   FollowPosition(Cobot<DoF>& robot,
                  xMateModel<DoF>& model,
                  const Eigen::Transform<double, 3, Eigen::Isometry>& endInFlange = Eigen::Transform<double, 3, Eigen::Isometry>::Identity());

   ~FollowPosition();

    /**
     * @brief 初始化FollowPosition
     * @param robot rokae::Robot实例
     * @param model rokae::xMateModel实例, 通过robot.model()获取
     */
   void init(Cobot<DoF>& robot, XMateModel<DoF>& model);

   /**
    * @brief 开始目标跟随 - 笛卡尔位姿。该接口非阻塞。
    * @param[in] bMe_desire 期望的目标位姿，为末端相对于基坐标系，即TCP位姿.
    */
   void start(const Eigen::Transform<double, 3, Eigen::Isometry>& bMe_desire);

   /**
    * @brief 开始目标跟随 - 轴角度。该接口非阻塞。
    * @param[in] jnt_desire 期望的轴角度
    */
   void start(const std::array<double, DoF> &jnt_desire);

   /**
    * @brief 停止目标跟随
    */
   void stop();

   /**
    * @brief 更新期望的位姿
    * @param[in] bMe_desire 末端相对于基坐标系，即TCP位姿
    */
   void update(const Eigen::Transform<double, 3, Eigen::Isometry>& bMe_desire);

   /**
    * @brief 更新期望的目标轴角度
    * @param[in] jnt_desired 轴角度, 单位: 弧度
    */
   void update(const std::array<double, DoF>& jnt_desired);

   /**
    * @brief 设置速度比例，可在目标跟随的过程中随时调整。
    * @param[in] scale 速度比例，默认为0.5
    */
   void setScale(double scale);

  XCORESDK_DECLARE_IMPL

 };

 #endif // #XMATEMODEL_LIB_SUPPORTED

}  // namespace rokae

#endif //ROKAEAPI_PLANNER_H
