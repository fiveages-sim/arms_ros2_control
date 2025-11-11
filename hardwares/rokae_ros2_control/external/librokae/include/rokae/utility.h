/**
 * @file utility.h
 * @brief 数据类型转换
 * @copyright Copyright (C) 2024 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef ROKAEAPI_INCLUDE_ROKAE_UTILITY_H_
#define ROKAEAPI_INCLUDE_ROKAE_UTILITY_H_

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include "base.h"
#include "robot.h"
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>

namespace rokae {
 /**
  * @class Utils
  * @brief 数据类型转换工具类
  */
 class XCORE_API Utils {

  public:

   /**
    * @brief 度转弧度
    */
   static double degToRad(double degrees) {
     return degrees * (EIGEN_PI / 180);
   }

   /**
    * @brief 弧度转度
    */
   static double radToDeg(double rad) {
     return rad / EIGEN_PI * 180.0;
   }

   /**
    * @brief 数组度转弧度
    */
   template<size_t S>
   static std::array<double, S> degToRad(const std::array<double, S> &degrees) {
     std::array<double, S> ret {};
     std::transform(degrees.cbegin(), degrees.cend(), ret.begin(),
                    [](const double &d) { return degToRad(d);});
     return ret;
   }

   /**
    * @brief 数组度转弧度
    */
   static std::vector<double> degToRad(const std::vector<double> &degrees) {
     std::vector<double> ret {};
     std::transform(degrees.cbegin(), degrees.cend(), std::back_inserter(ret),
                    [](const double &d) { return degToRad(d);});
     return ret;
   }


   /**
    * @brief 数组弧度转度
    */
   template<size_t S>
   static std::array<double, S> radToDeg(const std::array<double, S> &rad) {
     std::array<double, S> ret {};
     std::transform(rad.cbegin(), rad.cend(), ret.begin(),
                    [](const double &d) { return radToDeg(d);});
     return ret;
   }

   /**
    * @brief 数组弧度转度
    */
   static std::vector<double> radToDeg(const std::vector<double> &rad) {
     std::vector<double> ret {};
     std::transform(rad.cbegin(), rad.cend(), std::back_inserter(ret),
                    [](const double &d) { return radToDeg(d);});
     return ret;
   }
   /**
    * @brief 变换矩阵转为数组
    * @param[in] rot 旋转矩阵
    * @param[in] trans 平移向量
    * @param[out] array 转换结果，行优先
    */
   static void transMatrixToArray(const Eigen::Matrix3d& rot, const Eigen::Vector3d& trans, std::array<double, 16>& array) {
     array = {{rot(0, 0), rot(0, 1), rot(0, 2), trans(0),
               rot(1, 0), rot(1, 1), rot(1, 2), trans(1),
               rot(2, 0), rot(2, 1), rot(2, 2), trans(2),
               0,         0,         0,         1       }};
   }

   /**
    * @brief 变换矩阵转为数组
    * @param[in] R 变换矩阵
    * @param[out] array 转换结果，行优先
    */
   static void transMatrixToArray_all(const Eigen::Matrix4d& R ,std::array<double, 16>& array) {
       array = { { R(0,0),R(0,1),R(0,2),R(0,3),
                   R(1,0),R(1,1),R(1,2),R(1,3),
                   R(2,0),R(2,1),R(2,2),R(2,3),
                   R(3,0),R(3,1),R(3,2),R(3,3)}};
   }

   /**
    * @brief 数组转为变换矩阵
    * @param[in] array 数组, 行优先
    * @param[out] rot 旋转矩阵
    * @param[out] trans 平移向量
    */
   static void arrayToTransMatrix(const std::array<double, 16>& array, Eigen::Matrix3d& rot, Eigen::Vector3d& trans) {
     rot << array[0], array[1], array[2], array[4], array[5], array[6], array[8], array[9], array[10];
     trans << array[3], array[7], array[11];
   }

   /**
    * @brief 数组转为变换矩阵
    * @param[in] array 数组, 行优先
    * @param[out] 4*4变换矩阵
    */
   static void arrayToTransMatrix_all(const std::array<double, 16>& array, Eigen::Matrix4d& R) {
       R << array[0], array[1], array[2],array[3], array[4], array[5], array[6],array[7], array[8], array[9], array[10],array[11],array[12],array[13],array[14],array[15];
   }

   /**
    * @brief 将表示位姿的数组{X, Y, Z, Rx, Ry, Rz}转换成行优先齐次变换矩阵
    * @param[in] xyz_abc 输入位姿, {X, Y, Z, Rx, Ry, Rz}
    * @param[out] transMatrix 转换结果
    */
   static void postureToTransArray(const std::array<double, 6> &xyz_abc, std::array<double, 16> &transMatrix) {
     using namespace Eigen;
     Vector3d vec;
     Matrix3d rot;
     rot = (AngleAxisd(xyz_abc[5], Vector3d::UnitZ()) *
       AngleAxisd(xyz_abc[4], Vector3d::UnitY()) *
       AngleAxisd(xyz_abc[3], Vector3d::UnitX())).toRotationMatrix();
     vec << xyz_abc[0], xyz_abc[1], xyz_abc[2];
     Utils::transMatrixToArray(rot, vec, transMatrix);
   }

   /**
    * @brief 将行优先齐次变换矩阵转换成{X, Y, Z, Rx, Ry, Rz}数组
    * @param[in] transMatrix 行优先齐次变换矩阵
    * @param[out]  xyz_abc 转换结果
    */
   static void transArrayToPosture(const std::array<double, 16> &transMatrix, std::array<double, 6> &xyz_abc) {
     xyz_abc[0] = transMatrix[3];
     xyz_abc[1] = transMatrix[7];
     xyz_abc[2] = transMatrix[11];
     xyz_abc[4] = atan2(-transMatrix[8], sqrt(pow(transMatrix[0], 2.0) + pow(transMatrix[4],2.0)));
     if(fabs(xyz_abc[4]) > (M_PI/2.0 - 1E-12)) {
       xyz_abc[5] = atan2(-transMatrix[1], transMatrix[5]);
       xyz_abc[3] = 0.0;
     } else {
       xyz_abc[3] = atan2(transMatrix[9], transMatrix[10]);
       xyz_abc[5] = atan2(transMatrix[4], transMatrix[0]);
     }
   }

   /**
    * @brief 欧拉角转为旋转矩阵
    * @param[in] euler 欧拉角, 顺序[z, y, x], 单位: 弧度
    * @param[out] matrix 旋转矩阵
    */
   static void eulerToMatrix(const Eigen::Vector3d& euler, Eigen::Matrix3d& matrix) {
     Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX()));
     Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()));
     Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()));
     matrix = yawAngle * pitchAngle * rollAngle;
   }

   /**
    * @brief 四元数转欧拉角
    * @param[in] w Q1
    * @param[in] x Q2
    * @param[in] y Q3
    * @param[in] z Q4
    * @return { Rx, Ry, Rz }
    */
   static std::array<double, 3> quaternionToEuler(double w, double x, double y, double z) {
     Eigen::Quaterniond  quaterniond(w, x, y, z);
     Eigen::Vector3d euler = quaterniond.matrix().eulerAngles(2, 1, 0);
     return { euler(2), euler(1), euler(0) };
   }

   /**
    * @brief Toolset转换成工具和工件
    * @param[in] tool_set 工具工件组
    * @param[out] ref_trans 外部坐标系变换矩阵
    * @param[out] end_trans 末端坐标系变换矩阵
    */
   static inline void toolsetCalcPos(const Toolset &tool_set, std::array<double, 16> &ref_trans, std::array<double, 16> &end_trans) {
       std::array<double, 6> ref{}, end{};
       for (int i=0 ; i<3 ; i++)
       {
           ref[i] = tool_set.ref.trans[i];
           end[i] = tool_set.end.trans[i];
       }
	   for (int j = 0; j < 3; j++)
	   {
		   ref[j+3] = tool_set.ref.rpy[j];
		   end[j+3] = tool_set.end.rpy[j];
	   }

       postureToTransArray(end, end_trans);
       postureToTransArray(ref, ref_trans);
   }

   /**
    * @brief 坐标系转换：将 末端相对与外部参考坐标 转换为 法兰相对于基坐标系的坐标
    * @param[in] base_in_world 基坐标系相对世界坐标系设置
    * @param[in] tool_set 工具工件设置
    * @param[in] end_in_ref 末端相对（外部）参考坐标系坐标
    * @return 法兰相对于基坐标系坐标
    */
   static std::array<double, 6> EndInRefToFlanInBase(const std::array<double, 6>& base_in_world, const Toolset &tool_set,
                                                     const std::array<double, 6> &end_in_ref){
	   using namespace Eigen;

       //处理基坐标设置
	   Matrix4d R_BIW;
       std::array<double, 16> BIW{};//基坐标&世界坐标
       postureToTransArray(base_in_world, BIW);//把位姿坐标转换为转换矩阵（数组）
	   arrayToTransMatrix_all( BIW,R_BIW);//把数组转为4*4矩阵

	   //toolset直接处理
       std::array<double, 16> EIF{};//末端&法兰坐标
       std::array<double, 16> RIW{};//参考&世界坐标
	   toolsetCalcPos(tool_set, RIW, EIF);//计算toolset的pos
	   Matrix4d R_EIF;
	   arrayToTransMatrix_all(EIF, R_EIF);//把数组转为4*4矩阵
	   Matrix4d R_RIW;
	   arrayToTransMatrix_all(RIW, R_RIW);//把数组转为4*4矩阵

       //处理输入末端相对参考位姿
       std::array<double, 16> EIR{};//末端&参考
       postureToTransArray(end_in_ref, EIR);//把位姿坐标转换为转换矩阵（数组）
	   Matrix4d R_EIR;
	   arrayToTransMatrix_all(EIR, R_EIR);//把数组转为4*4矩阵

       //计算输出法兰相对基坐标位姿
       std::array<double, 16> FIB{};//法兰&基坐标
       Matrix4d R_FIB;
       R_FIB=R_BIW.inverse() * R_RIW * R_EIR * R_EIF.inverse();//矩阵运算
       transMatrixToArray_all(R_FIB, FIB);//把4*4矩阵转为数组
       std::array<double, 6> flan_in_base{};
       transArrayToPosture(FIB, flan_in_base);//数组转为位姿

       return flan_in_base;
   }

   /**
    * @brief 坐标系转换：将 法兰相对于基坐标系坐标 转换为 末端相对于外部参考系坐标
     * @param[in] base_in_world 基坐标系相对世界坐标系设置
     * @param[in] tool_set 工具工件设置
     * @param[in] flan_in_base 法兰相对于基坐标系坐标
     * @return 末端相对（外部）参考坐标系坐标
     */
   static std::array<double, 6> FlanInBaseToEndInRef(const std::array<double, 6>& base_in_world, const Toolset &tool_set,
                                                     const std::array<double, 6> flan_in_base) {
       using namespace Eigen;

       //处理基坐标设置
       Matrix4d R_BIW;
       std::array<double, 16> BIW{};//基坐标&世界坐标
       postureToTransArray(base_in_world, BIW);//把位姿坐标转换为转换矩阵（数组）
       arrayToTransMatrix_all(BIW, R_BIW);//把数组转为4*4矩阵

       //toolset直接处理
       std::array<double, 16> EIF{};//末端&法兰坐标
       std::array<double, 16> RIW{};//参考&世界坐标
       toolsetCalcPos(tool_set, RIW, EIF);//计算toolset的pos
	   Matrix4d R_EIF;
	   arrayToTransMatrix_all(EIF, R_EIF);//把数组转为4*4矩阵
       Matrix4d R_RIW;
       arrayToTransMatrix_all(RIW, R_RIW);//把数组转为4*4矩阵

       //处理输入法兰相对基坐标位姿
       std::array<double, 16> FIB{};//末端&参考
       postureToTransArray(flan_in_base, FIB);//把位姿坐标转换为转换矩阵（数组）
       Matrix4d R_FIB;
       arrayToTransMatrix_all(FIB, R_FIB);//把数组转为4*4矩阵

       //计算输出末端相对参考位姿
       std::array<double, 16> EIR{};//法兰&基坐标
       Matrix4d R_EIR;
       R_EIR = R_RIW.inverse() * R_BIW * R_FIB * R_EIF;//矩阵运算
       transMatrixToArray_all(R_EIR, EIR);//把4*4矩阵转为数组
       std::array<double, 6> end_in_ref{};
       transArrayToPosture(EIR, end_in_ref);//数组转为位姿

       return end_in_ref;
   }

   Utils() = delete;
   ~Utils() = delete;
 };

}  // namespace rokae

#endif //ROKAEAPI_INCLUDE_ROKAE_UTILITY_H_
