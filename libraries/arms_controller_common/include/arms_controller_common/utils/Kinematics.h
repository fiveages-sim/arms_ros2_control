#pragma once
#include <Eigen/Dense>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <random>
#include <string>
#include <vector>
#include <chrono>
#include <geometry_msgs/msg/pose.hpp>
#include "arms_ros2_control_msgs/srv/kinematics_service.hpp"

extern "C" {
#include "FxRobot.h"
}

namespace arms_controller_common
{
    // 末端执行器位姿结构体
    struct EndEffectorPose
    {
        Eigen::Vector3d position;
        Eigen::Matrix3d rotationMatrix;
        Eigen::Quaterniond quaternion;

        EndEffectorPose()
        {
            position.setZero();
            rotationMatrix.setIdentity();
            quaternion.setIdentity();
        }

        void setRotation(const Eigen::Matrix3d& R)
        {
            rotationMatrix = R;
            quaternion = Eigen::Quaterniond(R);
        }

        void setQuaternion(const Eigen::Quaterniond& q)
        {
            quaternion = q;
            rotationMatrix = q.toRotationMatrix();
        }
    };

    // 机器人状态结构体
    struct RobotState
    {
        size_t leftArmJointCount = 7;
        size_t rightArmJointCount = 7;
        Eigen::VectorXd leftArmJoints;
        Eigen::VectorXd rightArmJoints;

        RobotState(size_t left_count = 7, size_t right_count = 7)
            : leftArmJointCount(left_count), rightArmJointCount(right_count),
              leftArmJoints(left_count), rightArmJoints(right_count)
        {
            leftArmJoints.setZero();
            rightArmJoints.setZero();
        }

        Eigen::VectorXd getAllJoints() const
        {
            Eigen::VectorXd all(leftArmJointCount + rightArmJointCount);
            all.head(leftArmJointCount) = leftArmJoints;
            all.tail(rightArmJointCount) = rightArmJoints;
            return all;
        }

        void setAllJoints(const Eigen::VectorXd& joints)
        {
            if (joints.size() == static_cast<Eigen::Index>(leftArmJointCount + rightArmJointCount))
            {
                leftArmJoints = joints.head(leftArmJointCount);
                rightArmJoints = joints.tail(rightArmJointCount);
            }
        }
    };

    class ArmKinematics
    {
    public:
        // 求解器类型枚举
        enum class SolverType
        {
            BFGS, // BFGS 梯度投影法（适合复杂约束、冗余机械臂）
            DLS, // 阻尼最小二乘法（适合非冗余、速度快）
            SDK, //新增天机官方sdk，先这么写，后面在考虑好的代码结构
            AUTO // 自动选择（先尝试 DLS，失败后使用 BFGS）
        };

        ArmKinematics(const std::string& urdf_path,
                      const std::string& baseFrameName = "base_link");
        ArmKinematics(const pinocchio::Model& model);
        ~ArmKinematics();

        // ==================== 正运动学 ====================

        /**
         * @brief 计算单臂末端执行器的位姿
         * @param state 机器人状态
         * @param arm_type 手臂类型 ("left" 或 "right")
         * @return 末端位姿（在基坐标系中）
         */
        EndEffectorPose computeSingleEndEffectorPose(const RobotState& state,
                                                     std::string arm_type);

        /**
         * @brief 计算双臂末端执行器位姿
         * @param state 机器人状态
         * @param leftPose 左臂位姿输出
         * @param rightPose 右臂位姿输出
         */
        void computeBothEndEffectorPose(const RobotState& state,
                                        EndEffectorPose& leftPose,
                                        EndEffectorPose& rightPose);

        /**
         * @brief 计算任意框架的位姿
         * @param state 机器人状态
         * @param frameName 框架名称
         * @return 框架位姿
         */
        EndEffectorPose computeFramePose(const RobotState& state,
                                         const std::string& frameName);

        /**
 * @brief 使用SDK计算正运动学（直接得到TCP位姿）
 * @param joint_angles 关节角度（弧度）
 * @param arm_type 手臂类型 ("left" 或 "right")
 * @param pose 输出的TCP位姿
 * @return 是否成功
 */
        bool computeForwardKinematicsWithSDK(const Eigen::VectorXd& joint_angles,
                                             const std::string& arm_type,
                                             EndEffectorPose& pose);

        // ==================== 逆运动学 ====================

        struct SolverParams
        {
            // 通用参数
            SolverType solverType = SolverType::AUTO;
            int maxIterations = 1500;
            double solutionTolerance = 1e-4;

            // BFGS 专用参数
            double gradientTolerance = 1e-7;
            double armijoBeta = 0.4;
            double armijoSigma = 1e-5;
            double stepTolerance = 1e-14;
            bool constraintsOn = true;
            bool randomRestart = false; // 默认关闭，DLS 不需要
            int maxRestarts = 3;
            double lambdaRegularization = 0.001;
            bool useRegularization = false; // 默认关闭

            // DLS 专用参数
            double dlsDamping = 0.01;
            double dlsStepLimit = 0.3;
            int dlsStagnationLimit = 8;

            //sdk的参数
            double dgr1 = 0.05; // 奇异鲁棒参数1
            double dgr2 = 0.05; // 奇异鲁棒参数2
            double dgr3 = 0.0; // 奇异鲁棒参数3
            bool use_nsp = false; // 是否使用零空间约束
            double nsp_angle = 0.0; // 零空间角度调整
            Eigen::Vector3d nsp_direction; // 零空间方向（当use_nsp=true时）
        };

        // 新增SDK专用参数结构


        struct SolutionInfo
        {
            int iterations = 0;
            int numRandomRestarts = 0;
            double poseErrorNorm = 0.0;
            int exitFlag = 0;
            std::string status;
            SolverType usedSolver = SolverType::AUTO;
        };

        bool solveSingleArmIK(const EndEffectorPose& targetPose,
                              const Eigen::VectorXd& initialGuess,
                              Eigen::VectorXd& solution,
                              std::string arm_type,
                              int maxIterations = 1500,
                              double tolerance = 1e-4);

        bool solveSingleArmIKWithInfo(const EndEffectorPose& targetPose,
                                      const Eigen::VectorXd& initialGuess,
                                      Eigen::VectorXd& solution,
                                      SolutionInfo& info,
                                      std::string arm_type,
                                      int maxIterations = 1500,
                                      double tolerance = 1e-4);

        // 使用TJ_FX_ROBOT_CONTROL_SDK的逆运动学
        bool solveSingleArmIKWithSDK(const EndEffectorPose& targetPose,
                                     const Eigen::VectorXd& initialGuess,
                                     Eigen::VectorXd& solution,
                                     std::string arm_type);

        void setWeight(const Eigen::VectorXd& weight) { weight_ = weight; }
        void setSolverParams(const SolverParams& params) { params_ = params; }
        SolverParams getSolverParams() const { return params_; }
        void setSolverType(SolverType type) { params_.solverType = type; }

        bool solveBothArmsIK(const EndEffectorPose& leftTargetPose,
                             const EndEffectorPose& rightTargetPose,
                             const Eigen::VectorXd& initialGuess,
                             Eigen::VectorXd& solution,
                             int maxIterations = 1500,
                             double tolerance = 1e-4);

        // ==================== 雅可比矩阵计算 ====================
        /**
         * @brief 计算雅可比矩阵
         * @param state 机器人状态(双臂的关节角度）
         * @param armType 手臂类型（"left"、"right"或"both"）
         * @return 单臂（6×7 雅可比矩阵），双臂(12×14 雅可比矩阵)
         */
        Eigen::MatrixXd computeJacobian(const RobotState& state,
                                        const std::string& armType);

        // ==================== 辅助函数 ====================

        /**
         * @brief 获取关节限位
         * @param armType 手臂类型
         * @param lower 下限输出
         * @param upper 上限输出
         */
        void getJointLimits(const std::string& armType, Eigen::VectorXd& lower,
                            Eigen::VectorXd& upper) const;

        /**
         * @brief 应用关节限位
         * @param joints 关节角度
         * @param armType 手臂类型
         */
        void applyJointLimits(Eigen::VectorXd& joints, const std::string& armType);

        /**
         * @brief 获取Pinocchio模型（用于调试）
         */
        const pinocchio::Model& getModel() const { return model_; }

        size_t getLeftArmJointCount() const { return leftArmJointCount_; }
        size_t getRightArmJointCount() const { return rightArmJointCount_; }

        std::vector<std::string> getLeftArmJointNames() const
        {
            return leftArmJointNames_;
        }

        std::vector<std::string> getRightArmJointNames() const
        {
            return rightArmJointNames_;
        }

        /**
         * @brief 处理运动学服务请求
         * @param request 服务请求
         * @param response 服务响应
         */
        void handleKinematicsService(
            const std::shared_ptr<arms_ros2_control_msgs::srv::KinematicsService::Request> request,
            const std::shared_ptr<arms_ros2_control_msgs::srv::KinematicsService::Response> response);

        /**
         * @brief 执行正运动学计算
         */
        bool computeForwardKinematics(
            const std::string& arm_type,
            const std::vector<double>& joint_angles,
            std::vector<EndEffectorPose>& result_poses,
            std::string& error_msg);

        /**
         * @brief 执行逆运动学计算
         */
        bool computeInverseKinematics(
            const std::string& arm_type,
            const std::vector<double>& initial_guess,
            const std::vector<EndEffectorPose>& target_poses,
            std::vector<double>& result_joint_angles,
            int max_iterations,
            double tolerance,
            std::string& error_msg);

        geometry_msgs::msg::Pose endEffectorPoseToROSPose(const EndEffectorPose& pose);
        EndEffectorPose rosPoseToEndEffectorPose(const geometry_msgs::msg::Pose& pose);

        void initializeFromParameters(
            const std::vector<std::string>& joint_names,
            const std::string& left_ee_name = "left_tcp",
            const std::string& right_ee_name = "right_tcp");

        void setEndEffectorNames(const std::string& left_name,
                                 const std::string& right_name)
        {
            leftEndEffectorName_ = left_name;
            rightEndEffectorName_ = right_name;
        }

        void setJointNames(const std::vector<std::string>& joint_names);

        Eigen::MatrixXd getJacobian(std::string armType);

    private:
        // Pinocchio模型和数据
        pinocchio::Model model_;
        mutable pinocchio::Data data_;

        // TJ_FX_ROBOT_CONTROL_SDK 相关
        int robotSerial_; // SDK机器人序列号

        // 框架名称
        std::string baseFrameName_;
        std::string leftEndEffectorName_ = "left_tcp";
        std::string rightEndEffectorName_ = "right_tcp";

        // 关节名称
        std::vector<std::string> leftArmJointNames_;
        std::vector<std::string> rightArmJointNames_;

        // 关节ID映射
        std::map<std::string, int> jointNameToId_;
        std::map<std::string, int> frameNameToId_;

        // 关节数量
        size_t leftArmJointCount_ = 7;
        size_t rightArmJointCount_ = 7;

        // 关节限位
        Eigen::VectorXd leftLowerLimits_;
        Eigen::VectorXd leftUpperLimits_;
        Eigen::VectorXd rightLowerLimits_;
        Eigen::VectorXd rightUpperLimits_;

        // 求解器参数
        SolverParams params_;
        Eigen::VectorXd weight_;

        // 内部辅助函数
        void buildMappings();
        void extractJointLimits();
        void updateKinematics(const Eigen::VectorXd& jointPositions);

        int getFrameId(const std::string& frameName) const;
        int getJointId(const std::string& jointName) const;

        Eigen::VectorXd stateToJointVector(const RobotState& state) const;
        RobotState jointVectorToState(const Eigen::VectorXd& joint, std::string name);

        Eigen::Matrix<double, 6, 1>
        compute6DError(const EndEffectorPose& current,
                       const EndEffectorPose& target) const;

        // DLS 求解器

        // 缓存
        mutable Eigen::VectorXd lastLeftSolution_;
        mutable Eigen::VectorXd lastRightSolution_;
        mutable std::string lastArmType_;
        std::pair<Eigen::VectorXd, SolutionInfo>
        solveDLS(const Eigen::VectorXd& seed,
                 const Eigen::VectorXd& lower,
                 const Eigen::VectorXd& upper,
                 std::string arm_type,
                 const EndEffectorPose& target);

        Eigen::VectorXd dampedLeastSquares(const Eigen::MatrixXd& J,
                                           const Eigen::VectorXd& error,
                                           double damping = 0.01) const;

        double computeAdaptiveDamping(const Eigen::MatrixXd& J, double baseDamping) const;
        void limitStepSize(Eigen::VectorXd& deltaQ, double maxStep = 0.3);

        // BFGS 求解器
        void computeCostAndGrad(const Eigen::VectorXd& error,
                                const Eigen::MatrixXd& J,
                                const Eigen::VectorXd& weight,
                                Eigen::VectorXd& grad,
                                double* cost);

        void computeCostAndGradWithReg(const Eigen::VectorXd& error,
                                       const Eigen::MatrixXd& J,
                                       const Eigen::VectorXd& weight,
                                       Eigen::VectorXd& grad,
                                       double* cost,
                                       const Eigen::VectorXd& q_ref,
                                       const Eigen::VectorXd& q,
                                       double lambda_reg);

        bool isPositiveDefinite(const Eigen::MatrixXd& matrix);
        Eigen::VectorXd randomConfig(const Eigen::VectorXd& lower,
                                     const Eigen::VectorXd& upper);
        Eigen::VectorXd clampToLimits(const Eigen::VectorXd& q,
                                      const Eigen::VectorXd& lower,
                                      const Eigen::VectorXd& upper);

        void updateHessian(Eigen::MatrixXd& H, const Eigen::VectorXd& s,
                           const Eigen::VectorXd& y);

        void projectHessianToConstraints(Eigen::MatrixXd& H,
                                         const Eigen::MatrixXd& activeConstraints);

        void buildConstraintMatrix(Eigen::MatrixXd& A, Eigen::VectorXd& b,
                                   const Eigen::VectorXd& lower,
                                   const Eigen::VectorXd& upper);

        void identifyActiveConstraints(const Eigen::VectorXd& x,
                                       const Eigen::MatrixXd& A,
                                       const Eigen::VectorXd& b,
                                       Eigen::VectorXi& activeSet,
                                       Eigen::MatrixXd& activeConstraints);

        bool isLocalMinimum(const Eigen::VectorXd& Hg, const Eigen::VectorXd& grad,
                            const Eigen::MatrixXd& activeConstraints,
                            const Eigen::VectorXd& x, const Eigen::MatrixXd& A,
                            const Eigen::VectorXd& b);

        void manageConstraints(Eigen::MatrixXd& H, const Eigen::VectorXd& grad,
                               Eigen::MatrixXd& activeConstraints,
                               Eigen::VectorXi& activeSet, const Eigen::MatrixXd& A,
                               const Eigen::VectorXd& b, const Eigen::VectorXd& x);

        double computeStepBound(const Eigen::VectorXd& x, const Eigen::VectorXd& s,
                                const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                                const Eigen::VectorXi& activeSet);

        double lineSearchWithBound(const Eigen::VectorXd& x, const Eigen::VectorXd& s,
                                   double cost, const Eigen::VectorXd& grad,
                                   double maxGamma, std::string arm_type,
                                   const EndEffectorPose& target,
                                   const Eigen::VectorXd& q_ref);

        std::pair<Eigen::VectorXd, SolutionInfo>
        solveBFGS(const Eigen::VectorXd& seed,
                  const Eigen::VectorXd& lower,
                  const Eigen::VectorXd& upper,
                  std::string arm_type,
                  const EndEffectorPose& target);

        std::pair<Eigen::VectorXd, SolutionInfo>
        solveBFGSInternal(const Eigen::VectorXd& seed,
                          const Eigen::VectorXd& lower,
                          const Eigen::VectorXd& upper,
                          std::string arm_type,
                          const EndEffectorPose& target);
    };
} // namespace arms_controller_common
