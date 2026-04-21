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

namespace arms_controller_common
{
    // 末端执行器位姿结构体
    struct EndEffectorPose
    {
        Eigen::Vector3d position; // 位置 [x, y, z]
        Eigen::Matrix3d rotationMatrix; // 旋转矩阵
        Eigen::Quaterniond quaternion; // 四元数

        EndEffectorPose()
        {
            position.setZero();
            rotationMatrix.setIdentity();
            quaternion.setIdentity();
        }

        // 从旋转矩阵设置四元数
        void setRotation(const Eigen::Matrix3d& R)
        {
            rotationMatrix = R;
            quaternion = Eigen::Quaterniond(R);
        }

        // 从四元数设置旋转矩阵
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

        // 获取所有关节（左臂+右臂）
        Eigen::VectorXd getAllJoints() const
        {
            Eigen::VectorXd all(leftArmJointCount + rightArmJointCount);
            all.head(leftArmJointCount) = leftArmJoints;
            all.tail(rightArmJointCount) = rightArmJoints;
            return all;
        }

        // 设置所有关节
        void setAllJoints(const Eigen::VectorXd& joints)
        {
            if (joints.size() ==
                static_cast<Eigen::Index>(leftArmJointCount + rightArmJointCount))
            {
                leftArmJoints = joints.head(leftArmJointCount);
                rightArmJoints = joints.tail(rightArmJointCount);
            }
        }
    };

    class ArmKinematics
    {
    public:
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

        // ==================== 逆运动学 ====================

        struct SolverParams
        {
            int maxIterations = 1500;
            double maxTime = 10.0; // 秒
            double gradientTolerance = 1e-7;
            double solutionTolerance = 1e-6;
            double armijoBeta = 0.4;
            double armijoSigma = 1e-5;
            double stepTolerance = 1e-14;
            bool constraintsOn = true;
            bool randomRestart = true;
            int maxRestarts = 10;
        };

        struct SolutionInfo
        {
            int iterations = 0;
            int numRandomRestarts = 0;
            double poseErrorNorm = 0.0;
            int exitFlag = 0;
            std::string status;
        };

        /**
         * @brief 单臂数值逆运动学（BFGS梯度投影法）
         * @param targetPose 目标末端位姿
         * @param initialGuess 初始关节角度猜测
         * @param solution 输出的关节角度
         * @param arm_type 手臂类型（"left"或"right"）
         * @param maxIterations 最大迭代次数
         * @param tolerance 容差
         * @return 是否成功求解
         */
        bool solveSingleArmIK(const EndEffectorPose& targetPose,
                              const Eigen::VectorXd& initialGuess,
                              Eigen::VectorXd& solution, std::string arm_type,
                              int maxIterations = 1500, double tolerance = 1e-6);

        void setWeight(const Eigen::VectorXd& weight) { weight_ = weight; }

        void setSolverParams(const SolverParams& params) { params_ = params; }
        SolverParams getSolverParams() const { return params_; }

        /**
         * @brief 双臂协同逆运动学
         */
        bool solveBothArmsIK(const EndEffectorPose& leftTargetPose,
                             const EndEffectorPose& rightTargetPose,
                             const Eigen::VectorXd& initialGuess,
                             Eigen::VectorXd& solution, int maxIterations = 1500,
                             double tolerance = 1e-6);

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

        /**
         * @brief 转换 EndEffectorPose 到 ROS Pose 消息
         */
        geometry_msgs::msg::Pose endEffectorPoseToROSPose(const EndEffectorPose& pose);

        /**
         * @brief 转换 ROS Pose 消息到 EndEffectorPose
         */
        EndEffectorPose rosPoseToEndEffectorPose(const geometry_msgs::msg::Pose& pose);
        // 新增：从参数接口初始化的方法
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

        // 计算6D误差（角度轴表示）
        Eigen::Matrix<double, 6, 1>
        compute6DError(const EndEffectorPose& current,
                       const EndEffectorPose& target) const;

        // 代价函数和梯度计算
        void computeCostAndGrad(const Eigen::VectorXd& error,
                                const Eigen::MatrixXd& J,
                                const Eigen::VectorXd& weight,
                                Eigen::VectorXd& grad,
                                double* cost);

        // 矩阵正定性检查
        bool isPositiveDefinite(const Eigen::MatrixXd& matrix);

        // 随机配置生成
        Eigen::VectorXd randomConfig(const Eigen::VectorXd& lower,
                                     const Eigen::VectorXd& upper);

        // 约束边界clamp
        Eigen::VectorXd clampToLimits(const Eigen::VectorXd& q,
                                      const Eigen::VectorXd& lower,
                                      const Eigen::VectorXd& upper);

        // BFGS Hessian更新
        void updateHessian(Eigen::MatrixXd& H, const Eigen::VectorXd& s,
                           const Eigen::VectorXd& y);

        // Hessian投影到激活约束
        void projectHessianToConstraints(Eigen::MatrixXd& H,
                                         const Eigen::MatrixXd& activeConstraints);

        // 构建约束矩阵 A*x <= b
        void buildConstraintMatrix(Eigen::MatrixXd& A, Eigen::VectorXd& b,
                                   const Eigen::VectorXd& lower,
                                   const Eigen::VectorXd& upper);

        // 识别激活约束
        void identifyActiveConstraints(const Eigen::VectorXd& x,
                                       const Eigen::MatrixXd& A,
                                       const Eigen::VectorXd& b,
                                       Eigen::VectorXi& activeSet,
                                       Eigen::MatrixXd& activeConstraints);

        // 检查是否达到局部极小
        bool isLocalMinimum(const Eigen::VectorXd& Hg, const Eigen::VectorXd& grad,
                            const Eigen::MatrixXd& activeConstraints,
                            const Eigen::VectorXd& x, const Eigen::MatrixXd& A,
                            const Eigen::VectorXd& b);

        // 管理约束（放弃不再需要的约束）
        void manageConstraints(Eigen::MatrixXd& H, const Eigen::VectorXd& grad,
                               Eigen::MatrixXd& activeConstraints,
                               Eigen::VectorXi& activeSet, const Eigen::MatrixXd& A,
                               const Eigen::VectorXd& b, const Eigen::VectorXd& x);

        // 计算步长上限
        double computeStepBound(const Eigen::VectorXd& x, const Eigen::VectorXd& s,
                                const Eigen::MatrixXd& A, const Eigen::VectorXd& b,
                                const Eigen::VectorXi& activeSet);

        // 带步长限制的线搜索
        double lineSearchWithBound(const Eigen::VectorXd& x, const Eigen::VectorXd& s,
                                   double cost, const Eigen::VectorXd& grad,
                                   double maxGamma, std::string arm_type,
                                   const EndEffectorPose& target);

        // 内部求解器
        std::pair<Eigen::VectorXd, SolutionInfo>
        solveInternal(const Eigen::VectorXd& seed, const Eigen::VectorXd& lower,
                      const Eigen::VectorXd& upper, std::string arm_type,
                      const EndEffectorPose& target);

        // 主求解入口（带随机重启）
        std::pair<Eigen::VectorXd, SolutionInfo>
        solve(const Eigen::VectorXd& initialGuess,
              const Eigen::VectorXd& lower, const Eigen::VectorXd& upper,
              std::string arm_type, const EndEffectorPose& target);
    };
} // namespace arms_controller_common
