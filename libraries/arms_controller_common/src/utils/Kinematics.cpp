#include "arms_controller_common/utils/Kinematics.h"
#include <algorithm>
#include <chrono>
#include <iostream>

namespace arms_controller_common
{
    ArmKinematics::ArmKinematics(
        const std::string& urdf_path,
        const std::string& baseFrameName)
    {
        // 从URDF文件加载Pinocchio模型
        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);
        baseFrameName_ = baseFrameName;
        buildMappings();
        // get_joint_names_from_model();
        // extractJointLimits();
    }

    ArmKinematics::ArmKinematics(const pinocchio::Model& model)
        : model_(model)
    {
        data_ = pinocchio::Data(model_);
        buildMappings();
        // get_joint_names_from_model();
        // extractJointLimits();
    }

    void ArmKinematics::initializeFromParameters(
        const std::vector<std::string>& joint_names,
        const std::string& left_ee_name,
        const std::string& right_ee_name)
    {
        setJointNames(joint_names);
        setEndEffectorNames(left_ee_name, right_ee_name);
    }

    ArmKinematics::~ArmKinematics() = default;

    // ==================== 正运动学 ====================
    EndEffectorPose
    ArmKinematics::computeSingleEndEffectorPose(const RobotState& state,
                                                std::string arm_type)
    {
        if (arm_type == "right")
        {
            return computeFramePose(state, rightEndEffectorName_);
        }
        else if (arm_type == "left")
        {
            return computeFramePose(state, leftEndEffectorName_);
        }
        else
        {
            throw std::invalid_argument("Invalid arm type: " + arm_type);
        }
    };

    void ArmKinematics::computeBothEndEffectorPose(const RobotState& state,
                                                   EndEffectorPose& leftPose,
                                                   EndEffectorPose& rightPose)
    {
        Eigen::VectorXd joints = stateToJointVector(state);
        updateKinematics(joints);

        int leftFrameId = getFrameId(leftEndEffectorName_);
        int rightFrameId = getFrameId(rightEndEffectorName_);

        if (leftFrameId >= 0)
        {
            const auto& leftPlacement = data_.oMf[leftFrameId];
            leftPose.position = leftPlacement.translation();
            leftPose.setRotation(leftPlacement.rotation());
        }

        if (rightFrameId >= 0)
        {
            const auto& rightPlacement = data_.oMf[rightFrameId];
            rightPose.position = rightPlacement.translation();
            rightPose.setRotation(rightPlacement.rotation());
        }
    }

    EndEffectorPose
    ArmKinematics::computeFramePose(const RobotState& state,
                                    const std::string& frameName)
    {
        Eigen::VectorXd joints = stateToJointVector(state);
        updateKinematics(joints);

        int frameId = getFrameId(frameName);
        if (frameId < 0)
        {
            throw std::runtime_error("Frame " + frameName + " not found");
        }

        const auto& placement = data_.oMf[frameId];

        EndEffectorPose pose;
        pose.position = placement.translation();
        pose.setRotation(placement.rotation());

        return pose;
    }

    // ==================== 逆运动学 ====================
    // 优化后的单臂逆运动学求解器
    bool ArmKinematics::solveSingleArmIK(const EndEffectorPose& targetPose,
                                         const Eigen::VectorXd& initialGuess,
                                         Eigen::VectorXd& solution,
                                         std::string arm_type,
                                         int maxIterations,
                                         double tolerance,
                                         double damping)
    {
        // 1. 参数验证
        int jointCount = (arm_type == "left") ? leftArmJointCount_ : rightArmJointCount_;
        if (initialGuess.size() != static_cast<Eigen::Index>(jointCount))
        {
            std::cerr << "Initial guess size mismatch: expected " << jointCount
                << ", got " << initialGuess.size() << std::endl;
            return false;
        }

        // 2. 热启动：使用上次求解结果作为初始猜测
        if (lastArmType_ == arm_type)
        {
            if (arm_type == "left" && lastLeftSolution_.size() == jointCount)
            {
                solution = lastLeftSolution_;
            }
            else if (arm_type == "right" && lastRightSolution_.size() == jointCount)
            {
                solution = lastRightSolution_;
            }
            else
            {
                solution = initialGuess;
            }
        }
        else
        {
            solution = initialGuess;
        }

        // 3. 准备工作
        RobotState state(leftArmJointCount_, rightArmJointCount_);
        int frameId;

        if (arm_type == "left")
        {
            state.leftArmJoints = solution;
            frameId = getFrameId(leftEndEffectorName_);
        }
        else
        {
            state.rightArmJoints = solution;
            frameId = getFrameId(rightEndEffectorName_);
        }

        if (frameId < 0)
        {
            std::cerr << "End effector frame not found: "
                << (arm_type == "left" ? leftEndEffectorName_ : rightEndEffectorName_) << std::endl;
            return false;
        }

        // 4. 预分配内存（避免重复分配）
        Eigen::MatrixXd jacobian(6, jointCount);
        Eigen::Matrix < double, 6, 1 > error;
        Eigen::VectorXd deltaQ(jointCount);
        Eigen::VectorXd joints = stateToJointVector(state);

        double previousError = std::numeric_limits<double>::max();
        int stagnationCount = 0;

        // 5. 迭代求解
        for (int iter = 0; iter < maxIterations; ++iter)
        {
            // 更新运动学
            updateKinematics(joints);

            // 获取当前位姿
            const auto& placement = data_.oMf[frameId];
            EndEffectorPose currentPose;
            currentPose.position = placement.translation();
            currentPose.setRotation(placement.rotation());

            // 计算误差
            error = compute6DError(currentPose, targetPose);
            double currentError = error.norm();

            // 收敛检查
            if (currentError < tolerance)
            {
                // 缓存成功解
                if (arm_type == "left")
                {
                    lastLeftSolution_ = solution;
                }
                else
                {
                    lastRightSolution_ = solution;
                }
                lastArmType_ = arm_type;
                std::cout << "IK Iter " << iter << ": error=" << currentError
                    << ", step=" << deltaQ.norm() << std::endl;
                return true;
            }

            // 停滞检测（误差不再减小）
            if (currentError >= previousError * 0.99)
            {
                stagnationCount++;
                if (stagnationCount > 8)
                {
                    // 如果不是从初始猜测开始的，尝试重置到初始猜测
                    if (solution != initialGuess)
                    {
                        solution = initialGuess;
                        stagnationCount = 0;
                        previousError = std::numeric_limits<double>::max();
                        if (arm_type == "left")
                        {
                            state.leftArmJoints = solution;
                        }
                        else
                        {
                            state.rightArmJoints = solution;
                        }
                        joints = stateToJointVector(state);
                        continue;
                    }
                    return false;
                }
            }
            else
            {
                stagnationCount = 0;
            }
            previousError = currentError;

            // 计算雅可比矩阵
            jacobian = getJacbian(arm_type);

            // 自适应阻尼
            double currentDamping = damping;
            if (iter > 5)
            {
                // 前几次迭代使用固定阻尼，之后开始自适应
                currentDamping = computeAdaptiveDamping(jacobian, damping);
            }

            // 求解增量
            deltaQ = dampedLeastSquares(jacobian, error, currentDamping);

            // 限制步长，防止震荡
            limitStepSize(deltaQ, 0.3);

            // 更新解
            solution += deltaQ;

            // 应用关节限位
            applyJointLimits(solution, arm_type);

            // 更新状态
            if (arm_type == "left")
            {
                state.leftArmJoints = solution;
            }
            else
            {
                state.rightArmJoints = solution;
            }
            joints = stateToJointVector(state);

            // 调试输出（可选，每10次迭代打印一次）
            // if (iter % 20 == 0 && iter > 0)
            // {
            //     std::cout << "IK Iter " << iter << ": error=" << currentError
            //         << ", damping=" << currentDamping
            //         << ", step=" << deltaQ.norm() << std::endl;
            // }
        }

        std::cerr << "IK failed to converge after " << maxIterations
            << " iterations, final error: " << previousError << std::endl;
        return false;
    }

    bool ArmKinematics::solveBothArmsIK(const EndEffectorPose& leftTargetPose,
                                        const EndEffectorPose& rightTargetPose,
                                        const Eigen::VectorXd& initialGuess,
                                        Eigen::VectorXd& solution,
                                        int maxIterations, double tolerance, double damping)
    {
        if (initialGuess.size() != static_cast<Eigen::Index>(leftArmJointCount_ + rightArmJointCount_))
            return false;
        solution = initialGuess;
        bool leftArmSuccess = false, rightArmSuccess = false;
        int leftFrameId = getFrameId(leftEndEffectorName_);
        int rightFrameId = getFrameId(rightEndEffectorName_);
        if (leftFrameId < 0 || rightFrameId < 0)
            return false;

        for (int iter = 0; iter < maxIterations; ++iter)
        {
            updateKinematics(solution);
            // 因为同时求解两臂的IK，12✖14的雅可比矩阵太大了，两个一起计算需要很长时间，所以单独求解两臂的IK
            if (!leftArmSuccess)
            {
                const auto& leftPlacement = data_.oMf[leftFrameId];
                EndEffectorPose leftCurrent;
                leftCurrent.position = leftPlacement.translation();
                leftCurrent.setRotation(leftPlacement.rotation());
                Eigen::Matrix < double, 6, 1 > leftError =
                    compute6DError(leftCurrent, leftTargetPose);
                if (leftError.norm() < tolerance)
                {
                    leftArmSuccess = true;
                    if (rightArmSuccess)
                    {
                        return true;
                    }
                }
                Eigen::MatrixXd leftJacobian = getJacbian("left");
                Eigen::VectorXd left_deltaQ = dampedLeastSquares(leftJacobian, leftError, damping);
                Eigen::VectorXd leftSol = solution.head(leftArmJointCount_) + left_deltaQ;
                applyJointLimits(leftSol, "left");
                solution.head(leftArmJointCount_) = leftSol;
            }

            if (!rightArmSuccess)
            {
                const auto& rightPlacement = data_.oMf[rightFrameId];
                EndEffectorPose rightCurrent;
                rightCurrent.position = rightPlacement.translation();
                rightCurrent.setRotation(rightPlacement.rotation());

                Eigen::Matrix < double, 6, 1 > rightError =
                    compute6DError(rightCurrent, rightTargetPose);

                if (rightError.norm() < tolerance)
                {
                    rightArmSuccess = true;
                    if (leftArmSuccess)
                    {
                        return true;
                    }
                }

                Eigen::MatrixXd rightJacobian = getJacbian("right");
                Eigen::VectorXd right_deltaQ =
                    dampedLeastSquares(rightJacobian, rightError, damping);
                Eigen::VectorXd rightSol = solution.tail(rightArmJointCount_) + right_deltaQ;
                applyJointLimits(rightSol, "right");
                solution.tail(rightArmJointCount_) = rightSol;
            }
        }
        return false;
    }


    // 雅可比矩阵计算函数
    Eigen::MatrixXd ArmKinematics::computeJacobian(const RobotState& state,
                                                   const std::string& armType)
    {
        Eigen::VectorXd joints = stateToJointVector(state);
        updateKinematics(joints);
        Eigen::MatrixXd Jacobian = getJacbian(armType);
        // std::cout << armType << "Jacobian : " << Jacobian << std::endl;
        return Jacobian;
    }

    void ArmKinematics::getJointLimits(const std::string& armType,
                                       Eigen::VectorXd& lower,
                                       Eigen::VectorXd& upper) const
    {
        if (armType == "left")
        {
            lower = leftLowerLimits_;
            upper = leftUpperLimits_;
        }
        else if (armType == "right")
        {
            lower = rightLowerLimits_;
            upper = rightUpperLimits_;
        }
    }

    void ArmKinematics::applyJointLimits(Eigen::VectorXd& joints,
                                         const std::string& armType)
    {
        Eigen::VectorXd lower, upper;
        getJointLimits(armType, lower, upper);

        for (int i = 0; i < joints.size(); ++i)
        {
            joints[i] = std::max(lower[i], std::min(upper[i], joints[i]));
        }
    }


    // 一些内部辅助函数的实现
    void ArmKinematics::buildMappings()
    {
        // 建立关节名称到ID的映射
        for (size_t i = 0; i < model_.names.size(); ++i)
        {
            jointNameToId_[model_.names[i]] = static_cast<int>(i);
        }

        // 建立框架名称到ID的映射
        for (int i = 0; i < model_.nframes; ++i)
        {
            frameNameToId_[model_.frames[i].name] = i;
        }
    };

    // void ArmKinematics::get_joint_names_from_model()
    // {
    //     // 定义双臂的前缀
    //     std::string leftPrefix = "left_";
    //     std::string rightPrefix = "right_";
    //
    //     // 清空现有列表
    //     leftArmJointNames_.clear();
    //     rightArmJointNames_.clear();
    //
    //     // 修复：将 int 改为 size_t 或使用 static_cast
    //     for (pinocchio::JointIndex i = 1; i <= static_cast<pinocchio::JointIndex>(model_.njoints); ++i)
    //     {
    //         const auto& jointName = model_.names[i];
    //         const auto& jointModel = model_.joints[i];
    //
    //         // 检查关节是否有自由度（nv() > 0 表示非固定关节）
    //         if (jointModel.nv() > 0)
    //         {
    //             if (jointName.find(leftPrefix) == 0)
    //             {
    //                 leftArmJointNames_.push_back(jointName);
    //             }
    //             else if (jointName.find(rightPrefix) == 0)
    //             {
    //                 rightArmJointNames_.push_back(jointName);
    //             }
    //         }
    //     }
    //
    //     // 更新关节数量
    //     leftArmJointCount_ = leftArmJointNames_.size();
    //     rightArmJointCount_ = rightArmJointNames_.size();
    //
    //     if (leftArmJointCount_ != rightArmJointCount_)
    //     {
    //         std::cerr << "Warning: Left and right arm joint counts do not match! Left: "
    //             << leftArmJointCount_ << ", Right: " << rightArmJointCount_
    //             << std::endl;
    //     }
    //
    //     bool print_joint_names = false; // 设置为true以输出关节名称
    //     if (print_joint_names)
    //     {
    //         std::cout << "Extracted Joint Names from Model:" << std::endl;
    //         std::cout << "Left Arm Joints (" << leftArmJointCount_ << "): ";
    //         for (const auto& name : leftArmJointNames_)
    //         {
    //             std::cout << name << " ";
    //         }
    //         std::cout << std::endl;
    //         std::cout << "Right Arm Joints (" << rightArmJointCount_ << "): ";
    //         for (const auto& name : rightArmJointNames_)
    //         {
    //             std::cout << name << " ";
    //         }
    //         std::cout << std::endl;
    //     }
    // }

    void ArmKinematics::extractJointLimits()
    {
        // 提取左臂关节限位
        leftLowerLimits_.resize(leftArmJointCount_);
        leftUpperLimits_.resize(leftArmJointCount_);

        for (size_t i = 0; i < leftArmJointNames_.size(); ++i)
        {
            int jointId = getJointId(leftArmJointNames_[i]);
            if (jointId >= 0 && jointId < model_.lowerPositionLimit.size())
            {
                leftLowerLimits_[i] = model_.lowerPositionLimit[jointId];
                leftUpperLimits_[i] = model_.upperPositionLimit[jointId];
            }
            else
            {
                leftLowerLimits_[i] = -M_PI;
                leftUpperLimits_[i] = M_PI;
            }
        }

        // 提取右臂关节限位
        rightLowerLimits_.resize(rightArmJointCount_);
        rightUpperLimits_.resize(rightArmJointCount_);

        for (size_t i = 0; i < rightArmJointNames_.size(); ++i)
        {
            int jointId = getJointId(rightArmJointNames_[i]);
            if (jointId >= 0 && jointId < model_.lowerPositionLimit.size())
            {
                rightLowerLimits_[i] = model_.lowerPositionLimit[jointId];
                rightUpperLimits_[i] = model_.upperPositionLimit[jointId];
            }
            else
            {
                rightLowerLimits_[i] = -M_PI;
                rightUpperLimits_[i] = M_PI;
            }
        }
    }

    void ArmKinematics::updateKinematics(const Eigen::VectorXd& jointPositions)
    {
        pinocchio::forwardKinematics(model_, data_, jointPositions);
        pinocchio::updateFramePlacements(model_, data_);
        pinocchio::computeJointJacobians(model_, data_, jointPositions);
    }

    int ArmKinematics::getFrameId(const std::string& frameName) const
    {
        auto it = frameNameToId_.find(frameName);
        if (it != frameNameToId_.end())
        {
            return it->second;
        }
        return -1;
    }

    int ArmKinematics::getJointId(const std::string& jointName) const
    {
        auto it = jointNameToId_.find(jointName);
        if (it != jointNameToId_.end())
        {
            return it->second;
        }
        return -1;
    }

    Eigen::MatrixXd ArmKinematics::getJacbian(std::string armType)
    {
        if (armType == "left")
        {
            int frameId = getFrameId(leftEndEffectorName_);
            if (frameId < 0)
            {
                std::cerr << "Invalid end effector name: " << leftEndEffectorName_
                    << std::endl;
                return Eigen::MatrixXd::Zero(6, leftArmJointCount_);
            }
            Eigen::MatrixXd fullJacobian(6, model_.nv);
            fullJacobian.setZero();
            pinocchio::getFrameJacobian(model_, data_, frameId,
                                        pinocchio::LOCAL_WORLD_ALIGNED, fullJacobian);

            // 提取左臂相关的部分（前7列）
            Eigen::MatrixXd leftJacobian = fullJacobian.leftCols(leftArmJointCount_);
            return leftJacobian;
        }
        else if (armType == "right")
        {
            int frameId = getFrameId(rightEndEffectorName_);
            if (frameId < 0)
            {
                std::cerr << "Invalid end effector name: " << rightEndEffectorName_
                    << std::endl;
                return Eigen::MatrixXd::Zero(6, rightArmJointCount_);
            }
            Eigen::MatrixXd fullJacobian(6, model_.nv);
            fullJacobian.setZero();
            pinocchio::getFrameJacobian(model_, data_, frameId,
                                        pinocchio::LOCAL_WORLD_ALIGNED, fullJacobian);
            // 提取右臂相关的部分（后7列）
            Eigen::MatrixXd rightJacobian = fullJacobian.rightCols(rightArmJointCount_);
            return rightJacobian;
        }
        else if (armType == "both")
        {
            int leftFrameId = getFrameId(leftEndEffectorName_);
            int rightFrameId = getFrameId(rightEndEffectorName_);

            if (leftFrameId < 0 || rightFrameId < 0)
            {
                std::cerr << "Invalid end effector name: " << leftEndEffectorName_
                    << " or " << rightEndEffectorName_ << std::endl;
                return Eigen::MatrixXd::Zero(12, leftArmJointCount_ + rightArmJointCount_);
            }

            Eigen::MatrixXd leftJacobian(6, leftArmJointCount_ + rightArmJointCount_);
            Eigen::MatrixXd rightJacobian(6, leftArmJointCount_ + rightArmJointCount_);
            leftJacobian.setZero();
            rightJacobian.setZero();

            pinocchio::getFrameJacobian(model_, data_, leftFrameId,
                                        pinocchio::LOCAL_WORLD_ALIGNED, leftJacobian);
            pinocchio::getFrameJacobian(model_, data_, rightFrameId,
                                        pinocchio::LOCAL_WORLD_ALIGNED, rightJacobian);

            Eigen::MatrixXd combinedJacobian(12, leftArmJointCount_ + rightArmJointCount_);
            combinedJacobian.topRows(leftArmJointCount_) = leftJacobian;
            combinedJacobian.bottomRows(rightArmJointCount_) = rightJacobian;
            return combinedJacobian;
        }
        else
        {
            std::cerr << "Invalid arm type: " << armType << std::endl;
            return Eigen::MatrixXd();
        }
    }

    Eigen::VectorXd
    ArmKinematics::stateToJointVector(const RobotState& state) const
    {
        if (leftArmJointCount_ != state.leftArmJointCount ||
            rightArmJointCount_ != state.rightArmJointCount)
        {
            std::cerr << "Error: RobotState joint counts do not match Kinematics model!"
                << std::endl;
            return Eigen::VectorXd();
        }

        Eigen::VectorXd joints(leftArmJointCount_ + rightArmJointCount_);
        joints.head(leftArmJointCount_) = state.leftArmJoints;
        joints.tail(rightArmJointCount_) = state.rightArmJoints;
        return joints;
    }

    Eigen::Matrix<double, 6, 1>
    ArmKinematics::compute6DError(const EndEffectorPose& current,
                                  const EndEffectorPose& target) const
    {
        Eigen::Matrix < double, 6, 1 > error;

        // 位置误差
        error.head<3>() = target.position - current.position;

        // 姿态误差（轴角表示）
        Eigen::Matrix3d R_error =
            target.rotationMatrix * current.rotationMatrix.transpose();
        Eigen::AngleAxisd angleAxis(R_error);
        error.tail<3>() = angleAxis.angle() * angleAxis.axis();

        return error;
    }

    Eigen::VectorXd
    ArmKinematics::dampedLeastSquares(const Eigen::MatrixXd& J,
                                      const Eigen::VectorXd& error,
                                      double damping) const
    {
        Eigen::MatrixXd JTJ = J.transpose() * J;
        JTJ.diagonal().array() += damping;

        return JTJ.ldlt().solve(J.transpose() * error);
    }

    geometry_msgs::msg::Pose ArmKinematics::endEffectorPoseToROSPose(const EndEffectorPose& pose)
    {
        geometry_msgs::msg::Pose ros_pose;

        // 设置位置
        ros_pose.position.x = pose.position.x();
        ros_pose.position.y = pose.position.y();
        ros_pose.position.z = pose.position.z();

        // 设置四元数
        ros_pose.orientation.x = pose.quaternion.x();
        ros_pose.orientation.y = pose.quaternion.y();
        ros_pose.orientation.z = pose.quaternion.z();
        ros_pose.orientation.w = pose.quaternion.w();

        return ros_pose;
    }

    EndEffectorPose ArmKinematics::rosPoseToEndEffectorPose(const geometry_msgs::msg::Pose& pose)
    {
        EndEffectorPose end_effector_pose;

        // 设置位置
        end_effector_pose.position.x() = pose.position.x;
        end_effector_pose.position.y() = pose.position.y;
        end_effector_pose.position.z() = pose.position.z;

        // 设置四元数
        Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
                             pose.orientation.y, pose.orientation.z);
        end_effector_pose.setQuaternion(q);

        return end_effector_pose;
    }

    bool ArmKinematics::computeForwardKinematics(
        const std::string& arm_type,
        const std::vector<double>& joint_angles,
        std::vector<EndEffectorPose>& result_poses,
        std::string& error_msg)
    {
        result_poses.clear();

        // 构建机器人状态
        RobotState state(leftArmJointCount_, rightArmJointCount_);

        if (arm_type == "left")
        {
            if (joint_angles.size() != leftArmJointCount_)
            {
                error_msg = "Left arm requires " + std::to_string(leftArmJointCount_) +
                    " joints, got " + std::to_string(joint_angles.size());
                return false;
            }

            for (size_t i = 0; i < leftArmJointCount_; ++i)
            {
                state.leftArmJoints(i) = joint_angles[i];
            }

            EndEffectorPose pose = computeSingleEndEffectorPose(state, "left");
            result_poses.push_back(pose);
        }
        else if (arm_type == "right")
        {
            if (joint_angles.size() != rightArmJointCount_)
            {
                error_msg = "Right arm requires " + std::to_string(rightArmJointCount_) +
                    " joints, got " + std::to_string(joint_angles.size());
                return false;
            }

            for (size_t i = 0; i < rightArmJointCount_; ++i)
            {
                state.rightArmJoints(i) = joint_angles[i];
            }

            EndEffectorPose pose = computeSingleEndEffectorPose(state, "right");
            result_poses.push_back(pose);
        }
        else if (arm_type == "both")
        {
            if (joint_angles.size() != leftArmJointCount_ + rightArmJointCount_)
            {
                error_msg = "Both arms require " + std::to_string(leftArmJointCount_ + rightArmJointCount_) +
                    " joints, got " + std::to_string(joint_angles.size());
                return false;
            }

            for (size_t i = 0; i < leftArmJointCount_; ++i)
            {
                state.leftArmJoints(i) = joint_angles[i];
            }
            for (size_t i = 0; i < rightArmJointCount_; ++i)
            {
                state.rightArmJoints(i) = joint_angles[i];
            }

            EndEffectorPose left_pose, right_pose;
            computeBothEndEffectorPose(state, left_pose, right_pose);
            result_poses.push_back(left_pose);
            result_poses.push_back(right_pose);
        }
        else
        {
            error_msg = "Invalid arm_type: " + arm_type + ". Must be 'left', 'right', or 'both'";
            return false;
        }

        return true;
    }

    bool ArmKinematics::computeInverseKinematics(
        const std::string& arm_type,
        const std::vector<double>& initial_guess,
        const std::vector<EndEffectorPose>& target_poses,
        std::vector<double>& result_joint_angles,
        int max_iterations,
        double tolerance,
        double damping,
        std::string& error_msg)
    {
        result_joint_angles.clear();

        if (arm_type == "left")
        {
            if (target_poses.size() != 1)
            {
                error_msg = "Left arm IK requires exactly 1 target pose";
                return false;
            }

            if (initial_guess.size() != leftArmJointCount_)
            {
                error_msg = "Left arm initial guess requires " + std::to_string(leftArmJointCount_) +
                    " joints, got " + std::to_string(initial_guess.size());
                return false;
            }

            Eigen::VectorXd initial_eigen(leftArmJointCount_);
            for (size_t i = 0; i < leftArmJointCount_; ++i)
            {
                initial_eigen(i) = initial_guess[i];
            }

            Eigen::VectorXd solution;
            bool success = solveSingleArmIK(target_poses[0], initial_eigen, solution,
                                            "left", max_iterations, tolerance, damping);

            if (success)
            {
                result_joint_angles.resize(leftArmJointCount_);
                for (size_t i = 0; i < leftArmJointCount_; ++i)
                {
                    result_joint_angles[i] = solution(i);
                }
            }
            else
            {
                error_msg = "Left arm IK failed to converge";
                return false;
            }
        }
        else if (arm_type == "right")
        {
            if (target_poses.size() != 1)
            {
                error_msg = "Right arm IK requires exactly 1 target pose";
                return false;
            }

            if (initial_guess.size() != rightArmJointCount_)
            {
                error_msg = "Right arm initial guess requires " + std::to_string(rightArmJointCount_) +
                    " joints, got " + std::to_string(initial_guess.size());
                return false;
            }

            Eigen::VectorXd initial_eigen(rightArmJointCount_);
            for (size_t i = 0; i < rightArmJointCount_; ++i)
            {
                initial_eigen(i) = initial_guess[i];
            }

            Eigen::VectorXd solution;
            bool success = solveSingleArmIK(target_poses[0], initial_eigen, solution,
                                            "right", max_iterations, tolerance, damping);

            if (success)
            {
                result_joint_angles.resize(rightArmJointCount_);
                for (size_t i = 0; i < rightArmJointCount_; ++i)
                {
                    result_joint_angles[i] = solution(i);
                }
            }
            else
            {
                error_msg = "Right arm IK failed to converge";
                return false;
            }
        }
        else if (arm_type == "both")
        {
            if (target_poses.size() != 2)
            {
                error_msg = "Both arms IK requires exactly 2 target poses (left, right)";
                return false;
            }

            if (initial_guess.size() != leftArmJointCount_ + rightArmJointCount_)
            {
                error_msg = "Both arms initial guess requires " +
                    std::to_string(leftArmJointCount_ + rightArmJointCount_) +
                    " joints, got " + std::to_string(initial_guess.size());
                return false;
            }

            Eigen::VectorXd initial_eigen(leftArmJointCount_ + rightArmJointCount_);
            for (size_t i = 0; i < initial_guess.size(); ++i)
            {
                initial_eigen(i) = initial_guess[i];
            }

            Eigen::VectorXd solution;
            bool success = solveBothArmsIK(target_poses[0], target_poses[1],
                                           initial_eigen, solution,
                                           max_iterations, tolerance, damping);

            if (success)
            {
                result_joint_angles.resize(leftArmJointCount_ + rightArmJointCount_);
                for (size_t i = 0; i < leftArmJointCount_ + rightArmJointCount_; ++i)
                {
                    result_joint_angles[i] = solution(i);
                }
            }
            else
            {
                error_msg = "Both arms IK failed to converge";
                return false;
            }
        }
        else
        {
            error_msg = "Invalid arm_type: " + arm_type + ". Must be 'left', 'right', or 'both'";
            return false;
        }

        return true;
    }

    void ArmKinematics::handleKinematicsService(
        const std::shared_ptr<arms_ros2_control_msgs::srv::KinematicsService::Request> request,
        const std::shared_ptr<arms_ros2_control_msgs::srv::KinematicsService::Response> response)
    {
        auto start_time = std::chrono::high_resolution_clock::now();

        // 设置默认参数
        int max_iterations = request->max_iterations > 0 ? request->max_iterations : 100;
        double tolerance = request->tolerance > 0 ? request->tolerance : 1e-6;
        double damping = request->damping > 0 ? request->damping : 0.01;

        if (request->operation_type == "fk")
        {
            // 正运动学
            std::vector<EndEffectorPose> result_poses;
            std::string error_msg;

            bool success = computeForwardKinematics(
                request->arm_type,
                request->joint_angles,
                result_poses,
                error_msg);

            if (success)
            {
                response->success = true;
                response->message = "Forward kinematics computed successfully";

                for (const auto& pose : result_poses)
                {
                    response->result_poses.push_back(endEffectorPoseToROSPose(pose));
                }
            }
            else
            {
                response->success = false;
                response->message = error_msg;
            }
        }
        else if (request->operation_type == "ik")
        {
            // 逆运动学
            std::vector<EndEffectorPose> target_poses;
            for (const auto& pose_msg : request->target_poses)
            {
                target_poses.push_back(rosPoseToEndEffectorPose(pose_msg));
            }

            std::vector<double> result_joint_angles;
            std::string error_msg;

            bool success = computeInverseKinematics(
                request->arm_type,
                request->joint_angles, // 这里 joint_angles 作为初始猜测
                target_poses,
                result_joint_angles,
                max_iterations,
                tolerance,
                damping,
                error_msg);

            if (success)
            {
                response->success = true;
                response->message = "Inverse kinematics computed successfully";
                response->result_joint_angles = result_joint_angles;
            }
            else
            {
                response->success = false;
                response->message = error_msg;
            }
        }
        else
        {
            response->success = false;
            response->message = "Invalid operation_type: " + request->operation_type +
                ". Must be 'fk' or 'ik'";
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        response->computation_time_ms = std::chrono::duration<double, std::milli>(
            end_time - start_time).count();
    }

    // 自适应阻尼计算
    double ArmKinematics::computeAdaptiveDamping(const Eigen::MatrixXd& J, double baseDamping) const
    {
        // 基于雅可比条件数调整阻尼
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J);
        double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size() - 1);

        // 条件数越大（接近奇异），阻尼越大
        double condDamping = baseDamping * (1.0 + std::min(10.0, cond / 100.0));

        return std::min(0.1, condDamping); // 限制最大阻尼
    }

    // 限制步长
    void ArmKinematics::limitStepSize(Eigen::VectorXd& deltaQ, double maxStep)
    {
        double norm = deltaQ.norm();
        if (norm > maxStep)
        {
            deltaQ *= (maxStep / norm);
        }
    }
} // namespace arms_controller_common
