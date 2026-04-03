#include "arms_controller_common/utils/Kinematics.h"
#include <algorithm>
#include <chrono>
#include <iostream>

namespace arms_controller_common
{
    M6CCSKinematics::M6CCSKinematics(
        const std::string& urdf_path,
        const std::string& baseFrameName)
    {
        // 从URDF文件加载Pinocchio模型
        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);
        baseFrameName_ = baseFrameName;
        buildMappings();
        get_joint_names_from_model();
        extractJointLimits();
    }

    M6CCSKinematics::M6CCSKinematics(const pinocchio::Model& model)
        : model_(model)
    {
        data_ = pinocchio::Data(model_);
        buildMappings();
        get_joint_names_from_model();
        extractJointLimits();
    }

    M6CCSKinematics::~M6CCSKinematics() = default;

    // ==================== 正运动学 ====================
    EndEffectorPose
    M6CCSKinematics::computeSingleEndEffectorPose(const RobotState& state,
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

    void M6CCSKinematics::computeBothEndEffectorPose(const RobotState& state,
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
    M6CCSKinematics::computeFramePose(const RobotState& state,
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
    bool M6CCSKinematics::solveSingleArmIK(const EndEffectorPose& targetPose,
                                           const Eigen::VectorXd& initialGuess,
                                           Eigen::VectorXd& solution,
                                           std::string arm_type, int maxIterations,
                                           double tolerance)
    {
        solution = initialGuess;
        RobotState state(leftArmJointCount_, rightArmJointCount_);
        int frameId;
        if (arm_type == "left")
        {
            if (initialGuess.size() != static_cast<Eigen::Index>(leftArmJointCount_))
                return false;
            state.leftArmJoints = solution;
            state.rightArmJoints.setZero();
            frameId = getFrameId(leftEndEffectorName_);
            if (frameId < 0)
                return false;
        }
        else if (arm_type == "right")
        {
            if (initialGuess.size() != static_cast<Eigen::Index>(rightArmJointCount_))
                return false;
            state.leftArmJoints.setZero();
            state.rightArmJoints = solution;
            frameId = getFrameId(rightEndEffectorName_);
            if (frameId < 0)
                return false;
        }
        else
        {
            std::cerr << "Invalid arm type: " << arm_type << std::endl;
            return false;
        }

        for (int iter = 0; iter < maxIterations; ++iter)
        {
            Eigen::VectorXd joints = stateToJointVector(state);
            updateKinematics(joints);

            const auto& placement = data_.oMf[frameId];
            EndEffectorPose currentPose;
            currentPose.position = placement.translation();
            currentPose.setRotation(placement.rotation());

            Eigen::Matrix < double, 6, 1 > error = compute6DError(currentPose, targetPose);
            if (error.norm() < tolerance)
                return true;

            Eigen::MatrixXd jacobian = getJacbian(arm_type);

            Eigen::VectorXd deltaQ = dampedLeastSquares(jacobian, error);
            solution += deltaQ;
            applyJointLimits(solution, arm_type);
            if (arm_type == "left")
            {
                state.leftArmJoints = solution;
            }
            else
            {
                state.rightArmJoints = solution;
            }
        }
        return false;
    }

    bool M6CCSKinematics::solveBothArmsIK(const EndEffectorPose& leftTargetPose,
                                          const EndEffectorPose& rightTargetPose,
                                          const Eigen::VectorXd& initialGuess,
                                          Eigen::VectorXd& solution,
                                          int maxIterations, double tolerance)
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
                Eigen::VectorXd left_deltaQ = dampedLeastSquares(leftJacobian, leftError);
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
                    dampedLeastSquares(rightJacobian, rightError);
                Eigen::VectorXd rightSol = solution.tail(rightArmJointCount_) + right_deltaQ;
                applyJointLimits(rightSol, "right");
                solution.tail(rightArmJointCount_) = rightSol;
            }
        }
        return false;
    }


    // 雅可比矩阵计算函数
    Eigen::MatrixXd M6CCSKinematics::computeJacobian(const RobotState& state,
                                                     const std::string& armType)
    {
        Eigen::VectorXd joints = stateToJointVector(state);
        updateKinematics(joints);
        Eigen::MatrixXd Jacobian = getJacbian(armType);
        // std::cout << armType << "Jacobian : " << Jacobian << std::endl;
        return Jacobian;
    }

    void M6CCSKinematics::getJointLimits(const std::string& armType,
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

    void M6CCSKinematics::applyJointLimits(Eigen::VectorXd& joints,
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
    void M6CCSKinematics::buildMappings()
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

    void M6CCSKinematics::get_joint_names_from_model()
    {
        // 定义双臂的前缀
        std::string leftPrefix = "left_";
        std::string rightPrefix = "right_";
        // 遍历所有关节名
        for (size_t i = 0; i < model_.names.size(); ++i)
        {
            const auto& jointName = model_.names[i];

            if (jointName.find(leftPrefix) == 0)
            {
                leftArmJointNames_.push_back(jointName);
            }
            else if (jointName.find(rightPrefix) == 0)
            {
                rightArmJointNames_.push_back(jointName);
            }
        }

        // 更新关节数量
        leftArmJointCount_ = leftArmJointNames_.size();
        rightArmJointCount_ = rightArmJointNames_.size();

        if (leftArmJointCount_ != rightArmJointCount_)
        {
            std::cerr << "Warning: Left and right arm joint counts do not match! Left: "
                << leftArmJointCount_ << ", Right: " << rightArmJointCount_
                << std::endl;
        }

        bool print_joint_names = false; // 设置为true以输出关节名称
        if (print_joint_names)
        {
            std::cout << "Extracted Joint Names from Model:" << std::endl;
            std::cout << "Left Arm Joints: ";
            for (const auto& name : leftArmJointNames_)
            {
                std::cout << name << " ";
            }
            std::cout << std::endl;
            std::cout << "Right Arm Joints: ";
            for (const auto& name : rightArmJointNames_)
            {
                std::cout << name << " ";
            }
            std::cout << std::endl;
        }
    };

    void M6CCSKinematics::extractJointLimits()
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

    void M6CCSKinematics::updateKinematics(const Eigen::VectorXd& jointPositions)
    {
        pinocchio::forwardKinematics(model_, data_, jointPositions);
        pinocchio::updateFramePlacements(model_, data_);
        pinocchio::computeJointJacobians(model_, data_, jointPositions);
    }

    int M6CCSKinematics::getFrameId(const std::string& frameName) const
    {
        auto it = frameNameToId_.find(frameName);
        if (it != frameNameToId_.end())
        {
            return it->second;
        }
        return -1;
    }

    int M6CCSKinematics::getJointId(const std::string& jointName) const
    {
        auto it = jointNameToId_.find(jointName);
        if (it != jointNameToId_.end())
        {
            return it->second;
        }
        return -1;
    }

    Eigen::MatrixXd M6CCSKinematics::getJacbian(std::string armType)
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
    M6CCSKinematics::stateToJointVector(const RobotState& state) const
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
    M6CCSKinematics::compute6DError(const EndEffectorPose& current,
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
    M6CCSKinematics::dampedLeastSquares(const Eigen::MatrixXd& J,
                                        const Eigen::VectorXd& error,
                                        double damping) const
    {
        Eigen::MatrixXd JTJ = J.transpose() * J;
        JTJ.diagonal().array() += damping;

        return JTJ.ldlt().solve(J.transpose() * error);
    }
} // namespace arms_controller_common
