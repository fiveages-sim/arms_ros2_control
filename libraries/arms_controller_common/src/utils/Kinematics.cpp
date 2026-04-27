#include "arms_controller_common/utils/Kinematics.h"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <ament_index_cpp/get_package_share_directory.hpp>
// #include <ament_index_cpp/get_package_share_directory.hpp>
#include <stdexcept>  // for std::runtime_error
#include <fstream>
#include <cstdlib>

namespace arms_controller_common
{
    namespace
    {
        std::string resolveKinematicsConfigPath()
        {
            if (const char* env_path = std::getenv("ARMS_KINEMATICS_CONFIG_PATH"))
            {
                std::ifstream env_file(env_path);
                if (env_file.good())
                {
                    return env_path;
                }
                std::cerr << "ARMS_KINEMATICS_CONFIG_PATH is set but file is not readable: "
                    << env_path << std::endl;
            }

            const auto package_share =
                ament_index_cpp::get_package_share_directory("arms_controller_common");
            const auto config_path = package_share + "/config/ccs_m6.MvKDCfg";

            std::ifstream config_file(config_path);
            if (!config_file.good())
            {
                throw std::runtime_error("Kinematics config file not found: " + config_path);
            }

            return config_path;
        }

        void initializeSdkKinematics()
        {
            // 初始化TJ_FX_ROBOT_CONTROL_SDK - 从ccs_m6.MvKDCfg加载M6CCS参数
            FX_INT32L TYPE[2];
            FX_DOUBLE GRV[2][3];
            FX_DOUBLE DH[2][8][4];
            FX_DOUBLE PNVA[2][7][4];
            FX_DOUBLE BD[2][4][3];
            FX_DOUBLE Mass[2][7];
            FX_DOUBLE MCP[2][7][3];
            FX_DOUBLE I[2][7][6];

            const auto configPath = resolveKinematicsConfigPath();

            if (LOADMvCfg((char*)configPath.c_str(), TYPE, GRV, DH, PNVA, BD, Mass, MCP, I) == FX_TRUE)
            {
                std::cout << "Loaded M6CCS config successfully from " << configPath << std::endl;
            }
            else
            {
                std::cerr << "Failed to load M6CCS config from " << configPath << std::endl;
                throw std::runtime_error("Failed to load M6CCS kinematics config");
            }

            // 初始化左右臂（serial 0: left, serial 1: right）
            for (int serial = 0; serial < 2; serial++)
            {
                if (FX_Robot_Init_Type(serial, TYPE[serial]) == FX_FALSE)
                {
                    std::cerr << "Failed to init robot type for serial " << serial << std::endl;
                }
                if (FX_Robot_Init_Kine(serial, DH[serial]) == FX_FALSE)
                {
                    std::cerr << "Failed to init kinematics for serial " << serial << std::endl;
                }
                if (FX_Robot_Init_Lmt(serial, PNVA[serial], BD[serial]) == FX_FALSE)
                {
                    std::cerr << "Failed to init limits for serial " << serial << std::endl;
                }
            }
        }
    }

    // ==================== 构造函数 ====================

    ArmKinematics::ArmKinematics(const std::string& urdf_path,
                                 const std::string& baseFrameName)
    {
        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);
        baseFrameName_ = baseFrameName;
        buildMappings();

        // 初始化权重为单位权重
        weight_ = Eigen::VectorXd::Ones(6);

        initializeSdkKinematics();
    }

    ArmKinematics::ArmKinematics(const pinocchio::Model& model) : model_(model)
    {
        data_ = pinocchio::Data(model_);
        buildMappings();
        weight_ = Eigen::VectorXd::Ones(6);

        initializeSdkKinematics();
    }

    ArmKinematics::~ArmKinematics() = default;

    void ArmKinematics::initializeFromParameters(
        const std::vector<std::string>& joint_names,
        const std::string& left_ee_name, const std::string& right_ee_name)
    {
        setJointNames(joint_names);
        setEndEffectorNames(left_ee_name, right_ee_name);
    }

    void ArmKinematics::setJointNames(const std::vector<std::string>& joint_names)
    {
        size_t half = joint_names.size() / 2;
        leftArmJointNames_.assign(joint_names.begin(), joint_names.begin() + half);
        rightArmJointNames_.assign(joint_names.begin() + half, joint_names.end());

        leftArmJointCount_ = leftArmJointNames_.size();
        rightArmJointCount_ = rightArmJointNames_.size();

        std::cout << "Left joint names: ";
        for (const auto& name : leftArmJointNames_)
        {
            std::cout << name << " ";
        }
        std::cout << std::endl;

        std::cout << "Right joint names: ";
        for (const auto& name : rightArmJointNames_)
        {
            std::cout << name << " ";
        }
        std::cout << std::endl;

        extractJointLimits();
    }

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
    }

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

    EndEffectorPose ArmKinematics::computeFramePose(const RobotState& state,
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

    // ==================== 逆运动学入口 ====================

    bool ArmKinematics::solveSingleArmIK(const EndEffectorPose& targetPose,
                                         const Eigen::VectorXd& initialGuess,
                                         Eigen::VectorXd& solution,
                                         std::string arm_type,
                                         int maxIterations,
                                         double tolerance)
    {
        if (params_.solverType == SolverType::SDK)
        {
            return solveSingleArmIKWithSDK(targetPose, initialGuess, solution, arm_type);
        }
        else
        {
            SolutionInfo info;
            return solveSingleArmIKWithInfo(targetPose, initialGuess, solution, info,
                                            arm_type, maxIterations, tolerance);
        }
    }

    bool ArmKinematics::solveSingleArmIKWithInfo(const EndEffectorPose& targetPose,
                                                 const Eigen::VectorXd& initialGuess,
                                                 Eigen::VectorXd& solution,
                                                 SolutionInfo& info,
                                                 std::string arm_type,
                                                 int maxIterations,
                                                 double tolerance)
    {
        int jointCount = (arm_type == "left") ? leftArmJointCount_ : rightArmJointCount_;
        if (initialGuess.size() != static_cast<Eigen::Index>(jointCount))
        {
            std::cerr << "Initial guess size mismatch: expected " << jointCount
                << ", got " << initialGuess.size() << std::endl;
            return false;
        }

        params_.maxIterations = maxIterations;
        params_.solutionTolerance = tolerance;

        Eigen::VectorXd lower, upper;
        getJointLimits(arm_type, lower, upper);

        std::pair<Eigen::VectorXd, SolutionInfo> result;

        switch (params_.solverType)
        {
        case SolverType::DLS:
            result = solveDLS(initialGuess, lower, upper, arm_type, targetPose);
            result.second.usedSolver = SolverType::DLS;
            break;

        case SolverType::BFGS:
            result = solveBFGS(initialGuess, lower, upper, arm_type, targetPose);
            result.second.usedSolver = SolverType::BFGS;
            break;

        case SolverType::AUTO:
        default:
            // 先尝试 DLS（快速）
            result = solveDLS(initialGuess, lower, upper, arm_type, targetPose);
            result.second.usedSolver = SolverType::DLS;

            // 如果 DLS 失败，尝试 BFGS
            if (result.second.status != "success")
            {
                std::cout << "DLS failed, trying BFGS..." << std::endl;
                result = solveBFGS(initialGuess, lower, upper, arm_type, targetPose);
                result.second.usedSolver = SolverType::BFGS;
            }
            break;
        }

        solution = result.first;
        info = result.second;

        if (info.status == "success")
        {
            // std::cout << "IK succeeded (" << (info.usedSolver == SolverType::DLS ? "DLS" : "BFGS")
            //     << "): " << info.iterations << " iterations, error="
            //     << info.poseErrorNorm << std::endl;

            return true;
        }

        std::cerr << "IK failed (" << (info.usedSolver == SolverType::DLS ? "DLS" : "BFGS")
            << "): " << info.status << ", final error: " << info.poseErrorNorm << std::endl;
        return false;
    }


    std::pair<Eigen::VectorXd, ArmKinematics::SolutionInfo>
    ArmKinematics::solveDLS(const Eigen::VectorXd& seed,
                            const Eigen::VectorXd& lower,
                            const Eigen::VectorXd& upper,
                            std::string arm_type,
                            const EndEffectorPose& target)
    {
        SolutionInfo info;

        // 热启动
        Eigen::VectorXd q = seed;
        // if (lastArmType_ == arm_type)
        // {
        //     if (arm_type == "left" && lastLeftSolution_.size() == seed.size())
        //         q = lastLeftSolution_;
        //     else if (arm_type == "right" && lastRightSolution_.size() == seed.size())
        //         q = lastRightSolution_;
        // }
        q = clampToLimits(q, lower, upper);

        RobotState state(leftArmJointCount_, rightArmJointCount_);
        int frameId = (arm_type == "left")
                          ? getFrameId(leftEndEffectorName_)
                          : getFrameId(rightEndEffectorName_);
        if (frameId < 0)
        {
            info.status = "invalid frame";
            info.poseErrorNorm = std::numeric_limits<double>::max();
            return {q, info};
        }

        double previousError = std::numeric_limits<double>::max();
        int stagnationCount = 0;

        for (int iter = 0; iter < params_.maxIterations; ++iter)
        {
            if (arm_type == "left")
                state.leftArmJoints = q;
            else
                state.rightArmJoints = q;
            Eigen::VectorXd joints = stateToJointVector(state);
            updateKinematics(joints);

            const auto& placement = data_.oMf[frameId];
            EndEffectorPose currentPose;
            currentPose.position = placement.translation();
            currentPose.setRotation(placement.rotation());

            Eigen::Matrix < double, 6, 1 > error = compute6DError(currentPose, target);
            double currentError = error.norm();

            if (currentError < params_.solutionTolerance)
            {
                info.iterations = iter;
                info.poseErrorNorm = currentError;
                info.status = "success";
                return {q, info};
            }

            // 停滞检测
            if (currentError >= previousError * 0.99)
            {
                stagnationCount++;
                if (stagnationCount > params_.dlsStagnationLimit)
                {
                    info.iterations = iter;
                    info.poseErrorNorm = currentError;
                    info.status = "stagnation";
                    return {q, info};
                }
            }
            else
            {
                stagnationCount = 0;
            }
            previousError = currentError;

            // 计算雅可比和更新
            Eigen::MatrixXd J = getJacobian(arm_type);
            double damping = computeAdaptiveDamping(J, params_.dlsDamping);
            Eigen::VectorXd dq = dampedLeastSquares(J, error, damping);
            limitStepSize(dq, params_.dlsStepLimit);

            q = q + dq;
            q = clampToLimits(q, lower, upper);
        }

        info.iterations = params_.maxIterations;
        info.poseErrorNorm = previousError;
        info.status = "max iterations";
        return {q, info};
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

    bool ArmKinematics::solveBothArmsIK(const EndEffectorPose& leftTargetPose,
                                        const EndEffectorPose& rightTargetPose,
                                        const Eigen::VectorXd& initialGuess,
                                        Eigen::VectorXd& solution,
                                        int maxIterations, double tolerance)
    {
        if (initialGuess.size() !=
            static_cast<Eigen::Index>(leftArmJointCount_ + rightArmJointCount_))
        {
            return false;
        }

        // 分别求解左臂和右臂
        Eigen::VectorXd leftSolution, rightSolution;
        bool leftSuccess = solveSingleArmIK(leftTargetPose,
                                            initialGuess.head(leftArmJointCount_),
                                            leftSolution, "left", maxIterations, tolerance);
        bool rightSuccess = solveSingleArmIK(rightTargetPose,
                                             initialGuess.tail(rightArmJointCount_),
                                             rightSolution, "right", maxIterations, tolerance);

        if (leftSuccess && rightSuccess)
        {
            solution.resize(leftArmJointCount_ + rightArmJointCount_);
            solution.head(leftArmJointCount_) = leftSolution;
            solution.tail(rightArmJointCount_) = rightSolution;
            return true;
        }

        return false;
    }

    // ==================== 雅可比矩阵 ====================

    Eigen::MatrixXd ArmKinematics::computeJacobian(const RobotState& state,
                                                   const std::string& armType)
    {
        Eigen::VectorXd joints = stateToJointVector(state);
        updateKinematics(joints);
        return getJacobian(armType);
    }

    Eigen::MatrixXd ArmKinematics::getJacobian(std::string armType)
    {
        if (armType == "left")
        {
            int frameId = getFrameId(leftEndEffectorName_);
            if (frameId < 0)
            {
                std::cerr << "Invalid end effector name: " << leftEndEffectorName_ << std::endl;
                return Eigen::MatrixXd::Zero(6, leftArmJointCount_);
            }
            Eigen::MatrixXd fullJacobian(6, model_.nv);
            fullJacobian.setZero();
            pinocchio::getFrameJacobian(model_, data_, frameId,
                                        pinocchio::LOCAL_WORLD_ALIGNED, fullJacobian);
            return fullJacobian.leftCols(leftArmJointCount_);
        }
        else if (armType == "right")
        {
            int frameId = getFrameId(rightEndEffectorName_);
            if (frameId < 0)
            {
                std::cerr << "Invalid end effector name: " << rightEndEffectorName_ << std::endl;
                return Eigen::MatrixXd::Zero(6, rightArmJointCount_);
            }
            Eigen::MatrixXd fullJacobian(6, model_.nv);
            fullJacobian.setZero();
            pinocchio::getFrameJacobian(model_, data_, frameId,
                                        pinocchio::LOCAL_WORLD_ALIGNED, fullJacobian);
            return fullJacobian.rightCols(rightArmJointCount_);
        }
        else if (armType == "both")
        {
            int leftFrameId = getFrameId(leftEndEffectorName_);
            int rightFrameId = getFrameId(rightEndEffectorName_);

            if (leftFrameId < 0 || rightFrameId < 0)
            {
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
            combinedJacobian.topRows(6) = leftJacobian;
            combinedJacobian.bottomRows(6) = rightJacobian;
            return combinedJacobian;
        }
        return Eigen::MatrixXd();
    }

    // ==================== 辅助函数 ====================

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
        joints = clampToLimits(joints, lower, upper);
    }

    void ArmKinematics::buildMappings()
    {
        jointNameToId_.clear();
        frameNameToId_.clear();

        for (pinocchio::JointIndex i = 1; i < static_cast<pinocchio::JointIndex>(model_.njoints); ++i)
        {
            jointNameToId_[model_.names[i]] = static_cast<int>(i);
        }

        for (int i = 0; i < model_.nframes; ++i)
        {
            frameNameToId_[model_.frames[i].name] = i;
        }
    }

    void ArmKinematics::extractJointLimits()
    {
        leftLowerLimits_.resize(leftArmJointCount_);
        leftUpperLimits_.resize(leftArmJointCount_);
        rightLowerLimits_.resize(rightArmJointCount_);
        rightUpperLimits_.resize(rightArmJointCount_);

        for (size_t i = 0; i < leftArmJointNames_.size(); ++i)
        {
            const std::string& jointName = leftArmJointNames_[i];
            pinocchio::JointIndex jointId = model_.getJointId(leftArmJointNames_[i]);
            if (jointId < model_.joints.size())
            {
                int idx_q = model_.joints[jointId].idx_q();
                if (model_.joints[jointId].nq() == 1 &&
                    idx_q >= 0 && idx_q < static_cast<int>(model_.lowerPositionLimit.size()))
                {
                    leftLowerLimits_[i] = model_.lowerPositionLimit[idx_q];
                    leftUpperLimits_[i] = model_.upperPositionLimit[idx_q];
                    std::cout << "Left joint " << jointName
                        << " (jointId=" << jointId << ", idx_q=" << idx_q
                        << "): [" << leftLowerLimits_[i] << ", "
                        << leftUpperLimits_[i] << "]" << std::endl;
                }
                else
                {
                    leftLowerLimits_[i] = -M_PI;
                    leftUpperLimits_[i] = M_PI;
                }
            }
            else
            {
                leftLowerLimits_[i] = -M_PI;
                leftUpperLimits_[i] = M_PI;
            }
        }

        for (size_t i = 0; i < rightArmJointNames_.size(); ++i)
        {
            const std::string& jointName = rightArmJointNames_[i];
            pinocchio::JointIndex jointId = model_.getJointId(rightArmJointNames_[i]);
            if (jointId < model_.joints.size())
            {
                int idx_q = model_.joints[jointId].idx_q();
                if (model_.joints[jointId].nq() == 1 &&
                    idx_q >= 0 && idx_q < static_cast<int>(model_.lowerPositionLimit.size()))
                {
                    rightLowerLimits_[i] = model_.lowerPositionLimit[idx_q];
                    rightUpperLimits_[i] = model_.upperPositionLimit[idx_q];
                    std::cout << "Right joint " << jointName
                        << " (jointId=" << jointId << ", idx_q=" << idx_q
                        << "): [" << rightLowerLimits_[i] << ", "
                        << rightUpperLimits_[i] << "]" << std::endl;
                }
                else
                {
                    rightLowerLimits_[i] = -M_PI;
                    rightUpperLimits_[i] = M_PI;
                }
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
        return (it != frameNameToId_.end()) ? it->second : -1;
    }

    int ArmKinematics::getJointId(const std::string& jointName) const
    {
        auto it = jointNameToId_.find(jointName);
        return (it != jointNameToId_.end()) ? it->second : -1;
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

    RobotState ArmKinematics::jointVectorToState(const Eigen::VectorXd& joint,
                                                 std::string name)
    {
        RobotState res(leftArmJointCount_, rightArmJointCount_);
        if (name == "left")
        {
            res.leftArmJoints = joint;
        }
        else if (name == "right")
        {
            res.rightArmJoints = joint;
        }
        else if (name == "both")
        {
            res.leftArmJoints = joint.head(leftArmJointCount_);
            res.rightArmJoints = joint.tail(rightArmJointCount_);
        }
        return res;
    }

    Eigen::Matrix<double, 6, 1>
    ArmKinematics::compute6DError(const EndEffectorPose& current,
                                  const EndEffectorPose& target) const
    {
        Eigen::Matrix < double, 6, 1 > error;

        // 位置误差
        error.head<3>() = target.position - current.position;

        // 姿态误差（轴角表示）
        Eigen::Matrix3d R_error = target.rotationMatrix * current.rotationMatrix.transpose();
        Eigen::AngleAxisd angleAxis(R_error);
        error.tail<3>() = angleAxis.angle() * angleAxis.axis();

        return error;
    }

    // ==================== BFGS求解器核心函数 ====================

    void ArmKinematics::computeCostAndGrad(const Eigen::VectorXd& error,
                                           const Eigen::MatrixXd& J,
                                           const Eigen::VectorXd& weight,
                                           Eigen::VectorXd& grad,
                                           double* cost)
    {
        Eigen::MatrixXd W = weight.asDiagonal();
        *cost = 0.5 * error.transpose() * W * error;
        grad = -J.transpose() * W * error;
    }

    // ==================== 新增：带正则化的代价函数 ====================
    void ArmKinematics::computeCostAndGradWithReg(const Eigen::VectorXd& error,
                                                  const Eigen::MatrixXd& J,
                                                  const Eigen::VectorXd& weight,
                                                  Eigen::VectorXd& grad,
                                                  double* cost,
                                                  const Eigen::VectorXd& q_ref,
                                                  const Eigen::VectorXd& q,
                                                  double lambda_reg)
    {
        Eigen::MatrixXd W = weight.asDiagonal();

        // 原始任务代价和梯度
        double task_cost = 0.5 * error.transpose() * W * error;
        Eigen::VectorXd task_grad = -J.transpose() * W * error;

        // 正则化项：惩罚偏离参考关节位置
        Eigen::VectorXd q_diff = q - q_ref;
        double reg_cost = 0.5 * lambda_reg * q_diff.squaredNorm();
        Eigen::VectorXd reg_grad = lambda_reg * q_diff;

        // 合并
        *cost = task_cost + reg_cost;
        grad = task_grad + reg_grad;
    }

    bool ArmKinematics::isPositiveDefinite(const Eigen::MatrixXd& matrix)
    {
        Eigen::LLT<Eigen::MatrixXd> llt(matrix);
        return llt.info() == Eigen::Success;
    }

    Eigen::VectorXd ArmKinematics::randomConfig(const Eigen::VectorXd& lower,
                                                const Eigen::VectorXd& upper)
    {
        static std::random_device rd;
        static std::mt19937 gen(rd());

        Eigen::VectorXd q(lower.size());
        for (int i = 0; i < q.size(); ++i)
        {
            std::uniform_real_distribution<> dis(lower(i), upper(i));
            q(i) = dis(gen);
        }
        return q;
    }

    Eigen::VectorXd ArmKinematics::clampToLimits(const Eigen::VectorXd& q,
                                                 const Eigen::VectorXd& lower,
                                                 const Eigen::VectorXd& upper)
    {
        return q.cwiseMax(lower).cwiseMin(upper);
    }

    void ArmKinematics::updateHessian(Eigen::MatrixXd& H,
                                      const Eigen::VectorXd& s,
                                      const Eigen::VectorXd& y)
    {
        double d1 = 0.2, d2 = 0.8;
        double sy = s.dot(y);
        double yHy = y.dot(H * y);

        // 阻尼BFGS
        double theta = 1.0;
        if (sy < d1 * yHy)
        {
            theta = d2 * yHy / (yHy - sy);
        }

        Eigen::VectorXd s_new = theta * s + (1.0 - theta) * (H * y);
        double rho = s_new.dot(y);

        if (std::abs(rho) > 1e-10)
        {
            Eigen::MatrixXd I = Eigen::MatrixXd::Identity(H.rows(), H.cols());
            Eigen::MatrixXd V = I - (s_new * y.transpose()) / rho;
            H = V * H * V.transpose() + (s_new * s_new.transpose()) / rho;
        }

        // 确保正定性
        if (!isPositiveDefinite(H))
        {
            H = Eigen::MatrixXd::Identity(H.rows(), H.cols());
        }
    }

    void ArmKinematics::projectHessianToConstraints(
        Eigen::MatrixXd& H, const Eigen::MatrixXd& activeConstraints)
    {
        Eigen::MatrixXd H_proj = H;
        for (int i = 0; i < activeConstraints.cols(); ++i)
        {
            Eigen::VectorXd a = activeConstraints.col(i);
            double rho = a.transpose() * H_proj * a;
            if (std::abs(rho) > 1e-10)
            {
                H_proj = H_proj - (1.0 / rho) * H_proj * (a * a.transpose()) * H_proj;
            }
        }
        H = H_proj;
    }

    void ArmKinematics::buildConstraintMatrix(Eigen::MatrixXd& A,
                                              Eigen::VectorXd& b,
                                              const Eigen::VectorXd& lower,
                                              const Eigen::VectorXd& upper)
    {
        int n = lower.size();
        int m = 2 * n;

        A.resize(m, n);
        b.resize(m);
        A.setZero();

        for (int i = 0; i < n; ++i)
        {
            // q_i <= upper_i  ->  q_i - upper_i <= 0
            A(2 * i, i) = 1.0;
            b(2 * i) = upper(i);

            // q_i >= lower_i  ->  -q_i + lower_i <= 0
            A(2 * i + 1, i) = -1.0;
            b(2 * i + 1) = -lower(i);
        }
    }

    void ArmKinematics::identifyActiveConstraints(
        const Eigen::VectorXd& x, const Eigen::MatrixXd& A,
        const Eigen::VectorXd& b, Eigen::VectorXi& activeSet,
        Eigen::MatrixXd& activeConstraints)
    {
        std::vector<int> activeIndices;
        const double tol = 1e-8;

        for (int i = 0; i < A.rows(); ++i)
        {
            double violation = A.row(i).dot(x) - b(i);
            if (violation >= -tol)
            {
                activeIndices.push_back(i);
            }
        }

        activeSet.resize(activeIndices.size());
        activeConstraints.resize(A.cols(), activeIndices.size());

        for (size_t i = 0; i < activeIndices.size(); ++i)
        {
            activeSet(i) = activeIndices[i];
            activeConstraints.col(i) = A.row(activeIndices[i]).transpose();
        }
    }

    bool ArmKinematics::isLocalMinimum(const Eigen::VectorXd& Hg,
                                       const Eigen::VectorXd& grad,
                                       const Eigen::MatrixXd& activeConstraints,
                                       const Eigen::VectorXd& x,
                                       const Eigen::MatrixXd& A,
                                       const Eigen::VectorXd& b)
    {
        (void)x;
        (void)A;
        (void)b;
        // 检查投影梯度范数
        if (Hg.norm() >= params_.gradientTolerance)
        {
            return false;
        }

        // 检查Lagrange乘子
        if (activeConstraints.cols() > 0)
        {
            Eigen::MatrixXd ATA = activeConstraints.transpose() * activeConstraints;
            Eigen::VectorXd alpha = ATA.ldlt().solve(activeConstraints.transpose() * grad);

            for (int i = 0; i < alpha.size(); ++i)
            {
                if (alpha(i) > 1e-8)
                {
                    return false;
                }
            }
        }

        return true;
    }

    void ArmKinematics::manageConstraints(Eigen::MatrixXd& H,
                                          const Eigen::VectorXd& grad,
                                          Eigen::MatrixXd& activeConstraints,
                                          Eigen::VectorXi& activeSet,
                                          const Eigen::MatrixXd& A,
                                          const Eigen::VectorXd& b,
                                          const Eigen::VectorXd& x)
    {
        (void)b;
        (void)x;
        if (activeConstraints.cols() == 0) return;

        // 计算Lagrange乘子
        Eigen::MatrixXd ATA = activeConstraints.transpose() * activeConstraints;
        Eigen::VectorXd alpha = ATA.ldlt().solve(activeConstraints.transpose() * grad);

        // 找到最大的乘子
        int dropIdx = -1;
        double maxAlpha = -std::numeric_limits<double>::max();
        for (int i = 0; i < alpha.size(); ++i)
        {
            if (alpha(i) > maxAlpha)
            {
                maxAlpha = alpha(i);
                dropIdx = i;
            }
        }

        // 检查是否放弃约束
        Eigen::VectorXd Hg = H * grad;
        if (Hg.norm() < 0.5 * maxAlpha)
        {
            // 放弃该约束
            Eigen::VectorXi newActiveSet(activeSet.size() - 1);
            Eigen::MatrixXd newActiveConstraints(A.cols(), activeSet.size() - 1);

            int idx = 0;
            for (int i = 0; i < activeSet.size(); ++i)
            {
                if (i != dropIdx)
                {
                    newActiveSet(idx) = activeSet(i);
                    newActiveConstraints.col(idx) = activeConstraints.col(i);
                    idx++;
                }
            }

            int droppedConstraintIdx = activeSet(dropIdx);
            Eigen::VectorXd a_dropped = A.row(droppedConstraintIdx).transpose();

            activeSet = newActiveSet;
            activeConstraints = newActiveConstraints;

            // 更新Hessian
            if (activeConstraints.cols() > 0)
            {
                Eigen::MatrixXd ATA_new = activeConstraints.transpose() * activeConstraints;
                Eigen::MatrixXd P = Eigen::MatrixXd::Identity(A.cols(), A.cols()) -
                    activeConstraints * ATA_new.inverse() * activeConstraints.transpose();
                double denom = a_dropped.transpose() * P * a_dropped;
                if (std::abs(denom) > 1e-10)
                {
                    H = H + (1.0 / denom) * P * (a_dropped * a_dropped.transpose()) * P;
                }
            }
            else
            {
                // 没有约束了，不需要投影
            }
        }
    }

    double ArmKinematics::computeStepBound(const Eigen::VectorXd& x,
                                           const Eigen::VectorXd& s,
                                           const Eigen::MatrixXd& A,
                                           const Eigen::VectorXd& b,
                                           const Eigen::VectorXi& activeSet)
    {
        double lambda = std::numeric_limits<double>::max();

        for (int i = 0; i < A.rows(); ++i)
        {
            // 检查是否为非激活约束
            bool isActive = false;
            for (int j = 0; j < activeSet.size(); ++j)
            {
                if (activeSet(j) == i)
                {
                    isActive = true;
                    break;
                }
            }

            if (!isActive)
            {
                double Ai_dot_s = A.row(i).dot(s);
                if (Ai_dot_s > 1e-10)
                {
                    double bi_minus_Aix = b(i) - A.row(i).dot(x);
                    double lambda_i = bi_minus_Aix / Ai_dot_s;
                    if (lambda_i > 0 && lambda_i < lambda)
                    {
                        lambda = lambda_i;
                    }
                }
            }
        }

        return lambda;
    }

    double ArmKinematics::lineSearchWithBound(const Eigen::VectorXd& x,
                                              const Eigen::VectorXd& s,
                                              double cost,
                                              const Eigen::VectorXd& grad,
                                              double maxGamma,
                                              std::string arm_type,
                                              const EndEffectorPose& target,
                                              const Eigen::VectorXd& q_ref)
    {
        double gamma = std::min(1.0, maxGamma);

        while (gamma > params_.stepTolerance)
        {
            // 直接更新，不clamp
            Eigen::VectorXd x_new = x + gamma * s;

            // 计算新代价
            RobotState state = jointVectorToState(x_new, arm_type);
            EndEffectorPose current_pose = computeSingleEndEffectorPose(state, arm_type);
            Eigen::MatrixXd jacobian = getJacobian(arm_type);
            Eigen::Matrix < double, 6, 1 > error = compute6DError(current_pose, target);

            double cost_new;
            Eigen::VectorXd grad_new;
            // 根据是否启用正则化选择代价函数
            if (params_.useRegularization)
            {
                computeCostAndGradWithReg(error, jacobian, weight_, grad_new, &cost_new,
                                          q_ref, x_new, params_.lambdaRegularization);
            }
            else
            {
                computeCostAndGrad(error, jacobian, weight_, grad_new, &cost_new);
            }
            // Armijo条件
            double expectedDecrease = -params_.armijoSigma * grad.dot(gamma * s);
            double actualDecrease = cost - cost_new;

            if (actualDecrease >= expectedDecrease)
            {
                return gamma;
            }

            gamma *= params_.armijoBeta;
        }

        return 0.0;
    }

    // ==================== 内部求解器 ====================

    std::pair<Eigen::VectorXd, ArmKinematics::SolutionInfo>
    ArmKinematics::solveBFGSInternal(const Eigen::VectorXd& seed,
                                     const Eigen::VectorXd& lower,
                                     const Eigen::VectorXd& upper,
                                     std::string arm_type,
                                     const EndEffectorPose& target)
    {
        SolutionInfo info;

        // 初始点只clamp一次
        Eigen::VectorXd x = clampToLimits(seed, lower, upper);
        int n = x.size();

        // 保存参考关节位置（使用初始猜测 seed，而不是 clamp 后的 x）
        Eigen::VectorXd q_ref = seed;

        // 计算初始误差
        RobotState state = jointVectorToState(x, arm_type);
        EndEffectorPose current_pose = computeSingleEndEffectorPose(state, arm_type);
        Eigen::Matrix < double, 6, 1 > error = compute6DError(current_pose, target);

        // 如果初始误差已经很小，直接返回
        if (error.norm() <= params_.solutionTolerance)
        {
            info.poseErrorNorm = error.norm();
            info.iterations = 0;
            info.exitFlag = 0;
            info.status = "success";
            return {x, info};
        }

        // 计算初始代价和梯度
        Eigen::VectorXd grad;
        double cost;
        state = jointVectorToState(x, arm_type);
        current_pose = computeSingleEndEffectorPose(state, arm_type);
        Eigen::MatrixXd jacobian = getJacobian(arm_type);
        error = compute6DError(current_pose, target);

        // 根据是否启用正则化选择代价函数
        if (params_.useRegularization)
        {
            computeCostAndGradWithReg(error, jacobian, weight_, grad, &cost,
                                      q_ref, x, params_.lambdaRegularization);
        }
        else
        {
            computeCostAndGrad(error, jacobian, weight_, grad, &cost);
        }

        // 初始化Hessian近似
        Eigen::MatrixXd H = Eigen::MatrixXd::Identity(n, n);

        // 构建约束矩阵
        Eigen::MatrixXd A;
        Eigen::VectorXd b;
        if (params_.constraintsOn)
        {
            buildConstraintMatrix(A, b, lower, upper);
        }

        // 识别激活约束
        Eigen::VectorXi activeSet;
        Eigen::MatrixXd activeConstraints;
        if (params_.constraintsOn)
        {
            identifyActiveConstraints(x, A, b, activeSet, activeConstraints);
            if (activeConstraints.cols() > 0)
            {
                projectHessianToConstraints(H, activeConstraints);
            }
        }

        // 主迭代循环
        for (int iter = 0; iter < params_.maxIterations; ++iter)
        {
            // 计算投影梯度
            Eigen::VectorXd Hg = H * grad;

            // 检查KKT条件
            if (isLocalMinimum(Hg, grad, activeConstraints, x, A, b))
            {
                info.exitFlag = 0;
                info.iterations = iter;
                break;
            }

            // 约束管理
            if (params_.constraintsOn && activeConstraints.cols() > 0)
            {
                manageConstraints(H, grad, activeConstraints, activeSet, A, b, x);
            }

            // 搜索方向
            Eigen::VectorXd s = -Hg;

            // 计算步长上限
            double maxGamma = 1.0;
            if (params_.constraintsOn)
            {
                maxGamma = computeStepBound(x, s, A, b, activeSet);
            }

            // 线搜索（传入 q_ref）
            double gamma = lineSearchWithBound(x, s, cost, grad, maxGamma, arm_type, target, q_ref);

            if (gamma < params_.stepTolerance)
            {
                info.exitFlag = 4;
                info.iterations = iter;
                break;
            }

            // 更新状态（不clamp）
            Eigen::VectorXd x_new = x + gamma * s;

            // 调试：检查约束
            if (params_.constraintsOn)
            {
                bool violated = false;
                for (int i = 0; i < A.rows(); ++i)
                {
                    double violation = A.row(i).dot(x_new) - b(i);
                    if (violation > 1e-6)
                    {
                        violated = true;
                    }
                }
                if (violated)
                {
                    x_new = clampToLimits(x_new, lower, upper);
                }
            }

            // 计算新代价和梯度
            double cost_new;
            Eigen::VectorXd grad_new;
            state = jointVectorToState(x_new, arm_type);
            current_pose = computeSingleEndEffectorPose(state, arm_type);
            jacobian = getJacobian(arm_type);
            error = compute6DError(current_pose, target);

            // 根据是否启用正则化选择代价函数
            if (params_.useRegularization)
            {
                computeCostAndGradWithReg(error, jacobian, weight_, grad_new, &cost_new,
                                          q_ref, x_new, params_.lambdaRegularization);
            }
            else
            {
                computeCostAndGrad(error, jacobian, weight_, grad_new, &cost_new);
            }

            // 检查收敛（只检查任务误差）
            if (error.norm() < params_.solutionTolerance)
            {
                info.exitFlag = 0;
                info.iterations = iter + 1;
                x = x_new;
                grad = grad_new;
                cost = cost_new;
                break;
            }

            // 更新激活约束集
            if (params_.constraintsOn)
            {
                identifyActiveConstraints(x_new, A, b, activeSet, activeConstraints);
                if (std::abs(gamma - maxGamma) < 1e-8 && maxGamma < 1.0)
                {
                    projectHessianToConstraints(H, activeConstraints);
                }
            }

            // BFGS更新
            Eigen::VectorXd s_bfgs = x_new - x;
            Eigen::VectorXd y = grad_new - grad;

            if (s_bfgs.norm() > 1e-10)
            {
                updateHessian(H, s_bfgs, y);
            }

            // 更新变量
            x = x_new;
            grad = grad_new;
            cost = cost_new;
            info.iterations = iter + 1;
        }

        info.poseErrorNorm = error.norm();
        return {x, info};
    }

    std::pair<Eigen::VectorXd, ArmKinematics::SolutionInfo>
    ArmKinematics::solveBFGS(const Eigen::VectorXd& initialGuess,
                             const Eigen::VectorXd& lower,
                             const Eigen::VectorXd& upper,
                             std::string arm_type,
                             const EndEffectorPose& target)
    {
        SolutionInfo bestInfo;
        Eigen::VectorXd bestSolution = initialGuess;
        double bestError = std::numeric_limits<double>::max();
        int restartCount = 0;
        int maxRestarts = params_.randomRestart ? params_.maxRestarts : 0;

        while (restartCount <= maxRestarts)
        {
            Eigen::VectorXd q;
            if (restartCount == 0)
                q = initialGuess;
            else
            {
                q = randomConfig(lower, upper);
                bestInfo.numRandomRestarts++;
            }

            auto [solution, localInfo] = solveBFGSInternal(q, lower, upper, arm_type, target);

            if (localInfo.poseErrorNorm < bestError)
            {
                bestError = localInfo.poseErrorNorm;
                bestSolution = solution;
                bestInfo = localInfo;
            }

            if (bestError <= params_.solutionTolerance)
            {
                bestInfo.status = "success";
                break;
            }

            restartCount++;

            if (!params_.randomRestart || localInfo.exitFlag == 1 || localInfo.exitFlag == 2)
                break;
        }

        bestInfo.poseErrorNorm = bestError;
        if (bestInfo.status.empty())
            bestInfo.status = "best available";

        return {bestSolution, bestInfo};
    }

    // ==================== 使用TJ_FX_ROBOT_CONTROL_SDK的逆运动学 ====================

    bool ArmKinematics::solveSingleArmIKWithSDK(const EndEffectorPose& targetPose,
                                                const Eigen::VectorXd& initialGuess,
                                                Eigen::VectorXd& solution,
                                                std::string arm_type)
    {
        int serial = (arm_type == "left") ? 0 : 1;
        int jointCount = (arm_type == "left") ? leftArmJointCount_ : rightArmJointCount_;

        if (initialGuess.size() != jointCount)
        {
            std::cerr << "Joint count mismatch: expected " << jointCount
                << ", got " << initialGuess.size() << std::endl;
            return false;
        }

        // ========== 获取从 SDK 基座到 URDF 基座的固定变换的逆变换 ==========
        Eigen::Matrix4d base_transform = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d base_transform_inv = Eigen::Matrix4d::Identity();

        if (arm_type == "left")
        {
            std::string base_frame = "left_base_link";
            int frame_id = getFrameId(base_frame);

            if (frame_id >= 0)
            {
                Eigen::VectorXd zero_joints = Eigen::VectorXd::Zero(model_.nv);
                updateKinematics(zero_joints);

                const auto& placement = data_.oMf[frame_id];
                base_transform = placement.toHomogeneousMatrix();
                base_transform_inv = base_transform.inverse();
            }
            else
            {
                std::cerr << "Warning: Could not find frame " << base_frame << std::endl;
            }
        }
        else if (arm_type == "right")
        {
            std::string base_frame = "right_base_link";
            int frame_id = getFrameId(base_frame);

            if (frame_id >= 0)
            {
                Eigen::VectorXd zero_joints = Eigen::VectorXd::Zero(model_.nv);
                updateKinematics(zero_joints);

                const auto& placement = data_.oMf[frame_id];
                base_transform = placement.toHomogeneousMatrix();
                base_transform_inv = base_transform.inverse();
            }
            else
            {
                std::cerr << "Warning: Could not find frame " << base_frame << std::endl;
            }
        }

        // ========== 将 URDF 坐标系下的目标位姿转换到 SDK 坐标系下 ==========
        // 构建 URDF 目标位姿的 4x4 矩阵
        Eigen::Matrix4d urdf_target_pose = Eigen::Matrix4d::Identity();
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                urdf_target_pose(i, j) = targetPose.rotationMatrix(i, j);
            }
            urdf_target_pose(i, 3) = targetPose.position(i); // 已经是米
        }

        // 应用逆变换: T_sdk_target = T_base^{-1} * T_urdf_target
        Eigen::Matrix4d sdk_target_pose = base_transform_inv * urdf_target_pose;

        // 转换到 SDK 需要的 Matrix4 格式（注意：SDK 使用毫米作为长度单位）
        Matrix4 targetTCP = {0};
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                targetTCP[i][j] = sdk_target_pose(i, j);
            }
            // SDK 使用毫米，所以乘以 1000
            targetTCP[i][3] = sdk_target_pose(i, 3) * 1000.0;
        }
        targetTCP[3][3] = 1.0;

        // 准备FX_InvKineSolvePara
        FX_InvKineSolvePara solvePara;
        memset(&solvePara, 0, sizeof(FX_InvKineSolvePara));
        memcpy(solvePara.m_Input_IK_TargetTCP, targetTCP, sizeof(Matrix4));

        // 转换初始猜测到Vect7 (SDK使用度)
        FX_DOUBLE initial_deg[7] = {0};
        for (int i = 0; i < jointCount; i++)
        {
            initial_deg[i] = initialGuess(i) * 180.0 / M_PI;
            solvePara.m_Input_IK_RefJoint[i] = initial_deg[i];
        }

        if (params_.use_nsp)
        {
            // 如果用户设置了臂角方向，直接使用
            if (params_.nsp_direction.norm() > 0)
            {
                solvePara.m_Input_IK_ZSPType = 1;
                solvePara.m_Input_IK_ZSPPara[0] = params_.nsp_direction.x();
                solvePara.m_Input_IK_ZSPPara[1] = params_.nsp_direction.y();
                solvePara.m_Input_IK_ZSPPara[2] = params_.nsp_direction.z();
                solvePara.m_Input_ZSP_Angle = params_.nsp_angle;
            }
            else
            {
                // 没有设置方向，从初始角度的正解自动计算臂角
                Matrix4 temp_pose;
                Matrix3 nsp_matrix;
                if (FX_Robot_Kine_FK_NSP(serial, initial_deg, temp_pose, nsp_matrix))
                {
                    solvePara.m_Input_IK_ZSPType = 1;
                    // 使用臂角矩阵的第一列作为方向
                    solvePara.m_Input_IK_ZSPPara[0] = nsp_matrix[0][0];
                    solvePara.m_Input_IK_ZSPPara[1] = nsp_matrix[1][0];
                    solvePara.m_Input_IK_ZSPPara[2] = nsp_matrix[2][0];
                    solvePara.m_Input_ZSP_Angle = params_.nsp_angle;
                }
                else
                {
                    solvePara.m_Input_IK_ZSPType = 0;
                }
            }
        }
        else
        {
            solvePara.m_Input_IK_ZSPType = 0;
        }

        solvePara.m_DGR1 = params_.dgr1;
        solvePara.m_DGR2 = params_.dgr2;
        solvePara.m_DGR3 = params_.dgr3;

        // 调用SDK IK
        if (FX_Robot_Kine_IK(serial, &solvePara))
        {
            // 检查输出
            if (!solvePara.m_Output_IsOutRange && solvePara.m_OutPut_Result_Num > 0)
            {
                // 转换输出到Eigen (从度转换为弧度)
                solution.resize(jointCount);
                for (int i = 0; i < jointCount; i++)
                {
                    solution(i) = solvePara.m_Output_RetJoint[i] * M_PI / 180.0;
                }
                return true;
            }
            else
            {
                if (solvePara.m_Output_IsOutRange)
                    std::cerr << "SDK IK: Target pose is out of workspace" << std::endl;
                if (solvePara.m_OutPut_Result_Num == 0)
                    std::cerr << "SDK IK: No solution found" << std::endl;
            }
        }
        return false;
    }


    //服务相关的代码
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
            EndEffectorPose pose;
            if (params_.solverType == SolverType::SDK)
            {
                bool success = computeForwardKinematicsWithSDK(state.leftArmJoints, "left", pose);
                if (!success)
                {
                    error_msg = "SDK forward kinematics failed for left arm";
                    return false;
                }
            }
            else
            {
                pose = computeSingleEndEffectorPose(state, "left");
            }


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
            EndEffectorPose pose;
            if (params_.solverType == SolverType::SDK)
            {
                bool success = computeForwardKinematicsWithSDK(state.rightArmJoints, "right", pose);
                if (!success)
                {
                    error_msg = "SDK forward kinematics failed for right arm";
                    return false;
                }
            }
            else
            {
                pose = computeSingleEndEffectorPose(state, "right");
            }

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
            if (params_.solverType == SolverType::SDK)
            {
                bool success_left = computeForwardKinematicsWithSDK(state.leftArmJoints, "left", left_pose);
                bool success_right = computeForwardKinematicsWithSDK(state.rightArmJoints, "right", right_pose);
                if (!success_left || !success_right)
                {
                    error_msg = "SDK forward kinematics failed for one or both arms";
                    return false;
                }
            }
            else
            {
                computeBothEndEffectorPose(state, left_pose, right_pose);
            }


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
            bool success;
                success = solveSingleArmIK(target_poses[0], initial_eigen, solution,
                                           "left", max_iterations, tolerance);


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
            bool success;
                success = solveSingleArmIK(target_poses[0], initial_eigen, solution,
                                           "right", max_iterations, tolerance);

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
                                           max_iterations, tolerance);

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
        //设置求解器类型
        if (request->solver_type == "SDK")
        {
            params_.solverType = SolverType::SDK;
        }
        else if (request->solver_type == "BFGS")
        {
            params_.solverType = SolverType::BFGS;
        }
        else if (request->solver_type == "DLS")
        {
            params_.solverType = SolverType::DLS;
        }
        else
        {
            params_.solverType = SolverType::AUTO;
        }
        // 设置求解参数
        params_.maxIterations = request->max_iterations > 0 ? request->max_iterations : 1000;
        params_.solutionTolerance = request->tolerance > 0 ? request->tolerance : 1e-4;
        params_.dlsDamping = request->dls_damping > 0 ? request->dls_damping : 0.01;
        params_.randomRestart = request->enable_random_restart;

        // SDK参数
        params_.dgr1 = request->dgr1 > 0 ? request->dgr1 : 0.05;
        params_.dgr2 = request->dgr2 > 0 ? request->dgr2 : 0.05;
        params_.dgr3 = request->dgr3;


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


    bool ArmKinematics::computeForwardKinematicsWithSDK(
        const Eigen::VectorXd& joint_angles,
        const std::string& arm_type,
        EndEffectorPose& pose)
    {
        int serial = (arm_type == "left") ? 0 : 1;
        int jointCount = (arm_type == "left") ? leftArmJointCount_ : rightArmJointCount_;

        if (joint_angles.size() != jointCount)
        {
            std::cerr << "Joint count mismatch: expected " << jointCount
                << ", got " << joint_angles.size() << std::endl;
            return false;
        }
        // 转换关节角为度（SDK使用度）
        FX_DOUBLE joints_deg[7] = {0};
        for (int i = 0; i < jointCount; i++)
        {
            joints_deg[i] = joint_angles(i) * 180.0 / M_PI;
        }

        // 调用SDK正解（得到TCP位姿，因为工具已经在初始化时设置好了）
        Matrix4 result_matrix;
        memset(result_matrix, 0, sizeof(Matrix4));

        bool success = FX_Robot_Kine_FK(serial, joints_deg, result_matrix);

        if (!success)
        {
            std::cerr << "SDK FK failed for " << arm_type << " arm" << std::endl;
            return false;
        }

        // 将 SDK 结果转换为 Eigen 矩阵 (4x4)
        Eigen::Matrix4d sdk_pose = Eigen::Matrix4d::Identity();
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                sdk_pose(i, j) = result_matrix[i][j];
            }
            sdk_pose(i, 3) = result_matrix[i][3] * 0.001; // 毫米转米
        }

        // ========== 获取从 SDK 基座到 URDF 基座的固定变换 ==========
        Eigen::Matrix4d base_transform = Eigen::Matrix4d::Identity();

        // 创建临时状态来获取固定关节的变换
        if (arm_type == "left")
        {
            // 获取 left_arm_base_joint 的变换
            // 这个方法会计算从 base_link 到 left_base_link 的变换
            std::string base_frame = "left_base_link";
            int frame_id = getFrameId(base_frame);

            if (frame_id >= 0)
            {
                // 需要先更新一次运动学（使用零位关节角）
                Eigen::VectorXd zero_joints = Eigen::VectorXd::Zero(model_.nv);
                updateKinematics(zero_joints);

                // 获取从 base_link 到 left_base_link 的变换
                const auto& placement = data_.oMf[frame_id];
                base_transform = placement.toHomogeneousMatrix();
            }
            else
            {
                std::cerr << "Warning: Could not find frame " << base_frame << std::endl;
            }
        }
        else if (arm_type == "right")
        {
            // 获取 right_arm_base_joint 的变换
            std::string base_frame = "right_base_link";
            int frame_id = getFrameId(base_frame);

            if (frame_id >= 0)
            {
                Eigen::VectorXd zero_joints = Eigen::VectorXd::Zero(model_.nv);
                updateKinematics(zero_joints);

                const auto& placement = data_.oMf[frame_id];
                base_transform = placement.toHomogeneousMatrix();
            }
            else
            {
                std::cerr << "Warning: Could not find frame " << base_frame << std::endl;
            }
        }

        // 应用变换: SDK 的结果是在 SDK 基座坐标系下，需要转换到 URDF 的 base_link 坐标系
        // SDK 的基座可能是 left_base_link/right_base_link，而我们需要的是 base_link 坐标系下的位姿
        Eigen::Matrix4d final_pose = base_transform * sdk_pose;

        // 提取旋转矩阵和平移
        Eigen::Matrix3d rot_mat = final_pose.block < 3,
        3 > (0, 0);
        Eigen::Vector3d trans = final_pose.block < 3,
        1 > (0, 3);

        pose.position = trans;
        pose.setRotation(rot_mat);
        return true;
    }
} // namespace arms_controller_common
