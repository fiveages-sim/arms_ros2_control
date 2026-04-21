#include "arms_controller_common/utils/Kinematics.h"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>

namespace arms_controller_common
{
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
    }

    ArmKinematics::ArmKinematics(const pinocchio::Model& model) : model_(model)
    {
        data_ = pinocchio::Data(model_);
        buildMappings();
        weight_ = Eigen::VectorXd::Ones(6);
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
                                         std::string arm_type, int maxIterations,
                                         double tolerance)
    {
        // 参数验证
        int jointCount = (arm_type == "left") ? leftArmJointCount_ : rightArmJointCount_;
        if (initialGuess.size() != static_cast<Eigen::Index>(jointCount))
        {
            std::cerr << "Initial guess size mismatch: expected " << jointCount
                << ", got " << initialGuess.size() << std::endl;
            return false;
        }

        // 更新参数
        params_.maxIterations = maxIterations;
        params_.solutionTolerance = tolerance;

        Eigen::VectorXd lower, upper;
        getJointLimits(arm_type, lower, upper);

        // 调用求解器
        auto [res, info] = solve(initialGuess, lower, upper, arm_type, targetPose);

        solution = res;

        if (info.status == "success")
        {
            std::cout << "IK succeeded: " << info.iterations << " iterations, "
                << info.numRandomRestarts << " restarts, error="
                << info.poseErrorNorm << std::endl;
            return true;
        }

        std::cerr << "IK failed: " << info.status
            << ", final error: " << info.poseErrorNorm << std::endl;
        return false;
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
                                              const EndEffectorPose& target)
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
            computeCostAndGrad(error, jacobian, weight_, grad_new, &cost_new);

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
    ArmKinematics::solveInternal(const Eigen::VectorXd& seed,
                                 const Eigen::VectorXd& lower,
                                 const Eigen::VectorXd& upper,
                                 std::string arm_type,
                                 const EndEffectorPose& target)
    {
        SolutionInfo info;

        // 初始点只clamp一次
        Eigen::VectorXd x = clampToLimits(seed, lower, upper);
        int n = x.size();
        // 计算初始误差
        RobotState state = jointVectorToState(x, arm_type);
        EndEffectorPose current_pose = computeSingleEndEffectorPose(state, arm_type);
        Eigen::Matrix < double, 6, 1 > error = compute6DError(current_pose, target);

        // ========== 新增：如果初始误差已经很小，直接返回 ==========
        if (error.norm() <= params_.solutionTolerance)
        {
            info.poseErrorNorm = error.norm();
            info.iterations = 0;
            info.exitFlag = 0;
            info.status = "success";
            return {x, info};
        }
        // ======================================================

        // 计算初始代价和梯度
        Eigen::VectorXd grad;
        double cost;
        state = jointVectorToState(x, arm_type);
        current_pose = computeSingleEndEffectorPose(state, arm_type);
        Eigen::MatrixXd jacobian = getJacobian(arm_type);
        error = compute6DError(current_pose, target);
        computeCostAndGrad(error, jacobian, weight_, grad, &cost);

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

            // 线搜索
            double gamma = lineSearchWithBound(x, s, cost, grad, maxGamma, arm_type, target);

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
                        // std::cerr << "Warning: Constraint " << i << " violated: " << violation << std::endl;
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
            computeCostAndGrad(error, jacobian, weight_, grad_new, &cost_new);

            // 检查收敛
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
    ArmKinematics::solve(const Eigen::VectorXd& initialGuess,
                         const Eigen::VectorXd& lower,
                         const Eigen::VectorXd& upper,
                         std::string arm_type,
                         const EndEffectorPose& target)
    {
        SolutionInfo bestInfo;
        Eigen::VectorXd bestSolution = initialGuess;
        double bestError = std::numeric_limits<double>::max();
        // ========== 新增：先检查初始猜测是否已经满足精度 ==========
        RobotState initState = jointVectorToState(initialGuess, arm_type);
        EndEffectorPose initPose = computeSingleEndEffectorPose(initState, arm_type);
        Eigen::Matrix < double, 6, 1 > initError = compute6DError(initPose, target);
        double initErrorNorm = initError.norm();

        if (initErrorNorm <= params_.solutionTolerance)
        {
            // 初始猜测已经足够好，直接返回
            bestInfo.poseErrorNorm = initErrorNorm;
            bestInfo.iterations = 0;
            bestInfo.numRandomRestarts = 0;
            bestInfo.status = "success";
            bestInfo.exitFlag = 0;
            std::cout << "IK: Initial guess already satisfies tolerance, returning it." << std::endl;
            return {initialGuess, bestInfo};
        }
        int restartCount = 0;
        while (restartCount <= params_.maxRestarts)
        {
            Eigen::VectorXd q;
            if (restartCount == 0)
            {
                q = initialGuess;
            }
            else
            {
                q = randomConfig(lower, upper);
                bestInfo.numRandomRestarts++;
            }

            auto [solution, localInfo] = solveInternal(q, lower, upper, arm_type, target);

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

            if (!params_.randomRestart ||
                localInfo.exitFlag == 1 ||
                localInfo.exitFlag == 2)
            {
                break;
            }
        }

        bestInfo.poseErrorNorm = bestError;
        if (bestInfo.status.empty())
        {
            bestInfo.status = "best available";
        }

        return {bestSolution, bestInfo};
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
            bool success = solveSingleArmIK(target_poses[0], initial_eigen, solution,
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
} // namespace arms_controller_common
