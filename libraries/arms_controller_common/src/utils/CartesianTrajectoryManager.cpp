#include "arms_controller_common/utils/CartesianTrajectoryManager.h"
#include "arms_controller_common/utils/TrajectoryRecorder.h"
#include <Eigen/Geometry>
#include <arms_ros2_control_msgs/msg/linear_message.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <limits>
#include <sstream>
#include <utility>

#include "../../include/arms_controller_common/utils/Kinematics.h"

namespace arms_controller_common
{
    namespace
    {
        Eigen::Isometry3d toIsometry(const EndEffectorPose& pose)
        {
            Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
            transform.translation() = pose.position;
            transform.linear() = pose.rotationMatrix;
            return transform;
        }

        bool poseMsgToIsometry(const geometry_msgs::msg::Pose& pose,
                               Eigen::Isometry3d& transform)
        {
            const Eigen::Quaterniond quaternion(
                pose.orientation.w, pose.orientation.x,
                pose.orientation.y, pose.orientation.z);
            if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
                !std::isfinite(pose.position.z) || !std::isfinite(quaternion.w()) ||
                !std::isfinite(quaternion.x()) || !std::isfinite(quaternion.y()) ||
                !std::isfinite(quaternion.z()) || quaternion.norm() < 1.0e-9)
            {
                return false;
            }
            transform = Eigen::Isometry3d::Identity();
            transform.translation() = Eigen::Vector3d(
                pose.position.x, pose.position.y, pose.position.z);
            transform.linear() = quaternion.normalized().toRotationMatrix();
            return true;
        }

        EndEffectorPose toEndEffectorPose(const Eigen::Isometry3d& transform)
        {
            EndEffectorPose pose;
            pose.position = transform.translation();
            pose.setRotation(transform.rotation());
            return pose;
        }

        double rotationError(const Eigen::Matrix3d& lhs, const Eigen::Matrix3d& rhs)
        {
            return Eigen::AngleAxisd(lhs.transpose() * rhs).angle();
        }

        double quinticProgress(double ratio)
        {
            const double u = std::clamp(ratio, 0.0, 1.0);
            return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
        }

        double haltonValue(std::size_t index, unsigned int base)
        {
            double value = 0.0;
            double fraction = 1.0;
            while (index > 0)
            {
                fraction /= static_cast<double>(base);
                value += fraction * static_cast<double>(index % base);
                index /= base;
            }
            return value;
        }

        std::vector<Eigen::VectorXd> makeDeterministicIkSeeds(
            const Eigen::VectorXd& initial,
            const Eigen::VectorXd& lower,
            const Eigen::VectorXd& upper,
            std::size_t low_discrepancy_seed_count)
        {
            static constexpr std::array<unsigned int, 14> primes{
                2, 3, 5, 7, 11, 13, 17,
                19, 23, 29, 31, 37, 41, 43};
            std::vector<Eigen::VectorXd> seeds;
            seeds.reserve(low_discrepancy_seed_count + 3);
            seeds.push_back(initial.cwiseMax(lower).cwiseMin(upper));
            seeds.push_back(0.5 * (lower + upper));
            seeds.push_back((lower + upper - initial).cwiseMax(lower).cwiseMin(upper));
            for (std::size_t sample = 1;
                 sample <= low_discrepancy_seed_count; ++sample)
            {
                Eigen::VectorXd seed(initial.size());
                for (Eigen::Index joint = 0; joint < initial.size(); ++joint)
                {
                    // Keep global seeds slightly away from hard joint limits.
                    const double unit = 0.025 + 0.95 * haltonValue(
                        sample, primes[static_cast<std::size_t>(joint)]);
                    seed[joint] = lower[joint] + unit * (upper[joint] - lower[joint]);
                }
                seeds.push_back(std::move(seed));
            }
            return seeds;
        }

        struct ArmIkCandidates
        {
            std::vector<Eigen::VectorXd> solutions;
            ArmKinematics::SolutionInfo best_failure;
            bool has_failure{false};
            std::size_t attempted_seeds{0};
        };

        class SolverParamsGuard
        {
        public:
            explicit SolverParamsGuard(ArmKinematics& kinematics)
                : kinematics_(kinematics), saved_(kinematics.getSolverParams())
            {
            }

            ~SolverParamsGuard()
            {
                kinematics_.setSolverParams(saved_);
            }

            SolverParamsGuard(const SolverParamsGuard&) = delete;
            SolverParamsGuard& operator=(const SolverParamsGuard&) = delete;

        private:
            ArmKinematics& kinematics_;
            ArmKinematics::SolverParams saved_;
        };

        ArmIkCandidates collectArmIkCandidates(
            ArmKinematics& kinematics,
            const EndEffectorPose& target,
            const Eigen::VectorXd& initial,
            const std::string& arm,
            int max_iterations,
            double tolerance,
            std::size_t max_solutions = 8,
            std::size_t low_discrepancy_seed_count = 32)
        {
            Eigen::VectorXd lower;
            Eigen::VectorXd upper;
            kinematics.getJointLimits(arm, lower, upper);
            ArmIkCandidates result;
            const auto seeds = makeDeterministicIkSeeds(
                initial, lower, upper, low_discrepancy_seed_count);
            for (const auto& seed : seeds)
            {
                ++result.attempted_seeds;
                Eigen::VectorXd solution;
                ArmKinematics::SolutionInfo info;
                if (kinematics.solveSingleArmIKWithInfo(
                        target, seed, solution, info, arm,
                        max_iterations, tolerance, false))
                {
                    const bool duplicate = std::any_of(
                        result.solutions.begin(), result.solutions.end(),
                        [&solution](const Eigen::VectorXd& existing)
                        {
                            return (existing - solution).norm() < 1.0e-3;
                        });
                    if (!duplicate)
                    {
                        result.solutions.push_back(std::move(solution));
                        if (result.solutions.size() >= max_solutions)
                        {
                            break;
                        }
                    }
                }
                else if (!result.has_failure ||
                         info.poseErrorNorm < result.best_failure.poseErrorNorm)
                {
                    result.best_failure = info;
                    result.has_failure = true;
                }
            }
            std::sort(
                result.solutions.begin(), result.solutions.end(),
                [&initial](const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs)
                {
                    return (lhs - initial).squaredNorm() <
                           (rhs - initial).squaredNorm();
                });
            return result;
        }

        Eigen::VectorXd monotonePathTangent(
            const std::vector<Eigen::VectorXd>& path,
            const std::vector<double>& cumulative_length,
            std::size_t index,
            std::size_t begin,
            std::size_t end)
        {
            if (begin >= end)
            {
                return Eigen::VectorXd::Zero(path[index].size());
            }
            if (index <= begin)
            {
                const double h = cumulative_length[begin + 1] -
                                 cumulative_length[begin];
                if (h > 1.0e-12)
                {
                    return (path[begin + 1] - path[begin]) / h;
                }
                return Eigen::VectorXd::Zero(path[index].size());
            }
            if (index >= end)
            {
                const double h = cumulative_length[end] -
                                 cumulative_length[end - 1];
                if (h > 1.0e-12)
                {
                    return (path[end] - path[end - 1]) / h;
                }
                return Eigen::VectorXd::Zero(path[index].size());
            }

            const double h_previous = cumulative_length[index] -
                                      cumulative_length[index - 1];
            const double h_next = cumulative_length[index + 1] -
                                  cumulative_length[index];
            if (h_previous <= 1.0e-12 || h_next <= 1.0e-12)
            {
                return Eigen::VectorXd::Zero(path[index].size());
            }
            const Eigen::VectorXd previous_slope =
                (path[index] - path[index - 1]) / h_previous;
            const Eigen::VectorXd next_slope =
                (path[index + 1] - path[index]) / h_next;
            Eigen::VectorXd tangent = Eigen::VectorXd::Zero(path[index].size());
            const double weight_previous = 2.0 * h_next + h_previous;
            const double weight_next = h_next + 2.0 * h_previous;
            for (Eigen::Index joint = 0; joint < tangent.size(); ++joint)
            {
                const double previous = previous_slope[joint];
                const double next = next_slope[joint];
                if (previous * next > 0.0)
                {
                    tangent[joint] =
                        (weight_previous + weight_next) /
                        (weight_previous / previous + weight_next / next);
                }
            }
            return tangent;
        }

        Eigen::VectorXd interpolatePathCubic(
            const std::vector<Eigen::VectorXd>& path,
            const std::vector<double>& cumulative_length,
            std::size_t lower,
            std::size_t upper,
            std::size_t milestone_begin,
            std::size_t milestone_end,
            double target_length)
        {
            const double segment_length =
                cumulative_length[upper] - cumulative_length[lower];
            if (segment_length <= 1.0e-12)
            {
                return path[lower];
            }
            const double u = std::clamp(
                (target_length - cumulative_length[lower]) / segment_length,
                0.0, 1.0);
            const double u2 = u * u;
            const double u3 = u2 * u;
            const double h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
            const double h10 = u3 - 2.0 * u2 + u;
            const double h01 = -2.0 * u3 + 3.0 * u2;
            const double h11 = u3 - u2;
            const Eigen::VectorXd lower_tangent = monotonePathTangent(
                path, cumulative_length, lower, milestone_begin, milestone_end);
            const Eigen::VectorXd upper_tangent = monotonePathTangent(
                path, cumulative_length, upper, milestone_begin, milestone_end);
            return h00 * path[lower] +
                   h10 * segment_length * lower_tangent +
                   h01 * path[upper] +
                   h11 * segment_length * upper_tangent;
        }

        Eigen::Vector3d rotationIncrement(
            const Eigen::Matrix3d& from, const Eigen::Matrix3d& to)
        {
            const Eigen::AngleAxisd delta(from.transpose() * to);
            return delta.angle() * delta.axis();
        }

        struct LimitRatios
        {
            double joint_velocity{0.0};
            double joint_acceleration{0.0};
            double linear_velocity{0.0};
            double linear_acceleration{0.0};
            double angular_velocity{0.0};
            double angular_acceleration{0.0};

            double durationScale() const
            {
                return std::max({1.0, joint_velocity, linear_velocity, angular_velocity,
                                 std::sqrt(joint_acceleration),
                                 std::sqrt(linear_acceleration),
                                 std::sqrt(angular_acceleration)});
            }
        };

        LimitRatios evaluateLimits(
            const std::vector<Eigen::VectorXd>& joints,
            const std::vector<Eigen::Isometry3d>& left_poses,
            const std::vector<Eigen::Isometry3d>& right_poses,
            double duration,
            const Eigen::VectorXd& joint_velocity_limits,
            double joint_acceleration_limit,
            double linear_velocity_limit,
            double linear_acceleration_limit,
            double angular_velocity_limit,
            double angular_acceleration_limit)
        {
            LimitRatios ratios;
            if (joints.size() < 2 || joints.size() != left_poses.size() ||
                joints.size() != right_poses.size() || duration <= 0.0)
            {
                return ratios;
            }
            const double dt = duration / static_cast<double>(joints.size() - 1);
            Eigen::VectorXd previous_joint_velocity =
                Eigen::VectorXd::Zero(joints.front().size());
            std::array<Eigen::Vector3d, 2> previous_linear_velocity{
                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
            std::array<Eigen::Vector3d, 2> previous_angular_velocity{
                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
            for (size_t i = 1; i < joints.size(); ++i)
            {
                const Eigen::VectorXd joint_velocity = (joints[i] - joints[i - 1]) / dt;
                ratios.joint_velocity = std::max(
                    ratios.joint_velocity,
                    (joint_velocity.cwiseAbs().array() /
                     joint_velocity_limits.array()).maxCoeff());
                const Eigen::VectorXd joint_acceleration =
                    (joint_velocity - previous_joint_velocity) / dt;
                ratios.joint_acceleration = std::max(
                    ratios.joint_acceleration,
                    joint_acceleration.cwiseAbs().maxCoeff() / joint_acceleration_limit);
                previous_joint_velocity = joint_velocity;

                size_t arm_index = 0;
                for (const auto* poses : {&left_poses, &right_poses})
                {
                    const Eigen::Vector3d linear_velocity =
                        ((*poses)[i].translation() - (*poses)[i - 1].translation()) / dt;
                    const Eigen::Vector3d angular_velocity = rotationIncrement(
                        (*poses)[i - 1].rotation(), (*poses)[i].rotation()) / dt;
                    ratios.linear_velocity = std::max(
                        ratios.linear_velocity, linear_velocity.norm() / linear_velocity_limit);
                    ratios.angular_velocity = std::max(
                        ratios.angular_velocity, angular_velocity.norm() / angular_velocity_limit);
                    ratios.linear_acceleration = std::max(
                        ratios.linear_acceleration,
                        (linear_velocity - previous_linear_velocity[arm_index]).norm() /
                            (dt * linear_acceleration_limit));
                    ratios.angular_acceleration = std::max(
                        ratios.angular_acceleration,
                        (angular_velocity - previous_angular_velocity[arm_index]).norm() /
                            (dt * angular_acceleration_limit));
                    previous_linear_velocity[arm_index] = linear_velocity;
                    previous_angular_velocity[arm_index] = angular_velocity;
                    ++arm_index;
                }
            }
            ratios.joint_acceleration = std::max(
                ratios.joint_acceleration,
                previous_joint_velocity.cwiseAbs().maxCoeff() /
                    (dt * joint_acceleration_limit));
            for (size_t arm_index = 0; arm_index < 2; ++arm_index)
            {
                ratios.linear_acceleration = std::max(
                    ratios.linear_acceleration,
                    previous_linear_velocity[arm_index].norm() /
                        (dt * linear_acceleration_limit));
                ratios.angular_acceleration = std::max(
                    ratios.angular_acceleration,
                    previous_angular_velocity[arm_index].norm() /
                        (dt * angular_acceleration_limit));
            }
            return ratios;
        }

        bool resampleProjectedJointPath(
            const std::vector<Eigen::VectorXd>& path,
            const std::vector<std::size_t>& milestone_indices,
            size_t segment_count,
            const std::shared_ptr<BimanualClosedChainConstraint>& constraint,
            const std::shared_ptr<BimanualSelfCollisionChecker>& collision_checker,
            std::vector<Eigen::VectorXd>& result,
            std::string& error_message)
        {
            result.clear();
            if (path.empty() || milestone_indices.empty() ||
                milestone_indices.front() != 0 ||
                milestone_indices.back() != path.size() - 1 ||
                segment_count == 0 || !constraint)
            {
                error_message = "Cannot resample an invalid cooperative path or milestone list";
                return false;
            }
            for (size_t i = 1; i < milestone_indices.size(); ++i)
            {
                if (milestone_indices[i] <= milestone_indices[i - 1] ||
                    milestone_indices[i] >= path.size())
                {
                    error_message = "Cooperative path milestones are not strictly increasing";
                    return false;
                }
            }

            std::vector<double> cumulative_length(path.size(), 0.0);
            for (size_t i = 1; i < path.size(); ++i)
            {
                cumulative_length[i] = cumulative_length[i - 1] +
                    (path[i] - path[i - 1]).norm();
            }

            // Stop only at the original RRTConnect milestones, not at every
            // dense constrained-interpolation point. Allocate milestone time
            // by sqrt(path length) to balance acceleration demand.
            std::vector<double> cumulative_milestone_weight(
                milestone_indices.size(), 0.0);
            for (size_t i = 1; i < milestone_indices.size(); ++i)
            {
                const double milestone_length =
                    cumulative_length[milestone_indices[i]] -
                    cumulative_length[milestone_indices[i - 1]];
                cumulative_milestone_weight[i] =
                    cumulative_milestone_weight[i - 1] +
                    std::sqrt(std::max(milestone_length, 0.0));
            }
            const double total_weight = cumulative_milestone_weight.back();
            result.reserve(segment_count + 1);
            for (size_t i = 0; i <= segment_count; ++i)
            {
                Eigen::VectorXd state;
                if (i == 0 || total_weight <= 1.0e-12)
                {
                    state = path.front();
                }
                else if (i == segment_count)
                {
                    state = path.back();
                }
                else
                {
                    const double target_weight =
                        static_cast<double>(i) /
                        static_cast<double>(segment_count) * total_weight;
                    const auto upper_it = std::upper_bound(
                        cumulative_milestone_weight.begin(),
                        cumulative_milestone_weight.end(),
                        target_weight);
                    const size_t milestone_upper = std::min(
                        static_cast<size_t>(std::distance(
                            cumulative_milestone_weight.begin(), upper_it)),
                        milestone_indices.size() - 1);
                    const size_t milestone_lower =
                        milestone_upper > 0 ? milestone_upper - 1 : 0;
                    const double milestone_weight =
                        cumulative_milestone_weight[milestone_upper] -
                        cumulative_milestone_weight[milestone_lower];
                    const double local_time = milestone_weight > 1.0e-12
                        ? (target_weight -
                           cumulative_milestone_weight[milestone_lower]) /
                              milestone_weight
                        : 0.0;
                    const std::size_t path_begin =
                        milestone_indices[milestone_lower];
                    const std::size_t path_end =
                        milestone_indices[milestone_upper];
                    const double milestone_start_length =
                        cumulative_length[path_begin];
                    const double milestone_path_length =
                        cumulative_length[path_end] - milestone_start_length;
                    const double target_length = milestone_start_length +
                        quinticProgress(local_time) * milestone_path_length;
                    const auto path_upper_it = std::upper_bound(
                        cumulative_length.begin() +
                            static_cast<std::ptrdiff_t>(path_begin),
                        cumulative_length.begin() +
                            static_cast<std::ptrdiff_t>(path_end + 1),
                        target_length);
                    const size_t upper = std::min(
                        static_cast<size_t>(std::distance(
                            cumulative_length.begin(), path_upper_it)),
                        path_end);
                    const size_t lower = upper > path_begin
                        ? upper - 1 : path_begin;
                    // The dense constrained OMPL path is piecewise linear in
                    // ambient joint space. Directly following those chords
                    // makes velocity jump at every dense waypoint, which the
                    // finite-difference limiter observes as large linear and
                    // angular acceleration spikes. A component-wise monotone
                    // cubic Hermite curve preserves the path ordering while
                    // providing a continuous tangent inside each original
                    // RRTConnect milestone interval.
                    state = interpolatePathCubic(
                        path, cumulative_length, lower, upper,
                        path_begin, path_end, target_length);
                }

                // Project every state that does not meet the numerical
                // manifold, including the OMPL start/goal endpoints.
                if (!constraint->isSatisfied(state) &&
                    !constraint->project(state))
                {
                    error_message = "Closed-chain projection failed while time-resampling at sample " +
                        std::to_string(i) + "/" + std::to_string(segment_count);
                    result.clear();
                    return false;
                }
                if (!constraint->isSatisfied(state))
                {
                    error_message = "Time-resampled state violates the closed-chain constraint at sample " +
                        std::to_string(i) + "/" + std::to_string(segment_count);
                    result.clear();
                    return false;
                }
                if ((state.array() < constraint->lowerBounds().array()).any() ||
                    (state.array() > constraint->upperBounds().array()).any())
                {
                    error_message = "Time-resampled state violates joint bounds at sample " +
                        std::to_string(i) + "/" + std::to_string(segment_count);
                    result.clear();
                    return false;
                }
                if (collision_checker)
                {
                    const auto collision = collision_checker->check(state);
                    if (!collision.query_valid)
                    {
                        error_message = "Collision query failed during cooperative time-resampling: " +
                            collision.message;
                        result.clear();
                        return false;
                    }
                    if (!collision.collision_free)
                    {
                        error_message = "Cooperative time-resampled state is in collision: " +
                            collision.message;
                        result.clear();
                        return false;
                    }
                }
                result.push_back(std::move(state));
            }
            return true;
        }
    } // namespace

    CartesianTrajectoryManager::CartesianTrajectoryManager()
        : logger_(rclcpp::get_logger("cartesian_trajectory_manager"))
    {
    }

    CartesianTrajectoryManager::~CartesianTrajectoryManager() = default;

    void CartesianTrajectoryManager::setKinematicsSolver(
        const std::shared_ptr<ArmKinematics>& kinematics)
    {
        arm_kinematics_ = kinematics;
    }

    void CartesianTrajectoryManager::setCooperativeCollisionGeometry(
        pinocchio::GeometryModel geometry_model,
        double minimum_allowed_distance)
    {
        cooperative_collision_geometry_ = std::move(geometry_model);
        cooperative_minimum_collision_distance_ = minimum_allowed_distance;
    }

    void CartesianTrajectoryManager::clearCooperativeCollisionGeometry()
    {
        cooperative_collision_geometry_.reset();
        cooperative_minimum_collision_distance_ = 0.0;
    }

    bool CartesianTrajectoryManager::planSingleArmMoveL(
        const std::vector<double>& start_joint_pos,
        const arms_ros2_control_msgs::msg::LinearMessage& target_point_msg,
        const double period)
    {
        last_error_.clear();
        completed_ = false;
        dual_arm_mode_ = false;
        left_arm_active_ = false;
        right_arm_active_ = false;
        if (!arm_kinematics_)
        {
            last_error_ = "kinematics is not set";
            std::cout << last_error_ << std::endl;
            return false;
        }
        arm_type_ = target_point_msg.arm_name;
        RobotState js(arm_kinematics_->getLeftArmJointCount(),
                      arm_kinematics_->getRightArmJointCount());
        if (arm_type_ == "left")
        {
            js.leftArmJoints = Eigen::Map<const Eigen::VectorXd>(
                start_joint_pos.data(), start_joint_pos.size());
        }
        else if (arm_type_ == "right")
        {
            js.rightArmJoints = Eigen::Map<const Eigen::VectorXd>(
                start_joint_pos.data(), start_joint_pos.size());
        }
        else
        {
            last_error_ = "invalid arm type: " + arm_type_;
            std::cout << last_error_ << std::endl;
            return false;
        }
        Eigen::VectorXd current_joint_pos = Eigen::Map<const Eigen::VectorXd>(
            start_joint_pos.data(), start_joint_pos.size());

        EndEffectorPose start_pose =
            arm_kinematics_->computeSingleEndEffectorPose(js, arm_type_);

        std::unique_ptr<planning::moveL> planner = std::make_unique<planning::moveL>();
        planning::TrajectPoint ps = setCartesianPoint(start_pose);

        planning::TrajectPoint pe = setCartesianPoint(target_point_msg.endpoint);

        planning::TrajectoryParameter para = setCartesianParameter(target_point_msg);
        planning::TrajectoryInitParameters init_para(ps, pe, para, period);
        if (!planner->init(init_para))
        {
            last_error_ = "Failed to initialize moveL planner";
            std::cout << last_error_ << std::endl;
            return false;
        }
        ArmKinematics::SolverType ik_tp;
        if (target_point_msg.ik_type == "BFGS")
        {
            ik_tp = ArmKinematics::SolverType::BFGS;
            std::cout << "BFGS is set" << std::endl;
        }
        else if (target_point_msg.ik_type == "DLS")
        {
            ik_tp = ArmKinematics::SolverType::DLS;
            std::cout << "DLS is set" << std::endl;
        }
        else
        {
            ik_tp = ArmKinematics::SolverType::AUTO;
            std::cout << "Auto is set" << std::endl;
        }
        arm_kinematics_->setSolverType(ik_tp);
        planner->setRealStartTime(0.0);
        planningTime_ = planner->getTotalTime();
        path_type_ = PathType::LINE;
        if (arm_type_ == "left")
        {
            left_movel_planner_ = std::move(planner);
            left_current_joint_pos_ = current_joint_pos;
            right_movel_planner_.reset();
            right_current_joint_pos_.resize(0);
            left_arm_active_ = true;
            right_arm_active_ = false;
        }
        else
        {
            right_movel_planner_ = std::move(planner);
            right_current_joint_pos_ = current_joint_pos;
            left_movel_planner_.reset();
            left_current_joint_pos_.resize(0);
            left_arm_active_ = false;
            right_arm_active_ = true;
        }
        return true;
    }

    bool CartesianTrajectoryManager::planSingleArmMoveC(
        const std::vector<double>& start_joint_pos,
        const arms_ros2_control_msgs::msg::CircleMessage& target_circle_msg,
        const double period)
    {
        last_error_.clear();
        completed_ = false;
        dual_arm_mode_ = false;
        left_arm_active_ = false;
        right_arm_active_ = false;
        if (!arm_kinematics_)
        {
            last_error_ = "kinematics is not set";
            std::cout << last_error_ << std::endl;
            return false;
        }
        std::string error_msg;
        if (!validateCircleMsg(target_circle_msg, error_msg))
        {
            last_error_ = error_msg;
            std::cout << last_error_ << std::endl;
            return false;
        }

        arm_type_ = target_circle_msg.arm_name;
        RobotState js(arm_kinematics_->getLeftArmJointCount(),
                      arm_kinematics_->getRightArmJointCount());
        if (arm_type_ == "left")
        {
            js.leftArmJoints = Eigen::Map<const Eigen::VectorXd>(
                start_joint_pos.data(), start_joint_pos.size());
        }
        else if (arm_type_ == "right")
        {
            js.rightArmJoints = Eigen::Map<const Eigen::VectorXd>(
                start_joint_pos.data(), start_joint_pos.size());
        }
        else
        {
            last_error_ = "invalid arm type: " + arm_type_;
            std::cout << last_error_ << std::endl;
            return false;
        }
        Eigen::VectorXd current_joint_pos = Eigen::Map<const Eigen::VectorXd>(
            start_joint_pos.data(), start_joint_pos.size());

        EndEffectorPose start_pose =
            arm_kinematics_->computeSingleEndEffectorPose(js, arm_type_);

        // 更新起点
        planning::TrajectPoint ps = setCartesianPoint(start_pose);
        // 更新终点
        planning::TrajectPoint pe = setCartesianPoint(target_circle_msg.endpoint);

        planning::TrajectoryParameter para = setCartesianParameter(target_circle_msg);

        std::unique_ptr<planning::CircularCurver> planner = std::make_unique<planning::CircularCurver>();

        if (!target_circle_msg.use_three_point_method)
        {
            planning::TrajectoryInitParameters init_para(ps, pe, para, period);
            if (!planner->init(init_para))
            {
                last_error_ = "Failed to initialize MoveC planner";
                return false;
            }
        }
        else
        {
            planning::TrajectPoint pm = setCartesianPoint(target_circle_msg.midpoint);
            planning::TrajectoryInitParameters init_para(ps, pm, pe, para, period);
            if (!planner->init(init_para))
            {
                last_error_ = "Failed to initialize MoveC planner";
                return false;
            }
        }
        ArmKinematics::SolverType ik_tp;
        if (target_circle_msg.ik_type == "BFGS")
        {
            ik_tp = ArmKinematics::SolverType::BFGS;
        }
        else if (target_circle_msg.ik_type == "DLS")
        {
            ik_tp = ArmKinematics::SolverType::DLS;
        }
        else
        {
            ik_tp = ArmKinematics::SolverType::AUTO;
        }

        arm_kinematics_->setSolverType(ik_tp);
        planner->setRealStartTime(0.0);
        planningTime_ = planner->getTotalTime();
        path_type_ = PathType::CIRCLE;
        if (arm_type_ == "left")
        {
            left_movec_planner_ = std::move(planner);
            left_current_joint_pos_ = current_joint_pos;
            right_movec_planner_.reset();
            right_current_joint_pos_.resize(0);
            left_arm_active_ = true;
            right_arm_active_ = false;
        }
        else
        {
            right_movec_planner_ = std::move(planner);
            right_current_joint_pos_ = current_joint_pos;
            left_movec_planner_.reset();
            left_current_joint_pos_.resize(0);
            left_arm_active_ = false;
            right_arm_active_ = true;
        }
        return true;
    }

    bool CartesianTrajectoryManager::planDualArmMoveL(
        const std::vector<double>& start_joint_pos,
        const arms_ros2_control_msgs::msg::LinearMessage& target_point_msg,
        const double period)
    {
        last_error_.clear();
        completed_ = false;
        dual_arm_mode_ = true;
        left_arm_active_ = true;
        right_arm_active_ = true;

        if (!arm_kinematics_)
        {
            last_error_ = "kinematics is not set";
            return false;
        }

        const size_t left_count = arm_kinematics_->getLeftArmJointCount();
        const size_t right_count = arm_kinematics_->getRightArmJointCount();
        if (start_joint_pos.size() != left_count + right_count)
        {
            last_error_ = "Dual-arm MoveL requires " + std::to_string(left_count + right_count) +
                " start joints, got " + std::to_string(start_joint_pos.size());
            return false;
        }

        arms_ros2_control_msgs::msg::LinearMessage left_msg = target_point_msg;
        left_msg.arm_name = "left";
        arms_ros2_control_msgs::msg::LinearMessage right_msg = target_point_msg;
        right_msg.arm_name = "right";
        right_msg.endpoint = target_point_msg.right_endpoint;

        std::vector<double> left_start(start_joint_pos.begin(), start_joint_pos.begin() + left_count);
        std::vector<double> right_start(start_joint_pos.begin() + left_count, start_joint_pos.end());

        if (!planSingleArmMoveL(left_start, left_msg, period))
        {
            dual_arm_mode_ = false;
            return false;
        }

        const double left_time = planningTime_;
        auto left_planner = std::move(left_movel_planner_);
        auto left_joint_pos = left_current_joint_pos_;

        if (!planSingleArmMoveL(right_start, right_msg, period))
        {
            dual_arm_mode_ = false;
            left_movel_planner_.reset();
            left_current_joint_pos_.resize(0);
            return false;
        }

        const double right_time = planningTime_;
        auto right_planner = std::move(right_movel_planner_);
        auto right_joint_pos = right_current_joint_pos_;

        left_movel_planner_ = std::move(left_planner);
        left_current_joint_pos_ = left_joint_pos;
        right_movel_planner_ = std::move(right_planner);
        right_current_joint_pos_ = right_joint_pos;
        path_type_ = PathType::LINE;
        planningTime_ = std::max(left_time, right_time);
        arm_type_ = "both";
        dual_arm_mode_ = true;
        left_arm_active_ = true;
        right_arm_active_ = true;
        completed_ = false;
        return true;
    }

    bool CartesianTrajectoryManager::planDualArmMoveC(
        const std::vector<double>& start_joint_pos,
        const arms_ros2_control_msgs::msg::CircleMessage& target_circle_msg,
        const double period)
    {
        last_error_.clear();
        completed_ = false;
        dual_arm_mode_ = true;
        left_arm_active_ = true;
        right_arm_active_ = true;

        if (!arm_kinematics_)
        {
            last_error_ = "kinematics is not set";
            return false;
        }

        const size_t left_count = arm_kinematics_->getLeftArmJointCount();
        const size_t right_count = arm_kinematics_->getRightArmJointCount();
        if (start_joint_pos.size() != left_count + right_count)
        {
            last_error_ = "Dual-arm MoveC requires " + std::to_string(left_count + right_count) +
                " start joints, got " + std::to_string(start_joint_pos.size());
            return false;
        }

        arms_ros2_control_msgs::msg::CircleMessage left_msg = target_circle_msg;
        left_msg.arm_name = "left";
        arms_ros2_control_msgs::msg::CircleMessage right_msg = target_circle_msg;
        right_msg.arm_name = "right";
        right_msg.midpoint = target_circle_msg.right_midpoint;
        right_msg.endpoint = target_circle_msg.right_endpoint;
        right_msg.center = target_circle_msg.right_center;
        right_msg.axis = target_circle_msg.right_axis;
        right_msg.rotate_angle = target_circle_msg.right_rotate_angle;

        std::vector<double> left_start(start_joint_pos.begin(), start_joint_pos.begin() + left_count);
        std::vector<double> right_start(start_joint_pos.begin() + left_count, start_joint_pos.end());

        if (!planSingleArmMoveC(left_start, left_msg, period))
        {
            dual_arm_mode_ = false;
            return false;
        }

        const double left_time = planningTime_;
        auto left_planner = std::move(left_movec_planner_);
        auto left_joint_pos = left_current_joint_pos_;

        if (!planSingleArmMoveC(right_start, right_msg, period))
        {
            dual_arm_mode_ = false;
            left_movec_planner_.reset();
            left_current_joint_pos_.resize(0);
            return false;
        }

        const double right_time = planningTime_;
        auto right_planner = std::move(right_movec_planner_);
        auto right_joint_pos = right_current_joint_pos_;

        left_movec_planner_ = std::move(left_planner);
        left_current_joint_pos_ = left_joint_pos;
        right_movec_planner_ = std::move(right_planner);
        right_current_joint_pos_ = right_joint_pos;
        path_type_ = PathType::CIRCLE;
        planningTime_ = std::max(left_time, right_time);
        arm_type_ = "both";
        dual_arm_mode_ = true;
        left_arm_active_ = true;
        right_arm_active_ = true;
        completed_ = false;
        return true;
    }

    bool CartesianTrajectoryManager::planCooperativeMotion(
        const std::vector<double>& start_joint_pos,
        const arms_ros2_control_msgs::msg::CooperativeMotion& motion,
        const double period)
    {
        clearPlanner();
        const size_t left_count = arm_kinematics_ ? arm_kinematics_->getLeftArmJointCount() : 0;
        const size_t right_count = arm_kinematics_ ? arm_kinematics_->getRightArmJointCount() : 0;
        if (!arm_kinematics_)
        {
            last_error_ = "kinematics is not set";
            return false;
        }
        if (start_joint_pos.size() != left_count + right_count)
        {
            last_error_ = "Cooperative motion requires " + std::to_string(left_count + right_count) +
                " start joints, got " + std::to_string(start_joint_pos.size());
            return false;
        }
        if (!std::isfinite(period) || period <= 0.0 ||
            !std::isfinite(motion.duration) || motion.duration <= 0.0)
        {
            last_error_ = "Cooperative motion requires positive duration and control period";
            return false;
        }
        const std::string& kinematics_base_frame = arm_kinematics_->getBaseFrameName();
        if (kinematics_base_frame.empty())
        {
            last_error_ = "Kinematics base frame is not configured";
            return false;
        }
        if (!motion.frame_id.empty() && motion.frame_id != kinematics_base_frame)
        {
            last_error_ = "Cooperative poses must be expressed in kinematics base frame '" +
                kinematics_base_frame + "', got '" + motion.frame_id + "'";
            return false;
        }

        Eigen::Isometry3d requested_left_start;
        Eigen::Isometry3d requested_right_start;
        Eigen::Isometry3d requested_left_goal;
        Eigen::Isometry3d requested_right_goal;
        if (!poseMsgToIsometry(motion.left_start, requested_left_start) ||
            !poseMsgToIsometry(motion.right_start, requested_right_start) ||
            !poseMsgToIsometry(motion.left_goal, requested_left_goal) ||
            !poseMsgToIsometry(motion.right_goal, requested_right_goal))
        {
            last_error_ = "Cooperative motion contains a non-finite pose or zero-norm quaternion";
            return false;
        }

        const double start_position_tolerance = motion.start_position_tolerance > 0.0
                                                    ? motion.start_position_tolerance : 0.01;
        const double start_orientation_tolerance = motion.start_orientation_tolerance > 0.0
                                                       ? motion.start_orientation_tolerance : 0.05;
        const double relative_position_tolerance = motion.relative_position_tolerance > 0.0
                                                       ? motion.relative_position_tolerance : 0.002;
        const double relative_orientation_tolerance = motion.relative_orientation_tolerance > 0.0
                                                          ? motion.relative_orientation_tolerance : 0.01;
        const int ik_max_iterations = motion.ik_max_iterations > 0
                                          ? motion.ik_max_iterations : 300;
        const double ik_tolerance = motion.ik_tolerance > 0.0
                                        ? motion.ik_tolerance : 1.0e-4;
        const std::array<double, 7> positive_limit_parameters{
            motion.max_translation_step,
            motion.max_rotation_step,
            motion.max_linear_velocity,
            motion.max_linear_acceleration,
            motion.max_angular_velocity,
            motion.max_angular_acceleration,
            motion.max_joint_acceleration};
        if (std::any_of(
                positive_limit_parameters.begin(), positive_limit_parameters.end(),
                [](double value) { return !std::isfinite(value) || value <= 0.0; }) ||
            !std::isfinite(motion.joint_velocity_scale) ||
            motion.joint_velocity_scale <= 0.0 || motion.joint_velocity_scale > 1.0)
        {
            last_error_ = "Cooperative sampling and motion limits must be finite and positive; "
                "joint_velocity_scale must be in (0, 1]";
            return false;
        }
        const double max_linear_velocity = motion.max_linear_velocity;
        const double max_linear_acceleration = motion.max_linear_acceleration;
        const double max_angular_velocity = motion.max_angular_velocity;
        const double max_angular_acceleration = motion.max_angular_acceleration;
        const double joint_velocity_scale = motion.joint_velocity_scale;
        const double max_joint_acceleration = motion.max_joint_acceleration;
        Eigen::VectorXd joint_velocity_limits =
            arm_kinematics_->getJointVelocityLimits("both") * joint_velocity_scale;
        if (joint_velocity_limits.size() != static_cast<Eigen::Index>(left_count + right_count) ||
            !joint_velocity_limits.allFinite() ||
            (joint_velocity_limits.array() <= 0.0).any())
        {
            last_error_ = "Valid positive URDF velocity limits are required for all cooperative joints";
            return false;
        }

        RobotState current_state(left_count, right_count);
        current_state.leftArmJoints = Eigen::Map<const Eigen::VectorXd>(
            start_joint_pos.data(), static_cast<Eigen::Index>(left_count));
        current_state.rightArmJoints = Eigen::Map<const Eigen::VectorXd>(
            start_joint_pos.data() + left_count, static_cast<Eigen::Index>(right_count));
        EndEffectorPose actual_left_pose;
        EndEffectorPose actual_right_pose;
        arm_kinematics_->computeBothEndEffectorPose(
            current_state, actual_left_pose, actual_right_pose);
        const Eigen::Isometry3d actual_left_start = toIsometry(actual_left_pose);
        const Eigen::Isometry3d actual_right_start = toIsometry(actual_right_pose);

        const double left_start_position_error =
            (actual_left_start.translation() - requested_left_start.translation()).norm();
        const double right_start_position_error =
            (actual_right_start.translation() - requested_right_start.translation()).norm();
        const double left_start_orientation_error =
            rotationError(actual_left_start.rotation(), requested_left_start.rotation());
        const double right_start_orientation_error =
            rotationError(actual_right_start.rotation(), requested_right_start.rotation());
        if (std::max(left_start_position_error, right_start_position_error) > start_position_tolerance ||
            std::max(left_start_orientation_error, right_start_orientation_error) > start_orientation_tolerance)
        {
            std::ostringstream oss;
            oss << "Requested cooperative start pose does not match current FK: position_error="
                << std::max(left_start_position_error, right_start_position_error)
                << ", orientation_error="
                << std::max(left_start_orientation_error, right_start_orientation_error);
            last_error_ = oss.str();
            return false;
        }

        const Eigen::Isometry3d requested_start_left_to_right =
            requested_left_start.inverse() * requested_right_start;
        const Eigen::Isometry3d requested_goal_left_to_right =
            requested_left_goal.inverse() * requested_right_goal;
        relative_position_error_ =
            (requested_start_left_to_right.translation() -
             requested_goal_left_to_right.translation()).norm();
        relative_orientation_error_ = rotationError(
            requested_start_left_to_right.rotation(), requested_goal_left_to_right.rotation());
        if (relative_position_error_ > relative_position_tolerance ||
            relative_orientation_error_ > relative_orientation_tolerance)
        {
            std::ostringstream oss;
            oss << "Goal left-to-right transform differs from the start transform: position_error="
                << relative_position_error_ << ", orientation_error="
                << relative_orientation_error_;
            last_error_ = oss.str();
            return false;
        }

        const Eigen::Isometry3d fixed_left_to_right =
            actual_left_start.inverse() * actual_right_start;
        const double actual_relative_position_error =
            (fixed_left_to_right.translation() - requested_start_left_to_right.translation()).norm();
        const double actual_relative_orientation_error = rotationError(
            fixed_left_to_right.rotation(), requested_start_left_to_right.rotation());
        if (actual_relative_position_error > relative_position_tolerance ||
            actual_relative_orientation_error > relative_orientation_tolerance)
        {
            std::ostringstream oss;
            oss << "Current FK left-to-right transform differs from the requested start transform: "
                << "position_error=" << actual_relative_position_error
                << ", orientation_error=" << actual_relative_orientation_error;
            last_error_ = oss.str();
            return false;
        }

        // Solver settings are request-local. In particular, do not inherit
        // random-restart state left behind by the public kinematics service.
        SolverParamsGuard solver_params_guard(*arm_kinematics_);
        ArmKinematics::SolverParams cooperative_solver_params =
            arm_kinematics_->getSolverParams();
        cooperative_solver_params.solverType =
            motion.ik_type == "BFGS"
                ? ArmKinematics::SolverType::BFGS
                : motion.ik_type == "DLS"
                    ? ArmKinematics::SolverType::DLS
                    : ArmKinematics::SolverType::AUTO;
        cooperative_solver_params.maxIterations = ik_max_iterations;
        cooperative_solver_params.solutionTolerance = ik_tolerance;
        cooperative_solver_params.randomRestart = false;
        arm_kinematics_->setSolverParams(cooperative_solver_params);

        constexpr size_t max_trajectory_samples = 1000000;
        const Eigen::VectorXd start_state = Eigen::Map<const Eigen::VectorXd>(
            start_joint_pos.data(),
            static_cast<Eigen::Index>(start_joint_pos.size()));
        const auto& model = arm_kinematics_->getModel();
        if (static_cast<Eigen::Index>(model.nq) != start_state.size())
        {
            last_error_ = "Closed-chain planning requires a fixed-base Pinocchio model whose nq matches the dual-arm joint count";
            return false;
        }

        // The action tolerances describe what the task is willing to accept;
        // they are far too loose for the numerical manifold used to generate
        // control-rate samples.  Reusing (for example) a 3 mm task tolerance
        // here creates a dead band where some cubic samples are left untouched
        // while neighbouring samples are projected.  The resulting micromotion
        // is amplified by the second finite difference into acceleration spikes.
        const double requested_relative_tolerance = std::min(
            relative_position_tolerance, relative_orientation_tolerance);
        const double manifold_tolerance = std::max(
            1.0e-10,
            std::min(1.0e-6, 0.1 * requested_relative_tolerance));
        RCLCPP_DEBUG(
            logger_,
            "Cooperative internal manifold tolerance: %.3e "
            "(task position %.3e, orientation %.3e)",
            manifold_tolerance, relative_position_tolerance,
            relative_orientation_tolerance);
        auto constraint = std::make_shared<BimanualClosedChainConstraint>(
            model,
            arm_kinematics_->getLeftArmJointNames(),
            arm_kinematics_->getRightArmJointNames(),
            arm_kinematics_->getLeftEndEffectorName(),
            arm_kinematics_->getRightEndEffectorName(),
            start_state,
            pinocchio::SE3(fixed_left_to_right.rotation(),
                           fixed_left_to_right.translation()),
            manifold_tolerance);
        constraint->setMaxIterations(200);

        // The real grasp relation, not a slightly inconsistent requested right
        // pose, defines the closed chain throughout the motion.
        const Eigen::Isometry3d rigid_right_goal =
            requested_left_goal * fixed_left_to_right;
        const ArmIkCandidates left_ik = collectArmIkCandidates(
            *arm_kinematics_, toEndEffectorPose(requested_left_goal),
            start_state.head(static_cast<Eigen::Index>(left_count)), "left",
            ik_max_iterations, ik_tolerance);
        const ArmIkCandidates right_ik = collectArmIkCandidates(
            *arm_kinematics_, toEndEffectorPose(rigid_right_goal),
            start_state.tail(static_cast<Eigen::Index>(right_count)), "right",
            ik_max_iterations, ik_tolerance);
        RCLCPP_INFO(
            logger_,
            "Cooperative endpoint IK candidates: left=%zu/%zu seeds, right=%zu/%zu seeds",
            left_ik.solutions.size(), left_ik.attempted_seeds,
            right_ik.solutions.size(), right_ik.attempted_seeds);
        if (left_ik.solutions.empty() || right_ik.solutions.empty())
        {
            std::ostringstream oss;
            oss << "Cooperative goal IK failed after deterministic multi-start";
            if (left_ik.solutions.empty())
            {
                oss << "; left="
                    << (left_ik.has_failure ? left_ik.best_failure.status : "no candidate")
                    << " (best_error="
                    << (left_ik.has_failure
                            ? left_ik.best_failure.poseErrorNorm
                            : std::numeric_limits<double>::infinity())
                    << ")";
            }
            if (right_ik.solutions.empty())
            {
                oss << "; right="
                    << (right_ik.has_failure ? right_ik.best_failure.status : "no candidate")
                    << " (best_error="
                    << (right_ik.has_failure
                            ? right_ik.best_failure.poseErrorNorm
                            : std::numeric_limits<double>::infinity())
                    << ")";
            }
            last_error_ = oss.str();
            return false;
        }

        std::vector<std::pair<double, Eigen::VectorXd>> paired_candidates;
        paired_candidates.reserve(
            left_ik.solutions.size() * right_ik.solutions.size());
        for (const auto& left_solution : left_ik.solutions)
        {
            for (const auto& right_solution : right_ik.solutions)
            {
                Eigen::VectorXd candidate(start_state.size());
                candidate.head(static_cast<Eigen::Index>(left_count)) =
                    left_solution;
                candidate.tail(static_cast<Eigen::Index>(right_count)) =
                    right_solution;
                paired_candidates.emplace_back(
                    (candidate - start_state).squaredNorm(),
                    std::move(candidate));
            }
        }
        std::sort(
            paired_candidates.begin(), paired_candidates.end(),
            [](const auto& lhs, const auto& rhs)
            {
                return lhs.first < rhs.first;
            });

        constexpr std::size_t max_projected_goal_candidates = 16;
        std::vector<Eigen::VectorXd> goal_candidates;
        goal_candidates.reserve(std::min(
            max_projected_goal_candidates, paired_candidates.size()));
        for (auto& [distance, candidate] : paired_candidates)
        {
            (void)distance;
            if (!constraint->project(candidate) ||
                (candidate.array() < constraint->lowerBounds().array()).any() ||
                (candidate.array() > constraint->upperBounds().array()).any())
            {
                continue;
            }

            RobotState projected_goal_state(left_count, right_count);
            projected_goal_state.leftArmJoints = candidate.head(
                static_cast<Eigen::Index>(left_count));
            projected_goal_state.rightArmJoints = candidate.tail(
                static_cast<Eigen::Index>(right_count));
            EndEffectorPose projected_left_goal_pose;
            EndEffectorPose projected_right_goal_pose;
            arm_kinematics_->computeBothEndEffectorPose(
                projected_goal_state,
                projected_left_goal_pose,
                projected_right_goal_pose);
            const Eigen::Isometry3d projected_left_goal =
                toIsometry(projected_left_goal_pose);
            const Eigen::Isometry3d projected_right_goal =
                toIsometry(projected_right_goal_pose);
            const bool endpoint_valid =
                (projected_left_goal.translation() -
                    requested_left_goal.translation()).norm() <=
                    relative_position_tolerance &&
                rotationError(projected_left_goal.rotation(),
                              requested_left_goal.rotation()) <=
                    relative_orientation_tolerance &&
                (projected_right_goal.translation() -
                    rigid_right_goal.translation()).norm() <=
                    relative_position_tolerance &&
                rotationError(projected_right_goal.rotation(),
                              rigid_right_goal.rotation()) <=
                    relative_orientation_tolerance;
            if (!endpoint_valid)
            {
                continue;
            }
            const bool duplicate = std::any_of(
                goal_candidates.begin(), goal_candidates.end(),
                [&candidate](const Eigen::VectorXd& existing)
                {
                    return (existing - candidate).norm() < 1.0e-3;
                });
            if (!duplicate)
            {
                goal_candidates.push_back(std::move(candidate));
                if (goal_candidates.size() >= max_projected_goal_candidates)
                {
                    break;
                }
            }
        }
        if (goal_candidates.empty())
        {
            last_error_ = "No deterministic cooperative IK candidate remains valid after closed-chain projection";
            return false;
        }
        RCLCPP_INFO(
            logger_,
            "Cooperative endpoint candidates after projection: %zu (nearest joint distance %.3f rad)",
            goal_candidates.size(),
            (goal_candidates.front() - start_state).norm());

        std::shared_ptr<BimanualSelfCollisionChecker> collision_checker;
        if (cooperative_collision_geometry_)
        {
            try
            {
                collision_checker =
                    std::make_shared<BimanualSelfCollisionChecker>(
                        constraint, *cooperative_collision_geometry_,
                        cooperative_minimum_collision_distance_);
            }
            catch (const std::exception& error)
            {
                last_error_ = std::string(
                    "Failed to initialize cooperative collision checking: ") +
                    error.what();
                return false;
            }
        }
        else
        {
            RCLCPP_WARN(
                logger_,
                "Cooperative OMPL planning is running without a configured collision checker");
        }

        BimanualProjectedPlanner planner(constraint, collision_checker);
        BimanualProjectedPlanner::Options planner_options;
        planner_options.planning_time_seconds = 5.0;
        planner_options.projection_max_iterations = 200;
        planner_options.planner_range = 0.5;
        const auto plan_result = planner.plan(
            start_state, goal_candidates, planner_options);
        if (!plan_result.success())
        {
            last_error_ = std::string("Closed-chain OMPL planning failed (") +
                BimanualProjectedPlanner::statusName(plan_result.status) +
                "): " + plan_result.message;
            return false;
        }
        const std::vector<Eigen::VectorXd>& geometric_joint_path =
            plan_result.path;
        if (geometric_joint_path.empty())
        {
            last_error_ = "Closed-chain OMPL planning returned an empty path";
            return false;
        }
        const std::size_t selected_goal_display =
            plan_result.selected_goal_index < goal_candidates.size()
                ? plan_result.selected_goal_index + 1
                : 0;
        RCLCPP_INFO(
            logger_,
            "Closed-chain OMPL path accepted: %zu states, %zu milestones, goal=%zu/%zu, planning %.3f s, collision_checked=%s",
            geometric_joint_path.size(), plan_result.milestone_indices.size(),
            selected_goal_display, goal_candidates.size(),
            plan_result.planning_time_seconds,
            plan_result.collision_checked ? "true" : "false");

        double actual_duration = motion.duration;
        std::vector<Eigen::Isometry3d> execution_left_path;
        std::vector<Eigen::Isometry3d> execution_right_path;
        LimitRatios last_execution_ratios;
        double last_sampled_duration = actual_duration;
        constexpr int max_time_scaling_iterations = 10;
        for (int iteration = 0; iteration < max_time_scaling_iterations; ++iteration)
        {
            const double execution_segments_required = std::ceil(actual_duration / period);
            if (!std::isfinite(execution_segments_required) ||
                execution_segments_required > static_cast<double>(max_trajectory_samples))
            {
                last_error_ = "Cooperative execution trajectory exceeds the one-million-sample "
                    "safety limit";
                return false;
            }
            const size_t execution_segments = std::max<size_t>(
                1, static_cast<size_t>(execution_segments_required));
            const double sampled_duration = static_cast<double>(execution_segments) * period;
            std::vector<Eigen::VectorXd> execution_with_start;
            if (!resampleProjectedJointPath(
                    geometric_joint_path, plan_result.milestone_indices,
                    execution_segments,
                    constraint, collision_checker,
                    execution_with_start, last_error_))
            {
                return false;
            }
            execution_left_path.clear();
            execution_right_path.clear();
            execution_left_path.reserve(execution_with_start.size());
            execution_right_path.reserve(execution_with_start.size());

            bool fk_valid = true;
            for (size_t i = 0; i < execution_with_start.size(); ++i)
            {
                RobotState state(left_count, right_count);
                state.leftArmJoints = execution_with_start[i].head(
                    static_cast<Eigen::Index>(left_count));
                state.rightArmJoints = execution_with_start[i].tail(
                    static_cast<Eigen::Index>(right_count));
                EndEffectorPose left_pose;
                EndEffectorPose right_pose;
                arm_kinematics_->computeBothEndEffectorPose(state, left_pose, right_pose);
                const Eigen::Isometry3d left = toIsometry(left_pose);
                const Eigen::Isometry3d right = toIsometry(right_pose);
                const Eigen::Isometry3d relative = left.inverse() * right;
                if ((relative.translation() - fixed_left_to_right.translation()).norm() >
                        relative_position_tolerance ||
                    rotationError(relative.rotation(), fixed_left_to_right.rotation()) >
                        relative_orientation_tolerance)
                {
                    fk_valid = false;
                    last_error_ = "Time-resampled cooperative trajectory violates relative-pose tolerance";
                    break;
                }
                execution_left_path.push_back(left);
                execution_right_path.push_back(right);
            }
            if (!fk_valid)
            {
                return false;
            }

            const LimitRatios execution_ratios = evaluateLimits(
                execution_with_start, execution_left_path, execution_right_path,
                sampled_duration, joint_velocity_limits, max_joint_acceleration,
                max_linear_velocity, max_linear_acceleration,
                max_angular_velocity, max_angular_acceleration);
            last_execution_ratios = execution_ratios;
            last_sampled_duration = sampled_duration;
            const double scale = execution_ratios.durationScale();
            RCLCPP_DEBUG(
                logger_,
                "Cooperative time scaling iteration %d: duration=%.3f scale=%.6f "
                "joint_v=%.6f joint_a=%.6f linear_v=%.6f linear_a=%.6f "
                "angular_v=%.6f angular_a=%.6f",
                iteration, sampled_duration, scale,
                execution_ratios.joint_velocity,
                execution_ratios.joint_acceleration,
                execution_ratios.linear_velocity,
                execution_ratios.linear_acceleration,
                execution_ratios.angular_velocity,
                execution_ratios.angular_acceleration);
            if (scale <= 1.0 + 1.0e-6)
            {
                precomputed_joint_trajectory_.assign(
                    execution_with_start.begin() + 1, execution_with_start.end());
                actual_duration = sampled_duration;
                break;
            }
            if (!motion.auto_extend_duration)
            {
                std::ostringstream oss;
                oss << "Requested duration violates limits after control-rate resampling; "
                    << "minimum duration is about " << sampled_duration * scale << " s";
                last_error_ = oss.str();
                return false;
            }
            // Velocity scales with 1/T and acceleration with 1/T^2;
            // durationScale() already applies sqrt() to acceleration ratios.
            // Multiplying by the raw acceleration ratio here would square the
            // required extension and can turn a short motion into minutes.
            // A two-percent guard absorbs control-period quantization and the
            // small residual left by manifold projection.  Always advance by
            // at least one controller sample so a near-unity ratio cannot
            // repeat the same discretization indefinitely.
            actual_duration = std::max(
                sampled_duration * scale * 1.02,
                sampled_duration + period);
        }
        if (precomputed_joint_trajectory_.empty())
        {
            std::ostringstream oss;
            oss << "Failed to converge cooperative trajectory time scaling after "
                << max_time_scaling_iterations << " iterations"
                << "; duration=" << last_sampled_duration
                << ", joint_v=" << last_execution_ratios.joint_velocity
                << ", joint_a=" << last_execution_ratios.joint_acceleration
                << ", linear_v=" << last_execution_ratios.linear_velocity
                << ", linear_a=" << last_execution_ratios.linear_acceleration
                << ", angular_v=" << last_execution_ratios.angular_velocity
                << ", angular_a=" << last_execution_ratios.angular_acceleration;
            last_error_ = oss.str();
            return false;
        }

        precomputed_index_ = 0;
        cooperative_mode_ = true;
        dual_arm_mode_ = true;
        left_arm_active_ = true;
        right_arm_active_ = true;
        completed_ = false;
        planningTime_ = actual_duration;
        arm_type_ = "both";
        if (actual_duration > motion.duration + period)
        {
            RCLCPP_INFO(
                logger_,
                "Extended cooperative duration from %.3f s to %.3f s to satisfy motion limits",
                motion.duration, actual_duration);
        }
        return true;
    }

    bool CartesianTrajectoryManager::getNextJointPos(std::vector<double>& res)
    {
        last_error_.clear();
        if (cooperative_mode_)
        {
            if (precomputed_joint_trajectory_.empty())
            {
                last_error_ = "Cooperative trajectory buffer is empty";
                return false;
            }
            const Eigen::VectorXd& point = precomputed_joint_trajectory_[precomputed_index_];
            res.assign(point.data(), point.data() + point.size());
            if (precomputed_index_ + 1 < precomputed_joint_trajectory_.size())
            {
                ++precomputed_index_;
            }
            else
            {
                completed_ = true;
            }
            return true;
        }
        if (dual_arm_mode_&& path_type_

        ==
        PathType::LINE
        )
        {
            std::vector<double> left_res;
            std::vector<double> right_res;
            if (left_arm_active_ && !stepLineArm("left", left_movel_planner_, left_current_joint_pos_, left_res))
            {
                return false;
            }
            if (right_arm_active_ && !stepLineArm("right", right_movel_planner_, right_current_joint_pos_, right_res))
            {
                return false;
            }
            res.clear();
            res.insert(res.end(), left_res.begin(), left_res.end());
            res.insert(res.end(), right_res.begin(), right_res.end());
            const bool left_done = !left_arm_active_ || !left_movel_planner_ || left_movel_planner_->isMotionOver();
            const bool right_done = !right_arm_active_ || !right_movel_planner_ || right_movel_planner_->isMotionOver();
            completed_ = left_done && right_done;
            return true;
        }
        else
        if (dual_arm_mode_&& path_type_

        ==
        PathType::CIRCLE
        )
        {
            std::vector<double> left_res;
            std::vector<double> right_res;
            if (left_arm_active_ && !stepCircleArm("left", left_movec_planner_, left_current_joint_pos_, left_res))
            {
                return false;
            }
            if (right_arm_active_ && !stepCircleArm("right", right_movec_planner_, right_current_joint_pos_, right_res))
            {
                return false;
            }
            res.clear();
            res.insert(res.end(), left_res.begin(), left_res.end());
            res.insert(res.end(), right_res.begin(), right_res.end());
            const bool left_done = !left_arm_active_ || !left_movec_planner_ || left_movec_planner_->isMotionOver();
            const bool right_done = !right_arm_active_ || !right_movec_planner_ || right_movec_planner_->isMotionOver();
            completed_ = left_done && right_done;
            return true;
        }
        else
        if (left_movel_planner_&& path_type_

        ==
        PathType::LINE&& left_arm_active_
        )
        {
            return stepLineArm("left", left_movel_planner_, left_current_joint_pos_, res);
        }
        else
        if (right_movel_planner_&& path_type_

        ==
        PathType::LINE&& right_arm_active_
        )
        {
            return stepLineArm("right", right_movel_planner_, right_current_joint_pos_, res);
        }
        else
        if (left_movec_planner_&& path_type_

        ==
        PathType::CIRCLE&& left_arm_active_
        )
        {
            return stepCircleArm("left", left_movec_planner_, left_current_joint_pos_, res);
        }
        else
        if (right_movec_planner_&& path_type_

        ==
        PathType::CIRCLE&& right_arm_active_
        )
        {
            return stepCircleArm("right", right_movec_planner_, right_current_joint_pos_, res);
        }
        else
        {
            last_error_ = "moveL planner or moveC planner not initialized for DOUBLES interpolation";
            RCLCPP_ERROR(logger_, "moveL planner or moveC planner not initialized for "
                         "DOUBLES interpolation");
            return false;
        }
    };

    std::vector<std::string>
    CartesianTrajectoryManager::getMovelJointNames(std::string arm_name)
    {
        if (!arm_kinematics_)
        {
            std::cout << "kinematics is not set" << std::endl;
            return {};
        }
        if (arm_name == "left")
        {
            return arm_kinematics_->getLeftArmJointNames();
        }
        else if (arm_name == "right")
        {
            return arm_kinematics_->getRightArmJointNames();
        }
        else if (arm_name == "both")
        {
            auto left_names = arm_kinematics_->getLeftArmJointNames();
            auto right_names = arm_kinematics_->getRightArmJointNames();
            left_names.insert(left_names.end(), right_names.begin(), right_names.end());
            return left_names;
        }
        else
        {
            std::cout << "invalid arm name: " << arm_name << std::endl;
            return {};
        }
    }

    void CartesianTrajectoryManager::clearPlanner()
    {
        left_movel_planner_.reset();
        right_movel_planner_.reset();
        left_movec_planner_.reset();
        right_movec_planner_.reset();
        left_current_joint_pos_.resize(0);
        right_current_joint_pos_.resize(0);
        precomputed_joint_trajectory_.clear();
        precomputed_index_ = 0;
        cooperative_mode_ = false;
        relative_position_error_ = 0.0;
        relative_orientation_error_ = 0.0;
        dual_arm_mode_ = false;
        left_arm_active_ = false;
        right_arm_active_ = false;
        completed_ = false;
        last_error_.clear();
    }

    bool CartesianTrajectoryManager::validatePose(
        const geometry_msgs::msg::Pose& pose,
        const std::string& pose_name,
        std::string& error_message) const
    {
        if (std::isnan(pose.position.x) || std::isnan(pose.position.y) || std::isnan(pose.position.z))
        {
            error_message = pose_name + " position contains NaN values";
            return false;
        }

        const double qx = pose.orientation.x;
        const double qy = pose.orientation.y;
        const double qz = pose.orientation.z;
        const double qw = pose.orientation.w;
        if (std::isnan(qx) || std::isnan(qy) || std::isnan(qz) || std::isnan(qw))
        {
            error_message = pose_name + " orientation contains NaN values";
            return false;
        }
        return true;
    }

    bool CartesianTrajectoryManager::stepLineArm(
        const std::string& arm_name,
        std::unique_ptr<planning::moveL>& planner,
        Eigen::VectorXd& current_joint_pos,
        std::vector<double>& result)
    {
        if (!planner)
        {
            last_error_ = "MoveL planner is not initialized for " + arm_name + " arm";
            return false;
        }

        if (planner->isMotionOver())
        {
            result.assign(current_joint_pos.data(), current_joint_pos.data() + current_joint_pos.size());
            return true;
        }

        planning::TrajectPoint movel_point = planner->run();
        EndEffectorPose pose;
        pose.position = movel_point.cart_pos;
        Eigen::Quaterniond q(
            movel_point.quaternion_point.q.w, movel_point.quaternion_point.q.x,
            movel_point.quaternion_point.q.y, movel_point.quaternion_point.q.z);
        pose.setQuaternion(q);

        auto& rec = arms_controller_common::TrajectoryRecorder::instance();
        if (rec.enabled())
        {
            arms_controller_common::TrajSample cs;
            cs.stamp_sec = last_sample_stamp_;
            cs.position = {movel_point.cart_pos(0), movel_point.cart_pos(1), movel_point.cart_pos(2)};
            cs.quat_xyzw = {movel_point.quaternion_point.q.x, movel_point.quaternion_point.q.y,
                            movel_point.quaternion_point.q.z, movel_point.quaternion_point.q.w};
            rec.appendCal(arm_name, cs);
        }

        Eigen::VectorXd solution;
        ArmKinematics::SolutionInfo info;
        if (!arm_kinematics_->solveSingleArmIKWithInfo(
            pose, current_joint_pos, solution, info, arm_name, 50))
        {
            std::ostringstream oss;
            oss << "IK failed while calculating MoveL joint position for " << arm_name
                << " arm: solver=" << ArmKinematics::solverTypeName(info.usedSolver)
                << ", status=" << info.status
                << ", final_error=" << info.poseErrorNorm;
            last_error_ = oss.str();
            RCLCPP_ERROR(logger_, "%s", last_error_.c_str());
            return false;
        }

        current_joint_pos = solution;
        result.assign(solution.data(), solution.data() + solution.size());
        if (planner->isMotionOver())
        {
            completed_ = !dual_arm_mode_;
        }
        return true;
    }

    bool CartesianTrajectoryManager::stepCircleArm(
        const std::string& arm_name,
        std::unique_ptr<planning::CircularCurver>& planner,
        Eigen::VectorXd& current_joint_pos,
        std::vector<double>& result)
    {
        if (!planner)
        {
            last_error_ = "MoveC planner is not initialized for " + arm_name + " arm";
            return false;
        }

        if (planner->isMotionOver())
        {
            result.assign(current_joint_pos.data(), current_joint_pos.data() + current_joint_pos.size());
            return true;
        }

        planning::TrajectPoint movec_point = planner->run();
        EndEffectorPose pose;
        pose.position = movec_point.cart_pos;
        Eigen::Quaterniond q(
            movec_point.quaternion_point.q.w, movec_point.quaternion_point.q.x,
            movec_point.quaternion_point.q.y, movec_point.quaternion_point.q.z);
        pose.setQuaternion(q);

        auto& rec = arms_controller_common::TrajectoryRecorder::instance();
        if (rec.enabled())
        {
            arms_controller_common::TrajSample cs;
            cs.stamp_sec = last_sample_stamp_;
            cs.position = {movec_point.cart_pos(0), movec_point.cart_pos(1), movec_point.cart_pos(2)};
            cs.quat_xyzw = {movec_point.quaternion_point.q.x, movec_point.quaternion_point.q.y,
                            movec_point.quaternion_point.q.z, movec_point.quaternion_point.q.w};
            rec.appendCal(arm_name, cs);
        }

        Eigen::VectorXd solution;
        ArmKinematics::SolutionInfo info;
        if (!arm_kinematics_->solveSingleArmIKWithInfo(
            pose, current_joint_pos, solution, info, arm_name, 50, 1e-4))
        {
            std::ostringstream oss;
            oss << "IK failed while calculating MoveC joint position for " << arm_name
                << " arm: solver=" << ArmKinematics::solverTypeName(info.usedSolver)
                << ", status=" << info.status
                << ", final_error=" << info.poseErrorNorm;
            last_error_ = oss.str();
            RCLCPP_ERROR(logger_, "%s", last_error_.c_str());
            return false;
        }

        current_joint_pos = solution;
        result.assign(solution.data(), solution.data() + solution.size());
        if (planner->isMotionOver())
        {
            completed_ = !dual_arm_mode_;
        }
        return true;
    }

    bool CartesianTrajectoryManager::validateCircleMsg(
        const arms_ros2_control_msgs::msg::CircleMessage& target_circle_msg,
        std::string& error_message)
    {
        // 检查时间模式下的时间参数
        if (target_circle_msg.time_mode && target_circle_msg.duration < 0.001)
        {
            error_message = "In time mode, a valid time parameter (duration > 0.001) "
                "must be provided";
            return false;
        }

        // 检查速度模式下的速度参数
        if (!target_circle_msg.time_mode)
        {
            if (fabs(target_circle_msg.max_linear_velocity) < min_val)
            {
                error_message =
                    "In speed mode, the maximum linear speed must be provided";
                return false;
            }
            if (fabs(target_circle_msg.max_angular_velocity) < min_val)
            {
                error_message = "When using attitude interpolation in velocity mode, the "
                    "maximum angular velocity must be provided";
                return false;
            }
            if (fabs(target_circle_msg.max_linear_acceleration) < min_val)
            {
                error_message =
                    "In speed mode, the maximum linear acceleration must be provided";
                return false;
            }
            if (fabs(target_circle_msg.max_angular_acceleration) < min_val)
            {
                error_message = "When using pose interpolation in velocity mode, the "
                    "maximum angular acceleration must be provided";
                return false;
            }
            if (fabs(target_circle_msg.max_linear_jerk) < min_val)
            {
                error_message =
                    "In speed mode, the maximum linear acceleration must be provided";
                return false;
            }
            if (fabs(target_circle_msg.max_angular_jerk) < min_val)
            {
                error_message = "When using attitude interpolation in velocity mode, the "
                    "maximum angular jerk must be provided";
                return false;
            }
        }

        if (target_circle_msg.use_three_point_method)
        {
            // 检查输入的中间点和终点是不是有问题
            if (fabs(target_circle_msg.midpoint.position.x) < min_val &&
                fabs(target_circle_msg.midpoint.position.y) < min_val &&
                fabs(target_circle_msg.midpoint.position.z) < min_val &&
                fabs(target_circle_msg.midpoint.orientation.x) < min_val &&
                fabs(target_circle_msg.midpoint.orientation.y) < min_val &&
                fabs(target_circle_msg.midpoint.orientation.z) < min_val &&
                fabs(target_circle_msg.midpoint.orientation.w) < min_val)
            {
                error_message = "The midpoint of the arc must be inputted";
                return false;
            }
            if (std::isnan(target_circle_msg.midpoint.position.x) ||
                std::isnan(target_circle_msg.midpoint.position.y) ||
                std::isnan(target_circle_msg.midpoint.position.z) ||
                std::isnan(target_circle_msg.midpoint.orientation.x) ||
                std::isnan(target_circle_msg.midpoint.orientation.y) ||
                std::isnan(target_circle_msg.midpoint.orientation.z) ||
                std::isnan(target_circle_msg.midpoint.orientation.w))
            {
                error_message = "The midpoint of the arc must be inputted";
                return false;
            }
            if (fabs(target_circle_msg.endpoint.position.x) < min_val &&
                fabs(target_circle_msg.endpoint.position.y) < min_val &&
                fabs(target_circle_msg.endpoint.position.z) < min_val &&
                fabs(target_circle_msg.endpoint.orientation.x) < min_val &&
                fabs(target_circle_msg.endpoint.orientation.y) < min_val &&
                fabs(target_circle_msg.endpoint.orientation.z) < min_val &&
                fabs(target_circle_msg.endpoint.orientation.w) < min_val)
            {
                error_message = "The endpoint of the arc must be inputted";
                return false;
            }
            if (std::isnan(target_circle_msg.endpoint.position.x) ||
                std::isnan(target_circle_msg.endpoint.position.y) ||
                std::isnan(target_circle_msg.endpoint.position.z) ||
                std::isnan(target_circle_msg.endpoint.orientation.x) ||
                std::isnan(target_circle_msg.endpoint.orientation.y) ||
                std::isnan(target_circle_msg.endpoint.orientation.z) ||
                std::isnan(target_circle_msg.endpoint.orientation.w))
            {
                error_message = "The endpoint of the arc must be inputted";
                return false;
            }
        }
        else
        {
            if (fabs(target_circle_msg.axis.x) < min_val && fabs(target_circle_msg.axis.y) < min_val &&
                fabs(target_circle_msg.axis.z) < min_val)
            {
                error_message = "Please input the axis of rotation for the circular arc";
                return false;
            }

            if (target_circle_msg.rotate_angle < min_val)
            {
                error_message = "Please input the axis of rotation for the circular arc";
                return false;
            }
        }

        return true;
    }

    planning::TrajectPoint CartesianTrajectoryManager::setCartesianPoint(
        const EndEffectorPose& input_point)
    {
        planning::TrajectPoint res;
        res.cart_pos = input_point.position;
        res.quaternion_point.q.w = input_point.quaternion.w();
        res.quaternion_point.q.x = input_point.quaternion.x();
        res.quaternion_point.q.y = input_point.quaternion.y();
        res.quaternion_point.q.z = input_point.quaternion.z();
        return res;
    }

    planning::TrajectPoint CartesianTrajectoryManager::setCartesianPoint(
        const geometry_msgs::msg::Pose& input_point)
    {
        planning::TrajectPoint res;
        res.cart_pos(0) = input_point.position.x;
        res.cart_pos(1) = input_point.position.y;
        res.cart_pos(2) = input_point.position.z;
        res.quaternion_point.q.x = input_point.orientation.x;
        res.quaternion_point.q.y = input_point.orientation.y;
        res.quaternion_point.q.z = input_point.orientation.z;
        res.quaternion_point.q.w = input_point.orientation.w;
        return res;
    }

    planning::TrajectoryParameter CartesianTrajectoryManager::setCartesianParameter(
        const arms_ros2_control_msgs::msg::LinearMessage& target_point_msg)
    {
        if (target_point_msg.time_mode)
        {
            // 时间模式
            double duration = target_point_msg.duration;
            duration = duration > 0.01 ? duration : 5.0;
            planning::TrajectoryParameter time_mode_para(true, duration);
            if (fabs(target_point_msg.max_linear_velocity) > min_val &&
                fabs(target_point_msg.max_linear_acceleration) > min_val &&
                fabs(target_point_msg.max_linear_jerk) > min_val &&
                fabs(target_point_msg.max_angular_velocity) > min_val &&
                fabs(target_point_msg.max_angular_acceleration) > min_val &&
                fabs(target_point_msg.max_angular_jerk) > min_val)
            {
                time_mode_para.max_linear_vel = target_point_msg.max_linear_velocity;
                time_mode_para.max_linear_acc = target_point_msg.max_linear_acceleration;
                time_mode_para.max_linear_jerk = target_point_msg.max_linear_jerk;
                time_mode_para.max_angular_vel = target_point_msg.max_angular_velocity;
                time_mode_para.max_angular_acc =
                    target_point_msg.max_angular_acceleration;
                time_mode_para.max_angular_jerk = target_point_msg.max_angular_jerk;
            }
            return time_mode_para;
        }
        else
        {
            // 速度模式
            planning::TrajectoryParameter speed_mode_para(
                target_point_msg.max_linear_velocity,
                target_point_msg.max_linear_acceleration,
                target_point_msg.max_linear_jerk, target_point_msg.max_angular_velocity,
                target_point_msg.max_angular_acceleration,
                target_point_msg.max_angular_jerk, true);
            return speed_mode_para;
        }
    }

    planning::TrajectoryParameter CartesianTrajectoryManager::setCartesianParameter(
        const arms_ros2_control_msgs::msg::CircleMessage& target_circle_msg)
    {
        if (target_circle_msg.time_mode)
        {
            // 时间模式
            double duration = target_circle_msg.duration;
            duration = duration > 0.01 ? duration : 5.0;

            planning::TrajectoryParameter time_mode_para(
                target_circle_msg.use_slerp_for_orientation, duration);
            if (fabs(target_circle_msg.max_linear_velocity) > min_val &&
                fabs(target_circle_msg.max_linear_acceleration) > min_val &&
                fabs(target_circle_msg.max_linear_jerk) > min_val &&
                fabs(target_circle_msg.max_angular_velocity) > min_val &&
                fabs(target_circle_msg.max_angular_acceleration) > min_val &&
                fabs(target_circle_msg.max_angular_jerk) > min_val)
            {
                time_mode_para.max_linear_vel = target_circle_msg.max_linear_velocity;
                time_mode_para.max_linear_acc = target_circle_msg.max_linear_acceleration;
                time_mode_para.max_linear_jerk = target_circle_msg.max_linear_jerk;
                time_mode_para.max_angular_vel = target_circle_msg.max_angular_velocity;
                time_mode_para.max_angular_acc =
                    target_circle_msg.max_angular_acceleration;
                time_mode_para.max_angular_jerk = target_circle_msg.max_angular_jerk;
            }
            if (!target_circle_msg.use_three_point_method)
            {
                // 直接输入参数
                time_mode_para.circular_rotation_axis(0) = target_circle_msg.axis.x;
                time_mode_para.circular_rotation_axis(1) = target_circle_msg.axis.y;
                time_mode_para.circular_rotation_axis(2) = target_circle_msg.axis.z;

                time_mode_para.center_of_circle(0) = target_circle_msg.center.x;
                time_mode_para.center_of_circle(1) = target_circle_msg.center.y;
                time_mode_para.center_of_circle(2) = target_circle_msg.center.z;


                time_mode_para.no_need_to_calculate_circle_parameters_with_three_point =
                    true;
            }
            time_mode_para.circular_angle = target_circle_msg.rotate_angle;
            return time_mode_para;
        }
        else
        {
            // 速度模式
            planning::TrajectoryParameter speed_mode_para(
                target_circle_msg.max_linear_velocity,
                target_circle_msg.max_linear_acceleration,
                target_circle_msg.max_linear_jerk,
                target_circle_msg.max_angular_velocity,
                target_circle_msg.max_angular_acceleration,
                target_circle_msg.max_angular_jerk,
                target_circle_msg.use_slerp_for_orientation);

            if (!target_circle_msg.use_three_point_method)
            {
                // 直接输入参数
                speed_mode_para.circular_rotation_axis(0) = target_circle_msg.axis.x;
                speed_mode_para.circular_rotation_axis(1) = target_circle_msg.axis.y;
                speed_mode_para.circular_rotation_axis(2) = target_circle_msg.axis.z;

                speed_mode_para.center_of_circle(0) = target_circle_msg.center.x;
                speed_mode_para.center_of_circle(1) = target_circle_msg.center.y;
                speed_mode_para.center_of_circle(2) = target_circle_msg.center.z;


                speed_mode_para.no_need_to_calculate_circle_parameters_with_three_point =
                    true;
            }
            speed_mode_para.circular_angle = target_circle_msg.rotate_angle;
            return speed_mode_para;
        }
    }
} // namespace arms_controller_common
