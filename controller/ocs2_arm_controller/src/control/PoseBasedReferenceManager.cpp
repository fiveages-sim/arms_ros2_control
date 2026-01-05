//
// Created for OCS2 Arm Controller - PoseBasedReferenceManager
//

#include "ocs2_arm_controller/control/PoseBasedReferenceManager.h"
#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <Eigen/Geometry>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ocs2::mobile_manipulator
{
    PoseBasedReferenceManager::PoseBasedReferenceManager(
        std::string topicPrefix,
        std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr,
        std::shared_ptr<MobileManipulatorInterface> interfacePtr,
        double trajectoryDuration,
        double moveLDuration)
        : ReferenceManagerDecorator(std::move(referenceManagerPtr)),
          topic_prefix_(std::move(topicPrefix)),
          interface_(std::move(interfacePtr)),
          logger_(rclcpp::get_logger("PoseBasedReferenceManager")),
          trajectory_duration_(trajectoryDuration),
          moveL_duration_(moveLDuration)
    {
        dual_arm_mode_ = interface_->dual_arm_;

        // 获取base frame
        base_frame_ = interface_->getManipulatorModelInfo().baseFrame;

        // 初始化target state缓存
        left_target_state_ = vector_t::Zero(7);
        right_target_state_ = vector_t::Zero(7);
    }

    void PoseBasedReferenceManager::subscribe(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
    {
        // 保存node的logger用于后续日志输出
        logger_ = node->get_logger();

        // 保存clock用于时间戳
        clock_ = node->get_clock();

        // 初始化TF2 buffer和listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 左臂pose订阅者（单臂机器人也使用这个）
        auto leftCallback = [this](const geometry_msgs::msg::Pose::SharedPtr msg)
        {
            leftPoseCallback(msg);
        };
        left_pose_subscriber_ = node->create_subscription<geometry_msgs::msg::Pose>(
            "left_target", 1, leftCallback);

        // 右臂pose订阅者（仅双臂机器人）
        if (dual_arm_mode_)
        {
            auto rightCallback = [this](const geometry_msgs::msg::Pose::SharedPtr msg)
            {
                rightPoseCallback(msg);
            };
            right_pose_subscriber_ = node->create_subscription<geometry_msgs::msg::Pose>(
                "right_target", 1, rightCallback);
        }

        // 左臂PoseStamped订阅者（单臂机器人也使用这个）
        auto leftStampedCallback = [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            leftPoseStampedCallback(msg);
        };
        left_pose_stamped_subscriber_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
            "left_target/stamped", 1, leftStampedCallback);

        // 右臂PoseStamped订阅者（仅双臂机器人）
        if (dual_arm_mode_)
        {
            auto rightStampedCallback = [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                rightPoseStampedCallback(msg);
            };
            right_pose_stamped_subscriber_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
                "right_target/stamped", 1, rightStampedCallback);
        }

        // 双目标PoseStamped订阅者（仅双臂机器人，用于同时更新左右两个目标）
        // Path长度为2，第一个是左臂，第二个是右臂
        if (dual_arm_mode_)
        {
            auto dualTargetStampedCallback = [this](const nav_msgs::msg::Path::SharedPtr msg)
            {
                this->dualTargetStampedCallback(msg);
            };
            dual_target_stamped_subscriber_ = node->create_subscription<nav_msgs::msg::Path>(
                "dual_target/stamped", 1, dualTargetStampedCallback);
        }

        // Path订阅者（单臂和双臂机器人都支持）
        auto pathCallback = [this](const nav_msgs::msg::Path::SharedPtr msg)
        {
            this->pathCallback(msg);
        };
        path_subscriber_ = node->create_subscription<nav_msgs::msg::Path>(
            "target_path", 1, pathCallback);

        // 初始化发布器：发布当前目标
        left_target_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
            "left_current_target", 1);

        if (dual_arm_mode_)
        {
            right_target_publisher_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
                "right_current_target", 1);
        }
    }

    void PoseBasedReferenceManager::setCurrentObservation(const SystemObservation& observation)
    {
        current_observation_ = observation;
    }

    void PoseBasedReferenceManager::updateTargetTrajectory()
    {
        // 创建合并的target state
        vector_t combined_target_state;

        if (dual_arm_mode_)
        {
            combined_target_state = vector_t::Zero(14);
            combined_target_state.segment(0, 7) = left_target_state_;
            combined_target_state.segment(7, 7) = right_target_state_;
        }
        else
        {
            combined_target_state = left_target_state_;
        }

        // 使用当前系统时间作为目标时间
        double target_time = current_observation_.time;

        // 创建TargetTrajectories - 只使用一个时间点
        scalar_array_t time_trajectory = {target_time};
        vector_array_t state_trajectory = {combined_target_state};
        vector_array_t input_trajectory(1, vector_t::Zero(interface_->getManipulatorModelInfo().inputDim));

        TargetTrajectories target_trajectories(time_trajectory, state_trajectory, input_trajectory);

        // 设置到ReferenceManager
        referenceManagerPtr_->setTargetTrajectories(std::move(target_trajectories));
    }

    void PoseBasedReferenceManager::updateTrajectory(const vector_t& previous_left_target_state,
                                                     const vector_t& previous_right_target_state)
    {
        // 使用 moveL_duration_ 作为插值时间
        const double actual_duration = moveL_duration_;

        // 采样间隔（秒）
        constexpr double kSampleInterval = 0.04; // 0.04秒一个采样点（25Hz）

        // 根据时间长度动态计算采样点数量
        const size_t kNumSamples = std::max(
            static_cast<size_t>(std::ceil(actual_duration / kSampleInterval)) + 1,
            static_cast<size_t>(2) // 至少2个采样点
        );

        // 起始/终止时间
        const double t0 = current_observation_.time;
        const double t1 = t0 + actual_duration;
        const double dt = (t1 - t0) / static_cast<double>(kNumSamples - 1);

        // 组装 start / goal 的合并 state
        // 在双臂模式下，总是同时更新两个臂的轨迹
        vector_t start_state;
        vector_t goal_state;
        if (dual_arm_mode_)
        {
            start_state = vector_t::Zero(14);
            goal_state = vector_t::Zero(14);

            // 左臂：从previous到current
            start_state.segment(0, 7) = previous_left_target_state;
            goal_state.segment(0, 7) = left_target_state_;

            // 右臂：从previous到current
            start_state.segment(7, 7) = previous_right_target_state;
            goal_state.segment(7, 7) = right_target_state_;
        }
        else
        {
            start_state = previous_left_target_state;
            goal_state = left_target_state_;
        }

        auto interpolatePose7 = [](const vector_t& s0, const vector_t& s1, double alpha) -> vector_t
        {
            vector_t out = vector_t::Zero(7);
            // position
            out.segment<3>(0) = (1.0 - alpha) * s0.segment<3>(0) + alpha * s1.segment<3>(0);

            // quaternion stored as [qx, qy, qz, qw]
            Eigen::Quaterniond q0(s0(6), s0(3), s0(4), s0(5));
            Eigen::Quaterniond q1(s1(6), s1(3), s1(4), s1(5));
            if (q0.norm() < 1e-9)
            {
                q0 = Eigen::Quaterniond::Identity();
            }
            else
            {
                q0.normalize();
            }
            if (q1.norm() < 1e-9)
            {
                q1 = q0;
            }
            else
            {
                q1.normalize();
            }

            // 保证走最短弧（避免四元数符号翻转导致绕远）
            if (q0.dot(q1) < 0.0)
            {
                q1.coeffs() *= -1.0;
            }
            Eigen::Quaterniond q = q0.slerp(alpha, q1);
            q.normalize();

            out(3) = q.x();
            out(4) = q.y();
            out(5) = q.z();
            out(6) = q.w();
            return out;
        };

        scalar_array_t time_trajectory;
        time_trajectory.reserve(kNumSamples);
        vector_array_t state_trajectory;
        state_trajectory.reserve(kNumSamples);

        for (size_t i = 0; i < kNumSamples; ++i)
        {
            const double t = t0 + static_cast<double>(i) * dt;
            const double alpha = std::clamp(static_cast<double>(i) / static_cast<double>(kNumSamples - 1), 0.0, 1.0);

            vector_t xt;
            if (dual_arm_mode_)
            {
                xt = vector_t::Zero(14);
                xt.segment(0, 7) = interpolatePose7(start_state.segment(0, 7), goal_state.segment(0, 7), alpha);
                xt.segment(7, 7) = interpolatePose7(start_state.segment(7, 7), goal_state.segment(7, 7), alpha);
            }
            else
            {
                xt = interpolatePose7(start_state, goal_state, alpha);
            }

            time_trajectory.push_back(t);
            state_trajectory.push_back(std::move(xt));
        }

        vector_array_t input_trajectory(kNumSamples, vector_t::Zero(interface_->getManipulatorModelInfo().inputDim));
        TargetTrajectories target_trajectories(time_trajectory, state_trajectory, input_trajectory);
        referenceManagerPtr_->setTargetTrajectories(std::move(target_trajectories));
    }

    void PoseBasedReferenceManager::leftPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        // 转换pose到状态向量（单臂和双臂模式都使用前7维）
        vector_t target_state = vector_t::Zero(7);
        target_state(0) = msg->position.x;
        target_state(1) = msg->position.y;
        target_state(2) = msg->position.z;
        target_state(3) = msg->orientation.x;
        target_state(4) = msg->orientation.y;
        target_state(5) = msg->orientation.z;
        target_state(6) = msg->orientation.w;

        // 更新左臂target state缓存
        left_target_state_ = target_state;

        // 更新target trajectory
        updateTargetTrajectory();

        // 发布当前目标
        publishCurrentTargets();
    }

    void PoseBasedReferenceManager::rightPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        // 转换pose到状态向量（右臂状态为7维）
        vector_t target_state = vector_t::Zero(7);
        target_state(0) = msg->position.x;
        target_state(1) = msg->position.y;
        target_state(2) = msg->position.z;
        target_state(3) = msg->orientation.x;
        target_state(4) = msg->orientation.y;
        target_state(5) = msg->orientation.z;
        target_state(6) = msg->orientation.w;

        // 更新右臂target state缓存
        right_target_state_ = target_state;

        // 更新target trajectory
        updateTargetTrajectory();

        // 发布当前目标
        publishCurrentTargets();
    }

    void PoseBasedReferenceManager::leftPoseStampedPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        // 保存上一帧缓存（用于插值起点）
        const vector_t previous_left_target_state = left_target_state_;

        // 更新缓存到新目标
        vector_t target_state = vector_t::Zero(7);
        target_state(0) = msg->position.x;
        target_state(1) = msg->position.y;
        target_state(2) = msg->position.z;
        target_state(3) = msg->orientation.x;
        target_state(4) = msg->orientation.y;
        target_state(5) = msg->orientation.z;
        target_state(6) = msg->orientation.w;
        left_target_state_ = target_state;

        // 生成轨迹
        if (dual_arm_mode_)
        {
            // 双臂模式：同时更新两个臂
            const vector_t previous_right_target_state = right_target_state_;
            updateTrajectory(previous_left_target_state, previous_right_target_state);
        }
        else
        {
            // 单臂模式：只更新左臂，传递零向量作为右臂占位符（不会被使用）
            updateTrajectory(previous_left_target_state, vector_t::Zero(7));
        }

        // 发布当前目标（只发布左臂）
        publishCurrentTargets("left");
    }

    void PoseBasedReferenceManager::rightPoseStampedPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        // 保存上一帧缓存（用于插值起点）
        const vector_t previous_left_target_state = left_target_state_;
        const vector_t previous_right_target_state = right_target_state_;

        // 更新缓存到新目标
        vector_t target_state = vector_t::Zero(7);
        target_state(0) = msg->position.x;
        target_state(1) = msg->position.y;
        target_state(2) = msg->position.z;
        target_state(3) = msg->orientation.x;
        target_state(4) = msg->orientation.y;
        target_state(5) = msg->orientation.z;
        target_state(6) = msg->orientation.w;
        right_target_state_ = target_state;

        // 生成轨迹（在双臂模式下，同时更新两个臂）
        updateTrajectory(previous_left_target_state, previous_right_target_state);

        // 发布当前目标（只发布右臂）
        publishCurrentTargets("right");
    }

    void PoseBasedReferenceManager::processPoseStamped(
        const geometry_msgs::msg::PoseStamped::SharedPtr& msg,
        std::function<void(geometry_msgs::msg::Pose::SharedPtr)> callback)
    {
        // 如果源frame和目标frame相同，直接使用
        if (msg->header.frame_id == base_frame_)
        {
            auto pose_msg = std::make_shared<geometry_msgs::msg::Pose>();
            pose_msg->position = msg->pose.position;
            pose_msg->orientation = msg->pose.orientation;
            callback(pose_msg);
            return;
        }

        // 使用TF转换到base frame（使用最新变换）
        try
        {
            geometry_msgs::msg::PoseStamped transformed_pose;

            // 使用最新可用变换
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                base_frame_, msg->header.frame_id, tf2::TimePointZero);

            // 使用doTransform进行转换
            tf2::doTransform(*msg, transformed_pose, transform);

            // 调用指定的回调方法
            auto pose_msg = std::make_shared<geometry_msgs::msg::Pose>();
            pose_msg->position = transformed_pose.pose.position;
            pose_msg->orientation = transformed_pose.pose.orientation;
            callback(pose_msg);
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN(logger_,
                        "无法将pose从 %s 转换到 %s: %s",
                        msg->header.frame_id.c_str(), base_frame_.c_str(), ex.what());
        }
    }

    void PoseBasedReferenceManager::leftPoseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        processPoseStamped(msg, [this](geometry_msgs::msg::Pose::SharedPtr pose_msg)
        {
            leftPoseStampedPoseCallback(pose_msg);
        });
    }

    void PoseBasedReferenceManager::rightPoseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        processPoseStamped(msg, [this](geometry_msgs::msg::Pose::SharedPtr pose_msg)
        {
            rightPoseStampedPoseCallback(pose_msg);
        });
    }

    void PoseBasedReferenceManager::dualTargetStampedCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (!dual_arm_mode_)
        {
            RCLCPP_WARN(logger_, "Dual target stamped callback called but dual arm mode is not enabled");
            return;
        }

        // 检查path长度必须为2
        if (msg->poses.size() != 2)
        {
            RCLCPP_WARN(logger_,
                        "Dual target path must contain exactly 2 poses (left and right), got %zu",
                        msg->poses.size());
            return;
        }

        // 保存上一帧缓存（用于插值起点）
        const vector_t previous_left_target_state = left_target_state_;
        const vector_t previous_right_target_state = right_target_state_;

        // 转换pose到状态向量的辅助函数
        auto poseToState = [](const geometry_msgs::msg::Pose& pose) -> vector_t
        {
            vector_t state = vector_t::Zero(7);
            state(0) = pose.position.x;
            state(1) = pose.position.y;
            state(2) = pose.position.z;
            state(3) = pose.orientation.x;
            state(4) = pose.orientation.y;
            state(5) = pose.orientation.z;
            state(6) = pose.orientation.w;
            return state;
        };

        // 处理左臂（第一个pose）
        const auto& left_pose_stamped = msg->poses[0];
        vector_t left_target_state;

        if (left_pose_stamped.header.frame_id == base_frame_)
        {
            left_target_state = poseToState(left_pose_stamped.pose);
        }
        else
        {
            try
            {
                geometry_msgs::msg::PoseStamped transformed_pose;
                geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                    base_frame_, left_pose_stamped.header.frame_id, tf2::TimePointZero);
                tf2::doTransform(left_pose_stamped, transformed_pose, transform);
                left_target_state = poseToState(transformed_pose.pose);
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_WARN(logger_,
                            "无法将左臂pose从 %s 转换到 %s: %s",
                            left_pose_stamped.header.frame_id.c_str(), base_frame_.c_str(), ex.what());
                return;
            }
        }

        // 处理右臂（第二个pose）
        const auto& right_pose_stamped = msg->poses[1];
        vector_t right_target_state;

        if (right_pose_stamped.header.frame_id == base_frame_)
        {
            right_target_state = poseToState(right_pose_stamped.pose);
        }
        else
        {
            try
            {
                geometry_msgs::msg::PoseStamped transformed_pose;
                geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                    base_frame_, right_pose_stamped.header.frame_id, tf2::TimePointZero);
                tf2::doTransform(right_pose_stamped, transformed_pose, transform);
                right_target_state = poseToState(transformed_pose.pose);
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_WARN(logger_,
                            "无法将右臂pose从 %s 转换到 %s: %s",
                            right_pose_stamped.header.frame_id.c_str(), base_frame_.c_str(), ex.what());
                return;
            }
        }

        // 更新缓存到新目标
        left_target_state_ = left_target_state;
        right_target_state_ = right_target_state;

        // 同时更新两个臂的轨迹
        updateTrajectory(previous_left_target_state, previous_right_target_state);

        // 发布当前目标
        publishCurrentTargets();
    }

    void PoseBasedReferenceManager::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (msg->poses.empty())
        {
            RCLCPP_WARN(logger_, "Received empty path");
            return;
        }

        const size_t total_points = msg->poses.size();

        // 转换路径点到状态向量
        auto poseToState = [](const geometry_msgs::msg::PoseStamped& pose_stamped) -> vector_t
        {
            vector_t state = vector_t::Zero(7);
            state(0) = pose_stamped.pose.position.x;
            state(1) = pose_stamped.pose.position.y;
            state(2) = pose_stamped.pose.position.z;
            state(3) = pose_stamped.pose.orientation.x;
            state(4) = pose_stamped.pose.orientation.y;
            state(5) = pose_stamped.pose.orientation.z;
            state(6) = pose_stamped.pose.orientation.w;
            return state;
        };

        // 处理TF转换：将路径点转换到base frame
        std::vector<vector_t> left_arm_waypoints;
        std::vector<vector_t> right_arm_waypoints;

        for (size_t i = 0; i < total_points; ++i)
        {
            const auto& pose_stamped = msg->poses[i];
            vector_t state;

            // 如果frame_id不是base_frame，需要转换
            if (pose_stamped.header.frame_id != base_frame_)
            {
                try
                {
                    geometry_msgs::msg::PoseStamped transformed_pose;
                    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                        base_frame_, pose_stamped.header.frame_id, tf2::TimePointZero);
                    tf2::doTransform(pose_stamped, transformed_pose, transform);
                    state = poseToState(transformed_pose);
                }
                catch (const tf2::TransformException& ex)
                {
                    RCLCPP_WARN(logger_,
                                "无法将路径点 %zu 从 %s 转换到 %s: %s",
                                i, pose_stamped.header.frame_id.c_str(), base_frame_.c_str(), ex.what());
                    continue; // 跳过无法转换的点
                }
            }
            else
            {
                state = poseToState(pose_stamped);
            }

            // 根据模式分配路径点
            if (dual_arm_mode_)
            {
                // 双臂模式：将路径点分成两半，前半段给左臂，后半段给右臂
                const size_t left_points = (total_points + 1) / 2; // 前半段（向上取整）
                if (i < left_points)
                {
                    left_arm_waypoints.push_back(state);
                }
                else
                {
                    right_arm_waypoints.push_back(state);
                }
            }
            else
            {
                // 单臂模式：所有路径点都用于左臂（单臂机器人使用左臂主题）
                left_arm_waypoints.push_back(state);
            }
        }

        // 检查是否有有效的路径点
        if (left_arm_waypoints.empty() && right_arm_waypoints.empty())
        {
            RCLCPP_WARN(logger_, "没有有效的路径点可以处理");
            return;
        }

        // 如果某个臂没有路径点，使用当前缓存的状态（仅双臂模式）
        if (dual_arm_mode_)
        {
            if (left_arm_waypoints.empty())
            {
                left_arm_waypoints.push_back(left_target_state_);
            }
            if (right_arm_waypoints.empty())
            {
                right_arm_waypoints.push_back(right_target_state_);
            }
        }
        else
        {
            // 单臂模式：如果左臂没有路径点，使用当前缓存的状态
            if (left_arm_waypoints.empty())
            {
                left_arm_waypoints.push_back(left_target_state_);
            }
        }

        // 为每个臂生成插值轨迹
        // 采样间隔（秒）
        constexpr double kSampleInterval = 0.01; // 0.04秒一个采样点（25Hz）

        // 根据时间长度动态计算采样点数量
        const size_t kNumSamples = std::max(
            static_cast<size_t>(std::ceil(trajectory_duration_ / kSampleInterval)) + 1,
            static_cast<size_t>(2) // 至少2个采样点
        );

        const double t0 = current_observation_.time;
        const double t1 = t0 + trajectory_duration_;
        const double dt = (t1 - t0) / static_cast<double>(kNumSamples - 1);

        // 插值函数（复用现有的interpolatePose7逻辑）
        auto interpolatePose7 = [](const vector_t& s0, const vector_t& s1, double alpha) -> vector_t
        {
            vector_t out = vector_t::Zero(7);
            // position
            out.segment<3>(0) = (1.0 - alpha) * s0.segment<3>(0) + alpha * s1.segment<3>(0);

            // quaternion stored as [qx, qy, qz, qw]
            Eigen::Quaterniond q0(s0(6), s0(3), s0(4), s0(5));
            Eigen::Quaterniond q1(s1(6), s1(3), s1(4), s1(5));
            if (q0.norm() < 1e-9)
            {
                q0 = Eigen::Quaterniond::Identity();
            }
            else
            {
                q0.normalize();
            }
            if (q1.norm() < 1e-9)
            {
                q1 = q0;
            }
            else
            {
                q1.normalize();
            }

            // 保证走最短弧（避免四元数符号翻转导致绕远）
            if (q0.dot(q1) < 0.0)
            {
                q1.coeffs() *= -1.0;
            }
            Eigen::Quaterniond q = q0.slerp(alpha, q1);
            q.normalize();

            out(3) = q.x();
            out(4) = q.y();
            out(5) = q.z();
            out(6) = q.w();
            return out;
        };

        // 构建完整的路径点序列（包含起始状态）
        std::vector<vector_t> left_full_path;
        left_full_path.push_back(left_target_state_); // 起始点
        left_full_path.insert(left_full_path.end(), left_arm_waypoints.begin(), left_arm_waypoints.end());

        std::vector<vector_t> right_full_path;
        if (dual_arm_mode_)
        {
            right_full_path.push_back(right_target_state_); // 起始点
            right_full_path.insert(right_full_path.end(), right_arm_waypoints.begin(), right_arm_waypoints.end());
        }

        // 生成轨迹
        scalar_array_t time_trajectory;
        time_trajectory.reserve(kNumSamples);
        vector_array_t state_trajectory;
        state_trajectory.reserve(kNumSamples);

        // 辅助函数：根据全局alpha值在路径点序列中插值
        auto interpolateAlongPath = [&interpolatePose7](const std::vector<vector_t>& path,
                                                        double global_alpha) -> vector_t
        {
            if (path.size() == 1)
            {
                return path[0];
            }

            // 计算当前应该在哪两个路径点之间
            const size_t num_segments = path.size() - 1;
            const double segment_size = 1.0 / static_cast<double>(num_segments);
            size_t segment_idx = static_cast<size_t>(global_alpha / segment_size);
            segment_idx = std::min(segment_idx, num_segments - 1);

            const double segment_start = static_cast<double>(segment_idx) * segment_size;
            const double segment_alpha = (global_alpha - segment_start) / segment_size;
            const double clamped_alpha = std::clamp(segment_alpha, 0.0, 1.0);

            return interpolatePose7(path[segment_idx], path[segment_idx + 1], clamped_alpha);
        };

        for (size_t i = 0; i < kNumSamples; ++i)
        {
            const double t = t0 + static_cast<double>(i) * dt;
            const double global_alpha = static_cast<double>(i) / static_cast<double>(kNumSamples - 1);

            vector_t combined_state;
            if (dual_arm_mode_)
            {
                // 双臂模式：分别插值左右臂，然后合并
                vector_t left_state = interpolateAlongPath(left_full_path, global_alpha);
                vector_t right_state = interpolateAlongPath(right_full_path, global_alpha);

                combined_state = vector_t::Zero(14);
                combined_state.segment(0, 7) = left_state;
                combined_state.segment(7, 7) = right_state;
            }
            else
            {
                // 单臂模式：只插值左臂
                combined_state = interpolateAlongPath(left_full_path, global_alpha);
            }

            time_trajectory.push_back(t);
            state_trajectory.push_back(combined_state);
        }

        // 更新缓存
        if (!left_arm_waypoints.empty())
        {
            left_target_state_ = left_arm_waypoints.back();
        }
        if (dual_arm_mode_ && !right_arm_waypoints.empty())
        {
            right_target_state_ = right_arm_waypoints.back();
        }

        // 创建并设置轨迹
        vector_array_t input_trajectory(kNumSamples, vector_t::Zero(interface_->getManipulatorModelInfo().inputDim));
        TargetTrajectories target_trajectories(time_trajectory, state_trajectory, input_trajectory);
        referenceManagerPtr_->setTargetTrajectories(std::move(target_trajectories));

        // 发布当前目标
        publishCurrentTargets();

        if (dual_arm_mode_)
        {
            RCLCPP_INFO(logger_,
                        "处理路径（双臂模式）：左臂 %zu 个路径点，右臂 %zu 个路径点，生成 %zu 个轨迹点",
                        left_arm_waypoints.size(), right_arm_waypoints.size(), kNumSamples);
        }
        else
        {
            RCLCPP_INFO(logger_,
                        "处理路径（单臂模式）：%zu 个路径点，生成 %zu 个轨迹点",
                        left_arm_waypoints.size(), kNumSamples);
        }
    }

    void PoseBasedReferenceManager::resetTargetStateCache()
    {
        // 重置target state缓存
        left_target_state_ = vector_t::Zero(7);
        right_target_state_ = vector_t::Zero(7);

        RCLCPP_INFO(logger_,
                    "Target state cache reset - cleared all cached target states");
    }

    void PoseBasedReferenceManager::setCurrentEndEffectorPoses(const vector_t& left_ee_pose,
                                                               const vector_t& right_ee_pose)
    {
        // 设置左臂pose到缓存
        left_target_state_ = left_ee_pose;

        // 如果是双臂模式，设置右臂pose到缓存
        if (dual_arm_mode_)
        {
            right_target_state_ = right_ee_pose;
        }

        // 发布当前目标
        publishCurrentTargets();
    }

    void PoseBasedReferenceManager::publishCurrentTargets(const std::string& arm_type)
    {
        // 根据 arm_type 决定发布哪个臂的目标
        bool publish_left = (arm_type.empty() || arm_type == "both" || arm_type == "left");
        bool publish_right = (arm_type.empty() || arm_type == "both" || arm_type == "right");

        // 发布左臂当前目标
        if (publish_left)
        {
            geometry_msgs::msg::PoseStamped left_target_msg;
            left_target_msg.header.stamp = clock_->now();
            left_target_msg.header.frame_id = base_frame_;
            left_target_msg.pose.position.x = left_target_state_(0);
            left_target_msg.pose.position.y = left_target_state_(1);
            left_target_msg.pose.position.z = left_target_state_(2);
            left_target_msg.pose.orientation.x = left_target_state_(3);
            left_target_msg.pose.orientation.y = left_target_state_(4);
            left_target_msg.pose.orientation.z = left_target_state_(5);
            left_target_msg.pose.orientation.w = left_target_state_(6);
            left_target_publisher_->publish(left_target_msg);
        }

        // 发布右臂当前目标（仅双臂模式）
        if (dual_arm_mode_ && publish_right)
        {
            geometry_msgs::msg::PoseStamped right_target_msg;
            right_target_msg.header.stamp = clock_->now();
            right_target_msg.header.frame_id = base_frame_;
            right_target_msg.pose.position.x = right_target_state_(0);
            right_target_msg.pose.position.y = right_target_state_(1);
            right_target_msg.pose.position.z = right_target_state_(2);
            right_target_msg.pose.orientation.x = right_target_state_(3);
            right_target_msg.pose.orientation.y = right_target_state_(4);
            right_target_msg.pose.orientation.z = right_target_state_(5);
            right_target_msg.pose.orientation.w = right_target_state_(6);
            right_target_publisher_->publish(right_target_msg);
        }
    }
} // namespace ocs2::mobile_manipulator
