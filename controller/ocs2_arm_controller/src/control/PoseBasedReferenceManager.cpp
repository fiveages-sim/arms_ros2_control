//
// Created for OCS2 Arm Controller - PoseBasedReferenceManager
//

#include "ocs2_arm_controller/control/PoseBasedReferenceManager.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>

namespace ocs2::mobile_manipulator
{
    PoseBasedReferenceManager::PoseBasedReferenceManager(
        std::string topicPrefix,
        std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr,
        std::shared_ptr<MobileManipulatorInterface> interfacePtr)
        : ReferenceManagerDecorator(std::move(referenceManagerPtr)),
          topic_prefix_(std::move(topicPrefix)),
          interface_(std::move(interfacePtr)),
          logger_(rclcpp::get_logger("PoseBasedReferenceManager")),
          trajectory_duration_(2.0), moveL_duration_(2.0)
    {
        dual_arm_mode_ = interface_->dual_arm_;

        // 获取base frame
        base_frame_ = interface_->getManipulatorModelInfo().baseFrame;

        // 初始化target state缓存
        left_target_state_ = vector_t::Zero(7);
        right_target_state_ = vector_t::Zero(7);

        // 初始化圆弧指针
#ifdef HAS_LINA_PLANNING
        left_circle_curve_ = std::make_shared<planning::CircularCurver>();
        right_circle_curve_ = std::make_shared<planning::CircularCurver>();
        // 初始化Service状态
        left_service_state_.arm_name = "left";
        left_service_state_.curve = left_circle_curve_;

        if (dual_arm_mode_)
        {
            right_service_state_.arm_name = "right";
            right_service_state_.curve = right_circle_curve_;
        }
#endif
    }

    void PoseBasedReferenceManager::subscribe(
        const rclcpp_lifecycle::LifecycleNode::SharedPtr& node)
    {
        // 保存node引用用于访问参数服务器
        node_ = node;

        // 保存node的logger用于后续日志输出
        logger_ = node->get_logger();

        // 保存clock用于时间戳
        clock_ = node->get_clock();

        // 初始化参数（从参数服务器获取）
        updateParam();

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
            right_pose_subscriber_ =
                node->create_subscription<geometry_msgs::msg::Pose>("right_target", 1,
                                                                    rightCallback);
        }

        // 左臂PoseStamped订阅者（单臂机器人也使用这个）
        auto leftStampedCallback =
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            leftPoseStampedCallback(msg);
        };
        left_pose_stamped_subscriber_ =
            node->create_subscription<geometry_msgs::msg::PoseStamped>(
                "left_target/stamped", 1, leftStampedCallback);

        // 右臂PoseStamped订阅者（仅双臂机器人）
        if (dual_arm_mode_)
        {
            auto rightStampedCallback =
                [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                rightPoseStampedCallback(msg);
            };
            right_pose_stamped_subscriber_ =
                node->create_subscription<geometry_msgs::msg::PoseStamped>(
                    "right_target/stamped", 1, rightStampedCallback);
        }
        // 删除圆弧订阅者，添加Service
#ifdef HAS_LINA_PLANNING
        // 左臂圆弧Service
        auto leftServiceCallback =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<arms_ros2_control_msgs::srv::ExecuteCircle::Request> request,
                   std::shared_ptr<arms_ros2_control_msgs::srv::ExecuteCircle::Response> response)
        {
            this->handleLeftCircleService(request_header, request, response);
        };

        left_circle_service_ = node->create_service<arms_ros2_control_msgs::srv::ExecuteCircle>(
            "execute_left_circle",
            leftServiceCallback);

        // 右臂圆弧Service（仅双臂机器人）
        if (dual_arm_mode_)
        {
            auto rightServiceCallback =
                [this](const std::shared_ptr<rmw_request_id_t> request_header,
                       const std::shared_ptr<arms_ros2_control_msgs::srv::ExecuteCircle::Request> request,
                       std::shared_ptr<arms_ros2_control_msgs::srv::ExecuteCircle::Response> response)
            {
                this->handleRightCircleService(request_header, request, response);
            };

            right_circle_service_ = node->create_service<arms_ros2_control_msgs::srv::ExecuteCircle>(
                "execute_right_circle",
                rightServiceCallback);
        }
#else
        RCLCPP_WARN(logger_,
                    "lina_planning not available, circle execution services are disabled");
#endif

        // 创建执行定时器
        // execution_timer_ = node->create_wall_timer(
        //     std::chrono::milliseconds(40), // 25Hz，与之前的采样率一致
        //     [this]() { this->executionTimerCallback(); });

        // 双目标PoseStamped订阅者（仅双臂机器人，用于同时更新左右两个目标）
        // Path长度为2，第一个是左臂，第二个是右臂
        if (dual_arm_mode_)
        {
            auto dualTargetStampedCallback =
                [this](const nav_msgs::msg::Path::SharedPtr msg)
            {
                this->dualTargetStampedCallback(msg);
            };
            dual_target_stamped_subscriber_ =
                node->create_subscription<nav_msgs::msg::Path>(
                    "dual_target/stamped", 1, dualTargetStampedCallback);
        }

        // Path订阅者（单臂和双臂机器人都支持）
        auto pathCallback = [this](const nav_msgs::msg::Path::SharedPtr msg)
        {
            this->pathCallback(msg);
        };
        path_subscriber_ = node->create_subscription<nav_msgs::msg::Path>(
            "target_path", 1, pathCallback);

        auto executePathCallback =
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<arms_ros2_control_msgs::srv::ExecutePath::Request>
                       request,
                   std::shared_ptr<arms_ros2_control_msgs::srv::ExecutePath::Response> response)
        {
            (void)request_header;
            this->handleExecutePathService(request, response);
        };
        execute_path_service_ =
            node->create_service<arms_ros2_control_msgs::srv::ExecutePath>(
                "execute_path", executePathCallback);

        // 初始化发布器：发布当前目标
        left_target_publisher_ =
            node->create_publisher<geometry_msgs::msg::PoseStamped>(
                "left_current_target", 1);

        if (dual_arm_mode_)
        {
            right_target_publisher_ =
                node->create_publisher<geometry_msgs::msg::PoseStamped>(
                    "right_current_target", 1);
        }
    }

    void PoseBasedReferenceManager::setCurrentObservation(
        const SystemObservation& observation)
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
        vector_array_t input_trajectory(
            1, vector_t::Zero(interface_->getManipulatorModelInfo().inputDim));

        TargetTrajectories target_trajectories(time_trajectory, state_trajectory,
                                               input_trajectory);

        // 设置到ReferenceManager
        referenceManagerPtr_->setTargetTrajectories(std::move(target_trajectories));
    }

    void PoseBasedReferenceManager::updateTrajectory(
        const vector_t& previous_left_target_state,
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

        auto interpolatePose7 = [](const vector_t& s0, const vector_t& s1,
                                   double alpha) -> vector_t
        {
            vector_t out = vector_t::Zero(7);
            // position
            out.segment<3>(0) =
                (1.0 - alpha) * s0.segment<3>(0) + alpha * s1.segment<3>(0);

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
            const double alpha = std::clamp(static_cast<double>(i) /
                                            static_cast<double>(kNumSamples - 1),
                                            0.0, 1.0);

            vector_t xt;
            if (dual_arm_mode_)
            {
                xt = vector_t::Zero(14);
                xt.segment(0, 7) = interpolatePose7(start_state.segment(0, 7),
                                                    goal_state.segment(0, 7), alpha);
                xt.segment(7, 7) = interpolatePose7(start_state.segment(7, 7),
                                                    goal_state.segment(7, 7), alpha);
            }
            else
            {
                xt = interpolatePose7(start_state, goal_state, alpha);
            }

            time_trajectory.push_back(t);
            state_trajectory.push_back(std::move(xt));
        }

        vector_array_t input_trajectory(
            kNumSamples,
            vector_t::Zero(interface_->getManipulatorModelInfo().inputDim));
        TargetTrajectories target_trajectories(time_trajectory, state_trajectory,
                                               input_trajectory);
        referenceManagerPtr_->setTargetTrajectories(std::move(target_trajectories));
    }

    void PoseBasedReferenceManager::updateParam()
    {
        trajectory_duration_ =
            node_->get_parameter("movel_trajectory_duration").as_double();
        moveL_duration_ = node_->get_parameter("movel_duration").as_double();
    }

    void PoseBasedReferenceManager::leftPoseCallback(
        const geometry_msgs::msg::Pose::SharedPtr msg)
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

    void PoseBasedReferenceManager::rightPoseCallback(
        const geometry_msgs::msg::Pose::SharedPtr msg)
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

    void PoseBasedReferenceManager::leftPoseStampedPoseCallback(
        const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        updateParam();

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

    void PoseBasedReferenceManager::rightPoseStampedPoseCallback(
        const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        updateParam();

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
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_->lookupTransform(base_frame_, msg->header.frame_id,
                                            tf2::TimePointZero);

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
            RCLCPP_WARN(logger_, "无法将pose从 %s 转换到 %s: %s",
                        msg->header.frame_id.c_str(), base_frame_.c_str(), ex.what());
        }
    }

    void PoseBasedReferenceManager::leftPoseStampedCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        processPoseStamped(msg, [this](geometry_msgs::msg::Pose::SharedPtr pose_msg)
        {
            leftPoseStampedPoseCallback(pose_msg);
        });
    }

    void PoseBasedReferenceManager::rightPoseStampedCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        processPoseStamped(msg, [this](geometry_msgs::msg::Pose::SharedPtr pose_msg)
        {
            rightPoseStampedPoseCallback(pose_msg);
        });
    }

    void PoseBasedReferenceManager::dualTargetStampedCallback(
        const nav_msgs::msg::Path::SharedPtr msg)
    {
        updateParam();

        if (!dual_arm_mode_)
        {
            RCLCPP_WARN(
                logger_,
                "Dual target stamped callback called but dual arm mode is not enabled");
            return;
        }

        // 检查path长度必须为2
        if (msg->poses.size() != 2)
        {
            RCLCPP_WARN(logger_,
                        "Dual target path must contain exactly 2 poses (left and "
                        "right), got %zu",
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
                geometry_msgs::msg::TransformStamped transform =
                    tf_buffer_->lookupTransform(base_frame_,
                                                left_pose_stamped.header.frame_id,
                                                tf2::TimePointZero);
                tf2::doTransform(left_pose_stamped, transformed_pose, transform);
                left_target_state = poseToState(transformed_pose.pose);
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_WARN(logger_, "无法将左臂pose从 %s 转换到 %s: %s",
                            left_pose_stamped.header.frame_id.c_str(),
                            base_frame_.c_str(), ex.what());
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
                geometry_msgs::msg::TransformStamped transform =
                    tf_buffer_->lookupTransform(base_frame_,
                                                right_pose_stamped.header.frame_id,
                                                tf2::TimePointZero);
                tf2::doTransform(right_pose_stamped, transformed_pose, transform);
                right_target_state = poseToState(transformed_pose.pose);
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_WARN(logger_, "无法将右臂pose从 %s 转换到 %s: %s",
                            right_pose_stamped.header.frame_id.c_str(),
                            base_frame_.c_str(), ex.what());
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

    void PoseBasedReferenceManager::pathCallback(
        const nav_msgs::msg::Path::SharedPtr msg)
    {
        updateParam();

        if (msg->poses.empty())
        {
            RCLCPP_WARN(logger_, "Received empty path");
            return;
        }

        const size_t total_points = msg->poses.size();

        // 转换路径点到状态向量
        auto poseToState =
            [](const geometry_msgs::msg::PoseStamped& pose_stamped) -> vector_t
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

        // 处理路径点：默认所有点都在base frame下，不做TF转换
        std::vector<vector_t> left_arm_waypoints;
        std::vector<vector_t> right_arm_waypoints;

        for (size_t i = 0; i < total_points; ++i)
        {
            const auto& pose_stamped = msg->poses[i];
            // 直接转换，假设所有点都在base frame下
            vector_t state = poseToState(pose_stamped);

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

        runInterpolatedPathTrajectory(left_arm_waypoints, right_arm_waypoints,
                                      trajectory_duration_);
    }

    void PoseBasedReferenceManager::runInterpolatedPathTrajectory(
        const std::vector<vector_t>& left_arm_waypoints,
        const std::vector<vector_t>& right_arm_waypoints,
        double trajectory_duration_sec)
    {
        // 采样间隔（秒）
        constexpr double kSampleInterval = 0.04; // 0.04秒一个采样点（25Hz）

        const size_t kNumSamples = std::max(
            static_cast<size_t>(std::ceil(trajectory_duration_sec / kSampleInterval)) +
                1,
            static_cast<size_t>(2));

        const double t0 = current_observation_.time;
        const double t1 = t0 + trajectory_duration_sec;
        const double dt = (t1 - t0) / static_cast<double>(kNumSamples - 1);

        auto interpolatePose7 = [](const vector_t& s0, const vector_t& s1,
                                   double alpha) -> vector_t
        {
            vector_t out = vector_t::Zero(7);
            out.segment<3>(0) =
                (1.0 - alpha) * s0.segment<3>(0) + alpha * s1.segment<3>(0);

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

        std::vector<vector_t> left_full_path;
        left_full_path.push_back(left_target_state_);
        left_full_path.insert(left_full_path.end(), left_arm_waypoints.begin(),
                              left_arm_waypoints.end());

        std::vector<vector_t> right_full_path;
        if (dual_arm_mode_)
        {
            right_full_path.push_back(right_target_state_);
            right_full_path.insert(right_full_path.end(), right_arm_waypoints.begin(),
                                  right_arm_waypoints.end());
        }

        scalar_array_t time_trajectory;
        time_trajectory.reserve(kNumSamples);
        vector_array_t state_trajectory;
        state_trajectory.reserve(kNumSamples);

        auto interpolateAlongPath =
            [&interpolatePose7](const std::vector<vector_t>& path,
                                double global_alpha) -> vector_t
        {
            if (path.size() == 1)
            {
                return path[0];
            }

            const size_t num_segments = path.size() - 1;
            const double segment_size = 1.0 / static_cast<double>(num_segments);
            size_t segment_idx = static_cast<size_t>(global_alpha / segment_size);
            segment_idx = std::min(segment_idx, num_segments - 1);

            const double segment_start =
                static_cast<double>(segment_idx) * segment_size;
            const double segment_alpha = (global_alpha - segment_start) / segment_size;
            const double clamped_alpha = std::clamp(segment_alpha, 0.0, 1.0);

            return interpolatePose7(path[segment_idx], path[segment_idx + 1],
                                    clamped_alpha);
        };

        for (size_t i = 0; i < kNumSamples; ++i)
        {
            const double t = t0 + static_cast<double>(i) * dt;
            const double global_alpha =
                static_cast<double>(i) / static_cast<double>(kNumSamples - 1);

            vector_t combined_state;
            if (dual_arm_mode_)
            {
                vector_t left_state = interpolateAlongPath(left_full_path, global_alpha);
                vector_t right_state =
                    interpolateAlongPath(right_full_path, global_alpha);

                combined_state = vector_t::Zero(14);
                combined_state.segment(0, 7) = left_state;
                combined_state.segment(7, 7) = right_state;
            }
            else
            {
                combined_state = interpolateAlongPath(left_full_path, global_alpha);
            }

            time_trajectory.push_back(t);
            state_trajectory.push_back(combined_state);
        }

        if (!left_arm_waypoints.empty())
        {
            left_target_state_ = left_arm_waypoints.back();
        }
        if (dual_arm_mode_ && !right_arm_waypoints.empty())
        {
            right_target_state_ = right_arm_waypoints.back();
        }

        vector_array_t input_trajectory(
            kNumSamples,
            vector_t::Zero(interface_->getManipulatorModelInfo().inputDim));
        TargetTrajectories target_trajectories(time_trajectory, state_trajectory,
                                               input_trajectory);
        referenceManagerPtr_->setTargetTrajectories(std::move(target_trajectories));

        publishCurrentTargets();

        if (dual_arm_mode_)
        {
            RCLCPP_INFO(logger_,
                        "处理路径（双臂模式）：左臂 %zu 个路径点，右臂 %zu "
                        "个路径点，生成 %zu 个轨迹点",
                        left_arm_waypoints.size(), right_arm_waypoints.size(),
                        kNumSamples);
        }
        else
        {
            RCLCPP_INFO(logger_,
                        "处理路径（单臂模式）：%zu 个路径点，生成 %zu 个轨迹点",
                        left_arm_waypoints.size(), kNumSamples);
        }
    }

    void PoseBasedReferenceManager::handleExecutePathService(
        const std::shared_ptr<arms_ros2_control_msgs::srv::ExecutePath::Request> request,
        std::shared_ptr<arms_ros2_control_msgs::srv::ExecutePath::Response> response)
    {
        updateParam();

        double duration = request->trajectory_duration;
        if (duration <= 0.0)
        {
            duration = trajectory_duration_;
        }

        auto poseToState =
            [](const geometry_msgs::msg::PoseStamped& pose_stamped) -> vector_t
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

        std::vector<vector_t> left_arm_waypoints;
        std::vector<vector_t> right_arm_waypoints;

        for (const auto& pose_stamped : request->left_arm_path.poses)
        {
            left_arm_waypoints.push_back(poseToState(pose_stamped));
        }
        if (dual_arm_mode_)
        {
            for (const auto& pose_stamped : request->right_arm_path.poses)
            {
                right_arm_waypoints.push_back(poseToState(pose_stamped));
            }
        }

        if (dual_arm_mode_ && request->left_arm_path.poses.empty() &&
            request->right_arm_path.poses.empty())
        {
            response->success = false;
            response->message = "both left_arm_path and right_arm_path are empty";
            response->estimated_duration = 0.0;
            return;
        }

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
            if (left_arm_waypoints.empty())
            {
                left_arm_waypoints.push_back(left_target_state_);
            }
        }

        runInterpolatedPathTrajectory(left_arm_waypoints, right_arm_waypoints, duration);

        response->success = true;
        response->message = "trajectory applied";
        response->estimated_duration = duration;
    }

    void PoseBasedReferenceManager::resetTargetStateCache()
    {
        // 重置target state缓存
        left_target_state_ = vector_t::Zero(7);
        right_target_state_ = vector_t::Zero(7);

        RCLCPP_INFO(logger_,
                    "Target state cache reset - cleared all cached target states");
    }

    void PoseBasedReferenceManager::setCurrentEndEffectorPoses(
        const vector_t& left_ee_pose, const vector_t& right_ee_pose)
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

    void PoseBasedReferenceManager::publishCurrentTargets(
        const std::string& arm_type)
    {
        // 根据 arm_type 决定发布哪个臂的目标
        bool publish_left =
            (arm_type.empty() || arm_type == "both" || arm_type == "left");
        bool publish_right =
            (arm_type.empty() || arm_type == "both" || arm_type == "right");

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

    // 新增圆形轨迹相关函数
#ifdef HAS_LINA_PLANNING
    bool PoseBasedReferenceManager::initCircleCurve(
        vector_t start_pose,
        arms_ros2_control_msgs::msg::CircleMessage::SharedPtr msg,
        std::shared_ptr<planning::CircularCurver> circle_ptr)
    {
        if (start_pose.size() < 7)
        {
            RCLCPP_WARN(logger_,
                        "No starting point provided,please update current point!");
            return false;
        }
        // 更新起点
        planning::TrajectPoint ps;
        ps.cart_pos(0) = start_pose(0);
        ps.cart_pos(1) = start_pose(1);
        ps.cart_pos(2) = start_pose(2);
        ps.quaternion_point.q.x = start_pose(3);
        ps.quaternion_point.q.y = start_pose(4);
        ps.quaternion_point.q.z = start_pose(5);
        ps.quaternion_point.q.w = start_pose(6);
        // 更新终点，无论是哪种计算方式都给一个终点，终点只有在直接给值且在姿态不是slerp插值的时候才不会用到，这时候可以给任意值
        planning::TrajectPoint pe;
        pe.cart_pos(0) = msg->endpoint.position.x;
        pe.cart_pos(1) = msg->endpoint.position.y;
        pe.cart_pos(2) = msg->endpoint.position.z;
        pe.quaternion_point.q.x = msg->endpoint.orientation.x;
        pe.quaternion_point.q.y = msg->endpoint.orientation.y;
        pe.quaternion_point.q.z = msg->endpoint.orientation.z;
        pe.quaternion_point.q.w = msg->endpoint.orientation.w;
        // 初始化圆弧参数
        if (msg->time_mode)
        {
            // 时间模式
            if (msg->duration < 0.001)
            {
                RCLCPP_WARN(logger_,
                            "The time mode does not have the exercise time inputted. "
                            "Please input the time or change the exercise mode");
                return false;
            }
            planning::TrajectoryParameter time_mode_para(msg->use_slerp_for_orientation,
                                                         msg->duration);

            if (fabs(msg->max_linear_velocity) > min_val &&
                fabs(msg->max_linear_acceleration) > min_val &&
                fabs(msg->max_linear_jerk) > min_val &&
                fabs(msg->max_angular_velocity) > min_val &&
                fabs(msg->max_angular_acceleration) > min_val &&
                fabs(msg->max_angular_jerk) > min_val)
            {
                // 如果用户设置了参数
                time_mode_para.max_linear_vel = msg->max_linear_velocity;
                time_mode_para.max_linear_acc = msg->max_linear_acceleration;
                time_mode_para.max_linear_jerk = msg->max_linear_jerk;
                time_mode_para.max_angular_vel = msg->max_angular_velocity;
                time_mode_para.max_angular_acc = msg->max_angular_acceleration;
                time_mode_para.max_angular_jerk = msg->max_angular_jerk;
            }

            if (!msg->use_three_point_method)
            {
                // 如果是直接输入参数
                time_mode_para.circular_rotation_axis(0) = msg->axis.x;
                time_mode_para.circular_rotation_axis(1) = msg->axis.y;
                time_mode_para.circular_rotation_axis(2) = msg->axis.z;

                time_mode_para.center_of_circle(0) = msg->center.x;
                time_mode_para.center_of_circle(1) = msg->center.y;
                time_mode_para.center_of_circle(2) = msg->center.z;

                time_mode_para.circular_angle = msg->rotate_angle;
                time_mode_para.no_need_to_calculate_circle_parameters_with_three_point =
                    true;
                planning::TrajectoryInitParameters init_para(ps, pe, time_mode_para,
                                                             0.01);
                if (!circle_ptr->init(init_para))
                {
                    RCLCPP_WARN(logger_, "Circular curve init error!");
                    return false;
                }
            }
            else
            {
                // 三点计算
                planning::TrajectPoint pm;
                pm.cart_pos(0) = msg->midpoint.position.x;
                pm.cart_pos(1) = msg->midpoint.position.y;
                pm.cart_pos(2) = msg->midpoint.position.z;
                pm.quaternion_point.q.x = msg->midpoint.orientation.x;
                pm.quaternion_point.q.y = msg->midpoint.orientation.y;
                pm.quaternion_point.q.z = msg->midpoint.orientation.z;
                pm.quaternion_point.q.w = msg->midpoint.orientation.w;
                planning::TrajectoryInitParameters init_para(ps, pm, pe, time_mode_para,
                                                             0.01);
                if (!circle_ptr->init(init_para))
                {
                    RCLCPP_WARN(logger_, "Circular curve init error!");
                    return false;
                }
            }
        }
        else
        {
            // 参数模式
            planning::TrajectoryParameter speed_mode_para(
                msg->max_linear_velocity, msg->max_linear_acceleration,
                msg->max_linear_jerk, msg->max_angular_velocity,
                msg->max_angular_acceleration, msg->max_angular_jerk,
                msg->use_slerp_for_orientation);
            if (!msg->use_three_point_method)
            {
                // 如果是直接输入参数
                speed_mode_para.circular_rotation_axis(0) = msg->axis.x;
                speed_mode_para.circular_rotation_axis(1) = msg->axis.y;
                speed_mode_para.circular_rotation_axis(2) = msg->axis.z;

                speed_mode_para.center_of_circle(0) = msg->center.x;
                speed_mode_para.center_of_circle(1) = msg->center.y;
                speed_mode_para.center_of_circle(2) = msg->center.z;

                speed_mode_para.circular_angle = msg->rotate_angle;
                speed_mode_para.no_need_to_calculate_circle_parameters_with_three_point =
                    true;
                planning::TrajectoryInitParameters init_para(ps, pe, speed_mode_para,
                                                             0.01);
                if (!circle_ptr->init(init_para))
                {
                    RCLCPP_WARN(logger_, "Circular curve init error!");
                    return false;
                }
            }
            else
            {
                // 三点计算
                planning::TrajectPoint pm;
                pm.cart_pos(0) = msg->midpoint.position.x;
                pm.cart_pos(1) = msg->midpoint.position.y;
                pm.cart_pos(2) = msg->midpoint.position.z;
                pm.quaternion_point.q.x = msg->midpoint.orientation.x;
                pm.quaternion_point.q.y = msg->midpoint.orientation.y;
                pm.quaternion_point.q.z = msg->midpoint.orientation.z;
                pm.quaternion_point.q.w = msg->midpoint.orientation.w;
                planning::TrajectoryInitParameters init_para(ps, pm, pe, speed_mode_para,
                                                             0.01);
                if (!circle_ptr->init(init_para))
                {
                    RCLCPP_WARN(logger_, "Circular curve init error!");
                    return false;
                }
            }
        }

        return true;
    };


    // 使用已有的变换转换位姿
    static geometry_msgs::msg::Pose
    transformPose(const geometry_msgs::msg::Pose& pose,
                  const geometry_msgs::msg::TransformStamped& transform)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.pose = pose;
        pose_stamped.header.frame_id = transform.header.frame_id;
        pose_stamped.header.stamp = transform.header.stamp;

        geometry_msgs::msg::PoseStamped transformed;
        tf2::doTransform(pose_stamped, transformed, transform);

        return transformed.pose;
    };

    // 转换向量（只旋转不平移）
    static geometry_msgs::msg::Vector3
    transformVector3(const geometry_msgs::msg::Vector3& vector,
                     const geometry_msgs::msg::TransformStamped& transform)
    {
        geometry_msgs::msg::Vector3Stamped vector_in, vector_out;
        vector_in.vector = vector;
        vector_in.header.frame_id = transform.header.frame_id;
        vector_in.header.stamp = transform.header.stamp;

        tf2::doTransform(vector_in, vector_out, transform);
        return vector_out.vector;
    };

    void PoseBasedReferenceManager::transCircleMessageToBaseFrame(const
                                                                  arms_ros2_control_msgs::msg::CircleMessage& msg,
                                                                  arms_ros2_control_msgs::msg::CircleMessage::SharedPtr
                                                                  base_msg)
    {
        // 把输入的圆弧参数转换成base_frame
        if (msg.frame_id == base_frame_)
        {
            *base_msg = msg;
            return;
        }
        // 如果不是极坐标系，需要对其中的参数进行转换
        try
        {
            // 复制所有的参数
            *base_msg = msg;

            // 获取变换
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_->lookupTransform(base_frame_, msg.frame_id,
                                            tf2::TimePointZero);
            // 根据不同的方法转换不同的数据
            if (msg.use_three_point_method)
            {
                // 转换三点法中的两个点
                base_msg->midpoint = transformPose(msg.midpoint, transform);
                base_msg->endpoint = transformPose(msg.endpoint, transform);
            }
            else
            {
                // 转换参数法中的圆心
                geometry_msgs::msg::Pose center_pose;
                center_pose.position = msg.center;
                center_pose.orientation = geometry_msgs::msg::Quaternion();
                auto transformed_center = transformPose(center_pose, transform);
                base_msg->center = transformed_center.position;

                // 转换旋转轴（只旋转不平移）
                base_msg->axis = transformVector3(msg.axis, transform);
            }
            // 更新坐标系ID
            base_msg->frame_id = base_frame_;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN(logger_, "无法将pose从 %s 转换到 %s: %s", msg.frame_id.c_str(),
                        base_frame_.c_str(), ex.what());
        }
    };


    void PoseBasedReferenceManager::handleLeftCircleService(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<arms_ros2_control_msgs::srv::ExecuteCircle::Request> request,
        std::shared_ptr<arms_ros2_control_msgs::srv::ExecuteCircle::Response> response)
    {
        (void)request_header; // 不使用

        RCLCPP_INFO(logger_, "Received left arm arc execution request");
        // 获取起始状态
        const vector_t start_state = left_target_state_;
        // 验证请求
        std::string error_message;
        if (!validateCircleRequest(start_state, request, error_message))
        {
            response->success = false;
            response->message = "Request verification failed: " + error_message;
            RCLCPP_WARN(logger_, "%s", response->message.c_str());
            return;
        }
        //由于没有ocs2运行结束的反馈，先不加线程锁
        // // 检查是否正在执行
        // {
        //     std::lock_guard<std::mutex> lock(left_service_state_.mutex);
        //     if (left_service_state_.is_executing)
        //     {
        //         response->success = false;
        //         response->message =
        //             "The left arm is executing another circular arc trajectory. Please wait for it to complete";
        //         RCLCPP_WARN(logger_, "%s", response->message.c_str());
        //         return;
        //     }
        // }

        // 转换坐标系
        arms_ros2_control_msgs::msg::CircleMessage::SharedPtr base_frame_msg =
            std::make_shared<arms_ros2_control_msgs::msg::CircleMessage>();
        transCircleMessageToBaseFrame(request->circle_params, base_frame_msg);


        // 开始执行
        startServiceExecution(left_service_state_, start_state, *base_frame_msg, "left");
        sendPlannedTrajectoryToOCS2(left_service_state_.time_trajectory, left_service_state_.state_trajectory, "left");


        response->success = true;
        response->message = "The left arm starts to execute a circular arc trajectory, with an estimated time of: " +
            std::to_string(left_service_state_.total_duration) + "s";
        response->estimated_duration = left_service_state_.total_duration;

        RCLCPP_INFO(logger_, "%s", response->message.c_str());
    }

    void PoseBasedReferenceManager::handleRightCircleService(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<arms_ros2_control_msgs::srv::ExecuteCircle::Request> request,
        std::shared_ptr<arms_ros2_control_msgs::srv::ExecuteCircle::Response> response)
    {
        (void)request_header; // 不使用

        if (!dual_arm_mode_)
        {
            response->success = false;
            response->message = "The robot is not in a dual-arm mode and does not support right arm arc";
            RCLCPP_WARN(logger_, "%s", response->message.c_str());
            return;
        }

        RCLCPP_INFO(logger_, "Received right arm arc execution request");
        // 获取起始状态
        const vector_t start_state = right_target_state_;
        // 验证请求
        std::string error_message;
        if (!validateCircleRequest(start_state, request, error_message))
        {
            response->success = false;
            response->message = "Request verification failed: " + error_message;
            RCLCPP_WARN(logger_, "%s", response->message.c_str());
            return;
        }
        //由于没有合适的反馈，暂时不加线程锁
        // // 检查是否正在执行
        // {
        //     std::lock_guard<std::mutex> lock(right_service_state_.mutex);
        //     if (right_service_state_.is_executing)
        //     {
        //         response->success = false;
        //         response->message =
        //             "The right arm is executing another circular arc trajectory. Please wait for it to complete";
        //         RCLCPP_WARN(logger_, "%s", response->message.c_str());
        //         return;
        //     }
        // }

        // 转换坐标系
        arms_ros2_control_msgs::msg::CircleMessage::SharedPtr base_frame_msg =
            std::make_shared<arms_ros2_control_msgs::msg::CircleMessage>();
        transCircleMessageToBaseFrame(request->circle_params, base_frame_msg);


        // 开始执行
        startServiceExecution(right_service_state_, start_state, *base_frame_msg, "right");
        sendPlannedTrajectoryToOCS2(right_service_state_.time_trajectory, right_service_state_.state_trajectory,
                                    "right");
        response->success = true;
        response->message = "The right arm starts to execute a circular arc trajectory, with an estimated time of: " +
            std::to_string(right_service_state_.total_duration) + "s";
        response->estimated_duration = right_service_state_.total_duration;

        RCLCPP_INFO(logger_, "%s", response->message.c_str());
    }

    bool PoseBasedReferenceManager::validateCircleRequest(vector_t start_pose,
                                                          const
                                                          arms_ros2_control_msgs::srv::ExecuteCircle::Request::SharedPtr
                                                          request,
                                                          std::string& error_message)
    {
        const auto& msg = request->circle_params;
        //检查起点的位姿
        if (start_pose.size() < 7)
        {
            error_message =
                "The starting point of the arc has not been entered. Please update the starting point of the arc！";
            return false;
        }
        bool all_start_pose_is_zero = true;
        for (int i = 0; i < 7; i++)
        {
            if (fabs(start_pose(i)) > min_val)
            {
                all_start_pose_is_zero = false;
            }
            if (std::isnan(start_pose(i)))
            {
                error_message = "Please check the starting point！";
                return false;
            }
        }
        if (all_start_pose_is_zero)
        {
            error_message = "Please check the starting point！";
            return false;
        }

        // 检查时间模式下的时间参数
        if (msg.time_mode && msg.duration < 0.001)
        {
            error_message = "In time mode, a valid time parameter (duration > 0.001) must be provided";
            return false;
        }

        // 检查速度模式下的速度参数
        if (!msg.time_mode)
        {
            if (fabs(msg.max_linear_velocity) < min_val)
            {
                error_message = "In speed mode, the maximum linear speed must be provided";
                return false;
            }
            if (fabs(msg.max_angular_velocity) < min_val)
            {
                error_message =
                    "When using attitude interpolation in velocity mode, the maximum angular velocity must be provided";
                return false;
            }
            if (fabs(msg.max_linear_acceleration) < min_val)
            {
                error_message = "In speed mode, the maximum linear acceleration must be provided";
                return false;
            }
            if (fabs(msg.max_angular_acceleration) < min_val)
            {
                error_message =
                    "When using pose interpolation in velocity mode, the maximum angular acceleration must be provided";
                return false;
            }
            if (fabs(msg.max_linear_jerk) < min_val)
            {
                error_message = "In speed mode, the maximum linear acceleration must be provided";
                return false;
            }
            if (fabs(msg.max_angular_jerk) < min_val)
            {
                error_message =
                    "When using attitude interpolation in velocity mode, the maximum angular jerk must be provided";
                return false;
            }
        }


        if (msg.use_three_point_method)
        {
            //检查输入的中间点和终点是不是有问题
            if (fabs(msg.midpoint.position.x) < min_val && fabs(msg.midpoint.position.y) < min_val && fabs(
                    msg.midpoint.position.z) < min_val && fabs(msg.midpoint.orientation.x) < min_val &&
                fabs(msg.midpoint.orientation.y) < min_val && fabs(msg.midpoint.orientation.z) < min_val && fabs(
                    msg.midpoint.orientation.w) < min_val)
            {
                error_message = "The midpoint of the arc must be inputted";
                return false;
            }
            if (std::isnan(msg.midpoint.position.x) || std::isnan(msg.midpoint.position.y) || std::isnan(
                msg.midpoint.position.z) || std::isnan(msg.midpoint.orientation.x) || std::isnan(
                msg.midpoint.orientation.y) || std::isnan(msg.midpoint.orientation.z) || std::isnan(
                msg.midpoint.orientation.w))
            {
                error_message = "The midpoint of the arc must be inputted";
                return false;
            }
            if (fabs(msg.endpoint.position.x) < min_val && fabs(msg.endpoint.position.y) < min_val && fabs(
                    msg.endpoint.position.z) < min_val && fabs(msg.endpoint.orientation.x) < min_val &&
                fabs(msg.endpoint.orientation.y) < min_val && fabs(msg.endpoint.orientation.z) < min_val && fabs(
                    msg.endpoint.orientation.w) < min_val)
            {
                error_message = "The endpoint of the arc must be inputted";
                return false;
            }
            if (std::isnan(msg.endpoint.position.x) || std::isnan(msg.endpoint.position.y) || std::isnan(
                msg.endpoint.position.z) || std::isnan(msg.endpoint.orientation.x) || std::isnan(
                msg.endpoint.orientation.y) || std::isnan(msg.endpoint.orientation.z) || std::isnan(
                msg.endpoint.orientation.w))
            {
                error_message = "The endpoint of the arc must be inputted";
                return false;
            }

            if (fabs(msg.midpoint.position.x - start_pose(0)) < min_val && fabs(msg.midpoint.position.y - start_pose(1))
                < min_val && fabs(msg.midpoint.position.z - start_pose(2)) < min_val && fabs(
                    msg.midpoint.orientation.x - start_pose(3)) < min_val && fabs(
                    msg.midpoint.orientation.y - start_pose(4)) < min_val && fabs(
                    msg.midpoint.orientation.z - start_pose(5)) < min_val && fabs(
                    msg.midpoint.orientation.w - start_pose(6)) < min_val)
            {
                error_message = "The starting point and the middle point of the arc are equal！";
                return false;
            }

            if (fabs(msg.endpoint.position.x - start_pose(0)) < min_val && fabs(msg.endpoint.position.y - start_pose(1))
                < min_val && fabs(msg.endpoint.position.z - start_pose(2)) < min_val && fabs(
                    msg.endpoint.orientation.x - start_pose(3)) < min_val && fabs(
                    msg.endpoint.orientation.y - start_pose(4)) < min_val && fabs(
                    msg.endpoint.orientation.z - start_pose(5)) < min_val && fabs(
                    msg.endpoint.orientation.w - start_pose(6)) < min_val)
            {
                error_message = "The starting point and the end point of the arc are equal！";
                return false;
            }
        }
        else
        {
            if (fabs(msg.axis.x) < min_val && fabs(msg.axis.y) < min_val && fabs(msg.axis.z) < min_val)
            {
                error_message = "Please input the axis of rotation for the circular arc";
                return false;
            }

            if (msg.rotate_angle < min_val)
            {
                error_message = "Please input the axis of rotation for the circular arc";
                return false;
            }
        }

        return true;
    }

    void PoseBasedReferenceManager::startServiceExecution(
        ServiceExecutionState& state,
        const vector_t& start_pose,
        const arms_ros2_control_msgs::msg::CircleMessage& msg,
        const std::string& arm_name)
    {
        std::lock_guard<std::mutex> lock(state.mutex);

        // 初始化圆弧曲线
        if (!initCircleCurve(start_pose,
                             std::make_shared<arms_ros2_control_msgs::msg::CircleMessage>(msg),
                             state.curve))
        {
            RCLCPP_ERROR(logger_, "Initialization failure of the circular arc curve of the %s arm", arm_name.c_str());
            return;
        }

        // 获取总时间
        state.total_duration = state.curve->getTotalTime();

        // 预计算轨迹点
        constexpr double kSampleInterval = 0.04; // 25Hz
        const size_t kNumSamples = std::max(
            static_cast<size_t>(std::ceil(state.total_duration / kSampleInterval)) + 1,
            static_cast<size_t>(2)
        );

        // 清空之前的轨迹
        state.time_trajectory.clear();
        state.state_trajectory.clear();
        state.time_trajectory.reserve(kNumSamples);
        state.state_trajectory.reserve(kNumSamples);

        const double dt = state.total_duration / static_cast<double>(kNumSamples - 1);

        // 预计算所有轨迹点
        planning::TrajectPoint point;
        //保存计算的点
        // std::ofstream file("/home/lina/lina/data/cal_movec_data.csv");
        // file << "t" << "," << "x" << "," << "y" << "," << "z" << ","
        //     << "qx" << "," << "qy" << "," << "qz" << "," << "qw" << std::endl;
        for (size_t i = 0; i < kNumSamples; ++i)
        {
            const double t = static_cast<double>(i) * dt;
            point = state.curve->calculatePointAtTInRealTime(t);

            vector_t state_vec = vector_t::Zero(7);
            state_vec(0) = point.cart_pos(0);
            state_vec(1) = point.cart_pos(1);
            state_vec(2) = point.cart_pos(2);
            state_vec(3) = point.quaternion_point.q.x;
            state_vec(4) = point.quaternion_point.q.y;
            state_vec(5) = point.quaternion_point.q.z;
            state_vec(6) = point.quaternion_point.q.w;
            // file << std::fixed << std::setprecision(12) << t << "," << state_vec(0) << "," << state_vec(1) << "," <<
            //     state_vec(2) << "," << state_vec(3) << "," << state_vec(4) << "," << state_vec(5) << "," << state_vec(6)
            //     << std::endl;
            state.time_trajectory.push_back(t);
            state.state_trajectory.push_back(state_vec);
        }
        // file.close();
        // std::cout << "数据已保存" << std::endl;

        // 设置执行状态
        state.start_state = start_pose;
        state.start_time = std::chrono::steady_clock::now();
        state.current_point_index = 0;
        // state.is_executing = true;
    }

    void PoseBasedReferenceManager::sendPlannedTrajectoryToOCS2(const scalar_array_t& time_traj,
                                                                const vector_array_t& pose_trajectory,
                                                                const std::string& arm_name)
    {
        const vector_t previous_left_target_state = left_target_state_;
        const vector_t previous_right_target_state = right_target_state_;
        size_t time_size = time_traj.size();
        size_t traj_size = pose_trajectory.size();
        if (traj_size != time_size)
        {
            RCLCPP_WARN(logger_, "The number of input times matches the number of trajectories");
            return;
        }
        // 起始时间
        const double t0 = current_observation_.time;
        scalar_array_t time_trajectory;
        time_trajectory.reserve(time_size);
        vector_array_t state_trajectory;
        state_trajectory.reserve(time_size);
        if (dual_arm_mode_)
        {
            if (arm_name == "left")
            {
                for (size_t i = 0; i < time_size; i++)
                {
                    vector_t current_state = vector_t::Zero(14);
                    current_state.segment(0, 7) = pose_trajectory[i];
                    current_state.segment(7, 7) = previous_right_target_state; // 保持右臂不变
                    time_trajectory.push_back(t0 + time_traj[i]);
                    state_trajectory.push_back(current_state);
                }
            }
            else if (arm_name == "right")
            {
                for (size_t i = 0; i < time_size; i++)
                {
                    vector_t current_state = vector_t::Zero(14);
                    current_state.segment(0, 7) = previous_left_target_state;
                    current_state.segment(7, 7) = pose_trajectory[i];
                    time_trajectory.push_back(t0 + time_traj[i]);
                    state_trajectory.push_back(current_state);
                }
            }
        }
        else
        {
            for (size_t i = 0; i < time_size; i++)
            {
                vector_t current_state = vector_t::Zero(7);
                current_state = pose_trajectory[i]; // 保持右臂不变
                time_trajectory.push_back(t0 + time_traj[i]);
                state_trajectory.push_back(current_state);
            }
        }

        vector_array_t input_trajectory(
            time_size,
            vector_t::Zero(interface_->getManipulatorModelInfo().inputDim));
        TargetTrajectories target_trajectories(time_trajectory, state_trajectory,
                                               input_trajectory);
        referenceManagerPtr_->setTargetTrajectories(std::move(target_trajectories));


        if (arm_name == "left")
        {
            left_target_state_ = pose_trajectory.back();
            // 发布当前目标（只发布左臂）
            publishCurrentTargets("left");
        }
        else if (arm_name == "right")
        {
            right_target_state_ = pose_trajectory.back();
            // 发布当前目标（只发布左臂）
            publishCurrentTargets("right");
        }
    }
#endif
} // namespace ocs2::mobile_manipulator
