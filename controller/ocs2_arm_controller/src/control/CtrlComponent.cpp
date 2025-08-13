//
// Created for OCS2 Arm Controller - CtrlComponent
//

#include "ocs2_arm_controller/control/CtrlComponent.h"
#include "ocs2_arm_controller/Ocs2ArmController.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <exception>

namespace ocs2::mobile_manipulator
{
    CtrlComponent::CtrlComponent(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                                 CtrlInterfaces& ctrl_interfaces)
        : node_(node), ctrl_interfaces_(ctrl_interfaces)
    {
        // 获取机器人名称
        robot_name_ = node_->get_parameter("robot_name").as_string();

        // 获取关节名称
        joint_names_ = node_->get_parameter("joints").as_string_array();
        const std::string info_file_name = node_->get_parameter("info_file_name").as_string();
        // 自动构建文件路径并初始化接口
        const std::string robot_pkg = robot_name_ + "_description";
        const std::string config_path = ament_index_cpp::get_package_share_directory(robot_pkg);
        const std::string task_file = config_path + "/config/ocs2/" + info_file_name + ".info";
        const std::string lib_folder = config_path + "/ocs2";
        const std::string urdf_file = config_path + "/urdf/" + robot_name_ + ".urdf";

        // 初始化接口
        setupInterface(task_file, lib_folder, urdf_file);

        // 检测是否为双臂模式
        dual_arm_mode_ = interface_->dual_arm_;
        RCLCPP_INFO(node_->get_logger(), "Dual arm mode: %s", dual_arm_mode_ ? "enabled" : "disabled");

        // 初始化MPC组件
        setupMpcComponents();

        // 初始化发布器
        setupPublisher();

        RCLCPP_INFO(node_->get_logger(), "CtrlComponent initialized for robot: %s", robot_name_.c_str());
    }

    void CtrlComponent::setupInterface(const std::string& task_file,
                                       const std::string& lib_folder,
                                       const std::string& urdf_file)
    {
        // 创建Mobile Manipulator接口
        interface_ = std::make_shared<MobileManipulatorInterface>(task_file, lib_folder, urdf_file);

        // 获取baseFrame信息
        base_frame_ = interface_->getManipulatorModelInfo().baseFrame;
        RCLCPP_INFO(node_->get_logger(), "Base frame: %s", base_frame_.c_str());

        // 设置发布器
        setupPublisher();

        RCLCPP_INFO(node_->get_logger(), "Mobile Manipulator Interface setup completed");
        RCLCPP_INFO(node_->get_logger(), "Task file: %s", task_file.c_str());
        RCLCPP_INFO(node_->get_logger(), "URDF file: %s", urdf_file.c_str());
    }

    void CtrlComponent::setupPublisher()
    {
        // 统一创建左臂和右臂的末端执行器发布器
        left_end_effector_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            robot_name_ + "_left_end_effector_pose", 1);
        right_end_effector_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            robot_name_ + "_right_end_effector_pose", 1);

        if (dual_arm_mode_)
        {
            RCLCPP_INFO(node_->get_logger(), "Dual arm mode: Left and right end effector pose publishers created");
            RCLCPP_INFO(node_->get_logger(), "  Left: %s_left_end_effector_pose", robot_name_.c_str());
            RCLCPP_INFO(node_->get_logger(), "  Right: %s_right_end_effector_pose", robot_name_.c_str());
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Single arm mode: Using left end effector pose publisher");
            RCLCPP_INFO(node_->get_logger(), "  Left: %s_left_end_effector_pose", robot_name_.c_str());
        }

        // 初始化MPC观测发布器
        mpc_observation_publisher_ = node_->create_publisher<ocs2_msgs::msg::MpcObservation>(
            robot_name_ + "_mpc_observation", 1);

        RCLCPP_INFO(node_->get_logger(), "MPC observation publisher created for %s_mpc_observation topic",
                    robot_name_.c_str());
    }

    void CtrlComponent::publishEndEffectorPose(const rclcpp::Time& time) const
    {
        if (!interface_)
        {
            RCLCPP_WARN(node_->get_logger(), "Interface not available, cannot publish end effector pose");
            return;
        }

        if (dual_arm_mode_)
        {
            // 双臂模式：发布左臂和右臂的末端执行器位置
            publishLeftEndEffectorPose(time);
            publishRightEndEffectorPose(time);
        }
        else
        {
            // 单臂模式：使用左臂发布器
            publishLeftEndEffectorPose(time);
        }
    }

    void CtrlComponent::publishLeftEndEffectorPose(const rclcpp::Time& time) const
    {
        // 计算左臂末端执行器位置
        const auto ee_pose = computeLeftEndEffectorPose(observation_.state);
        // 发布左臂位置信息
        geometry_msgs::msg::PoseStamped ee_pose_msg;
        ee_pose_msg.header.stamp = time;
        ee_pose_msg.header.frame_id = base_frame_; // 使用baseFrame而不是"world"
        ee_pose_msg.pose.position.x = ee_pose(0);
        ee_pose_msg.pose.position.y = ee_pose(1);
        ee_pose_msg.pose.position.z = ee_pose(2);
        ee_pose_msg.pose.orientation.w = ee_pose(6);
        ee_pose_msg.pose.orientation.x = ee_pose(3);
        ee_pose_msg.pose.orientation.y = ee_pose(4);
        ee_pose_msg.pose.orientation.z = ee_pose(5);
        left_end_effector_pose_publisher_->publish(ee_pose_msg);
    }

    void CtrlComponent::publishRightEndEffectorPose(const rclcpp::Time& time) const
    {
        // 计算右臂末端执行器位置
        const auto ee_pose = computeRightEndEffectorPose(observation_.state);
        // 发布右臂位置信息
        geometry_msgs::msg::PoseStamped ee_pose_msg;
        ee_pose_msg.header.stamp = time;
        ee_pose_msg.header.frame_id = base_frame_; // 使用baseFrame而不是"world"
        ee_pose_msg.pose.position.x = ee_pose(0);
        ee_pose_msg.pose.position.y = ee_pose(1);
        ee_pose_msg.pose.position.z = ee_pose(2);
        ee_pose_msg.pose.orientation.w = ee_pose(6);
        ee_pose_msg.pose.orientation.x = ee_pose(3);
        ee_pose_msg.pose.orientation.y = ee_pose(4);
        ee_pose_msg.pose.orientation.z = ee_pose(5);
        right_end_effector_pose_publisher_->publish(ee_pose_msg);
    }

    vector_t CtrlComponent::computeEndEffectorPose(const vector_t& joint_positions) const
    {
        vector_t ee_state = vector_t::Zero(7); // 3维位置 + 4维四元数

        try
        {
            // 使用Pinocchio计算末端执行器位置和姿态
            const auto& pinocchioInterface = interface_->getPinocchioInterface();
            const auto& model = pinocchioInterface.getModel();
            auto data = pinocchioInterface.getData();

            // 前向运动学
            pinocchio::forwardKinematics(model, data, joint_positions);
            pinocchio::updateFramePlacements(model, data);

            // 获取末端执行器frame的ID
            const auto& eeFrameName = interface_->getManipulatorModelInfo().eeFrame;
            const auto eeFrameId = model.getFrameId(eeFrameName);

            // 获取末端执行器的位置和姿态
            const auto& framePlacement = data.oMf[eeFrameId];
            ee_state.head<3>() = framePlacement.translation();

            // 获取四元数并转换为ROS格式 [w, x, y, z]
            Eigen::Quaterniond quat(framePlacement.rotation());
            ee_state(3) = quat.x(); // x
            ee_state(4) = quat.y(); // y
            ee_state(5) = quat.z(); // z
            ee_state(6) = quat.w(); // w
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(node_->get_logger(), "Failed to compute end-effector pose: %s", e.what());
            // 如果计算失败，使用默认值
            ee_state.head<3>() = vector_t::Zero(3);
            ee_state.tail<4>() = vector_t::Zero(4);
            ee_state(3) = 1.0; // w分量设为1
        }

        return ee_state;
    }

    vector_t CtrlComponent::computeLeftEndEffectorPose(const vector_t& joint_positions) const
    {
        vector_t ee_state = vector_t::Zero(7); // 3维位置 + 4维四元数

        try
        {
            // 使用Pinocchio计算左臂末端执行器位置和姿态
            const auto& pinocchioInterface = interface_->getPinocchioInterface();
            const auto& model = pinocchioInterface.getModel();
            auto data = pinocchioInterface.getData();

            // 前向运动学
            pinocchio::forwardKinematics(model, data, joint_positions);
            pinocchio::updateFramePlacements(model, data);

            // 获取左臂末端执行器frame的ID (eeFrame - 左臂)
            const auto& eeFrameName = interface_->getManipulatorModelInfo().eeFrame;
            const auto eeFrameId = model.getFrameId(eeFrameName);

            // 获取左臂末端执行器的位置和姿态
            const auto& framePlacement = data.oMf[eeFrameId];
            ee_state.head<3>() = framePlacement.translation();

            // 获取四元数并转换为ROS格式 [w, x, y, z]
            Eigen::Quaterniond quat(framePlacement.rotation());
            ee_state(3) = quat.x(); // x
            ee_state(4) = quat.y(); // y
            ee_state(5) = quat.z(); // z
            ee_state(6) = quat.w(); // w
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(node_->get_logger(), "Failed to compute left end-effector pose: %s", e.what());
            // 如果计算失败，使用默认值
            ee_state.head<3>() = vector_t::Zero(3);
            ee_state.tail<4>() = vector_t::Zero(4);
            ee_state(3) = 1.0; // w分量设为1
        }

        return ee_state;
    }

    vector_t CtrlComponent::computeRightEndEffectorPose(const vector_t& joint_positions) const
    {
        vector_t ee_state = vector_t::Zero(7); // 3维位置 + 4维四元数

        try
        {
            // 使用Pinocchio计算右臂末端执行器位置和姿态
            const auto& pinocchioInterface = interface_->getPinocchioInterface();
            const auto& model = pinocchioInterface.getModel();
            auto data = pinocchioInterface.getData();

            // 前向运动学
            pinocchio::forwardKinematics(model, data, joint_positions);
            pinocchio::updateFramePlacements(model, data);

            // 获取右臂末端执行器frame的ID (eeFrame1 - 右臂)
            const auto& eeFrameName = interface_->getManipulatorModelInfo().eeFrame1;
            const auto eeFrameId = model.getFrameId(eeFrameName);

            // 获取右臂末端执行器的位置和姿态
            const auto& framePlacement = data.oMf[eeFrameId];
            ee_state.head<3>() = framePlacement.translation();

            // 获取四元数并转换为ROS格式 [w, x, y, z]
            Eigen::Quaterniond quat(framePlacement.rotation());
            ee_state(3) = quat.x(); // x
            ee_state(4) = quat.y(); // y
            ee_state(5) = quat.z(); // z
            ee_state(6) = quat.w(); // w
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(node_->get_logger(), "Failed to compute right end-effector pose: %s", e.what());
            // 如果计算失败，使用默认值
            ee_state.head<3>() = vector_t::Zero(3);
            ee_state.tail<4>() = vector_t::Zero(4);
            ee_state(3) = 1.0; // w分量设为1
        }

        return ee_state;
    }

    void CtrlComponent::setupMpcComponents()
    {
        // 创建RosReferenceManager并订阅ROS话题
        ros_reference_manager_ = std::make_shared<RosReferenceManager>(
            robot_name_, interface_->getReferenceManagerPtr());
        ros_reference_manager_->subscribe(node_);

        // 创建MPC求解器
        mpc_ = std::make_unique<GaussNewtonDDP_MPC>(
            interface_->mpcSettings(),
            interface_->ddpSettings(),
            interface_->getRollout(),
            interface_->getOptimalControlProblem(),
            interface_->getInitializer());

        // 创建统一的MPC_MRT_Interface，使用RosReferenceManager
        mpc_mrt_interface_ = std::make_unique<MPC_MRT_Interface>(*mpc_);

        // 重要：设置RosReferenceManager到MPC求解器中
        mpc_->getSolverPtr()->setReferenceManager(ros_reference_manager_);

        observation_.state = interface_->getInitialState();
        observation_.input = vector_t::Zero(interface_->getManipulatorModelInfo().inputDim);
        observation_.time = 0.0; // 在enter()方法中会更新为当前时间

        RCLCPP_INFO(node_->get_logger(), "MPC components setup completed");
        RCLCPP_INFO(node_->get_logger(), "RosReferenceManager subscribed to %s_mpc_target topic", robot_name_.c_str());
    }

    void CtrlComponent::updateObservation(const rclcpp::Time& time)
    {
        if (!mpc_mrt_interface_)
        {
            return;
        }
        // 更新观测状态
        observation_.time = time.seconds();
        for (int i = 0; i < joint_names_.size(); ++i)
        {
            observation_.state[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
        }
        observation_.input = vector_t::Zero(interface_->getManipulatorModelInfo().inputDim);
    }

    void CtrlComponent::evaluatePolicy(const rclcpp::Time& time)
    {
        if (!mpc_mrt_interface_)
        {
            RCLCPP_WARN(node_->get_logger(), "MPC MRT interface not available");
            return;
        }
        mpc_mrt_interface_->updatePolicy();

        mpc_mrt_interface_->setCurrentObservation(observation_);

        const auto observation_msg = ros_msg_conversions::createObservationMsg(observation_);
        mpc_observation_publisher_->publish(observation_msg);

        size_t planned_mode = 0;
        // 评估MPC策略
        mpc_mrt_interface_->evaluatePolicy(time.seconds(), observation_.state, optimized_state_, optimized_input_,
                                           planned_mode);


        try
        {
            // 获取MPC策略中的完整轨迹
            const auto& policy = mpc_mrt_interface_->getPolicy();

            // 计算未来时间点（使用ros2_control频率）
            double dt = 1.0 / ctrl_interfaces_.frequency_;
            double future_time = observation_.time + dt;

            // 使用线性插值获取未来时间点的状态
            vector_t future_state = LinearInterpolation::interpolate(
                future_time,
                policy.timeTrajectory_,
                policy.stateTrajectory_
            );

            // 从状态中提取关节位置并设置为命令
            for (size_t i = 0; i < joint_names_.size() && i < future_state.size(); ++i)
            {
                ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(future_state(i));
            }

            // // 输出调试信息
            // RCLCPP_INFO(node_->get_logger(), "Using trajectory: current_time=%.3f, future_time=%.3f, dt=%.4f",
            //             observation_.time, future_time, dt);
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(node_->get_logger(), "Failed to get trajectory, falling back to integration: %s", e.what());

            // 回退到积分方法
            vector_t current_positions(joint_names_.size());
            for (int i = 0; i < joint_names_.size(); ++i)
            {
                current_positions(i) = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
            }

            double dt = 1.0 / ctrl_interfaces_.frequency_;
            for (size_t i = 0; i < joint_names_.size() && i < optimized_input_.size(); ++i)
            {
                double new_position = current_positions(i) + optimized_input_(i) * dt;
                ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(new_position);
            }
        }
    }

    void CtrlComponent::resetMpc()
    {
        // 更新观测时间到当前时间
        observation_.time = node_->now().seconds();

        TargetTrajectories target_trajectories;

        if (dual_arm_mode_)
        {
            // 双臂模式：计算左臂和右臂的初始末端执行器位置
            vector_t left_initial_ee_state = computeLeftEndEffectorPose(observation_.state);
            vector_t right_initial_ee_state = computeRightEndEffectorPose(observation_.state);

            // 输出初始目标轨迹信息
            RCLCPP_INFO(node_->get_logger(),
                        "Initial Target Trajectory - Time: %.3f", observation_.time);
            RCLCPP_INFO(node_->get_logger(),
                        "Left EE State: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                        left_initial_ee_state(0), left_initial_ee_state(1), left_initial_ee_state(2), // 位置 x, y, z
                        left_initial_ee_state(3), left_initial_ee_state(4), left_initial_ee_state(5),
                        left_initial_ee_state(6)); // 四元数 w, x, y, z
            RCLCPP_INFO(node_->get_logger(),
                        "Right EE State: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                        right_initial_ee_state(0), right_initial_ee_state(1), right_initial_ee_state(2), // 位置 x, y, z
                        right_initial_ee_state(3), right_initial_ee_state(4), right_initial_ee_state(5),
                        right_initial_ee_state(6)); // 四元数 w, x, y, z

            // 双臂模式：创建包含两个末端执行器的目标轨迹
            // 14维状态向量：[left_x, left_y, left_z, left_qw, left_qx, left_qy, left_qz,
            //                right_x, right_y, right_z, right_qw, right_qx, right_qy, right_qz]
            vector_t dual_arm_target = (vector_t(14) <<
                left_initial_ee_state, right_initial_ee_state).finished();

            target_trajectories = TargetTrajectories({observation_.time},
                                                     {dual_arm_target},
                                                     {observation_.input});
        }
        else
        {
            // 单臂模式：保持原有逻辑
            // 计算初始末端执行器的位置和姿态
            vector_t initial_ee_state = computeEndEffectorPose(observation_.state);

            // 输出初始目标轨迹信息
            RCLCPP_INFO(node_->get_logger(),
                        "Initial Target Trajectory - Time: %.3f, EE State: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                        observation_.time,
                        initial_ee_state(0), initial_ee_state(1), initial_ee_state(2), // 位置 x, y, z
                        initial_ee_state(3), initial_ee_state(4), initial_ee_state(5),
                        initial_ee_state(6)); // 四元数 w, x, y, z

            // 初始化TargetTrajectories - 使用末端执行器的位置和姿态
            target_trajectories = TargetTrajectories({observation_.time},
                                                     {initial_ee_state},
                                                     {observation_.input});
        }

        // 设置初始观测和目标轨迹
        mpc_mrt_interface_->setCurrentObservation(observation_);
        mpc_mrt_interface_->resetMpcNode(target_trajectories);

        RCLCPP_INFO(node_->get_logger(), "Waiting for the initial policy ...");
        while (!mpc_mrt_interface_->initialPolicyReceived())
        {
            advanceMpc();
            rclcpp::WallRate(interface_->mpcSettings().mrtDesiredFrequency_).sleep();
        }
    }

    void CtrlComponent::advanceMpc() const
    {
        mpc_mrt_interface_->advanceMpc();
    }
} // namespace ocs2::mobile_manipulator
