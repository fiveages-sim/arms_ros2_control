//
// Created for OCS2 Arm Controller - StateOCS2 Implementation
//

#include "ocs2_arm_controller/FSM/StateOCS2.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace ocs2::mobile_manipulator
{
    StateOCS2::StateOCS2(CtrlInterfaces& ctrl_interfaces,
                         const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node)
        : FSMState(FSMStateName::OCS2, "OCS2"),
          ctrl_interfaces_(ctrl_interfaces),
          node_(node)
    {
        // 获取配置参数
        joint_names_ = node_->get_parameter("joints").as_string_array();

        // 获取机器人名称，自动生成包名
        robot_name_ = node_->get_parameter("robot_name").as_string();
        const std::string robot_pkg = robot_name_ + "_description";

        RCLCPP_INFO(node_->get_logger(), "Robot name: %s, Package: %s", robot_name_.c_str(), robot_pkg.c_str());

        try
        {
            // 获取机器人描述包的路径
            const std::string package_share_directory = ament_index_cpp::get_package_share_directory(robot_pkg);

            // 生成URDF文件路径
            urdf_file_ = package_share_directory + "/urdf/" + robot_name_ + ".urdf";

            // 生成任务配置文件路径
            task_file_ = package_share_directory + "/config/ocs2/task.info";

            // 参考四足机器人的lib_folder生成方式
            // 使用home目录下的ocs2_cpp_ad目录
            const char* home_dir = std::getenv("HOME");
            if (home_dir == nullptr)
            {
                throw std::runtime_error("HOME environment variable not set");
            }
            lib_folder_ = std::string(home_dir) + "/ocs2_cpp_ad/" + robot_name_;

            RCLCPP_INFO(node_->get_logger(), "Generated paths:");
            RCLCPP_INFO(node_->get_logger(), "  URDF: %s", urdf_file_.c_str());
            RCLCPP_INFO(node_->get_logger(), "  Task: %s", task_file_.c_str());
            RCLCPP_INFO(node_->get_logger(), "  Lib: %s", lib_folder_.c_str());
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get package share directory: %s", e.what());
            throw;
        }

        // 获取MPC更新频率
        mpc_period_ = 1.0 / 50.0; // 默认50Hz，可以从配置文件读取

        // 设置OCS2组件
        setupOCS2Components();

        RCLCPP_INFO(node_->get_logger(), "StateOCS2 initialized successfully");
    }

    void StateOCS2::enter()
    {
        RCLCPP_INFO(node_->get_logger(), "Entering OCS2 state");

        last_mpc_update_time_ = node_->now();

        // 更新观测时间到当前时间
        observation_.time = node_->now().seconds();

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
        const TargetTrajectories target_trajectories({observation_.time},
                                                     {initial_ee_state},
                                                     {observation_.input});

        // 设置初始观测和目标轨迹
        mpc_mrt_interface_->setCurrentObservation(observation_);
        mpc_mrt_interface_->getReferenceManager().setTargetTrajectories(target_trajectories);

        // 等待初始策略准备完成（参考四足机器人实现）
        RCLCPP_INFO(node_->get_logger(), "Waiting for the initial policy ...");
        while (!mpc_mrt_interface_->initialPolicyReceived())
        {
            mpc_mrt_interface_->advanceMpc();
            rclcpp::WallRate(interface_->mpcSettings().mrtDesiredFrequency_).sleep();
        }
        last_mpc_update_time_ = node_->now();

        RCLCPP_INFO(node_->get_logger(), "Initial policy has been received.");
        RCLCPP_INFO(node_->get_logger(), "OCS2 state activated with initial MPC update");
    }

    vector_t StateOCS2::computeEndEffectorPose(const vector_t& joint_positions) const
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
            ee_state.tail<4>() = Eigen::Quaterniond(framePlacement.rotation()).coeffs();
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

    void StateOCS2::run(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        // 检查是否需要更新MPC
        if (time.seconds() - last_mpc_update_time_.seconds() >= mpc_period_)
        {
            // 推进MPC计算
            mpc_mrt_interface_->advanceMpc();
            last_mpc_update_time_ = time;
        }

        // 加载最新的MPC策略（参考四足机器人实现）
        mpc_mrt_interface_->updatePolicy();

        // 更新硬件观测状态（在updatePolicy之后，确保有最新的策略用于滤波）
        updateObservationFromHardware(time, period);
        mpc_mrt_interface_->setCurrentObservation(observation_);

        // 发布MPC观测（供TargetTrajectoryMarker使用）
        const auto observation_msg = ros_msg_conversions::createObservationMsg(observation_);
        mpc_observation_publisher_->publish(observation_msg);

        size_t planned_mode = 0;
        mpc_mrt_interface_->evaluatePolicy(observation_.time,
                                           observation_.state,
                                           optimized_state_,
                                           optimized_input_, planned_mode);
        observation_.input = optimized_input_;
        
        // 设置关节命令（直接使用未来时间点的位置轨迹）
        setJointCommands();
    }

    void StateOCS2::exit()
    {
        RCLCPP_INFO(node_->get_logger(), "Exiting OCS2 state");
    }

    FSMStateName StateOCS2::checkChange()
    {
        // 检查控制输入进行状态切换
        switch (ctrl_interfaces_.control_inputs_.command)
        {
        case 1: return FSMStateName::ZERO;
        case 2: return FSMStateName::HOME;
        default: return FSMStateName::OCS2;
        }
    }

    void StateOCS2::setupOCS2Components()
    {
        // 创建Mobile Manipulator接口
        interface_ = std::make_shared<MobileManipulatorInterface>(
            task_file_, lib_folder_, urdf_file_);

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
        mpc_mrt_interface_->initRollout(&interface_->getRollout());

        // 重要：设置RosReferenceManager到MPC求解器中
        mpc_->getSolverPtr()->setReferenceManager(ros_reference_manager_);

        // 初始化观测状态
        observation_.state = interface_->getInitialState();
        observation_.input = vector_t::Zero(interface_->getManipulatorModelInfo().inputDim);
        observation_.time = 0.0; // 在enter()方法中会更新为当前时间

        optimized_state_ = observation_.state;
        optimized_input_ = observation_.input;

        // 输出维度信息用于调试
        RCLCPP_INFO(node_->get_logger(), "State dimension: %ld, Input dimension: %ld",
                    observation_.state.size(), observation_.input.size());
        RCLCPP_INFO(node_->get_logger(), "Joint names count: %ld", joint_names_.size());
        RCLCPP_INFO(node_->get_logger(), "Interface state dim: %ld, Interface input dim: %ld",
                    interface_->getManipulatorModelInfo().stateDim,
                    interface_->getManipulatorModelInfo().inputDim);

        // 初始化MPC观测发布器
        mpc_observation_publisher_ = node_->create_publisher<ocs2_msgs::msg::MpcObservation>(
            robot_name_ + "_mpc_observation", 1);

        RCLCPP_INFO(node_->get_logger(), "OCS2 components initialized successfully");
        RCLCPP_INFO(node_->get_logger(), "RosReferenceManager subscribed to %s_mpc_target topic", robot_name_.c_str());
        RCLCPP_INFO(node_->get_logger(), "MPC observation publisher created for %s_mpc_observation topic",
                    robot_name_.c_str());
    }

    void StateOCS2::updateObservationFromHardware(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        // 构建OCS2观测状态
        observation_.time = time.seconds();
        for (int i = 0; i < joint_names_.size(); ++i)
        {
            observation_.state[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
        }
    }

    void StateOCS2::setJointCommands()
    {
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
            
            // 输出调试信息
            RCLCPP_INFO(node_->get_logger(), "Using trajectory: current_time=%.3f, future_time=%.3f, dt=%.4f", 
                        observation_.time, future_time, dt);
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
} // namespace ocs2::mobile_manipulator
