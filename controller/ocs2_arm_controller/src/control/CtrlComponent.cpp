//
// Created for OCS2 Arm Controller - CtrlComponent
//

#include "ocs2_arm_controller/control/CtrlComponent.h"
#include "ocs2_arm_controller/Ocs2ArmController.h"
#include <ocs2_controller_common/rt/ThreadScheduling.h>
#include <ocs2_core/thread_support/ThreadPool.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <arms_controller_common/utils/TrajectoryRecorder.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <exception>
#include <algorithm>
#include <filesystem>
#include <chrono>
#include <optional>

namespace ocs2::mobile_manipulator
{
    FrameOverrides loadInfoFrameDefaults(const std::string& task_file)
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_info(task_file, pt);
        FrameOverrides frames;
        loadData::loadPtreeValue<std::string>(pt, frames.baseFrame, "model_information.baseFrame", false);
        loadData::loadPtreeValue<std::string>(pt, frames.eeFrame, "model_information.eeFrame", false);
        loadData::loadPtreeValue<std::string>(pt, frames.eeFrame1, "model_information.eeFrame1", false);
        return frames;
    }

    rcl_interfaces::msg::SetParametersResult CtrlComponent::on_parameter_change(
        const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        // Apply dir before enabled so a same-batch update uses the new directory.
        std::optional<bool> traj_record_enabled_update;
        for (const auto& param : parameters)
        {
            if (param.get_name() == "hardware_latency")
            {
                hardware_latency_ = param.as_double();
                RCLCPP_INFO(node_->get_logger(), "Updated hardware_latency to: %f", hardware_latency_);
            }
            else if (param.get_name() == "traj_record_dir")
            {
                traj_record_dir_ = param.as_string();
            }
            else if (param.get_name() == "traj_record_enabled")
            {
                traj_record_enabled_update = param.as_bool();
            }
            else if (param.get_name() == "rt_timing_enabled")
            {
                rt_timing_.enabled.store(param.as_bool(), std::memory_order_relaxed);
                RCLCPP_INFO(node_->get_logger(),
                            "rt_timing_enabled=%s. Remote-desktop: HOLD 20s / OCS2 idle 20s / "
                            "drag markers 20s / HOLD. Collect [rt_timing] and Overrun detected.",
                            param.as_bool() ? "true" : "false");
            }
        }
        if (traj_record_enabled_update.has_value())
        {
            arms_controller_common::TrajectoryRecorder::instance()
                .setEnabled(*traj_record_enabled_update, traj_record_dir_);
            RCLCPP_INFO(node_->get_logger(), "traj_record_enabled=%d, dir=%s",
                        *traj_record_enabled_update, traj_record_dir_.c_str());
        }

        return result;
    }


    void CtrlComponent::setupInterface(const std::string& task_file,
                                       const std::string& lib_folder,
                                       const std::string& urdf_file,
                                       const FrameOverrides& frame_overrides)
    {
        // Create Mobile Manipulator interface (frame overrides applied in-memory)
        interface_ = std::make_shared<MobileManipulatorInterface>(
            task_file, lib_folder, urdf_file, frame_overrides);
        task_file_ = task_file;
        urdf_file_ = urdf_file;
        // Setup publishers
        setupPublisher();
    }

    void CtrlComponent::setupPublisher()
    {
        // Initialize MPC observation publisher
        mpc_observation_publisher_ = node_->create_publisher<ocs2_msgs::msg::MpcObservation>(
            robot_name_ + "_mpc_observation", 1);

        // Initialize FSM command publisher (for stopping all controllers)
        fsm_command_publisher_ = node_->create_publisher<std_msgs::msg::Int32>("/fsm_command", 10);
    }

    void CtrlComponent::updateObservation(const rclcpp::Time& time)
    {
        if (!mpc_mrt_interface_)
        {
            return;
        }
        // Update observation state
        observation_.time = time.seconds();
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            auto value = ctrl_interfaces_.joint_position_state_interface_[i].get().get_optional();
            observation_.state[i] = value.value_or(0.0);
        }
        observation_.input = vector_t::Zero(interface_->getManipulatorModelInfo().inputDim);

        // 更新PoseBasedReferenceManager的当前观测
        if (pose_reference_manager_)
        {
            pose_reference_manager_->setCurrentObservation(observation_);
        }

        if (visualizer_)
        {
            ocs2::controller_common::Ocs2PinocchioVisualizer::EndEffectorPoses poses;
            {
                auto fk_scope = rt_timing_.scope(&rt_timing_.ee_fk_us);
                poses = visualizer_->computeEndEffectorPoses(observation_.state);
            }
            auto pub_scope = rt_timing_.scope(&rt_timing_.ee_pub_us);
            visualizer_->publishEndEffectorPoses(time, poses);
        }
    }

    void CtrlComponent::evaluatePolicy(const rclcpp::Time& time)
    {
        auto eval_scope = rt_timing_.scope(&rt_timing_.eval_us);
        if (!mpc_mrt_interface_)
        {
            RCLCPP_WARN(node_->get_logger(), "MPC MRT interface not available");
            return;
        }
        {
            auto upd_scope = rt_timing_.scope(&rt_timing_.eval_upd_us);
            if (mpc_mrt_interface_->updatePolicy())
            {
                policy_active_ = true;
            }
        }
        bool trigger_cached_state = (time - last_execute_time_).seconds() < hardware_latency_;
        if (trigger_cached_state && cached_ob_state_)
        {
            observation_.state = cached_last_action_;
        }

        mpc_mrt_interface_->setCurrentObservation(observation_);
        {
            auto pub_scope = rt_timing_.scope(&rt_timing_.eval_obs_pub_us);
            if (shouldPublishDds(time))
            {
                mpc_observation_publisher_->publish(ros_msg_conversions::createObservationMsg(observation_));
            }
        }

        if (!policy_active_)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "Policy not ready (no active MRT policy yet), holding");
            holdLastSentPositions();
            return;
        }

        size_t planned_mode = 0;
        vector_t future_state;
        vector_t future_input;
        {
            auto mrt_scope = rt_timing_.scope(&rt_timing_.eval_mrt_us);
            mpc_mrt_interface_->evaluatePolicy(time.seconds(), observation_.state, optimized_state_, optimized_input_,
                                               planned_mode);
            const auto& policy = mpc_mrt_interface_->getPolicy();
            const double future_time = observation_.time + future_time_offset_ / ctrl_interfaces_.frequency_;
            future_state = LinearInterpolation::interpolate(
                future_time, policy.timeTrajectory_, policy.stateTrajectory_);
            future_input = LinearInterpolation::interpolate(
                future_time, policy.timeTrajectory_, policy.inputTrajectory_);
            pending_viz_has_pred_ = true;
            pending_viz_pred_time_ = future_time;
            pending_viz_pred_state_ = future_state;
        }

        auto cmd_scope = rt_timing_.scope(&rt_timing_.eval_cmd_us);
        if (ctrl_interfaces_.control_mode_ == ControlMode::POSITION)
        {
            for (size_t i = 0; i < joint_names_.size() && i < static_cast<size_t>(future_state.size()); ++i)
            {
                ctrl_interfaces_.setJointPositionCommand(i, future_state(i));
            }
            cached_last_action_ = future_state;
            last_execute_time_ = time;
        }
        else if (ctrl_interfaces_.control_mode_ == ControlMode::MIX)
        {
            vector_t static_torques = calculateStaticTorques();
            for (size_t i = 0; i < joint_names_.size() && i < static_cast<size_t>(static_torques.size()); ++i)
            {
                std::ignore = ctrl_interfaces_.joint_force_command_interface_[i].get().set_value(static_torques(i));
            }
            for (size_t i = 0; i < joint_names_.size() && i < static_cast<size_t>(future_state.size()); ++i)
            {
                ctrl_interfaces_.setJointPositionCommand(i, future_state(i));
            }
            for (size_t i = 0; i < joint_names_.size() && i < static_cast<size_t>(future_input.size()); ++i)
            {
                std::ignore = ctrl_interfaces_.joint_velocity_command_interface_[i].get().
                    set_value(future_input(i));
            }
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Unknown control output mode");
        }
    }

    void CtrlComponent::resetMpc()
    {
        // Update observation time to current time
        observation_.time = node_->now().seconds();

        // Use last sent joint positions for initial state (avoids jumps when entering OCS2 state)
        for (size_t i = 0; i < joint_names_.size() && i < ctrl_interfaces_.last_sent_joint_positions_.size(); ++i)
        {
            observation_.state[i] = ctrl_interfaces_.last_sent_joint_positions_[i];
        }

        TargetTrajectories target_trajectories;

        if (dual_arm_mode_)
        {
            const auto poses = visualizer_->computeEndEffectorPoses(observation_.state);
            vector_t left_initial_ee_state = poses.left;
            vector_t right_initial_ee_state = poses.right;

            // Output initial target trajectory information
            RCLCPP_INFO(node_->get_logger(),
                        "Initial Target Trajectory - Time: %.3f", observation_.time);
            RCLCPP_INFO(node_->get_logger(),
                        "Left EE State: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                        left_initial_ee_state(0), left_initial_ee_state(1), left_initial_ee_state(2),
                        // Position x, y, z
                        left_initial_ee_state(3), left_initial_ee_state(4), left_initial_ee_state(5),
                        left_initial_ee_state(6)); // Quaternion w, x, y, z
            RCLCPP_INFO(node_->get_logger(),
                        "Right EE State: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                        right_initial_ee_state(0), right_initial_ee_state(1), right_initial_ee_state(2),
                        // Position x, y, z
                        right_initial_ee_state(3), right_initial_ee_state(4), right_initial_ee_state(5),
                        right_initial_ee_state(6)); // Quaternion w, x, y, z

            // 将当前末端执行器pose设置到PoseBasedReferenceManager缓存
            if (pose_reference_manager_)
            {
                pose_reference_manager_->setCurrentObservation(observation_);
                pose_reference_manager_->setCurrentEndEffectorPoses(left_initial_ee_state, right_initial_ee_state,
                                                                    false);
                // Explicit enter publish: markers subscribe to *_current_target.
                pose_reference_manager_->publishCurrentTargetsFromCache();
            }

            // Dual arm mode: create target trajectory containing two end effectors
            // 14-dimensional state vector: [left_x, left_y, left_z, left_qw, left_qx, left_qy, left_qz,
            //                               right_x, right_y, right_z, right_qw, right_qx, right_qy, right_qz]
            vector_t dual_arm_target = (vector_t(14) <<
                left_initial_ee_state, right_initial_ee_state).finished();

            target_trajectories = TargetTrajectories({observation_.time},
                                                     {dual_arm_target},
                                                     {observation_.input});
        }
        else
        {
            // Single arm mode: maintain original logic
            // Calculate initial end effector position and orientation
            vector_t initial_ee_state = visualizer_->computeEndEffectorPose(observation_.state);

            // Output initial target trajectory information
            RCLCPP_INFO(node_->get_logger(),
                        "Initial Target Trajectory - Time: %.3f, EE State: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                        observation_.time,
                        initial_ee_state(0), initial_ee_state(1), initial_ee_state(2), // Position x, y, z
                        initial_ee_state(3), initial_ee_state(4), initial_ee_state(5),
                        initial_ee_state(6)); // Quaternion w, x, y, z

            // 将当前末端执行器pose设置到PoseBasedReferenceManager缓存（单臂模式，右臂使用零向量）
            if (pose_reference_manager_)
            {
                pose_reference_manager_->setCurrentObservation(observation_);
                vector_t zero_pose = vector_t::Zero(7);
                pose_reference_manager_->setCurrentEndEffectorPoses(initial_ee_state, zero_pose, false);
                pose_reference_manager_->publishCurrentTargetsFromCache();
            }

            // Initialize TargetTrajectories - use end effector position and orientation
            target_trajectories = TargetTrajectories({observation_.time},
                                                     {initial_ee_state},
                                                     {observation_.input});
        }

        // Set initial observation and target trajectory (non-blocking).
        // Waiting for initialPolicyReceived() must happen on the MPC worker thread —
        // blocking here runs inside controller_manager::update and causes overruns.
        mpc_mrt_interface_->reset();
        policy_active_ = false;
        collision_active_.store(false, std::memory_order_release);
        mpc_mrt_interface_->setCurrentObservation(observation_);
        mpc_mrt_interface_->resetMpcNode(target_trajectories);
        RCLCPP_INFO(node_->get_logger(),
                    "MPC reset complete; waiting for initial policy on MPC thread");
    }

    bool CtrlComponent::initialPolicyReceived() const
    {
        return mpc_mrt_interface_ && mpc_mrt_interface_->initialPolicyReceived();
    }

    void CtrlComponent::holdLastSentPositions() const
    {
        for (size_t i = 0; i < joint_names_.size() && i < ctrl_interfaces_.last_sent_joint_positions_.size(); ++i)
        {
            ctrl_interfaces_.setJointPositionCommand(i, ctrl_interfaces_.last_sent_joint_positions_[i]);
        }
    }

    void CtrlComponent::publishCachedCurrentTargets() const
    {
        if (pose_reference_manager_)
        {
            pose_reference_manager_->publishCurrentTargetsFromCache();
        }
    }

    void CtrlComponent::setMpcPeriodSec(double period_sec)
    {
        if (period_sec <= 0.0)
        {
            return;
        }
        mpc_period_us_.store(static_cast<int64_t>(period_sec * 1e6 + 0.5), std::memory_order_relaxed);
    }

    void CtrlComponent::advanceMpc()
    {
        const auto t0 = std::chrono::steady_clock::now();
        mpc_mrt_interface_->advanceMpc();
        const auto us = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now() - t0)
                            .count();
        const auto period_us = mpc_period_us_.load(std::memory_order_relaxed);
        if (period_us > 0 && us > period_us)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "MPC solve %ld us exceeds period %ld us (%.1f Hz); tracking will lag",
                                 static_cast<long>(us), static_cast<long>(period_us),
                                 1e6 / static_cast<double>(period_us));
        }
    }

    void CtrlComponent::startVisualizationThread(int thread_sleep_ms, double visualization_period_sec)
    {
        if (visualization_running_.load())
        {
            return;
        }
        visualization_thread_sleep_ms_ = std::max(1, thread_sleep_ms);
        visualization_period_sec_ = std::max(0.001, visualization_period_sec);
        if (visualizer_)
        {
            visualizer_->setPosePublishPeriod(visualization_period_sec_);
        }
        visualization_running_ = true;
        // Do not set visualization_update_requested_ here — caller must call
        // requestVisualizationUpdate() after observation_ is ready (e.g. post-resetMpc),
        // otherwise the first snapshot can be empty/stale.
        visualization_thread_ = std::thread(&CtrlComponent::visualizationThreadLoop, this);
    }

    void CtrlComponent::stopVisualizationThread()
    {
        if (!visualization_running_.exchange(false) && !visualization_thread_.joinable())
        {
            return;
        }
        if (visualization_thread_.joinable())
        {
            visualization_thread_.join();
        }
        collision_active_.store(false, std::memory_order_release);
    }

    void CtrlComponent::requestVisualizationUpdate()
    {
        auto snap = std::make_shared<VisualizationSnapshot>();
        snap->observation = observation_;
        snap->collision_state = buildCollisionState();
        snap->has_pred = pending_viz_has_pred_;
        snap->pred_time = pending_viz_pred_time_;
        snap->pred_state = std::move(pending_viz_pred_state_);
        pending_viz_has_pred_ = false;
        pending_viz_pred_time_ = 0.0;
        {
            auto traj_scope = rt_timing_.scope(&rt_timing_.eval_traj_us);
            if (policy_active_ && mpc_mrt_interface_)
            {
                snap->stateTrajectory = mpc_mrt_interface_->getVisualizationStateTrajectory();
                snap->has_policy_traj = snap->stateTrajectory && !snap->stateTrajectory->empty();
            }
        }
        std::atomic_store_explicit(&visualization_snapshot_, std::move(snap), std::memory_order_release);
        visualization_update_requested_ = true;
    }

    void CtrlComponent::maybeRequestVisualizationUpdate(const rclcpp::Time& time)
    {
        if (!visualization_running_)
        {
            return;
        }
        if (last_visualization_request_time_.nanoseconds() != 0 &&
            (time - last_visualization_request_time_).seconds() < visualization_period_sec_)
        {
            return;
        }
        requestVisualizationUpdate();
        last_visualization_request_time_ = time;
    }

    void CtrlComponent::visualizationThreadLoop()
    {
        applyWorkerThreadScheduling("viz");
        RCLCPP_DEBUG(node_->get_logger(),
                     "Visualization thread started (sleep=%d ms, MPC-rate)",
                     visualization_thread_sleep_ms_);
        while (visualization_running_.load())
        {
            if (visualization_update_requested_.load())
            {
                auto snap = std::atomic_load_explicit(&visualization_snapshot_, std::memory_order_acquire);
                if (snap)
                {
                    runVisualizationOnce(*snap);
                }
                visualization_update_requested_ = false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(visualization_thread_sleep_ms_));
        }
        RCLCPP_DEBUG(node_->get_logger(), "Visualization thread stopped");
    }

    void CtrlComponent::runVisualizationOnce(const VisualizationSnapshot& snap)
    {
        const auto& obs = snap.observation;
        if (!visualizer_ || obs.state.size() == 0)
        {
            return;
        }

        // One FCL per viz tick on command-overlay q: updates markers + collision_active_ for RT.
        const vector_t& collision_q =
            snap.collision_state.size() > 0 ? snap.collision_state : obs.state;
        bool active = false;
        if (visualizer_->publishSelfCollisionVisualization(collision_q))
        {
            const scalar_t thr = interface_->getSelfCollisionMinimumDistance();
            active = visualizer_->getLastMinDistance() <= thr;
        }
        collision_active_.store(active, std::memory_order_release);

        if (snap.has_policy_traj && snap.stateTrajectory && !snap.stateTrajectory->empty())
        {
            visualizer_->updateEndEffectorTrajectoryFromStates(*snap.stateTrajectory);
            visualizer_->publishEndEffectorTrajectory(node_->now());
        }

        auto& rec = arms_controller_common::TrajectoryRecorder::instance();
        if (rec.enabled())
        {
            const double t = obs.time;
            const vector_t lp = visualizer_->computeEndEffectorPose(obs.state);
            arms_controller_common::TrajSample ls;
            ls.stamp_sec = t;
            ls.position = {lp(0), lp(1), lp(2)};
            ls.quat_xyzw = {lp(3), lp(4), lp(5), lp(6)};
            rec.appendReal("left", ls);
            if (dual_arm_mode_)
            {
                const vector_t rp = visualizer_->computeRightEndEffectorPose(obs.state);
                arms_controller_common::TrajSample rs;
                rs.stamp_sec = t;
                rs.position = {rp(0), rp(1), rp(2)};
                rs.quat_xyzw = {rp(3), rp(4), rp(5), rp(6)};
                rec.appendReal("right", rs);
            }

            if (snap.has_pred && snap.pred_state.size() > 0)
            {
                arms_controller_common::TrajSample lps;
                lps.stamp_sec = snap.pred_time;
                const vector_t lpe = visualizer_->computeEndEffectorPose(snap.pred_state);
                lps.position = {lpe(0), lpe(1), lpe(2)};
                lps.quat_xyzw = {lpe(3), lpe(4), lpe(5), lpe(6)};
                rec.appendPred("left", lps);
                if (dual_arm_mode_)
                {
                    arms_controller_common::TrajSample rps;
                    rps.stamp_sec = snap.pred_time;
                    const vector_t rpe = visualizer_->computeRightEndEffectorPose(snap.pred_state);
                    rps.position = {rpe(0), rpe(1), rpe(2)};
                    rps.quat_xyzw = {rpe(3), rpe(4), rpe(5), rpe(6)};
                    rec.appendPred("right", rps);
                }
            }
        }
    }

    void CtrlComponent::clearTrajectoryVisualization()
    {
        visualizer_->clearTrajectoryHistory();
        pose_reference_manager_->resetTargetStateCache();
    }

    std::string CtrlComponent::resolvePlanningUrdfPath() const
    {
        if (planning_urdf_variant_ != "xacro" || planning_urdf_path_.empty())
        {
            throw std::runtime_error(
                "OCS2 planning URDF must come from xacro (planning_urdf_variant:=xacro, "
                "planning_urdf_path set by robot_common_launch). Static urdf/ lookup was removed.");
        }
        if (!std::filesystem::exists(planning_urdf_path_))
        {
            throw std::runtime_error(
                "planning_urdf_path does not exist: " + planning_urdf_path_);
        }
        RCLCPP_INFO(node_->get_logger(),
                    "Using xacro-generated planning URDF: %s",
                    planning_urdf_path_.c_str());
        return planning_urdf_path_;
    }

    vector_t CtrlComponent::calculateStaticTorques() const
    {
        const auto& pinocchio_model = interface_->getPinocchioInterface().getModel();
        if (!rnea_data_)
        {
            rnea_data_.emplace(pinocchio_model);
        }
        Eigen::VectorXd q = observation_.state;
        return pinocchio::rnea(pinocchio_model, *rnea_data_, q,
                               Eigen::VectorXd::Zero(pinocchio_model.nv),
                               Eigen::VectorXd::Zero(pinocchio_model.nv));
    }

    bool CtrlComponent::shouldPublishDds(const rclcpp::Time& time)
    {
        if (dds_publish_period_sec_ <= 0.0)
        {
            return true;
        }
        if (last_dds_publish_time_.nanoseconds() != 0 &&
            (time - last_dds_publish_time_).seconds() < dds_publish_period_sec_)
        {
            return false;
        }
        last_dds_publish_time_ = time;
        return true;
    }

    void CtrlComponent::applyWorkerThreadScheduling(const char* thread_name) const
    {
        const bool is_mpc = thread_name != nullptr && std::string(thread_name) == "MPC";
        if (is_mpc)
        {
            const std::vector<int>& cpus = mpc_thread_cpus_.empty() ? cpu_affinity_ : mpc_thread_cpus_;
            ocs2::controller_common::applyThisThreadSchedulingLogged(
                thread_priority_, cpus, node_->get_logger(), thread_name);
            return;
        }
        // viz: CFS on the MPC core. FIFO 85 on DDP cores would preempt DDP workers (priority 50).
        const std::vector<int>& cpus = mpc_thread_cpus_.empty() ? cpu_affinity_ : mpc_thread_cpus_;
        ocs2::controller_common::applyThisThreadSchedulingLogged(0, cpus, node_->get_logger(), thread_name);
    }

    void CtrlComponent::applyRtLoopSchedulingOnce()
    {
        if (rt_loop_scheduling_applied_)
        {
            return;
        }
        rt_loop_scheduling_applied_ = true;
        if (rt_loop_cpus_.empty())
        {
            return;
        }
        ocs2::controller_common::applyThisThreadCpuAffinity(rt_loop_cpus_);
        RCLCPP_INFO(node_->get_logger(), "Pinned RT control loop to CPU [%s]",
                    ocs2::controller_common::formatCpuList(rt_loop_cpus_).c_str());
    }

    void CtrlComponent::prepareThreadIsolation()
    {
        const auto plan = ocs2::controller_common::planIsolatedCpus(cpu_affinity_);
        mpc_thread_cpus_ = plan.mpc;
        rt_loop_cpus_ = plan.rt;
        worker_thread_cpus_ = plan.workers;
        if (plan.isolated)
        {
            ocs2::ThreadPool::setLaunchWorkerCpuAffinity(plan.workers);
            RCLCPP_INFO(node_->get_logger(),
                        "CPU isolation: RT=[%s] MPC=[%s] DDP=[%s] (viz shares MPC core at CFS)",
                        ocs2::controller_common::formatCpuList(plan.rt).c_str(),
                        ocs2::controller_common::formatCpuList(plan.mpc).c_str(),
                        ocs2::controller_common::formatCpuList(plan.workers).c_str());
        }
    }

    vector_t CtrlComponent::buildCollisionState() const
    {
        vector_t q = observation_.state;
        const auto& cmd = ctrl_interfaces_.last_sent_joint_positions_;
        for (size_t i = 0; i < joint_names_.size() && i < cmd.size() && i < static_cast<size_t>(q.size()); ++i)
        {
            q[static_cast<Eigen::Index>(i)] = cmd[i];
        }
        return q;
    }

    void CtrlComponent::publishFsmCommand(int32_t command) const
    {
        if (fsm_command_publisher_)
        {
            std_msgs::msg::Int32 fsm_cmd;
            fsm_cmd.data = command;
            fsm_command_publisher_->publish(fsm_cmd);
        }
    }
} // namespace ocs2::mobile_manipulator
