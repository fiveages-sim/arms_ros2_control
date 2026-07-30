//
// Created for OCS2 Arm Controller - CtrlComponent
//

#include "ocs2_arm_controller/control/CtrlComponent.h"
#include "ocs2_arm_controller/Ocs2ArmController.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <arms_controller_common/utils/TrajectoryRecorder.h>
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

        // Lightweight EE/body pose: must keep streaming in HOLD too so markers can follow
        // measured EE before first OCS2 enter (and after mid-moveL HOLD). Heavy self-collision
        // / traj recording stays on the MPC-rate visualization thread.
        if (visualizer_)
        {
            visualizer_->publishEndEffectorPose(time, observation_.state);
        }
    }

    void CtrlComponent::evaluatePolicy(const rclcpp::Time& time)
    {
        if (!mpc_mrt_interface_)
        {
            RCLCPP_WARN(node_->get_logger(), "MPC MRT interface not available");
            return;
        }
        mpc_mrt_interface_->updatePolicy();
        // use cached action as current state if the hardware has some latency to predict next action
        bool trigger_cached_state = (time - last_execute_time_).seconds() < hardware_latency_;
        if (trigger_cached_state && cached_ob_state_)
        {
            observation_.state = cached_last_action_;
        }

        mpc_mrt_interface_->setCurrentObservation(observation_);

        const auto observation_msg = ros_msg_conversions::createObservationMsg(observation_);
        mpc_observation_publisher_->publish(observation_msg);

        size_t planned_mode = 0;
        try
        {
            // initialPolicyReceived() can be true while active policy is still null
            // (policy only in buffer until updatePolicy() swaps). Guard the whole path.
            mpc_mrt_interface_->evaluatePolicy(time.seconds(), observation_.state, optimized_state_, optimized_input_,
                                               planned_mode);
            const auto& policy = mpc_mrt_interface_->getPolicy();

            double future_time = observation_.time + future_time_offset_ / ctrl_interfaces_.frequency_;
            vector_t future_state = LinearInterpolation::interpolate(
                future_time,
                policy.timeTrajectory_,
                policy.stateTrajectory_
            );
            vector_t future_input = LinearInterpolation::interpolate(
                future_time,
                policy.timeTrajectory_,
                policy.inputTrajectory_
            );

            {
                std::lock_guard<std::mutex> lock(visualization_mutex_);
                visualization_has_pred_ = true;
                visualization_pred_time_ = future_time;
                visualization_pred_state_ = future_state;
            }

            // Trajectory markers: same RT thread as updatePolicy/getPolicy (MRT-safe).
            if (visualizer_)
            {
                visualizer_->updateEndEffectorTrajectory(policy);
                visualizer_->publishEndEffectorTrajectory(node_->now());
            }

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
        catch (const std::exception& e)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                 "Policy not ready / trajectory eval failed, holding: %s", e.what());
            holdLastSentPositions();
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
            vector_t left_initial_ee_state = visualizer_->computeEndEffectorPose(observation_.state);
            vector_t right_initial_ee_state = visualizer_->computeRightEndEffectorPose(observation_.state);

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

    void CtrlComponent::advanceMpc()
    {
        mpc_mrt_interface_->advanceMpc();
    }

    void CtrlComponent::startVisualizationThread(int thread_sleep_ms)
    {
        if (visualization_running_)
        {
            return;
        }
        visualization_thread_sleep_ms_ = std::max(1, thread_sleep_ms);
        visualization_thread_should_stop_ = false;
        visualization_thread_finished_ = false;
        visualization_running_ = true;
        // Do not set visualization_update_requested_ here — caller must call
        // requestVisualizationUpdate() after observation_ is ready (e.g. post-resetMpc),
        // otherwise the first snapshot can be empty/stale.
        visualization_thread_ = std::thread(&CtrlComponent::visualizationThreadLoop, this);
    }

    void CtrlComponent::requestStopVisualizationThread()
    {
        if (!visualization_running_)
        {
            visualization_thread_finished_ = true;
            return;
        }
        visualization_thread_should_stop_ = true;
    }

    bool CtrlComponent::isVisualizationThreadFinished() const
    {
        return visualization_thread_finished_.load();
    }

    void CtrlComponent::joinVisualizationThread()
    {
        if (visualization_thread_.joinable())
        {
            visualization_thread_.join();
        }
        visualization_running_ = false;
        visualization_thread_finished_ = true;
    }

    void CtrlComponent::stopVisualizationThread()
    {
        requestStopVisualizationThread();
        joinVisualizationThread();
    }

    void CtrlComponent::requestVisualizationUpdate()
    {
        {
            std::lock_guard<std::mutex> lock(visualization_mutex_);
            visualization_observation_ = observation_;
            visualization_stamp_ = node_->now();
        }
        visualization_update_requested_ = true;
    }

    void CtrlComponent::visualizationThreadLoop()
    {
        RCLCPP_DEBUG(node_->get_logger(),
                     "Visualization thread started (sleep=%d ms, MPC-rate)",
                     visualization_thread_sleep_ms_);
        while (!visualization_thread_should_stop_.load())
        {
            if (visualization_update_requested_.load())
            {
                SystemObservation obs;
                rclcpp::Time stamp(0, 0, RCL_ROS_TIME);
                bool has_pred = false;
                double pred_time = 0.0;
                vector_t pred_state;
                {
                    std::lock_guard<std::mutex> lock(visualization_mutex_);
                    obs = visualization_observation_;
                    stamp = visualization_stamp_;
                    has_pred = visualization_has_pred_;
                    pred_time = visualization_pred_time_;
                    pred_state = visualization_pred_state_;
                    visualization_has_pred_ = false;
                }
                try
                {
                    runVisualizationOnce(obs, stamp, has_pred, pred_time, pred_state);
                }
                catch (const std::exception& e)
                {
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                                         "Visualization update failed: %s", e.what());
                }
                visualization_update_requested_ = false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(visualization_thread_sleep_ms_));
        }
        visualization_thread_finished_ = true;
        RCLCPP_DEBUG(node_->get_logger(), "Visualization thread stopped");
    }

    void CtrlComponent::runVisualizationOnce(const SystemObservation& obs, const rclcpp::Time& /*stamp*/,
                                             bool has_pred, double pred_time, const vector_t& pred_state)
    {
        if (!visualizer_ || obs.state.size() == 0)
        {
            return;
        }

        visualizer_->publishSelfCollisionVisualization(obs.state);
        // EE pose is published every RT cycle in updateObservation (incl. HOLD).

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

            if (has_pred && pred_state.size() > 0)
            {
                arms_controller_common::TrajSample lps;
                lps.stamp_sec = pred_time;
                const vector_t lpe = visualizer_->computeEndEffectorPose(pred_state);
                lps.position = {lpe(0), lpe(1), lpe(2)};
                lps.quat_xyzw = {lpe(3), lpe(4), lpe(5), lpe(6)};
                rec.appendPred("left", lps);
                if (dual_arm_mode_)
                {
                    arms_controller_common::TrajSample rps;
                    rps.stamp_sec = pred_time;
                    const vector_t rpe = visualizer_->computeRightEndEffectorPose(pred_state);
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
        // Get the Pinocchio model and data from the interface
        const auto& pinocchio_model = interface_->getPinocchioInterface().getModel();
        auto pinocchio_data = interface_->getPinocchioInterface().getData();


        Eigen::VectorXd q = observation_.state;
        return pinocchio::rnea(pinocchio_model, pinocchio_data, q,
                               Eigen::VectorXd::Zero(pinocchio_model.nv),
                               Eigen::VectorXd::Zero(pinocchio_model.nv));
    }

    bool CtrlComponent::isCollisionDetected(scalar_t threshold) const
    {
        // Reuse the cached value from visualization (no extra computation)
        return visualizer_->isCollisionDetected(threshold);
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
