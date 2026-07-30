//
// Created for OCS2 Arm Controller - CtrlComponent
//
#pragma once


#include <cstdlib>

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ocs2_controller_common/reference/PoseBasedReferenceManager.h>
#include <ocs2_controller_common/visualization/Ocs2PinocchioVisualizer.h>
#include <ocs2_core/Types.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/int32.hpp>

#include <arms_controller_common/CtrlInterfaces.h>
#include <arms_controller_common/utils/TrajectoryRecorder.h>

namespace ocs2::mobile_manipulator
{
    // Use CtrlInterfaces from arms_controller_common
    using CtrlInterfaces = arms_controller_common::CtrlInterfaces;

    /** Writable directory for OCS2 CppAD generated libraries (must not live under read-only share/). */
    inline std::string defaultOcs2LibraryFolder(const std::string& robot_pkg)
    {
        if (const char* xdg = std::getenv("XDG_CACHE_HOME"); xdg && xdg[0] != '\0')
        {
            return std::string(xdg) + "/ocs2_ros2/" + robot_pkg;
        }
        if (const char* home = std::getenv("HOME"); home && home[0] != '\0')
        {
            return std::string(home) + "/.ros/ocs2_cache/" + robot_pkg;
        }
        return std::string("/tmp/ocs2_ros2/") + robot_pkg;
    }

    /** Read model frame defaults from task.info (eeFrame=left, eeFrame1=right). */
    FrameOverrides loadInfoFrameDefaults(const std::string& task_file);

    class CtrlComponent
    {
    public:
        template <typename AutoDeclareFunc>
        explicit CtrlComponent(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                               CtrlInterfaces& ctrl_interfaces,
                               AutoDeclareFunc auto_declare)
            : node_(node), ctrl_interfaces_(ctrl_interfaces)
        {
            cached_ob_state_ = auto_declare("cached_ob_state", true);
            joint_speed_threshold_ = auto_declare("joint_speed_threshold", 0.1);
            hardware_latency_ = auto_declare("hardware_latency", 0.2);
            // Declare dir before enabled so both can be applied together from yaml/callback.
            traj_record_dir_ = auto_declare("traj_record_dir", std::string("/tmp/traj_record"));
            const bool traj_record_enabled = auto_declare("traj_record_enabled", false);
            // Sync initial yaml/launch value (parameter callbacks do not fire on declare).
            arms_controller_common::TrajectoryRecorder::instance()
                .setEnabled(traj_record_enabled, traj_record_dir_);

            parameter_callback_handle_ = node_->add_on_set_parameters_callback(
                [this](const std::vector<rclcpp::Parameter>& parameters) {
                    return on_parameter_change(parameters);
                });

            robot_name_ = auto_declare("robot_name", std::string("cr5"));
            planning_urdf_variant_ = auto_declare("planning_urdf_variant", std::string(""));
            planning_urdf_path_ = auto_declare("planning_urdf_path", std::string(""));
            future_time_offset_ = auto_declare("future_time_offset", 1.0);
            const std::string info_file_name = auto_declare("info_file_name", std::string("task"));
            joint_names_ = node_->get_parameter("joints").as_string_array();
            const std::string robot_pkg = robot_name_ + "_description";
            const std::string config_path = ament_index_cpp::get_package_share_directory(robot_pkg);

            const std::string task_file = config_path + "/config/ocs2/" + info_file_name + ".info";
            const std::string lib_folder =
                auto_declare("ocs2_library_folder", defaultOcs2LibraryFolder(robot_pkg));
            const std::string urdf_file = resolvePlanningUrdfPath();

            // Frame params: YAML overrides task.info defaults; applied in-memory into Interface.
            const auto info_frames = loadInfoFrameDefaults(task_file);
            const std::string base_frame =
                auto_declare("base_frame", info_frames.baseFrame);
            const std::string left_ee_frame =
                auto_declare("left_ee_frame", info_frames.eeFrame);
            const std::string right_ee_frame =
                auto_declare("right_ee_frame", info_frames.eeFrame1);
            FrameOverrides frame_overrides;
            frame_overrides.baseFrame = base_frame;
            frame_overrides.eeFrame = left_ee_frame;
            frame_overrides.eeFrame1 = right_ee_frame;

            setupInterface(task_file, lib_folder, urdf_file, frame_overrides);

            dual_arm_mode_ = interface_->dual_arm_;

            setupPublisher();

            {
                ocs2::controller_common::Ocs2VisualizerConfig viz_cfg;
                viz_cfg.urdf_file = urdf_file;
                viz_cfg.dual_arm = interface_->dual_arm_;
                const auto& minfo = interface_->getManipulatorModelInfo();
                viz_cfg.base_frame = minfo.baseFrame;
                viz_cfg.left_ee_frame = minfo.eeFrame;
                viz_cfg.right_ee_frame = minfo.eeFrame1;
                if (auto geo = interface_->getPinocchioGeometryInterface())
                {
                    viz_cfg.pinocchio_geometry = std::move(*geo);
                    viz_cfg.self_collision_activation_distance = interface_->getSelfCollisionActivationDistance();
                }
                visualizer_ = std::make_unique<ocs2::controller_common::Ocs2PinocchioVisualizer>(
                    node_, interface_->getPinocchioInterface(), std::move(viz_cfg));
            }
            visualizer_->initialize();
            RCLCPP_INFO(node_->get_logger(), "Future time offset: %.2f seconds", future_time_offset_);

            auto_declare("movel_trajectory_duration", 2.0);
            auto_declare("movel_duration", 2.0);
            auto_declare("movel_sample_interval", 0.04);
            auto_declare("movel_max_linear_velocity", 0.3);
            auto_declare("movel_max_linear_acceleration", 1.0);
            auto_declare("movel_max_linear_jerk", 2.0);
            auto_declare("movel_max_angular_velocity", 1.0);
            auto_declare("movel_max_angular_acceleration", 2.0);
            auto_declare("movel_max_angular_jerk", 4.0);
            auto_declare("movel_auto_extend_duration", true);

            controller_common::Ocs2ReferenceTargetContext ref_ctx;
            ref_ctx.dual_arm = interface_->dual_arm_;
            ref_ctx.base_frame = interface_->getManipulatorModelInfo().baseFrame;
            ref_ctx.input_dim = interface_->getManipulatorModelInfo().inputDim;
            pose_reference_manager_ = std::make_shared<ocs2::controller_common::PoseBasedReferenceManager>(
                robot_name_, interface_->getReferenceManagerPtr(), ref_ctx);
            pose_reference_manager_->subscribe(node_);

            mpc_ = std::make_unique<GaussNewtonDDP_MPC>(
                interface_->mpcSettings(),
                interface_->ddpSettings(),
                interface_->getRollout(),
                interface_->getOptimalControlProblem(),
                interface_->getInitializer());

            mpc_mrt_interface_ = std::make_unique<MPC_MRT_Interface>(*mpc_);

            mpc_->getSolverPtr()->setReferenceManager(pose_reference_manager_);

            observation_.state = interface_->getInitialState();
            observation_.input = vector_t::Zero(interface_->getManipulatorModelInfo().inputDim);
            observation_.time = 0.0;

            // Initialize cached state
            last_execute_time_ = node_->now();
            cached_last_action_ = observation_.state;
        }

        ~CtrlComponent() { stopVisualizationThread(); }

        void updateObservation(const rclcpp::Time& time);
        void evaluatePolicy(const rclcpp::Time& time);
        /** Reset observation/targets and MPC node. Does NOT block for the first policy (RT-safe). */
        void resetMpc();
        /** True once the first MPC policy after resetMpc() is available. */
        [[nodiscard]] bool initialPolicyReceived() const;
        void advanceMpc();
        /** Hold joints at last commanded positions (used while waiting for initial policy). */
        void holdLastSentPositions() const;
        /** Publish left/right(/body) current_target from cache (call on OCS2 enter for marker sync). */
        void publishCachedCurrentTargets() const;

        /**
         * Self-collision markers + traj recording at MPC cadence on a dedicated thread
         * (not RT update). Detection stays sync on RT for FSM safety.
         * Start on controller activate, stop on deactivate. Request updates via
         * maybeRequestVisualizationUpdate() from the common control path (all FSM states).
         */
        void startVisualizationThread(int thread_sleep_ms, double visualization_period_sec);
        /** Blocking stop: clear running + join (destructor / deactivate). */
        void stopVisualizationThread();
        void requestVisualizationUpdate();
        /** Throttle requestVisualizationUpdate to visualization_period_sec (MPC rate). */
        void maybeRequestVisualizationUpdate(const rclcpp::Time& time);

        // Visualization management
        void clearTrajectoryVisualization();

        // Torque calculation for force control
        vector_t calculateStaticTorques() const;

        /** Sync check on full OCS2 state via publishDistances (may also publish markers). */
        bool checkSelfCollision(const vector_t& state, scalar_t threshold = 0.0) const;
        /**
         * Sync check using last commanded joints (and current observation base pose if any).
         * Intended for MOVEJ safety so collision follows command, not lagged measurement.
         */
        bool checkSelfCollisionOnCommand(scalar_t threshold = 0.0) const;
        /** Sync check using current observation_.state (measured). For OCS2 FSM. */
        bool checkSelfCollisionOnObservation(scalar_t threshold = 0.0) const;

        // Publish FSM command to stop all controllers
        // @param command: FSM command value (typically 2 for HOLD)
        void publishFsmCommand(int32_t command) const;

        // Get node reference
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> getNode() const { return node_; }

        // OCS2 interface (public access)
        std::shared_ptr<MobileManipulatorInterface> interface_;

        // MPC components
        std::unique_ptr<MPC_BASE> mpc_;
        std::unique_ptr<MPC_MRT_Interface> mpc_mrt_interface_;
        std::shared_ptr<controller_common::PoseBasedReferenceManager> pose_reference_manager_;

        // Observation state
        SystemObservation observation_;
        vector_t optimized_state_;
        vector_t optimized_input_;
        std::string getUrdfFilePath() const { return urdf_file_; }
        std::string getTaskFilePath() const { return task_file_; }

    private:
        void setupInterface(const std::string& task_file,
                            const std::string& lib_folder,
                            const std::string& urdf_file,
                            const FrameOverrides& frame_overrides = FrameOverrides{});
        void setupPublisher();

        /** Requires launch-injected xacro planning URDF (planning_urdf_path). */
        std::string resolvePlanningUrdfPath() const;

        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        CtrlInterfaces& ctrl_interfaces_;

        rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr mpc_observation_publisher_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr fsm_command_publisher_;

        // Visualization component
        std::unique_ptr<ocs2::controller_common::Ocs2PinocchioVisualizer> visualizer_;

        // Visualization worker (MPC-rate, off RT update path)
        struct VisualizationSnapshot
        {
            SystemObservation observation;
            bool has_pred{false};
            double pred_time{0.0};
            vector_t pred_state;
        };

        void visualizationThreadLoop();
        void runVisualizationOnce(const VisualizationSnapshot& snap);

        std::thread visualization_thread_;
        std::atomic_bool visualization_running_{false};
        std::atomic_bool visualization_update_requested_{false};
        int visualization_thread_sleep_ms_{2};
        double visualization_period_sec_{0.05};
        rclcpp::Time last_visualization_request_time_{0, 0, RCL_ROS_TIME};
        /** Published via atomic_store/load only (RT → viz handoff, no mutex). */
        std::shared_ptr<VisualizationSnapshot> visualization_snapshot_;
        /** RT-only staging for pred samples; folded into snapshot in requestVisualizationUpdate(). */
        bool pending_viz_has_pred_{false};
        double pending_viz_pred_time_{0.0};
        vector_t pending_viz_pred_state_;

        // Configuration
        std::string robot_name_;
        std::string planning_urdf_variant_; // must be "xacro"
        std::string planning_urdf_path_; // xacro-generated URDF cache path
        std::vector<std::string> joint_names_;
        bool dual_arm_mode_;
        double future_time_offset_; // Future time offset
        /// cached MPC observation state (which is different from real observation on purpose) 
        bool cached_ob_state_;
        double joint_speed_threshold_;
        rclcpp::Time last_execute_time_;
        vector_t cached_last_action_;
        double hardware_latency_;
        std::string traj_record_dir_{"/tmp/traj_record"};
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
        rcl_interfaces::msg::SetParametersResult on_parameter_change(
            const std::vector<rclcpp::Parameter>& parameters);

        std::string urdf_file_;
        std::string task_file_;
    };
}
