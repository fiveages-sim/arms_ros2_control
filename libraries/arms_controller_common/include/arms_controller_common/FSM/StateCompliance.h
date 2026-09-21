//
// Common StateCompliance — first-order hybrid force/position control
//
// Shared by ocs2_arm_controller (and any controller that instantiates it).
// Entered from HOLD via fsm_command=5; returns to HOLD on fsm_command=2.
//
// Per-axis control law (base frame, each cycle):
//   S(i)=1 (force):  v = (F_err + I) / D_force   (first-order admittance)
//   S(i)=0 (pos):    v = K_pos/(1+D) · Δx + I     (softened proportional)
//
//   qdot = J⁺_DLS · v,  q += qdot · dt,  clamp.
//
#pragma once

#include "arms_controller_common/FSM/FSMState.h"
#include "arms_controller_common/FSM/ComplianceSupport.h"
#include "arms_controller_common/utils/GravityCompensation.h"
#include "arms_controller_common/utils/Kinematics.h"

#include <atomic>
#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <arms_ros2_control_msgs/msg/compliance_force_status.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace arms_controller_common
{
    class StateCompliance : public FSMState
    {
    public:
        explicit StateCompliance(CtrlInterfaces& ctrl_interfaces,
                                 std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = nullptr,
                                 const std::shared_ptr<GravityCompensation>& gravity_compensation = nullptr,
                                 const std::shared_ptr<ArmKinematics>& kinematics = nullptr);

        void enter() override;
        void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        void exit() override;
        FSMStateName checkChange() override;

        /** Process tare requests and publish compensated FT telemetry in other FSM states. */
        void publishWrenchTelemetry(const rclcpp::Time& time, const rclcpp::Duration& period);

    private:
        // ── Per-arm runtime state (index 0 = left, 1 = right) ──
        struct ArmSide
        {
            EndEffectorPose cmd_pose;
            EndEffectorPose target;
            bool target_valid{false};
            EndEffectorPose diag_target;
            bool diag_target_valid{false};
            size_t target_updates{0};
            size_t diag_target_updates{0};

            geometry_msgs::msg::Pose pending_pose;
            bool pending_pose_valid{false};

            Eigen::Matrix<double, 6, 1> pos_integral{Eigen::Matrix<double, 6, 1>::Zero()};
            Eigen::Matrix<double, 6, 1> force_integral{Eigen::Matrix<double, 6, 1>::Zero()};
            Eigen::Matrix<double, 6, 1> force_disp{Eigen::Matrix<double, 6, 1>::Zero()};
            Eigen::Vector3d workspace_position_reference{Eigen::Vector3d::Zero()};
            // 力控旋转轴的加性逐轴行程 [rad]：每周期把实测姿态的小角度增量按
            // base 轴分量累加。原实现用 log(R·R_refᵀ) 的投影，有两个致命缺陷：
            // (1) 总转角接近 180° 时该投影会失真甚至恒为 0 → 越界保护完全失效；
            // (2) 旋转不可交换，别的轴转出来的姿态会被算进本轴行程 → 力矩轴的
            //     行程预算被别的轴吃掉，随后被软限位静默冻结、无法再调整。
            Eigen::Vector3d workspace_rot_travel{Eigen::Vector3d::Zero()};
            Eigen::Matrix3d workspace_rot_prev{Eigen::Matrix3d::Identity()};
            bool workspace_rot_prev_valid{false};
            std::array<bool, 6> workspace_axis_active{};
            // 力控轴行程已顶到软限位且仍在往外推（本周期输出被强制为 0）。
            std::array<bool, 6> force_disp_pinned{};
            // 力控轴"导纳自身"的虚拟位移（积分 v_des，有界）——软限位门限用它。
            // 用实测位移做门限时，运动被堵住它不增长 → 门限永不触发 → 位置指令
            // 无界累积（"力矩读数一直涨"）。对自己的状态限幅后，即使环境完全
            // 没有响应，指令也停在 ±xmax。同时它是"响应守卫"的分子/分母之一。
            Eigen::Matrix<double, 6, 1> force_vdisp{Eigen::Matrix<double, 6, 1>::Zero()};
            // 响应守卫：连续处于"请求了但实测没动"的时长 [s] 与确认标志。
            std::array<double, 6> force_stall_time{};
            std::array<bool, 6> force_axis_stalled{};
            // 曲面贴合：力矩驱动的目标姿态偏置（轴角向量，有界）。
            compliance_detail::AlignmentState alignment;
            Eigen::Matrix<double, 6, 1> wrench_filt{Eigen::Matrix<double, 6, 1>::Zero()};
            // Low-pass filtered force-axis velocity (adds virtual inertia / damping
            // to the admittance law, suppresses low-frequency drag oscillation).
            Eigen::Matrix<double, 6, 1> v_des_filt{Eigen::Matrix<double, 6, 1>::Zero()};

            // 上一周期解算的关节速度（位控轴速度阻尼反馈用）：J·qdot_prev
            // 近似任务速度。不用实测位置差分——底层反馈有延迟且刷新/量化
            // 会造成与速度成正比的差分纹波，直接反馈会引入高频抖动。
            Eigen::VectorXd qdot_prev;
            // 上一周期位置轴 v_des（加速度斜坡限幅用）。力控轴恒置 0。
            Eigen::Matrix<double, 6, 1> v_pos_prev{Eigen::Matrix<double, 6, 1>::Zero()};
            // 斜坡后的低通状态（拐角圆滑/S 曲线化，压反向时的加速度突变）。
            Eigen::Matrix<double, 6, 1> v_pos_filt{Eigen::Matrix<double, 6, 1>::Zero()};
            std::array<double, 6> zero_cal_sum{};
            long zero_cal_samples{0};
            std::array<double, 6> wrench_bias{};

            std::array<double, 6> wrench{};
            bool ft_active{false};
            rclcpp::Time ft_stamp{0, 0, RCL_ROS_TIME};
            std::string wrench_frame_id;

            Eigen::VectorXd joint_lower;
            Eigen::VectorXd joint_upper;
            std::vector<double> dyn_param;
            size_t joint_count{0};

            std::string wrench_topic;
            rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub;
            rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_sub;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_stamped_sub;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub;
            rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr filtered_wrench_pub;

            void resetControlState()
            {
                wrench_filt.setZero();
                v_des_filt.setZero();
                force_integral.setZero();
                pos_integral.setZero();
                force_disp.setZero();
                force_vdisp.setZero();
                force_stall_time.fill(0.0);
                force_axis_stalled.fill(false);
                alignment.reset();
                qdot_prev.resize(0);
                v_pos_prev.setZero();
                v_pos_filt.setZero();
                target_valid = false;
                diag_target_valid = false;
                target_updates = 0;
                diag_target_updates = 0;
                pending_pose_valid = false;
            }

            void resetComplianceIo()
            {
                target_sub.reset();
                target_stamped_sub.reset();
                pose_pub.reset();
                target_pub.reset();
            }
        };

        struct WrenchSnapshot
        {
            std::array<std::array<double, 6>, 2> net{};
            std::array<bool, 2> active{false, false};
            std::array<std::string, 2> frame_id;
        };

        ArmSide& arm(bool is_left) { return arms_[is_left ? 0 : 1]; }

        void updateParam();
        bool checkMeasuredWorkspace();
        bool stopForWorkspaceViolation(const std::string& reason);
        void setupWrenchSubscriptions();
        void setupTeleopSubscriptions();
        void setupZeroWrenchService();
        WrenchSnapshot sampleAndPublishWrenches(const rclcpp::Time& time);
        void updateZeroCalibration(const WrenchSnapshot& snapshot, const rclcpp::Time& time,
                                   const rclcpp::Duration& period);
        bool kinematicsAvailable() const;
        /** measured=true: joint state interfaces; else hold_positions_. */
        RobotState makeRobotState(bool measured) const;

        bool computeToolGravityWrenchInSensorFrame(
            int side_index, const std::string& sensor_frame,
            Eigen::Matrix<double, 6, 1>& wrench_sensor) const;
        Eigen::Matrix<double, 6, 1> wrenchToBase(int side_index,
                                                  const std::string& sensor_frame,
                                                  const std::array<double, 6>& wrench_ee) const;

        bool stampedPoseToBase(const geometry_msgs::msg::PoseStamped& msg,
                               geometry_msgs::msg::Pose& pose_base) const;
        bool updateTargetFromPose(bool is_left, const geometry_msgs::msg::Pose& pose_base);

        void stepHybridControl(bool is_left, double dt,
                               const Eigen::Matrix<double, 6, 1>& contact_wrench,
                               bool ft_active);

        // 逆解（笛卡尔速度 v_des → 关节速度 qdot），可选 DLS 或 QP。
        // QP: 加权单层 min 0.5·qdotᵀ(JᵀW²J+λ²I)qdot − (JᵀW²v)ᵀqdot
        //     s.t. |qdot_i|≤vmax，W=diag(w,w,w,1,1,1)（平移软优先级）。
        Eigen::VectorXd solveVelocityQp(const Eigen::MatrixXd& J,
                                        const Eigen::VectorXd& v_des,
                                        double lambda, const Eigen::VectorXd& vmax_up,
                                        const Eigen::VectorXd& vmax_dn) const;

        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        std::shared_ptr<GravityCompensation> gravity_compensation_;
        std::shared_ptr<ArmKinematics> kinematics_;

        std::array<ArmSide, 2> arms_;

        // ── Shared joint command buffer (left then right) ──
        std::vector<double> hold_positions_;
        bool workspace_fault_{false};  // Latched until explicitly entering COMPLIANCE again.

        // ── Teleop / selection ──
        bool teleop_enable_{true};
        std::string teleop_base_frame_{"base_link"};
        std::mutex teleop_mutex_;
        std::vector<double> task_selection_{1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        // 力控→位置控切换检测：取消柔顺轴后需就地重捕获 target（防回弹）。
        std::vector<double> task_selection_prev_;
        bool pending_retarget_{false};

        // ── Position-axis gains ──
        // 这里是所有 compliance_* 参数的唯一定义处：updateParam() 会用下面的
        // 初值在节点上补声明缺失参数，机型 YAML 覆盖同名参数即可生效。
        // 位置环实测延迟 τ≈34ms、>3Hz 谐振峰 → 环路带宽 ωc=K 须 <3Hz：
        // K=20(3.2Hz) 贴近上限；K=30(4.8Hz) 会来回振荡。D softens as K/(1+D).
        std::vector<double> hybrid_pos_stiffness_{20.0, 20.0, 20.0, 10.0, 10.0, 10.0};
        std::vector<double> hybrid_pos_damping_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        // 位控轴速度阻尼（随速度渐隐）：v -= B·fade·J·qdot_prev，|v|>vmax/3
        // 时全额生效（防拖动反转振荡/带速过零冲过头），低速接近目标时线性
        // 淡出（避免指令被提前压塌 → 末端刹停-爬行抖一下），末段交给纯 P。
        double hybrid_pos_vel_damping_{0.0};
        // 位控轴加速度斜坡：限制 v_des 每周期变化量 a=vmax/T。目标阶跃时
        // P 律瞬间饱和（0→vmax 台阶）、到达时指数衰减等效减速度过大 →
        // 底层急加/急减。起步/停止/反向均变为受限斜坡；0 关闭。
        double hybrid_pos_accel_ramp_{0.0};
        // 拐角圆滑时间常数 τ [s]（jerk 上界 ≈ 2·vmax/(T·τ)）。独立于加速
        // 度参数 T，可单独压 jerk；0 = 自动取 T/3（既有行为）。
        double hybrid_pos_jerk_tau_{0.0};
        double hybrid_pos_ki_{0.0};
        double hybrid_pos_ki_max_{0.02};

        // ── Force-axis gains ──
        std::vector<double> hybrid_force_damping_{5000.0, 5000.0, 5000.0, 250.0, 250.0, 250.0};
        // 力控轴上的回弹（位控）刚度 [1/s]：v -= K·实测行程。0 = 纯导纳（现状）。
        // 纯导纳没有恢复力：力/力矩误差一旦不可达（几何增益变号、被环境约束），
        // 轴会一路漂到行程限位，然后被软限位静默冻结——既不能继续调整，也不会
        // 自己回中。旋转力控轴设 0.5~2.0 可稳定在有限偏置并在误差消失后回中。
        std::vector<double> hybrid_force_pos_stiffness_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        // 响应守卫（鲁棒导纳）：请求了运动但实测几乎没动 → 判为"环路没合上"
        // （环境不给增益 / 被刚性约束堵住）→ 保持当前位形、不再加大指令，并让
        // 虚拟位移缓慢回退以自动重试。realize_tol：实测/指令行程比阈值；
        // stall_time：确认时长 [s]；stall_release：虚拟位移回退速率 [1/s]。
        double hybrid_force_realize_tol_{0.25};
        double hybrid_force_stall_time_{0.2};
        double hybrid_force_stall_release_{1.0};
        // ── 曲面贴合：力矩驱动的"目标姿态偏置"外环（compliance_align_enable）──
        // 与力控轴不同：输出不进关节位置积分，而是偏置**目标姿态**，由位控轴
        // 跟踪。位控轴永远有增益（K·Δx），所以环境给增益就收敛（贴合成功）；
        // 不给增益也只是有界地"错"（align_max），不会 windup / 越顶越狠。
        bool align_enabled_{false};
        double align_max_{0.26};        // 偏置上限 [rad]（15°）
        double align_release_{1.0};     // 无接触时偏置的回中速率 [1/s]
        double align_torque_deadband_{0.02};  // Nm, independent of force-axis deadband
        double align_rotational_damping_{20.0};  // Nm s/rad
        bool align_rcc_{true};          // 转动中心取接触点估计（RCC，防嘬入）
        double hybrid_force_ki_{2.0};
        double hybrid_force_ki_max_{10.0};
        double hybrid_force_ki_leak_{0.5};        // integral leakage [1/s], anti-windup during drag
        double hybrid_force_deadband_{0.5};
        std::vector<double> force_setpoint_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double force_feedback_sign_{-1.0};
        // Low-pass on admittance output v_des (virtual inertia). Smaller = more damping.
        double force_vel_lpf_alpha_{0.3};

        // ── Limits ──
        std::vector<double> hybrid_cart_vmax_{0.15, 0.15, 0.15, 0.6, 0.6, 0.6};
        double hybrid_force_xmax_lin_{0.2};
        double hybrid_force_xmax_ang_{0.3};
        double hybrid_force_xmax_margin_ratio_{0.2};  // soft-limit fade margin as fraction of xmax
        double hybrid_joint_vmax_{0.8};
        double hybrid_joint_limit_margin_{0.02};
        // 关节接近软限位时，在完整 6D 任务的精确零空间内向中位回退。
        // 默认关闭；具体机型通过 compliance.yaml 配置。
        double hybrid_joint_limit_avoidance_gain_{0.0};
        // 6/7 轴关节位置耦合限位（说明书图 4-4）：可行域八边形的四条斜边统一为
        // L1 约束 |q6|+|q7| ≤ max（M6 CCS 为 110°，左右臂相同；直边已由 URDF 覆盖）。
        // compliance_wrist_coupling_max 配置，≤0 时关闭。
        double wrist_coupling_max_{0.0};
        double hybrid_dls_lambda_{0.05};
        double hybrid_qp_lambda_{1.0};              // QP 逆解正则化权重（≥DLS λ，保证投影梯度收敛）
        // QP 平移行权重（旋转=1）：平移/旋转冲突时平移软优先（残差权重 w²）。
        // 代码默认 1.0=无权重（保持基线行为），机型配置经 compliance.yaml
        // 设 5.0；过大（>10）在近奇异时反而诱发旋转反号。
        double hybrid_lin_task_weight_{1.0};
        std::string hybrid_inverse_method_{"DLS"};  // 逆解方法: "DLS" 或 "QP"

        // ── Wrench / zero-cal ──
        double wrench_lpf_alpha_{0.15};
        double zero_cal_duration_{10.0};
        double zero_cal_settle_{0.2};
        double zero_cal_still_vel_{0.02};
        // Start automatically on first COMPLIANCE entry, or explicitly via service.
        bool zero_cal_enabled_{false};
        bool zero_cal_pending_{false};
        bool zero_cal_running_{false};
        bool zero_cal_done_{false};
        rclcpp::Time zero_cal_start_;

        // ── 诊断日志（排查"追踪不到位/下垂"：奇异衰减/关节限位/力轴残差/硬件跟踪间隙）──
        bool diag_log_{true};
        double diag_log_period_{1.0};
        std::array<std::chrono::steady_clock::time_point, 2> last_diag_log_{};

        Eigen::VectorXd q_meas_prev_all_;
        double measured_joint_vel_max_{0.0};
        double gravity_accel_{9.81};

        // ── TF / topics ──
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::string gravity_frame_{"world"};
        double ft_timeout_sec_{0.2};

        static constexpr const char* kFtFrame[2]  = {"left_ft_sensor_link", "right_ft_sensor_link"};
        static constexpr const char* kTcpFrame[2] = {"left_tcp", "right_tcp"};
        static constexpr const char* kTargetTopic[2] = {"left_target", "right_target"};
        static constexpr const char* kTargetStampedTopic[2] = {
            "left_target/stamped", "right_target/stamped"};
        static constexpr const char* kCurrentPoseTopic[2] = {
            "left_current_pose", "right_current_pose"};
        static constexpr const char* kCurrentTargetTopic[2] = {
            "left_current_target", "right_current_target"};
        static constexpr const char* kDefaultWrenchTopic[2] = {
            "/left_ft_broadcaster/wrench", "/right_ft_broadcaster/wrench"};
        static constexpr const char* kFilteredWrenchTopic[2] = {
            "/left_ft_broadcaster/wrench_filtered",
            "/right_ft_broadcaster/wrench_filtered"};
        static constexpr const char* kArmName[2] = {"left", "right"};

        rclcpp::Publisher<arms_ros2_control_msgs::msg::ComplianceForceStatus>::SharedPtr
            force_status_pub_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_wrench_service_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr force_setpoint_callback_;
        std::atomic_bool zero_cal_requested_{false};
        rclcpp::Time last_force_status_pub_{0, 0, RCL_ROS_TIME};

        std::mutex wrench_mutex_;
    };
} // namespace arms_controller_common
