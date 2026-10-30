//
// Created for Arms ROS2 Control - WholeBodyVrTeleop
//

#include "arms_target_manager/WholeBodyVrTeleop.h"

#include "arms_target_manager/ArmsTargetManager.h"

#include <arms_ros2_control_msgs/msg/wbc_current_state.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace arms_ros2_control::command
{
    namespace
    {
        // 固定轴 RPY（度）→ 旋转矩阵，与 URDF rpy 一致：R = Rz(yaw)·Ry(pitch)·Rx(roll)。
        Eigen::Matrix3d rpyDegToMatrix(const std::vector<double>& rpy_deg)
        {
            if (rpy_deg.size() != 3)
            {
                return Eigen::Matrix3d::Identity();
            }
            return (Eigen::AngleAxisd(rpy_deg[2] * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(rpy_deg[1] * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(rpy_deg[0] * M_PI / 180.0, Eigen::Vector3d::UnitX()))
                .toRotationMatrix();
        }

        double wrapPi(double angle)
        {
            while (angle > M_PI) angle -= 2.0 * M_PI;
            while (angle < -M_PI) angle += 2.0 * M_PI;
            return angle;
        }

        double yawOfQuaternion(const Eigen::Quaterniond& q)
        {
            const Eigen::Matrix3d m = q.normalized().toRotationMatrix();
            return std::atan2(m(1, 0), m(0, 0));
        }

        Eigen::Matrix2d yawRotation2d(double yaw)
        {
            const double c = std::cos(yaw);
            const double s = std::sin(yaw);
            Eigen::Matrix2d r;
            r << c, -s,
                 s, c;
            return r;
        }

        Eigen::Vector2d stepTowards(const Eigen::Vector2d& current,
                                    const Eigen::Vector2d& target,
                                    double max_step)
        {
            const Eigen::Vector2d delta = target - current;
            const double norm = delta.norm();
            if (norm <= max_step || norm < 1e-12)
            {
                return target;
            }
            return current + delta * (max_step / norm);
        }

        double stepTowards(double current, double target, double max_step)
        {
            const double delta = target - current;
            if (std::abs(delta) <= max_step)
            {
                return target;
            }
            return current + (delta > 0.0 ? max_step : -max_step);
        }
    } // namespace

    WholeBodyVrTeleop::WholeBodyVrTeleop(
        rclcpp::Node::SharedPtr node,
        ArmsTargetManager* target_manager,
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_head_target,
        const std::string& vr_follow_frame,
        const AbsoluteTeleopOptions& options,
        Hooks hooks)
        : node_(std::move(node))
          , target_manager_(target_manager)
          , pub_head_target_(std::move(pub_head_target))
          , vr_follow_frame_(vr_follow_frame)
          , absolute_options_(options)
          , hooks_(std::move(hooks))
    {
        // 接管渐入参数：四条目标链路（左/右臂、胸、头）共用一套速度约束。
        {
            arms_controller_common::PoseRampConfig ramp_config;
            ramp_config.enable = absolute_options_.pose_ramp_enable;
            ramp_config.max_linear_velocity = absolute_options_.pose_ramp_max_linear_velocity;
            ramp_config.max_angular_velocity = absolute_options_.pose_ramp_max_angular_velocity;
            ramp_config.min_duration = absolute_options_.pose_ramp_min_duration_sec;
            ramp_config.max_duration = absolute_options_.pose_ramp_max_duration_sec;
            ramp_config.resume_gap = absolute_options_.pose_ramp_resume_gap_sec;
            left_arm_ramp_.configure(ramp_config);
            right_arm_ramp_.configure(ramp_config);
            body_ramp_.configure(ramp_config);
            head_ramp_.configure(ramp_config);
        }

        // 数据源坐标系 → 目标坐标系的固定旋转偏置（启动时定值，逐帧不重算）
        abs_hand_to_ee_left_ = rpyDegToMatrix(absolute_options_.hand_to_ee_rpy_deg_left);
        abs_hand_to_ee_right_ = rpyDegToMatrix(absolute_options_.hand_to_ee_rpy_deg_right);
        abs_body_src_to_ee_ = rpyDegToMatrix(absolute_options_.body_src_to_ee_rpy_deg);
        abs_head_src_to_ee_ = rpyDegToMatrix(absolute_options_.head_src_to_ee_rpy_deg);

        // 全身骨骼关节订阅器（/teleop/body_<关节名>_pose 为动态话题：头显开启全身追踪后
        // 才会出现发布者；此处常驻订阅，发布者出现/重建时由 DDS 自动重新匹配）
        sub_body_pelvis_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/teleop/body_pelvis_pose", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg) { bodyPelvisCallback(msg); });
        sub_body_spine3_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/teleop/body_spine3_pose", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg) { bodySpine3Callback(msg); });
        sub_body_head_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/teleop/body_head_pose", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg) { bodyHeadCallback(msg); });

        // 骨架手部话题：全身绝对映射的末端数据源（hand / wrist 由 body_ee_source 选择）
        const std::string body_ee_joint =
            (absolute_options_.ee_source == "wrist") ? "wrist" : "hand";
        sub_body_left_hand_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/teleop/body_left_" + body_ee_joint + "_pose", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg) { bodyHandCallback(true, msg); });
        sub_body_right_hand_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/teleop/body_right_" + body_ee_joint + "_pose", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg) { bodyHandCallback(false, msg); });

        // 人体基坐标系由 vr_pose_publisher 统一计算（骨盆地面投影 xy + 双足较低者 z
        // + 双踝 x 轴平均 yaw），本节点只消费结果，不再重复订阅踝/足话题。
        sub_human_footprint_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/teleop/human_footprint_pose", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg) { humanFootprintCallback(msg); });

        // 肩部：肩宽 / 臂长尺度链的起点
        sub_body_left_shoulder_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/teleop/body_left_shoulder_pose", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg) { bodyShoulderCallback(true, msg); });
        sub_body_right_shoulder_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/teleop/body_right_shoulder_pose", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg) { bodyShoulderCallback(false, msg); });

        // 底盘跟踪位置外环：常驻低频定时器，只在 base_tracking_enabled_ 时真正发布。
        // 人体 footprint 帧率低于此频率时靠缓存补齐（updateBaseFollow 内做看门狗）。
        {
            const double rate = std::max(1.0, absolute_options_.base_follow_rate_hz);
            base_follow_timer_ = node_->create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(1.0 / rate)),
                [this]() { updateBaseFollow(); });
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️🦴 WholeBodyVrTeleop created: subscribed to /teleop/body_* "
            "(pelvis, spine3, head, %s hands, shoulders) and /teleop/human_footprint_pose",
            body_ee_joint.c_str());
        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️🦴 全身模式仅支持：FSM(11/12)、夹爪(9/10/13/14)、"
            "右摇杆短按(4)=全部跟踪、左摇杆按键(7)=底盘跟踪、组合键(30)=退出；"
            "其余普通模式组合键在全身模式下不可用");
    }

    // ================= 模式开关 =================

    void WholeBodyVrTeleop::toggleMode()
    {
        toggleWholeBodyTeleop();
    }

    void WholeBodyVrTeleop::toggleWholeBodyTeleop()
    {
        if (whole_body_teleop_active_.load())
        {
            stopWholeBodyTeleop();
            hooks_.resume_normal_mode_state();
            RCLCPP_INFO(
                node_->get_logger(),
                "🕹️🦴 [case 30] 全身遥操作已关闭：恢复普通 VR 遥操作"
                "（普通模式从 STORAGE 开始，镜像保持关闭，需重新按左摇杆按键开启）");
            return;
        }

        if (!hooks_.vr_enabled() ||
            !hooks_.is_full_body_topology() ||
            hooks_.resolved_fsm_state() != 3)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "🔘 [case 30] 忽略全身遥操作开关：要求 VR enabled、FULL_BODY、OCS2");
            return;
        }

        startWholeBodyTeleop();
    }

    void WholeBodyVrTeleop::startWholeBodyTeleop()
    {
        // 进入模式前先清掉普通模式的所有运行态：
        // 停止 UPDATE/STORAGE、强制关闭镜像、退出 CHASSIS 并零速、清 pending/摇杆偏移/
        // 握把边沿/臂暂停，避免这些状态在全身模式下继续生效或退出时残留。
        hooks_.suspend_normal_mode_state();

        whole_body_teleop_active_.store(true);
        // 进入模式只开通“允许跟踪”；绝对位姿映射需再按右摇杆开启
        absolute_tracking_active_.store(false);
        resetAbsoluteTracking();
        base_tracking_enabled_.store(false);
        resetBaseFollow();
        stopHeadTracking();

        // 目标由骨架绝对映射发布，确保 marker 处于连续发布模式
        if (target_manager_ &&
            target_manager_->getCurrentMode() != MarkerState::CONTINUOUS)
        {
            target_manager_->togglePublishMode();
        }

        // 身体模式改为「跟随」、头部模式改为「跟踪」；目标随后由 /teleop/body_* 计算。
        // 全身模式摇杆不参与请求确认，因此直接发命令，不走普通模式的 pending/回中逻辑。
        hooks_.publish_mode_command("BODY_TRACKING");
        hooks_.publish_mode_command("HEAD_TRACKING");

        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️🦴 [case 30] 全身遥操作已开启：身体=跟随（BODY_TRACKING）、"
            "头部=跟踪（HEAD_TRACKING）；右摇杆短按(4)开/关「全部跟踪」，"
            "左摇杆按键(7)开/关「底盘跟踪」；其余普通模式组合键不可用；"
            "再次按左右握把+右摇杆按下退出");
    }

    void WholeBodyVrTeleop::stopWholeBodyTeleop()
    {
        const bool was_active = whole_body_teleop_active_.exchange(false);
        absolute_tracking_active_.store(false);
        resetAbsoluteTracking();
        base_tracking_enabled_.store(false);
        resetBaseFollow();
        stopHeadTracking();

        if (was_active)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "🕹️🦴 [case 30] 全身遥操作已关闭（「全部跟踪」与「底盘跟踪」一并关闭，"
                "WBC 保持当前目标）");
        }
    }

    void WholeBodyVrTeleop::deactivate()
    {
        stopWholeBodyTeleop();
    }

    void WholeBodyVrTeleop::onFsmStateChanged()
    {
        if (!whole_body_teleop_active_.load())
        {
            return;
        }
        const int32_t fsm_state = hooks_.resolved_fsm_state();
        if (fsm_state == 3)  // 3 = OCS2
        {
            return;
        }

        stopWholeBodyTeleop();
        hooks_.resume_normal_mode_state();
        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️🦴 FSM 已离开 OCS2（状态=%d）：全身遥操作自动退出"
            "（「全部跟踪」与「底盘跟踪」一并关闭，WBC 保持当前目标）；"
            "回到 OCS2 后需重新按「左右握把+右摇杆按下」开启",
            fsm_state);
    }

    void WholeBodyVrTeleop::onTopologyChanged(bool is_full_body)
    {
        if (!is_full_body && whole_body_teleop_active_.load())
        {
            stopWholeBodyTeleop();
            hooks_.resume_normal_mode_state();
            RCLCPP_INFO(
                node_->get_logger(),
                "🔘 控制拓扑不再是 FULL_BODY：全身遥操作已自动退出");
        }
    }

    void WholeBodyVrTeleop::onHeadStateChanged(int head_state)
    {
        if (head_state != arms_ros2_control_msgs::msg::WbcCurrentState::HEAD_TRACKING &&
            whole_body_teleop_active_.load())
        {
            stopHeadTracking();
        }
    }

    void WholeBodyVrTeleop::onSharedTargetCacheCleared()
    {
        resetAbsoluteRamps();
        hooks_.reset_stale_catch_up_ramp();
    }

    void WholeBodyVrTeleop::handleButtonEvent(int32_t code)
    {
        switch (code)
        {
            case 4:  // 右摇杆短按：开/关「全部跟踪」
                toggleAbsoluteTracking();
                break;
            case 7:  // 左摇杆按键：开/关「底盘跟踪」
                toggleBaseTracking();
                break;
            default:
                RCLCPP_INFO_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 2000,
                    "🔘 [全身遥操作] 事件 %d 在全身模式下不可用；"
                    "请先按「左右握把+右摇杆按下」退出全身遥操作",
                    code);
                break;
        }
    }

    // ================= 骨架输入 =================

    void WholeBodyVrTeleop::bodyPelvisCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        const Eigen::Vector3d position(msg->position.x, msg->position.y, msg->position.z);
        const Eigen::Quaterniond orientation(
            msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        if (!position.allFinite() || !orientation.coeffs().allFinite() || orientation.norm() < 1e-9)
        {
            has_body_pelvis_pose_ = false;
            return;
        }
        body_pelvis_position_ = position;
        body_pelvis_orientation_ = orientation.normalized();
        has_body_pelvis_pose_ = true;
        body_pelvis_stamp_ = std::chrono::steady_clock::now();
    }

    void WholeBodyVrTeleop::bodySpine3Callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        const Eigen::Vector3d position(msg->position.x, msg->position.y, msg->position.z);
        const Eigen::Quaterniond orientation(
            msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        if (!position.allFinite() || !orientation.coeffs().allFinite() || orientation.norm() < 1e-9)
        {
            has_body_spine3_pose_ = false;
            return;
        }
        body_spine3_position_ = position;
        body_spine3_orientation_ = orientation.normalized();
        has_body_spine3_pose_ = true;
        body_spine3_stamp_ = std::chrono::steady_clock::now();
        // 以脊柱帧作为身体目标的计算触发点（骨盆数据仅缓存）
        publishBodyTarget();
    }

    void WholeBodyVrTeleop::bodyHeadCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        const Eigen::Vector3d position(msg->position.x, msg->position.y, msg->position.z);
        const Eigen::Quaterniond orientation(
            msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        if (!position.allFinite() || !orientation.coeffs().allFinite() || orientation.norm() < 1e-9)
        {
            has_body_head_pose_ = false;
            return;
        }
        body_head_position_ = position;
        body_head_orientation_ = orientation.normalized();
        has_body_head_pose_ = true;
        body_head_stamp_ = std::chrono::steady_clock::now();
        if (whole_body_teleop_active_.load())
        {
            publishHeadTarget();
        }
    }

    void WholeBodyVrTeleop::humanFootprintCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        const Eigen::Vector3d position(msg->position.x, msg->position.y, msg->position.z);
        const Eigen::Quaterniond orientation(
            msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        if (!position.allFinite() || !orientation.coeffs().allFinite() || orientation.norm() < 1e-9)
        {
            has_human_footprint_pose_ = false;
            return;
        }
        human_footprint_position_ = position;
        human_footprint_orientation_ = orientation.normalized();
        human_footprint_stamp_ = std::chrono::steady_clock::now();
        has_human_footprint_pose_ = true;
    }

    void WholeBodyVrTeleop::bodyHandCallback(
        bool is_left, const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        const Eigen::Vector3d position(msg->position.x, msg->position.y, msg->position.z);
        const Eigen::Quaterniond orientation(
            msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        if (!position.allFinite() || !orientation.coeffs().allFinite() || orientation.norm() < 1e-9)
        {
            if (is_left)
            {
                has_body_left_hand_pose_ = false;
            }
            else
            {
                has_body_right_hand_pose_ = false;
            }
            return;
        }

        if (is_left)
        {
            body_left_hand_position_ = position;
            body_left_hand_orientation_ = orientation.normalized();
            has_body_left_hand_pose_ = true;
            body_left_hand_stamp_ = std::chrono::steady_clock::now();
        }
        else
        {
            body_right_hand_position_ = position;
            body_right_hand_orientation_ = orientation.normalized();
            has_body_right_hand_pose_ = true;
            body_right_hand_stamp_ = std::chrono::steady_clock::now();
        }

        // 只重算本次更新的那一侧，避免另一侧被无谓重发（会干扰冻结追赶斜坡判定）
        publishAbsoluteArmTarget(is_left);
    }

    void WholeBodyVrTeleop::bodyShoulderCallback(
        bool is_left, const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        SkeletonPoint& point = is_left ? body_left_shoulder_ : body_right_shoulder_;
        const Eigen::Vector3d position(msg->position.x, msg->position.y, msg->position.z);
        if (!position.allFinite())
        {
            point.valid = false;
            return;
        }
        point.xy = Eigen::Vector2d(position.x(), position.y());
        point.z = position.z();
        point.valid = true;
        point.stamp = std::chrono::steady_clock::now();
    }

    // ================= 绝对位姿映射 =================

    void WholeBodyVrTeleop::toggleAbsoluteTracking()
    {
        if (!hooks_.vr_enabled() ||
            !hooks_.is_full_body_topology() ||
            hooks_.resolved_fsm_state() != 3)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "🔘 [case 4] 忽略全身全部跟踪开关：要求 VR enabled、FULL_BODY、OCS2");
            return;
        }

        if (absolute_tracking_active_.load())
        {
            stopAbsoluteTracking();
        }
        else
        {
            startAbsoluteTracking();
        }
    }

    void WholeBodyVrTeleop::startAbsoluteTracking()
    {
        absolute_tracking_active_.store(true);
        absolute_maps_captured_ = false;
        settle_pending_ = true;
        settle_started_at_ = std::chrono::steady_clock::now();

        abs_left_filter_primed_ = false;
        abs_right_filter_primed_ = false;
        abs_body_filter_primed_ = false;
        abs_head_filter_primed_ = false;

        if (target_manager_ &&
            target_manager_->getCurrentMode() != MarkerState::CONTINUOUS)
        {
            target_manager_->togglePublishMode();
        }

        // 目标来源切换，清掉冻结追赶斜坡，避免切换瞬间跳变
        hooks_.reset_stale_catch_up_ramp();

        // 接管渐入：以机器人当前保持的位姿为锚点，按速度约束过渡到实时骨架期望位姿。
        armAbsoluteRamps();

        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️🦴 [case 4] 全身「全部跟踪」已开启：胸/头/双臂末端改为 /teleop/body_* 绝对位姿映射"
            "（等待骨架稳定后标定）；再次短按右摇杆关闭");
    }

    void WholeBodyVrTeleop::stopAbsoluteTracking()
    {
        absolute_tracking_active_.store(false);
        resetAbsoluteTracking();
        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️🦴 [case 4] 全身「全部跟踪」已关闭：停止下发胸/头/双臂目标"
            "（WBC 保持当前目标；左摇杆按键的底盘跟踪开关独立不受影响）");
    }

    void WholeBodyVrTeleop::resetAbsoluteTracking()
    {
        absolute_maps_captured_ = false;
        settle_pending_ = false;
        abs_left_filter_primed_ = false;
        abs_right_filter_primed_ = false;
        abs_body_filter_primed_ = false;
        abs_head_filter_primed_ = false;
        abs_left_arm_ref_len_ = 0.0;
        abs_right_arm_ref_len_ = 0.0;
        hooks_.reset_stale_catch_up_ramp();
        resetAbsoluteRamps();
        // 交还臂目标独占标志（唯一消费者是手柄/键盘 control_input 的 twist 改写）
        hooks_.set_arm_vr_active(true, false);
        hooks_.set_arm_vr_active(false, false);
    }

    bool WholeBodyVrTeleop::wholeBodyTeleopReady() const
    {
        // 与双臂末端跟随同步：只有「全部跟踪」开着才下发躯干目标
        return target_manager_ &&
               hooks_.is_full_body_topology() &&
               hooks_.vr_enabled() &&
               absolute_tracking_active_.load() &&
               hooks_.resolved_fsm_state() == 3 &&
               target_manager_->getCurrentBodyState() ==
                   arms_ros2_control_msgs::msg::WbcCurrentState::BODY_TRACKING &&
               has_body_pelvis_pose_ && has_body_spine3_pose_;
    }

    bool WholeBodyVrTeleop::headTrackingReady() const
    {
        return target_manager_ &&
               hooks_.is_full_body_topology() &&
               hooks_.vr_enabled() &&
               absolute_tracking_active_.load() &&
               hooks_.resolved_fsm_state() == 3 &&
               target_manager_->getCurrentHeadState() ==
                   arms_ros2_control_msgs::msg::WbcCurrentState::HEAD_TRACKING &&
               has_body_head_pose_ &&
               skeletonJointFresh(body_head_stamp_);
    }

    void WholeBodyVrTeleop::stopHeadTracking()
    {
        head_tracking_active_ = false;
        if (target_manager_)
        {
            target_manager_->setHeadVrActive(false);
        }
    }

    void WholeBodyVrTeleop::publishHeadTarget()
    {
        if (!headTrackingReady())
        {
            stopHeadTracking();
            return;
        }
        if (!absolute_maps_captured_)
        {
            if (settle_pending_ &&
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - settle_started_at_).count() <
                    std::max(0.0, absolute_options_.capture_settle_sec))
            {
                return;
            }
            if (!captureAbsoluteTrackingMaps())
            {
                return;
            }
        }

        // 人体基坐标系 H：把人体行走/整体转身剥离出去（只去驱动底盘）
        Eigen::Isometry3d T_H_world;
        if (!humanFootprintTransform(T_H_world))
        {
            return;
        }

        // 分段尺度变形（只改位置，姿态不变）；头部挂在 spine3 上，随身长尺度一起抬高/降低
        DeformedSkeleton skeleton;
        buildDeformedSkeleton(T_H_world, skeleton);
        const Eigen::Matrix3d R_world_to_H = T_H_world.linear().transpose();
        const Eigen::Vector3d head_in_H = skeleton.head;
        const Eigen::Matrix3d head_rot_in_H =
            R_world_to_H * body_head_orientation_.toRotationMatrix();

        const Eigen::Vector3d position_base = head_in_H;
        const Eigen::Quaterniond orientation_base = dropRoll(
            Eigen::Quaterniond(head_rot_in_H * abs_head_src_to_ee_).normalized());

        const Eigen::Isometry3d T_base_world = currentBaseToWorld();
        const Eigen::Vector3d position =
            T_base_world.linear() * position_base + T_base_world.translation();
        const Eigen::Quaterniond orientation(
            T_base_world.linear() * orientation_base.toRotationMatrix());

        const double alpha = std::clamp(absolute_options_.filter_alpha, 0.0, 1.0);
        if (!abs_head_filter_primed_)
        {
            abs_head_pos_lp_ = position;
            abs_head_ori_lp_ = orientation;
            abs_head_filter_primed_ = true;
        }
        else
        {
            abs_head_pos_lp_ += alpha * (position - abs_head_pos_lp_);
            abs_head_ori_lp_ = abs_head_ori_lp_.slerp(alpha, orientation).normalized();
        }

        // 接管渐入：与双臂/胸部同一套速度约束，收敛后直通
        Eigen::Vector3d ramped_position = abs_head_pos_lp_;
        Eigen::Quaterniond ramped_orientation = abs_head_ori_lp_;
        const bool head_ramping = head_ramp_.update(
            abs_head_pos_lp_, abs_head_ori_lp_, std::chrono::steady_clock::now(),
            ramped_position, ramped_orientation);
        if (head_ramping && head_ramp_.lastPlanWasCapped())
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 3000,
                "🕹️🦴 头部接管渐入超出最长时长 %.1f s，实际峰值速度会高于设定上限",
                absolute_options_.pose_ramp_max_duration_sec);
        }

        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = hooks_.ee_frame_id();
        target.header.stamp = node_->now();
        target.pose.position.x = ramped_position.x();
        target.pose.position.y = ramped_position.y();
        target.pose.position.z = ramped_position.z();
        target.pose.orientation.x = ramped_orientation.x();
        target.pose.orientation.y = ramped_orientation.y();
        target.pose.orientation.z = ramped_orientation.z();
        target.pose.orientation.w = ramped_orientation.w();

        target_manager_->setHeadVrActive(true);
        pub_head_target_->publish(target.pose);
        last_head_target_ = target;
        last_head_target_stamp_ = std::chrono::steady_clock::now();
        head_tracking_active_ = true;
    }

    void WholeBodyVrTeleop::publishBodyTarget()
    {
        if (!whole_body_teleop_active_.load())
        {
            return;
        }
        if (!wholeBodyTeleopReady())
        {
            return;
        }

        if (!absolute_maps_captured_)
        {
            if (settle_pending_ &&
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - settle_started_at_).count() <
                    std::max(0.0, absolute_options_.capture_settle_sec))
            {
                return;
            }
            if (!captureAbsoluteTrackingMaps())
            {
                return;
            }
        }
        // 数据失效：冻结该任务（保持 WBC 当前目标）
        if (!skeletonJointFresh(body_spine3_stamp_) ||
            !skeletonJointFresh(body_pelvis_stamp_))
        {
            return;
        }

        Eigen::Isometry3d T_H_world;
        if (!humanFootprintTransform(T_H_world))
        {
            return;
        }
        // 分段尺度变形（只改位置，姿态不变），然后取骨盆/脊柱点
        DeformedSkeleton skeleton;
        buildDeformedSkeleton(T_H_world, skeleton);
        const Eigen::Matrix3d R_world_to_H = T_H_world.linear().transpose();
        const Eigen::Matrix3d spine_rot_in_H =
            R_world_to_H * body_spine3_orientation_.toRotationMatrix();

        // 位置：沿「躯干轴」（spine3 → pelvis）向下退到解剖对应点。
        // 机器人 body_link4 比人体 spine3 偏下、大致在骨盆高度，所以 λ 默认为 1
        // （即直接取骨盆位置）；λ=0 时取 spine3 位置，中间值为插值。
        // 姿态仍完全由 spine3 决定。
        const double lambda = std::clamp(absolute_options_.body_anchor_lambda, 0.0, 1.0);
        const Eigen::Vector3d anchor_in_H =
            skeleton.spine3 + lambda * (skeleton.pelvis - skeleton.spine3);

        const Eigen::Vector3d position_base = anchor_in_H;
        // 再右乘源→目标固定偏置，最后按可达自由度（ZYX 分解）丢弃 roll
        const Eigen::Quaterniond orientation_base = dropRoll(
            Eigen::Quaterniond(spine_rot_in_H * abs_body_src_to_ee_).normalized());

        const Eigen::Isometry3d T_base_world = currentBaseToWorld();
        const Eigen::Vector3d position =
            T_base_world.linear() * position_base + T_base_world.translation();
        const Eigen::Quaterniond orientation(
            T_base_world.linear() * orientation_base.toRotationMatrix());

        const double alpha = std::clamp(absolute_options_.filter_alpha, 0.0, 1.0);
        if (!abs_body_filter_primed_)
        {
            abs_body_pos_lp_ = position;
            abs_body_ori_lp_ = orientation;
            abs_body_filter_primed_ = true;
        }
        else
        {
            abs_body_pos_lp_ += alpha * (position - abs_body_pos_lp_);
            abs_body_ori_lp_ = abs_body_ori_lp_.slerp(alpha, orientation).normalized();
        }

        // 接管渐入：与双臂同一套速度约束，收敛后直通
        Eigen::Vector3d ramped_position = abs_body_pos_lp_;
        Eigen::Quaterniond ramped_orientation = abs_body_ori_lp_;
        const bool body_ramping = body_ramp_.update(
            abs_body_pos_lp_, abs_body_ori_lp_, std::chrono::steady_clock::now(),
            ramped_position, ramped_orientation);
        if (body_ramping && body_ramp_.lastPlanWasCapped())
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 3000,
                "🕹️🦴 胸部接管渐入超出最长时长 %.1f s，实际峰值速度会高于设定上限",
                absolute_options_.pose_ramp_max_duration_sec);
        }

        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = hooks_.ee_frame_id();
        target.header.stamp = node_->now();
        target.pose.position.x = ramped_position.x();
        target.pose.position.y = ramped_position.y();
        target.pose.position.z = ramped_position.z();
        target.pose.orientation.x = ramped_orientation.x();
        target.pose.orientation.y = ramped_orientation.y();
        target.pose.orientation.z = ramped_orientation.z();
        target.pose.orientation.w = ramped_orientation.w();

        if (!target_manager_->setBodyMarkerPose(target.pose))
        {
            return;
        }
        // 缓存本节点实际下发的胸部目标，供下次接管渐入取锚点
        has_last_body_target_ = true;
        last_body_target_position_ = ramped_position;
        last_body_target_orientation_ = ramped_orientation;
        last_body_target_stamp_ = std::chrono::steady_clock::now();
        // 注：RViz marker 拖拽/菜单不门控（调试通道，允许随时手工干预）
    }

    void WholeBodyVrTeleop::publishAbsoluteArmTarget(bool is_left)
    {
        if (!absolute_tracking_active_.load() || !whole_body_teleop_active_.load())
        {
            return;
        }
        if (!hooks_.vr_enabled() ||
            !hooks_.is_full_body_topology() ||
            hooks_.resolved_fsm_state() != 3)
        {
            return;
        }

        const bool has_hand = is_left ? has_body_left_hand_pose_ : has_body_right_hand_pose_;
        const bool fresh = skeletonJointFresh(
            is_left ? body_left_hand_stamp_ : body_right_hand_stamp_);
        if (!has_hand || !fresh)
        {
            // 数据失效：冻结该臂，不回退到旧目标
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 3000,
                "🕹️🦴 骨架%s手数据%s，冻结该臂目标（检查头显全身追踪是否开启）",
                is_left ? "左" : "右", has_hand ? "超时" : "缺失");
            return;
        }

        if (!absolute_maps_captured_)
        {
            if (settle_pending_ &&
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - settle_started_at_).count() <
                    std::max(0.0, absolute_options_.capture_settle_sec))
            {
                return;  // 仍在稳定窗口内
            }
            if (!captureAbsoluteTrackingMaps())
            {
                return;
            }
        }

        // WBC 单臂禁用/恢复的安全层：全身模式不再依赖手柄 pose，
        // 直接用实测末端位姿重建基准。
        if (hooks_.is_arm_vr_suppressed(is_left))
        {
            if (!hooks_.prepare_arm_vr_input(is_left))
            {
                return;
            }
            (is_left ? left_arm_ramp_ : right_arm_ramp_).reset();
        }

        const Eigen::Vector3d hand_pos =
            is_left ? body_left_hand_position_ : body_right_hand_position_;
        const Eigen::Quaterniond hand_ori =
            is_left ? body_left_hand_orientation_ : body_right_hand_orientation_;
        if (!geometryToleranceOk(is_left, hand_pos))
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "🕹️🦴 骨架%s手几何一致性超差，冻结该臂目标", is_left ? "左" : "右");
            return;
        }

        // 人体基坐标系 H：把「人在走 / 人在整体转身」完全剥离出去（只去驱动底盘）
        Eigen::Isometry3d T_H_world;
        if (!humanFootprintTransform(T_H_world))
        {
            return;
        }
        // 骨架分段尺度（身长/肩宽/臂长）在人体根系里对「段向量」缩放，只改位置不改姿态
        DeformedSkeleton skeleton;
        buildDeformedSkeleton(T_H_world, skeleton);
        const Eigen::Vector3d hand_in_H = is_left ? skeleton.hand_left : skeleton.hand_right;
        const Eigen::Matrix3d R_world_to_H = T_H_world.linear().transpose();
        const Eigen::Matrix3d hand_rot_in_H =
            R_world_to_H * hand_ori.toRotationMatrix();

        // 机器人「底座系」里的目标 = 人体手相对人体根的当前位姿（固定映射，无按下时基准）
        const Eigen::Matrix3d& hand_to_ee =
            is_left ? abs_hand_to_ee_left_ : abs_hand_to_ee_right_;
        const Eigen::Vector3d position_base = hand_in_H;
        const Eigen::Matrix3d rot_base = hand_rot_in_H * hand_to_ee;

        // 底座 → world（优先 TF；无 TF 时用自下发 /cmd_vel 的积分估计）
        const Eigen::Isometry3d T_base_world = currentBaseToWorld();
        const Eigen::Vector3d position =
            T_base_world.linear() * position_base + T_base_world.translation();
        const Eigen::Quaterniond orientation(
            T_base_world.linear() * rot_base);

        // 一阶低通：绝对映射会把骨架抖动直接送进高增益末端约束，必须滤波
        const double alpha = std::clamp(absolute_options_.filter_alpha, 0.0, 1.0);
        bool& primed = is_left ? abs_left_filter_primed_ : abs_right_filter_primed_;
        Eigen::Vector3d& lp_pos = is_left ? abs_left_pos_lp_ : abs_right_pos_lp_;
        Eigen::Quaterniond& lp_ori = is_left ? abs_left_ori_lp_ : abs_right_ori_lp_;
        if (!primed)
        {
            lp_pos = position;
            lp_ori = orientation.normalized();
            primed = true;
        }
        else
        {
            lp_pos += alpha * (position - lp_pos);
            lp_ori = lp_ori.slerp(alpha, orientation.normalized()).normalized();
        }

        // 接管渐入：只在「开启全部跟踪」后的过渡段生效，收敛后 update() 直通。
        Eigen::Vector3d ramped_pos = lp_pos;
        Eigen::Quaterniond ramped_ori = lp_ori;
        arms_controller_common::PoseRamp& arm_ramp =
            is_left ? left_arm_ramp_ : right_arm_ramp_;
        const bool arm_ramping = arm_ramp.update(
            lp_pos, lp_ori, std::chrono::steady_clock::now(), ramped_pos, ramped_ori);
        if (arm_ramping && arm_ramp.lastPlanWasCapped())
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 3000,
                "🕹️🦴 %s臂接管渐入超出最长时长 %.1f s，实际峰值速度会高于设定上限"
                "（调大 absolute_ramp_max_duration 或调小 absolute_ramp_max_*_velocity）",
                is_left ? "左" : "右", absolute_options_.pose_ramp_max_duration_sec);
        }

        // 本模式下该臂目标由全身遥操作独占：
        // 只有真正发布成功后才禁掉 RViz marker 拖拽/菜单与 control_input 改写，
        // 避免坐标系未就绪时错误地夺走普通控制通道。
        if (hooks_.publish_arm_target(is_left, ramped_pos, ramped_ori))
        {
            hooks_.set_arm_vr_active(is_left, true);
        }
    }

    bool WholeBodyVrTeleop::humanFootprintTransform(Eigen::Isometry3d& T_H_world)
    {
        // 人体基坐标系由 vr_pose_publisher 的 HumanFootprintEstimator 统一计算：
        //   位置 xy = 骨盆地面投影，z = 双足较低者（地面），
        //   朝向    = 双踝 x 轴地平面投影平均，俯仰/滚转恒为 0。
        // 低通与原点偏置也在发布端完成，本节点只消费结果。
        if (!has_human_footprint_pose_)
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 3000,
                "🕹️🦴 等待 /teleop/human_footprint_pose"
                "（请确认全身追踪已开启、骨盆/双踝/双足数据齐备）");
            return false;
        }
        if (!skeletonJointFresh(human_footprint_stamp_))
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 3000,
                "🕹️🦴 人体基坐标系数据超时，冻结全身绝对目标（检查全身追踪链路）");
            return false;
        }

        // H 恒为竖直系：上游已做地面投影，这里再强制一次 yaw-only，
        // 避免上游实现变化把 roll/pitch 带进来。
        T_H_world.setIdentity();
        T_H_world.linear() =
            Eigen::AngleAxisd(
                yawOfQuaternion(human_footprint_orientation_),
                Eigen::Vector3d::UnitZ())
                .toRotationMatrix();
        T_H_world.translation() = human_footprint_position_;
        return true;
    }

    void WholeBodyVrTeleop::buildDeformedSkeleton(
        const Eigen::Isometry3d& T_H_world, DeformedSkeleton& out)
    {
        const Eigen::Matrix3d R_world_to_H = T_H_world.linear().transpose();
        const Eigen::Vector3d root = T_H_world.translation();
        auto in_H = [&](const Eigen::Vector3d& p) { return R_world_to_H * (p - root); };
        auto point_in_H = [&](const SkeletonPoint& pt) {
            return Eigen::Vector3d(pt.xy.x(), pt.xy.y(), pt.z);
        };

        // 原始（未缩放）位置，均在人体根系下
        const Eigen::Vector3d pelvis = in_H(body_pelvis_position_);
        const Eigen::Vector3d spine3_raw = in_H(body_spine3_position_);
        const Eigen::Vector3d head_raw = in_H(body_head_position_);
        const Eigen::Vector3d hand_l_raw = in_H(body_left_hand_position_);
        const Eigen::Vector3d hand_r_raw = in_H(body_right_hand_position_);
        const Eigen::Vector3d shoulder_l_raw = in_H(point_in_H(body_left_shoulder_));
        const Eigen::Vector3d shoulder_r_raw = in_H(point_in_H(body_right_shoulder_));

        const double s_body = std::clamp(absolute_options_.body_length_scale, 0.1, 5.0);
        const double s_shoulder = std::clamp(absolute_options_.shoulder_width_scale, 0.1, 5.0);
        const double s_arm = std::clamp(absolute_options_.arm_length_scale, 0.1, 5.0);

        // ① 身长：pelvis → spine3，并以同样尺度推 spine3 → head。
        //    只缩放「段向量」，因此各关节姿态完全不变、人体根不动。
        const bool body_ok = pelvis.allFinite() && spine3_raw.allFinite();
        if (!body_ok)
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 5000,
                "🕹️🦴 骨盆/脊柱数据不可用，身长尺度暂按 1.0");
        }
        out.pelvis = pelvis;
        out.spine3 = body_ok ? (pelvis + s_body * (spine3_raw - pelvis)) : spine3_raw;
        out.head = (body_ok && head_raw.allFinite())
                       ? (out.spine3 + s_body * (head_raw - spine3_raw))
                       : head_raw;

        // ② 肩宽：spine3 → shoulder，绕**变形后**的 spine3 缩放
        const auto scale_shoulder = [&](const Eigen::Vector3d& shoulder,
                                        const char* side) -> Eigen::Vector3d {
            if (!body_ok || !shoulder.allFinite())
            {
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 5000,
                    "🕹️🦴 %s肩数据不可用，肩宽尺度暂按 1.0", side);
                return shoulder;
            }
            return Eigen::Vector3d(out.spine3 + s_shoulder * (shoulder - spine3_raw));
        };
        out.shoulder_left = scale_shoulder(shoulder_l_raw, "左");
        out.shoulder_right = scale_shoulder(shoulder_r_raw, "右");

        // ③ 臂长：把「原始肩→原始的段向量」缩放 s_arm 后，挂到**变形后**的肩上。
        //    注意不能用变形后的肩去作差，否则肩的变形量会污染手臂向量。
        const auto scale_arm = [&](const Eigen::Vector3d& shoulder_deformed,
                                   const Eigen::Vector3d& shoulder_raw,
                                   const Eigen::Vector3d& hand,
                                   const char* side) -> Eigen::Vector3d {
            if (!shoulder_raw.allFinite() || !hand.allFinite())
            {
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 5000,
                    "🕹️🦴 %s侧肩/末端数据不可用，臂长尺度暂按 1.0", side);
                return hand;
            }
            return Eigen::Vector3d(shoulder_deformed + s_arm * (hand - shoulder_raw));
        };
        out.hand_left = scale_arm(out.shoulder_left, shoulder_l_raw, hand_l_raw, "左");
        out.hand_right = scale_arm(out.shoulder_right, shoulder_r_raw, hand_r_raw, "右");
    }

    bool WholeBodyVrTeleop::skeletonJointFresh(
        const std::chrono::steady_clock::time_point& stamp) const
    {
        if (stamp == std::chrono::steady_clock::time_point{})
        {
            return false;
        }
        const double age =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - stamp).count();
        return age <= std::max(0.0, absolute_options_.skeleton_timeout_sec);
    }

    Eigen::Quaterniond WholeBodyVrTeleop::dropRoll(const Eigen::Quaterniond& q)
    {
        // ZYX 分解 R = Rz(yaw)·Ry(pitch)·Rx(roll)，丢弃 roll。
        // W2 腰部为 3×pitch + 1×yaw、无 roll；头部仅 yaw/pitch，超出的分量不可达。
        const Eigen::Matrix3d m = q.normalized().toRotationMatrix();
        const double yaw = std::atan2(m(1, 0), m(0, 0));
        const double pitch =
            std::atan2(-m(2, 0), std::hypot(m(2, 1), m(2, 2)));
        return Eigen::Quaterniond(
                   Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                   Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()))
            .normalized();
    }

    bool WholeBodyVrTeleop::geometryToleranceOk(
        bool is_left, const Eigen::Vector3d& hand_position) const
    {
        const double tolerance = absolute_options_.geometry_tolerance;
        if (tolerance <= 0.0 || !has_body_spine3_pose_ || !settle_pending_)
        {
            return true;
        }
        const double ref_len =
            is_left ? abs_left_arm_ref_len_ : abs_right_arm_ref_len_;
        if (ref_len <= 1e-6)
        {
            return true;
        }
        const double current_len =
            (hand_position - body_spine3_position_).norm();
        return std::abs(current_len - ref_len) <= tolerance * ref_len;
    }

    bool WholeBodyVrTeleop::captureAbsoluteTrackingMaps()
    {
        if (!hooks_.ee_frame_ready())
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "🕹️🦴 全身绝对跟踪等待末端参考坐标系（left_current_pose）初始化");
            return false;
        }

        // 目标不再以机器人当前姿态为基准，因此标定只需要人体骨架数据
        std::vector<const char*> missing;
        if (!has_body_pelvis_pose_) missing.push_back("/teleop/body_pelvis_pose");
        if (!has_body_spine3_pose_) missing.push_back("/teleop/body_spine3_pose");
        if (!has_body_head_pose_) missing.push_back("/teleop/body_head_pose");
        if (!has_body_left_hand_pose_) missing.push_back("左侧骨架手部话题");
        if (!has_body_right_hand_pose_) missing.push_back("右侧骨架手部话题");
        // 人体基坐标系必须已由上游发布（骨盆 xy / 双踝朝向 / 双足地面 z），
        // 否则所有目标位置都无意义。
        if (!has_human_footprint_pose_)
            missing.push_back("/teleop/human_footprint_pose（人体基坐标系）");
        else if (!skeletonJointFresh(human_footprint_stamp_))
            missing.push_back("/teleop/human_footprint_pose（已超时）");
        // 肩部是「肩宽 / 臂长」两条尺度链的起点
        if (!body_left_shoulder_.valid) missing.push_back("/teleop/body_left_shoulder_pose（肩宽/臂长尺度）");
        if (!body_right_shoulder_.valid) missing.push_back("/teleop/body_right_shoulder_pose（肩宽/臂长尺度）");
        if (!missing.empty())
        {
            std::string joined;
            for (const auto* name : missing)
            {
                if (!joined.empty()) joined += ", ";
                joined += name;
            }
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "🕹️🦴 全身绝对跟踪等待骨架数据：%s", joined.c_str());
            return false;
        }

        // 人体参考臂长（手–脊柱距离），供几何一致性过滤使用
        abs_left_arm_ref_len_ =
            (body_left_hand_position_ - body_spine3_position_).norm();
        abs_right_arm_ref_len_ =
            (body_right_hand_position_ - body_spine3_position_).norm();

        settle_pending_ = false;
        absolute_maps_captured_ = true;
        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️🦴 全身绝对跟踪已生效：人体基坐标系 H 来自 /teleop/human_footprint_pose"
            "（xy 取骨盆地面投影 / 朝向取两踝 x 轴地平面投影均值 / z 取两足较低者，"
            "俯仰滚转恒为 0；低通与原点偏置在 vr_pose_publisher 侧配置），"
            "发布系=%s，尺度[身长/肩宽/臂长]=%.2f/%.2f/%.2f，末端源=body_%s，胸部锚点 λ=%.2f；"
            "人体参考臂长 L/R=%.3f/%.3f m。"
            "映射按「人体系」锚定：人体各量先表达在人体根自身坐标系里，故人体朝向 ≡ 机器人机头朝向"
            "（无需人工偏航常数）；人体各部位相对人体根的当前位姿即为机器人对应部位相对底座的目标",
            hooks_.ee_frame_id().c_str(),
            absolute_options_.body_length_scale,
            absolute_options_.shoulder_width_scale,
            absolute_options_.arm_length_scale,
            absolute_options_.ee_source.c_str(),
            absolute_options_.body_anchor_lambda,
            abs_left_arm_ref_len_, abs_right_arm_ref_len_);
        return true;
    }

    void WholeBodyVrTeleop::armAbsoluteRamps()
    {
        if (!absolute_options_.pose_ramp_enable)
        {
            resetAbsoluteRamps();
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        // 锚点新鲜度：超过该时长的「最后下发值」可能已被 RViz marker / control_input
        // 改写过，不再代表控制器当前保持的目标，此时退回实测位姿更安全。
        constexpr double kAnchorMaxAgeSec = 1.0;
        const auto isFresh = [&now](const std::chrono::steady_clock::time_point& stamp)
        {
            return stamp != std::chrono::steady_clock::time_point{} &&
                   std::chrono::duration<double>(now - stamp).count() <= kAnchorMaxAgeSec;
        };

        // ---- 双臂：优先最后下发的 command target，其次实测末端位姿 ----
        const auto arm_anchor = [&](bool is_left) -> std::optional<LastArmTarget>
        {
            const LastArmTarget last = hooks_.last_published_target(is_left);
            if (last.valid && isFresh(last.stamp))
            {
                return last;
            }
            Eigen::Vector3d position;
            Eigen::Quaterniond orientation;
            if (hooks_.robot_current_pose(is_left, position, orientation))
            {
                LastArmTarget measured;
                measured.valid = true;
                measured.position = position;
                measured.orientation = orientation;
                measured.stamp = now;
                return measured;
            }
            return std::nullopt;
        };

        const auto left_anchor = arm_anchor(true);
        if (left_anchor)
        {
            left_arm_ramp_.armFrom(left_anchor->position, left_anchor->orientation);
        }
        else
        {
            left_arm_ramp_.reset();
        }

        const auto right_anchor = arm_anchor(false);
        if (right_anchor)
        {
            right_arm_ramp_.armFrom(right_anchor->position, right_anchor->orientation);
        }
        else
        {
            right_arm_ramp_.reset();
        }

        // ---- 胸部：本节点最后一次实际下发的值（与发布系 ee_frame_id_ 一致）----
        if (has_last_body_target_ && isFresh(last_body_target_stamp_))
        {
            body_ramp_.armFrom(
                last_body_target_position_, last_body_target_orientation_);
        }
        else
        {
            body_ramp_.reset();
        }

        // ---- 头部：最后一次下发的目标（可能来自普通模式的另一个 frame），
        //      先转到发布系；不可用则退回实测头部位姿 ----
        const std::string target_frame = hooks_.ee_frame_id();
        bool head_armed = false;
        const auto arm_head_from = [&](const Eigen::Vector3d& position,
                                       const Eigen::Quaterniond& orientation,
                                       const std::string& frame_id)
        {
            if (frame_id == target_frame)
            {
                head_ramp_.armFrom(position, orientation);
                head_armed = true;
                return;
            }
            Eigen::Vector3d transformed_position;
            Eigen::Quaterniond transformed_orientation;
            if (hooks_.transform_pose(
                    "head", frame_id, target_frame, position, orientation,
                    transformed_position, transformed_orientation))
            {
                head_ramp_.armFrom(transformed_position, transformed_orientation);
                head_armed = true;
            }
        };

        if (last_head_target_ && isFresh(last_head_target_stamp_))
        {
            const auto& p = last_head_target_->pose.position;
            const auto& q = last_head_target_->pose.orientation;
            arm_head_from(
                Eigen::Vector3d(p.x, p.y, p.z),
                Eigen::Quaterniond(q.w, q.x, q.y, q.z),
                last_head_target_->header.frame_id);
        }
        geometry_msgs::msg::PoseStamped robot_head;
        if (!head_armed && hooks_.robot_head_pose(robot_head))
        {
            const auto& p = robot_head.pose.position;
            const auto& q = robot_head.pose.orientation;
            arm_head_from(
                Eigen::Vector3d(p.x, p.y, p.z),
                Eigen::Quaterniond(q.w, q.x, q.y, q.z),
                robot_head.header.frame_id);
        }
        if (!head_armed)
        {
            head_ramp_.reset();
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️🦴 [case 4] 接管渐入已布置：lin=%.2f m/s, ang=%.2f rad/s, "
            "臂(L/R)=%s/%s, 胸=%s, 头=%s（未布置的任务直接跟踪）",
            absolute_options_.pose_ramp_max_linear_velocity,
            absolute_options_.pose_ramp_max_angular_velocity,
            left_arm_ramp_.ramping() ? "on" : "off",
            right_arm_ramp_.ramping() ? "on" : "off",
            body_ramp_.ramping() ? "on" : "off",
            head_ramp_.ramping() ? "on" : "off");
    }

    void WholeBodyVrTeleop::resetAbsoluteRamps()
    {
        left_arm_ramp_.reset();
        right_arm_ramp_.reset();
        body_ramp_.reset();
        head_ramp_.reset();
    }

    Eigen::Isometry3d WholeBodyVrTeleop::currentBaseToWorld()
    {
        const std::string target_frame = hooks_.ee_frame_id();
        Eigen::Vector3d position;
        Eigen::Quaterniond rotation;
        if (hooks_.ee_frame_ready() && !vr_follow_frame_.empty() && !target_frame.empty() &&
            hooks_.transform_pose(
                "chassis", vr_follow_frame_, target_frame,
                Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                position, rotation))
        {
            Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
            transform.linear() = rotation.toRotationMatrix();
            transform.translation() = position;
            return transform;
        }

        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "🕹️🚗 无 %s → %s TF，改用「自下发 /cmd_vel 积分」估计底座位姿；"
            "打滑、限速饱和或被外部（导航/手柄）驱动时会有偏差",
            vr_follow_frame_.c_str(), target_frame.c_str());

        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.linear() =
            Eigen::AngleAxisd(base_est_yaw_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        transform.translation() =
            Eigen::Vector3d(base_est_pos_.x(), base_est_pos_.y(), 0.0);
        return transform;
    }

    void WholeBodyVrTeleop::integrateBaseOdometry(const geometry_msgs::msg::Twist& velocity)
    {
        const auto now = std::chrono::steady_clock::now();
        if (base_odom_valid_)
        {
            const double dt = std::clamp(
                std::chrono::duration<double>(now - base_odom_last_tick_).count(),
                0.0, 0.1);
            base_est_yaw_ += velocity.angular.z * dt;
            const Eigen::Vector2d v_body(velocity.linear.x, velocity.linear.y);
            if (v_body.squaredNorm() > 0.0)
            {
                base_est_pos_ += yawRotation2d(base_est_yaw_) * v_body * dt;
            }
        }
        base_odom_valid_ = true;
        base_odom_last_tick_ = now;
    }

    // ================= 底盘跟踪（位置外环 → /cmd_vel） =================

    void WholeBodyVrTeleop::toggleBaseTracking()
    {
        if (!absolute_options_.base_follow_enable)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "🔘 [case 7] 忽略底盘跟踪开关：base_follow_enable=false");
            return;
        }
        if (!hooks_.vr_enabled() ||
            !hooks_.is_full_body_topology() ||
            hooks_.resolved_fsm_state() != 3)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "🔘 [case 7] 忽略底盘跟踪开关：要求 VR enabled、FULL_BODY、OCS2");
            return;
        }

        if (base_tracking_enabled_.load())
        {
            base_tracking_enabled_.store(false);
            resetBaseFollow();
            RCLCPP_INFO(
                node_->get_logger(),
                "🔘 [case 7] 底盘跟踪已关闭（已发一次零速）");
        }
        else
        {
            base_tracking_enabled_.store(true);
            RCLCPP_INFO(
                node_->get_logger(),
                "🔘 [case 7] 底盘跟踪已开启：人体 footprint → 位置外环 → /cmd_vel"
                "（BASE_UNLOCKED 时冻结跟随并让路给 WBC）");
        }
    }

    bool WholeBodyVrTeleop::baseFollowReady() const
    {
        if (!absolute_options_.base_follow_enable || !base_tracking_enabled_.load())
        {
            return false;
        }
        if (!hooks_.vr_enabled() || !hooks_.is_full_body_topology())
        {
            return false;
        }
        if (hooks_.resolved_fsm_state() != 3)
        {
            return false;
        }
        if (!has_human_footprint_pose_ ||
            human_footprint_stamp_ == std::chrono::steady_clock::time_point{})
        {
            return false;
        }
        const double age =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - human_footprint_stamp_).count();
        return age <= std::max(0.0, absolute_options_.base_follow_data_timeout);
    }

    void WholeBodyVrTeleop::captureBaseFollowBase()
    {
        base_follow_footprint_ref_ = human_footprint_position_;
        base_follow_yaw_ref_ = yawOfQuaternion(human_footprint_orientation_);
        base_follow_yaw_offset_ = 0.0;
        base_follow_p_ref_.setZero();
        base_follow_v_cmd_.setZero();
        base_follow_w_cmd_ = 0.0;
        base_follow_v_out_.setZero();
        base_follow_w_out_ = 0.0;
        base_follow_last_tick_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️🚗 底盘跟踪基准已建立（人体 footprint yaw=%.1f°）",
            base_follow_yaw_ref_ * 180.0 / M_PI);
    }

    void WholeBodyVrTeleop::resetBaseFollow()
    {
        const bool was_active = base_follow_active_;
        base_follow_active_ = false;
        base_follow_p_ref_.setZero();
        base_follow_v_cmd_.setZero();
        base_follow_w_cmd_ = 0.0;
        base_follow_v_out_.setZero();
        base_follow_w_out_ = 0.0;
        base_follow_yaw_offset_ = 0.0;
        if (was_active)
        {
            // 只在真正跟过的时候补一次零速，避免无关场景向 /cmd_vel 发零
            hooks_.publish_chassis_velocity(geometry_msgs::msg::Twist());
        }
    }

    void WholeBodyVrTeleop::updateBaseFollow()
    {
        // FSM 可能由 RViz/手柄/控制器异常等非 /fsm_command 来源切换；
        // 外环以 50Hz 运行，这里做一次兜底退出。
        if (whole_body_teleop_active_.load() && hooks_.resolved_fsm_state() != 3)
        {
            onFsmStateChanged();
            return;
        }

        if (!base_tracking_enabled_.load())
        {
            return;
        }

        if (!baseFollowReady())
        {
            if (base_follow_active_)
            {
                resetBaseFollow();
            }
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        if (!base_follow_active_)
        {
            captureBaseFollowBase();
            base_follow_active_ = true;
            return;
        }

        double dt = std::chrono::duration<double>(now - base_follow_last_tick_).count();
        base_follow_last_tick_ = now;
        dt = std::clamp(dt, 1e-3, 0.1);

        // yaw 补偿：用自己下发的 ω 积分出机器人自身的转向量，
        // 使 VR「人体前方」始终对应机头前方（不依赖外部 TF）
        base_follow_yaw_offset_ += base_follow_w_out_ * dt;

        // 人体 footprint 位移：转到「接管时刻机器人本体系」F（人体 yaw 在接管时与机头对齐）
        const Eigen::Vector3d footprint_delta =
            human_footprint_position_ - base_follow_footprint_ref_;
        const Eigen::Vector2d delta_F =
            yawRotation2d(-base_follow_yaw_ref_) * footprint_delta.head<2>();

        // 软参考「跑步机」：p_ref 以 τ 追赶人体位置，避免人走出追踪区
        const double tau = std::max(1e-3, absolute_options_.base_follow_tau);
        base_follow_p_ref_ += (delta_F - base_follow_p_ref_) * std::min(1.0, dt / tau);
        const Eigen::Vector2d error_F = delta_F - base_follow_p_ref_;

        // 水平速度：死区内为零；稳态增益 ≈ Kp·τ
        const double deadband = std::max(0.0, absolute_options_.base_follow_deadband);
        Eigen::Vector2d v_des_F = Eigen::Vector2d::Zero();
        if (error_F.norm() > deadband)
        {
            v_des_F = absolute_options_.base_follow_kp * error_F;
            const double vmax = std::max(0.0, absolute_options_.base_vmax);
            if (v_des_F.norm() > vmax)
            {
                v_des_F *= vmax / v_des_F.norm();
            }
        }

        // 偏航：绝对跟踪（人体转身多少，机头转多少），死区 + 限幅
        const double delta_yaw =
            wrapPi(yawOfQuaternion(human_footprint_orientation_) - base_follow_yaw_ref_);
        const double error_yaw = wrapPi(delta_yaw - base_follow_yaw_offset_);
        double w_des = 0.0;
        if (std::abs(error_yaw) > std::max(0.0, absolute_options_.base_follow_yaw_deadband))
        {
            w_des = absolute_options_.base_follow_kpsi * error_yaw;
            const double wmax = std::max(0.0, absolute_options_.base_wmax);
            w_des = std::clamp(w_des, -wmax, wmax);
        }

        // 一阶低通 + 速率限幅
        const double cmd_tau = std::max(1e-3, absolute_options_.base_cmd_tau);
        const double lp_alpha = std::min(1.0, dt / cmd_tau);
        base_follow_v_cmd_ += lp_alpha * (v_des_F - base_follow_v_cmd_);
        base_follow_w_cmd_ += lp_alpha * (w_des - base_follow_w_cmd_);
        base_follow_v_out_ = stepTowards(
            base_follow_v_out_, base_follow_v_cmd_,
            std::max(0.0, absolute_options_.base_acc_max) * dt);
        base_follow_w_out_ = stepTowards(
            base_follow_w_out_, base_follow_w_cmd_,
            std::max(0.0, absolute_options_.base_alpha_max) * dt);

        // F 系速度 → 机体系速度
        const Eigen::Vector2d v_B =
            yawRotation2d(-base_follow_yaw_offset_) * base_follow_v_out_;

        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = v_B.x();
        cmd_vel.linear.y = v_B.y();
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = base_follow_w_out_;

        if (hooks_.wbc_owns_chassis_velocity())
        {
            // 让路给 WBC：不发指令，但把软参考与人体同步，回到 BASE_LOCK 时不产生冲击
            base_follow_p_ref_ = delta_F;
            base_follow_v_cmd_.setZero();
            base_follow_v_out_.setZero();
            base_follow_w_cmd_ = 0.0;
            base_follow_w_out_ = 0.0;
            base_follow_yaw_offset_ = delta_yaw;
            RCLCPP_DEBUG_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "🕹️🚗 底盘跟踪让路给 WBC（BASE_UNLOCKED），保持软参考同步");
            return;
        }

        hooks_.publish_chassis_velocity(cmd_vel);
        // 无 TF 时用这条积分当作底座位姿估计（绝对映射的手臂目标要转到 world 系）
        integrateBaseOdometry(cmd_vel);
    }

} // namespace arms_ros2_control::command
