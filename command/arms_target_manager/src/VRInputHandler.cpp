//
// Created for Arms ROS2 Control - VRInputHandler
//

#include "arms_target_manager/VRInputHandler.h"
#include "arms_target_manager/ArmsTargetManager.h"
#include "arms_controller_common/utils/FSMStateTransitionValidator.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <arms_ros2_control_msgs/msg/wbc_current_state.hpp>
#include <arms_ros2_control_msgs/msg/wbc_capability.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <nav_msgs/msg/path.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace arms_ros2_control::command
{
    // 静态常量定义
    const double VRInputHandler::POSITION_THRESHOLD = 0.01; // 1cm threshold for position changes
    const double VRInputHandler::ORIENTATION_THRESHOLD = 0.005; // threshold for orientation changes (quaternion angle)
    const int VRInputHandler::STALE_MIN_FROZEN_FRAMES = 2; // 连续 >=2 帧逐位相同即判定上游冻结

    namespace
    {
        // 固定轴 RPY（度）→ 旋转矩阵，与 URDF rpy 一致：R = Rz(yaw)·Ry(pitch)·Rx(roll)。
        // 用于「数据源坐标系 → 机器人目标坐标系」的固定旋转偏置（启动时求一次）。
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

    } // namespace

    VRInputHandler::VRInputHandler(
        rclcpp::Node::SharedPtr node,
        ArmsTargetManager* targetManager,
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_left_target,
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_right_target,
        const std::vector<std::string>& handControllers,
        double vr_thumbstick_linear_scale,
        double vr_thumbstick_angular_scale,
        double vr_pose_scale,
        const std::string& reference_link,
        const std::string& vr_follow_frame,
        const AbsoluteTeleopOptions& absolute_options)
        : node_(std::move(node))
          , target_manager_(targetManager)
          , pub_left_target_(std::move(pub_left_target))
          , pub_right_target_(std::move(pub_right_target))
          , enabled_(false)
          , is_update_mode_(false)
          , mirror_mode_(false)
          , left_arm_paused_(false)
          , right_arm_paused_(false)
          , left_grip_mode_(false)
          , right_grip_mode_(false)
          , chassis_mode_(false)
          , current_fsm_state_(2)  // 默认HOLD状态
          , hand_controllers_(handControllers)
          , left_gripper_open_(false)
          , right_gripper_open_(false)
          , left_gripper_percent_mode_(true)
          , right_gripper_percent_mode_(true)
          , left_gripper_percent_armed_(false)
          , right_gripper_percent_armed_(false)
          , left_gripper_percent_saw_release_(false)
          , right_gripper_percent_saw_release_(false)
          , left_gripper_percent_hold_(1.0)
          , right_gripper_percent_hold_(1.0)
          , vr_thumbstick_linear_scale_(vr_thumbstick_linear_scale)
          , vr_thumbstick_angular_scale_(vr_thumbstick_angular_scale)
          , vr_pose_scale_(vr_pose_scale)
          , reference_link_(reference_link)
          , vr_follow_frame_(vr_follow_frame)
    {
        absolute_options_ = absolute_options;

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

        // 创建 controller topology 检测 client 和启动期重试 timer
        list_controllers_client_ =
            node_->create_client<ListControllers>(
                "/controller_manager/list_controllers");

        control_topology_timer_ = node_->create_wall_timer(
            std::chrono::seconds(1),
            [this]() { detectControlTopology(); });

        // 底盘跟踪位置外环：常驻低频定时器，只在 base_tracking_enabled_ 时真正发布。
        // 骨盆骨架帧率低于此频率时靠缓存补齐（updateBaseFollow 内做看门狗）。
        {
            const double rate = std::max(1.0, absolute_options_.base_follow_rate_hz);
            base_follow_timer_ = node_->create_wall_timer(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(1.0 / rate)),
                [this]() { updateBaseFollow(); });
        }

        // 初始化 TF 组件（用于在 reference_link 与末端 frame 之间进行坐标变换）
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        RCLCPP_INFO(
            node_->get_logger(),
            "VR stale-input catch-up ramp recovery is enabled (frozen >= %d identical frames).",
            STALE_MIN_FROZEN_FRAMES);
        // 检测左右控制器名称
        detectGripperControllers(hand_controllers_);
        
        // 创建夹爪状态订阅器（用于同步夹爪状态）
        if (!left_gripper_controller_name_.empty())
        {
            std::string left_topic = "/" + left_gripper_controller_name_ + "/target_command";
            auto leftGripperStateCallback = [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                this->leftGripperStateCallback(msg);
            };
            sub_left_gripper_state_ = node_->create_subscription<std_msgs::msg::Int32>(
                left_topic, 10, leftGripperStateCallback);
            RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Subscribed to left gripper state: %s", left_topic.c_str());
        }
        
        if (!right_gripper_controller_name_.empty())
        {
            std::string right_topic = "/" + right_gripper_controller_name_ + "/target_command";
            auto rightGripperStateCallback = [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                this->rightGripperStateCallback(msg);
            };
            sub_right_gripper_state_ = node_->create_subscription<std_msgs::msg::Int32>(
                right_topic, 10, rightGripperStateCallback);
            RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Subscribed to right gripper state: %s", right_topic.c_str());
        }

        // 创建VR订阅器
        auto vrLeftCallback = [this](const geometry_msgs::msg::Pose::SharedPtr msg)
        {
            this->vrLeftCallback(msg);
        };
        sub_left_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/teleop/left_ee_pose", 10, vrLeftCallback);

        auto vrRightCallback = [this](const geometry_msgs::msg::Pose::SharedPtr msg)
        {
            this->vrRightCallback(msg);
        };
        sub_right_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/teleop/right_ee_pose", 10, vrRightCallback);

        // 头显位姿订阅器
        auto vrHeadCallback = [this](const geometry_msgs::msg::Pose::SharedPtr msg)
        {
            this->vrHeadCallback(msg);
        };
        sub_head_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/teleop/head_pose", 10, vrHeadCallback);

        sub_robot_head_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "head_current_pose", 10,
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { robotHeadPoseCallback(msg); });
        pub_head_target_ = node_->create_publisher<geometry_msgs::msg::Pose>("head_target", 10);

        // 全身骨骼关节订阅器（/teleop/body_<关节名>_pose 为动态话题：头显开启全身追踪后
        // 才会出现发布者；此处常驻订阅，发布者出现/重建时由 DDS 自动重新匹配）
        sub_body_pelvis_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/teleop/body_pelvis_pose", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg) { this->bodyPelvisCallback(msg); });
        sub_body_spine3_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/teleop/body_spine3_pose", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg) { this->bodySpine3Callback(msg); });
        sub_body_head_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/teleop/body_head_pose", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg) { this->bodyHeadCallback(msg); });
        // 骨架手部话题：全身绝对映射的末端数据源（hand / wrist 由 body_ee_source 选择）
        const std::string body_ee_joint =
            (absolute_options_.ee_source == "wrist") ? "wrist" : "hand";
        sub_body_left_hand_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/teleop/body_left_" + body_ee_joint + "_pose", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg) { this->bodyHandCallback(true, msg); });
        sub_body_right_hand_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/teleop/body_right_" + body_ee_joint + "_pose", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg) { this->bodyHandCallback(false, msg); });
        // 人体基坐标系由 vr_pose_publisher 统一计算（骨盆地面投影 xy + 双足较低者 z
        // + 双踝 x 轴平均 yaw），本节点只消费结果，不再重复订阅踝/足话题。
        sub_human_footprint_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/teleop/human_footprint_pose", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg) { this->humanFootprintCallback(msg); });
        // 肩部：肩宽 / 臂长尺度链的起点
        sub_body_left_shoulder_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/teleop/body_left_shoulder_pose", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg) { this->bodyShoulderCallback(true, msg); });
        sub_body_right_shoulder_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/teleop/body_right_shoulder_pose", 10,
            [this](const geometry_msgs::msg::Pose::SharedPtr msg) { this->bodyShoulderCallback(false, msg); });
        RCLCPP_INFO(node_->get_logger(),
                    "🕹️🦴 Subscribed to full-body absolute tracking topics: "
                    "/teleop/body_left_%s_pose, /teleop/body_right_%s_pose, "
                    "/teleop/body_pelvis_pose, /teleop/body_spine3_pose, /teleop/body_head_pose, "
                    "/teleop/human_footprint_pose（人体基坐标系）, "
                    "/teleop/body_{left,right}_shoulder_pose（肩宽/臂长尺度）",
                    body_ee_joint.c_str(), body_ee_joint.c_str());

        // 创建按钮事件订阅器（Int32类型）
        auto controllerStateCallback = [this](const std_msgs::msg::Int32::SharedPtr msg)
        {
            this->processButtonEvent(msg);
        };
        sub_controller_state_ = node_->create_subscription<std_msgs::msg::Int32>(
            "/teleop/controller_state", 10, controllerStateCallback);
        
        // 创建摇杆轴值订阅器（合并订阅左右摇杆，使用 Twist 消息）
        // linear.x/y 表示左摇杆，angular.x/y 表示右摇杆
        auto thumbstickAxesCallback = [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            this->thumbstickAxesCallback(msg);
        };
        sub_thumbstick_axes_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "/teleop/thumbstick_axes", 10, thumbstickAxesCallback);

        // 创建扳机模拟量订阅器（合并订阅左右扳机，使用 Twist 消息）
        // linear.x 表示左扳机拉动比例，angular.x 表示右扳机拉动比例
        auto triggerValuesCallback = [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            this->triggerValuesCallback(msg);
        };
        sub_trigger_values_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "/teleop/trigger_values", 10, triggerValuesCallback);

        // 双臂目标位姿发布器（仅双臂模式；用于尺度校准后发送校准目标）
        if (pub_right_target_)
        {
            pub_dual_target_stamped_ = node_->create_publisher<nav_msgs::msg::Path>(
                "/dual_target/stamped", 1);
        }

        // 创建 FSM 命令发布器（使用通用工具类，自动处理command=100的特殊情况）
        auto pub_fsm_command = node_->create_publisher<std_msgs::msg::Int32>("/fsm_command", 10);
        fsm_command_publisher_ = std::make_unique<arms_controller_common::FSMCommandPublisher>(
            node_, pub_fsm_command);

        // 底盘控制模式相关发布器（case 20 触发）
        pub_cmd_vel_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // WBC 模式切换命令发布器（case 21–24 请求身体模式）
        pub_mode_command_ = node_->create_publisher<std_msgs::msg::String>("/mode_command", 10);

        sub_wbc_capability_ =
            node_->create_subscription<arms_ros2_control_msgs::msg::WbcCapability>(
                "/ocs2_wbc_controller/wbc_capabilities",
                rclcpp::QoS(1).transient_local(),
                [this](const arms_ros2_control_msgs::msg::WbcCapability::ConstSharedPtr msg)
                {
                    this->wbcCapabilityCallback(msg);
                });

        // 注意：FSM命令订阅已移除，改为在 arms_target_manager_node 中统一处理
        // 这样可以避免与 ArmsTargetManager 的订阅冲突
        // FSM状态更新现在通过 fsmCommandCallback() 方法由外部调用

        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VRInputHandler created");
        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️ VR frames: reference_link=%s, full_body_follow_frame=%s",
            reference_link_.c_str(),
            vr_follow_frame_.c_str());
        RCLCPP_INFO(node_->get_logger(),
                    "🕹️🕶️🕹️ Chassis mode (case 20 = L+R thumbstick): L.stick→chassis XY, R.stick→waist lift/turn");
        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️ FULL_BODY directions: "
            "left grip + left stick cases 21-24; "
            "right grip + right stick cases 25-26, 28 (27 unused)");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Subscribed to button event topic: /teleop/controller_state (Int32)");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Subscribed to thumbstick axes topic: /teleop/thumbstick_axes (ThumbstickAxes)");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Subscribed to trigger values topic: /teleop/trigger_values (linear.x=left, angular.x=right)");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Trigger hand mode: percent control by default; press left Y + right B together to toggle percent/switch mode");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Thumbstick scaling: linear=%.3f, angular=%.3f",
                    vr_thumbstick_linear_scale_, vr_thumbstick_angular_scale_);
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VR pose scale: left=%.3f, right=%.3f",
                    vr_pose_scale_, vr_pose_scale_);
        RCLCPP_INFO(node_->get_logger(),
                    "🕹️🕶️🕹️ Grip short-press toggles thumbstick mode: XY-translation ↔ Z-height + Yaw-rotation");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VR control is DISABLED by default.");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Right thumbstick toggles between STORAGE and UPDATE modes.");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ STORAGE mode: Store VR and robot base poses (no marker update)");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ UPDATE mode: Calculate pose differences and update markers");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ MIRROR mode: Synced from xr_target_node (left thumbstick toggles)");
        
        // 输出检测到的控制器信息
        if (!left_gripper_controller_name_.empty())
        {
            RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Left gripper controller: %s", left_gripper_controller_name_.c_str());
        }
        if (!right_gripper_controller_name_.empty())
        {
            RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Right gripper controller: %s", right_gripper_controller_name_.c_str());
        }
        if (left_gripper_controller_name_.empty() && right_gripper_controller_name_.empty())
        {
            RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ No gripper controllers detected - gripper control disabled");
        }
    }

    void VRInputHandler::detectGripperControllers(const std::vector<std::string>& hand_controllers)
    {
        // 清空之前的检测结果
        left_gripper_controller_name_.clear();
        right_gripper_controller_name_.clear();

        // 从hand_controllers参数中提取左右控制器名称
        for (const auto& controller_name : hand_controllers)
        {
            // 转换为小写以便比较
            std::string name_lower = controller_name;
            std::transform(name_lower.begin(), name_lower.end(), name_lower.begin(), ::tolower);

            // 检测左控制器
            if (name_lower.find("left") != std::string::npos)
            {
                left_gripper_controller_name_ = controller_name;
                RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Detected left gripper controller: %s", controller_name.c_str());
            }
            // 检测右控制器
            else if (name_lower.find("right") != std::string::npos)
            {
                right_gripper_controller_name_ = controller_name;
                RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Detected right gripper controller: %s", controller_name.c_str());
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(),
                            "🕹️ Hand controller '%s' has no left/right in name; VR gripper mapping skipped",
                            controller_name.c_str());
            }
        }
    }

    void VRInputHandler::publishGripperCommand(const std::string& controller_name, int32_t command)
    {
        if (controller_name.empty())
        {
            return;
        }

        // 创建发布器（如果还没有）- 类似ControlInputHandler的方式
        if (gripper_command_publishers_.find(controller_name) == gripper_command_publishers_.end())
        {
            std::string topic_name = "/" + controller_name + "/target_command";
            gripper_command_publishers_[controller_name] = 
                node_->create_publisher<std_msgs::msg::Int32>(topic_name, 10);
            RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Created gripper command publisher: %s", topic_name.c_str());
        }

        // 发布命令
        auto target_msg = std_msgs::msg::Int32();
        target_msg.data = command;
        gripper_command_publishers_[controller_name]->publish(target_msg);

        // 开关命令也要写进位置缓存：last_gripper_percent_ 是"最后一次下发的夹爪位置"
        // 的唯一真相，百分比/开合两种模式都必须维护它，否则模式切回百分比时
        // 取不到真实的接管点 hold。
        last_gripper_percent_[controller_name] = (command == 1) ? 1.0 : 0.0;
    }

    void VRInputHandler::publishGripperPercent(const std::string& controller_name, double percent)
    {
        if (controller_name.empty())
        {
            return;
        }

        percent = std::clamp(percent, 0.0, 1.0);
        auto last_it = last_gripper_percent_.find(controller_name);
        if (last_it != last_gripper_percent_.end() && std::abs(last_it->second - percent) < 0.005)
        {
            return;
        }
        last_gripper_percent_[controller_name] = percent;

        if (gripper_percent_publishers_.find(controller_name) == gripper_percent_publishers_.end())
        {
            std::string topic_name = "/" + controller_name + "/target_percent";
            gripper_percent_publishers_[controller_name] =
                node_->create_publisher<std_msgs::msg::Float64>(topic_name, 10);
            RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Created gripper percent publisher: %s", topic_name.c_str());
        }

        auto target_msg = std_msgs::msg::Float64();
        target_msg.data = percent;
        gripper_percent_publishers_[controller_name]->publish(target_msg);
    }

    bool VRInputHandler::isGripperPercentMode(bool is_target_left_arm) const
    {
        return is_target_left_arm ? left_gripper_percent_mode_.load()
                                  : right_gripper_percent_mode_.load();
    }

    void VRInputHandler::prepareGripperForModeSwitch(bool is_target_left_arm, bool percent_mode)
    {
        const std::string& controller_name = is_target_left_arm
            ? left_gripper_controller_name_
            : right_gripper_controller_name_;
        if (controller_name.empty())
        {
            RCLCPP_WARN(node_->get_logger(),
                        "🕹️ [夹爪模式切换] 未检测到%s臂控制器，模式已切换但不会生效",
                        is_target_left_arm ? "左" : "右");
            return;
        }

        if (percent_mode)
        {
            // 切到百分比：不发任何指令，夹爪停在切换前的位置。
            // 记下接管点 hold——注意不能清 last_gripper_percent_，它现在由
            // publishGripperCommand 和两个 GripperStateCallback 一起维护，
            // 对开合模式同样有效，是"夹爪当前在哪"的唯一真相。
            auto it = last_gripper_percent_.find(controller_name);
            const double hold = (it != last_gripper_percent_.end())
                ? std::clamp(it->second, 0.0, 1.0)
                : 1.0;  // 查不到说明该侧还没收到过任何夹爪指令，按张开处理

            if (is_target_left_arm)
            {
                left_gripper_percent_hold_.store(hold);
                left_gripper_percent_saw_release_.store(false);
                left_gripper_percent_armed_.store(false);
            }
            else
            {
                right_gripper_percent_hold_.store(hold);
                right_gripper_percent_saw_release_.store(false);
                right_gripper_percent_armed_.store(false);
            }

            RCLCPP_INFO(node_->get_logger(),
                        "🕹️ [夹爪模式切换] %s夹爪保持在 %.2f，松开扳机不会动它；"
                        "把扳机扣到该位置（hand_percent<=%.2f）才接管",
                        is_target_left_arm ? "左" : "右", hold, hold);
        }
        else
        {
            // 切到开合：同样不发指令，但要把开合 flag 对齐到夹爪当前实际位置。
            // left/right_gripper_open_ 只订阅 /<controller>/target_command
            // （VRInputHandler.cpp:82-100），完全看不到 percent 命令，在百分比模式
            // 待过一段后必然是陈旧的；不对齐则下一次 case 9/10 的开合方向会反。
            auto it = last_gripper_percent_.find(controller_name);
            if (it != last_gripper_percent_.end())
            {
                // >0 视为仍张开：半闭（如 0.3）下次 9/10 应闭合，而不是按 0.5 阈值
                // 被当成已关闭后反向张开。完全闭合（0）才视为 close，下次打开。
                const bool open_now = it->second > 0.0;
                if (is_target_left_arm)
                {
                    left_gripper_open_.store(open_now);
                }
                else
                {
                    right_gripper_open_.store(open_now);
                }
            }
            // 查不到缓存说明该侧从未走过百分比，保持原 flag 不动
        }
    }

    void VRInputHandler::toggleGripperPercentMode(bool is_left_trigger)
    {
        std::string controller_name;
        bool is_target_left_arm = true;
        resolveTriggerTarget(is_left_trigger, controller_name, is_target_left_arm);

        auto& last_toggle = is_target_left_arm
            ? last_left_gripper_mode_toggle_time_
            : last_right_gripper_mode_toggle_time_;
        const auto now = std::chrono::steady_clock::now();
        if (now - last_toggle < std::chrono::milliseconds(500))
        {
            RCLCPP_DEBUG(node_->get_logger(),
                         "🔘 [%s] 夹爪模式切换过近，已忽略防抖",
                         is_left_trigger ? "左握把+左扳机" : "右握把+右扳机");
            return;
        }
        last_toggle = now;

        auto& mode_flag = is_target_left_arm
            ? left_gripper_percent_mode_
            : right_gripper_percent_mode_;
        const bool new_mode = !mode_flag.load();
        mode_flag.store(new_mode);

        prepareGripperForModeSwitch(is_target_left_arm, new_mode);
        RCLCPP_INFO(node_->get_logger(),
                    "🔘 [%s] %s夹爪切换为: %s，夹爪保持当前位置%s%s ｜ 当前模式 左=%s 右=%s",
                    is_left_trigger ? "左握把+左扳机" : "右握把+右扳机",
                    is_target_left_arm ? "左" : "右",
                    new_mode ? "比例控制(0~1，按压深度控制闭合)" : "开关控制(0/1，按一下开/关)",
                    new_mode ? "，扣扳机回到该位置才接管" : "",
                    mirror_mode_.load() ? " [镜像模式]" : "",
                    left_gripper_percent_mode_.load() ? "比例" : "开关",
                    right_gripper_percent_mode_.load() ? "比例" : "开关");
    }

    void VRInputHandler::runScaleCalibration(bool use_left_z)
    {
        const char* tag = use_left_z ? "左组合键" : "右组合键";

        mirror_mode_.store(false);
        RCLCPP_INFO(node_->get_logger(), "🔘 [%s] 已切换至非镜像模式", tag);
        is_update_mode_.store(false);
        stopHeadTracking();
        RCLCPP_INFO(node_->get_logger(),
                    "🔘 [%s] 自动切换到 STORAGE 模式 - 正在更新映射尺度，请重新进入 UPDATE 模式",
                    tag);

        const double head_ctrl_dist = use_left_z
            ? std::abs(vr_head_position_.z() - vr_left_position_raw_.z())
            : std::abs(vr_head_position_.z() - vr_right_position_raw_.z());
        RCLCPP_INFO(node_->get_logger(),
                    "🕹️🕶️🕹️ [%s][校准调试] frame初始化=%s, ee_frame_id='%s', reference_link='%s'",
                    tag,
                    ee_frame_id_initialized_ ? "true" : "false",
                    ee_frame_id_.c_str(),
                    reference_link_.c_str());
        RCLCPP_INFO(node_->get_logger(),
                    "🕹️🕶️🕹️ [%s][校准调试] VR raw: head=[%.4f, %.4f, %.4f], "
                    "left=[%.4f, %.4f, %.4f], right=[%.4f, %.4f, %.4f], head_ctrl_z_dist=%.6f",
                    tag,
                    vr_head_position_.x(), vr_head_position_.y(), vr_head_position_.z(),
                    vr_left_position_raw_.x(), vr_left_position_raw_.y(), vr_left_position_raw_.z(),
                    vr_right_position_raw_.x(), vr_right_position_raw_.y(), vr_right_position_raw_.z(),
                    head_ctrl_dist);

        // 校准使用即时 follow frame 计算完整目标，再转回 ee_frame_id_。
        // 连续手臂控制则只使用恢复时冻结的旋转映射增量。
        // 手柄-头显偏移量来自 VR 世界系（已在 xr 侧转成前x/左y/上z），其分量按
        // vr_follow_frame_ 的轴系解释。直接加在 ee_frame_id_ 上时，轮式底盘的
        // ee_frame_id_ 是 world，底盘一转向偏移方向就整体转错。
        const std::string& calc_frame = vr_follow_frame_;

        double ee_dist_from_ref = std::numeric_limits<double>::quiet_NaN();
        Eigen::Vector3d p_C_ref = Eigen::Vector3d::Zero();
        Eigen::Vector3d robot_left_pos_calc = Eigen::Vector3d::Zero();
        Eigen::Quaterniond robot_left_ori_calc = Eigen::Quaterniond::Identity();
        Eigen::Vector3d robot_right_pos_calc = Eigen::Vector3d::Zero();
        Eigen::Quaterniond robot_right_ori_calc = Eigen::Quaterniond::Identity();
        bool calc_frame_ready = false;
        if (ee_frame_id_initialized_)
        {
            try
            {
                geometry_msgs::msg::TransformStamped tf_ref_in_C =
                    tf_buffer_->lookupTransform(
                        calc_frame,
                        reference_link_,
                        tf2::TimePointZero);

                p_C_ref = Eigen::Vector3d(
                    tf_ref_in_C.transform.translation.x,
                    tf_ref_in_C.transform.translation.y,
                    tf_ref_in_C.transform.translation.z);
                RCLCPP_INFO(node_->get_logger(),
                            "🕹️🕶️🕹️ [%s][校准调试] TF %s -> %s: translation=[%.4f, %.4f, %.4f], "
                            "rotation=[x=%.4f, y=%.4f, z=%.4f, w=%.4f]",
                            tag,
                            reference_link_.c_str(), calc_frame.c_str(),
                            p_C_ref.x(), p_C_ref.y(), p_C_ref.z(),
                            tf_ref_in_C.transform.rotation.x,
                            tf_ref_in_C.transform.rotation.y,
                            tf_ref_in_C.transform.rotation.z,
                            tf_ref_in_C.transform.rotation.w);

                // 机器人当前末端位姿来自 current_pose（ee_frame_id_），先转到计算系，
                // 保证尺度用的 Z 距离和后面的加法在同一个系里。
                calc_frame_ready =
                    transformPoseBetweenFrames(
                        "left",
                        robot_current_left_position_,
                        robot_current_left_orientation_,
                        ee_frame_id_, calc_frame,
                        robot_left_pos_calc, robot_left_ori_calc) &&
                    transformPoseBetweenFrames(
                        "right",
                        robot_current_right_position_,
                        robot_current_right_orientation_,
                        ee_frame_id_, calc_frame,
                        robot_right_pos_calc, robot_right_ori_calc);

                if (calc_frame_ready)
                {
                    const Eigen::Vector3d& robot_ee = use_left_z
                        ? robot_left_pos_calc
                        : robot_right_pos_calc;
                    ee_dist_from_ref = std::abs(robot_ee.z() - p_C_ref.z());
                    RCLCPP_INFO(node_->get_logger(),
                                "🕹️🕶️🕹️ [%s][校准调试] Robot current in %s: left=[%.4f, %.4f, %.4f], "
                                "right=[%.4f, %.4f, %.4f], ref=[%.4f, %.4f, %.4f], ee_z_dist=%.6f",
                                tag,
                                calc_frame.c_str(),
                                robot_left_pos_calc.x(), robot_left_pos_calc.y(),
                                robot_left_pos_calc.z(),
                                robot_right_pos_calc.x(), robot_right_pos_calc.y(),
                                robot_right_pos_calc.z(),
                                p_C_ref.x(), p_C_ref.y(), p_C_ref.z(),
                                ee_dist_from_ref);
                }
                else
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "🕹️🕶️🕹️ [%s][校准调试] %s -> %s 变换不可用，"
                                "无法把机器人当前位姿转到计算系",
                                tag, ee_frame_id_.c_str(), calc_frame.c_str());
                }
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_WARN(node_->get_logger(),
                            "Failed to compute Z-dist(robot_%s_ee, %s) in frame %s: %s",
                            use_left_z ? "left" : "right",
                            reference_link_.c_str(), calc_frame.c_str(), ex.what());
            }
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                        "🕹️🕶️🕹️ [%s][校准调试] 无法查询TF：ee_frame_id_尚未从current_pose初始化",
                        tag);
        }

        if (calc_frame_ready && !std::isnan(ee_dist_from_ref) && head_ctrl_dist > 1e-6)
        {
            const double unclamped_scale = ee_dist_from_ref / head_ctrl_dist;
            vr_pose_scale_ = std::clamp(unclamped_scale, 0.75, 1.5);
            RCLCPP_INFO(node_->get_logger(),
                        "🕹️🕶️🕹️ [%s][校准调试] scale计算: robot_z_dist=%.6f / "
                        "vr_z_dist=%.6f => raw_scale=%.6f, clamped_scale=%.6f",
                        tag, ee_dist_from_ref, head_ctrl_dist, unclamped_scale, vr_pose_scale_);

            // 加法在计算系里做：p_C_ref 与 (手柄-头显) 偏移都表达在 calc_frame 轴系
            const Eigen::Vector3d left_target_pos_calc =
                p_C_ref + (vr_left_position_raw_ - vr_head_position_) * vr_pose_scale_;
            const Eigen::Vector3d right_target_pos_calc =
                p_C_ref + (vr_right_position_raw_ - vr_head_position_) * vr_pose_scale_;
            const Eigen::Quaterniond left_target_ori_calc =
                robot_left_ori_calc.normalized();
            const Eigen::Quaterniond right_target_ori_calc =
                robot_right_ori_calc.normalized();
            RCLCPP_INFO(node_->get_logger(),
                        "🕹️🕶️🕹️ [%s][校准调试] VR offset raw: left-head=[%.4f, %.4f, %.4f], "
                        "right-head=[%.4f, %.4f, %.4f]",
                        tag,
                        (vr_left_position_raw_ - vr_head_position_).x(),
                        (vr_left_position_raw_ - vr_head_position_).y(),
                        (vr_left_position_raw_ - vr_head_position_).z(),
                        (vr_right_position_raw_ - vr_head_position_).x(),
                        (vr_right_position_raw_ - vr_head_position_).y(),
                        (vr_right_position_raw_ - vr_head_position_).z());
            RCLCPP_INFO(node_->get_logger(),
                        "🕹️🕶️🕹️ [%s] vr_pose_scale_ 已更新: "
                        "robot_z_dist=%.4f, vr_z_dist=%.4f, scale=%.4f",
                        tag, ee_dist_from_ref, head_ctrl_dist, vr_pose_scale_);
            RCLCPP_INFO(node_->get_logger(),
                        "🕹️🕶️🕹️ [%s] 计算系(%s) 左臂目标: [%.3f, %.3f, %.3f]  "
                        "右臂目标: [%.3f, %.3f, %.3f]",
                        tag, calc_frame.c_str(),
                        left_target_pos_calc.x(), left_target_pos_calc.y(), left_target_pos_calc.z(),
                        right_target_pos_calc.x(), right_target_pos_calc.y(), right_target_pos_calc.z());

            if (pub_dual_target_stamped_)
            {
                // 计算完毕，转回发布系（ee_frame_id_）再发出去
                Eigen::Vector3d left_pub_pos = left_target_pos_calc;
                Eigen::Quaterniond left_pub_ori = left_target_ori_calc;
                Eigen::Vector3d right_pub_pos = right_target_pos_calc;
                Eigen::Quaterniond right_pub_ori = right_target_ori_calc;
                if (calc_frame != ee_frame_id_ &&
                    !(transformPoseBetweenFrames(
                          "left", left_target_pos_calc, left_target_ori_calc,
                          calc_frame, ee_frame_id_,
                          left_pub_pos, left_pub_ori) &&
                      transformPoseBetweenFrames(
                          "right", right_target_pos_calc, right_target_ori_calc,
                          calc_frame, ee_frame_id_,
                          right_pub_pos, right_pub_ori)))
                {
                    // 转不回发布系就整个放弃：不发布也不动缓存，机器人保持原状，
                    // 比发一个系错了的目标安全。
                    RCLCPP_WARN(node_->get_logger(),
                                "🕹️🕶️🕹️ [%s] %s -> %s 变换不可用，本次校准目标未发布；"
                                "请等 TF 就绪后重新校准",
                                tag, calc_frame.c_str(), ee_frame_id_.c_str());
                    return;
                }

                nav_msgs::msg::Path dual_path;
                dual_path.header.stamp = node_->get_clock()->now();
                dual_path.header.frame_id = ee_frame_id_;
                dual_path.poses.resize(2);
                dual_path.poses[0].header = dual_path.header;
                dual_path.poses[0].pose.position.x = left_pub_pos.x();
                dual_path.poses[0].pose.position.y = left_pub_pos.y();
                dual_path.poses[0].pose.position.z = left_pub_pos.z();
                dual_path.poses[0].pose.orientation.x = left_pub_ori.x();
                dual_path.poses[0].pose.orientation.y = left_pub_ori.y();
                dual_path.poses[0].pose.orientation.z = left_pub_ori.z();
                dual_path.poses[0].pose.orientation.w = left_pub_ori.w();
                dual_path.poses[1].header = dual_path.header;
                dual_path.poses[1].pose.position.x = right_pub_pos.x();
                dual_path.poses[1].pose.position.y = right_pub_pos.y();
                dual_path.poses[1].pose.position.z = right_pub_pos.z();
                dual_path.poses[1].pose.orientation.x = right_pub_ori.x();
                dual_path.poses[1].pose.orientation.y = right_pub_ori.y();
                dual_path.poses[1].pose.orientation.z = right_pub_ori.z();
                dual_path.poses[1].pose.orientation.w = right_pub_ori.w();
                pub_dual_target_stamped_->publish(dual_path);
                RCLCPP_INFO(node_->get_logger(),
                            "🕹️🕶️🕹️ [%s] 已发布校准目标到 /dual_target/stamped (frame: %s): "
                            "left=[%.3f, %.3f, %.3f], right=[%.3f, %.3f, %.3f]",
                            tag, ee_frame_id_.c_str(),
                            left_pub_pos.x(), left_pub_pos.y(), left_pub_pos.z(),
                            right_pub_pos.x(), right_pub_pos.y(), right_pub_pos.z());

                // 校准绕过 publishTargetPoseDirect 直接改变了机器人目标，缓存的
                // command target 已经过期；不处理的话，下次进入 UPDATE 会拿校准前的
                // 旧目标当锚点，手臂会从校准位姿弹回旧位置。
                // 这里把校准终点当作"刚刚发布过的目标"写回缓存：锚点即终点，
                // 于是不必等手臂收敛就能进 UPDATE，未走完的运动会继续走完。
                // last_published_* 存发布系的值（与 publishTargetPoseDirect 一致），
                // prev_calculated_* 存发布系的值——它被 hasPoseChanged() 无条件读取。
                recordLastPublishedTarget("left", left_pub_pos, left_pub_ori);
                recordLastPublishedTarget("right", right_pub_pos, right_pub_ori);
                // 让 robot_base_* 失效，下次进 UPDATE 从上面写回的 command target 重新派生
                left_robot_base_valid_ = false;
                right_robot_base_valid_ = false;
                prev_calculated_left_position_ = left_pub_pos;
                prev_calculated_left_orientation_ = left_pub_ori;
                prev_calculated_right_position_ = right_pub_pos;
                prev_calculated_right_orientation_ = right_pub_ori;
                RCLCPP_INFO(node_->get_logger(),
                            "🕹️🕶️🕹️ [%s] 已将校准终点写回 command target 缓存"
                            "（锚点系: %s）；无需等手臂收敛即可进入 UPDATE",
                            tag, calc_frame.c_str());
            }
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                        "🕹️🕶️🕹️ [%s] 无法更新 vr_pose_scale_: "
                        "计算系(%s)就绪=%s, robot_z_dist=%s, vr_z_dist=%.4f",
                        tag,
                        calc_frame.c_str(),
                        calc_frame_ready ? "true" : "false",
                        std::isnan(ee_dist_from_ref) ? "NaN" : std::to_string(ee_dist_from_ref).c_str(),
                        head_ctrl_dist);
        }
    }

    void VRInputHandler::resolveTriggerTarget(bool is_left_trigger,
                                             std::string& controller_name,
                                             bool& is_target_left_arm) const
    {
        if (mirror_mode_.load())
        {
            if (is_left_trigger)
            {
                controller_name = right_gripper_controller_name_;
                is_target_left_arm = false;
            }
            else
            {
                controller_name = left_gripper_controller_name_;
                is_target_left_arm = true;
            }
        }
        else
        {
            if (is_left_trigger)
            {
                controller_name = left_gripper_controller_name_;
                is_target_left_arm = true;
            }
            else
            {
                controller_name = right_gripper_controller_name_;
                is_target_left_arm = false;
            }
        }
    }

    void VRInputHandler::toggleGripperByTrigger(bool is_left_trigger)
    {
        std::string target_controller_name;
        bool is_target_left_arm = true;
        resolveTriggerTarget(is_left_trigger, target_controller_name, is_target_left_arm);

        const char* trigger_name = is_left_trigger ? "左扳机按钮" : "右扳机按钮";
        if (target_controller_name.empty())
        {
            RCLCPP_WARN(node_->get_logger(),
                        "🔘 [%s] 按下 - 功能: 控制夹爪开合 - 操作: 失败（未检测到%s臂控制器）",
                        trigger_name, is_target_left_arm ? "左" : "右");
            return;
        }

        bool current_gripper_open = is_target_left_arm ? left_gripper_open_.load() : right_gripper_open_.load();
        int32_t command = current_gripper_open ? 0 : 1; // 0=close, 1=open
        publishGripperCommand(target_controller_name, command);

        RCLCPP_INFO(node_->get_logger(),
                    "🔘 [%s] 按下 - 模式: 开关控制(0/1) - 操作: %s夹爪已%s%s",
                    trigger_name,
                    is_target_left_arm ? "左" : "右",
                    (command == 1) ? "打开" : "关闭",
                    mirror_mode_.load() ? " [镜像模式]" : "");
    }

    void VRInputHandler::leftGripperStateCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        // 同步左夹爪状态（从 topic 中获取）
        left_gripper_open_.store(msg->data == 1);
        // 同步位置缓存，覆盖外部节点（rviz 面板 / teleop）下发的开关命令
        if (!left_gripper_controller_name_.empty())
        {
            last_gripper_percent_[left_gripper_controller_name_] = (msg->data == 1) ? 1.0 : 0.0;
        }
        RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Left gripper state synced: %s",
                     (msg->data == 1) ? "open" : "close");
    }

    void VRInputHandler::rightGripperStateCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        // 同步右夹爪状态（从 topic 中获取）
        right_gripper_open_.store(msg->data == 1);
        // 同步位置缓存，覆盖外部节点（rviz 面板 / teleop）下发的开关命令
        if (!right_gripper_controller_name_.empty())
        {
            last_gripper_percent_[right_gripper_controller_name_] = (msg->data == 1) ? 1.0 : 0.0;
        }
        RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Right gripper state synced: %s",
                     (msg->data == 1) ? "open" : "close");
    }

    void VRInputHandler::sendFsmCommand(int32_t command)
    {
        // 使用通用工具类发布FSM命令（自动处理command=100的特殊情况）
        if (fsm_command_publisher_)
        {
            fsm_command_publisher_->publishCommand(command);
        }
    }

    int32_t VRInputHandler::resolvedFsmState() const
    {
        if (target_manager_)
        {
            const std::string& name = target_manager_->getCurrentFsmState();
            if (name == "HOME")
            {
                return 1;
            }
            if (name == "HOLD")
            {
                return 2;
            }
            if (name == "OCS2")
            {
                return 3;
            }
            if (name == "MOVEJ")
            {
                return 4;
            }
        }
        return current_fsm_state_.load();
    }

    void VRInputHandler::enable()
    {
        enabled_.store(true);
        // 启用VR控制时，自动设置为连续发布模式以获得更好的响应性
        if (target_manager_ && target_manager_->getCurrentMode() != MarkerState::CONTINUOUS)
        {
            target_manager_->togglePublishMode();
            RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ ArmsTargetManager switched to CONTINUOUS mode for VR control");
        }
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VR control ENABLED!");
    }

    void VRInputHandler::disable()
    {
        enabled_.store(false);
        left_robot_base_valid_ = false;
        right_robot_base_valid_ = false;
        stopHeadTracking();
        // 全身遥操随 VR 禁用一起退出
        if (whole_body_teleop_active_.load())
        {
            whole_body_teleop_active_.store(false);
        }
        // 绝对位姿映射与底盘跟踪随 VR 禁用一起退出
        absolute_tracking_active_.store(false);
        resetAbsoluteTracking();
        base_tracking_enabled_.store(false);
        resetBaseFollow();
        left_grip_direction_suppressed_.store(false);
        body_mode_request_pending_.store(false);
        requested_body_state_.store(-1);
        right_grip_direction_suppressed_.store(false);
        right_wbc_toggle_request_pending_.store(false);
        requested_wbc_toggle_target_.store(WbcToggleTarget::NONE);
        resetYbModeLatchAndConversions();
        resetStaleCatchUpRamp();
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VR control DISABLED!");

        // 禁用 VR 控制时，若处于底盘模式则退出并清零底盘/腰部命令，防止残留运动
        if (chassis_mode_.load())
        {
            chassis_mode_.store(false);
        }
        resetChassisAndWaistCommands();
    }

    void VRInputHandler::robotLeftPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // 初始化末端参考坐标系（frame_id）- 左右手共用一次即可
        if (!msg->header.frame_id.empty() &&
            (!ee_frame_id_initialized_ || ee_frame_id_ != msg->header.frame_id))
        {
            clearLastPublishedTargets();
            has_robot_current_left_pose_.store(false);
            has_robot_current_right_pose_.store(false);
            ee_frame_id_ = msg->header.frame_id;
            ee_frame_id_initialized_ = true;
            RCLCPP_INFO(node_->get_logger(),
                        "🕹️🕶️🕹️ EE frame_id initialized from left_current_pose: %s",
                        ee_frame_id_.c_str());
        }

        // 直接使用 Pose 版本，避免代码重复
        auto pose_msg = std::make_shared<geometry_msgs::msg::Pose>(msg->pose);
        Eigen::Matrix4d pose = poseMsgToMatrix(pose_msg);
        matrixToPosOri(pose, robot_current_left_position_, robot_current_left_orientation_);
        has_robot_current_left_pose_.store(true);
    }

    void VRInputHandler::robotRightPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // 如果还未初始化 frame_id，也可以从右手话题初始化（通常左右相同）
        if (!msg->header.frame_id.empty() &&
            (!ee_frame_id_initialized_ || ee_frame_id_ != msg->header.frame_id))
        {
            clearLastPublishedTargets();
            has_robot_current_left_pose_.store(false);
            has_robot_current_right_pose_.store(false);
            ee_frame_id_ = msg->header.frame_id;
            ee_frame_id_initialized_ = true;
            RCLCPP_INFO(node_->get_logger(),
                        "🕹️🕶️🕹️ EE frame_id initialized from right_current_pose: %s",
                        ee_frame_id_.c_str());
        }

        // 直接使用 Pose 版本，避免代码重复
        auto pose_msg = std::make_shared<geometry_msgs::msg::Pose>(msg->pose);
        Eigen::Matrix4d pose = poseMsgToMatrix(pose_msg);
        matrixToPosOri(pose, robot_current_right_position_, robot_current_right_orientation_);
        has_robot_current_right_pose_.store(true);
    }

    void VRInputHandler::vrHeadCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        const Eigen::Vector3d position(msg->position.x, msg->position.y, msg->position.z);
        const Eigen::Quaterniond orientation(
            msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        if (!position.allFinite() || !orientation.coeffs().allFinite() || orientation.norm() < 1e-9)
        {
            has_vr_head_pose_ = false;
            stopHeadTracking();
            return;
        }
        vr_head_position_ = position;
        vr_head_orientation_ = orientation.normalized();
        has_vr_head_pose_ = true;
        publishHeadTarget();
    }

    void VRInputHandler::robotHeadPoseCallback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
        const auto& p = msg->pose.position;
        const auto& q = msg->pose.orientation;
        const Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
        if (msg->header.frame_id.empty() || !Eigen::Vector3d(p.x, p.y, p.z).allFinite() ||
            !orientation.coeffs().allFinite() || orientation.norm() < 1e-9)
        {
            has_robot_head_pose_ = false;
            stopHeadTracking();
            return;
        }
        robot_head_pose_ = *msg;
        const auto normalized = orientation.normalized();
        robot_head_pose_.pose.orientation.x = normalized.x();
        robot_head_pose_.pose.orientation.y = normalized.y();
        robot_head_pose_.pose.orientation.z = normalized.z();
        robot_head_pose_.pose.orientation.w = normalized.w();
        has_robot_head_pose_ = true;
    }

    bool VRInputHandler::headTrackingReady() const
    {
        return target_manager_ && isFullBodyMode() && enabled_.load() && is_update_mode_.load() &&
            resolvedFsmState() == 3 &&
            target_manager_->getCurrentHeadState() ==
                arms_ros2_control_msgs::msg::WbcCurrentState::HEAD_TRACKING &&
            (whole_body_teleop_active_.load() ? has_body_head_pose_ : has_vr_head_pose_) &&
            has_robot_head_pose_;
    }

    void VRInputHandler::stopHeadTracking()
    {
        head_tracking_active_ = false;
        // Preserve the commanded equilibrium across pauses and compliant displacement.
        if (target_manager_)
        {
            target_manager_->setHeadVrActive(false);
        }
    }

    void VRInputHandler::publishHeadTarget()
    {
        if (!headTrackingReady())
        {
            stopHeadTracking();
            return;
        }

        // 全身遥操作模式**只有绝对映射**（body_head 骨架位姿 → head_link2）：
        // 按下右摇杆开「全部跟踪」后直接采当前骨架位姿，不存在相对模式。
        if (whole_body_teleop_active_.load())
        {
            if (!absolute_tracking_active_.load())
            {
                return;  // 跟踪未开：不下发头部目标，由 WBC 保持当前目标
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
            if (!skeletonJointFresh(body_head_stamp_))
            {
                return;  // 数据失效：冻结头部任务
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
                Eigen::Quaterniond(
                    head_rot_in_H * abs_head_src_to_ee_)
                    .normalized());

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
            target.header.frame_id = ee_frame_id_;
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
            return;
        }

        // 非全身遥操作：头显（/teleop/head_pose）驱动 WBC 头部目标的相对增量路径（保留）
        const Eigen::Vector3d vr_head_position = vr_head_position_;
        const Eigen::Quaterniond vr_head_orientation = vr_head_orientation_;
        if (!head_tracking_active_)
        {
            const auto& base = last_head_target_ ? *last_head_target_ : robot_head_pose_;
            const auto& p = base.pose.position;
            const auto& q = base.pose.orientation;
            if (!transformPoseBetweenFrames("head", Eigen::Vector3d(p.x, p.y, p.z),
                    Eigen::Quaterniond(q.w, q.x, q.y, q.z), base.header.frame_id, vr_follow_frame_,
                    robot_head_base_position_, robot_head_base_orientation_))
            {
                return;
            }
            vr_head_base_position_ = vr_head_position;
            vr_head_base_orientation_ = vr_head_orientation;
        }

        // Both upstream wrappers already convert OpenXR axes to robot axes.
        // Align those axes with vr_follow_frame, independently of hand scale/mirror settings.
        const Eigen::Vector3d position = robot_head_base_position_ +
            (vr_head_position - vr_head_base_position_);
        const Eigen::Quaterniond orientation = (vr_head_orientation *
            vr_head_base_orientation_.inverse() * robot_head_base_orientation_).normalized();
        Eigen::Vector3d publishedPosition;
        Eigen::Quaterniond publishedOrientation;
        const auto& frame = robot_head_pose_.header.frame_id;
        if (!transformPoseBetweenFrames("head", position, orientation, vr_follow_frame_, frame,
                publishedPosition, publishedOrientation))
        {
            stopHeadTracking();
            return;
        }
        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = frame;
        target.header.stamp = node_->now();
        target.pose.position.x = publishedPosition.x();
        target.pose.position.y = publishedPosition.y();
        target.pose.position.z = publishedPosition.z();
        target.pose.orientation.x = publishedOrientation.x();
        target.pose.orientation.y = publishedOrientation.y();
        target.pose.orientation.z = publishedOrientation.z();
        target.pose.orientation.w = publishedOrientation.w();
        target_manager_->setHeadVrActive(true);
        pub_head_target_->publish(target.pose);
        last_head_target_ = target;
        last_head_target_stamp_ = std::chrono::steady_clock::now();
        head_tracking_active_ = true;
    }

    // ================= 全身遥操（case 30）=================

    void VRInputHandler::bodyPelvisCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
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

    void VRInputHandler::bodySpine3Callback(const geometry_msgs::msg::Pose::SharedPtr msg)
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

    void VRInputHandler::bodyHeadCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
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

    void VRInputHandler::humanFootprintCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
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

    bool VRInputHandler::wholeBodyTeleopReady() const
    {
        // 与双臂末端跟随同步：只有「全部跟踪」开着才下发躯干目标
        return target_manager_ && isFullBodyMode() && enabled_.load() &&
            is_update_mode_.load() &&
            resolvedFsmState() == 3 &&
            target_manager_->getCurrentBodyState() ==
                arms_ros2_control_msgs::msg::WbcCurrentState::BODY_TRACKING &&
            has_body_pelvis_pose_ && has_body_spine3_pose_;
    }

    void VRInputHandler::publishBodyTarget()
    {
        if (!whole_body_teleop_active_.load())
        {
            return;
        }
        if (!wholeBodyTeleopReady())
        {
            return;
        }

        // 全身遥操作模式**只有绝对映射**：按下右摇杆开「全部跟踪」后直接采当前骨架位姿，
        // 不存在「按下时记录基准 + 跟踪相对变化量」的相对模式；
        // 跟踪未开（is_update_mode_ 为 false）时不下发任何目标，由 WBC 保持当前目标。
        if (!absolute_tracking_active_.load())
        {
            return;
        }

        {
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
                Eigen::Quaterniond(
                    spine_rot_in_H * abs_body_src_to_ee_)
                    .normalized());

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
            target.header.frame_id = ee_frame_id_;
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
    }

    void VRInputHandler::toggleWholeBodyTeleop()
    {
        if (!enabled_.load() || !isFullBodyMode() || current_fsm_state_.load() != 3)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "🔘 [case 30] 忽略全身遥操作开关：要求 VR enabled、FULL_BODY、OCS2");
            return;
        }

        if (whole_body_teleop_active_.load())
        {
            stopWholeBodyTeleop();
        }
        else
        {
            startWholeBodyTeleop();
        }
    }

    void VRInputHandler::startWholeBodyTeleop()
    {
        whole_body_teleop_active_.store(true);
        // 进入模式只开通“允许跟踪”；绝对位姿映射需再短按右摇杆才开启
        absolute_tracking_active_.store(false);
        is_update_mode_.store(false);
        resetAbsoluteTracking();
        base_tracking_enabled_.store(false);
        resetBaseFollow();
        // 头部数据源切换为骨架：清掉本轮跟踪状态，下一帧用骨架数据重新建立基准
        stopHeadTracking();

        // 清掉可能残留的身体模式请求与摇杆累积偏移：
        // 本模式下摇杆不再给末端/腰部做微调（见 thumbstickAxesCallback 的早退），
        // 因此不能带着旧偏移进入绝对映射。
        body_mode_request_pending_.store(false);
        requested_body_state_.store(-1);
        left_thumbstick_offset_.setZero();
        right_thumbstick_offset_.setZero();
        left_thumbstick_yaw_offset_ = 0.0;
        right_thumbstick_yaw_offset_ = 0.0;

        // 身体模式改为「跟随」、头部模式改为「跟踪」；目标随后由 /teleop/body_* 计算
        requestBodyMode(30, "左右握把+右摇杆按下", "跟随", "BODY_TRACKING",
                        arms_ros2_control_msgs::msg::WbcCurrentState::BODY_TRACKING);
        publishHumanoidModeCommand("HEAD_TRACKING");

        RCLCPP_INFO(node_->get_logger(),
                    "🕹️🦴 [case 30] 全身遥操作已开启：身体=跟随（BODY_TRACKING）、"
                    "头部=跟踪（HEAD_TRACKING）；按下右摇杆进入 UPDATE 后开始跟随"
                    "（与双臂同步），目标位姿由 /teleop/body_* 骨骼数据计算；"
                    "该模式下左右摇杆不再微调末端/腰部；"
                    "再次按左右握把+右摇杆按下关闭");
    }

    void VRInputHandler::stopWholeBodyTeleop()
    {
        whole_body_teleop_active_.store(false);
        // 绝对跟踪与底盘跟踪随全身模式一起退出
        absolute_tracking_active_.store(false);
        is_update_mode_.store(false);
        resetAbsoluteTracking();
        base_tracking_enabled_.store(false);
        resetBaseFollow();
        // 头部数据源切回头显：清掉跟踪状态，让后续用头显数据重新建立基准
        stopHeadTracking();
        RCLCPP_INFO(node_->get_logger(),
                    "🕹️🦴 [case 30] 全身遥操作已关闭（WBC 身体/头部模式保持当前状态，"
                    "可用原有组合键切换）");
    }

    bool VRInputHandler::exitWholeBodyTeleopIfFsmNotOcs2()
    {
        if (!whole_body_teleop_active_.load())
        {
            return false;
        }
        const int32_t fsm_state = resolvedFsmState();
        if (fsm_state == 3)  // 3 = OCS2
        {
            return false;
        }

        // 先同步本地 FSM 缓存：stopWholeBodyTeleop() → resetBaseFollow() 的零速会经过
        // wbcOwnsChassisVelocity()，而它读的是 current_fsm_state_。缓存落后于权威状态时
        // 会被判成「OCS2 + BASE_UNLOCKED 让路给 WBC」，把退出时该发的那一次零速吞掉。
        if (fsm_state != current_fsm_state_.load())
        {
            current_fsm_state_.store(fsm_state);
        }
        stopWholeBodyTeleop();
        RCLCPP_INFO(node_->get_logger(),
                    "🕹️🦴 FSM 已离开 OCS2（状态=%d）：全身遥操作自动退出"
                    "（「全部跟踪」与「底盘跟踪」一并关闭，WBC 保持当前目标）；"
                    "回到 OCS2 后需重新按「左右握把+右摇杆按下」开启",
                    fsm_state);
        return true;
    }

    // ================= 全身骨架绝对位姿映射（全部跟踪） =================
    //
    // 语义：全身模式（case 30）内短按右摇杆开/关「全部跟踪」，之后胸 / 头 / 双臂末端
    // 目标全部由 /teleop/body_* 的**当前**位姿给出，不含任何「按下时记录状态」：
    //   1) 人体根 H = 下肢骨架（xy/偏航取两踝、z 取两足较低者）+ 固定原点偏置；
    //      它只用来驱动底盘速度指令，人体的行走/整体转身因此不会漏进上肢目标；
    //   2) 骨架分段尺度（身长/肩宽/臂长）在 H 里缩放「段向量」（姿态不变）；
    //   3) 直接作为机器人对应部位「相对底座」的目标：
    //        位置 = R_align · p'                （p' 的 z 即离地高度）
    //        姿态 = R_align · R_H · C_src→ee
    //   4) 再乘底座位姿得到 world 下的目标：优先 TF(vr_follow_frame→ee_frame_id)，
    //      无 TF 时回退到「对自己下发 /cmd_vel 的积分」。
    // 该模式下摇杆不参与微调（见 thumbstickAxesCallback 的早退）。
    // 效果：底盘锁定时人走动不拖手臂；底盘跟随时手臂保持相对机体的姿态；
    //       人伸手/转腰/转头则立即反映到目标上。

    namespace
    {
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

    bool VRInputHandler::humanFootprintTransform(Eigen::Isometry3d& T_H_world)
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

    Eigen::Isometry3d VRInputHandler::currentBaseToWorld()
    {
        Eigen::Vector3d position;
        Eigen::Quaterniond rotation;
        if (ee_frame_id_initialized_ && !vr_follow_frame_.empty() &&
            transformPoseBetweenFrames(
                "chassis", Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                vr_follow_frame_, ee_frame_id_, position, rotation))
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
            vr_follow_frame_.c_str(), ee_frame_id_.c_str());

        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.linear() =
            Eigen::AngleAxisd(base_est_yaw_, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        transform.translation() =
            Eigen::Vector3d(base_est_pos_.x(), base_est_pos_.y(), 0.0);
        return transform;
    }

    void VRInputHandler::integrateBaseOdometry(const geometry_msgs::msg::Twist& velocity)
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

    void VRInputHandler::bodyHandCallback(
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

    void VRInputHandler::bodyShoulderCallback(
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

    void VRInputHandler::buildDeformedSkeleton(
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

    bool VRInputHandler::skeletonJointFresh(
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

    void VRInputHandler::toggleAbsoluteTracking()
    {
        if (!enabled_.load() || !isFullBodyMode() || resolvedFsmState() != 3)
        {
            RCLCPP_WARN(node_->get_logger(),
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

    void VRInputHandler::startAbsoluteTracking()
    {
        absolute_tracking_active_.store(true);
        absolute_maps_captured_ = false;
        settle_pending_ = true;
        settle_started_at_ = std::chrono::steady_clock::now();

        abs_left_filter_primed_ = false;
        abs_right_filter_primed_ = false;
        abs_body_filter_primed_ = false;
        abs_head_filter_primed_ = false;

        // 复用 is_update_mode_：非全身模式下的既有语义（UPDATE/STORAGE）不变；
        // 全身模式下它就是「全部跟踪开关」。
        is_update_mode_.store(true);
        if (target_manager_ && target_manager_->getCurrentMode() != MarkerState::CONTINUOUS)
        {
            target_manager_->togglePublishMode();
        }

        // 目标来源切换，清掉摇杆累积与追赶斜坡，避免切换瞬间跳变
        left_thumbstick_offset_.setZero();
        right_thumbstick_offset_.setZero();
        left_thumbstick_yaw_offset_ = 0.0;
        right_thumbstick_yaw_offset_ = 0.0;
        resetStaleCatchUpRamp();

        // 接管渐入：以机器人当前保持的位姿为锚点，按速度约束过渡到实时 VR 期望位姿。
        // 标定（captureAbsoluteTrackingMaps）只校验骨架数据，因此锚点必须在这里取。
        armAbsoluteRamps();

        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️🦴 [case 4] 全身「全部跟踪」已开启：胸/头/双臂末端改为 /teleop/body_* 绝对位姿映射"
            "（等待机器人当前位姿与骨架稳定后标定）；再次短按右摇杆关闭");
    }

    void VRInputHandler::stopAbsoluteTracking()
    {
        absolute_tracking_active_.store(false);
        is_update_mode_.store(false);
        resetAbsoluteTracking();
        RCLCPP_INFO(node_->get_logger(),
                    "🕹️🦴 [case 4] 全身「全部跟踪」已关闭：停止下发胸/头/双臂目标"
                    "（WBC 保持当前目标，长按右摇杆的底盘跟踪开关独立不受影响）");
    }

    Eigen::Quaterniond VRInputHandler::dropRoll(const Eigen::Quaterniond& q)
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

    bool VRInputHandler::geometryToleranceOk(
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

    bool VRInputHandler::captureAbsoluteTrackingMaps()
    {
        if (!ee_frame_id_initialized_)
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
            ee_frame_id_.c_str(),
            absolute_options_.body_length_scale,
            absolute_options_.shoulder_width_scale,
            absolute_options_.arm_length_scale,
            absolute_options_.ee_source.c_str(),
            absolute_options_.body_anchor_lambda,
            abs_left_arm_ref_len_, abs_right_arm_ref_len_);
        return true;
    }

    void VRInputHandler::publishAbsoluteArmTarget(bool is_left)
    {
        if (!absolute_tracking_active_.load() || !whole_body_teleop_active_.load())
        {
            return;
        }
        if (!enabled_.load() || !isFullBodyMode() || resolvedFsmState() != 3)
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

        const WbcToggleTarget target =
            is_left ? WbcToggleTarget::LEFT_ARM : WbcToggleTarget::RIGHT_ARM;
        if (isArmVrInputSuppressed(target))
        {
            return;
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
        Eigen::Vector3d position_base = hand_in_H;
        Eigen::Matrix3d rot_base = hand_rot_in_H * hand_to_ee;

        // 摇杆微调（在底座系里叠加）
        const Eigen::Vector3d stick_offset =
            is_left ? left_thumbstick_offset_ : right_thumbstick_offset_;
        const double stick_yaw =
            is_left ? left_thumbstick_yaw_offset_ : right_thumbstick_yaw_offset_;
        if (!stick_offset.isZero(1e-9))
        {
            position_base += stick_offset;
        }
        if (std::abs(stick_yaw) > 1e-9)
        {
            rot_base = Eigen::AngleAxisd(stick_yaw, Eigen::Vector3d::UnitZ())
                           .toRotationMatrix() * rot_base;
        }

        // 底座 → world（优先 TF；无 TF 时用自下发 /cmd_vel 的积分估计）
        const Eigen::Isometry3d T_base_world = currentBaseToWorld();
        Eigen::Vector3d position =
            T_base_world.linear() * position_base + T_base_world.translation();
        Eigen::Quaterniond orientation(
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

        // 接管渐入：只在「开启全部跟踪」后的过渡段生效，收敛后 update() 直通，
        // 稳态跟随手感与改动前一致。放在 publishTargetPoseDirect 之前，
        // 使冻结追赶斜坡看到的就是真正要发布的目标。
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

        publishTargetPoseDirect(is_left ? "left" : "right", ramped_pos, ramped_ori);
        if (target_manager_)
        {
            // 本模式下该臂目标由全身遥操作独占：
            // 禁掉 RViz 交互 marker 拖拽/菜单 与手柄/键盘 control_input 增量的改写。
            target_manager_->setArmVrActive(is_left, true);
        }
    }

    void VRInputHandler::resetAbsoluteTracking()
    {
        absolute_maps_captured_ = false;
        settle_pending_ = false;
        abs_left_filter_primed_ = false;
        abs_right_filter_primed_ = false;
        abs_body_filter_primed_ = false;
        abs_head_filter_primed_ = false;
        abs_left_arm_ref_len_ = 0.0;
        abs_right_arm_ref_len_ = 0.0;
        resetStaleCatchUpRamp();
        resetAbsoluteRamps();
        // 交还臂目标独占标志（唯一消费者是手柄/键盘 control_input 的 twist 改写）
        if (target_manager_)
        {
            target_manager_->setArmVrActive(true, false);
            target_manager_->setArmVrActive(false, false);
        }
    }

    void VRInputHandler::armAbsoluteRamps()
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
        if (has_last_published_left_target_ && isFresh(last_published_left_stamp_))
        {
            left_arm_ramp_.armFrom(
                last_published_left_position_, last_published_left_orientation_);
        }
        else if (has_robot_current_left_pose_.load())
        {
            left_arm_ramp_.armFrom(
                robot_current_left_position_, robot_current_left_orientation_);
        }
        else
        {
            left_arm_ramp_.reset();
        }

        if (has_last_published_right_target_ && isFresh(last_published_right_stamp_))
        {
            right_arm_ramp_.armFrom(
                last_published_right_position_, last_published_right_orientation_);
        }
        else if (has_robot_current_right_pose_.load())
        {
            right_arm_ramp_.armFrom(
                robot_current_right_position_, robot_current_right_orientation_);
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

        // ---- 头部：最后一次下发的目标（可能来自非全身模式的另一个 frame），
        //      先转到发布系；不可用则退回实测头部位姿 ----
        bool head_armed = false;
        const auto arm_head_from = [this, &head_armed](
                                       const Eigen::Vector3d& position,
                                       const Eigen::Quaterniond& orientation,
                                       const std::string& frame_id)
        {
            if (frame_id == ee_frame_id_)
            {
                head_ramp_.armFrom(position, orientation);
                head_armed = true;
                return;
            }
            Eigen::Vector3d transformed_position;
            Eigen::Quaterniond transformed_orientation;
            if (transformPoseBetweenFrames(
                    "head", position, orientation, frame_id, ee_frame_id_,
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
        if (!head_armed && has_robot_head_pose_)
        {
            const auto& p = robot_head_pose_.pose.position;
            const auto& q = robot_head_pose_.pose.orientation;
            arm_head_from(
                Eigen::Vector3d(p.x, p.y, p.z),
                Eigen::Quaterniond(q.w, q.x, q.y, q.z),
                robot_head_pose_.header.frame_id);
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

    void VRInputHandler::resetAbsoluteRamps()
    {
        left_arm_ramp_.reset();
        right_arm_ramp_.reset();
        body_ramp_.reset();
        head_ramp_.reset();
    }

    // ================= 底盘跟踪（位置外环 → /cmd_vel） =================

    void VRInputHandler::toggleBaseTracking()
    {
        if (!absolute_options_.base_follow_enable)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "🔘 [case 31] 忽略底盘跟踪开关：base_follow_enable=false");
            return;
        }
        if (!enabled_.load() || !isFullBodyMode() || resolvedFsmState() != 3)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "🔘 [case 31] 忽略底盘跟踪开关：要求 VR enabled、FULL_BODY、OCS2");
            return;
        }
        if (!whole_body_teleop_active_.load())
        {
            RCLCPP_WARN(node_->get_logger(),
                        "🔘 [case 31] 忽略底盘跟踪开关：请先用「左右握把+右摇杆按下」进入全身遥操作模式");
            return;
        }

        if (base_tracking_enabled_.load())
        {
            base_tracking_enabled_.store(false);
            resetBaseFollow();
            RCLCPP_INFO(node_->get_logger(),
                        "🔘 [case 31] 底盘跟踪已关闭（已发一次零速）");
        }
        else
        {
            base_tracking_enabled_.store(true);
            RCLCPP_INFO(node_->get_logger(),
                        "🔘 [case 31] 底盘跟踪已开启：骨盆水平位姿 → 位置外环 → /cmd_vel"
                        "（BASE_UNLOCKED 时冻结跟随并让路给 WBC）");
        }
    }

    bool VRInputHandler::baseFollowReady() const
    {
        if (!absolute_options_.base_follow_enable || !base_tracking_enabled_.load())
        {
            return false;
        }
        if (!whole_body_teleop_active_.load() || !enabled_.load() || !isFullBodyMode())
        {
            return false;
        }
        if (resolvedFsmState() != 3)
        {
            return false;
        }
        if (!has_body_pelvis_pose_ ||
            body_pelvis_stamp_ == std::chrono::steady_clock::time_point{})
        {
            return false;
        }
        const double age =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - body_pelvis_stamp_).count();
        return age <= std::max(0.0, absolute_options_.base_follow_data_timeout);
    }

    void VRInputHandler::captureBaseFollowBase()
    {
        base_follow_pelvis_ref_ = body_pelvis_position_;
        base_follow_yaw_ref_ = yawOfQuaternion(body_pelvis_orientation_);
        base_follow_yaw_offset_ = 0.0;
        base_follow_p_ref_.setZero();
        base_follow_v_cmd_.setZero();
        base_follow_w_cmd_ = 0.0;
        base_follow_v_out_.setZero();
        base_follow_w_out_ = 0.0;
        base_follow_last_tick_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(node_->get_logger(),
                    "🕹️🚗 底盘跟踪基准已建立（人体骨盆 yaw=%.1f°）",
                    base_follow_yaw_ref_ * 180.0 / M_PI);
    }

    void VRInputHandler::resetBaseFollow()
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
            publishChassisVelocity(geometry_msgs::msg::Twist());
        }
    }

    void VRInputHandler::updateBaseFollow()
    {
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

        // 人体骨盆位移：转到「接管时刻机器人本体系」F（人体 yaw 在接管时与机头对齐）
        const Eigen::Vector3d pelvis_delta = body_pelvis_position_ - base_follow_pelvis_ref_;
        const Eigen::Vector2d delta_F =
            yawRotation2d(-base_follow_yaw_ref_) * pelvis_delta.head<2>();

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
            wrapPi(yawOfQuaternion(body_pelvis_orientation_) - base_follow_yaw_ref_);
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

        if (wbcOwnsChassisVelocity())
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

        publishChassisVelocity(cmd_vel);
    }

    void VRInputHandler::vrLeftCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        if (!enabled_.load())
        {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "🕹️🕶️🕹️ VR left pose received, VR control ENABLED!");
            this->enable();
        }

        // 左话题接收的是左手柄数据
        left_ee_pose_ = poseMsgToMatrix(msg);
        // 提取原始（未缩放）左手柄位姿
        matrixToPosOri(left_ee_pose_, vr_left_position_raw_, left_orientation_);
        // 在原始位姿基础上应用缩放，得到用于控制的left_position_
        left_position_ = vr_left_position_raw_;
        left_position_ *= vr_pose_scale_;

        has_vr_left_pose_.store(true);
        const WbcToggleTarget controlledArm = mirror_mode_.load()
            ? WbcToggleTarget::RIGHT_ARM
            : WbcToggleTarget::LEFT_ARM;
        if (enabled_.load() &&
            isFullBodyMode() &&
            !prepareArmVrInput(controlledArm))
        {
            return;
        }

        if (enabled_.load())
        {
            // 根据镜像模式决定使用哪个臂的状态和参数
            bool is_mirror = mirror_mode_.load();
            bool arm_paused = is_mirror ? right_arm_paused_.load() : left_arm_paused_.load();

            // 全身「全部跟踪」开启时目标改由骨架绝对位姿发布，
            // 禁用基于手柄的相对增量路径，避免两个来源争抢 left/right_target。
            if (is_update_mode_.load() && !absolute_tracking_active_.load())
            {
                // 更新模式：基于差值计算pose并更新marker
                Eigen::Vector3d calculatedPos;
                Eigen::Quaterniond calculatedOri;

                if (is_mirror)
                {
                    // 镜像模式：左话题数据用于右臂
                    // 如果暂停，使用暂停时刻的VR位姿；否则使用当前VR位姿
                    Eigen::Vector3d vr_position = arm_paused ? paused_left_position_ : left_position_;
                    Eigen::Quaterniond vr_orientation = arm_paused ? paused_left_orientation_ : left_orientation_;
                    
                    // 将右摇杆累积偏移转换到世界坐标系（与手柄移动保持一致）
                    Eigen::Vector3d thumbstick_offset_world = vr_base_left_orientation_ * right_thumbstick_offset_;
                    // 应用右摇杆累积偏移到VR位姿（暂停时使用暂停时刻的位姿）
                    Eigen::Vector3d position_with_offset = vr_position + thumbstick_offset_world;

                    // 应用右摇杆累积Yaw旋转到VR姿态
                    Eigen::Quaterniond orientation_with_yaw = vr_orientation;
                    if (std::abs(right_thumbstick_yaw_offset_) > 0.001)
                    {
                        Eigen::AngleAxisd yawRotation(right_thumbstick_yaw_offset_, Eigen::Vector3d::UnitZ());
                        orientation_with_yaw = Eigen::Quaterniond(yawRotation) * vr_orientation;
                        orientation_with_yaw.normalize();
                    }

                    if (calculateAndPublishTarget(
                            "right",
                            position_with_offset,
                            orientation_with_yaw,
                            vr_base_left_position_,
                            vr_base_left_orientation_,
                            calculatedPos,
                            calculatedOri))
                    {
                        RCLCPP_DEBUG(
                            node_->get_logger(),
                            "🕹️ [Mirror] Left VR -> Right Arm target published");
                    }
                }
                else
                {
                    // 正常模式：左话题数据用于左臂
                    // 如果暂停，使用暂停时刻的VR位姿；否则使用当前VR位姿
                    Eigen::Vector3d vr_position = arm_paused ? paused_left_position_ : left_position_;
                    Eigen::Quaterniond vr_orientation = arm_paused ? paused_left_orientation_ : left_orientation_;
                    
                    // 将左摇杆累积偏移（局部坐标系）转换到世界坐标系（与手柄移动保持一致）
                    // 摇杆偏移量是在局部坐标系下的（相对于进入UPDATE时手柄的朝向）
                    Eigen::Vector3d thumbstick_offset_world = vr_base_left_orientation_ * left_thumbstick_offset_;
                    // 应用左摇杆累积偏移到VR位姿（暂停时使用暂停时刻的位姿）
                    Eigen::Vector3d position_with_offset = vr_position + thumbstick_offset_world;

                    // 应用左摇杆累积Yaw旋转到VR姿态
                    Eigen::Quaterniond orientation_with_yaw = vr_orientation;
                    if (std::abs(left_thumbstick_yaw_offset_) > 0.001)
                    {
                        Eigen::AngleAxisd yawRotation(left_thumbstick_yaw_offset_, Eigen::Vector3d::UnitZ());
                        orientation_with_yaw = Eigen::Quaterniond(yawRotation) * vr_orientation;
                        orientation_with_yaw.normalize();
                    }

                    if (calculateAndPublishTarget(
                            "left",
                            position_with_offset,
                            orientation_with_yaw,
                            vr_base_left_position_,
                            vr_base_left_orientation_,
                            calculatedPos,
                            calculatedOri))
                    {
                        RCLCPP_DEBUG(
                            node_->get_logger(),
                            "🕹️ Left VR -> Left Arm target published");
                    }
                }
            }
            else
            {
                // 存储模式：只存储VR pose，不更新marker
                // 不计算和发布目标位姿
            }
        }
    }

    void VRInputHandler::vrRightCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        // 右话题接收的是右手柄数据
        right_ee_pose_ = poseMsgToMatrix(msg);
        // 提取原始（未缩放）右手柄位姿
        matrixToPosOri(right_ee_pose_, vr_right_position_raw_, right_orientation_);
        // 在原始位姿基础上应用缩放，得到用于控制的right_position_
        right_position_ = vr_right_position_raw_;
        right_position_ *= vr_pose_scale_;

        has_vr_right_pose_.store(true);
        const WbcToggleTarget controlledArm = mirror_mode_.load()
            ? WbcToggleTarget::LEFT_ARM
            : WbcToggleTarget::RIGHT_ARM;
        if (enabled_.load() &&
            isFullBodyMode() &&
            !prepareArmVrInput(controlledArm))
        {
            return;
        }

        if (enabled_.load())
        {
            // 根据镜像模式决定使用哪个臂的状态和参数
            bool is_mirror = mirror_mode_.load();
            bool arm_paused = is_mirror ? left_arm_paused_.load() : right_arm_paused_.load();

            // 全身「全部跟踪」开启时目标改由骨架绝对位姿发布，
            // 禁用基于手柄的相对增量路径，避免两个来源争抢 left/right_target。
            if (is_update_mode_.load() && !absolute_tracking_active_.load())
            {
                // 更新模式：基于差值计算pose并更新marker
                Eigen::Vector3d calculatedPos;
                Eigen::Quaterniond calculatedOri;

                if (is_mirror)
                {
                    // 镜像模式：右话题数据用于左臂
                    // 如果暂停，使用暂停时刻的VR位姿；否则使用当前VR位姿
                    Eigen::Vector3d vr_position = arm_paused ? paused_right_position_ : right_position_;
                    Eigen::Quaterniond vr_orientation = arm_paused ? paused_right_orientation_ : right_orientation_;
                    
                    // 将左摇杆累积偏移（局部坐标系）转换到世界坐标系（与手柄移动保持一致）
                    // 摇杆偏移量是在局部坐标系下的（相对于进入UPDATE时手柄的朝向）
                    Eigen::Vector3d thumbstick_offset_world = vr_base_right_orientation_ * left_thumbstick_offset_;
                    // 应用左摇杆累积偏移到VR位姿（暂停时使用暂停时刻的位姿）
                    Eigen::Vector3d position_with_offset = vr_position + thumbstick_offset_world;

                    // 应用左摇杆累积Yaw旋转到VR姿态
                    Eigen::Quaterniond orientation_with_yaw = vr_orientation;
                    if (std::abs(left_thumbstick_yaw_offset_) > 0.001)
                    {
                        Eigen::AngleAxisd yawRotation(left_thumbstick_yaw_offset_, Eigen::Vector3d::UnitZ());
                        orientation_with_yaw = Eigen::Quaterniond(yawRotation) * vr_orientation;
                        orientation_with_yaw.normalize();
                    }

                    if (calculateAndPublishTarget(
                            "left",
                            position_with_offset,
                            orientation_with_yaw,
                            vr_base_right_position_,
                            vr_base_right_orientation_,
                            calculatedPos,
                            calculatedOri))
                    {
                        RCLCPP_DEBUG(
                            node_->get_logger(),
                            "🕹️ [Mirror] Right VR -> Left Arm target published");
                    }
                }
                else
                {
                    // 正常模式：右话题数据用于右臂
                    // 如果暂停，使用暂停时刻的VR位姿；否则使用当前VR位姿
                    Eigen::Vector3d vr_position = arm_paused ? paused_right_position_ : right_position_;
                    Eigen::Quaterniond vr_orientation = arm_paused ? paused_right_orientation_ : right_orientation_;
                    
                    // 将右摇杆累积偏移（局部坐标系）转换到世界坐标系（与手柄移动保持一致）
                    // 摇杆偏移量是在局部坐标系下的（相对于进入UPDATE时手柄的朝向）
                    Eigen::Vector3d thumbstick_offset_world = vr_base_right_orientation_ * right_thumbstick_offset_;
                    // 应用右摇杆累积偏移到VR位姿（暂停时使用暂停时刻的位姿）
                    Eigen::Vector3d position_with_offset = vr_position + thumbstick_offset_world;

                    // 应用右摇杆累积Yaw旋转到VR姿态
                    Eigen::Quaterniond orientation_with_yaw = vr_orientation;
                    if (std::abs(right_thumbstick_yaw_offset_) > 0.001)
                    {
                        Eigen::AngleAxisd yawRotation(right_thumbstick_yaw_offset_, Eigen::Vector3d::UnitZ());
                        orientation_with_yaw = Eigen::Quaterniond(yawRotation) * vr_orientation;
                        orientation_with_yaw.normalize();
                    }

                    if (calculateAndPublishTarget(
                            "right",
                            position_with_offset,
                            orientation_with_yaw,
                            vr_base_right_position_,
                            vr_base_right_orientation_,
                            calculatedPos,
                            calculatedOri))
                    {
                        RCLCPP_DEBUG(
                            node_->get_logger(),
                            "🕹️ Right VR -> Right Arm target published");
                    }
                }
            }
            else
            {
                // 存储模式：只存储VR pose，不更新marker
                // 不计算和发布目标位姿
            }
        }
    }

    void VRInputHandler::recordLastPublishedTarget(const std::string& armType,
                                                   const Eigen::Vector3d& position,
                                                   const Eigen::Quaterniond& orientation)
    {
        if (armType == "left")
        {
            has_last_published_left_target_ = true;
            last_published_left_position_ = position;
            last_published_left_orientation_ = orientation;
            last_published_left_orientation_.normalize();
            last_published_left_stamp_ = std::chrono::steady_clock::now();
        }
        else if (armType == "right")
        {
            has_last_published_right_target_ = true;
            last_published_right_position_ = position;
            last_published_right_orientation_ = orientation;
            last_published_right_orientation_.normalize();
            last_published_right_stamp_ = std::chrono::steady_clock::now();
        }
    }

    void VRInputHandler::clearLastPublishedTargets()
    {
        has_last_published_left_target_ = false;
        last_published_left_position_ = Eigen::Vector3d::Zero();
        last_published_left_orientation_ = Eigen::Quaterniond::Identity();
        last_published_left_stamp_ = std::chrono::steady_clock::time_point{};

        has_last_published_right_target_ = false;
        last_published_right_position_ = Eigen::Vector3d::Zero();
        last_published_right_orientation_ = Eigen::Quaterniond::Identity();
        last_published_right_stamp_ = std::chrono::steady_clock::time_point{};

        left_robot_base_valid_ = false;
        right_robot_base_valid_ = false;

        resetStaleCatchUpRamp();
        // 坐标系/拓扑发生变化：旧锚点已经不具可比性，交还直通语义
        resetAbsoluteRamps();
    }

    bool VRInputHandler::transformPoseBetweenFrames(
        const std::string& armType,
        const Eigen::Vector3d& inputPosition,
        const Eigen::Quaterniond& inputOrientation,
        const std::string& sourceFrame,
        const std::string& targetFrame,
        Eigen::Vector3d& outputPosition,
        Eigen::Quaterniond& outputOrientation)
    {
        if (sourceFrame.empty() || targetFrame.empty())
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "🕹️ %s VR pose transform skipped: source='%s', target='%s'",
                armType.c_str(), sourceFrame.c_str(), targetFrame.c_str());
            return false;
        }

        if (!inputPosition.allFinite() ||
            !inputOrientation.coeffs().allFinite() ||
            inputOrientation.norm() < 1e-9)
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "🕹️ %s VR pose transform skipped: invalid pose in frame '%s'",
                armType.c_str(), sourceFrame.c_str());
            return false;
        }

        const Eigen::Quaterniond normalizedInput = inputOrientation.normalized();
        if (sourceFrame == targetFrame)
        {
            outputPosition = inputPosition;
            outputOrientation = normalizedInput;
            return true;
        }

        try
        {
            geometry_msgs::msg::PoseStamped input;
            input.header.frame_id = sourceFrame;
            input.header.stamp = node_->now();
            input.pose.position.x = inputPosition.x();
            input.pose.position.y = inputPosition.y();
            input.pose.position.z = inputPosition.z();
            input.pose.orientation.x = normalizedInput.x();
            input.pose.orientation.y = normalizedInput.y();
            input.pose.orientation.z = normalizedInput.z();
            input.pose.orientation.w = normalizedInput.w();

            const auto transform = tf_buffer_->lookupTransform(
                targetFrame, sourceFrame, tf2::TimePointZero);
            geometry_msgs::msg::PoseStamped transformed;
            tf2::doTransform(input, transformed, transform);

            Eigen::Vector3d nextPosition(
                transformed.pose.position.x,
                transformed.pose.position.y,
                transformed.pose.position.z);
            Eigen::Quaterniond nextOrientation(
                transformed.pose.orientation.w,
                transformed.pose.orientation.x,
                transformed.pose.orientation.y,
                transformed.pose.orientation.z);
            if (!nextPosition.allFinite() ||
                !nextOrientation.coeffs().allFinite() ||
                nextOrientation.norm() < 1e-9)
            {
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 2000,
                    "🕹️ %s VR pose transform produced invalid result: %s -> %s",
                    armType.c_str(), sourceFrame.c_str(), targetFrame.c_str());
                return false;
            }

            outputPosition = nextPosition;
            outputOrientation = nextOrientation.normalized();
            return true;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "🕹️ %s VR pose transform unavailable: %s -> %s: %s",
                armType.c_str(), sourceFrame.c_str(), targetFrame.c_str(), ex.what());
            return false;
        }
    }

    bool VRInputHandler::setRobotBase(const std::string& armType, bool preferLastCommand)
    {
        const bool isLeft = armType == "left";
        if (!isLeft && armType != "right")
        {
            return false;
        }
        bool& valid = isLeft ? left_robot_base_valid_ : right_robot_base_valid_;
        valid = false;
        const bool useLast = preferLastCommand && (isLeft
            ? has_last_published_left_target_ : has_last_published_right_target_);
        if (!ee_frame_id_initialized_ || (!useLast && !(isLeft
            ? has_robot_current_left_pose_.load() : has_robot_current_right_pose_.load())))
        {
            return false;
        }

        // Query F -> C once per rebase. Only its rotation maps VR increments.
        Eigen::Vector3d unusedTranslation;
        Eigen::Quaterniond rotation;
        if (!transformPoseBetweenFrames(armType, Eigen::Vector3d::Zero(),
                Eigen::Quaterniond::Identity(), vr_follow_frame_, ee_frame_id_,
                unusedTranslation, rotation))
        {
            return false;
        }
        const Eigen::Vector3d position = useLast
            ? (isLeft ? last_published_left_position_ : last_published_right_position_)
            : (isLeft ? robot_current_left_position_ : robot_current_right_position_);
        const Eigen::Quaterniond orientation = useLast
            ? (isLeft ? last_published_left_orientation_ : last_published_right_orientation_)
            : (isLeft ? robot_current_left_orientation_ : robot_current_right_orientation_);
        if (!position.allFinite() || !orientation.coeffs().allFinite() || orientation.norm() < 1e-9)
        {
            return false;
        }
        (isLeft ? robot_base_left_position_ : robot_base_right_position_) = position;
        (isLeft ? robot_base_left_orientation_ : robot_base_right_orientation_) = orientation.normalized();
        (isLeft ? left_follow_rotation_ : right_follow_rotation_) = rotation;
        resetStaleCatchUpRamp(isLeft);
        valid = true;
        return true;
    }

    bool VRInputHandler::calculateAndPublishTarget(
        const std::string& armType,
        const Eigen::Vector3d& vrCurrentPosition,
        const Eigen::Quaterniond& vrCurrentOrientation,
        const Eigen::Vector3d& vrBasePosition,
        const Eigen::Quaterniond& vrBaseOrientation,
        Eigen::Vector3d& calculatedPosition,
        Eigen::Quaterniond& calculatedOrientation)
    {
        const bool isLeft = armType == "left";
        const bool isRight = armType == "right";
        if (!isLeft && !isRight)
        {
            return false;
        }

        bool& valid = isLeft ? left_robot_base_valid_ : right_robot_base_valid_;
        if (!valid)
        {
            if (setRobotBase(armType, /*preferLastCommand=*/true))
            {
                // TF may have been unavailable at resume. Start the VR delta at
                // successful rebase, not at an old sample collected while waiting.
                const bool useLeftVr = mirror_mode_.load() ? !isLeft : isLeft;
                const bool paused = isLeft ? left_arm_paused_.load() : right_arm_paused_.load();
                (useLeftVr ? vr_base_left_position_ : vr_base_right_position_) = useLeftVr
                    ? (paused ? paused_left_position_ : left_position_)
                    : (paused ? paused_right_position_ : right_position_);
                (useLeftVr ? vr_base_left_orientation_ : vr_base_right_orientation_) = useLeftVr
                    ? (paused ? paused_left_orientation_ : left_orientation_)
                    : (paused ? paused_right_orientation_ : right_orientation_);
                (isLeft ? left_thumbstick_offset_ : right_thumbstick_offset_).setZero();
                (isLeft ? left_thumbstick_yaw_offset_ : right_thumbstick_yaw_offset_) = 0.0;
            }
            // The caller already applied the old offsets; use the next fresh sample.
            return false;
        }

        const Eigen::Vector3d& robotBasePosition = isLeft
            ? robot_base_left_position_
            : robot_base_right_position_;
        const Eigen::Quaterniond& robotBaseOrientation = isLeft
            ? robot_base_left_orientation_
            : robot_base_right_orientation_;

        calculatePoseFromDifference(
            vrCurrentPosition,
            vrCurrentOrientation,
            vrBasePosition,
            vrBaseOrientation,
            Eigen::Vector3d::Zero(),
            Eigen::Quaterniond::Identity(),
            calculatedPosition,
            calculatedOrientation);
        // The local VR delta is interpreted in F; anchors and outputs remain in C.
        const auto& rotation = isLeft ? left_follow_rotation_ : right_follow_rotation_;
        calculatedPosition = robotBasePosition + rotation * calculatedPosition;
        calculatedOrientation = (rotation * calculatedOrientation * rotation.conjugate() *
                                 robotBaseOrientation).normalized();
        return publishTargetPoseDirect(
            armType, calculatedPosition, calculatedOrientation);
    }

    bool VRInputHandler::isBimanualCoupled() const
    {
        using WbcState =
            arms_ros2_control_msgs::msg::WbcCurrentState;

        return target_manager_ &&
               target_manager_->getCurrentBimanualState() ==
                   WbcState::BIMANUAL_COUPLED;
    }

    void VRInputHandler::rebaseRightArmVrControl()
    {
        const bool paused = right_arm_paused_.load();
        resetStaleCatchUpRamp(false);

        if (mirror_mode_.load())
        {
            vr_base_left_position_ =
                paused ? paused_left_position_ : left_position_;
            vr_base_left_orientation_ =
                paused ? paused_left_orientation_ : left_orientation_;
        }
        else
        {
            vr_base_right_position_ =
                paused ? paused_right_position_ : right_position_;
            vr_base_right_orientation_ =
                paused ? paused_right_orientation_ : right_orientation_;
        }

        right_thumbstick_offset_ = Eigen::Vector3d::Zero();
        right_thumbstick_yaw_offset_ = 0.0;

        has_last_published_right_target_ = false;
        last_published_right_stamp_ = std::chrono::steady_clock::time_point{};
        right_robot_base_valid_ = false;
        // 解耦后的目标是实测位姿，重新渐入只会把已经就位的目标又拉长一次
        right_arm_ramp_.reset();
        const bool rightBaseReady =
            setRobotBase("right", /*preferLastCommand=*/true);

        prev_calculated_right_position_ = robot_current_right_position_;
        prev_calculated_right_orientation_ =
            robot_current_right_orientation_.normalized();

        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️ Right VR rebase after decoupling: %s",
            rightBaseReady ? "ready" : "waiting for TF");
    }

    bool VRInputHandler::publishTargetPoseDirect(
        const std::string& armType,
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation)
    {
        const bool bimanualCoupled = isBimanualCoupled();
        if (bimanualCoupled)
        {
            right_vr_target_suppressed_by_bimanual_ = true;
        }

        if (armType == "right" &&
            right_vr_target_suppressed_by_bimanual_ &&
            !bimanualCoupled)
        {
            rebaseRightArmVrControl();
            right_vr_target_suppressed_by_bimanual_ = false;
            RCLCPP_INFO(
                node_->get_logger(),
                "🕹️ 双臂已解耦，右臂 VR 控制基准已重建");
            return false;
        }

        const bool isLeft = armType == "left";
        const bool isRight = armType == "right";
        if ((!isLeft && !isRight) ||
            (isLeft && !pub_left_target_) ||
            (isRight && !pub_right_target_))
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "🕹️ Invalid armType or publisher not initialized: %s",
                armType.c_str());
            return false;
        }

        // 上游 XR 位姿冻结后会一帧补齐整段位移，先把追赶量摊到随后若干帧。
        // 在控制器坐标系中平滑，缓存保存实际发布的目标。
        Eigen::Vector3d rampedPosition = position;
        Eigen::Quaterniond rampedOrientation = orientation;
        applyStaleCatchUpRamp(isLeft, rampedPosition, rampedOrientation);

        Eigen::Vector3d publishPosition = rampedPosition;
        Eigen::Quaterniond publishOrientation = rampedOrientation;
        // Target and catch-up ramp are already expressed in the controller frame.
        if (!ee_frame_id_initialized_)
        {
            return false;
        }

        Eigen::Vector3d& previousPosition = isLeft
            ? prev_calculated_left_position_
            : prev_calculated_right_position_;
        Eigen::Quaterniond& previousOrientation = isLeft
            ? prev_calculated_left_orientation_
            : prev_calculated_right_orientation_;
        // Temporarily disable pose-change threshold filtering so every valid XR pose
        // callback publishes a target. Keep the original gate for quick A/B rollback.
        /*
        // Coupled: keep left_target heartbeating even when the left controller is still,
        // otherwise the 0.2s leader timeout lets right_target steal and inverse-sync left.
        const bool leftCoupledHeartbeat = bimanualCoupled && isLeft;
        if (!leftCoupledHeartbeat &&
            !hasPoseChanged(
                publishPosition,
                publishOrientation,
                previousPosition,
                previousOrientation))
        {
            return false;
        }
        */

        geometry_msgs::msg::Pose pose;
        pose.position.x = publishPosition.x();
        pose.position.y = publishPosition.y();
        pose.position.z = publishPosition.z();
        pose.orientation.x = publishOrientation.x();
        pose.orientation.y = publishOrientation.y();
        pose.orientation.z = publishOrientation.z();
        pose.orientation.w = publishOrientation.w();

        if (isLeft)
        {
            pub_left_target_->publish(pose);
        }
        else
        {
            pub_right_target_->publish(pose);
        }

        recordLastPublishedTarget(armType, rampedPosition, rampedOrientation);
        previousPosition = publishPosition;
        previousOrientation = publishOrientation.normalized();
        return true;
    }

    Eigen::Matrix4d VRInputHandler::poseMsgToMatrix(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose(0, 3) = msg->position.x;
        pose(1, 3) = msg->position.y;
        pose(2, 3) = msg->position.z;

        Eigen::Quaterniond q(
            msg->orientation.w,
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z);
        Eigen::Matrix3d rot = q.normalized().toRotationMatrix();
        pose.block<3, 3>(0, 0) = rot;

        return pose;
    }


    void VRInputHandler::matrixToPosOri(const Eigen::Matrix4d& matrix,
                                        Eigen::Vector3d& position,
                                        Eigen::Quaterniond& orientation)
    {
        position = matrix.block<3, 1>(0, 3);
        Eigen::Matrix3d rot = matrix.block<3, 3>(0, 0);
        orientation = Eigen::Quaterniond(rot);
    }

    bool VRInputHandler::hasPoseChanged(const Eigen::Vector3d& currentPos,
                                        const Eigen::Quaterniond& currentOri,
                                        const Eigen::Vector3d& prevPos,
                                        const Eigen::Quaterniond& prevOri)
    {
        // 检查位置变化
        double positionDiff = (currentPos - prevPos).norm();
        if (positionDiff > POSITION_THRESHOLD)
        {
            return true;
        }

        // 使用四元数角度差检查方向变化
        double orientationDiff = std::abs(currentOri.angularDistance(prevOri));
        if (orientationDiff > ORIENTATION_THRESHOLD)
        {
            return true;
        }

        return false;
    }

    void VRInputHandler::applyStaleCatchUpRamp(bool isLeft,
                                               Eigen::Vector3d& position,
                                               Eigen::Quaterniond& orientation)
    {
        StaleCatchUpRamp& ramp = isLeft ? left_stale_ramp_ : right_stale_ramp_;

        const Eigen::Vector3d inputPosition = position;
        const Eigen::Quaterniond inputOrientation = orientation;

        if (!ramp.has_previous_input)
        {
            ramp.has_previous_input = true;
            ramp.previous_input_position = inputPosition;
            ramp.previous_input_orientation = inputOrientation;
            ramp.output_position = inputPosition;
            ramp.output_orientation = inputOrientation;
            return;
        }

        // 逐位比较：上游冻结时 xr_target_node 重发同一帧，而本文件的换算是确定性的，
        // 于是算出的目标也逐位相同；摇杆偏移等真实指令运动不会命中这一条。
        const bool identical =
            inputPosition == ramp.previous_input_position &&
            inputOrientation.coeffs() == ramp.previous_input_orientation.coeffs();

        ramp.previous_input_position = inputPosition;
        ramp.previous_input_orientation = inputOrientation;

        bool armedThisFrame = false;
        if (identical)
        {
            ++ramp.frozen_frames;
        }
        else
        {
            if (ramp.frozen_frames >= STALE_MIN_FROZEN_FRAMES)
            {
                // 解冻帧：缺口 = 当前输入 - 当前输出，摊开帧数 N = 冻结帧数。
                // 缺口是现场量出来的，因此上一次斜坡没还完的部分会自动计入。
                const int spread = ramp.frozen_frames;
                const double keep =
                    static_cast<double>(spread - 1) / static_cast<double>(spread);

                Eigen::Quaterniond aligned = inputOrientation;
                if (aligned.dot(ramp.output_orientation) < 0.0)
                {
                    aligned.coeffs() = -aligned.coeffs();
                }

                const Eigen::Vector3d gap = inputPosition - ramp.output_position;
                ramp.residual_position = gap * keep;
                ramp.residual_orientation = Eigen::Quaterniond::Identity().slerp(
                    keep,
                    (ramp.output_orientation.conjugate() * aligned).normalized());
                ramp.remaining_frames = spread - 1;
                armedThisFrame = true;

                RCLCPP_DEBUG(
                    node_->get_logger(),
                    "🕹️ [%s] XR 冻结 %d 帧，追赶 %.1fmm 分摊到 %d 帧",
                    isLeft ? "left" : "right",
                    spread,
                    gap.norm() * 1000.0,
                    spread);
            }
            ramp.frozen_frames = 0;
        }

        if (!armedThisFrame && ramp.remaining_frames > 0)
        {
            // 等额递减：残差 <- 残差 * (剩余-1)/剩余，最后一帧精确归零。
            const double keep = static_cast<double>(ramp.remaining_frames - 1) /
                                static_cast<double>(ramp.remaining_frames);
            ramp.residual_position *= keep;
            ramp.residual_orientation =
                Eigen::Quaterniond::Identity().slerp(keep, ramp.residual_orientation);
            --ramp.remaining_frames;
            if (ramp.remaining_frames == 0)
            {
                ramp.residual_position.setZero();
                ramp.residual_orientation.setIdentity();
            }
        }

        if (ramp.remaining_frames > 0)
        {
            Eigen::Quaterniond aligned = inputOrientation;
            if (aligned.dot(ramp.output_orientation) < 0.0)
            {
                aligned.coeffs() = -aligned.coeffs();
            }
            position = inputPosition - ramp.residual_position;
            orientation =
                (aligned * ramp.residual_orientation.conjugate()).normalized();
        }
        // 无残差时不改写入参，正常运动逐位不受影响。

        ramp.output_position = position;
        ramp.output_orientation = orientation;
    }

    void VRInputHandler::resetStaleCatchUpRamp(bool isLeft)
    {
        (isLeft ? left_stale_ramp_ : right_stale_ramp_) = StaleCatchUpRamp{};
    }

    void VRInputHandler::resetStaleCatchUpRamp()
    {
        left_stale_ramp_ = StaleCatchUpRamp{};
        right_stale_ramp_ = StaleCatchUpRamp{};
    }

    void VRInputHandler::calculatePoseFromDifference(const Eigen::Vector3d& vrCurrentPos,
                                                     const Eigen::Quaterniond& vrCurrentOri,
                                                     const Eigen::Vector3d& vrBasePos,
                                                     const Eigen::Quaterniond& vrBaseOri,
                                                     const Eigen::Vector3d& robotBasePos,
                                                     const Eigen::Quaterniond& robotBaseOri,
                                                     Eigen::Vector3d& resultPos,
                                                     Eigen::Quaterniond& resultOri)
    {
        // 1. 计算VR位置在世界坐标系下的差值
        Eigen::Vector3d vrPosDiff_world = vrCurrentPos - vrBasePos;

        // 2. 将位置差值转换到VR手柄基准姿态的局部坐标系
        //    这样用户转身后，手柄"向前"移动仍然相对于进入UPDATE时手柄的朝向
        Eigen::Vector3d vrPosDiff_local = vrBaseOri.inverse() * vrPosDiff_world;

        // 3. 计算VR姿态差值（相对旋转）
        Eigen::Quaterniond vrOriDiff = vrBaseOri.inverse() * vrCurrentOri;

        // 4. 镜像模式处理（在局部坐标系下应用）
        if (mirror_mode_.load())
        {
            const auto profile = static_cast<MirrorModeProfile>(mirror_mode_profile_.load());

            if (profile == MirrorModeProfile::MODE_A)
            {
                // MODE A：保留为空白，供调用方自行填入自定义镜像处理逻辑
                // 这里不做额外处理，保持原始差值，避免擅自更改用户定义行为。
                vrPosDiff_local.y() = -vrPosDiff_local.y(); // 前后翻转

                vrOriDiff.x() = -vrOriDiff.x(); // 翻转X分量
                vrOriDiff.z() = -vrOriDiff.z(); // 翻转Z分量
            }
            else
            {
                // MODE B：恢复原来的镜像处理逻辑
                // 位置翻转（局部坐标系）
                vrPosDiff_local.x() = -vrPosDiff_local.x(); // 左右翻转
                vrPosDiff_local.y() = -vrPosDiff_local.y(); // 前后翻转
                // vrPosDiff_local.z() 保持不变（上下不翻转）

                // 旋转翻转（面对面镜像）
                vrOriDiff.y() = -vrOriDiff.y(); // 翻转Y分量
                vrOriDiff.x() = -vrOriDiff.x(); // 翻转X分量
            }

            vrOriDiff.normalize(); // 重新归一化
        }

        // 5. 将局部坐标系的位置差值转换回机器人坐标系并应用
        //    按照机器人手臂的朝向应用位移，保持相对运动一致性
        resultPos = robotBasePos + vrPosDiff_local;

        // 6. 应用姿态差值（保持原有逻辑）
        resultOri = vrOriDiff * robotBaseOri;

        // 7. 归一化四元数以避免漂移
        resultOri.normalize();
    }

    void VRInputHandler::thumbstickAxesCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 存储左摇杆轴值（从 linear.x/y 读取）
        left_thumbstick_axes_.x() = msg->linear.x;
        left_thumbstick_axes_.y() = msg->linear.y;
        // 方向抑制只清零 left_thumbstick_axes_，这里先留一份物理真值给 pending 判回中
        left_thumbstick_axes_raw_ = left_thumbstick_axes_;

        // 存储右摇杆轴值（从 angular.x/y 读取）
        right_thumbstick_axes_.x() = msg->angular.x;
        right_thumbstick_axes_.y() = msg->angular.y;
        right_thumbstick_axes_raw_ = right_thumbstick_axes_;

        // 读取左右握把实时状态（xr_target_node 复用 linear.z / angular.z 携带，1.0=按下）
        // 短按（松开且未超时、未被组合键消费）才切换摇杆 XY / Z+Yaw；长按留给组合键。
        const bool leftGripNow = msg->linear.z > 0.5;
        const bool rightGripNow = msg->angular.z > 0.5;
        const bool leftGripWas = left_grip_active_.load();
        const bool rightGripWas = right_grip_active_.load();
        left_grip_active_.store(leftGripNow);
        right_grip_active_.store(rightGripNow);
        if (leftGripNow != leftGripWas)
        {
            handleGripActiveEdge(true, leftGripNow);
        }
        if (rightGripNow != rightGripWas)
        {
            handleGripActiveEdge(false, rightGripNow);
        }

        constexpr double direction_reset_threshold = 0.3;

        if (!isFullBodyMode())
        {
            // 拓扑离开 FULL_BODY 时不保留组合抑制状态。
            left_grip_direction_suppressed_.store(false);
            right_grip_direction_suppressed_.store(false);
            right_wbc_toggle_request_pending_.store(false);
            requested_wbc_toggle_target_.store(WbcToggleTarget::NONE);
        }
        else
        {
            if (left_grip_active_.load())
            {
                left_grip_direction_suppressed_.store(true);
            }

            const bool left_thumbstick_centered =
                std::abs(left_thumbstick_axes_.x()) <= direction_reset_threshold &&
                std::abs(left_thumbstick_axes_.y()) <= direction_reset_threshold;

            if (left_grip_direction_suppressed_.load() &&
                !left_grip_active_.load() &&
                left_thumbstick_centered)
            {
                left_grip_direction_suppressed_.store(false);
            }

            if (left_grip_direction_suppressed_.load())
            {
                // 防止组合输入以及“先松握把、后松摇杆”误驱动末端或底盘。
                left_thumbstick_axes_.setZero();
            }

            if (right_grip_active_.load())
            {
                right_grip_direction_suppressed_.store(true);
            }

            const bool rightThumbstickCentered =
                std::abs(right_thumbstick_axes_raw_.x()) <=
                    direction_reset_threshold &&
                std::abs(right_thumbstick_axes_raw_.y()) <=
                    direction_reset_threshold;

            if (right_grip_direction_suppressed_.load() &&
                !right_grip_active_.load() &&
                rightThumbstickCentered)
            {
                right_grip_direction_suppressed_.store(false);
            }

            if (right_grip_direction_suppressed_.load())
            {
                right_thumbstick_axes_.setZero();
            }
        }

        updateYbModeConversionState();
        if (updateRightWbcToggleRequestState())
        {
            right_thumbstick_axes_.setZero();
        }

        // 底盘控制模式分流：进入该模式后摇杆不再驱动末端，改为驱动底盘/腰部
        if (chassis_mode_.load())
        {
            processChassisAxes();
            return;
        }

        // 全身遥操作模式：双臂末端与腰部目标全部由骨架绝对映射给出，
        // 因此**左右摇杆都不得再产生末端/腰部微调** ——
        // 否则拨右摇杆会累加 right_thumbstick_offset_，而绝对映射会把该偏移叠加到右臂目标上，
        // 表现为"拨右摇杆就推着右臂走"。
        // 注意：底盘跟踪用长按右摇杆（事件 31）开关、不读摇杆轴值；握把+摇杆方向组合键
        // （21–28）由事件链消费且轴值已被抑制清零，因此这里直接返回是安全的。
        if (whole_body_teleop_active_.load())
        {
            return;
        }

        // 身体模式请求未确认或摇杆未回中时，左摇杆既不给腰部也不给左臂
        if (updateBodyModeRequestState())
        {
            processRightThumbstickAxes();
            return;
        }

        // 跟随模式已确认：左摇杆改为控制腰部 marker，右摇杆保持右臂
        if (isBodyTrackingActive())
        {
            processBodyThumbstickAxes();
            processRightThumbstickAxes();
            return;
        }

        // 离开跟随后腰部控制平面回到默认 XY
        body_thumbstick_z_yaw_mode_.store(false);

        // 处理左摇杆轴值（末端控制）
        processLeftThumbstickAxes();
        
        // 处理右摇杆轴值（末端控制）
        processRightThumbstickAxes();
    }

    void VRInputHandler::triggerValuesCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        const double left_trigger_pull = std::clamp(msg->linear.x, 0.0, 1.0);
        const double right_trigger_pull = std::clamp(msg->angular.x, 0.0, 1.0);
        const double left_hand_percent = 1.0 - left_trigger_pull;
        const double right_hand_percent = 1.0 - right_trigger_pull;

        auto publishIfPercent = [&](bool is_left_trigger, double hand_percent) {
            std::string controller_name;
            bool is_target_left_arm = true;
            resolveTriggerTarget(is_left_trigger, controller_name, is_target_left_arm);
            if (!isGripperPercentMode(is_target_left_arm))
            {
                return;
            }
            auto& armed = is_target_left_arm ? left_gripper_percent_armed_
                                             : right_gripper_percent_armed_;
            if (!armed.load())
            {
                const char* side = is_target_left_arm ? "左" : "右";
                auto& saw_release = is_target_left_arm ? left_gripper_percent_saw_release_
                                                       : right_gripper_percent_saw_release_;

                // ① 组合键的收尾释放期：13/14 必须扣着扳机才能触发，松开它只是
                //    收尾动作，不是夹爪指令，所以这一段一条指令都不发。
                if (!saw_release.load())
                {
                    if (hand_percent >= 0.9)
                    {
                        saw_release.store(true);
                    }
                    RCLCPP_INFO_THROTTLE(
                        node_->get_logger(), *node_->get_clock(), 2000,
                        "🕹️ [%s夹爪] 已切到比例控制，等扳机松开（当前保持原位）", side);
                    return;
                }

                // ② 已松开过，但扳机还没扣回到夹爪所在位置：继续保持原位。
                //    这一段就是"松开扳机夹爪不动，直到再次收到夹爪指令"。
                const double hold = is_target_left_arm ? left_gripper_percent_hold_.load()
                                                       : right_gripper_percent_hold_.load();
                if (hand_percent > hold)
                {
                    RCLCPP_INFO_THROTTLE(
                        node_->get_logger(), *node_->get_clock(), 2000,
                        "🕹️ [%s夹爪] 保持在 %.2f，扣扳机到该位置（当前 %.2f）才接管",
                        side, hold, hand_percent);
                    return;
                }

                // ③ 扳机已走到夹爪当前位置：从这里 1:1 接管，接管点即当前位置，无跳变。
                armed.store(true);
                RCLCPP_INFO(node_->get_logger(),
                            "🕹️ [%s夹爪] 比例控制已接管（接管点 %.2f）", side, hand_percent);
            }
            publishGripperPercent(controller_name, hand_percent);
        };

        publishIfPercent(true, left_hand_percent);
        publishIfPercent(false, right_hand_percent);
    }
    
    void VRInputHandler::processLeftThumbstickAxes()
    {
        // 在UPDATE模式下累积摇杆输入
        if (enabled_.load() && is_update_mode_.load())
        {
            const WbcToggleTarget controlledArm = mirror_mode_.load()
                ? WbcToggleTarget::RIGHT_ARM
                : WbcToggleTarget::LEFT_ARM;
            if (isArmVrInputSuppressed(controlledArm))
            {
                return;
            }

            // 根据镜像模式决定使用哪个臂的参数
            bool is_mirror = mirror_mode_.load();

            if (is_mirror)
            {
                // 镜像模式：左话题数据用于右臂
                if (right_grip_mode_.load())
                {
                    // 高度旋转模式：Y轴→Z高度，X轴→Yaw旋转
                    double delta_z = left_thumbstick_axes_.y() * vr_thumbstick_linear_scale_;
                    double delta_yaw = left_thumbstick_axes_.x() * vr_thumbstick_angular_scale_;

                    // 累积Z轴偏移和Yaw旋转（使用右臂参数）
                    right_thumbstick_offset_.z() -= delta_z;
                    right_thumbstick_yaw_offset_ -= delta_yaw;

                    RCLCPP_DEBUG(node_->get_logger(),
                                 "🕹️ [Mirror] Left thumbstick → Right arm (Z+Yaw): Y=%.3f→ΔZ=%.4f, X=%.3f→ΔYaw=%.4f",
                                 left_thumbstick_axes_.y(), delta_z,
                                 left_thumbstick_axes_.x(), delta_yaw);
                }
                else
                {
                    // XY平移模式：Y轴→前后(X)，X轴→左右(Y)
                    double delta_x = left_thumbstick_axes_.y() * vr_thumbstick_linear_scale_;
                    double delta_y = left_thumbstick_axes_.x() * vr_thumbstick_linear_scale_;

                    // 累积XY偏移（使用右臂参数）
                    right_thumbstick_offset_.x() -= delta_x;
                    right_thumbstick_offset_.y() -= delta_y;

                    RCLCPP_DEBUG(node_->get_logger(),
                                 "🕹️ [Mirror] Left thumbstick → Right arm (XY): Y=%.3f→ΔX=%.4f, X=%.3f→ΔY=%.4f",
                                 left_thumbstick_axes_.y(), delta_x,
                                 left_thumbstick_axes_.x(), delta_y);
                }
            }
            else
            {
                // 正常模式：左话题数据用于左臂
                if (left_grip_mode_.load())
                {
                    // 高度旋转模式：Y轴→Z高度，X轴→Yaw旋转
                    double delta_z = left_thumbstick_axes_.y() * vr_thumbstick_linear_scale_;
                    double delta_yaw = left_thumbstick_axes_.x() * vr_thumbstick_angular_scale_;

                    // 累积Z轴偏移和Yaw旋转
                    left_thumbstick_offset_.z() -= delta_z;
                    left_thumbstick_yaw_offset_ -= delta_yaw;

                    RCLCPP_DEBUG(node_->get_logger(),
                                 "🕹️ Left thumbstick (Z+Yaw): Y=%.3f→ΔZ=%.4f, X=%.3f→ΔYaw=%.4f (累积Yaw=%.4f)",
                                 left_thumbstick_axes_.y(), delta_z,
                                 left_thumbstick_axes_.x(), delta_yaw,
                                 left_thumbstick_yaw_offset_);
                }
                else
                {
                    // XY平移模式：Y轴→前后(X)，X轴→左右(Y)
                    double delta_x = left_thumbstick_axes_.y() * vr_thumbstick_linear_scale_;
                    double delta_y = left_thumbstick_axes_.x() * vr_thumbstick_linear_scale_;

                    // 累积XY偏移
                    left_thumbstick_offset_.x() -= delta_x;
                    left_thumbstick_offset_.y() -= delta_y;

                    RCLCPP_DEBUG(node_->get_logger(), "🕹️ Left thumbstick (XY): Y=%.3f→ΔX=%.4f, X=%.3f→ΔY=%.4f",
                                 left_thumbstick_axes_.y(), delta_x,
                                 left_thumbstick_axes_.x(), delta_y);
                }
            }
        }
    }

    void VRInputHandler::processRightThumbstickAxes()
    {
        // 在UPDATE模式下累积摇杆输入
        if (enabled_.load() && is_update_mode_.load())
        {
            const WbcToggleTarget controlledArm = mirror_mode_.load()
                ? WbcToggleTarget::LEFT_ARM
                : WbcToggleTarget::RIGHT_ARM;
            if (isArmVrInputSuppressed(controlledArm))
            {
                return;
            }

            // 根据镜像模式决定使用哪个臂的参数
            bool is_mirror = mirror_mode_.load();

            if (is_mirror)
            {
                // 镜像模式：右话题数据用于左臂
                if (left_grip_mode_.load())
                {
                    // 高度旋转模式：Y轴→Z高度，X轴→Yaw旋转
                    double delta_z = right_thumbstick_axes_.y() * vr_thumbstick_linear_scale_;
                    double delta_yaw = right_thumbstick_axes_.x() * vr_thumbstick_angular_scale_;

                    // 累积Z轴偏移和Yaw旋转（使用左臂参数）
                    left_thumbstick_offset_.z() -= delta_z;
                    left_thumbstick_yaw_offset_ -= delta_yaw;

                    RCLCPP_DEBUG(node_->get_logger(),
                                 "🕹️ [Mirror] Right thumbstick → Left arm (Z+Yaw): Y=%.3f→ΔZ=%.4f, X=%.3f→ΔYaw=%.4f",
                                 right_thumbstick_axes_.y(), delta_z,
                                 right_thumbstick_axes_.x(), delta_yaw);
                }
                else
                {
                    // XY平移模式：Y轴→前后(X)，X轴→左右(Y)
                    double delta_x = right_thumbstick_axes_.y() * vr_thumbstick_linear_scale_;
                    double delta_y = right_thumbstick_axes_.x() * vr_thumbstick_linear_scale_;

                    // 累积XY偏移（使用左臂参数）
                    left_thumbstick_offset_.x() -= delta_x;
                    left_thumbstick_offset_.y() -= delta_y;

                    RCLCPP_DEBUG(node_->get_logger(),
                                 "🕹️ [Mirror] Right thumbstick → Left arm (XY): Y=%.3f→ΔX=%.4f, X=%.3f→ΔY=%.4f",
                                 right_thumbstick_axes_.y(), delta_x,
                                 right_thumbstick_axes_.x(), delta_y);
                }
            }
            else
            {
                // 正常模式：右话题数据用于右臂
                if (right_grip_mode_.load())
                {
                    // 高度旋转模式：Y轴→Z高度，X轴→Yaw旋转
                    double delta_z = right_thumbstick_axes_.y() * vr_thumbstick_linear_scale_;
                    double delta_yaw = right_thumbstick_axes_.x() * vr_thumbstick_angular_scale_;

                    // 累积Z轴偏移和Yaw旋转
                    right_thumbstick_offset_.z() -= delta_z;
                    right_thumbstick_yaw_offset_ -= delta_yaw;

                    RCLCPP_DEBUG(node_->get_logger(),
                                 "🕹️ Right thumbstick (Z+Yaw): Y=%.3f→ΔZ=%.4f, X=%.3f→ΔYaw=%.4f (累积Yaw=%.4f)",
                                 right_thumbstick_axes_.y(), delta_z,
                                 right_thumbstick_axes_.x(), delta_yaw,
                                 right_thumbstick_yaw_offset_);
                }
                else
                {
                    // XY平移模式：Y轴→前后(X)，X轴→左右(Y)
                    double delta_x = right_thumbstick_axes_.y() * vr_thumbstick_linear_scale_;
                    double delta_y = right_thumbstick_axes_.x() * vr_thumbstick_linear_scale_;

                    // 累积XY偏移
                    right_thumbstick_offset_.x() -= delta_x;
                    right_thumbstick_offset_.y() -= delta_y;

                    RCLCPP_DEBUG(node_->get_logger(), "🕹️ Right thumbstick (XY): Y=%.3f→ΔX=%.4f, X=%.3f→ΔY=%.4f",
                                 right_thumbstick_axes_.y(), delta_x,
                                 right_thumbstick_axes_.x(), delta_y);
                }
            }
        }
    }

    void VRInputHandler::requestBodyMode(
        int32_t eventCase,
        const char* direction,
        const char* modeName,
        const char* command,
        int expectedBodyState)
    {
        if (!enabled_.load())
        {
            RCLCPP_WARN(node_->get_logger(),
                        "🔘 [case %d] 忽略%s模式：VR 控制未启用",
                        eventCase, modeName);
            return;
        }

        if (!isFullBodyMode())
        {
            RCLCPP_WARN(node_->get_logger(),
                        "🔘 [case %d] 忽略%s模式：当前不是 FULL_BODY",
                        eventCase, modeName);
            return;
        }

        if (current_fsm_state_.load() != 3)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "🔘 [case %d] 忽略%s模式：当前 FSM 不是 OCS2",
                        eventCase, modeName);
            return;
        }

        if (expectedBodyState ==
                arms_ros2_control_msgs::msg::WbcCurrentState::BODY_TRACKING &&
            target_manager_ &&
            target_manager_->getCurrentMode() != MarkerState::CONTINUOUS)
        {
            target_manager_->togglePublishMode();
        }

        requested_body_state_.store(expectedBodyState);
        body_mode_request_time_ = std::chrono::steady_clock::now();
        body_mode_request_pending_.store(true);
        publishHumanoidModeCommand(command);

        RCLCPP_INFO(
            node_->get_logger(),
            "🔘 [case %d] %s：已请求%s（%s），等待 WBC 确认和左摇杆回中",
            eventCase, direction, modeName, command);
    }

    bool VRInputHandler::readWbcToggleState(
        WbcToggleTarget target,
        bool& enabled) const
    {
        if (!target_manager_)
        {
            return false;
        }

        using WbcState =
            arms_ros2_control_msgs::msg::WbcCurrentState;

        switch (target)
        {
            case WbcToggleTarget::BASE:
                enabled =
                    target_manager_->getCurrentBaseState() ==
                    WbcState::BASE_UNLOCKED;
                return true;
            case WbcToggleTarget::BIMANUAL:
                enabled =
                    target_manager_->getCurrentBimanualState() ==
                    WbcState::BIMANUAL_COUPLED;
                return true;
            case WbcToggleTarget::LEFT_ARM:
                enabled =
                    target_manager_->getCurrentLeftArmState() ==
                    WbcState::ARM_ENABLED;
                return true;
            case WbcToggleTarget::RIGHT_ARM:
                enabled =
                    target_manager_->getCurrentRightArmState() ==
                    WbcState::ARM_ENABLED;
                return true;
            case WbcToggleTarget::HOME_JOINT:
                enabled = home_joint_reference_enabled_.load();
                return true;
            case WbcToggleTarget::NONE:
                return false;
        }

        return false;
    }

    void VRInputHandler::requestWbcToggle(
        int32_t eventCase,
        const char* sourceDescription,
        WbcToggleTarget target)
    {
        if (!enabled_.load() ||
            !isFullBodyMode() ||
            current_fsm_state_.load() != 3)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "🔘 [case %d] 忽略%s：要求 VR enabled、FULL_BODY、OCS2",
                eventCase, sourceDescription);
            return;
        }

        if (target == WbcToggleTarget::HOME_JOINT &&
            !has_home_joint_reference_.load())
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "🔘 [case %d] 忽略%s：当前机器人没有参考关节能力",
                eventCase, sourceDescription);
            return;
        }

        if (right_wbc_toggle_request_pending_.load())
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "🔘 [case %d] 忽略%s：上一条 WBC 开关请求仍在 pending",
                eventCase, sourceDescription);
            return;
        }

        using WbcState =
            arms_ros2_control_msgs::msg::WbcCurrentState;

        const bool bimanualCoupled =
            target_manager_ &&
            target_manager_->getCurrentBimanualState() ==
                WbcState::BIMANUAL_COUPLED;
        const bool leftEnabled =
            target_manager_ &&
            target_manager_->getCurrentLeftArmState() ==
                WbcState::ARM_ENABLED;
        const bool rightEnabled =
            target_manager_ &&
            target_manager_->getCurrentRightArmState() ==
                WbcState::ARM_ENABLED;

        if ((target == WbcToggleTarget::LEFT_ARM ||
             target == WbcToggleTarget::RIGHT_ARM) &&
            bimanualCoupled)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "🔘 双臂耦合开启时禁止切换单臂启用状态");
            return;
        }

        if (target == WbcToggleTarget::BIMANUAL &&
            !bimanualCoupled &&
            (!leftEnabled || !rightEnabled))
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "🔘 任意一臂禁用时禁止开启双臂耦合");
            return;
        }

        bool currentEnabled = false;
        if (!readWbcToggleState(target, currentEnabled))
        {
            RCLCPP_WARN(node_->get_logger(),
                        "🔘 无法读取 WBC 开关状态");
            return;
        }

        const bool expectedEnabled = !currentEnabled;
        const char* command = nullptr;

        switch (target)
        {
            case WbcToggleTarget::BASE:
                command = expectedEnabled ? "BASE_UNLOCK" : "BASE_LOCK";
                break;
            case WbcToggleTarget::BIMANUAL:
                command = expectedEnabled ? "ARMS_COUPLED" : "ARMS_INDEPENDENT";
                break;
            case WbcToggleTarget::LEFT_ARM:
                command = expectedEnabled ? "LEFT_ARM_ENABLE" : "LEFT_ARM_DISABLE";
                break;
            case WbcToggleTarget::RIGHT_ARM:
                command = expectedEnabled ? "RIGHT_ARM_ENABLE" : "RIGHT_ARM_DISABLE";
                break;
            case WbcToggleTarget::HOME_JOINT:
                command = expectedEnabled ? "HOME_JOINT_ON" : "HOME_JOINT_OFF";
                break;
            case WbcToggleTarget::NONE:
                return;
        }

        requested_wbc_toggle_target_.store(target);
        requested_wbc_toggle_enabled_.store(expectedEnabled);
        right_wbc_toggle_request_time_ =
            std::chrono::steady_clock::now();
        right_wbc_toggle_request_pending_.store(true);
        publishHumanoidModeCommand(command);

        RCLCPP_INFO(
            node_->get_logger(),
            "🔘 [case %d] %s：发布 %s，等待 WBC 确认和右摇杆回中",
            eventCase, sourceDescription, command);
    }

    bool VRInputHandler::updateRightWbcToggleRequestState()
    {
        if (!right_wbc_toggle_request_pending_.load())
        {
            return false;
        }

        const bool centered =
            std::abs(right_thumbstick_axes_raw_.x()) <= 0.3 &&
            std::abs(right_thumbstick_axes_raw_.y()) <= 0.3;

        bool actualEnabled = false;
        const bool stateAvailable = readWbcToggleState(
            requested_wbc_toggle_target_.load(),
            actualEnabled);
        const bool confirmed =
            current_fsm_state_.load() == 3 &&
            stateAvailable &&
            actualEnabled ==
                requested_wbc_toggle_enabled_.load();

        if (confirmed && centered)
        {
            right_wbc_toggle_request_pending_.store(false);
            requested_wbc_toggle_target_.store(
                WbcToggleTarget::NONE);
            RCLCPP_INFO(
                node_->get_logger(),
                "🔘 右侧 WBC 开关已确认，右摇杆已回中");
            return false;
        }

        const bool timedOut =
            std::chrono::steady_clock::now() -
                right_wbc_toggle_request_time_ >=
            right_wbc_toggle_request_timeout_;

        if (timedOut && centered)
        {
            right_wbc_toggle_request_pending_.store(false);
            requested_wbc_toggle_target_.store(
                WbcToggleTarget::NONE);
            RCLCPP_WARN(
                node_->get_logger(),
                "🔘 右侧 WBC 开关请求未在 2 秒内确认，右摇杆已回中");
            return false;
        }

        return true;
    }

    void VRInputHandler::wbcStateCallback(
        arms_ros2_control_msgs::msg::WbcCurrentState::ConstSharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        if (msg->head_state != arms_ros2_control_msgs::msg::WbcCurrentState::HEAD_TRACKING)
        {
            stopHeadTracking();
        }
        home_joint_reference_enabled_.store(msg->home_joint_reference_enabled);

        updateArmWbcVrState(
            WbcToggleTarget::LEFT_ARM, msg->left_arm_state);
        updateArmWbcVrState(
            WbcToggleTarget::RIGHT_ARM, msg->right_arm_state);
    }

    void VRInputHandler::wbcCapabilityCallback(
        arms_ros2_control_msgs::msg::WbcCapability::ConstSharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        has_home_joint_reference_.store(msg->has_home_joint_reference);
    }

    void VRInputHandler::updateArmWbcVrState(
        WbcToggleTarget arm,
        int armState)
    {
        using WbcState = arms_ros2_control_msgs::msg::WbcCurrentState;

        std::atomic<int>* observed = nullptr;
        std::atomic<bool>* suppressed = nullptr;
        std::atomic<bool>* rebasePending = nullptr;
        const char* armName = nullptr;

        if (arm == WbcToggleTarget::LEFT_ARM)
        {
            observed = &observed_left_wbc_arm_state_;
            suppressed = &left_wbc_vr_suppressed_;
            rebasePending = &left_wbc_rebase_pending_;
            armName = "left";
        }
        else if (arm == WbcToggleTarget::RIGHT_ARM)
        {
            observed = &observed_right_wbc_arm_state_;
            suppressed = &right_wbc_vr_suppressed_;
            rebasePending = &right_wbc_rebase_pending_;
            armName = "right";
        }
        else
        {
            return;
        }

        const int previous = observed->exchange(armState);
        if (armState == WbcState::ARM_DISABLED)
        {
            suppressed->store(true);
            rebasePending->store(false);
            std::atomic<bool>* clearPause = arm == WbcToggleTarget::LEFT_ARM
                ? &left_clear_pause_after_disable_
                : &right_clear_pause_after_disable_;
            if (clearPause->load())
            {
                clearArmPause(arm);
                clearPause->store(false);
                RCLCPP_INFO(
                    node_->get_logger(),
                    "🕹️ %s VR pause cleared after WBC disable (case 16)",
                    armName);
            }
            return;
        }

        if (armState != WbcState::ARM_ENABLED)
        {
            suppressed->store(true);
            rebasePending->store(false);
            RCLCPP_WARN(
                node_->get_logger(),
                "🕹️ Unknown %s WBC arm state %d; VR target remains suppressed",
                armName, armState);
            return;
        }

        if (previous == WbcState::ARM_DISABLED)
        {
            suppressed->store(true);
            rebasePending->store(true);
            RCLCPP_INFO(
                node_->get_logger(),
                "🕹️ %s WBC arm enabled; waiting for current VR pose rebase",
                armName);
            return;
        }

        if (!rebasePending->load())
        {
            suppressed->store(false);
        }
    }

    void VRInputHandler::clearLastPublishedTarget(
        const std::string& armType)
    {
        if (armType == "left")
        {
            has_last_published_left_target_ = false;
            last_published_left_position_.setZero();
            last_published_left_orientation_.setIdentity();
            last_published_left_stamp_ = std::chrono::steady_clock::time_point{};
            resetStaleCatchUpRamp(true);
            left_arm_ramp_.reset();
        }
        else if (armType == "right")
        {
            has_last_published_right_target_ = false;
            last_published_right_position_.setZero();
            last_published_right_orientation_.setIdentity();
            last_published_right_stamp_ = std::chrono::steady_clock::time_point{};
            resetStaleCatchUpRamp(false);
            right_arm_ramp_.reset();
        }
    }

    bool VRInputHandler::rebaseArmVrControlFromCurrentPose(
        WbcToggleTarget arm)
    {
        const bool isLeftArm = arm == WbcToggleTarget::LEFT_ARM;
        const bool isRightArm = arm == WbcToggleTarget::RIGHT_ARM;
        if (!isLeftArm && !isRightArm)
        {
            return false;
        }

        const bool useLeftVr = mirror_mode_.load()
            ? isRightArm
            : isLeftArm;
        if ((useLeftVr && !has_vr_left_pose_.load()) ||
            (!useLeftVr && !has_vr_right_pose_.load()))
        {
            return false;
        }

        const std::string armType = isLeftArm ? "left" : "right";
        if (!setRobotBase(armType, /*preferLastCommand=*/false))
        {
            return false;
        }
        resetStaleCatchUpRamp(isLeftArm);

        if (useLeftVr)
        {
            vr_base_left_position_ = left_position_;
            vr_base_left_orientation_ = left_orientation_;
        }
        else
        {
            vr_base_right_position_ = right_position_;
            vr_base_right_orientation_ = right_orientation_;
        }

        if (isLeftArm)
        {
            left_thumbstick_offset_.setZero();
            left_thumbstick_yaw_offset_ = 0.0;
            prev_calculated_left_position_ = robot_current_left_position_;
            prev_calculated_left_orientation_ =
                robot_current_left_orientation_.normalized();
        }
        else
        {
            right_thumbstick_offset_.setZero();
            right_thumbstick_yaw_offset_ = 0.0;
            prev_calculated_right_position_ = robot_current_right_position_;
            prev_calculated_right_orientation_ =
                robot_current_right_orientation_.normalized();
        }

        clearLastPublishedTarget(armType);
        return true;
    }

    bool VRInputHandler::isArmVrInputSuppressed(
        WbcToggleTarget arm) const
    {
        if (!isFullBodyMode())
        {
            return false;
        }
        if (arm == WbcToggleTarget::LEFT_ARM)
        {
            return left_wbc_vr_suppressed_.load();
        }
        if (arm == WbcToggleTarget::RIGHT_ARM)
        {
            return right_wbc_vr_suppressed_.load();
        }
        return false;
    }

    bool VRInputHandler::prepareArmVrInput(WbcToggleTarget arm)
    {
        if (!isArmVrInputSuppressed(arm))
        {
            return true;
        }

        std::atomic<bool>* pending = arm == WbcToggleTarget::LEFT_ARM
            ? &left_wbc_rebase_pending_
            : arm == WbcToggleTarget::RIGHT_ARM
                ? &right_wbc_rebase_pending_
                : nullptr;
        std::atomic<bool>* suppressed = arm == WbcToggleTarget::LEFT_ARM
            ? &left_wbc_vr_suppressed_
            : arm == WbcToggleTarget::RIGHT_ARM
                ? &right_wbc_vr_suppressed_
                : nullptr;
        if (!pending || !suppressed || !pending->load())
        {
            return false;
        }

        if (!rebaseArmVrControlFromCurrentPose(arm))
        {
            return false;
        }

        std::atomic<bool>* pauseAfterEnable = arm == WbcToggleTarget::LEFT_ARM
            ? &left_pause_after_enable_
            : &right_pause_after_enable_;
        if (pauseAfterEnable->load())
        {
            applyPauseAfterRebase(arm);
            pauseAfterEnable->store(false);
            RCLCPP_INFO(
                node_->get_logger(),
                "🕹️ %s VR paused after WBC enable rebase (case 16)",
                arm == WbcToggleTarget::LEFT_ARM ? "left" : "right");
        }

        pending->store(false);
        suppressed->store(false);
        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️ %s WBC arm VR control rebased from current pose",
            arm == WbcToggleTarget::LEFT_ARM ? "left" : "right");
        return true;
    }

    void VRInputHandler::publishHumanoidModeCommand(const std::string& command)
    {
        if (!pub_mode_command_)
        {
            return;
        }

        std_msgs::msg::String msg;
        msg.data = command;
        pub_mode_command_->publish(msg);
    }

    bool VRInputHandler::isYbModeConversionPending() const
    {
        return left_pause_after_enable_.load() ||
               right_pause_after_enable_.load() ||
               left_clear_pause_after_disable_.load() ||
               right_clear_pause_after_disable_.load();
    }

    void VRInputHandler::resetYbModeLatchAndConversions()
    {
        full_body_yb_pause_mode_.store(false);
        left_pause_after_enable_.store(false);
        right_pause_after_enable_.store(false);
        left_clear_pause_after_disable_.store(false);
        right_clear_pause_after_disable_.store(false);
    }

    void VRInputHandler::updateYbModeConversionState()
    {
        const auto now = std::chrono::steady_clock::now();
        auto expire = [this, now](
            std::atomic<bool>& flag,
            std::chrono::steady_clock::time_point& started,
            const char* what) {
            if (!flag.load())
            {
                return;
            }
            if (now - started < yb_conversion_timeout_)
            {
                return;
            }
            flag.store(false);
            RCLCPP_WARN(
                node_->get_logger(),
                "🔘 case 16 %s 未在 2 秒内由 WBC 确认，已丢弃转换标志（锁存保持）",
                what);
        };
        expire(left_pause_after_enable_, left_yb_conversion_time_,
               "left pause_after_enable");
        expire(right_pause_after_enable_, right_yb_conversion_time_,
               "right pause_after_enable");
        expire(left_clear_pause_after_disable_, left_yb_conversion_time_,
               "left clear_pause_after_disable");
        expire(right_clear_pause_after_disable_, right_yb_conversion_time_,
               "right clear_pause_after_disable");
    }

    void VRInputHandler::applyPauseAfterRebase(WbcToggleTarget arm)
    {
        const bool isLeftArm = arm == WbcToggleTarget::LEFT_ARM;
        const bool isRightArm = arm == WbcToggleTarget::RIGHT_ARM;
        if (!isLeftArm && !isRightArm)
        {
            return;
        }
        const bool useLeftVr = mirror_mode_.load() ? isRightArm : isLeftArm;
        if (useLeftVr)
        {
            paused_left_position_ = left_position_;
            paused_left_orientation_ = left_orientation_;
        }
        else
        {
            paused_right_position_ = right_position_;
            paused_right_orientation_ = right_orientation_;
        }
        if (isLeftArm)
        {
            left_arm_paused_.store(true);
        }
        else
        {
            right_arm_paused_.store(true);
        }
        resetStaleCatchUpRamp(isLeftArm);
    }

    void VRInputHandler::clearArmPause(WbcToggleTarget arm)
    {
        const bool isLeftArm = arm == WbcToggleTarget::LEFT_ARM;
        const bool isRightArm = arm == WbcToggleTarget::RIGHT_ARM;
        if (!isLeftArm && !isRightArm)
        {
            return;
        }
        const bool useLeftVr = mirror_mode_.load() ? isRightArm : isLeftArm;
        if (isLeftArm)
        {
            left_arm_paused_.store(false);
        }
        else
        {
            right_arm_paused_.store(false);
        }
        if (useLeftVr)
        {
            paused_left_position_ = Eigen::Vector3d::Zero();
            paused_left_orientation_ = Eigen::Quaterniond::Identity();
        }
        else
        {
            paused_right_position_ = Eigen::Vector3d::Zero();
            paused_right_orientation_ = Eigen::Quaterniond::Identity();
        }
        resetStaleCatchUpRamp(isLeftArm);
    }

    bool VRInputHandler::handleCase16YbModeToggle()
    {
        if (!enabled_.load() || !isFullBodyMode() ||
            current_fsm_state_.load() != 3)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "🔘 [case 16] 忽略：要求 VR enabled、FULL_BODY、OCS2");
            return false;
        }
        if (right_wbc_toggle_request_pending_.load() ||
            isYbModeConversionPending())
        {
            RCLCPP_WARN(node_->get_logger(),
                        "🔘 [case 16] 忽略：WBC 开关或 16 转换仍在 pending");
            return false;
        }

        using WbcState = arms_ros2_control_msgs::msg::WbcCurrentState;
        const bool turningToDisable = full_body_yb_pause_mode_.load();
        const bool coupled = target_manager_ &&
            target_manager_->getCurrentBimanualState() ==
                WbcState::BIMANUAL_COUPLED;
        if (turningToDisable && coupled)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "🔘 [case 16] 忽略：双臂耦合时禁止把暂停语义切回禁用");
            return false;
        }

        const bool nowPause = !full_body_yb_pause_mode_.load();
        full_body_yb_pause_mode_.store(nowPause);
        RCLCPP_INFO(node_->get_logger(),
                    "🔘 [case 16] Y/B 语义切换为 %s",
                    nowPause ? "VR暂停" : "WBC单臂禁用");

        const auto now = std::chrono::steady_clock::now();
        if (nowPause)
        {
            if (target_manager_ &&
                target_manager_->getCurrentLeftArmState() ==
                    WbcState::ARM_DISABLED)
            {
                left_pause_after_enable_.store(true);
                left_yb_conversion_time_ = now;
                publishHumanoidModeCommand("LEFT_ARM_ENABLE");
            }
            if (target_manager_ &&
                target_manager_->getCurrentRightArmState() ==
                    WbcState::ARM_DISABLED)
            {
                right_pause_after_enable_.store(true);
                right_yb_conversion_time_ = now;
                publishHumanoidModeCommand("RIGHT_ARM_ENABLE");
            }
        }
        else
        {
            if (left_arm_paused_.load())
            {
                left_clear_pause_after_disable_.store(true);
                left_yb_conversion_time_ = now;
                publishHumanoidModeCommand("LEFT_ARM_DISABLE");
            }
            if (right_arm_paused_.load())
            {
                right_clear_pause_after_disable_.store(true);
                right_yb_conversion_time_ = now;
                publishHumanoidModeCommand("RIGHT_ARM_DISABLE");
            }
        }
        return true;
    }

    bool VRInputHandler::isBodyTrackingActive() const
    {
        return target_manager_ && target_manager_->shouldShowBodyMarker();
    }

    bool VRInputHandler::updateBodyModeRequestState()
    {
        if (!body_mode_request_pending_.load())
        {
            return false;
        }

        const bool centered =
            std::abs(left_thumbstick_axes_raw_.x()) <= 0.3 &&
            std::abs(left_thumbstick_axes_raw_.y()) <= 0.3;
        const int expectedState = requested_body_state_.load();
        const bool confirmed =
            current_fsm_state_.load() == 3 &&
            target_manager_ &&
            target_manager_->getCurrentBodyState() == expectedState;

        if (confirmed && centered)
        {
            body_mode_request_pending_.store(false);
            requested_body_state_.store(-1);
            RCLCPP_INFO(node_->get_logger(),
                        "🔘 身体模式已由 WBC 确认，左摇杆已回中");
            return false;
        }

        const bool timedOut =
            std::chrono::steady_clock::now() - body_mode_request_time_ >=
            body_mode_request_timeout_;

        if (timedOut && centered)
        {
            body_mode_request_pending_.store(false);
            requested_body_state_.store(-1);
            RCLCPP_WARN(
                node_->get_logger(),
                "🔘 身体模式请求未在 2 秒内确认，左摇杆已回中，按 WBC 实际状态恢复路由");
            return false;
        }

        return true;
    }

    void VRInputHandler::processBodyThumbstickAxes()
    {
        // 腰部增量直接改 BodyMarker 绝对目标，不依赖双臂 UPDATE/STORAGE 的 VR base pose，
        // 因此这里不检查 is_update_mode_。
        if (!enabled_.load() || !target_manager_ || !isBodyTrackingActive())
        {
            return;
        }

        std::array<double, 3> positionDelta{0.0, 0.0, 0.0};
        std::array<double, 3> rpyDelta{0.0, 0.0, 0.0};
        constexpr double body_thumbstick_step_scale = 0.1;

        if (body_thumbstick_z_yaw_mode_.load())
        {
            positionDelta[2] = -left_thumbstick_axes_.y() * vr_thumbstick_linear_scale_ * body_thumbstick_step_scale;
            rpyDelta[2] = -left_thumbstick_axes_.x() * vr_thumbstick_angular_scale_ * body_thumbstick_step_scale;
        }
        else
        {
            positionDelta[0] = -left_thumbstick_axes_.y() * vr_thumbstick_linear_scale_ * body_thumbstick_step_scale;
            positionDelta[1] = -left_thumbstick_axes_.x() * vr_thumbstick_linear_scale_ * body_thumbstick_step_scale;
        }

        // 回中时跳过，避免 20Hz 空发布刷新 last_marker_command_time_，
        // 永久压制 body_current_target 回显。
        if (std::abs(positionDelta[0]) < 1e-9 &&
            std::abs(positionDelta[1]) < 1e-9 &&
            std::abs(positionDelta[2]) < 1e-9 &&
            std::abs(rpyDelta[0]) < 1e-9 &&
            std::abs(rpyDelta[1]) < 1e-9 &&
            std::abs(rpyDelta[2]) < 1e-9)
        {
            return;
        }

        target_manager_->updateBodyMarkerPoseIncremental(positionDelta, rpyDelta);
    }

    void VRInputHandler::processChassisAxes()
    {
        // 全身模式下底盘跟踪（长按右摇杆）开启时，case 20 手动底盘遥控让路，
        // 避免两个来源同时往 /cmd_vel 发指令。
        if (whole_body_teleop_active_.load() && base_tracking_enabled_.load())
        {
            return;
        }

        // VR 手柄轴值约定（Pico）：Y 轴往前推是负数，与机器人"正数=向前/向右"相反，
        // 因此 chassis 模式下对四个轴值统一取负，使方向语义与 joystick_teleop.cpp 对齐。
        // 非 FULL_BODY 下按住右握把可作为腰部控制修饰符；
        // FULL_BODY 下右握把方向由 WBC 四开关组合消费，右轴会在进入这里前清零。
        const double left_x = -left_thumbstick_axes_.x();
        const double left_y = -left_thumbstick_axes_.y();
        const double right_x = -right_thumbstick_axes_.x();
        const double right_y = -right_thumbstick_axes_.y();
        // 底盘平移：左摇杆 Y → linear.x（前后），左摇杆 X → linear.y（左右，全向底盘）
        auto cmd_vel = geometry_msgs::msg::Twist();
        cmd_vel.linear.x  = left_y * chassis_linear_scale_;
        cmd_vel.linear.y  = left_x * chassis_linear_scale_;
        cmd_vel.linear.z  = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;

        // 腰部命令默认为 0（每次都 publish 含 0，避免松开摇杆后 latch 残留）
        // 注：VR 端 xr_target_node 已对摇杆值应用过死区（deadzone=0.1），
        // 这里不再加额外阈值，直接用摇杆值，与 cmd_vel 路径保持一致。
        auto waist_lifting = std_msgs::msg::Float64();
        auto waist_turning = std_msgs::msg::Float64();

        const bool right_grip = right_grip_active_.load();
        if (right_grip)
        {
            // 只发送主轴命令；切换需要超过滞回余量，避免斜推时升降/旋转互相抢占。
            cmd_vel.angular.z = 0.0;
            const double lift_magnitude = std::abs(right_y);
            const double turn_magnitude = std::abs(right_x);
            auto axis = waist_stick_axis_.load();
            if ((axis == WaistStickAxis::LIFTING && lift_magnitude == 0.0) ||
                (axis == WaistStickAxis::TURNING && turn_magnitude == 0.0))
                axis = WaistStickAxis::NONE;

            if (lift_magnitude == 0.0 && turn_magnitude == 0.0)
                axis = WaistStickAxis::NONE;
            else if (axis == WaistStickAxis::NONE)
                axis = lift_magnitude >= turn_magnitude
                    ? WaistStickAxis::LIFTING : WaistStickAxis::TURNING;
            else if (axis == WaistStickAxis::LIFTING &&
                     turn_magnitude > lift_magnitude + waist_axis_switch_margin_)
                axis = WaistStickAxis::TURNING;
            else if (axis == WaistStickAxis::TURNING &&
                     lift_magnitude > turn_magnitude + waist_axis_switch_margin_)
                axis = WaistStickAxis::LIFTING;

            waist_stick_axis_.store(axis);
            if (axis == WaistStickAxis::LIFTING)
                waist_lifting.data = right_y * waist_lifting_scale_;
            else if (axis == WaistStickAxis::TURNING)
                waist_turning.data = right_x * waist_turning_scale_;
        }
        else
        {
            waist_stick_axis_.store(WaistStickAxis::NONE);
            // 默认模式：右摇杆 X → 底盘转向 angular.z，右摇杆 Y 忽略
            cmd_vel.angular.z = right_x * chassis_angular_scale_;
            // waist_lifting / waist_turning 保持 0（已在上方初始化）
        }

        publishChassisVelocity(cmd_vel);
        if (pub_waist_lifting_) pub_waist_lifting_->publish(waist_lifting);
        if (pub_waist_turning_) pub_waist_turning_->publish(waist_turning);

        RCLCPP_DEBUG(node_->get_logger(),
                     "🕹️ [Chassis] grip=%d L.xy=(%.3f,%.3f) R.xy=(%.3f,%.3f) → cmd_vel(lin=%.3f,%.3f ang=%.3f) waist(lift=%.3f turn=%.3f)",
                     right_grip ? 1 : 0,
                     left_thumbstick_axes_.x(), left_thumbstick_axes_.y(),
                     right_thumbstick_axes_.x(), right_thumbstick_axes_.y(),
                     cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z,
                     waist_lifting.data, waist_turning.data);
    }

    bool VRInputHandler::wbcOwnsChassisVelocity() const
    {
        // 仅全身 OCS2 且底盘规划解锁时让路；HOLD / 锁底盘由 VR 直发 /cmd_vel。
        return isFullBodyMode() &&
               current_fsm_state_.load() == 3 &&
               target_manager_ &&
               target_manager_->getCurrentBaseState() ==
                   arms_ros2_control_msgs::msg::WbcCurrentState::BASE_UNLOCKED;
    }

    void VRInputHandler::publishChassisVelocity(const geometry_msgs::msg::Twist& velocity)
    {
        if (wbcOwnsChassisVelocity())
        {
            return;
        }

        pub_cmd_vel_->publish(velocity);
        // 无 TF 时用这条积分当作底座位姿估计（绝对映射的手臂目标要转到 world 系）
        integrateBaseOdometry(velocity);
    }

    void VRInputHandler::resetChassisAndWaistCommands()
    {
        waist_stick_axis_.store(WaistStickAxis::NONE);
        // 底盘清零同样遵守 WBC 占用规则；腰部照常清零。
        auto zero_vel = geometry_msgs::msg::Twist();
        publishChassisVelocity(zero_vel);

        auto zero_float = std_msgs::msg::Float64();
        zero_float.data = 0.0;
        if (pub_waist_lifting_) pub_waist_lifting_->publish(zero_float);
        if (pub_waist_turning_) pub_waist_turning_->publish(zero_float);
    }

    void VRInputHandler::toggleChassisMode()
    {
        const bool new_mode = !chassis_mode_.load();
        chassis_mode_.store(new_mode);

        if (new_mode)
        {
            // 进入底盘模式：先清零末端摇杆累积偏移（避免退出时跳变），
            // 再清零底盘/腰部命令（确保从静止开始）
            left_thumbstick_offset_ = Eigen::Vector3d::Zero();
            right_thumbstick_offset_ = Eigen::Vector3d::Zero();
            left_thumbstick_yaw_offset_ = 0.0;
            right_thumbstick_yaw_offset_ = 0.0;
            resetChassisAndWaistCommands();

            RCLCPP_INFO(node_->get_logger(),
                        "🔘 [case 20] 切换到 CHASSIS 模式（左摇杆→底盘XY，右摇杆→腰部升降/旋转）");
        }
        else
        {
            // 退出底盘模式：先停底盘/腰部，再恢复末端摇杆逻辑
            resetChassisAndWaistCommands();
            RCLCPP_INFO(node_->get_logger(),
                        "🔘 [case 20] 退出 CHASSIS 模式，恢复末端控制");
        }
    }

    void VRInputHandler::handleGripActiveEdge(bool isLeft, bool pressed)
    {
        auto& armed = isLeft ? left_grip_press_armed_ : right_grip_press_armed_;
        auto& comboConsumed = isLeft ? left_grip_combo_consumed_ : right_grip_combo_consumed_;
        auto& pressStart = isLeft ? left_grip_press_start_ : right_grip_press_start_;

        if (pressed)
        {
            pressStart = std::chrono::steady_clock::now();
            comboConsumed.store(false);
            armed.store(true);
            return;
        }

        if (!armed.exchange(false))
        {
            return;
        }

        if (comboConsumed.load())
        {
            return;
        }

        if (std::chrono::steady_clock::now() - pressStart > grip_short_press_max_)
        {
            return;
        }

        applyGripShortPress(isLeft);
    }

    void VRInputHandler::applyGripShortPress(bool isLeft)
    {
        if (isLeft && isBodyTrackingActive())
        {
            const bool newMode = !body_thumbstick_z_yaw_mode_.load();
            body_thumbstick_z_yaw_mode_.store(newMode);
            RCLCPP_INFO(node_->get_logger(),
                        "🔘 [左握把] 短按：腰部摇杆模式切换为 %s",
                        newMode ? "Z+Yaw" : "XY");
            return;
        }

        const bool isMirror = mirror_mode_.load();
        const bool toggleRightArm = isLeft ? isMirror : !isMirror;
        auto& gripMode = toggleRightArm ? right_grip_mode_ : left_grip_mode_;
        const bool newMode = !gripMode.load();
        gripMode.store(newMode);

        const char* gripName = isLeft ? "左握把" : "右握把";
        const char* armName = toggleRightArm ? "右臂" : "左臂";
        const char* modeName = newMode
            ? "Z轴+Yaw旋转模式 (Y→Z, X→Yaw)"
            : "XY平移模式 (Y→X, X→Y)";
        if (isMirror)
        {
            RCLCPP_INFO(node_->get_logger(),
                        "🔘 [%s] 短按 - 功能: 切换%s摇杆控制模式 - 操作: 切换到 %s [镜像模式]",
                        gripName, armName, modeName);
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(),
                        "🔘 [%s] 短按 - 功能: 切换%s摇杆控制模式 - 操作: 切换到 %s",
                        gripName, armName, modeName);
        }
    }

    void VRInputHandler::markGripComboConsumed(bool left, bool right)
    {
        if (left)
        {
            left_grip_combo_consumed_.store(true);
        }
        if (right)
        {
            right_grip_combo_consumed_.store(true);
        }
    }

    void VRInputHandler::processButtonEvent(const std_msgs::msg::Int32::SharedPtr msg)
    {
        // 处理按钮事件（button_event: 0=无事件, 1-6=按钮按下, 7=镜像模式切换, 9=左扳机, 10=右扳机）
        // xr_target_node 对多数按键做上升沿检测；握把 case 2/5 不再切换摇杆模式，改由短按松开判定。

        switch (msg->data)
        {
            case 1:  // 左摇杆按钮
            {
                // 左摇杆按钮功能已移至 xr_target_node.py（用于切换镜像模式）
                // 镜像模式的切换和相关逻辑在 case 7 中处理
                // 这里保留空处理以保持兼容性
                break;
            }
            case 2:  // 左握把按钮：上升沿由 xr_target_node 发出；摇杆模式改由短按松开判定
                break;
            case 3:  // 左Y按钮
            {
                const auto topology = controlTopology();
                if (topology == ControlTopology::UNKNOWN)
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "🔘 [case 3] 忽略左Y按钮：控制拓扑尚未确认");
                    break;
                }
                if (isYbModeConversionPending())
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "🔘 [case 3] 忽略左Y：case 16 转换仍在 pending");
                    break;
                }
                if (topology == ControlTopology::FULL_BODY &&
                    !isFullBodyYbPauseMode())
                {
                    const auto target = mirror_mode_.load()
                        ? WbcToggleTarget::RIGHT_ARM
                        : WbcToggleTarget::LEFT_ARM;
                    requestWbcToggle(3, "左Y按钮", target);
                    break;
                }
                // SPLIT_BODY，或 FULL_BODY 暂停语义：沿用下方现有暂停/恢复

                // 只在UPDATE模式下启用（右手摇杆按钮按下后进入UPDATE模式）
                if (!is_update_mode_.load())
                {
                    RCLCPP_DEBUG(node_->get_logger(), "🔘 [左Y按钮] 按下 - 功能: 切换左臂更新状态 - 操作: 忽略（当前不在UPDATE模式）");
                    break;
                }

                // 根据镜像模式决定控制哪个臂
                if (mirror_mode_.load())
                {
                    // 镜像模式：左话题数据用于右臂
                    if (right_arm_paused_.load())
                    {
                        // 当前是暂停状态，执行恢复操作
                        vr_base_left_position_ = left_position_;
                        vr_base_left_orientation_ = left_orientation_;
                        const bool baseReady =
                            setRobotBase("right", /*preferLastCommand=*/true);

                        // 重置右摇杆累积偏移
                        right_thumbstick_offset_ = Eigen::Vector3d::Zero();
                        right_thumbstick_yaw_offset_ = 0.0;

                        // 切换状态为运行
                        right_arm_paused_.store(false);
                        // 清除暂停时刻的VR位姿记录（恢复后不再使用）
                        paused_left_position_ = Eigen::Vector3d::Zero();
                        paused_left_orientation_ = Eigen::Quaterniond::Identity();

                        RCLCPP_INFO(node_->get_logger(), "🔘 [左Y按钮] 按下 - 功能: 切换右臂更新状态 - 操作: 恢复右臂更新（重置基准位姿和摇杆偏移） [镜像模式]");
                        RCLCPP_INFO(
                            node_->get_logger(),
                            "🕹️ right VR base %s after resume",
                            baseReady ? "ready" : "waiting for TF");
                        RCLCPP_DEBUG(node_->get_logger(),
                                    "   Right Robot Base Source: %s",
                                    has_last_published_right_target_ ? "last_command" : "current_pose");
                    }
                    else
                    {
                        // 当前是运行状态，执行暂停操作
                        // 记录暂停时刻的VR位姿（镜像模式：左话题数据用于右臂）
                        paused_left_position_ = left_position_;
                        paused_left_orientation_ = left_orientation_;
                        right_arm_paused_.store(true);
                        RCLCPP_INFO(node_->get_logger(), "🔘 [左Y按钮] 按下 - 功能: 切换右臂更新状态 - 操作: 暂停右臂更新（已记录暂停时刻VR位姿，摇杆可继续控制） [镜像模式]");
                    }
                }
                else
                {
                    // 正常模式：左话题数据用于左臂
                    if (left_arm_paused_.load())
                    {
                        // 当前是暂停状态，执行恢复操作
                        vr_base_left_position_ = left_position_;
                        vr_base_left_orientation_ = left_orientation_;
                        const bool baseReady =
                            setRobotBase("left", /*preferLastCommand=*/true);

                        // 重置左摇杆累积偏移
                        left_thumbstick_offset_ = Eigen::Vector3d::Zero();
                        left_thumbstick_yaw_offset_ = 0.0;

                        // 切换状态为运行
                        left_arm_paused_.store(false);
                        // 清除暂停时刻的VR位姿记录（恢复后不再使用）
                        paused_left_position_ = Eigen::Vector3d::Zero();
                        paused_left_orientation_ = Eigen::Quaterniond::Identity();

                        RCLCPP_INFO(node_->get_logger(), "🔘 [左Y按钮] 按下 - 功能: 切换左臂更新状态 - 操作: 恢复左臂更新（重置基准位姿和摇杆偏移）");
                        RCLCPP_INFO(
                            node_->get_logger(),
                            "🕹️ left VR base %s after resume",
                            baseReady ? "ready" : "waiting for TF");
                        RCLCPP_DEBUG(node_->get_logger(),
                                    "   VR Base Position: [%.3f, %.3f, %.3f]",
                                    vr_base_left_position_.x(), vr_base_left_position_.y(), vr_base_left_position_.z());
                        RCLCPP_DEBUG(node_->get_logger(),
                                    "   Robot Base Position: [%.3f, %.3f, %.3f]",
                                    robot_base_left_position_.x(), robot_base_left_position_.y(), robot_base_left_position_.z());
                        RCLCPP_DEBUG(node_->get_logger(),
                                    "   Left Robot Base Source: %s",
                                    has_last_published_left_target_ ? "last_command" : "current_pose");
                    }
                    else
                    {
                        // 当前是运行状态，执行暂停操作
                        // 记录暂停时刻的VR位姿（正常模式：左话题数据用于左臂）
                        paused_left_position_ = left_position_;
                        paused_left_orientation_ = left_orientation_;
                        left_arm_paused_.store(true);
                        RCLCPP_INFO(node_->get_logger(), "🔘 [左Y按钮] 按下 - 功能: 切换左臂更新状态 - 操作: 暂停左臂更新（已记录暂停时刻VR位姿，摇杆可继续控制）");
                    }
                }
                break;
            }
            case 4:  // 右摇杆按钮（短按松开沿）
            {
                // 全身遥操作模式内：短按右摇杆改为开/关「全部跟踪」
                // （胸/头/双臂末端改为 /teleop/body_* 绝对位姿映射）；
                // 非全身模式保持原有 UPDATE/STORAGE 语义不变。
                if (whole_body_teleop_active_.load())
                {
                    toggleAbsoluteTracking();
                    break;
                }

                // 只在OCS2状态下执行（状态值为3）
                if (current_fsm_state_.load() != 3)
                {
                    RCLCPP_DEBUG(node_->get_logger(), "🔘 [右摇杆按钮] 按下 - 功能: 切换UPDATE/STORAGE模式 - 操作: 忽略（当前FSM状态不是OCS2）");
                    break;
                }

                // 确保切换到连续发布模式（更稳健，防止用户手动切换回单次模式）
                if (target_manager_ && target_manager_->getCurrentMode() != MarkerState::CONTINUOUS)
                {
                    target_manager_->togglePublishMode();
                    RCLCPP_DEBUG(node_->get_logger(), "   ArmsTargetManager已切换到CONTINUOUS模式");
                }

                if (!is_update_mode_.load())
                {
                    // 切换到更新模式 - 存储当前poses作为base poses
                    vr_base_left_position_ = left_position_;
                    vr_base_left_orientation_ = left_orientation_;
                    vr_base_right_position_ = right_position_;
                    vr_base_right_orientation_ = right_orientation_;

                    const bool leftBaseReady =
                        setRobotBase("left", /*preferLastCommand=*/true);
                    const bool rightBaseReady =
                        setRobotBase("right", /*preferLastCommand=*/true);
                    RCLCPP_INFO(
                        node_->get_logger(),
                        "🕹️ UPDATE bases: left=%s, right=%s, calculation_frame=%s",
                        leftBaseReady ? "ready" : "pending_tf",
                        rightBaseReady ? "ready" : "pending_tf",
                        vr_follow_frame_.c_str());

                    // 重置摇杆累积偏移
                    left_thumbstick_offset_ = Eigen::Vector3d::Zero();
                    right_thumbstick_offset_ = Eigen::Vector3d::Zero();
                    left_thumbstick_yaw_offset_ = 0.0;
                    right_thumbstick_yaw_offset_ = 0.0;

                    if (!full_body_yb_pause_mode_.load())
                    {
                        // 重置暂停状态，确保切换到 UPDATE 模式时恢复更新
                        bool left_was_paused = left_arm_paused_.load();
                        bool right_was_paused = right_arm_paused_.load();
                        if (left_was_paused)
                        {
                            left_arm_paused_.store(false);
                            // 清除暂停时刻的VR位姿记录
                            paused_left_position_ = Eigen::Vector3d::Zero();
                            paused_left_orientation_ = Eigen::Quaterniond::Identity();
                        }
                        if (right_was_paused)
                        {
                            right_arm_paused_.store(false);
                            // 清除暂停时刻的VR位姿记录
                            paused_right_position_ = Eigen::Vector3d::Zero();
                            paused_right_orientation_ = Eigen::Quaterniond::Identity();
                        }
                    }
                    else
                    {
                        // 暂停语义下保留暂停。vr_base 已写成当前手柄，
                        // 必须把仍暂停臂的冻结位姿对齐，否则差分非零会跳变。
                        if (left_arm_paused_.load())
                        {
                            applyPauseAfterRebase(WbcToggleTarget::LEFT_ARM);
                        }
                        if (right_arm_paused_.load())
                        {
                            applyPauseAfterRebase(WbcToggleTarget::RIGHT_ARM);
                        }
                    }

                    is_update_mode_.store(true);
                    // 目标流在此中断/重启，清空斜坡，避免用陈旧输出量缺口
                    resetStaleCatchUpRamp();
                    RCLCPP_INFO(node_->get_logger(), "🔘 [右摇杆按钮] 按下 - 功能: 切换UPDATE/STORAGE模式 - 操作: 切换到UPDATE模式（已存储基准位姿，重置摇杆偏移）");
                    RCLCPP_DEBUG(node_->get_logger(),
                                "   VR Base Positions: Left [%.3f, %.3f, %.3f], Right [%.3f, %.3f, %.3f]",
                                vr_base_left_position_.x(), vr_base_left_position_.y(), vr_base_left_position_.z(),
                                vr_base_right_position_.x(), vr_base_right_position_.y(), vr_base_right_position_.z());
                    RCLCPP_DEBUG(node_->get_logger(),
                                "   Robot Base Positions: Left [%.3f, %.3f, %.3f], Right [%.3f, %.3f, %.3f]",
                                robot_base_left_position_.x(), robot_base_left_position_.y(), robot_base_left_position_.z(),
                                robot_base_right_position_.x(), robot_base_right_position_.y(),
                                robot_base_right_position_.z());
                    RCLCPP_DEBUG(node_->get_logger(),
                                "   Robot Base Source: Left=%s, Right=%s",
                                has_last_published_left_target_ ? "last_command" : "current_pose",
                                has_last_published_right_target_ ? "last_command" : "current_pose");
                }
                else
                {
                    // 切换到存储模式
                    is_update_mode_.store(false);
                    stopHeadTracking();
                    RCLCPP_INFO(node_->get_logger(), "🔘 [右摇杆按钮] 按下 - 功能: 切换UPDATE/STORAGE模式 - 操作: 切换到STORAGE模式（准备存储新的基准位姿）");
                }
                break;
            }
            case 5:  // 右握把按钮：上升沿由 xr_target_node 发出；摇杆模式改由短按松开判定
                break;
            case 6:  // 右B按钮
            {
                const auto topology = controlTopology();
                if (topology == ControlTopology::UNKNOWN)
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "🔘 [case 6] 忽略右B按钮：控制拓扑尚未确认");
                    break;
                }
                if (isYbModeConversionPending())
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "🔘 [case 6] 忽略右B：case 16 转换仍在 pending");
                    break;
                }
                if (topology == ControlTopology::FULL_BODY &&
                    !isFullBodyYbPauseMode())
                {
                    const auto target = mirror_mode_.load()
                        ? WbcToggleTarget::LEFT_ARM
                        : WbcToggleTarget::RIGHT_ARM;
                    requestWbcToggle(6, "右B按钮", target);
                    break;
                }
                // SPLIT_BODY，或 FULL_BODY 暂停语义：沿用下方现有暂停/恢复

                // 只在UPDATE模式下启用（右手摇杆按钮按下后进入UPDATE模式）
                if (!is_update_mode_.load())
                {
                    RCLCPP_DEBUG(node_->get_logger(), "🔘 [右B按钮] 按下 - 功能: 切换右臂更新状态 - 操作: 忽略（当前不在UPDATE模式）");
                    break;
                }

                // 根据镜像模式决定控制哪个臂
                if (mirror_mode_.load())
                {
                    // 镜像模式：右话题数据用于左臂
                    if (left_arm_paused_.load())
                    {
                        // 当前是暂停状态，执行恢复操作
                        vr_base_right_position_ = right_position_;
                        vr_base_right_orientation_ = right_orientation_;
                        const bool baseReady =
                            setRobotBase("left", /*preferLastCommand=*/true);

                        // 重置左摇杆累积偏移
                        left_thumbstick_offset_ = Eigen::Vector3d::Zero();
                        left_thumbstick_yaw_offset_ = 0.0;

                        // 切换状态为运行
                        left_arm_paused_.store(false);
                        // 清除暂停时刻的VR位姿记录（恢复后不再使用）
                        paused_right_position_ = Eigen::Vector3d::Zero();
                        paused_right_orientation_ = Eigen::Quaterniond::Identity();

                        RCLCPP_INFO(node_->get_logger(), "🔘 [右B按钮] 按下 - 功能: 切换左臂更新状态 - 操作: 恢复左臂更新（重置基准位姿和摇杆偏移） [镜像模式]");
                        RCLCPP_INFO(
                            node_->get_logger(),
                            "🕹️ left VR base %s after resume",
                            baseReady ? "ready" : "waiting for TF");
                        RCLCPP_DEBUG(node_->get_logger(),
                                    "   Left Robot Base Source: %s",
                                    has_last_published_left_target_ ? "last_command" : "current_pose");
                    }
                    else
                    {
                        // 当前是运行状态，执行暂停操作
                        // 记录暂停时刻的VR位姿（镜像模式：右话题数据用于左臂）
                        paused_right_position_ = right_position_;
                        paused_right_orientation_ = right_orientation_;
                        left_arm_paused_.store(true);
                        RCLCPP_INFO(node_->get_logger(), "🔘 [右B按钮] 按下 - 功能: 切换左臂更新状态 - 操作: 暂停左臂更新（已记录暂停时刻VR位姿，摇杆可继续控制） [镜像模式]");
                    }
                }
                else
                {
                    // 正常模式：右话题数据用于右臂
                    if (right_arm_paused_.load())
                    {
                        // 当前是暂停状态，执行恢复操作
                        vr_base_right_position_ = right_position_;
                        vr_base_right_orientation_ = right_orientation_;
                        const bool baseReady =
                            setRobotBase("right", /*preferLastCommand=*/true);

                        // 重置右摇杆累积偏移
                        right_thumbstick_offset_ = Eigen::Vector3d::Zero();
                        right_thumbstick_yaw_offset_ = 0.0;

                        // 切换状态为运行
                        right_arm_paused_.store(false);
                        // 清除暂停时刻的VR位姿记录（恢复后不再使用）
                        paused_right_position_ = Eigen::Vector3d::Zero();
                        paused_right_orientation_ = Eigen::Quaterniond::Identity();

                        RCLCPP_INFO(node_->get_logger(), "🔘 [右B按钮] 按下 - 功能: 切换右臂更新状态 - 操作: 恢复右臂更新（重置基准位姿和摇杆偏移）");
                        RCLCPP_INFO(
                            node_->get_logger(),
                            "🕹️ right VR base %s after resume",
                            baseReady ? "ready" : "waiting for TF");
                        RCLCPP_DEBUG(node_->get_logger(),
                                    "   VR Base Position: [%.3f, %.3f, %.3f]",
                                    vr_base_right_position_.x(), vr_base_right_position_.y(), vr_base_right_position_.z());
                        RCLCPP_DEBUG(node_->get_logger(),
                                    "   Robot Base Position: [%.3f, %.3f, %.3f]",
                                    robot_base_right_position_.x(), robot_base_right_position_.y(), robot_base_right_position_.z());
                        RCLCPP_DEBUG(node_->get_logger(),
                                    "   Right Robot Base Source: %s",
                                    has_last_published_right_target_ ? "last_command" : "current_pose");
                    }
                    else
                    {
                        // 当前是运行状态，执行暂停操作
                        // 记录暂停时刻的VR位姿（正常模式：右话题数据用于右臂）
                        paused_right_position_ = right_position_;
                        paused_right_orientation_ = right_orientation_;
                        right_arm_paused_.store(true);
                        RCLCPP_INFO(node_->get_logger(), "🔘 [右B按钮] 按下 - 功能: 切换右臂更新状态 - 操作: 暂停右臂更新（已记录暂停时刻VR位姿，摇杆可继续控制）");
                    }
                }
                break;
            }
            case 29:  // 左握把 + 左摇杆按下（顺序触发）：切换镜像处理方案
            {
                if (!mirror_mode_.load())
                {
                    RCLCPP_INFO(node_->get_logger(),
                                "🔘 [左握把+左摇杆顺序组合键] 按下 - 功能: 忽略（镜像模式当前未启用，需先通过左摇杆按钮启用）");
                    break;
                }

                const auto current = static_cast<MirrorModeProfile>(mirror_mode_profile_.load());
                const auto next = (current == MirrorModeProfile::MODE_A)
                    ? MirrorModeProfile::MODE_B
                    : MirrorModeProfile::MODE_A;
                mirror_mode_profile_.store(static_cast<int>(next));
                RCLCPP_INFO(node_->get_logger(),
                            "🔘 [左握把+左摇杆顺序组合键] 按下 - 功能: 切换镜像处理方案 -> %s",
                            next == MirrorModeProfile::MODE_A ? "MODE A（空白，等待自定义逻辑）" : "MODE B（原处理逻辑）");

                if (is_update_mode_.load())
                {
                    is_update_mode_.store(false);
                    stopHeadTracking();
                    left_thumbstick_offset_ = Eigen::Vector3d::Zero();
                    right_thumbstick_offset_ = Eigen::Vector3d::Zero();
                    left_thumbstick_yaw_offset_ = 0.0;
                    right_thumbstick_yaw_offset_ = 0.0;
                    paused_left_position_ = Eigen::Vector3d::Zero();
                    paused_left_orientation_ = Eigen::Quaterniond::Identity();
                    paused_right_position_ = Eigen::Vector3d::Zero();
                    paused_right_orientation_ = Eigen::Quaterniond::Identity();
                    RCLCPP_WARN(node_->get_logger(),
                                "   自动切换到STORAGE模式 - 请重新进入UPDATE模式以应用镜像处理方案切换");
                }
                break;
            }
            case 7:  // 镜像模式切换（toggle）
            {
                bool old_mirror_mode = mirror_mode_.load();
                bool new_mirror_mode = !old_mirror_mode;
                mirror_mode_.store(new_mirror_mode);
                if (!new_mirror_mode)
                {
                    mirror_mode_profile_.store(static_cast<int>(MirrorModeProfile::MODE_A));
                }

                if (new_mirror_mode)
                {
                    RCLCPP_INFO(node_->get_logger(),
                                "🔘 [左摇杆按钮] 按下 - 功能: 启用镜像模式（默认处理方案 A）");
                }
                else
                {
                    RCLCPP_INFO(node_->get_logger(),
                                "🔘 [左摇杆按钮] 按下 - 功能: 禁用镜像模式（恢复正常控制）");
                }

                if (is_update_mode_.load())
                {
                    is_update_mode_.store(false);
                    stopHeadTracking();
                    left_thumbstick_offset_ = Eigen::Vector3d::Zero();
                    right_thumbstick_offset_ = Eigen::Vector3d::Zero();
                    left_thumbstick_yaw_offset_ = 0.0;
                    right_thumbstick_yaw_offset_ = 0.0;
                    paused_left_position_ = Eigen::Vector3d::Zero();
                    paused_left_orientation_ = Eigen::Quaterniond::Identity();
                    paused_right_position_ = Eigen::Vector3d::Zero();
                    paused_right_orientation_ = Eigen::Quaterniond::Identity();
                    RCLCPP_WARN(node_->get_logger(),
                                "   自动切换到STORAGE模式 - 请重新进入UPDATE模式以应用镜像模式更改");
                }
                break;
            }
            case 9:  // 左扳机按钮
            {
                std::string controller_name;
                bool is_target_left_arm = true;
                resolveTriggerTarget(true, controller_name, is_target_left_arm);
                if (isGripperPercentMode(is_target_left_arm))
                {
                    RCLCPP_DEBUG(node_->get_logger(),
                                 "🔘 [左扳机按钮] 目标%s夹爪为比例控制(0~1)，忽略开关事件%s",
                                 is_target_left_arm ? "左" : "右",
                                 mirror_mode_.load() ? " [镜像模式]" : "");
                }
                else
                {
                    toggleGripperByTrigger(true);
                }
                break;
            }
            case 10: // 右扳机按钮
            {
                std::string controller_name;
                bool is_target_left_arm = true;
                resolveTriggerTarget(false, controller_name, is_target_left_arm);
                if (isGripperPercentMode(is_target_left_arm))
                {
                    RCLCPP_DEBUG(node_->get_logger(),
                                 "🔘 [右扳机按钮] 目标%s夹爪为比例控制(0~1)，忽略开关事件%s",
                                 is_target_left_arm ? "左" : "右",
                                 mirror_mode_.load() ? " [镜像模式]" : "");
                }
                else
                {
                    toggleGripperByTrigger(false);
                }
                break;
            }
            case 11: // 右A按钮（FSM状态控制）
            {
                // 与 ATM 共用校验后的状态，避免 RViz 切到 MOVEJ 后 VR 仍以为在 HOLD
                const int32_t current_state = resolvedFsmState();

                if (current_state == 2)  // HOLD
                {
                    // HOLD → OCS2
                    sendFsmCommand(3);
                    RCLCPP_INFO(node_->get_logger(), "🔘 [右A按钮] 按下 - 功能: FSM状态前进 - 操作: HOLD → OCS2");
                }
                else if (current_state == 1)  // HOME
                {
                    // HOME → HOLD
                    sendFsmCommand(2);
                    RCLCPP_INFO(node_->get_logger(), "🔘 [右A按钮] 按下 - 功能: FSM状态前进 - 操作: HOME → HOLD");
                }
                else if (current_state == 3)  // OCS2
                {
                    RCLCPP_WARN(node_->get_logger(), "🔘 [右A按钮] 按下 - 功能: FSM状态前进 - 操作: 失败（已在OCS2状态，无法继续前进）");
                }
                else if (current_state == 4)  // MOVEJ
                {
                    RCLCPP_WARN(node_->get_logger(), "🔘 [右A按钮] 按下 - 功能: FSM状态前进 - 操作: 失败（MOVEJ 需先回 HOLD，不能直接进 OCS2）");
                }
                break;
            }
            case 12: // 左X按钮（FSM状态控制）
            {
                const int32_t current_state = resolvedFsmState();

                if (current_state == 3)  // OCS2
                {
                    // OCS2 → HOLD
                    sendFsmCommand(2);
                    RCLCPP_INFO(node_->get_logger(), "🔘 [左X按钮] 按下 - 功能: FSM状态后退/切换 - 操作: OCS2 → HOLD");
                }
                else if (current_state == 4)  // MOVEJ
                {
                    // MOVEJ → HOLD（不能直接 HOME）
                    sendFsmCommand(2);
                    RCLCPP_INFO(node_->get_logger(), "🔘 [左X按钮] 按下 - 功能: FSM状态后退/切换 - 操作: MOVEJ → HOLD");
                }
                else if (current_state == 2)  // HOLD
                {
                    // HOLD → HOME
                    sendFsmCommand(1);
                    RCLCPP_INFO(node_->get_logger(), "🔘 [左X按钮] 按下 - 功能: FSM状态后退/切换 - 操作: HOLD → HOME");
                }
                else if (current_state == 1)  // HOME
                {
                    // HOME状态下，X按钮切换姿态 (HOME ↔ REST)
                    sendFsmCommand(100);
                    RCLCPP_INFO(node_->get_logger(), "🔘 [左X按钮] 按下 - 功能: FSM状态后退/切换 - 操作: 在HOME状态切换姿态 (HOME ↔ REST)");
                }
                break;
            }
            case 13: // 左握把+左扳机：切换左扳机所映射夹爪的百分比/开合
            {
                markGripComboConsumed(true, false);
                toggleGripperPercentMode(true);
                break;
            }
            case 14: // 右握把+右扳机：切换右扳机所映射夹爪的百分比/开合
            {
                markGripComboConsumed(false, true);
                toggleGripperPercentMode(false);
                break;
            }
            case 15: // 左Y+右B：尺度对齐（功能未完善，暂时禁用）
            {
                // runScaleCalibration(true);
                break;
            }
            case 16: // 左右握把+左Y+右B：切换 FULL_BODY 下 Y/B 禁用↔暂停
                markGripComboConsumed(true, true);
                handleCase16YbModeToggle();
                break;
            case 20: // 左摇杆 + 右摇杆同时按下（切换底盘/末端控制模式）
            {
                // 仅在 VR 控制已启用时才允许切换
                if (!enabled_.load())
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "🔘 [case 20] 忽略 - VR 控制未启用（enabled_=false）");
                    break;
                }
                toggleChassisMode();
                break;
            }
            case 21: // 左握把 + 左摇杆向上：竖直
                markGripComboConsumed(true, false);
                if (whole_body_teleop_active_.load())
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "🔘 [case 21] 全身遥操作已开启，忽略原有身体模式组合键（竖直）");
                    break;
                }
                requestBodyMode(
                    21, "左握把+左摇杆向上", "竖直", "BODY_RELATIVE",
                    arms_ros2_control_msgs::msg::WbcCurrentState::BODY_VERTICAL);
                break;
            case 22: // 左握把 + 左摇杆向下：锁定
                markGripComboConsumed(true, false);
                if (whole_body_teleop_active_.load())
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "🔘 [case 22] 全身遥操作已开启，忽略原有身体模式组合键（锁定）");
                    break;
                }
                requestBodyMode(
                    22, "左握把+左摇杆向下", "锁定", "BODY_LOCK",
                    arms_ros2_control_msgs::msg::WbcCurrentState::BODY_LOCKED);
                break;
            case 23: // 左握把 + 左摇杆向左：跟随
                markGripComboConsumed(true, false);
                if (whole_body_teleop_active_.load())
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "🔘 [case 23] 全身遥操作已开启，忽略原有身体模式组合键（跟随）");
                    break;
                }
                requestBodyMode(
                    23, "左握把+左摇杆向左", "跟随", "BODY_TRACKING",
                    arms_ros2_control_msgs::msg::WbcCurrentState::BODY_TRACKING);
                break;
            case 24: // 左握把 + 左摇杆向右：自定义
                markGripComboConsumed(true, false);
                if (whole_body_teleop_active_.load())
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "🔘 [case 24] 全身遥操作已开启，忽略原有身体模式组合键（自定义）");
                    break;
                }
                requestBodyMode(
                    24, "左握把+左摇杆向右", "自定义", "BODY_CUSTOM_LOCK",
                    arms_ros2_control_msgs::msg::WbcCurrentState::BODY_CUSTOM_LOCKED);
                break;
            case 25: // 右握把 + 右摇杆向上：双臂耦合
                markGripComboConsumed(false, true);
                requestWbcToggle(
                    25, "右握把+右摇杆向上", WbcToggleTarget::BIMANUAL);
                break;
            case 26: // 右握把 + 右摇杆向下：启用底盘
                markGripComboConsumed(false, true);
                requestWbcToggle(
                    26, "右握把+右摇杆向下", WbcToggleTarget::BASE);
                break;
            case 27: // 右握把 + 右摇杆向左：预留，暂不处理
                markGripComboConsumed(false, true);
                break;
            case 28: // 右握把 + 右摇杆向右：参考关节追踪
                markGripComboConsumed(false, true);
                requestWbcToggle(
                    28, "右握把+右摇杆向右", WbcToggleTarget::HOME_JOINT);
                break;
            case 30: // 左右握把 + 右摇杆按下：开关全身遥操作（骨架驱动身体/头部）
                markGripComboConsumed(true, true);
                toggleWholeBodyTeleop();
                break;
            case 31: // 右摇杆长按：全身模式内开关底盘跟踪（骨盆 → /cmd_vel）
                toggleBaseTracking();
                break;
            case 0:  // 无事件
            default:
                // 无按钮事件
                break;
        }
    }

    void VRInputHandler::detectControlTopology()
    {
        constexpr auto response_timeout = std::chrono::seconds(3);
        const auto now = std::chrono::steady_clock::now();

        if (pending_topology_request_)
        {
            if (pending_topology_request_->wait_for(std::chrono::seconds(0)) ==
                std::future_status::ready)
            {
                try
                {
                    const auto response = pending_topology_request_->get();
                    bool split_active = false;
                    bool full_active = false;
                    for (const auto& controller : response->controller)
                    {
                        if (controller.state != "active")
                        {
                            continue;
                        }
                        split_active |= controller.name == "ocs2_arm_controller";
                        full_active |= controller.name == "ocs2_wbc_controller";
                    }

                    ControlTopology next = ControlTopology::UNKNOWN;
                    if (full_active && !split_active)
                    {
                        next = ControlTopology::FULL_BODY;
                    }
                    else if (split_active && !full_active)
                    {
                        next = ControlTopology::SPLIT_BODY;
                    }
                    else if (split_active && full_active)
                    {
                        RCLCPP_ERROR_THROTTLE(
                            node_->get_logger(), *node_->get_clock(), 5000,
                            "Cannot determine VR control topology: "
                            "both main controllers are active");
                    }

                    const auto previous = control_topology_.exchange(next);
                    if (previous != next)
                    {
                        // WBC consumes grip/stick combinations as mode commands;
                        // only split control uses the direct waist velocity topics.
                        if (next == ControlTopology::SPLIT_BODY)
                        {
                            pub_waist_lifting_ = node_->create_publisher<std_msgs::msg::Float64>(
                                "/body_joint_controller/waist_lifting_command", 10);
                            pub_waist_turning_ = node_->create_publisher<std_msgs::msg::Float64>(
                                "/body_joint_controller/waist_turning_command", 10);
                        }
                        else
                        {
                            if (pub_waist_lifting_ || pub_waist_turning_)
                                resetChassisAndWaistCommands();
                            pub_waist_lifting_.reset();
                            pub_waist_turning_.reset();
                        }
                        clearLastPublishedTargets();
                        if (next != ControlTopology::FULL_BODY)
                        {
                            resetYbModeLatchAndConversions();
                            // 全身遥操只在 FULL_BODY 下有效：拓扑变化时退出整个模式
                            // （连带关掉「全部跟踪」与「底盘跟踪」），避免状态残留
                            if (whole_body_teleop_active_.load())
                            {
                                stopWholeBodyTeleop();
                                RCLCPP_INFO(node_->get_logger(),
                                            "🔘 控制拓扑不再是 FULL_BODY：全身遥操作已自动退出");
                            }
                        }
                        const char* name =
                            next == ControlTopology::FULL_BODY ? "FULL_BODY" :
                            next == ControlTopology::SPLIT_BODY ? "SPLIT_BODY" :
                            "UNKNOWN";
                        RCLCPP_INFO(
                            node_->get_logger(),
                            "VR control topology changed to %s", name);
                        RCLCPP_INFO(
                            node_->get_logger(),
                            "🕹️ Control topology changed; VR frame caches invalidated");
                    }

                    if (next != ControlTopology::UNKNOWN)
                    {
                        control_topology_timer_->cancel();
                    }
                }
                catch (const std::exception& error)
                {
                    control_topology_.store(ControlTopology::UNKNOWN);
                    RCLCPP_WARN_THROTTLE(
                        node_->get_logger(), *node_->get_clock(), 5000,
                        "Failed to read controller topology response: %s",
                        error.what());
                }

                pending_topology_request_.reset();
            }
            else if (now >= topology_request_deadline_)
            {
                list_controllers_client_->remove_pending_request(
                    pending_topology_request_->request_id);
                pending_topology_request_.reset();
                control_topology_.store(ControlTopology::UNKNOWN);
                RCLCPP_WARN_THROTTLE(
                    node_->get_logger(), *node_->get_clock(), 5000,
                    "Controller topology request timed out; retrying");
            }
            return;
        }

        if (!list_controllers_client_->service_is_ready())
        {
            control_topology_.store(ControlTopology::UNKNOWN);
            return;
        }

        try
        {
            auto request = std::make_shared<ListControllers::Request>();
            pending_topology_request_.emplace(
                list_controllers_client_->async_send_request(request));
            topology_request_deadline_ = now + response_timeout;
        }
        catch (const std::exception& error)
        {
            pending_topology_request_.reset();
            control_topology_.store(ControlTopology::UNKNOWN);
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 5000,
                "Failed to send controller topology request: %s", error.what());
        }
    }

    void VRInputHandler::fsmCommandCallback(std_msgs::msg::Int32::SharedPtr msg)
    {
        int32_t command = msg->data;
        
        // 忽略重置命令（command=0）
        if (command == 0)
        {
            return;
        }

        // REST姿态切换命令（不改变状态，只切换姿态）
        if (command == 100)
        {
            return;
        }

        const int32_t old_state = current_fsm_state_.load();
        std::string current_name;
        switch (old_state)
        {
        case 1:
            current_name = "HOME";
            break;
        case 3:
            current_name = "OCS2";
            break;
        case 4:
            current_name = "MOVEJ";
            break;
        case 2:
        default:
            current_name = "HOLD";
            break;
        }

        // 与 ArmsTargetManager 使用同一套合法转换表（含 MOVEJ=4）
        std::string new_name;
        if (!arms_controller_common::FSMStateTransitionValidator::validateTransition(
                current_name, command, new_name))
        {
            RCLCPP_DEBUG(node_->get_logger(),
                         "🕹️🕶️🕹️ Ignore invalid FSM transition: %s + command=%d",
                         current_name.c_str(), command);
            // 本地缓存与权威状态分歧时这条命令会被丢弃，但全身遥操作的退出判定必须
            // 依据权威状态走（否则模式会一直残留），这里兜底一次。
            exitWholeBodyTeleopIfFsmNotOcs2();
            return;
        }

        if (new_name != "OCS2")
        {
            stopHeadTracking();
        }
        int32_t new_state = old_state;
        if (new_name == "HOME")
        {
            new_state = 1;
        }
        else if (new_name == "HOLD")
        {
            new_state = 2;
        }
        else if (new_name == "OCS2")
        {
            new_state = 3;
        }
        else if (new_name == "MOVEJ")
        {
            new_state = 4;
        }

        if (new_state != old_state)
        {
            if (old_state == 2 && new_state == 3)
            {
                clearLastPublishedTargets();
                RCLCPP_INFO(node_->get_logger(),
                            "🕹️🕶️🕹️ HOLD → OCS2: cleared cached command targets; next UPDATE will anchor from current pose");
            }

            current_fsm_state_.store(new_state);
            
            // 如果当前状态不是OCS2，自动切换到存储模式
            if (new_state != 3)  // 3 = OCS2
            {
                if (is_update_mode_.load())
                {
                    is_update_mode_.store(false);
                    stopHeadTracking();
                    // 重置摇杆累积偏移
                    left_thumbstick_offset_ = Eigen::Vector3d::Zero();
                    right_thumbstick_offset_ = Eigen::Vector3d::Zero();
                    left_thumbstick_yaw_offset_ = 0.0;
                    right_thumbstick_yaw_offset_ = 0.0;
                    RCLCPP_INFO(node_->get_logger(), 
                                "🕹️🕶️🕹️ FSM状态不是OCS2，自动切换到STORAGE模式 (状态=%d)", new_state);
                    RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Thumbstick offsets reset!");
                }
            }
        }

        // 全身遥操作只在 OCS2 内有效：权威 FSM 一离开 OCS2（HOLD / HOME / MOVEJ，含控制器
        // 异常自动回退）就退出整个模式，与上面普通遥操作自动回 STORAGE 保持一致。
        exitWholeBodyTeleopIfFsmNotOcs2();
    }
} // namespace arms_ros2_control::command
