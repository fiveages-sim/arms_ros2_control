//
// VRInputHandler - VR 遥操作（普通模式 + 全身模式，单文件实现）
//
// 这个文件把原先分散在两处的实现合成一份，按“做事情的顺序”重排：
//
//   1  构造、生命周期与开关
//   2  按键与组合键分发        ← 想加组合键或功能，先看这里
//   3  手柄、头显与摇杆输入
//   4  普通模式：相对映射与目标发布
//   5  夹爪与手部命令
//   6  光学 R0 握手（可选协议）
//   7  WBC 门控与身体模式
//   8  底盘与腰部遥控
//   9  全身模式：模式开关与骨架输入
//  10  全身模式：映射、坐标与锚定
//  11  模式查询与共享工具
//
// 三条约束（改动前请先确认）：
//   * 目标只通过 ArmsTargetManager 的 TargetRouter 提交，不在这里直接发话题；
//   * 手部命令只通过 HandCommandBus 发送；
//   * 输入编号只在这里由 decodeAction() 翻译成动作，其他函数只处理动作。
//
// 组合键一览（编号→动作→允许的模式）：
//
//   11/12  FSM 前进/后退                     两种模式
//    9/10  左/右扳机（夹爪）                   两种模式
//   13/14  握把+扳机（夹爪模式切换）            两种模式
//   30     左右握把+右摇杆按下（全身模式开关）    两种模式
//   20     左右摇杆按下（底盘遥控）              两种模式
//   26     右握把+右摇杆向下（底盘解锁/锁定）     两种模式
//    4     右摇杆按下：普通=更新/存储，全身=全部跟踪  各自模式
//    3/6   左 Y / 右 B：普通=暂停/恢复，FULL_BODY=WBC 单臂开关
//    7     左摇杆按下（镜像开关）                普通
//   29     左握把+左摇杆按下（镜像方案切换）       普通
//   16     左右握把+左 Y+右 B（Y/B 语义切换）      普通
//   21-24  左握把+左摇杆 上/下/左/右（身体模式）    普通
//   25/28  右握把+右摇杆 上/右（耦合、参考关节）    普通
//   15     左 Y+右 B（尺度校准，当前禁用）         普通
//   27     右握把+右摇杆向左（预留，仅消费组合）     普通
//
// 新增组合键的完整步骤见 doc/05-扩展方法.md：
//   1) core/Action.h 里加动作与绑定；2) core/ModeRegistry.h 里加允许模式；
//   3) 下面第 2 节加一个 case（或一个处理函数）。
//
#include "arms_target_manager/VRInputHandler.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <limits>
#include <string_view>
#include <utility>

#include <tf2/exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace arms_ros2_control::command
{

    namespace
    {
        /** 固定轴 RPY（度）→ 旋转矩阵，与 URDF 的 rpy 约定一致：R = Rz·Ry·Rx。 */
        Eigen::Matrix3d rpyDegToMatrix(const std::vector<double>& rpy_deg)
        {
            if (rpy_deg.size() != 3)
            {
                return Eigen::Matrix3d::Identity();
            }
            const double rpy[3] = {rpy_deg[0], rpy_deg[1], rpy_deg[2]};
            return detail::rpyDegToMatrix(rpy);
        }

        /** 取四元数对应旋转的偏航角。 */
        double yawOfQuaternion(const Eigen::Quaterniond& q)
        {
            const Eigen::Matrix3d m = q.normalized().toRotationMatrix();
            return std::atan2(m(1, 0), m(0, 0));
        }

        /** 把名字列表拼成“a、b、c”，用于日志。 */
        std::string joinSources(const std::vector<std::string>& sources)
        {
            std::string joined;
            for (const auto& name : sources)
            {
                if (!joined.empty())
                {
                    joined += "、";
                }
                joined += name;
            }
            return joined;
        }
    } // namespace

    // ===== 冻结追赶判定的常量（实现见 detail::StaleCatchUpFilter）=====
    const int VRInputHandler::STALE_MIN_FROZEN_FRAMES = 2; // 连续 >=2 帧逐位相同即判定上游冻结

    VRInputHandler::VRInputHandler(
        rclcpp::Node::SharedPtr node,
        ArmsTargetManager* targetManager,
        const std::vector<std::string>& handControllers,
        double vr_thumbstick_linear_scale,
        double vr_thumbstick_angular_scale,
        double vr_pose_scale,
        const std::string& reference_link,
        const std::string& vr_follow_frame,
        const AbsoluteTeleopOptions& absolute_options)
        : node_(std::move(node))
          , target_manager_(targetManager)
          , enabled_(false)
          , is_update_mode_(false)
          , mirror_mode_(false)
          , left_arm_paused_(false)
          , right_arm_paused_(false)
          , left_grip_mode_(false)
          , right_grip_mode_(false)
          , chassis_mode_(false)
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
        // 创建 controller topology 检测 client 和启动期重试 timer
        list_controllers_client_ =
            node_->create_client<ListControllers>(
                "controller_manager/list_controllers");

        control_topology_timer_ = node_->create_wall_timer(
            std::chrono::seconds(1),
            [this]() { detectControlTopology(); });

        // Relative tf/tf_static (same as ATM/WBC) — multi-robot Isaac publishes /<ns>/tf.
        // 初始化 TF 组件（用于在 reference_link 与末端 frame 之间进行坐标变换）
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        std::string tf_topic = "tf";
        std::string tf_static_topic = "tf_static";
        if (!node_->has_parameter("tf_topic")) {
            node_->declare_parameter("tf_topic", tf_topic);
        }
        if (!node_->has_parameter("tf_static_topic")) {
            node_->declare_parameter("tf_static_topic", tf_static_topic);
        }
        tf_topic = node_->get_parameter("tf_topic").as_string();
        tf_static_topic = node_->get_parameter("tf_static_topic").as_string();
        tf_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
            tf_topic,
            rclcpp::QoS(rclcpp::KeepLast(100)).best_effort().durability_volatile(),
            [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
                for (const auto& t : msg->transforms) {
                    tf_buffer_->setTransform(t, "isaac_tf_authority", false);
                }
            });
        tf_static_sub_ = node_->create_subscription<tf2_msgs::msg::TFMessage>(
            tf_static_topic,
            rclcpp::QoS(rclcpp::KeepLast(100)).transient_local().reliable(),
            [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
                for (const auto& t : msg->transforms) {
                    tf_buffer_->setTransform(t, "isaac_tf_authority", true);
                }
            });
        RCLCPP_INFO(
            node_->get_logger(),
            "VR TF buffer on relative '%s'+'%s' (ns=%s); stale-input ramp enabled (frozen >= %d).",
            tf_topic.c_str(), tf_static_topic.c_str(), node_->get_namespace(),
            STALE_MIN_FROZEN_FRAMES);
        // 检测左右控制器名称
        detectGripperControllers(hand_controllers_);
        hand_bus_ = std::make_unique<HandCommandBus>(node_, hand_controllers_);
        
        // 创建夹爪状态订阅器（用于同步夹爪状态）
        if (!left_gripper_controller_name_.empty())
        {
            // Relative → /<ns>/<controller>/target_command (matches gripper ~/target_command)
            std::string left_topic = left_gripper_controller_name_ + "/target_command";
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
            std::string right_topic = right_gripper_controller_name_ + "/target_command";
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
        // 头部目标统一经 TargetRouter 发布，这里不再建自己的 head_target 发布器。

        // 创建按钮事件订阅器（Int32类型）
        auto controllerStateCallback = [this](const std_msgs::msg::Int32::SharedPtr msg)
        {
            this->processButtonEvent(msg);
        };
        sub_controller_state_ = node_->create_subscription<std_msgs::msg::Int32>(
            "/teleop/controller_state", 10, controllerStateCallback);

        pub_optical_r0_request_ = node_->create_publisher<std_msgs::msg::Int32>(
            "/teleop/optical_r0_request", 10);
        // Capability beacon is transient_local so ATM started later still learns
        // that this publisher speaks the handshake (see opticalR0CapsCallback).
        sub_optical_r0_caps_ = node_->create_subscription<std_msgs::msg::Int32>(
            "/teleop/optical_r0_caps",
            rclcpp::QoS(1).transient_local(),
            [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                this->opticalR0CapsCallback(msg);
            });
        sub_optical_r0_ready_ = node_->create_subscription<std_msgs::msg::Int32>(
            "/teleop/optical_r0_ready", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                this->opticalR0ReadyCallback(msg);
            });
        // Supervisor: no VR pose needed, so a dead peer still expires pendings.
        optical_handshake_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { this->superviseOpticalHandshake(); });
        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️ Optional optical R0 handshake "
            "(/teleop/optical_r0_caps|_request ↔ /teleop/optical_r0_ready; "
            "else main topic R0)");
        
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

        // 双臂目标由 ArmsTargetManager 的 TargetRouter 统一发布，这里不再重复创建。

        // FSM 命令不再由本类发布：统一走 FsmClient（节点注入）。

        // 底盘控制模式相关发布器（case 20 触发）
        pub_cmd_vel_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // 注意：mode_command 发布器与 wbc_capabilities 订阅已统一到 HumanoidModeClient，
        // 本类不再自己建（避免同进程重复订阅与两份缓存）。

        // Hand sinks / DualHand follow this latch (same bit as EE/trigger mirror).
        pub_mirror_mode_ = node_->create_publisher<std_msgs::msg::Bool>(
            "/teleop/mirror_mode",
            rclcpp::QoS(1).transient_local());
        publishMirrorMode();
        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️🕶️🕹️ Publishing /teleop/mirror_mode (Bool, transient_local)");

        // FSM 与 WBC 模式的订阅/发布都由统一入口接管（见 attachFsmClient /
        // attachHumanoidModeClient），本类只保留设备侧输入与映射。

        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VRInputHandler created（普通 + 全身同一份状态）");
        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️ VR frames: reference_link=%s, full_body_follow_frame=%s",
            reference_link_.c_str(),
            vr_follow_frame_.c_str());
        RCLCPP_INFO(node_->get_logger(),
                    "🕹️🕶️🕹️ Chassis mode (case 20 = L+R thumbstick): L.stick→chassis XY, R.stick→waist lift/turn");
        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️ FULL_BODY directions (normal mode only): "
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
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ MIRROR mode: toggled by left thumbstick button (normal mode only)");
        
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

        absolute_options_ = absolute_options;
        initializeWholeBodyTeleop();
    }

    // =====================================================================
    // 1. 构造、生命周期与开关
    // =====================================================================

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

    VRInputHandler::~VRInputHandler() = default;

    /**
     * @brief 使能 VR 控制
     *
     * 同时通知会话层，并把目标发布切到连续模式以获得更好的跟随性。
     */
    void VRInputHandler::enable()
    {
        enabled_.store(true);
        session_.setEnabled(true);
        // 启用VR控制时，自动设置为连续发布模式以获得更好的响应性
        if (target_manager_ && target_manager_->getCurrentMode() != MarkerState::CONTINUOUS)
        {
            target_manager_->togglePublishMode();
            RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ ArmsTargetManager switched to CONTINUOUS mode for VR control");
        }
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VR control ENABLED!");
    }

    /**
     * @brief 失能 VR 控制
     *
     * 交还所有目标控制权、停止头部跟踪、退出底盘/全身类运行态，
     * 避免失能后机器人还沿着上一条命令跑。
     */
    void VRInputHandler::disable()
    {
        enabled_.store(false);
        session_.setEnabled(false);
        // 交还所有目标通道，让摇杆/键盘和 RViz 能立即接手。
        if (target_manager_ && target_manager_->router())
        {
            target_manager_->router()->releaseAll(SourceId::Vr);
        }
        left_manual_override_.store(false, std::memory_order_relaxed);
        right_manual_override_.store(false, std::memory_order_relaxed);
        // 全身遥操随 VR 禁用一起退出（只做全身侧清理）
            deactivateWholeBodyTeleop();
        left_robot_base_valid_ = false;
        right_robot_base_valid_ = false;
        stopHeadTracking();
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


    // =====================================================================
    // 2. 按键与组合键分发
    // =====================================================================

    // 事件从 /teleop/controller_state 进来，处理顺序固定为三步：
    //   ① decodeAction()       设备编号 → 动作（core/Action.h 的绑定表）
    //   ② VrSession::allows()  当前模式是否允许这个动作（core/ModeRegistry.h）
    //   ③ 下面的 switch         动作 → 具体处理
    //
    // 因此新增一个组合键只要改三处，且都不会影响其它组合键：
    //   core/Action.h     加动作 + 一行绑定（编号、需要/禁止的修饰键）
    //   core/ModeRegistry 把动作加进允许集合
    //   本区             加一个 case 调用新的处理函数
    //
    // 处理函数请只接收“动作”，不要再判断编号；编号到动作的翻译只发生在 ①。
    /**
     * @brief 按键事件总入口
     *
     * 处理顺序固定为三步，新增组合键只改第一步的绑定表与第三步的分发：
     *   ① decodeAction() 把设备编号翻成动作；
     *   ② VrSession::allows() 判断当前模式是否允许该动作；
     *   ③ switch 分发给具体处理函数。
     *
     * @param msg 事件编号（0 表示无事件）
     */
    void VRInputHandler::processButtonEvent(const std_msgs::msg::Int32::SharedPtr msg)
    {
        const int32_t code = msg->data;
        // /teleop/controller_state 每帧都发 0（无事件）
        if (code == 0)
        {
            return;
        }

        // 兜底：FSM 可能由外部来源切走，低频入口处再确认一次
        onWholeBodyFsmStateChanged();

        // 这些编号由头显侧独占处理，本节点不消费。
        // 27 是「右握把 + 右摇杆向左」的预留组合键：功能未实现，
        // 但必须标记组合键已消费，否则松手会被当成握把短按。
        if (code == 27)
        {
            markGripComboConsumed(false, true);
            return;
        }
        if (code == 1 || code == 2 || code == 5)
        {
            return;
        }

        // 语义面：设备编号 → 动作
        Action action = decodeAction(code);
        if (action == Action::None)
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 5000,
                "🔘 未知按键事件 %d", code);
            return;
        }

        // 同一个键在全身模式下的含义
        if (action == Action::UpdateToggle && wholeBodyActive())
        {
            action = Action::AbsoluteTrackingToggle;
        }

        // 会话面：把当前实际状态同步给会话，再判断是否允许
        session_.setRelativeUpdate(is_update_mode_.load());
        session_.setOverlay(OverlayId::Chassis, chassis_mode_.load());
        if (!session_.allows(action))
        {
            RCLCPP_INFO_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 3000,
                "🔘 %s（事件 %d）在当前模式「%s」不可用",
                actionName(action), code, modeName(session_.mode()));
            return;
        }

        // 动作处理只有一份；模式差异只体现在“谁来执行”
        switch (action)
        {
            case Action::FsmNext:
            case Action::FsmBack:
                handleFsmButtonEvent(code);
                return;
            case Action::HandSwitchLeft:
            case Action::HandSwitchRight:
            case Action::HandModeToggleLeft:
            case Action::HandModeToggleRight:
                handleGripperButtonEvent(code);
                return;
            case Action::WholeBodyToggle:
                markGripComboConsumed(true, true);
                toggleWholeBodyTeleop();
                return;
            case Action::AbsoluteTrackingToggle:
                handleWholeBodyButtonEvent(code);
                return;
            case Action::ChassisToggle:
            case Action::BaseToggle:
                handleNormalAction(action);
                return;
            default:
                break;
        }

        if (wholeBodyActive())
        {
            handleWholeBodyButtonEvent(code);
            return;
        }
        handleNormalAction(action);
    }


    /**
     * @brief 普通模式的动作分发
     *
     * 只处理“普通模式独有”的动作；FSM、夹爪、全身开关等共用动作在上一级处理。
     *
     * @param action 已通过会话层允许性检查的动作
     */
    void VRInputHandler::handleNormalAction(Action action)
    {
        switch (action)
        {
            case Action::ArmToggleLeft:  // 原事件 3
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
                        resumeArmVrControl(
                            WbcToggleTarget::RIGHT_ARM, "左Y按钮");
                    }
                    else
                    {
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
                        resumeArmVrControl(
                            WbcToggleTarget::LEFT_ARM, "左Y按钮");
                    }
                    else
                    {
                        paused_left_position_ = left_position_;
                        paused_left_orientation_ = left_orientation_;
                        left_arm_paused_.store(true);
                        RCLCPP_INFO(node_->get_logger(), "🔘 [左Y按钮] 按下 - 功能: 切换左臂更新状态 - 操作: 暂停左臂更新（已记录暂停时刻VR位姿，摇杆可继续控制）");
                    }
                }
                break;
            }
            case Action::UpdateToggle:  // 原事件 4
            {
                // 只在OCS2状态下执行
                if (!isOcs2State())
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
                    // Optical handshake: request I+p_a, wait ready, snap R0=I.
                    // Arm on capability (not on live ready): a still headset
                    // emits no new frames, so ready would look dead and we would
                    // silently fall back to the offset-bearing topic R0. The 3s
                    // contract timeout is the deadline if R=I never arrives.
                    if (opticalR0HandshakeSupported())
                    {
                        optical_update_enter_pending_.store(true);
                        optical_update_enter_pending_since_ =
                            std::chrono::steady_clock::now();
                        optical_fallback_warned_.store(false);
                        refreshOpticalR0Request();
                        RCLCPP_INFO(
                            node_->get_logger(),
                            "🕹️ UPDATE enter pending optical R0 handshake");
                        tryCompleteOpticalUpdateEnter();
                        break;
                    }
                    optical_update_enter_pending_.store(false);
                    // main: R0 = topic (match origin/main)
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
                    optical_update_enter_pending_.store(false);
                    refreshOpticalR0Request();
                    is_update_mode_.store(false);
                    stopHeadTracking();
                    RCLCPP_INFO(node_->get_logger(), "🔘 [右摇杆按钮] 按下 - 功能: 切换UPDATE/STORAGE模式 - 操作: 切换到STORAGE模式（准备存储新的基准位姿）");
                }
                break;
            }
            case Action::ArmToggleRight:  // 原事件 6
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
                        resumeArmVrControl(
                            WbcToggleTarget::LEFT_ARM, "右B按钮");
                    }
                    else
                    {
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
                        resumeArmVrControl(
                            WbcToggleTarget::RIGHT_ARM, "右B按钮");
                    }
                    else
                    {
                        paused_right_position_ = right_position_;
                        paused_right_orientation_ = right_orientation_;
                        right_arm_paused_.store(true);
                        RCLCPP_INFO(node_->get_logger(), "🔘 [右B按钮] 按下 - 功能: 切换右臂更新状态 - 操作: 暂停右臂更新（已记录暂停时刻VR位姿，摇杆可继续控制）");
                    }
                }
                break;
            }
            case Action::MirrorProfileToggle:  // 原事件 29
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
            case Action::MirrorToggle:  // 原事件 7
            {
                bool old_mirror_mode = mirror_mode_.load();
                bool new_mirror_mode = !old_mirror_mode;
                mirror_mode_.store(new_mirror_mode);
                publishMirrorMode();
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
            case Action::ScaleCalibrate:  // 原事件 15
            {
                // runScaleCalibration(true);
                break;
            }
            case Action::YbSemanticsToggle:  // 原事件 16
                markGripComboConsumed(true, true);
                handleCase16YbModeToggle();
                break;
            case Action::ChassisToggle:  // 原事件 20
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
            case Action::BodyVertical:  // 原事件 21
                markGripComboConsumed(true, false);
                requestBodyMode(
                    21, "左握把+左摇杆向上", "竖直", "BODY_RELATIVE",
                    arms_ros2_control_msgs::msg::WbcCurrentState::BODY_VERTICAL);
                break;
            case Action::BodyLocked:  // 原事件 22
                markGripComboConsumed(true, false);
                requestBodyMode(
                    22, "左握把+左摇杆向下", "锁定", "BODY_LOCK",
                    arms_ros2_control_msgs::msg::WbcCurrentState::BODY_LOCKED);
                break;
            case Action::BodyTracking:  // 原事件 23
                markGripComboConsumed(true, false);
                requestBodyMode(
                    23, "左握把+左摇杆向左", "跟随", "BODY_TRACKING",
                    arms_ros2_control_msgs::msg::WbcCurrentState::BODY_TRACKING);
                break;
            case Action::BodyCustom:  // 原事件 24
                markGripComboConsumed(true, false);
                requestBodyMode(
                    24, "左握把+左摇杆向右", "自定义", "BODY_CUSTOM_LOCK",
                    arms_ros2_control_msgs::msg::WbcCurrentState::BODY_CUSTOM_LOCKED);
                break;
            case Action::BimanualToggle:  // 原事件 25
                markGripComboConsumed(false, true);
                requestWbcToggle(
                    25, "右握把+右摇杆向上", WbcToggleTarget::BIMANUAL);
                break;
            case Action::BaseToggle:  // 原事件 26
                markGripComboConsumed(false, true);
                requestWbcToggle(
                    26, "右握把+右摇杆向下", WbcToggleTarget::BASE);
                break;
            case Action::HomeJointToggle:  // 原事件 28
                markGripComboConsumed(false, true);
                requestWbcToggle(
                    28, "右握把+右摇杆向右", WbcToggleTarget::HOME_JOINT);
                break;
            default:
                // 无按钮事件
                break;
        }
    }


    /**
     * @brief 全身模式内的事件分发
     *
     * 全身模式下只保留 FSM、夹爪、事件 4（全部跟踪）等少数功能，
     * 其余编号会打一条限频提示，告诉用户要先退出全身模式。
     *
     * @param code 原始事件编号
     */
    void VRInputHandler::handleWholeBodyButtonEvent(int32_t code)
    {
        switch (code)
        {
            case 4:  // 右摇杆短按：开/关「全部跟踪」
                toggleAbsoluteTracking();
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


    /**
     * @brief FSM 前进/后退
     *
     * 设备编号只在这里翻译成意图；“当前状态能不能走”由 FsmClient 统一判定，
     * 因此不会再出现各处规则表不一致的情况。
     *
     * @param code 11=右A（前进），其它按左X（后退/切换）处理
     */
    void VRInputHandler::handleFsmButtonEvent(int32_t code)
    {
        // 设备编号只在这里翻译成意图；"这个状态下能不能走"由 FsmClient 统一判定，
        // 因此不会再出现 VR 用一份规则表、ATM 用另一份的情况。
        if (code == 11)
        {
            // 右A：FSM 前进
            RCLCPP_INFO(node_->get_logger(),
                        "🔘 [右A按钮] 按下 - 功能: FSM状态前进（当前 %s）",
                        fsm_ ? fsm_->stateName() : "HOLD");
            requestFsmIntent(FsmIntent::Next);
            return;
        }

        // 左X：FSM 后退/切换（HOME 下再按一次 = 切换预设姿态）
        RCLCPP_INFO(node_->get_logger(),
                    "🔘 [左X按钮] 按下 - 功能: FSM状态后退/切换（当前 %s）",
                    fsm_ ? fsm_->stateName() : "HOLD");
        requestFsmIntent(FsmIntent::Back);
    }


    /**
     * @brief 夹爪相关按键分发
     *
     * 9/10 是扳机开关命令，13/14 是握把+扳机切比例/开关模式。
     * 比例模式下的 9/10 会被忽略，因为连续量由 triggerValuesCallback 驱动。
     *
     * @param code 原始事件编号
     */
    void VRInputHandler::handleGripperButtonEvent(int32_t code)
    {
        if (code == 9 || code == 10)
        {
            const bool is_left_trigger = (code == 9);
            std::string controller_name;
            bool is_target_left_arm = true;
            resolveTriggerTarget(is_left_trigger, controller_name, is_target_left_arm);
            if (isGripperPercentMode(is_target_left_arm))
            {
                RCLCPP_DEBUG(node_->get_logger(),
                             "🔘 [%s扳机按钮] 目标%s夹爪为比例控制(0~1)，忽略开关事件%s",
                             is_left_trigger ? "左" : "右",
                             is_target_left_arm ? "左" : "右",
                             mirror_mode_.load() ? " [镜像模式]" : "");
            }
            else
            {
                toggleGripperByTrigger(is_left_trigger);
            }
            return;
        }

        // 13/14：握把+扳机，切换该侧夹爪百分比/开合
        const bool is_left_trigger = (code == 13);
        markGripComboConsumed(is_left_trigger, !is_left_trigger);
        toggleGripperPercentMode(is_left_trigger);
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


    // =====================================================================
    // 3. 手柄、头显与摇杆输入
    // =====================================================================

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

    void VRInputHandler::robotBodyPoseCallback(
        geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
    {
        // 只作为全身接管渐入的锚点，不参与映射计算：
        // 无效时把有效标志清掉，让渐入退回「1 s 内最后下发的目标」。
        if (!msg)
        {
            has_robot_body_pose_ = false;
            return;
        }
        const auto& p = msg->pose.position;
        const auto& q = msg->pose.orientation;
        const Eigen::Quaterniond orientation(q.w, q.x, q.y, q.z);
        if (msg->header.frame_id.empty() ||
            !Eigen::Vector3d(p.x, p.y, p.z).allFinite() ||
            !orientation.coeffs().allFinite() || orientation.norm() < 1e-9)
        {
            has_robot_body_pose_ = false;
            return;
        }
        robot_body_pose_ = *msg;
        has_robot_body_pose_ = true;
    }

    void VRInputHandler::vrHeadCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        // 全身模式的头部由骨架 /teleop/body_head_pose 驱动，头显不参与
        if (isWholeBodyActive())
        {
            return;
        }

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
            if (!isWholeBodyActive())
            {
                stopHeadTracking();
            }
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
            isOcs2State() &&
            target_manager_->getCurrentHeadState() ==
                arms_ros2_control_msgs::msg::WbcCurrentState::HEAD_TRACKING &&
            has_vr_head_pose_ && has_robot_head_pose_;
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

        // 普通遥操作：头显（/teleop/head_pose）驱动 WBC 头部目标的相对增量路径
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

        TargetRouter* router = target_manager_ ? target_manager_->router() : nullptr;
        if (!router)
        {
            stopHeadTracking();
            return;
        }

        Intent intent;
        intent.channel = Channel::Head;
        intent.source = SourceId::Vr;
        intent.priority = Priority::Normal;
        intent.position = publishedPosition;
        intent.orientation = publishedOrientation;
        intent.frame = frame;
        intent.lease = std::chrono::milliseconds(300);
        if (!router->submit(intent).accepted)
        {
            // RViz 正在手动拖头部时让位；下一帧重新建立基准后再接管。
            stopHeadTracking();
            return;
        }

        target_manager_->setHeadVrActive(true);
        last_head_target_ = target;
        last_head_target_stamp_ = std::chrono::steady_clock::now();
        head_tracking_active_ = true;
    }

    /**
     * @brief VR 左手柄位姿输入
     *
     * 镜像模式下这里的数据驱动右臂，因此不要在下游再按左右重新判断。
     */
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

        // 光学 R0 握手：待完成的 UPDATE 进入 / 单臂恢复靠手柄帧推进，
        // 与全身模式无关，必须保持在早退之前。
        tryCompleteOpticalUpdateEnter();
        tryCompleteOpticalResumes();

        // 全身模式的目标由骨架绝对映射发布，普通手柄不参与
        if (isWholeBodyActive())
        {
            return;
        }

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

            // 全身「全部跟踪」由本类第 9 区处理；这里只处理普通模式。
            if (is_update_mode_.load())
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

    /**
     * @brief VR 右手柄位姿输入（镜像语义与左手对称）
     */
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

        // 光学 R0 握手：待完成的 UPDATE 进入 / 单臂恢复靠手柄帧推进，
        // 与全身模式无关，必须保持在早退之前。
        tryCompleteOpticalUpdateEnter();
        tryCompleteOpticalResumes();

        // 全身模式的目标由骨架绝对映射发布，普通手柄不参与
        if (isWholeBodyActive())
        {
            return;
        }

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

            // 全身「全部跟踪」由本类第 9 区处理；这里只处理普通模式。
            if (is_update_mode_.load())
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

    /**
     * @brief 摇杆轴值输入（linear.x/y = 左摇杆，angular.x/y = 右摇杆）
     *
     * 根据当前模式分发给底盘遥控、身体模式或末端微调，本身不做积分。
     */
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

        // 全身模式：只同步轴值与握把 bit，不跑握把短按/方向抑制/腰部/末端等普通模式逻辑；
        // 事件 20 的手动底盘遥控与事件 26 的底盘解锁/锁定仍可用。
        if (isWholeBodyActive())
        {
            // FSM 可能由外部来源切走，摇杆消息是高频入口，这里顺带做一次兜底退出
            onWholeBodyFsmStateChanged();
            // 事件 26 的请求同样要等 WBC 确认并等右摇杆回中，否则 pending 永远不清。
            if (updateRightWbcToggleRequestState())
            {
                right_thumbstick_axes_.setZero();
            }
            if (isWholeBodyActive() && chassis_mode_.load())
            {
                // 全身模式下右握把属于 WBC/全身组合键的修饰符，不作为腰部修饰符；
                // 按住时屏蔽右摇杆，避免组合键动作同时驱动底盘转向。
                if (right_grip_active_.load())
                {
                    right_thumbstick_axes_.setZero();
                }
                processChassisAxes();
            }
            return;
        }

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

    /**
     * @brief 扳机模拟量输入（linear.x = 左扳机，angular.x = 右扳机）
     *
     * 比例模式下驱动夹爪百分比；开关模式只取上升沿做开/关，避免持续发指令。
     */
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

    /**
     * @brief 把摇杆增量叠加到某条手臂的 marker 上
     *
     * 内部维护累积偏移（平移 + 偏航），重启时清零，避免增量丢失或跳变。
     */
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

    /**
     * @brief 右摇杆末端微调（语义与左侧对称，镜像时作用于另一条臂）
     */
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

    /**
     * @brief BODY_TRACKING 下用左摇杆微调腰部 marker
     *
     * XY 模式：Y→X、X→Y；Z+Yaw 模式：Y→Z、X→Yaw。
     */
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


    // =====================================================================
    // 4. 普通模式：相对映射与目标发布
    // =====================================================================

    void VRInputHandler::publishMirrorMode()
    {
        if (!pub_mirror_mode_)
        {
            return;
        }
        std_msgs::msg::Bool msg;
        msg.data = mirror_mode_.load();
        pub_mirror_mode_->publish(msg);
    }

    /**
     * @brief 尺度校准（事件 15，当前入口固定用左侧）
     *
     * 用头显与手柄的实际距离校正比例，使虚拟位移和机器人末端位移一致。
     */
    void VRInputHandler::runScaleCalibration(bool use_left_z)
    {
        const char* tag = use_left_z ? "左组合键" : "右组合键";

        mirror_mode_.store(false);
        publishMirrorMode();
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

            TargetRouter* router = target_manager_ ? target_manager_->router() : nullptr;
            if (router)
            {
                // 计算完毕，转回发布系（ee_frame_id_）再提交。
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

                Intent left_intent;
                left_intent.channel = Channel::ArmLeft;
                left_intent.source = SourceId::Vr;
                left_intent.priority = Priority::Normal;
                left_intent.position = left_pub_pos;
                left_intent.orientation = left_pub_ori;
                left_intent.frame = ee_frame_id_;
                left_intent.lease = std::chrono::milliseconds(500);

                Intent right_intent = left_intent;
                right_intent.channel = Channel::ArmRight;
                right_intent.position = right_pub_pos;
                right_intent.orientation = right_pub_ori;

                const bool left_ok = router->submit(left_intent).accepted;
                const bool right_ok = router->submit(right_intent).accepted;
                if (!left_ok || !right_ok)
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "🕹️🕶️🕹️ [%s] 校准目标未能全部下发（可能被手动拖动占用）",
                                tag);
                }
                if (router->hasPublisher(Channel::ArmRight))
                {
                    target_manager_->sendDualArmTargetPose();
                }
                RCLCPP_INFO(node_->get_logger(),
                            "🕹️🕶️🕹️ [%s] 已下发校准目标 (frame: %s): "
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
                // prev_calculated_* 存发布系的值。
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
        // 坐标系/拓扑发生变化：同步清掉全身组件的渐入锚点
            onWholeBodyTargetCacheCleared();
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

    /**
     * @brief 按当前 VR 与机器人基准计算相对目标并交给发布函数
     *
     * 基准因 TF 缺失尚未就绪时会自动重试建立，因此调试期丢 TF 不会永久卡死。
     *
     * @return false 表示本次没有发布
     */
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

    void VRInputHandler::rebaseAfterManualOverride(bool is_left)
    {
        const std::string armType = is_left ? "left" : "right";
        const bool useLeftVr = mirror_mode_.load() ? !is_left : is_left;
        const bool paused = is_left ? left_arm_paused_.load() : right_arm_paused_.load();

        if (useLeftVr)
        {
            vr_base_left_position_ = paused ? paused_left_position_ : left_position_;
            vr_base_left_orientation_ = paused ? paused_left_orientation_ : left_orientation_;
        }
        else
        {
            vr_base_right_position_ = paused ? paused_right_position_ : right_position_;
            vr_base_right_orientation_ = paused ? paused_right_orientation_ : right_orientation_;
        }

        if (is_left)
        {
            left_thumbstick_offset_.setZero();
            left_thumbstick_yaw_offset_ = 0.0;
        }
        else
        {
            right_thumbstick_offset_.setZero();
            right_thumbstick_yaw_offset_ = 0.0;
        }

        setRobotBase(armType, /*preferLastCommand=*/true);
        resetStaleCatchUpRamp(is_left);
        (is_left ? left_manual_override_ : right_manual_override_)
            .store(false, std::memory_order_relaxed);
        RCLCPP_INFO(node_->get_logger(),
                    "🕹️ %s 臂从手动拖动恢复：已按最后下发目标重建基准",
                    armType.c_str());
    }

    /**
     * @brief 把控制器坐标系下的目标经平滑后发表
     *
     * @param armType "left" 或 "right"
     * @param position 控制器坐标系下的位置
     * @param orientation 控制器坐标系下的姿态
     * @return false 表示首次重建、TF 失败或发布器不可用
     */
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
        TargetRouter* router = target_manager_ ? target_manager_->router() : nullptr;
        if ((!isLeft && !isRight) || !router ||
            !router->hasPublisher(isLeft ? Channel::ArmLeft : Channel::ArmRight))
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "🕹️ Invalid armType or publisher not initialized: %s",
                armType.c_str());
            return false;
        }

        // 手动拖动优先：RViz 正在拖这条手臂时先让出控制权；
        // 拖动结束后从“最后一次实际下发的目标”重建基准，避免跳变。
        std::atomic<bool>& manual_hold = isLeft
            ? left_manual_override_
            : right_manual_override_;
        if (manual_hold.load(std::memory_order_relaxed))
        {
            TargetRouter* router = target_manager_ ? target_manager_->router() : nullptr;
            const Channel channel = isLeft ? Channel::ArmLeft : Channel::ArmRight;
            if (!router || !router->canWrite(channel, SourceId::Vr))
            {
                return false;  // 仍在手动拖动中，安静等待
            }
            rebaseAfterManualOverride(isLeft);
            return false;
        }

        // 上游 XR 位姿冻结后会一帧补齐整段位移，先把追赶量摊到随后若干帧。
        // 在控制器坐标系中平滑，缓存保存实际发布的目标。
        Eigen::Vector3d rampedPosition = position;
        Eigen::Quaterniond rampedOrientation = orientation;
        applyStaleCatchUpRamp(isLeft, rampedPosition, rampedOrientation);

        if (!ee_frame_id_initialized_ || !target_manager_)
        {
            return false;
        }

        Intent intent;
        intent.channel = isLeft ? Channel::ArmLeft : Channel::ArmRight;
        intent.source = SourceId::Vr;
        intent.priority = Priority::Normal;
        intent.position = rampedPosition;
        intent.orientation = rampedOrientation.normalized();
        intent.frame = ee_frame_id_;
        intent.lease = std::chrono::milliseconds(300);

        const Verdict verdict = router->submit(intent);
        if (!verdict.accepted)
        {
            if (std::string_view(verdict.reason) == "manual_override")
            {
                manual_hold.store(true, std::memory_order_relaxed);
            }
            RCLCPP_DEBUG_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "🕹️ %s 臂目标未下发：%s", armType.c_str(), verdict.reason);
            return false;
        }

        recordLastPublishedTarget(armType, rampedPosition, rampedOrientation);
        Eigen::Vector3d& previousPosition = isLeft
            ? prev_calculated_left_position_
            : prev_calculated_right_position_;
        Eigen::Quaterniond& previousOrientation = isLeft
            ? prev_calculated_left_orientation_
            : prev_calculated_right_orientation_;
        previousPosition = rampedPosition;
        previousOrientation = rampedOrientation.normalized();
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

    void VRInputHandler::applyStaleCatchUpRamp(bool isLeft,
                                         Eigen::Vector3d& position,
                                         Eigen::Quaterniond& orientation)
    {
        (isLeft ? left_stale_ramp_ : right_stale_ramp_).apply(position, orientation);
    }

    void VRInputHandler::resetStaleCatchUpRamp(bool isLeft)
    {
        (isLeft ? left_stale_ramp_ : right_stale_ramp_).reset();
    }

    void VRInputHandler::resetStaleCatchUpRamp()
    {
        left_stale_ramp_.reset();
        right_stale_ramp_.reset();
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
        }
        else if (armType == "right")
        {
            has_last_published_right_target_ = false;
            last_published_right_position_.setZero();
            last_published_right_orientation_.setIdentity();
            last_published_right_stamp_ = std::chrono::steady_clock::time_point{};
            resetStaleCatchUpRamp(false);
        }
    }


    // =====================================================================
    // 5. 夹爪与手部命令
    // =====================================================================

    void VRInputHandler::publishGripperCommand(const std::string& controller_name, int32_t command)
    {
        if (controller_name.empty())
        {
            return;
        }

        // 创建发布器（如果还没有）- 类似ControlInputHandler的方式
        hand_bus_->publishSwitch(controller_name, command == 1);

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

        hand_bus_->publishPercent(controller_name, percent);
    }

    bool VRInputHandler::isGripperPercentMode(bool is_target_left_arm) const
    {
        return is_target_left_arm ? left_gripper_percent_mode_.load()
                                  : right_gripper_percent_mode_.load();
    }

    /**
     * @brief 切到比例/开关夹爪模式
     *
     * 切比例：不发任何指令，夹爪停在当前位，等扳机回扣到该位置才接管；
     * 切开关：把开关标志对齐到当前位置，否则下一次 9/10 方向会反。
     */
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

    /**
     * @brief 切换夹爪控制模式（事件 13/14，带 500 ms 防抖）
     */
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


    // =====================================================================
    // 6. 光学 R0 握手（可选协议）
    // =====================================================================

    void VRInputHandler::opticalR0ReadyCallback(
        const std_msgs::msg::Int32::SharedPtr msg)
    {
        optical_r0_ready_bits_.store(msg->data);
        optical_r0_ready_last_ns_.store(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
        // Once caps has been seen, only caps may change support. Legacy
        // publishers that never send caps may still learn from ready.
        if (optical_caps_seen_.load())
        {
            return;
        }
        if (!optical_r0_peer_supports_.exchange(true))
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "🕹️ Optical R0 handshake peer detected "
                "(ready traffic, no caps yet); R0 handshake-driven until "
                "caps arrives or timeout&&alive unlearns");
        }
    }

    void VRInputHandler::opticalR0CapsCallback(
        const std_msgs::msg::Int32::SharedPtr msg)
    {
        const int32_t caps = msg ? msg->data : 0;
        optical_caps_seen_.store(true);
        const bool supports = (caps & optical_r0_caps_version_) != 0;
        const bool was = optical_r0_peer_supports_.exchange(supports);
        // A different publisher (or a restart) invalidates any contract level
        // and liveness left over from the previous process: drop them so the
        // first toggle cannot snap on a stale R=I.
        optical_r0_ready_bits_.store(0);
        optical_r0_ready_last_ns_.store(0);
        if (supports == was)
        {
            return;
        }
        if (supports)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "🕹️ Optical R0 handshake advertised by publisher "
                "(caps=%d); R0 becomes handshake-driven once ready flows",
                caps);
        }
        else
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "🕹️ Optical R0 handshake cleared by publisher "
                "(caps=%d); next toggle uses topic R0",
                caps);
        }
    }

    bool VRInputHandler::opticalR0HandshakeSupported() const
    {
        return optical_r0_peer_supports_.load();
    }

    bool VRInputHandler::opticalR0HandshakeAlive() const
    {
        const int64_t last = optical_r0_ready_last_ns_.load();
        if (last <= 0)
        {
            return false;
        }
        const int64_t now_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count();
        return (now_ns - last) <=
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                optical_r0_ready_alive_window_)
                .count();
    }

    /**
     * @brief 取消所有光学握手 pending，但不回滚手臂暂停状态
     */
    void VRInputHandler::cancelOpticalPending(const char* reason)
    {
        const bool hadUpdate = optical_update_enter_pending_.exchange(false);
        const bool hadLeft = optical_resume_pending_left_.exchange(false);
        const bool hadRight = optical_resume_pending_right_.exchange(false);
        if (!hadUpdate && !hadLeft && !hadRight)
        {
            return;
        }
        // ACK-open the publisher: request 0 lets fa-py resume world-Δ streaming
        // instead of sitting sticky on I after we gave up.
        publishOpticalR0Request(0);
        if (!optical_fallback_warned_.exchange(true))
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "🕹️ Optical R0 handshake %s — cancelled pending "
                "(UPDATE=%d L=%d R=%d)",
                reason,
                static_cast<int>(hadUpdate),
                static_cast<int>(hadLeft),
                static_cast<int>(hadRight));
        }
    }

    void VRInputHandler::unlearnOpticalR0Peer(const char* reason)
    {
        if (!optical_r0_peer_supports_.exchange(false))
        {
            return;
        }
        RCLCPP_WARN(
            node_->get_logger(),
            "🕹️ Optical R0 handshake unlearned (%s); next toggle uses "
            "topic R0 until caps=%d returns",
            reason,
            optical_r0_caps_version_);
    }

    /**
     * @brief 光学 R0 握手 100 ms 监管器
     *
     * 即使 VR 位姿一帧也不来（头显断流），也要能把 pending 推完或超时放弃，
     * 否则 pending 会一直卡住，false 地挡住后续操作。
     */
    void VRInputHandler::superviseOpticalHandshake()
    {
        const bool anyPending = optical_update_enter_pending_.load() ||
            optical_resume_pending_left_.load() ||
            optical_resume_pending_right_.load();
        if (!anyPending)
        {
            optical_fallback_warned_.store(false);
            return;
        }
        // Do NOT cancel on the 750ms alive window here: ready only flows on new
        // VR frames, so a paused headset legitimately goes quiet. The 3s
        // contract timeout inside tryComplete* is the real deadline.
        if (opticalR0HandshakeSupported() && opticalR0HandshakeAlive())
        {
            refreshOpticalR0Request();
        }
        tryCompleteOpticalUpdateEnter();
        tryCompleteOpticalResumes();
    }

    void VRInputHandler::publishOpticalR0Request(int32_t bits)
    {
        if (!pub_optical_r0_request_)
        {
            return;
        }
        optical_r0_request_bits_.store(bits);
        std_msgs::msg::Int32 msg;
        msg.data = bits;
        pub_optical_r0_request_->publish(msg);
    }

    int32_t VRInputHandler::desiredOpticalR0RequestBits() const
    {
        if (optical_update_enter_pending_.load())
        {
            return 3;
        }
        int32_t bits = 0;
        // Use the latched VR side when a resume is pending: mirror may flip
        // while waiting, and the publisher's bit means "which wrist frame",
        // not "which arm". WBC rebase alone keeps the original mirror formula.
        if (optical_resume_pending_left_.load())
        {
            bits |= optical_resume_use_left_vr_left_.load() ? 1 : 2;
        }
        else if (left_wbc_rebase_pending_.load())
        {
            bits |= left_wbc_rebase_use_left_vr_.load() ? 1 : 2;
        }
        if (optical_resume_pending_right_.load())
        {
            bits |= optical_resume_use_left_vr_right_.load() ? 1 : 2;
        }
        else if (right_wbc_rebase_pending_.load())
        {
            bits |= right_wbc_rebase_use_left_vr_.load() ? 1 : 2;
        }
        return bits;
    }

    void VRInputHandler::refreshOpticalR0Request()
    {
        publishOpticalR0Request(desiredOpticalR0RequestBits());
    }

    bool VRInputHandler::tryCompleteOpticalSideSnap(bool useLeftVr)
    {
        const int32_t need = useLeftVr ? 1 : 2;
        // Consume the bit: ready is a per-request contract, so the next request
        // must wait for a fresh R=I frame instead of reusing this level.
        if ((optical_r0_ready_bits_.fetch_and(~need) & need) == 0)
        {
            return false;
        }
        if (useLeftVr)
        {
            vr_base_left_position_ = left_position_;
            vr_base_left_orientation_ = Eigen::Quaterniond::Identity();
        }
        else
        {
            vr_base_right_position_ = right_position_;
            vr_base_right_orientation_ = Eigen::Quaterniond::Identity();
        }
        return true;
    }

    /**
     * @brief 尝试完成“进入 UPDATE”的光学握手
     *
     * 收到 ready 后把 R0 定在“手柄当前位姿”，使进入更新模式时不会因为
     * 话题回路里的偏置而跳一段距离；超时则退回主话题路径。
     */
    void VRInputHandler::tryCompleteOpticalUpdateEnter()
    {
        if (!optical_update_enter_pending_.load() || is_update_mode_.load())
        {
            return;
        }
        if (!isOcs2State())
        {
            // Leaving OCS2 is not a handshake fault: drop the pending silently
            // and keep the handshake armed for the next attempt.
            cancelOpticalPending("left OCS2 during UPDATE enter");
            return;
        }
        // Expiry must not depend on ready traffic: a dead publisher would
        // otherwise leave UPDATE-enter pending forever (and auto-fire on
        // reconnect, which moves the arms without a fresh operator action).
        const bool timed_out =
            std::chrono::steady_clock::now() - optical_update_enter_pending_since_ >=
            optical_r0_handshake_timeout_;
        if (!opticalR0HandshakeAlive())
        {
            // Ready only ticks on new VR frames. A still headset is normal, so
            // the deadline is the 3s contract timeout, not the 750ms window.
            // Do not unlearn: !alive is indistinguishable from a still headset.
            if (timed_out)
            {
                cancelOpticalPending("no ready frame before UPDATE enter");
                // Ready stopped mid-request, so the stored level may still hold
                // a stale contract bit; clear it to avoid a false snap later.
                optical_r0_ready_bits_.store(0);
            }
            return;
        }
        refreshOpticalR0Request();
        const bool freeze_ready =
            (optical_r0_ready_bits_.fetch_and(~int32_t{3}) & 3) == 3;
        if (!freeze_ready)
        {
            if (timed_out)
            {
                // Peer is alive but did not publish R=I: unlearn so the next
                // press uses topic R0 until caps=1 returns. Entering UPDATE
                // here would duplicate the event-4 handler's base reset.
                cancelOpticalPending("contract frame R=I not published");
                unlearnOpticalR0Peer("alive but no contract frame on UPDATE enter");
            }
            return;
        }
        vr_base_left_position_ = left_position_;
        vr_base_left_orientation_ = Eigen::Quaterniond::Identity();
        vr_base_right_position_ = right_position_;
        vr_base_right_orientation_ = Eigen::Quaterniond::Identity();

        const bool leftBaseReady =
            setRobotBase("left", /*preferLastCommand=*/true);
        const bool rightBaseReady =
            setRobotBase("right", /*preferLastCommand=*/true);
        left_thumbstick_offset_ = Eigen::Vector3d::Zero();
        right_thumbstick_offset_ = Eigen::Vector3d::Zero();
        left_thumbstick_yaw_offset_ = 0.0;
        right_thumbstick_yaw_offset_ = 0.0;

        if (!full_body_yb_pause_mode_.load())
        {
            left_arm_paused_.store(false);
            right_arm_paused_.store(false);
            paused_left_position_ = Eigen::Vector3d::Zero();
            paused_left_orientation_ = Eigen::Quaterniond::Identity();
            paused_right_position_ = Eigen::Vector3d::Zero();
            paused_right_orientation_ = Eigen::Quaterniond::Identity();
        }
        else
        {
            if (left_arm_paused_.load())
            {
                applyPauseAfterRebase(WbcToggleTarget::LEFT_ARM);
            }
            if (right_arm_paused_.load())
            {
                applyPauseAfterRebase(WbcToggleTarget::RIGHT_ARM);
            }
        }

        optical_update_enter_pending_.store(false);
        is_update_mode_.store(true);
        // Drop the stale level: ready is a per-request contract. Without this,
        // a leftover bit0|bit1 could satisfy the next request before fa-py has
        // actually republished R=I for it.
        optical_r0_ready_bits_.store(0);
        refreshOpticalR0Request();  // ACK → request=0 → fa-py opens
        resetStaleCatchUpRamp();
        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️ UPDATE ON via optical R0 handshake (R0=I world-Δ); "
            "bases L=%s R=%s frame=%s",
            leftBaseReady ? "ready" : "pending_tf",
            rightBaseReady ? "ready" : "pending_tf",
            vr_follow_frame_.c_str());
    }

    void VRInputHandler::tryCompleteOpticalResumes()
    {
        if (!optical_resume_pending_left_.load() &&
            !optical_resume_pending_right_.load())
        {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        auto expired = [&now](bool pending,
                              const std::chrono::steady_clock::time_point& since)
        {
            return pending && now - since >= optical_r0_handshake_timeout_;
        };
        const bool alive = opticalR0HandshakeAlive();
        const bool left_due = expired(
            optical_resume_pending_left_.load(), optical_resume_pending_left_since_);
        const bool right_due = expired(
            optical_resume_pending_right_.load(), optical_resume_pending_right_since_);
        if (!alive && !left_due && !right_due)
        {
            // Ready only ticks on new VR frames; a still headset looks the same
            // as a dead one, so wait out the contract window before giving up.
            return;
        }
        if (alive)
        {
            refreshOpticalR0Request();
        }
        else if (left_due || right_due)
        {
            // Give up on R=I for the expired sides. Never leave an arm paused
            // forever because the publisher stopped answering. Do not unlearn:
            // !alive looks the same as a still headset.
            RCLCPP_WARN(
                node_->get_logger(),
                "🕹️ optical R0 handshake resume timed out — falling back to "
                "topic R0 (wrist orientation may be offset)");
        }

        auto finishOne = [this, alive](
                             WbcToggleTarget arm,
                             bool useLeftVr,
                             std::atomic<bool>& pending,
                             bool due) -> bool
        {
            if (!pending.load())
            {
                return false;
            }
            const bool isLeftArm = arm == WbcToggleTarget::LEFT_ARM;
            // Identity() base requires the publisher's contract frame. If the
            // handshake is unusable (or this side expired) fall back to the
            // topic quat, exactly like origin/main, and say so once.
            const bool snapped = alive && tryCompleteOpticalSideSnap(useLeftVr);
            if (!snapped && !due)
            {
                return false;  // still inside the contract window
            }
            if (!snapped)
            {
                // Drop any late contract bit so the next request cannot snap
                // to a stale R=I that arrived after we fell back to topic.
                const int32_t need = useLeftVr ? 1 : 2;
                optical_r0_ready_bits_.fetch_and(~need);
            }
            if (isLeftArm)
            {
                if (snapped)
                {
                    vr_base_left_orientation_ = Eigen::Quaterniond::Identity();
                }
                else
                {
                    vr_base_left_position_ = left_position_;
                    vr_base_left_orientation_ = left_orientation_;
                }
            }
            else
            {
                if (snapped)
                {
                    vr_base_right_orientation_ = Eigen::Quaterniond::Identity();
                }
                else
                {
                    vr_base_right_position_ = right_position_;
                    vr_base_right_orientation_ = right_orientation_;
                }
            }
            const std::string armType = isLeftArm ? "left" : "right";
            setRobotBase(armType, /*preferLastCommand=*/true);
            if (isLeftArm)
            {
                left_thumbstick_offset_.setZero();
                left_thumbstick_yaw_offset_ = 0.0;
                left_arm_paused_.store(false);
            }
            else
            {
                right_thumbstick_offset_.setZero();
                right_thumbstick_yaw_offset_ = 0.0;
                right_arm_paused_.store(false);
            }
            pending.store(false);
            // Clear paused VR pose for the VR side that was controlling this arm.
            if (useLeftVr)
            {
                paused_left_position_.setZero();
                paused_left_orientation_.setIdentity();
            }
            else
            {
                paused_right_position_.setZero();
                paused_right_orientation_.setIdentity();
            }
            refreshOpticalR0Request();
            RCLCPP_INFO(
                node_->get_logger(),
                "🕹️ optical R0 resume complete (%s arm, %s)",
                armType.c_str(),
                snapped ? "R0=I" : "R0=topic");
            // true ⇒ timed out to topic while peer was alive (broken peer).
            return !snapped && alive;
        };

        const bool left_unlearn = finishOne(
            WbcToggleTarget::LEFT_ARM,
            optical_resume_use_left_vr_left_.load(),
            optical_resume_pending_left_,
            left_due);
        const bool right_unlearn = finishOne(
            WbcToggleTarget::RIGHT_ARM,
            optical_resume_use_left_vr_right_.load(),
            optical_resume_pending_right_,
            right_due);
        if (left_unlearn || right_unlearn)
        {
            unlearnOpticalR0Peer("alive but no contract frame on resume");
        }
    }

    /**
     * @brief 恢复一条手臂的 VR 控制
     *
     * 优先走光学 R0 握手（重新对齐起始位姿），握手不可用时退回到主话题直接接管。
     *
     * @param arm 哪条手臂
     * @param buttonTag 触发来源，仅用于日志
     * @return true 表示已恢复
     */
    bool VRInputHandler::resumeArmVrControl(
        WbcToggleTarget arm,
        const char* buttonTag)
    {
        const bool isLeftArm = arm == WbcToggleTarget::LEFT_ARM;
        const bool isRightArm = arm == WbcToggleTarget::RIGHT_ARM;
        if (!isLeftArm && !isRightArm)
        {
            return false;
        }
        const bool useLeftVr = mirror_mode_.load() ? isRightArm : isLeftArm;
        const char* armName = isLeftArm ? "left" : "right";

        if (opticalR0HandshakeSupported())
        {
            const auto now = std::chrono::steady_clock::now();
            if (isLeftArm)
            {
                optical_resume_use_left_vr_left_.store(useLeftVr);
                optical_resume_pending_left_since_ = now;
                optical_resume_pending_left_.store(true);
            }
            else
            {
                optical_resume_use_left_vr_right_.store(useLeftVr);
                optical_resume_pending_right_since_ = now;
                optical_resume_pending_right_.store(true);
            }
            refreshOpticalR0Request();
            tryCompleteOpticalResumes();
            // Caller must not fall through to the topic-R0 path below: that
            // would overwrite the Identity() base we either just snapped or
            // are waiting to snap. Completion is driven by the supervisor.
            const bool still_pending = optical_resume_pending_left_.load() ||
                optical_resume_pending_right_.load();
            RCLCPP_INFO(
                node_->get_logger(),
                "🔘 [%s] optical R0 handshake resume %s (%s arm%s)",
                buttonTag,
                still_pending ? "pending" : "complete",
                armName,
                mirror_mode_.load() ? ", mirror" : "");
            return true;
        }

        // main: R0 = topic (same as origin/main, no .normalized())
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
        const bool baseReady =
            setRobotBase(armName, /*preferLastCommand=*/true);
        if (isLeftArm)
        {
            left_thumbstick_offset_.setZero();
            left_thumbstick_yaw_offset_ = 0.0;
            left_arm_paused_.store(false);
        }
        else
        {
            right_thumbstick_offset_.setZero();
            right_thumbstick_yaw_offset_ = 0.0;
            right_arm_paused_.store(false);
        }
        if (useLeftVr)
        {
            paused_left_position_.setZero();
            paused_left_orientation_.setIdentity();
        }
        else
        {
            paused_right_position_.setZero();
            paused_right_orientation_.setIdentity();
        }

        const bool mirrored = mirror_mode_.load();
        RCLCPP_INFO(
            node_->get_logger(),
            "🔘 [%s] 按下 - 功能: 切换%s臂更新状态 - 操作: 恢复%s臂更新"
            "（重置基准位姿和摇杆偏移）%s",
            buttonTag,
            armName,
            armName,
            mirrored ? " [镜像模式]" : "");
        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️ %s VR base %s after resume",
            armName,
            baseReady ? "ready" : "waiting for TF");
        if (isLeftArm)
        {
            RCLCPP_DEBUG(
                node_->get_logger(),
                "   VR Base Position: [%.3f, %.3f, %.3f]",
                vr_base_left_position_.x(),
                vr_base_left_position_.y(),
                vr_base_left_position_.z());
            RCLCPP_DEBUG(
                node_->get_logger(),
                "   Robot Base Position: [%.3f, %.3f, %.3f]",
                robot_base_left_position_.x(),
                robot_base_left_position_.y(),
                robot_base_left_position_.z());
            RCLCPP_DEBUG(
                node_->get_logger(),
                "   Left Robot Base Source: %s",
                has_last_published_left_target_ ? "last_command"
                                                : "current_pose");
        }
        else
        {
            RCLCPP_DEBUG(
                node_->get_logger(),
                "   VR Base Position: [%.3f, %.3f, %.3f]",
                vr_base_right_position_.x(),
                vr_base_right_position_.y(),
                vr_base_right_position_.z());
            RCLCPP_DEBUG(
                node_->get_logger(),
                "   Robot Base Position: [%.3f, %.3f, %.3f]",
                robot_base_right_position_.x(),
                robot_base_right_position_.y(),
                robot_base_right_position_.z());
            RCLCPP_DEBUG(
                node_->get_logger(),
                "   Right Robot Base Source: %s",
                has_last_published_right_target_ ? "last_command"
                                                 : "current_pose");
        }
        return true;
    }


    // =====================================================================
    // 7. WBC 门控与身体模式
    // =====================================================================

    /**
     * @brief 请求身体约束模式（事件 21–24）
     *
     * 发出命令后进入 pending：只有 WBC 确认且左摇杆回中才认为生效，
     * 否则会在 2 秒后超时并放弃，避免“按了没反应又卡在 pending”
     *
     * @param eventCase 原始事件编号，仅用于日志
     * @param direction 方向描述，仅用于日志
     * @param modeName 模式中文名，仅用于日志
     * @param command 发送给控制器的模式命令字符串
     * @param expectedBodyState 期望的 WbcCurrentState::body_state 取值
     */
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

        if (!isOcs2State())
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

    /**
     * @brief 请求 WBC 开关翻转（单臂使能/底盘锁定/双臂耦合/参考关节）
     *
     * 先读当前状态取反，再发命令；同样走 pending + 摇杆回中确认。
     */
    void VRInputHandler::requestWbcToggle(
        int32_t eventCase,
        const char* sourceDescription,
        WbcToggleTarget target)
    {
        if (!enabled_.load() ||
            !isFullBodyMode() ||
            !isOcs2State())
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
            isOcs2State() &&
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

    /**
     * @brief 收到 WBC 模式状态
     *
     * 跟踪单臂可用性并驱动 rebase / 暂停/恢复相位，不做模式推断。
     */
    void VRInputHandler::wbcStateCallback(
        arms_ros2_control_msgs::msg::WbcCurrentState::ConstSharedPtr msg)
    {
        if (!msg)
        {
            return;
        }

        if (msg->head_state != arms_ros2_control_msgs::msg::WbcCurrentState::HEAD_TRACKING)
        {
            if (!isWholeBodyActive())
            {
                stopHeadTracking();
            }
                onWholeBodyHeadStateChanged(msg->head_state);
        }
        home_joint_reference_enabled_.store(msg->home_joint_reference_enabled);

        updateArmWbcVrState(
            WbcToggleTarget::LEFT_ARM, msg->left_arm_state);
        updateArmWbcVrState(
            WbcToggleTarget::RIGHT_ARM, msg->right_arm_state);
    }

    /**
     * @brief 收到 WBC 能力声明
     *
     * 能力是静态配置，用于提前判断这台机器支不支持全身/底盘等功能。
     */
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
            const bool useLeftVr = mirror_mode_.load()
                ? (arm == WbcToggleTarget::RIGHT_ARM)
                : (arm == WbcToggleTarget::LEFT_ARM);
            if (arm == WbcToggleTarget::LEFT_ARM)
            {
                left_wbc_rebase_use_left_vr_.store(useLeftVr);
                left_wbc_rebase_pending_since_ = std::chrono::steady_clock::now();
            }
            else
            {
                right_wbc_rebase_use_left_vr_.store(useLeftVr);
                right_wbc_rebase_pending_since_ = std::chrono::steady_clock::now();
            }
            rebasePending->store(true);
            refreshOpticalR0Request();
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

    bool VRInputHandler::rebaseArmVrControlFromCurrentPose(
        WbcToggleTarget arm)
    {
        const bool isLeftArm = arm == WbcToggleTarget::LEFT_ARM;
        const bool isRightArm = arm == WbcToggleTarget::RIGHT_ARM;
        if (!isLeftArm && !isRightArm)
        {
            return false;
        }

        const bool useLeftVr = isLeftArm
            ? left_wbc_rebase_use_left_vr_.load()
            : right_wbc_rebase_use_left_vr_.load();
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

        if (opticalR0HandshakeSupported())
        {
            refreshOpticalR0Request();
            if (!tryCompleteOpticalSideSnap(useLeftVr))
            {
                const auto since = isLeftArm
                    ? left_wbc_rebase_pending_since_
                    : right_wbc_rebase_pending_since_;
                const bool timed_out =
                    std::chrono::steady_clock::now() - since >=
                    optical_r0_handshake_timeout_;
                if (!timed_out)
                {
                    return false;  // contract frame pending: retry next frame
                }
                // Escape hatch: never keep an arm suppressed forever because
                // the publisher stopped answering. Fall back to topic R0 and
                // say loudly that the wrist orientation may be offset.
                RCLCPP_WARN(
                    node_->get_logger(),
                    "🕹️ %s WBC rebase: optical R0 handshake timed out — "
                    "falling back to topic R0 (wrist orientation may be offset)",
                    isLeftArm ? "left" : "right");
                const int32_t need = useLeftVr ? 1 : 2;
                optical_r0_ready_bits_.fetch_and(~need);
                if (opticalR0HandshakeAlive())
                {
                    unlearnOpticalR0Peer("alive but no contract frame on WBC rebase");
                }
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
            }
        }
        else if (useLeftVr)
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
        refreshOpticalR0Request();  // ACK → clear side bit → fa-py opens
        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️ %s WBC arm VR control rebased from current pose",
            arm == WbcToggleTarget::LEFT_ARM ? "left" : "right");
        return true;
    }

    /**
     * @brief 接入 WBC 模式统一入口
     *
     * 接入后 mode_command 与能力声明都走 client；未接入时相关命令会被忽略并打 WARN。
     */
    void VRInputHandler::attachHumanoidModeClient(HumanoidModeClient* client)
    {
        mode_client_ = client;
        if (!mode_client_)
        {
            return;
        }

        // 能力声明（静态）：先同步一次，再订变化。
        if (mode_client_->hasCapability())
        {
            wbcCapabilityCallback(
                std::make_shared<arms_ros2_control_msgs::msg::WbcCapability>(
                    mode_client_->capability()));
        }

        capability_observer_token_ = mode_client_->observeCapability(
            [this](const arms_ros2_control_msgs::msg::WbcCapability& msg)
            {
                wbcCapabilityCallback(
                    std::make_shared<arms_ros2_control_msgs::msg::WbcCapability>(msg));
            });

        RCLCPP_INFO(node_->get_logger(),
                    "🕹️ VRInputHandler 已接入 HumanoidModeClient（%s）",
                    mode_client_->hasCapability() ? "已收到能力声明" : "等待能力声明");
    }

    void VRInputHandler::publishHumanoidModeCommand(const std::string& command)
    {
        if (!mode_client_)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "🕹️ WBC 模式命令 %s 被忽略：未接入 HumanoidModeClient",
                        command.c_str());
            return;
        }
        mode_client_->publishCommand(command);
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
            !isOcs2State())
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
            isOcs2State() &&
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


    // =====================================================================
    // 8. 底盘与腰部遥控
    // =====================================================================

    /**
     * @brief 底盘/腰部摇杆遥控
     *
     * 普通模式与全身模式共用的事件 20。非全身拓扑下按住右握把可作为腰部修饰键；
     * 全身拓扑下右轴方向由 WBC 四开关组合消费，进入这里前已被清零。
     */
    void VRInputHandler::processChassisAxes()
    {
        // 普通模式与全身模式共用的事件 20 手动底盘遥控（全身模式的底盘跟随由 OCS2 负责）。
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
        // 用 FsmClient 订阅到的权威状态（fsm_state），而不是本地缓存，
        // 避免缓存落后时退出全身模式该发的零速被吞掉。
        return isFullBodyMode() &&
               isOcs2State() &&
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

    /**
     * @brief 切换底盘遥控模式（事件 20）
     *
     * 进入前清零末端摇杆累积偏移，退出时清零底盘/腰部命令，
     * 避免切换瞬间出现跳变或残留速度。
     */
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


    // =====================================================================
    // 9. 全身模式：模式开关与骨架输入
    // =====================================================================

    /**
     * @brief 全身遥操作的初始化（只做一次）
     *
     * 包含接管渐入参数、骨架订阅与目标通道绑定。
     */
    void VRInputHandler::initializeWholeBodyTeleop()
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

        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️🦴 全身模式已就绪：订阅 /teleop/body_* "
            "(pelvis, spine3, head, %s hands, shoulders) and /teleop/human_footprint_pose",
            body_ee_joint.c_str());
        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️🦴 全身模式仅支持：FSM(11/12)、夹爪(9/10/13/14)、右摇杆短按(4)=全部跟踪、"
            "组合键(20)=底盘遥控、组合键(26)=底盘解锁/锁定、组合键(30)=退出；"
            "其余普通模式组合键在全身模式下不可用");
    }

    /**
     * @brief 开关全身遥操作（事件 30）
     */
    void VRInputHandler::toggleWholeBodyTeleop()
    {
        if (whole_body_teleop_active_.load())
        {
            stopWholeBodyTeleop();
            resumeNormalModeState();
            RCLCPP_INFO(
                node_->get_logger(),
                "🕹️🦴 [case 30] 全身遥操作已关闭：恢复普通 VR 遥操作"
                "（普通模式从 STORAGE 开始，镜像保持关闭，需重新按左摇杆按键开启）");
            return;
        }

        if (!vrEnabled() ||
            !fullBodyTopology() ||
            !isOcs2State())
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "🔘 [case 30] 忽略全身遥操作开关：要求 VR enabled、FULL_BODY、OCS2");
            return;
        }

        // 没有全身骨架数据时整个模式没有任何数据源，直接拒绝进入，
        // 避免进入后既不报错也不下发目标（静默失效）。
        const std::vector<std::string> missing = missingSkeletonSources();
        if (!missing.empty())
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "🔘 [case 30] 拒绝进入全身遥操作：缺少 VR 全身骨架数据（%s）。"
                "请在头显中开启全身追踪，确认 /teleop/body_* 与 "
                "/teleop/human_footprint_pose 已稳定发布后重试",
                joinSources(missing).c_str());
            return;
        }

        startWholeBodyTeleop();
    }

    /**
     * @brief 进入全身遥操作
     *
     * 先把普通模式的运行态全部暂停（UPDATE/镜像/底盘/各类 pending），
     * 再请求身体跟随与头部跟踪；绝对位姿映射还需另按右摇杆开启。
     */
    void VRInputHandler::startWholeBodyTeleop()
    {
        // 进入模式前先清掉普通模式的所有运行态：
        // 停止 UPDATE/STORAGE、强制关闭镜像、退出 CHASSIS 并零速、清 pending/摇杆偏移/
        // 握把边沿/臂暂停，避免这些状态在全身模式下继续生效或退出时残留。
        suspendNormalModeState();

        whole_body_teleop_active_.store(true);
        // 进入模式只开通“允许跟踪”；绝对位姿映射需再按右摇杆开启
        absolute_tracking_active_.store(false);
        session_.setWholeBodyActive(true);
        session_.setAbsoluteTracking(false);
        resetAbsoluteTracking();
        stopWholeBodyHeadTracking();

        // 目标由骨架绝对映射发布，确保 marker 处于连续发布模式
        if (target_manager_ &&
            target_manager_->getCurrentMode() != MarkerState::CONTINUOUS)
        {
            target_manager_->togglePublishMode();
        }

        // 身体模式改为「跟随」、头部模式改为「跟踪」；目标随后由 /teleop/body_* 计算。
        // 全身模式摇杆不参与请求确认，因此直接发命令，不走普通模式的 pending/回中逻辑。
        publishHumanoidModeCommand("BODY_TRACKING");
        publishHumanoidModeCommand("HEAD_TRACKING");
        // 控制器可能因为不支持 BODY_TRACKING 而静默忽略，这里登记一次期望，
        // 超时会打 WARN，避免"开了全身模式但机器人不动"却没有线索。
        if (mode_client_)
        {
            mode_client_->expect(
                HumanoidModeClient::Expectation{
                    "全身模式进入：BODY_TRACKING",
                    [](const arms_ros2_control_msgs::msg::WbcCurrentState& state)
                    {
                        return state.body_state ==
                               arms_ros2_control_msgs::msg::WbcCurrentState::BODY_TRACKING;
                    }});
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️🦴 [case 30] 全身遥操作已开启：身体=跟随（BODY_TRACKING）、"
            "头部=跟踪（HEAD_TRACKING）；右摇杆短按(4)开/关「全部跟踪」，"
            "底盘锁定状态下的 marker 相对机器人底座；"
            "组合键(26)解锁底盘后 marker 锚定世界系、由 OCS2 驱动底盘跟随；"
            "组合键(20)可手动遥控底盘；再次按左右握把+右摇杆按下退出");
    }

    /**
     * @brief 退出全身遥操作并恢复普通模式的可用状态
     */
    void VRInputHandler::stopWholeBodyTeleop()
    {
        const bool was_active = whole_body_teleop_active_.exchange(false);
        absolute_tracking_active_.store(false);
        session_.setAbsoluteTracking(false);
        session_.setWholeBodyActive(false);
        resetAbsoluteTracking();
        stopWholeBodyHeadTracking();

        if (was_active)
        {
            RCLCPP_INFO(
                node_->get_logger(),
                "🕹️🦴 [case 30] 全身遥操作已关闭（「全部跟踪」已关闭，"
                "WBC 保持当前目标）");
        }
    }

    void VRInputHandler::deactivateWholeBodyTeleop()
    {
        stopWholeBodyTeleop();
    }

    /**
     * @brief 权威 FSM 变化时的兜底：离开 OCS2 就退出全身模式
     */
    void VRInputHandler::onWholeBodyFsmStateChanged()
    {
        if (!whole_body_teleop_active_.load())
        {
            return;
        }
        const int32_t fsm_state = fsmState();
        if (isOcs2State())
        {
            return;
        }

        stopWholeBodyTeleop();
        resumeNormalModeState();
        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️🦴 FSM 已离开 OCS2（状态=%d）：全身遥操作自动退出"
            "（「全部跟踪」已关闭，WBC 保持当前目标）；"
            "回到 OCS2 后需重新按「左右握把+右摇杆按下」开启",
            fsm_state);
    }

    void VRInputHandler::onWholeBodyTopologyChanged(bool is_full_body)
    {
        if (!is_full_body && whole_body_teleop_active_.load())
        {
            stopWholeBodyTeleop();
            resumeNormalModeState();
            RCLCPP_INFO(
                node_->get_logger(),
                "🔘 控制拓扑不再是 FULL_BODY：全身遥操作已自动退出");
        }
    }

    void VRInputHandler::onWholeBodyHeadStateChanged(int head_state)
    {
        if (head_state != arms_ros2_control_msgs::msg::WbcCurrentState::HEAD_TRACKING &&
            whole_body_teleop_active_.load())
        {
            stopWholeBodyHeadTracking();
        }
    }

    void VRInputHandler::onWholeBodyTargetCacheCleared()
    {
        resetAbsoluteRamps();
        resetStaleCatchUpRamp();
    }

    void VRInputHandler::rearmWholeBodyArmRamp(bool is_left)
    {
        if (!absolute_options_.pose_ramp_enable)
        {
            return;
        }
        LastArmTarget last;
        last = lastArmTarget(is_left);
        auto& ramp = is_left ? left_arm_ramp_ : right_arm_ramp_;
        if (last.valid)
        {
            ramp.armFrom(last.position, last.orientation);
        }
        else
        {
            ramp.reset();
        }
    }

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
        {
            PoseSample sample;
            sample.position = position;
            sample.orientation = body_pelvis_orientation_;
            sample.valid = true;
            input_hub_.setJoint(Joint::Pelvis, sample);
        }
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
        {
            PoseSample sample;
            sample.position = position;
            sample.orientation = body_spine3_orientation_;
            sample.valid = true;
            input_hub_.setJoint(Joint::Spine3, sample);
        }
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
        {
            PoseSample sample;
            sample.position = position;
            sample.orientation = body_head_orientation_;
            sample.valid = true;
            input_hub_.setJoint(Joint::Head, sample);
        }
        has_body_head_pose_ = true;
        body_head_stamp_ = std::chrono::steady_clock::now();
        if (whole_body_teleop_active_.load())
        {
            publishWholeBodyHeadTarget();
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
        {
            PoseSample sample;
            sample.position = position;
            sample.orientation = human_footprint_orientation_;
            sample.valid = true;
            input_hub_.setFootprint(sample);
        }
        human_footprint_stamp_ = std::chrono::steady_clock::now();
        has_human_footprint_pose_ = true;
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
            {
                PoseSample sample;
                sample.position = position;
                sample.orientation = body_left_hand_orientation_;
                sample.valid = true;
                input_hub_.setJoint(Joint::HandLeft, sample);
            }
            has_body_left_hand_pose_ = true;
            body_left_hand_stamp_ = std::chrono::steady_clock::now();
        }
        else
        {
            body_right_hand_position_ = position;
            body_right_hand_orientation_ = orientation.normalized();
            {
                PoseSample sample;
                sample.position = position;
                sample.orientation = body_right_hand_orientation_;
                sample.valid = true;
                input_hub_.setJoint(Joint::HandRight, sample);
            }
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
        {
            PoseSample sample;
            sample.position = Eigen::Vector3d(position.x(), position.y(), position.z());
            sample.valid = true;
            input_hub_.setJoint(
                is_left ? Joint::ShoulderLeft : Joint::ShoulderRight, sample);
        }
    }

    void VRInputHandler::toggleAbsoluteTracking()
    {
        if (!vrEnabled() ||
            !fullBodyTopology() ||
            !isOcs2State())
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

    void VRInputHandler::startAbsoluteTracking()
    {
        absolute_tracking_active_.store(true);
        session_.setAbsoluteTracking(true);
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
        resetStaleCatchUpRamp();

        // 目标汇聚层仍缓存着上一轮（普通模式 / 上次跟踪）实际发出的位姿，且会在
        // 「控制权换手」时从它按 handover_ramp 平滑过渡。那个位姿已经不代表机器人
        // 当前状态（带底盘时可能已被 OCS2 / 导航带走），直接沿用会让第一帧目标
        // 从别处出发。这里清掉缓存，起点统一交给下面的渐入锚点（实测位姿）。
        if (target_manager_ && target_manager_->router())
        {
            target_manager_->router()->invalidate();
        }

        // 接管渐入：以机器人当前实测位姿为锚点，按速度约束过渡到实时骨架期望位姿。
        armAbsoluteRamps();

        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️🦴 [case 4] 全身「全部跟踪」已开启：胸/头/双臂末端改为 /teleop/body_* 绝对位姿映射"
            "（等待骨架稳定后标定）；再次短按右摇杆关闭");
    }

    void VRInputHandler::stopAbsoluteTracking()
    {
        absolute_tracking_active_.store(false);
        session_.setAbsoluteTracking(false);
        resetAbsoluteTracking();
        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️🦴 [case 4] 全身「全部跟踪」已关闭：停止下发胸/头/双臂目标"
            "（WBC 保持当前目标）");
    }

    void VRInputHandler::resetAbsoluteTracking()
    {
        absolute_maps_captured_ = false;
        settle_pending_ = false;
        anchor_valid_ = false;
        abs_left_filter_primed_ = false;
        abs_right_filter_primed_ = false;
        abs_body_filter_primed_ = false;
        abs_head_filter_primed_ = false;
        abs_left_arm_ref_len_ = 0.0;
        abs_right_arm_ref_len_ = 0.0;
        resetStaleCatchUpRamp();
        resetAbsoluteRamps();
        // 交还臂目标独占标志（唯一消费者是手柄/键盘 control_input 的 twist 改写）
        target_manager_->setArmVrActive(true, false);
        target_manager_->setArmVrActive(false, false);
    }

    bool VRInputHandler::wholeBodyTeleopReady() const
    {
        // 与双臂末端跟随同步：只有「全部跟踪」开着才下发躯干目标
        return target_manager_ &&
               fullBodyTopology() &&
               vrEnabled() &&
               absolute_tracking_active_.load() &&
               isOcs2State() &&
               target_manager_->getCurrentBodyState() ==
                   arms_ros2_control_msgs::msg::WbcCurrentState::BODY_TRACKING &&
               has_body_pelvis_pose_ && has_body_spine3_pose_;
    }

    bool VRInputHandler::wholeBodyHeadTrackingReady() const
    {
        return target_manager_ &&
               fullBodyTopology() &&
               vrEnabled() &&
               absolute_tracking_active_.load() &&
               isOcs2State() &&
               target_manager_->getCurrentHeadState() ==
                   arms_ros2_control_msgs::msg::WbcCurrentState::HEAD_TRACKING &&
               has_body_head_pose_ &&
               skeletonJointFresh(body_head_stamp_);
    }

    void VRInputHandler::stopWholeBodyHeadTracking()
    {
        wb_head_tracking_active_ = false;
        if (target_manager_)
        {
            target_manager_->setHeadVrActive(false);
        }
    }

    void VRInputHandler::publishWholeBodyHeadTarget()
    {
        if (!wholeBodyHeadTrackingReady())
        {
            stopWholeBodyHeadTracking();
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

        const Eigen::Vector3d position_H = head_in_H;
        const Eigen::Quaterniond orientation_H = dropRoll(
            Eigen::Quaterniond(head_rot_in_H * abs_head_src_to_ee_).normalized());

        // H 系 → 目标表达系（下游 WBC 的 base_frame_）/ marker 固定系（通常是 world）
        Eigen::Isometry3d T_target_H;
        Eigen::Isometry3d T_marker_H;
        if (!resolveAnchorTransforms(T_H_world, T_target_H, T_marker_H))
        {
            return;
        }
        const Eigen::Vector3d position = T_target_H * position_H;
        const Eigen::Quaterniond orientation(
            T_target_H.linear() * orientation_H.toRotationMatrix());

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
        target.header.frame_id = controlFrame();
        target.header.stamp = node_->now();
        target.pose.position.x = ramped_position.x();
        target.pose.position.y = ramped_position.y();
        target.pose.position.z = ramped_position.z();
        target.pose.orientation.x = ramped_orientation.x();
        target.pose.orientation.y = ramped_orientation.y();
        target.pose.orientation.z = ramped_orientation.z();
        target.pose.orientation.w = ramped_orientation.w();

        if (!submitWholeBodyHeadTarget(target.pose, target.header.frame_id))
        {
            // RViz 正在手动拖头部，或目标被拒绝：先停止本帧头部跟踪，下一帧重新接管。
            stopWholeBodyHeadTracking();
            return;
        }
        target_manager_->setHeadVrActive(true);
        wb_last_head_target_ = target;
        wb_last_head_target_stamp_ = std::chrono::steady_clock::now();
        wb_head_tracking_active_ = true;
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

        const Eigen::Vector3d position_H = anchor_in_H;
        // 再右乘源→目标固定偏置，最后按可达自由度（ZYX 分解）丢弃 roll
        const Eigen::Quaterniond orientation_H = dropRoll(
            Eigen::Quaterniond(spine_rot_in_H * abs_body_src_to_ee_).normalized());

        // 同头部：目标表达在下游 WBC 的 base_frame_（移动底盘=world / 固定底盘=baseFootprint）
        Eigen::Isometry3d T_target_H;
        Eigen::Isometry3d T_marker_H;
        if (!resolveAnchorTransforms(T_H_world, T_target_H, T_marker_H))
        {
            return;
        }
        const Eigen::Vector3d position = T_target_H * position_H;
        const Eigen::Quaterniond orientation(
            T_target_H.linear() * orientation_H.toRotationMatrix());

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
        target.header.frame_id = controlFrame();
        target.header.stamp = node_->now();
        target.pose.position.x = ramped_position.x();
        target.pose.position.y = ramped_position.y();
        target.pose.position.z = ramped_position.z();
        target.pose.orientation.x = ramped_orientation.x();
        target.pose.orientation.y = ramped_orientation.y();
        target.pose.orientation.z = ramped_orientation.z();
        target.pose.orientation.w = ramped_orientation.w();

        // RViz 交互 marker 显示在 marker_fixed_frame（通常是 world）下；
        // /body_target 由 TargetRouter 统一换算到 control_base_frame
        // （下游 OCS2 base_frame_），两者不能混用。
        // marker 显示「实际下发的渐入值」：先回到人体 footprint 系再换到 marker 固定系。
        const Eigen::Isometry3d T_H_target = T_target_H.inverse();
        const Eigen::Vector3d marker_position =
            T_marker_H * (T_H_target * ramped_position);
        const Eigen::Quaterniond marker_orientation(
            T_marker_H.linear() * T_H_target.linear() *
            ramped_orientation.toRotationMatrix());
        geometry_msgs::msg::Pose marker_pose;
        marker_pose.position.x = marker_position.x();
        marker_pose.position.y = marker_position.y();
        marker_pose.position.z = marker_position.z();
        marker_pose.orientation.x = marker_orientation.x();
        marker_pose.orientation.y = marker_orientation.y();
        marker_pose.orientation.z = marker_orientation.z();
        marker_pose.orientation.w = marker_orientation.w();
        if (!target_manager_->setBodyMarkerPose(marker_pose))
        {
            // 被 RViz 手动拖动占用（或当前不发目标）：本帧不更新锚点缓存。
            return;
        }

        // 缓存本节点实际下发的胸部目标，供下次接管渐入取锚点
        has_last_body_target_ = true;
        last_body_target_position_ = ramped_position;
        last_body_target_orientation_ = ramped_orientation;
        last_body_target_stamp_ = std::chrono::steady_clock::now();
        // 注：RViz marker 拖拽/菜单不门控（调试通道，允许随时手工干预）
    }

    void VRInputHandler::publishAbsoluteArmTarget(bool is_left)
    {
        if (!absolute_tracking_active_.load() || !whole_body_teleop_active_.load())
        {
            return;
        }
        if (!vrEnabled() ||
            !fullBodyTopology() ||
            !isOcs2State())
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
        if (isArmVrInputSuppressed(
                is_left ? VRInputHandler::WbcToggleTarget::LEFT_ARM
                        : VRInputHandler::WbcToggleTarget::RIGHT_ARM))
        {
            if (!prepareWholeBodyArmInput(is_left))
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

        // 机器人目标 = 人体手相对人体根的当前位姿（固定映射，无按下时基准）；
        // 再经锚定变换表达为下游 WBC 的 base_frame_（移动底盘=world / 固定底盘=baseFootprint）
        const Eigen::Matrix3d& hand_to_ee =
            is_left ? abs_hand_to_ee_left_ : abs_hand_to_ee_right_;
        const Eigen::Vector3d position_H = hand_in_H;
        const Eigen::Quaterniond orientation_H(
            Eigen::Quaterniond(hand_rot_in_H * hand_to_ee).normalized());

        Eigen::Isometry3d T_target_H;
        Eigen::Isometry3d T_marker_H;
        if (!resolveAnchorTransforms(T_H_world, T_target_H, T_marker_H))
        {
            return;
        }
        const Eigen::Vector3d position = T_target_H * position_H;
        const Eigen::Quaterniond orientation(
            T_target_H.linear() * orientation_H.toRotationMatrix());

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
        if (publishArmTargetForWholeBody(is_left, ramped_pos, ramped_ori))
        {
            target_manager_->setArmVrActive(is_left, true);
        }
    }


    // =====================================================================
    // 10. 全身模式：映射、坐标与锚定
    // =====================================================================

    bool VRInputHandler::baseUnlocked() const
    {
        return target_manager_ &&
            target_manager_->getCurrentBaseState() ==
            arms_ros2_control_msgs::msg::WbcCurrentState::BASE_UNLOCKED;
    }

    bool VRInputHandler::transformPose(
        const std::string& label,
        const std::string& source_frame,
        const std::string& target_frame,
        const Eigen::Vector3d& input_position,
        const Eigen::Quaterniond& input_orientation,
        Eigen::Vector3d& output_position,
        Eigen::Quaterniond& output_orientation)
    {
        return transformPoseBetweenFrames(
            label, input_position, input_orientation,
            source_frame, target_frame, output_position, output_orientation);
    }

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

    Eigen::Vector3d VRInputHandler::humanToRobotOffsetInH() const
    {
        const std::vector<double>& offset =
            absolute_options_.human_to_robot_offset_xyz;
        if (offset.size() != 3)
        {
            // 长度非法已在节点启动时告警，这里按零偏置处理避免越界
            return Eigen::Vector3d::Zero();
        }
        const Eigen::Vector3d value(offset[0], offset[1], offset[2]);
        return value.allFinite() ? value : Eigen::Vector3d::Zero();
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

        // ④ 人体系 → 机体系的固定平移偏置：放在三段尺度之后整体相加，因此既不参与
        //    段向量缩放、也不改变任何姿态。
        //    人体 footprint 取骨盆地面投影与双足较低者，机器人 footprint 由底盘几何决定，
        //    两者原点相对身体的位置本来就不同。例如机器人 footprint 落在身体后下方
        //    （底盘后悬）、人体 footprint 在头部正下方，则机器人每个同名关键点都比人体
        //    整体偏前，把该前移量填进参数的 x 即可让「手放在胸前」落到机器人胸前。
        const Eigen::Vector3d offset = humanToRobotOffsetInH();
        if (!offset.isZero())
        {
            out.pelvis += offset;
            out.spine3 += offset;
            out.head += offset;
            out.shoulder_left += offset;
            out.shoulder_right += offset;
            out.hand_left += offset;
            out.hand_right += offset;
        }
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

    std::vector<std::string> VRInputHandler::missingSkeletonSources() const
    {
        std::vector<std::string> missing;
        // 骨架输入只有一份真相：InputHub。准入与标定前检查都走这里。
        const Caps required = capOf(InputCap::SkeletonCore) |
            capOf(InputCap::SkeletonArms) |
            capOf(InputCap::Footprint);
        (void)input_hub_.requireFresh(required, &missing);

        // 肩部是「肩宽 / 臂长」两条尺度链的起点，单独确认一次。
        const auto check_shoulder = [this, &missing](Joint joint, const char* name)
        {
            const auto& sample = input_hub_.joint(joint);
            if (sample.stamp.time_since_epoch().count() == 0)
            {
                missing.emplace_back(name);
            }
            else if (!skeletonJointFresh(sample.stamp))
            {
                missing.emplace_back(std::string(name) + "（已超时）");
            }
        };
        check_shoulder(Joint::ShoulderLeft, "左肩骨架数据");
        check_shoulder(Joint::ShoulderRight, "右肩骨架数据");
        return missing;
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
        if (!eeFrameReady())
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "🕹️🦴 全身绝对跟踪等待末端参考坐标系（left_current_pose）初始化");
            return false;
        }

        // 目标不再以机器人当前姿态为基准，因此标定只需要人体骨架数据
        const std::vector<std::string> missing = missingSkeletonSources();
        if (!missing.empty())
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "🕹️🦴 全身绝对跟踪等待骨架数据：%s", joinSources(missing).c_str());
            return false;
        }

        // 人体参考臂长（手–脊柱距离），供几何一致性过滤使用
        abs_left_arm_ref_len_ =
            (body_left_hand_position_ - body_spine3_position_).norm();
        abs_right_arm_ref_len_ =
            (body_right_hand_position_ - body_spine3_position_).norm();

        settle_pending_ = false;
        absolute_maps_captured_ = true;

        // 标定结束、真正开始下发的那一刻再取一次锚点：等待骨架稳定的窗口内机器人
        // 可能已经移动（带底盘时尤其明显），用按下瞬间的位姿起手会留下一段旧误差。
        armAbsoluteRamps();

        const Eigen::Vector3d human_to_robot_offset = humanToRobotOffsetInH();
        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️🦴 全身绝对跟踪已生效：人体基坐标系 H 来自 /teleop/human_footprint_pose"
            "（xy 取骨盆地面投影 / 朝向取两踝 x 轴地平面投影均值 / z 取两足较低者，"
            "俯仰滚转恒为 0；低通与原点偏置在 vr_pose_publisher 侧配置），"
            "目标系=%s，marker 固定系=%s，尺度[身长/肩宽/臂长]=%.2f/%.2f/%.2f，"
            "人体系→机体系平移偏置=[%.3f, %.3f, %.3f] m，"
            "末端源=body_%s，胸部锚点 λ=%.2f；人体参考臂长 L/R=%.3f/%.3f m。"
            "映射按「人体系」锚定：人体各量先表达在人体根自身坐标系里，故人体朝向 ≡ 机器人机头朝向"
            "（无需人工偏航常数）；"
            "底盘锁定时目标围绕机器人底座（人体走动不会拉离 marker），"
            "事件 26 解锁底盘后 marker 锚定世界系并随人体走动远离机器人，由 OCS2 驱动底盘跟随",
            controlFrame().c_str(),
            markerFrame().c_str(),
            absolute_options_.body_length_scale,
            absolute_options_.shoulder_width_scale,
            absolute_options_.arm_length_scale,
            human_to_robot_offset.x(),
            human_to_robot_offset.y(),
            human_to_robot_offset.z(),
            absolute_options_.ee_source.c_str(),
            absolute_options_.body_anchor_lambda,
            abs_left_arm_ref_len_, abs_right_arm_ref_len_);
        return true;
    }

    void VRInputHandler::armAbsoluteRamps(bool anchor_at_measured_pose)
    {
        if (!absolute_options_.pose_ramp_enable)
        {
            resetAbsoluteRamps();
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        // 锚点语义：必须是「机器人当前状态」。最后下发的目标只是「本节点曾经的意图」，
        // 带底盘时它可能已被 OCS2 / 导航挪走、被 RViz marker 拖过、或位于世界系的另一个
        // 位置；拿它当渐入起点，按下开始跟踪时就会看到目标点从别处飞向手柄位姿。
        // 因此开始跟踪时一律先用实测位姿（*_current_pose），实测不可用时才退回 1 s 内的
        // 最后下发值；底盘锁定/解锁切换（anchor_at_measured_pose=false）仍以最后下发值
        // 优先，保证切换瞬间目标严格连续、不因跟踪误差回抽。
        constexpr double kAnchorMaxAgeSec = 1.0;
        const auto isFresh = [&now](const std::chrono::steady_clock::time_point& stamp)
        {
            return stamp != std::chrono::steady_clock::time_point{} &&
                   std::chrono::duration<double>(now - stamp).count() <= kAnchorMaxAgeSec;
        };
        // 取最后下发值；不可用/过期时返回 false。
        const auto fresh_last_arm = [&](bool is_left) -> std::optional<LastArmTarget>
        {
            const LastArmTarget last = lastArmTarget(is_left);
            if (last.valid && isFresh(last.stamp))
            {
                return last;
            }
            return std::nullopt;
        };

        const std::string target_frame = controlFrame();
        // 锚点可能来自实测话题（自带 frame_id）或本节点缓存（目标表达系），
        // 布置渐入前统一换算到目标表达系，避免带底盘时两套坐标系混用。
        const auto ramp_from = [&](arms_controller_common::PoseRamp& ramp,
                                   const Eigen::Vector3d& position,
                                   const Eigen::Quaterniond& orientation,
                                   const std::string& frame_id,
                                   const char* task) -> bool
        {
            if (frame_id.empty() || frame_id == target_frame)
            {
                ramp.armFrom(position, orientation);
                return true;
            }
            Eigen::Vector3d transformed_position;
            Eigen::Quaterniond transformed_orientation;
            if (!transformPose(task, frame_id, target_frame, position, orientation,
                               transformed_position, transformed_orientation))
            {
                return false;
            }
            ramp.armFrom(transformed_position, transformed_orientation);
            return true;
        };

        // ---- 双臂：开始跟踪时优先实测末端位姿，其次最后下发的 command target ----
        const auto arm_anchor = [&](bool is_left) -> std::optional<LastArmTarget>
        {
            if (!anchor_at_measured_pose)
            {
                if (const auto last = fresh_last_arm(is_left))
                {
                    return last;
                }
            }
            Eigen::Vector3d position;
            Eigen::Quaterniond orientation;
            if (robotCurrentPose(is_left, position, orientation))
            {
                LastArmTarget measured;
                measured.valid = true;
                measured.position = position;
                measured.orientation = orientation;
                measured.stamp = now;
                return measured;
            }
            return fresh_last_arm(is_left);
        };

        bool left_armed = false;
        if (const auto anchor = arm_anchor(true))
        {
            left_armed = ramp_from(
                left_arm_ramp_, anchor->position, anchor->orientation,
                ee_frame_id_, "left");
        }
        if (!left_armed)
        {
            left_arm_ramp_.reset();
        }

        bool right_armed = false;
        if (const auto anchor = arm_anchor(false))
        {
            right_armed = ramp_from(
                right_arm_ramp_, anchor->position, anchor->orientation,
                ee_frame_id_, "right");
        }
        if (!right_armed)
        {
            right_arm_ramp_.reset();
        }

        // ---- 胸部：开始跟踪时优先实测胸部位姿（body_current_pose），其次最后下发的目标 ----
        const bool has_fresh_body_target =
            has_last_body_target_ && isFresh(last_body_target_stamp_);
        const auto last_body_ramp = [&]() -> bool
        {
            if (!has_fresh_body_target)
            {
                return false;
            }
            // 最后下发值本来就与目标表达系一致
            return ramp_from(
                body_ramp_, last_body_target_position_, last_body_target_orientation_,
                target_frame, "body");
        };
        bool body_armed = !anchor_at_measured_pose && last_body_ramp();
        Eigen::Vector3d body_position;
        Eigen::Quaterniond body_orientation;
        std::string body_frame_id;
        if (!body_armed && robotBodyPose(body_position, body_orientation, body_frame_id))
        {
            body_armed = ramp_from(
                body_ramp_, body_position, body_orientation, body_frame_id, "body");
        }
        if (!body_armed)
        {
            body_armed = last_body_ramp();
        }
        if (!body_armed)
        {
            body_ramp_.reset();
        }

        // ---- 头部：开始跟踪时优先实测头部位姿，其次最后一次下发的目标
        //      （两者都可能来自普通模式的另一个 frame，先转到目标表达系） ----
        const bool has_fresh_head_target =
            wb_last_head_target_ && isFresh(wb_last_head_target_stamp_);
        const auto last_head_ramp = [&]() -> bool
        {
            if (!has_fresh_head_target)
            {
                return false;
            }
            const auto& p = wb_last_head_target_->pose.position;
            const auto& q = wb_last_head_target_->pose.orientation;
            return ramp_from(
                head_ramp_,
                Eigen::Vector3d(p.x, p.y, p.z),
                Eigen::Quaterniond(q.w, q.x, q.y, q.z),
                wb_last_head_target_->header.frame_id, "head");
        };
        bool head_armed = !anchor_at_measured_pose && last_head_ramp();
        geometry_msgs::msg::PoseStamped robot_head;
        if (!head_armed && robotHeadPose(robot_head))
        {
            const auto& p = robot_head.pose.position;
            const auto& q = robot_head.pose.orientation;
            head_armed = ramp_from(
                head_ramp_,
                Eigen::Vector3d(p.x, p.y, p.z),
                Eigen::Quaterniond(q.w, q.x, q.y, q.z),
                robot_head.header.frame_id, "head");
        }
        if (!head_armed)
        {
            head_armed = last_head_ramp();
        }
        if (!head_armed)
        {
            head_ramp_.reset();
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️🦴 [case 4] 接管渐入已布置（锚点=%s）：lin=%.2f m/s, ang=%.2f rad/s, "
            "臂(L/R)=%s/%s, 胸=%s, 头=%s（未布置的任务直接跟踪）",
            anchor_at_measured_pose
                ? "机器人当前实测位姿（实测不可用时退回 1 s 内最后下发目标）"
                : "最后下发目标（底盘锁定/解锁切换，保持连续）",
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

    std::string VRInputHandler::controlFrame() const
    {
        // 目标（/left_target、/right_target、/body_target、/head_target 均为无 header 的 Pose）
        // 由下游 WBC 按自己的 base_frame_ 解释：移动底盘模型（manipulatorModelType 1/4）
        // 为 world，固定底盘模型为模型根（如 base_footprint）。
        if (target_manager_)
        {
            const std::string& frame = target_manager_->getControlBaseFrame();
            if (!frame.empty())
            {
                return frame;
            }
        }
        return ee_frame_id_;
    }

    std::string VRInputHandler::markerFrame() const
    {
        if (target_manager_)
        {
            const std::string& frame = target_manager_->getMarkerFixedFrame();
            if (!frame.empty())
            {
                return frame;
            }
        }
        return controlFrame();
    }

    bool VRInputHandler::basePoseInFrame(
        const std::string& frame, Eigen::Isometry3d& T_frame_base)
    {
        T_frame_base.setIdentity();
        if (frame.empty() || frame == vr_follow_frame_)
        {
            // vr_follow_frame_ 本身即机器人底座（base_footprint）
            return true;
        }
        if (!eeFrameReady())
        {
            return false;
        }

        Eigen::Vector3d position;
        Eigen::Quaterniond orientation;
        if (!transformPose(
                "chassis", vr_follow_frame_, frame,
                Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
                position, orientation))
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 3000,
                "🕹️🦴 无 %s → %s TF，冻结全身目标（底盘锁定场景可改用 base_footprint 作为目标系）",
                vr_follow_frame_.c_str(), frame.c_str());
            return false;
        }
        T_frame_base.linear() = orientation.normalized().toRotationMatrix();
        T_frame_base.translation() = position;
        return true;
    }

    bool VRInputHandler::resolveAnchorTransforms(
        const Eigen::Isometry3d& T_H_world,
        Eigen::Isometry3d& T_target_H,
        Eigen::Isometry3d& T_marker_H)
    {
        const std::string marker_frame = markerFrame();
        const std::string target_frame = controlFrame();
        if (marker_frame.empty() || target_frame.empty())
        {
            RCLCPP_WARN_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 3000,
                "🕹️🦴 等待目标/marker 坐标系（control='%s', marker='%s'）",
                target_frame.c_str(), marker_frame.c_str());
            return false;
        }

        // marker 固定系下机器人底座的位姿
        Eigen::Isometry3d T_M_base;
        if (!basePoseInFrame(marker_frame, T_M_base))
        {
            return false;
        }

        const bool base_unlocked = baseUnlocked();
        const bool fresh_anchor = !anchor_valid_;
        if (!base_unlocked || fresh_anchor)
        {
            // 底盘锁定（或首次锚定）：锚定随机器人底座实时刷新（T = base ⊗ H⁻¹），
            // 于是「目标 = 底座 ⊗ 人体相对 footprint 的位姿」。
            // 人体原地走动只改变其相对自身 footprint 的位姿，期望 marker 不会被拉离机器人。
            anchor_M_ = T_M_base * T_H_world.inverse();
            anchor_valid_ = true;
        }
        // 解锁期间 anchor_M_ 冻结：marker 锚定在世界系，人体走动时远离机器人，
        // 由 OCS2 的底盘自由度（BASE_UNLOCKED）自动把底座挪到可达的位置。

        const bool state_changed = !fresh_anchor && (anchor_base_unlocked_ != base_unlocked);
        anchor_base_unlocked_ = base_unlocked;
        if (state_changed)
        {
            // 锁定 ↔ 解锁切换时目标语义变化，重新布置接管渐入，把可能的位姿差
            // 按速度约束过渡掉（解锁瞬间本身连续，重排不产生额外滞后）。
            applyAnchorTransition();
        }

        T_marker_H = anchor_M_ * T_H_world;

        if (target_frame == marker_frame)
        {
            T_target_H = T_marker_H;
            return true;
        }

        // 目标表达系与 marker 固定系不同：目标 = 目标系下同一世界位姿
        Eigen::Isometry3d T_C_base;
        if (!basePoseInFrame(target_frame, T_C_base))
        {
            return false;
        }
        T_target_H = T_C_base * T_M_base.inverse() * T_marker_H;
        return true;
    }

    void VRInputHandler::applyAnchorTransition()
    {
        if (!absolute_tracking_active_.load())
        {
            return;
        }
        // 底盘锁定状态变化后，目标语义从「相对底盘」切到「世界系锚定」（或反过来），
        // 重新从最后一次实际下发的目标开始按速度约束过渡，避免直接跳变。
        // 这里刻意不用实测位姿：切换瞬间目标本身是连续的，用实测会把跟踪误差回抽成一段倒退。
        armAbsoluteRamps(/*anchor_at_measured_pose=*/false);
    }


    // =====================================================================
    // 11. 模式查询与共享工具
    // =====================================================================

    /**
     * @brief 接入 FSM 统一入口
     *
     * 接入后本类不再缓存 FSM 状态、也不再自己发 fsm_command；
     * 状态读 `fsm_->isOcs2()`，命令发 `fsm_->request(意图)`。
     */
    void VRInputHandler::attachFsmClient(FsmClient* client)
    {
        fsm_ = client;
        if (!fsm_)
        {
            return;
        }

        // 状态变化后的业务反应全部集中在这里，不再散在 fsmCommandCallback 里。
        fsm_observer_token_ = fsm_->observe(
            [this](FsmState from, FsmState to)
            {
                onFsmTransition(from, to);
            });

        RCLCPP_INFO(node_->get_logger(),
                    "🕹️ VRInputHandler 已接入 FsmClient（当前 %s，%s）",
                    fsm_->stateName(),
                    fsm_->hasReceivedState() ? "已收到控制器状态" : "等待控制器状态");
    }

    /**
     * @brief FSM 状态变化后的业务反应
     *
     * 这里只做“反应”，不再做任何状态推断：状态本身已由 FsmClient 给好。
     */
    void VRInputHandler::onFsmTransition(FsmState from, FsmState to)
    {
        // ① HOLD → OCS2：清空上一轮缓存的 command target，下一次 UPDATE 从当前位姿重新锚定
        if (from == FsmState::Hold && to == FsmState::Ocs2)
        {
            clearLastPublishedTargets();
            RCLCPP_INFO(node_->get_logger(),
                        "🕹️🕶️🕹️ HOLD → OCS2: 已清空缓存的 command target；"
                        "下一次 UPDATE 将从当前位姿重新锚定");
        }

        // ② 离开 OCS2：自动回 STORAGE、停头部跟踪、清摇杆累积
        if (to != FsmState::Ocs2)
        {
            stopHeadTracking();
            if (is_update_mode_.exchange(false))
            {
                left_thumbstick_offset_ = Eigen::Vector3d::Zero();
                right_thumbstick_offset_ = Eigen::Vector3d::Zero();
                left_thumbstick_yaw_offset_ = 0.0;
                right_thumbstick_yaw_offset_ = 0.0;
                RCLCPP_INFO(node_->get_logger(),
                            "🕹️🕶️🕹️ FSM 已离开 OCS2（%s）：自动切回 STORAGE 并清空摇杆累积",
                            fsm_->stateName());
            }
        }

        // ③ 全身遥操作只在 OCS2 内有效
        onWholeBodyFsmStateChanged();
    }

    bool VRInputHandler::isOcs2State() const noexcept
    {
        // 未接入 FsmClient 时按 HOLD 处理（与控制器初始状态一致），
        // 因此所有 OCS2 门控默认关闭。
        return fsm_ != nullptr && fsm_->isOcs2();
    }

    int32_t VRInputHandler::fsmState() const noexcept
    {
        return fsm_ ? static_cast<int32_t>(fsm_->state())
                    : static_cast<int32_t>(FsmState::Hold);
    }

    void VRInputHandler::requestFsmIntent(FsmIntent intent)
    {
        if (!fsm_)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "🕹️ FSM 请求被忽略：未接入 FsmClient");
            return;
        }
        fsm_->request(intent);
    }

    void VRInputHandler::sendFsmCommand(int32_t command)
    {
        if (!fsm_)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "🕹️ FSM 命令 %d 被忽略：未接入 FsmClient", command);
            return;
        }
        fsm_->requestCommand(command);
    }

    bool VRInputHandler::isWholeBodyActive() const
    {
        return wholeBodyActive();
    }

    bool VRInputHandler::isWholeBodyTeleopActive() const
    {
        return wholeBodyActive();
    }

    bool VRInputHandler::isAbsoluteTrackingActive() const
    {
        return absoluteTrackingActive();
    }

    void VRInputHandler::toggleWholeBodyMode()
    {
            toggleWholeBodyTeleop();
    }

    void VRInputHandler::suspendNormalModeState()
    {
        // 进入全身模式前，把普通模式的运行态全部清掉，避免全身模式下继续生效或退出时残留。
        is_update_mode_.store(false);
        // 镜像关掉后必须同步发给下游（灵巧手 / DualHand 依赖该 latch），
        // 否则全身模式下手的映射仍保持镜像。
        mirror_mode_.store(false);
        publishMirrorMode();
        mirror_mode_profile_.store(static_cast<int>(MirrorModeProfile::MODE_A));

        // 光学 R0 握手：待完成的 UPDATE 进入 / 单臂恢复在全身模式内不会再有推进时机，
        // 必须显式取消（否则 100ms 监管器会在全身模式内把 is_update_mode_ 拉回 true）。
        cancelOpticalPending("entering whole-body teleop");

        chassis_mode_.store(false);
        // 无论之前是否处于 CHASSIS，都向 /cmd_vel 与腰部话题各发一次 0
        resetChassisAndWaistCommands();

        right_wbc_toggle_request_pending_.store(false);
        requested_wbc_toggle_target_.store(WbcToggleTarget::NONE);
        body_mode_request_pending_.store(false);
        requested_body_state_.store(-1);
        resetYbModeLatchAndConversions();

        left_thumbstick_offset_.setZero();
        right_thumbstick_offset_.setZero();
        left_thumbstick_yaw_offset_ = 0.0;
        right_thumbstick_yaw_offset_ = 0.0;
        body_thumbstick_z_yaw_mode_.store(false);
        waist_stick_axis_.store(WaistStickAxis::NONE);
        left_grip_mode_.store(false);
        right_grip_mode_.store(false);
        left_arm_paused_.store(false);
        right_arm_paused_.store(false);
        left_grip_press_armed_.store(false);
        right_grip_press_armed_.store(false);
        left_grip_combo_consumed_.store(false);
        right_grip_combo_consumed_.store(false);
        left_grip_direction_suppressed_.store(false);
        right_grip_direction_suppressed_.store(false);
        left_robot_base_valid_ = false;
        right_robot_base_valid_ = false;
        right_vr_target_suppressed_by_bimanual_ = false;
        resetStaleCatchUpRamp();
        stopHeadTracking();
    }

    void VRInputHandler::resumeNormalModeState()
    {
        // 退出全身模式后，普通模式从 STORAGE 开始；镜像按设计保持关闭。
        is_update_mode_.store(false);
        right_wbc_toggle_request_pending_.store(false);
        requested_wbc_toggle_target_.store(WbcToggleTarget::NONE);
        body_mode_request_pending_.store(false);
        requested_body_state_.store(-1);
        resetYbModeLatchAndConversions();

        left_thumbstick_offset_.setZero();
        right_thumbstick_offset_.setZero();
        left_thumbstick_yaw_offset_ = 0.0;
        right_thumbstick_yaw_offset_ = 0.0;
        body_thumbstick_z_yaw_mode_.store(false);
        waist_stick_axis_.store(WaistStickAxis::NONE);
        left_grip_mode_.store(false);
        right_grip_mode_.store(false);
        left_arm_paused_.store(false);
        right_arm_paused_.store(false);
        left_grip_press_armed_.store(false);
        right_grip_press_armed_.store(false);
        left_grip_combo_consumed_.store(false);
        right_grip_combo_consumed_.store(false);
        left_grip_direction_suppressed_.store(false);
        right_grip_direction_suppressed_.store(false);
        left_robot_base_valid_ = false;
        right_robot_base_valid_ = false;
        right_vr_target_suppressed_by_bimanual_ = false;
        resetStaleCatchUpRamp();
        stopHeadTracking();
    }

    bool VRInputHandler::publishArmTargetForWholeBody(
        bool is_left,
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation)
    {
        if (!ee_frame_id_initialized_ || !target_manager_)
        {
            return false;
        }
        TargetRouter* router = target_manager_->router();
        if (!router)
        {
            return false;
        }

        // 手动拖动胸部/手臂时让出控制权；拖动结束后让全身渐入重新以
        // 最后一次实际下发的目标为起点，避免直接跳过去。
        std::atomic<bool>& manual_hold = is_left
            ? left_manual_override_
            : right_manual_override_;
        if (manual_hold.load(std::memory_order_relaxed))
        {
            const Channel channel = is_left ? Channel::ArmLeft : Channel::ArmRight;
            if (!router->canWrite(channel, SourceId::Vr))
            {
                return false;  // 仍在手动拖动中，安静等待
            }
                rearmWholeBodyArmRamp(is_left);
            manual_hold.store(false, std::memory_order_relaxed);
            return false;
        }

        // 全身模式不与普通模式的“双臂解耦右臂重建”交互：直接提交骨架目标。
        Eigen::Vector3d rampedPosition = position;
        Eigen::Quaterniond rampedOrientation = orientation;
        applyStaleCatchUpRamp(is_left, rampedPosition, rampedOrientation);

        Intent intent;
        intent.channel = is_left ? Channel::ArmLeft : Channel::ArmRight;
        intent.source = SourceId::Vr;
        intent.priority = Priority::Normal;
        intent.position = rampedPosition;
        intent.orientation = rampedOrientation.normalized();
        intent.frame = ee_frame_id_;
        intent.lease = std::chrono::milliseconds(300);

        const Verdict verdict = router->submit(intent);
        if (!verdict.accepted)
        {
            if (std::string_view(verdict.reason) == "manual_override")
            {
                manual_hold.store(true, std::memory_order_relaxed);
            }
            RCLCPP_DEBUG_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "🕹️ 全身 %s 臂目标未下发：%s",
                is_left ? "left" : "right", verdict.reason);
            return false;
        }

        if (is_left)
        {
            prev_calculated_left_position_ = rampedPosition;
            prev_calculated_left_orientation_ = rampedOrientation.normalized();
        }
        else
        {
            prev_calculated_right_position_ = rampedPosition;
            prev_calculated_right_orientation_ = rampedOrientation.normalized();
        }

        recordLastPublishedTarget(is_left ? "left" : "right", rampedPosition, rampedOrientation);
        return true;
    }

    bool VRInputHandler::submitWholeBodyHeadTarget(
        const geometry_msgs::msg::Pose& pose,
        const std::string& frame_id)
    {
        if (!target_manager_)
        {
            return false;
        }
        TargetRouter* router = target_manager_->router();
        if (!router)
        {
            return false;
        }

        Intent intent;
        intent.channel = Channel::Head;
        intent.source = SourceId::Vr;
        intent.priority = Priority::Normal;
        intent.position = Eigen::Vector3d(
            pose.position.x, pose.position.y, pose.position.z);
        intent.orientation = Eigen::Quaterniond(
            pose.orientation.w, pose.orientation.x,
            pose.orientation.y, pose.orientation.z);
        intent.frame = frame_id;
        intent.lease = std::chrono::milliseconds(300);
        return router->submit(intent).accepted;
    }

    bool VRInputHandler::prepareWholeBodyArmInput(bool is_left)
    {
        const WbcToggleTarget arm =
            is_left ? WbcToggleTarget::LEFT_ARM : WbcToggleTarget::RIGHT_ARM;
        if (!isArmVrInputSuppressed(arm))
        {
            return true;
        }

        std::atomic<bool>* pending = is_left
            ? &left_wbc_rebase_pending_
            : &right_wbc_rebase_pending_;
        std::atomic<bool>* suppressed = is_left
            ? &left_wbc_vr_suppressed_
            : &right_wbc_vr_suppressed_;
        if (!pending->load())
        {
            return false;
        }

        // 全身模式不依赖手柄 pose：直接用实测末端位姿建立基准
        const std::string armType = is_left ? "left" : "right";
        if (!setRobotBase(armType, /*preferLastCommand=*/false))
        {
            return false;
        }
        resetStaleCatchUpRamp(is_left);
        clearLastPublishedTarget(armType);
        if (is_left)
        {
            prev_calculated_left_position_ = robot_current_left_position_;
            prev_calculated_left_orientation_ = robot_current_left_orientation_.normalized();
        }
        else
        {
            prev_calculated_right_position_ = robot_current_right_position_;
            prev_calculated_right_orientation_ = robot_current_right_orientation_.normalized();
        }

        pending->store(false);
        suppressed->store(false);
        // desiredOpticalR0RequestBits() 会把 WBC rebase pending 也算进 request，
        // 全身模式在这里自己完成 rebase，仍须刷新一次把该位清掉（ACK 给 fa-py）。
        refreshOpticalR0Request();
        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️ %s WBC arm VR control rebased from current pose (whole-body)",
            is_left ? "left" : "right");
        return true;
    }

    bool VRInputHandler::robotCurrentPose(
        bool is_left,
        Eigen::Vector3d& position,
        Eigen::Quaterniond& orientation) const
    {
        if (is_left)
        {
            if (!has_robot_current_left_pose_.load())
            {
                return false;
            }
            position = robot_current_left_position_;
            orientation = robot_current_left_orientation_;
            return true;
        }
        if (!has_robot_current_right_pose_.load())
        {
            return false;
        }
        position = robot_current_right_position_;
        orientation = robot_current_right_orientation_;
        return true;
    }

    bool VRInputHandler::robotHeadPose(geometry_msgs::msg::PoseStamped& pose) const
    {
        if (!has_robot_head_pose_)
        {
            return false;
        }
        pose = robot_head_pose_;
        return true;
    }

    bool VRInputHandler::robotBodyPose(
        Eigen::Vector3d& position,
        Eigen::Quaterniond& orientation,
        std::string& frame_id) const
    {
        if (!has_robot_body_pose_)
        {
            return false;
        }
        const auto& p = robot_body_pose_.pose.position;
        const auto& q = robot_body_pose_.pose.orientation;
        position = Eigen::Vector3d(p.x, p.y, p.z);
        orientation = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized();
        frame_id = robot_body_pose_.header.frame_id;
        return position.allFinite() && orientation.coeffs().allFinite();
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
                                "body_joint_controller/waist_lifting_command", 10);
                            pub_waist_turning_ = node_->create_publisher<std_msgs::msg::Float64>(
                                "body_joint_controller/waist_turning_command", 10);
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
                        }
                        // 全身遥操只在 FULL_BODY 下有效：由全身组件自行退出并恢复普通模式
                            onWholeBodyTopologyChanged(next == ControlTopology::FULL_BODY);
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

                    // 保持 1 Hz 轮询：运行期切换 arms/wbc 控制器时也要能触发拓扑变化处理
                    // （全身模式自动退出、腰部发布器重建等）。
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

    // ===== 共享工具：最后下发目标查询 =====

    VRInputHandler::LastArmTarget VRInputHandler::lastArmTarget(bool is_left) const
    {
        LastArmTarget target;
        if (is_left)
        {
            target.valid = has_last_published_left_target_;
            target.position = last_published_left_position_;
            target.orientation = last_published_left_orientation_;
            target.stamp = last_published_left_stamp_;
        }
        else
        {
            target.valid = has_last_published_right_target_;
            target.position = last_published_right_position_;
            target.orientation = last_published_right_orientation_;
            target.stamp = last_published_right_stamp_;
        }
        return target;
    }

} // namespace arms_ros2_control::command
