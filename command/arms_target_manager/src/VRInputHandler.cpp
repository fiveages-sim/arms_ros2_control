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
    const std::string VRInputHandler::XR_NODE_NAME = "/xr_target_node";
    const double VRInputHandler::POSITION_THRESHOLD = 0.01; // 1cm threshold for position changes
    const double VRInputHandler::ORIENTATION_THRESHOLD = 0.005; // threshold for orientation changes (quaternion angle)
    const int VRInputHandler::STALE_MIN_FROZEN_FRAMES = 2; // 连续 >=2 帧逐位相同即判定上游冻结

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
        const std::string& vr_follow_frame)
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
        // 创建 controller topology 检测 client 和启动期重试 timer
        list_controllers_client_ =
            node_->create_client<ListControllers>(
                "/controller_manager/list_controllers");

        control_topology_timer_ = node_->create_wall_timer(
            std::chrono::seconds(1),
            [this]() { detectControlTopology(); });

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
            "/xr/left_ee_pose", 10, vrLeftCallback);

        auto vrRightCallback = [this](const geometry_msgs::msg::Pose::SharedPtr msg)
        {
            this->vrRightCallback(msg);
        };
        sub_right_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/xr/right_ee_pose", 10, vrRightCallback);

        // 头显位姿订阅器
        auto vrHeadCallback = [this](const geometry_msgs::msg::Pose::SharedPtr msg)
        {
            this->vrHeadCallback(msg);
        };
        sub_head_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "/xr/head_pose", 10, vrHeadCallback);

        // 创建按钮事件订阅器（Int32类型）
        auto controllerStateCallback = [this](const std_msgs::msg::Int32::SharedPtr msg)
        {
            this->processButtonEvent(msg);
        };
        sub_controller_state_ = node_->create_subscription<std_msgs::msg::Int32>(
            "/xr/controller_state", 10, controllerStateCallback);
        
        // 创建摇杆轴值订阅器（合并订阅左右摇杆，使用 Twist 消息）
        // linear.x/y 表示左摇杆，angular.x/y 表示右摇杆
        auto thumbstickAxesCallback = [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            this->thumbstickAxesCallback(msg);
        };
        sub_thumbstick_axes_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "/xr/thumbstick_axes", 10, thumbstickAxesCallback);

        // 创建扳机模拟量订阅器（合并订阅左右扳机，使用 Twist 消息）
        // linear.x 表示左扳机拉动比例，angular.x 表示右扳机拉动比例
        auto triggerValuesCallback = [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            this->triggerValuesCallback(msg);
        };
        sub_trigger_values_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "/xr/trigger_values", 10, triggerValuesCallback);

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
        pub_waist_lifting_ = node_->create_publisher<std_msgs::msg::Float64>(
            "/body_joint_controller/waist_lifting_command", 10);
        pub_waist_turning_ = node_->create_publisher<std_msgs::msg::Float64>(
            "/body_joint_controller/waist_turning_command", 10);

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
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Subscribed to button event topic: /xr/controller_state (Int32)");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Subscribed to thumbstick axes topic: /xr/thumbstick_axes (ThumbstickAxes)");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Subscribed to trigger values topic: /xr/trigger_values (linear.x=left, angular.x=right)");
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
        // 目标流在此中断/重启，清空斜坡，避免用陈旧输出量缺口
        resetStaleCatchUpRamp();
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

        // 计算系：与 publishTargetPoseDirect 保持一致——FULL_BODY 下目标相对底盘保持
        // （vr_follow_frame_），其余情况计算系就是发布系。
        // 手柄-头显偏移量来自 VR 世界系（已在 xr 侧转成前x/左y/上z，与底盘轴系同向），
        // 必须在底盘系里做加法。直接加在 ee_frame_id_ 上时，轮式底盘的 ee_frame_id_
        // 是 world，底盘一转向偏移方向就整体转错。
        const std::string& calc_frame =
            isFullBodyMode() ? vr_follow_frame_ : ee_frame_id_;

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
                // last_published_* 存计算系的值（与 publishTargetPoseDirect 一致），
                // prev_calculated_* 存发布系的值——它被 hasPoseChanged() 无条件读取。
                recordLastPublishedTarget("left", left_target_pos_calc, left_target_ori_calc);
                recordLastPublishedTarget("right", right_target_pos_calc, right_target_ori_calc);
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

    bool VRInputHandler::checkNodeExists(const std::shared_ptr<rclcpp::Node>& node, const std::string& targetNodeName)
    {
        std::vector<std::string> nodeNames = node->get_node_graph_interface()->get_node_names();

        for (const auto& name : nodeNames)
        {
            if (name == targetNodeName)
            {
                return true;
            }
        }
        return false;
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
        if (!ee_frame_id_initialized_ && !msg->header.frame_id.empty())
        {
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
        if (!ee_frame_id_initialized_ && !msg->header.frame_id.empty())
        {
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
        Eigen::Matrix4d pose = poseMsgToMatrix(msg);
        matrixToPosOri(pose, vr_head_position_, vr_head_orientation_);
    }

    void VRInputHandler::vrLeftCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        if (checkNodeExists(node_, XR_NODE_NAME) && !enabled_.load())
        {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "🕹️🕶️🕹️ xr_target_node found, VR control ENABLED!");
            this->enable();
        }
        else if (!checkNodeExists(node_, XR_NODE_NAME) && enabled_.load())
        {
            this->disable();
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "🕹️🕶️🕹️ xr_target_node not found, VR control DISABLED!");
            return;
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
        }
        else if (armType == "right")
        {
            has_last_published_right_target_ = true;
            last_published_right_position_ = position;
            last_published_right_orientation_ = orientation;
            last_published_right_orientation_.normalize();
        }
    }

    void VRInputHandler::clearLastPublishedTargets()
    {
        has_last_published_left_target_ = false;
        last_published_left_position_ = Eigen::Vector3d::Zero();
        last_published_left_orientation_ = Eigen::Quaterniond::Identity();

        has_last_published_right_target_ = false;
        last_published_right_position_ = Eigen::Vector3d::Zero();
        last_published_right_orientation_ = Eigen::Quaterniond::Identity();

        left_robot_base_valid_ = false;
        right_robot_base_valid_ = false;

        resetStaleCatchUpRamp();
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

    bool VRInputHandler::setRobotBaseFromLastCommandOrCurrent(
        const std::string& armType)
    {
        const bool isLeft = armType == "left";
        const bool isRight = armType == "right";
        if (!isLeft && !isRight)
        {
            RCLCPP_WARN(
                node_->get_logger(),
                "🕹️ Cannot set VR robot base for unknown arm '%s'",
                armType.c_str());
            return false;
        }

        bool& valid = isLeft ? left_robot_base_valid_ : right_robot_base_valid_;
        bool& hasLast = isLeft
            ? has_last_published_left_target_
            : has_last_published_right_target_;
        Eigen::Vector3d& basePosition = isLeft
            ? robot_base_left_position_
            : robot_base_right_position_;
        Eigen::Quaterniond& baseOrientation = isLeft
            ? robot_base_left_orientation_
            : robot_base_right_orientation_;
        const Eigen::Vector3d& lastPosition = isLeft
            ? last_published_left_position_
            : last_published_right_position_;
        const Eigen::Quaterniond& lastOrientation = isLeft
            ? last_published_left_orientation_
            : last_published_right_orientation_;
        const Eigen::Vector3d& currentPosition = isLeft
            ? robot_current_left_position_
            : robot_current_right_position_;
        const Eigen::Quaterniond& currentOrientation = isLeft
            ? robot_current_left_orientation_
            : robot_current_right_orientation_;

        valid = false;
        if (hasLast)
        {
            basePosition = lastPosition;
            baseOrientation = lastOrientation.normalized();
            valid = true;
            return true;
        }

        if (!isFullBodyMode())
        {
            basePosition = currentPosition;
            baseOrientation = currentOrientation.normalized();
            valid = true;
            return true;
        }

        Eigen::Vector3d transformedPosition;
        Eigen::Quaterniond transformedOrientation;
        if (!ee_frame_id_initialized_ ||
            !transformPoseBetweenFrames(
                armType,
                currentPosition,
                currentOrientation,
                ee_frame_id_,
                vr_follow_frame_,
                transformedPosition,
                transformedOrientation))
        {
            return false;
        }

        basePosition = transformedPosition;
        baseOrientation = transformedOrientation;
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
        if (!valid && !setRobotBaseFromLastCommandOrCurrent(armType))
        {
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
            robotBasePosition,
            robotBaseOrientation,
            calculatedPosition,
            calculatedOrientation);
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
        right_robot_base_valid_ = false;
        const bool rightBaseReady =
            setRobotBaseFromLastCommandOrCurrent("right");

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
        // 放在 frame 变换之前，使 recordLastPublishedTarget() 记录的也是真正发出去的目标。
        Eigen::Vector3d rampedPosition = position;
        Eigen::Quaterniond rampedOrientation = orientation;
        applyStaleCatchUpRamp(isLeft, rampedPosition, rampedOrientation);

        Eigen::Vector3d publishPosition = rampedPosition;
        Eigen::Quaterniond publishOrientation = rampedOrientation;
        if (isFullBodyMode())
        {
            if (!ee_frame_id_initialized_ ||
                !transformPoseBetweenFrames(
                    armType,
                    rampedPosition,
                    rampedOrientation,
                    vr_follow_frame_,
                    ee_frame_id_,
                    publishPosition,
                    publishOrientation))
            {
                return false;
            }
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
            // 位置翻转（局部坐标系）
            vrPosDiff_local.x() = -vrPosDiff_local.x(); // 左右翻转
            vrPosDiff_local.y() = -vrPosDiff_local.y(); // 前后翻转
            // vrPosDiff_local.z() 保持不变（上下不翻转）

            // 旋转翻转（面对面镜像）
            vrOriDiff.y() = -vrOriDiff.y(); // 翻转Y分量
            vrOriDiff.x() = -vrOriDiff.x(); // 翻转X分量
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
            "🔘 [case %d] 左握把+左摇杆%s：已请求%s（%s），等待 WBC 确认和左摇杆回中",
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

    bool VRInputHandler::setRobotBaseFromCurrentPose(
        const std::string& armType)
    {
        const bool isLeft = armType == "left";
        const bool isRight = armType == "right";
        if (!isLeft && !isRight)
        {
            return false;
        }

        const bool hasCurrent = isLeft
            ? has_robot_current_left_pose_.load()
            : has_robot_current_right_pose_.load();
        bool& valid = isLeft
            ? left_robot_base_valid_
            : right_robot_base_valid_;
        valid = false;
        if (!hasCurrent || !ee_frame_id_initialized_)
        {
            return false;
        }

        const Eigen::Vector3d& currentPosition = isLeft
            ? robot_current_left_position_
            : robot_current_right_position_;
        const Eigen::Quaterniond& currentOrientation = isLeft
            ? robot_current_left_orientation_
            : robot_current_right_orientation_;
        Eigen::Vector3d transformedPosition;
        Eigen::Quaterniond transformedOrientation;
        if (!transformPoseBetweenFrames(
                armType,
                currentPosition,
                currentOrientation,
                ee_frame_id_,
                vr_follow_frame_,
                transformedPosition,
                transformedOrientation))
        {
            return false;
        }

        Eigen::Vector3d& basePosition = isLeft
            ? robot_base_left_position_
            : robot_base_right_position_;
        Eigen::Quaterniond& baseOrientation = isLeft
            ? robot_base_left_orientation_
            : robot_base_right_orientation_;
        basePosition = transformedPosition;
        baseOrientation = transformedOrientation.normalized();
        valid = true;
        return true;
    }

    void VRInputHandler::clearLastPublishedTarget(
        const std::string& armType)
    {
        if (armType == "left")
        {
            has_last_published_left_target_ = false;
            last_published_left_position_.setZero();
            last_published_left_orientation_.setIdentity();
            resetStaleCatchUpRamp(true);
        }
        else if (armType == "right")
        {
            has_last_published_right_target_ = false;
            last_published_right_position_.setZero();
            last_published_right_orientation_.setIdentity();
            resetStaleCatchUpRamp(false);
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
        if (!setRobotBaseFromCurrentPose(armType))
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

        if (body_thumbstick_z_yaw_mode_.load())
        {
            positionDelta[2] = -left_thumbstick_axes_.y() * vr_thumbstick_linear_scale_;
            rpyDelta[2] = -left_thumbstick_axes_.x() * vr_thumbstick_angular_scale_;
        }
        else
        {
            positionDelta[0] = -left_thumbstick_axes_.y() * vr_thumbstick_linear_scale_;
            positionDelta[1] = -left_thumbstick_axes_.x() * vr_thumbstick_linear_scale_;
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
            // 修饰符模式（按住右握把）：右摇杆 X → 腰部旋转，右摇杆 Y → 腰部升降
            cmd_vel.angular.z = 0.0; // 修饰符模式下不发底盘转向
            waist_lifting.data = right_y * waist_lifting_scale_;
            // 腰部旋转方向对齐 joystick_teleop.cpp（D-pad 右 → 负命令），
            // 摇杆右拨 → right_x>0 → 命令为负 → 腰部右转
            waist_turning.data = -right_x * waist_turning_scale_;
        }
        else
        {
            // 默认模式：右摇杆 X → 底盘转向 angular.z，右摇杆 Y 忽略
            cmd_vel.angular.z = right_x * chassis_angular_scale_;
            // waist_lifting / waist_turning 保持 0（已在上方初始化）
        }

        pub_cmd_vel_->publish(cmd_vel);
        pub_waist_lifting_->publish(waist_lifting);
        pub_waist_turning_->publish(waist_turning);

        RCLCPP_DEBUG(node_->get_logger(),
                     "🕹️ [Chassis] grip=%d L.xy=(%.3f,%.3f) R.xy=(%.3f,%.3f) → cmd_vel(lin=%.3f,%.3f ang=%.3f) waist(lift=%.3f turn=%.3f)",
                     right_grip ? 1 : 0,
                     left_thumbstick_axes_.x(), left_thumbstick_axes_.y(),
                     right_thumbstick_axes_.x(), right_thumbstick_axes_.y(),
                     cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z,
                     waist_lifting.data, waist_turning.data);
    }

    void VRInputHandler::resetChassisAndWaistCommands()
    {
        // 三个话题各发一次 0，防止底盘/腰部残留运动
        auto zero_vel = geometry_msgs::msg::Twist();
        pub_cmd_vel_->publish(zero_vel);

        auto zero_float = std_msgs::msg::Float64();
        zero_float.data = 0.0;
        pub_waist_lifting_->publish(zero_float);
        pub_waist_turning_->publish(zero_float);
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
                            setRobotBaseFromLastCommandOrCurrent("right");

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
                            setRobotBaseFromLastCommandOrCurrent("left");

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
            case 4:  // 右摇杆按钮
            {
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
                        setRobotBaseFromLastCommandOrCurrent("left");
                    const bool rightBaseReady =
                        setRobotBaseFromLastCommandOrCurrent("right");
                    RCLCPP_INFO(
                        node_->get_logger(),
                        "🕹️ UPDATE bases: left=%s, right=%s, calculation_frame=%s",
                        leftBaseReady ? "ready" : "pending_tf",
                        rightBaseReady ? "ready" : "pending_tf",
                        isFullBodyMode() ? vr_follow_frame_.c_str() : ee_frame_id_.c_str());

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
                    // 目标流在此中断/重启，清空斜坡，避免用陈旧输出量缺口
                    resetStaleCatchUpRamp();
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
                            setRobotBaseFromLastCommandOrCurrent("left");

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
                            setRobotBaseFromLastCommandOrCurrent("right");

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
            case 7:  // 镜像模式切换（toggle）
            {
                bool old_mirror_mode = mirror_mode_.load();
                bool new_mirror_mode = !old_mirror_mode;
                mirror_mode_.store(new_mirror_mode);
                
                // 镜像模式发生变化，记录日志并自动切换到 STORAGE 模式
                if (new_mirror_mode)
                {
                    RCLCPP_INFO(node_->get_logger(),
                                "🔘 [左摇杆按钮] 按下 - 功能: 切换镜像模式 - 操作: 启用镜像模式（左手柄控制右臂，右手柄控制左臂）");
                }
                else
                {
                    RCLCPP_INFO(node_->get_logger(),
                                "🔘 [左摇杆按钮] 按下 - 功能: 切换镜像模式 - 操作: 禁用镜像模式（恢复正常控制）");
                }
                
                // 切换镜像模式后，自动切换到STORAGE模式，避免跳变
                if (is_update_mode_.load())
                {
                    is_update_mode_.store(false);
                    // 目标流在此中断/重启，清空斜坡，避免用陈旧输出量缺口
                    resetStaleCatchUpRamp();
                    // 重置摇杆累积偏移
                    left_thumbstick_offset_ = Eigen::Vector3d::Zero();
                    right_thumbstick_offset_ = Eigen::Vector3d::Zero();
                    left_thumbstick_yaw_offset_ = 0.0;
                    right_thumbstick_yaw_offset_ = 0.0;
                    // 清除暂停时刻的VR位姿记录（镜像模式切换后不再适用）
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
                requestBodyMode(
                    21, "向上", "竖直", "BODY_RELATIVE",
                    arms_ros2_control_msgs::msg::WbcCurrentState::BODY_VERTICAL);
                break;
            case 22: // 左握把 + 左摇杆向下：锁定
                markGripComboConsumed(true, false);
                requestBodyMode(
                    22, "向下", "锁定", "BODY_LOCK",
                    arms_ros2_control_msgs::msg::WbcCurrentState::BODY_LOCKED);
                break;
            case 23: // 左握把 + 左摇杆向左：跟随
                markGripComboConsumed(true, false);
                requestBodyMode(
                    23, "向左", "跟随", "BODY_TRACKING",
                    arms_ros2_control_msgs::msg::WbcCurrentState::BODY_TRACKING);
                break;
            case 24: // 左握把 + 左摇杆向右：自定义
                markGripComboConsumed(true, false);
                requestBodyMode(
                    24, "向右", "自定义", "BODY_CUSTOM_LOCK",
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
                        clearLastPublishedTargets();
                        if (next != ControlTopology::FULL_BODY)
                        {
                            resetYbModeLatchAndConversions();
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
            return;
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
                    // 目标流在此中断/重启，清空斜坡，避免用陈旧输出量缺口
                    resetStaleCatchUpRamp();
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
    }
} // namespace arms_ros2_control::command
