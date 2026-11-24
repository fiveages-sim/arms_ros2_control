//
// Created for Arms ROS2 Control - VRInputHandler
//

#include "arms_target_manager/VRInputHandler.h"
#include "arms_controller_common/utils/FSMStateTransitionValidator.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace arms_ros2_control::command
{
    const int VRInputHandler::STALE_MIN_FROZEN_FRAMES = 2;

    VRInputHandler::VRInputHandler(
        rclcpp::Node::SharedPtr node,
        ArmsTargetManager* targetManager,
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_left_target,
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_right_target,
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_left_twist,
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_right_twist,
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_head_target,
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_dual_target_stamped,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        const std::vector<std::string>& handControllers,
        double linear_scale,
        double angular_scale,
        double vr_pose_scale,
        const std::string& reference_link,
        const std::string& vr_follow_frame)
        : node_(std::move(node))
          , target_manager_(targetManager)
          , pub_left_target_(std::move(pub_left_target))
          , pub_right_target_(std::move(pub_right_target))
          , pub_dual_target_stamped_(std::move(pub_dual_target_stamped))
          , pub_left_twist_(std::move(pub_left_twist))
          , pub_right_twist_(std::move(pub_right_twist))
          , pub_head_target_(std::move(pub_head_target))
          , enabled_(false)
          , is_update_mode_(false)
          , mirror_mode_(false)
          , left_arm_paused_(false)
          , right_arm_paused_(false)
          , left_grip_mode_(false)
          , right_grip_mode_(false)
          , chassis_mode_(false)
          , current_fsm_state_(2)
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
          , linear_scale_(linear_scale)
          , angular_scale_(angular_scale)
          , vr_pose_scale_(vr_pose_scale)
          , reference_link_(reference_link)
          , vr_follow_frame_(vr_follow_frame)
          , tf_buffer_(std::move(tf_buffer))
    {
        list_controllers_client_ =
            node_->create_client<ListControllers>(
                "controller_manager/list_controllers");

        control_topology_timer_ = node_->create_wall_timer(
            std::chrono::seconds(1),
            [this]() { detectControlTopology(); });

        detectGripperControllers(hand_controllers_);

        if (!left_gripper_controller_name_.empty())
        {
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

        auto vrLeftCallback = [this](const geometry_msgs::msg::Pose::SharedPtr msg)
        {
            this->vrLeftCallback(msg);
        };
        sub_left_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "teleop/left_ee_pose", 10, vrLeftCallback);

        auto vrRightCallback = [this](const geometry_msgs::msg::Pose::SharedPtr msg)
        {
            this->vrRightCallback(msg);
        };
        sub_right_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "teleop/right_ee_pose", 10, vrRightCallback);

        auto vrHeadCallback = [this](const geometry_msgs::msg::Pose::SharedPtr msg)
        {
            this->vrHeadCallback(msg);
        };
        sub_head_ = node_->create_subscription<geometry_msgs::msg::Pose>(
            "teleop/head_pose", 10, vrHeadCallback);

        sub_robot_head_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "head_current_pose", 10,
            [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { robotHeadPoseCallback(msg); });

        auto controllerStateCallback = [this](const std_msgs::msg::Int32::SharedPtr msg)
        {
            this->processButtonEvent(msg);
        };
        sub_controller_state_ = node_->create_subscription<std_msgs::msg::Int32>(
            "teleop/controller_state", 10, controllerStateCallback);

        pub_optical_r0_request_ = node_->create_publisher<std_msgs::msg::Int32>(
            "teleop/optical_r0_request", 10);
        sub_optical_r0_caps_ = node_->create_subscription<std_msgs::msg::Int32>(
            "teleop/optical_r0_caps",
            rclcpp::QoS(1).transient_local(),
            [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                this->opticalR0CapsCallback(msg);
            });
        sub_optical_r0_ready_ = node_->create_subscription<std_msgs::msg::Int32>(
            "teleop/optical_r0_ready", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                this->opticalR0ReadyCallback(msg);
            });
        optical_handshake_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() { this->superviseOpticalHandshake(); });
        RCLCPP_INFO(
            node_->get_logger(),
            "🕹️ Optional optical R0 handshake "
            "(teleop/optical_r0_caps|_request ↔ teleop/optical_r0_ready; "
            "else main topic R0)");

        auto thumbstickAxesCallback = [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            this->thumbstickAxesCallback(msg);
        };
        sub_thumbstick_axes_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "teleop/thumbstick_axes", 10, thumbstickAxesCallback);

        auto triggerValuesCallback = [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            this->triggerValuesCallback(msg);
        };
        sub_trigger_values_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "teleop/trigger_values", 10, triggerValuesCallback);

        auto pub_fsm_command = node_->create_publisher<std_msgs::msg::Int32>("fsm_command", 10);
        fsm_command_publisher_ = std::make_unique<arms_controller_common::FSMCommandPublisher>(
            node_, pub_fsm_command);

        pub_cmd_vel_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        pub_body_twist_ = node_->create_publisher<geometry_msgs::msg::Twist>("body_target/twist", 10);

        pub_mode_command_ = node_->create_publisher<std_msgs::msg::String>("mode_command", 10);

        pub_teleop_mode_state_ =
            node_->create_publisher<arms_ros2_control_msgs::msg::TeleopModeState>(
                "teleop/mode_state",
                rclcpp::QoS(1).reliable().transient_local());
        publishTeleopModeState(true);

        sub_wbc_capability_ =
            node_->create_subscription<arms_ros2_control_msgs::msg::WbcCapability>(
                "ocs2_wbc_controller/wbc_capabilities",
                rclcpp::QoS(1).transient_local(),
                [this](const arms_ros2_control_msgs::msg::WbcCapability::ConstSharedPtr msg)
                {
                    this->wbcCapabilityCallback(msg);
                });


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
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Subscribed to button event topic: teleop/controller_state (Int32)");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Subscribed to thumbstick axes topic: teleop/thumbstick_axes");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Subscribed to trigger values topic: teleop/trigger_values (linear.x=left, angular.x=right)");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Trigger hand mode: percent control by default; press left Y + right B together to toggle percent/switch mode");
        RCLCPP_INFO(node_->get_logger(),
                    "🕹️🕶️🕹️ Thumbstick scales: linear=%.3f m/s, angular=%.3f rad/s",
                    linear_scale_, angular_scale_);
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VR pose scale: left=%.3f, right=%.3f",
                    vr_pose_scale_, vr_pose_scale_);
        RCLCPP_INFO(node_->get_logger(),
                    "🕹️🕶️🕹️ Grip short-press toggles thumbstick mode: XY-translation ↔ Z-height + Yaw-rotation");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VR control is DISABLED by default.");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Right thumbstick toggles between STORAGE and UPDATE modes.");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ STORAGE mode: Store VR and robot base poses (no marker update)");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ UPDATE mode: Calculate pose differences and update markers");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ MIRROR mode: Synced from xr_target_node (left thumbstick toggles)");
        
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
        left_gripper_controller_name_.clear();
        right_gripper_controller_name_.clear();

        for (const auto& controller_name : hand_controllers)
        {
            std::string name_lower = controller_name;
            std::transform(name_lower.begin(), name_lower.end(), name_lower.begin(), ::tolower);

            if (name_lower.find("left") != std::string::npos)
            {
                left_gripper_controller_name_ = controller_name;
                RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Detected left gripper controller: %s", controller_name.c_str());
            }
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

        if (gripper_command_publishers_.find(controller_name) == gripper_command_publishers_.end())
        {
            std::string topic_name = controller_name + "/target_command";
            gripper_command_publishers_[controller_name] = 
                node_->create_publisher<std_msgs::msg::Int32>(topic_name, 10);
            RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Created gripper command publisher: %s", topic_name.c_str());
        }

        auto target_msg = std_msgs::msg::Int32();
        target_msg.data = command;
        gripper_command_publishers_[controller_name]->publish(target_msg);

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
            std::string topic_name = controller_name + "/target_percent";
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
            auto it = last_gripper_percent_.find(controller_name);
            const double hold = (it != last_gripper_percent_.end())
                ? std::clamp(it->second, 0.0, 1.0)
                : 1.0;

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
            auto it = last_gripper_percent_.find(controller_name);
            if (it != last_gripper_percent_.end())
            {
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
        publishTeleopModeState();
    }

    void VRInputHandler::publishTeleopModeState(bool force)
    {
        if (!pub_teleop_mode_state_)
        {
            return;
        }

        using Msg = arms_ros2_control_msgs::msg::TeleopModeState;
        Msg msg;
        msg.stamp = node_->get_clock()->now();
        msg.enabled = enabled_.load();
        msg.update_mode = is_update_mode_.load();
        msg.chassis_mode = chassis_mode_.load();
        msg.yb_pause_semantics = full_body_yb_pause_mode_.load();
        msg.left_arm_paused = left_arm_paused_.load();
        msg.right_arm_paused = right_arm_paused_.load();
        msg.left_stick_plane = left_grip_mode_.load()
            ? Msg::PLANE_Z_YAW
            : Msg::PLANE_XY;
        msg.right_stick_plane = right_grip_mode_.load()
            ? Msg::PLANE_Z_YAW
            : Msg::PLANE_XY;
        msg.body_stick_plane = body_left_stick_alt_mode_.load()
            ? Msg::PLANE_Z_YAW
            : Msg::PLANE_XY;
        msg.body_right_stick_plane = body_right_stick_alt_mode_.load()
            ? Msg::PLANE_Z_YAW
            : Msg::PLANE_XY;
        msg.left_gripper_percent_mode = left_gripper_percent_mode_.load();
        msg.right_gripper_percent_mode = right_gripper_percent_mode_.load();
        msg.mirror_movement = mirror_mode_.load();
        msg.mirror_profile = static_cast<uint8_t>(mirror_mode_profile_.load());
        switch (control_topology_.load())
        {
            case ControlTopology::FULL_BODY:
                msg.topology = Msg::TOPOLOGY_FULL_BODY;
                break;
            case ControlTopology::SPLIT_BODY:
                msg.topology = Msg::TOPOLOGY_SPLIT_BODY;
                break;
            default:
                msg.topology = Msg::TOPOLOGY_UNKNOWN;
                break;
        }

        if (!force && has_published_teleop_mode_state_)
        {
            const auto& prev = last_teleop_mode_state_;
            if (prev.enabled == msg.enabled &&
                prev.update_mode == msg.update_mode &&
                prev.chassis_mode == msg.chassis_mode &&
                prev.yb_pause_semantics == msg.yb_pause_semantics &&
                prev.left_arm_paused == msg.left_arm_paused &&
                prev.right_arm_paused == msg.right_arm_paused &&
                prev.left_stick_plane == msg.left_stick_plane &&
                prev.right_stick_plane == msg.right_stick_plane &&
                prev.body_stick_plane == msg.body_stick_plane &&
                prev.body_right_stick_plane == msg.body_right_stick_plane &&
                prev.left_gripper_percent_mode == msg.left_gripper_percent_mode &&
                prev.right_gripper_percent_mode == msg.right_gripper_percent_mode &&
                prev.mirror_movement == msg.mirror_movement &&
                prev.mirror_profile == msg.mirror_profile &&
                prev.topology == msg.topology)
            {
                return;
            }
        }

        pub_teleop_mode_state_->publish(msg);
        last_teleop_mode_state_ = msg;
        has_published_teleop_mode_state_ = true;
    }

    void VRInputHandler::opticalR0ReadyCallback(
        const std_msgs::msg::Int32::SharedPtr msg)
    {
        optical_r0_ready_bits_.store(msg->data);
        optical_r0_ready_last_ns_.store(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
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

    void VRInputHandler::cancelOpticalPending(const char* reason)
    {
        const bool hadUpdate = optical_update_enter_pending_.exchange(false);
        const bool hadLeft = optical_resume_pending_left_.exchange(false);
        const bool hadRight = optical_resume_pending_right_.exchange(false);
        if (!hadUpdate && !hadLeft && !hadRight)
        {
            return;
        }
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

    void VRInputHandler::tryCompleteOpticalUpdateEnter()
    {
        if (!optical_update_enter_pending_.load() || is_update_mode_.load())
        {
            return;
        }
        if (current_fsm_state_.load() != 3)
        {
            cancelOpticalPending("left OCS2 during UPDATE enter");
            return;
        }
        const bool timed_out =
            std::chrono::steady_clock::now() - optical_update_enter_pending_since_ >=
            optical_r0_handshake_timeout_;
        if (!opticalR0HandshakeAlive())
        {
            if (timed_out)
            {
                cancelOpticalPending("no ready frame before UPDATE enter");
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
        optical_r0_ready_bits_.store(0);
        refreshOpticalR0Request();
        resetStaleCatchUpRamp();
        publishTeleopModeState();
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
            return;
        }
        if (alive)
        {
            refreshOpticalR0Request();
        }
        else if (left_due || right_due)
        {
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
            const bool snapped = alive && tryCompleteOpticalSideSnap(useLeftVr);
            if (!snapped && !due)
            {
                return false;
            }
            if (!snapped)
            {
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
            {                left_arm_paused_.store(false);
            }
            else
            {                right_arm_paused_.store(false);
            }
            pending.store(false);
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
            publishTeleopModeState();
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

    bool VRInputHandler::toggleArmVrPause(
        WbcToggleTarget arm,
        const char* tag)
    {
        const bool isLeftArm = arm == WbcToggleTarget::LEFT_ARM;
        const bool isRightArm = arm == WbcToggleTarget::RIGHT_ARM;
        if (!isLeftArm && !isRightArm)
        {
            return false;
        }
        if (!is_update_mode_.load())
        {
            RCLCPP_DEBUG(
                node_->get_logger(),
                "🔘 [%s] 忽略 VR 暂停切换：当前不在 UPDATE 模式",
                tag);
            return false;
        }

        const bool paused = isLeftArm
            ? left_arm_paused_.load()
            : right_arm_paused_.load();
        if (paused)
        {
            return resumeArmVrControl(arm, tag);
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
        publishTeleopModeState();
        RCLCPP_INFO(
            node_->get_logger(),
            "🔘 [%s] VR 暂停%s臂更新（已记录暂停时刻 VR 位姿）%s",
            tag,
            isLeftArm ? "左" : "右",
            mirror_mode_.load() ? " [镜像模式]" : "");
        return true;
    }

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
        {            left_arm_paused_.store(false);
        }
        else
        {            right_arm_paused_.store(false);
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
        publishTeleopModeState();
        return true;
    }

    void VRInputHandler::runScaleCalibration(bool use_left_z)
    {
        const char* tag = use_left_z ? "左组合键" : "右组合键";

        mirror_mode_.store(false);
        is_update_mode_.store(false);
        stopHeadTracking();
        publishTeleopModeState();
        RCLCPP_INFO(node_->get_logger(), "🔘 [%s] 已切换至非镜像模式", tag);
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

                recordLastPublishedTarget("left", left_pub_pos, left_pub_ori);
                recordLastPublishedTarget("right", right_pub_pos, right_pub_ori);
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
        if (!enabled_.load() || !is_update_mode_.load())
        {
            return;
        }

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
        int32_t command = current_gripper_open ? 0 : 1;
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
        left_gripper_open_.store(msg->data == 1);
        if (!left_gripper_controller_name_.empty())
        {
            last_gripper_percent_[left_gripper_controller_name_] = (msg->data == 1) ? 1.0 : 0.0;
        }
        RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Left gripper state synced: %s",
                     (msg->data == 1) ? "open" : "close");
    }

    void VRInputHandler::rightGripperStateCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        right_gripper_open_.store(msg->data == 1);
        if (!right_gripper_controller_name_.empty())
        {
            last_gripper_percent_[right_gripper_controller_name_] = (msg->data == 1) ? 1.0 : 0.0;
        }
        RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Right gripper state synced: %s",
                     (msg->data == 1) ? "open" : "close");
    }

    void VRInputHandler::sendFsmCommand(int32_t command)
    {
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
        if (target_manager_ && target_manager_->getCurrentMode() != MarkerState::CONTINUOUS)
        {
            target_manager_->togglePublishMode();
            RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ ArmsTargetManager switched to CONTINUOUS mode for VR control");
        }
        publishTeleopModeState();
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VR control ENABLED!");
    }

    void VRInputHandler::disable()
    {
        enabled_.store(false);
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

        if (chassis_mode_.load())
        {
            chassis_mode_.store(false);
        }
        body_left_stick_alt_mode_.store(false);
        body_right_stick_alt_mode_.store(false);
        body_tracking_arms_held_.store(false);
        left_arm_pose_muted_.store(false);
        right_arm_pose_muted_.store(false);
        publishArmTwist(true, geometry_msgs::msg::Twist{});
        publishArmTwist(false, geometry_msgs::msg::Twist{});
        pub_body_twist_->publish(geometry_msgs::msg::Twist{});
        resetChassisAndWaistCommands();
        publishTeleopModeState();
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VR control DISABLED!");
    }

    void VRInputHandler::robotLeftPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
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

        auto pose_msg = std::make_shared<geometry_msgs::msg::Pose>(msg->pose);
        Eigen::Matrix4d pose = poseMsgToMatrix(pose_msg);
        matrixToPosOri(pose, robot_current_left_position_, robot_current_left_orientation_);
        has_robot_current_left_pose_.store(true);
    }

    void VRInputHandler::robotRightPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
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
            has_vr_head_pose_ && has_robot_head_pose_;
    }

    void VRInputHandler::stopHeadTracking()
    {
        head_tracking_active_ = false;
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
            vr_head_base_position_ = vr_head_position_;
            vr_head_base_orientation_ = vr_head_orientation_;
        }

        const Eigen::Vector3d position = robot_head_base_position_ +
            (vr_head_position_ - vr_head_base_position_);
        const Eigen::Quaterniond orientation = (vr_head_orientation_ *
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
        head_tracking_active_ = true;
    }

    void VRInputHandler::vrLeftCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        if (!enabled_.load())
        {
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "🕹️🕶️🕹️ VR left pose received, VR control ENABLED!");
            this->enable();
        }

        left_ee_pose_ = poseMsgToMatrix(msg);
        matrixToPosOri(left_ee_pose_, vr_left_position_raw_, left_orientation_);
        left_position_ = vr_left_position_raw_;
        left_position_ *= vr_pose_scale_;

        has_vr_left_pose_.store(true);
        tryCompleteOpticalUpdateEnter();
        tryCompleteOpticalResumes();
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
            bool is_mirror = mirror_mode_.load();
            bool arm_paused = is_mirror ? right_arm_paused_.load() : left_arm_paused_.load();

            if (is_update_mode_.load())
            {
                Eigen::Vector3d calculatedPos;
                Eigen::Quaterniond calculatedOri;

                if (is_mirror)
                {
                    Eigen::Vector3d vr_position = arm_paused ? paused_left_position_ : left_position_;
                    Eigen::Quaterniond vr_orientation = arm_paused ? paused_left_orientation_ : left_orientation_;
                    
                    if (calculateAndPublishTarget(
                            "right",
                            vr_position,
                            vr_orientation,
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
                    Eigen::Vector3d vr_position = arm_paused ? paused_left_position_ : left_position_;
                    Eigen::Quaterniond vr_orientation = arm_paused ? paused_left_orientation_ : left_orientation_;
                    
                    if (calculateAndPublishTarget(
                            "left",
                            vr_position,
                            vr_orientation,
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
        }
    }

    void VRInputHandler::vrRightCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        right_ee_pose_ = poseMsgToMatrix(msg);
        matrixToPosOri(right_ee_pose_, vr_right_position_raw_, right_orientation_);
        right_position_ = vr_right_position_raw_;
        right_position_ *= vr_pose_scale_;

        has_vr_right_pose_.store(true);
        tryCompleteOpticalUpdateEnter();
        tryCompleteOpticalResumes();
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
            bool is_mirror = mirror_mode_.load();
            bool arm_paused = is_mirror ? left_arm_paused_.load() : right_arm_paused_.load();

            if (is_update_mode_.load())
            {
                Eigen::Vector3d calculatedPos;
                Eigen::Quaterniond calculatedOri;

                if (is_mirror)
                {
                    Eigen::Vector3d vr_position = arm_paused ? paused_right_position_ : right_position_;
                    Eigen::Quaterniond vr_orientation = arm_paused ? paused_right_orientation_ : right_orientation_;
                    
                    if (calculateAndPublishTarget(
                            "left",
                            vr_position,
                            vr_orientation,
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
                    Eigen::Vector3d vr_position = arm_paused ? paused_right_position_ : right_position_;
                    Eigen::Quaterniond vr_orientation = arm_paused ? paused_right_orientation_ : right_orientation_;
                    
                    if (calculateAndPublishTarget(
                            "right",
                            vr_position,
                            vr_orientation,
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
        if (isArmPoseMuted(isLeft))
        {
            return false;
        }

        bool& valid = isLeft ? left_robot_base_valid_ : right_robot_base_valid_;
        if (!valid)
        {
            if (setRobotBase(armType, /*preferLastCommand=*/true))
            {
                const bool useLeftVr = mirror_mode_.load() ? !isLeft : isLeft;
                const bool paused = isLeft ? left_arm_paused_.load() : right_arm_paused_.load();
                (useLeftVr ? vr_base_left_position_ : vr_base_right_position_) = useLeftVr
                    ? (paused ? paused_left_position_ : left_position_)
                    : (paused ? paused_right_position_ : right_position_);
                (useLeftVr ? vr_base_left_orientation_ : vr_base_right_orientation_) = useLeftVr
                    ? (paused ? paused_left_orientation_ : left_orientation_)
                    : (paused ? paused_right_orientation_ : right_orientation_);
            }
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
        has_last_published_right_target_ = false;
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

        Eigen::Vector3d rampedPosition = position;
        Eigen::Quaterniond rampedOrientation = orientation;
        applyStaleCatchUpRamp(isLeft, rampedPosition, rampedOrientation);

        Eigen::Vector3d publishPosition = rampedPosition;
        Eigen::Quaterniond publishOrientation = rampedOrientation;
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
        Eigen::Vector3d vrPosDiff_world = vrCurrentPos - vrBasePos;

        Eigen::Vector3d vrPosDiff_local = vrBaseOri.inverse() * vrPosDiff_world;

        Eigen::Quaterniond vrOriDiff = vrBaseOri.inverse() * vrCurrentOri;

        if (mirror_mode_.load())
        {
            const auto profile = static_cast<MirrorModeProfile>(mirror_mode_profile_.load());

            if (profile == MirrorModeProfile::MODE_A)
            {
                vrPosDiff_local.y() = -vrPosDiff_local.y();

                vrOriDiff.x() = -vrOriDiff.x();
                vrOriDiff.z() = -vrOriDiff.z();
            }
            else
            {
                vrPosDiff_local.x() = -vrPosDiff_local.x();
                vrPosDiff_local.y() = -vrPosDiff_local.y();

                vrOriDiff.y() = -vrOriDiff.y();
                vrOriDiff.x() = -vrOriDiff.x();
            }

            vrOriDiff.normalize();
        }

        resultPos = robotBasePos + vrPosDiff_local;

        resultOri = vrOriDiff * robotBaseOri;

        resultOri.normalize();
    }

    void VRInputHandler::thumbstickAxesCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        left_thumbstick_axes_.x() = msg->linear.x;
        left_thumbstick_axes_.y() = msg->linear.y;
        left_thumbstick_axes_raw_ = left_thumbstick_axes_;

        right_thumbstick_axes_.x() = msg->angular.x;
        right_thumbstick_axes_.y() = msg->angular.y;
        right_thumbstick_axes_raw_ = right_thumbstick_axes_;

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

        if (chassis_mode_.load())
        {
            processChassisAxes();
            return;
        }

        if (updateBodyModeRequestState())
        {
            processRightThumbstickAxes();
            return;
        }

        syncBodyTrackingArmPoseHold();
        if (isBodyTrackingActive())
        {
            processBodyThumbstickAxes();
            return;
        }

        const bool left_body_reset = body_left_stick_alt_mode_.exchange(false);
        const bool right_body_reset = body_right_stick_alt_mode_.exchange(false);
        if (left_body_reset || right_body_reset)
        {
            publishTeleopModeState();
        }

        processLeftThumbstickAxes();
        processRightThumbstickAxes();
    }

    void VRInputHandler::triggerValuesCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        if (!enabled_.load() || !is_update_mode_.load())
        {
            return;
        }

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

                armed.store(true);
                RCLCPP_INFO(node_->get_logger(),
                            "🕹️ [%s夹爪] 比例控制已接管（接管点 %.2f）", side, hand_percent);
            }
            publishGripperPercent(controller_name, hand_percent);
        };

        publishIfPercent(true, left_hand_percent);
        publishIfPercent(false, right_hand_percent);
    }
    

    bool VRInputHandler::isArmPoseMuted(bool is_left_arm) const
    {
        return is_left_arm ? left_arm_pose_muted_.load()
                           : right_arm_pose_muted_.load();
    }

    void VRInputHandler::publishArmTwist(
        bool is_left_arm,
        const geometry_msgs::msg::Twist& twist)
    {
        if (is_left_arm)
        {
            pub_left_twist_->publish(twist);
            return;
        }
        if (pub_right_twist_)
        {
            pub_right_twist_->publish(twist);
        }
    }

    void VRInputHandler::enterArmStickDrive(bool is_left_arm)
    {
        (is_left_arm ? left_arm_pose_muted_ : right_arm_pose_muted_).store(true);
    }

    void VRInputHandler::syncBodyTrackingArmPoseHold()
    {
        if (isBodyTrackingActive())
        {
            if (!body_tracking_arms_held_.exchange(true))
            {
                enterArmStickDrive(true);
                enterArmStickDrive(false);
                publishArmTwist(true, geometry_msgs::msg::Twist{});
                publishArmTwist(false, geometry_msgs::msg::Twist{});
            }
            return;
        }

        if (!body_tracking_arms_held_.exchange(false))
        {
            return;
        }

        const bool mirror = mirror_mode_.load();
        exitArmStickDrive(true, /*use_left_vr=*/!mirror);
        exitArmStickDrive(false, /*use_left_vr=*/mirror);
        pub_body_twist_->publish(geometry_msgs::msg::Twist{});
    }

    void VRInputHandler::exitArmStickDrive(bool is_left_arm, bool use_left_vr)
    {
        auto& muted = is_left_arm ? left_arm_pose_muted_ : right_arm_pose_muted_;
        if (!muted.load())
        {
            return;
        }

        geometry_msgs::msg::Twist zero;
        publishArmTwist(is_left_arm, zero);

        const bool arm_paused = is_left_arm ? left_arm_paused_.load()
                                            : right_arm_paused_.load();
        if (arm_paused)
        {
            if (use_left_vr)
            {
                vr_base_left_position_ = paused_left_position_;
                vr_base_left_orientation_ = paused_left_orientation_;
            }
            else
            {
                vr_base_right_position_ = paused_right_position_;
                vr_base_right_orientation_ = paused_right_orientation_;
            }
        }
        else if (use_left_vr)
        {
            vr_base_left_position_ = left_position_;
            vr_base_left_orientation_ = left_orientation_;
        }
        else
        {
            vr_base_right_position_ = right_position_;
            vr_base_right_orientation_ = right_orientation_;
        }
        const char* armName = is_left_arm ? "left" : "right";
        geometry_msgs::msg::PoseStamped current_target;
        const bool hasTarget = target_manager_ &&
            target_manager_->getLatestCurrentTarget(armName, current_target);
        if (hasTarget)
        {
            Eigen::Vector3d p(
                current_target.pose.position.x,
                current_target.pose.position.y,
                current_target.pose.position.z);
            Eigen::Quaterniond q(
                current_target.pose.orientation.w,
                current_target.pose.orientation.x,
                current_target.pose.orientation.y,
                current_target.pose.orientation.z);
            q.normalize();
            if (is_left_arm)
            {
                last_published_left_position_ = p;
                last_published_left_orientation_ = q;
                has_last_published_left_target_ = true;
            }
            else
            {
                last_published_right_position_ = p;
                last_published_right_orientation_ = q;
                has_last_published_right_target_ = true;
            }
            (void)setRobotBase(armName, /*preferLastCommand=*/true);
        }
        else
        {
            (void)setRobotBase(armName, /*preferLastCommand=*/false);
        }
        muted.store(false);
    }

    void VRInputHandler::processLeftThumbstickAxes()
    {
        if (!enabled_.load() || !is_update_mode_.load())
        {
            return;
        }
        const bool is_mirror = mirror_mode_.load();
        const bool is_left_arm = !is_mirror;
        const WbcToggleTarget controlledArm = is_mirror
            ? WbcToggleTarget::RIGHT_ARM
            : WbcToggleTarget::LEFT_ARM;
        if (isArmVrInputSuppressed(controlledArm))
        {
            return;
        }

        const bool alt = is_mirror ? right_grip_mode_.load() : left_grip_mode_.load();
        geometry_msgs::msg::Twist twist;
        if (alt)
        {
            twist.linear.z = -left_thumbstick_axes_.y() * linear_scale_;
            twist.angular.z = -left_thumbstick_axes_.x() * angular_scale_;
        }
        else
        {
            twist.linear.x = -left_thumbstick_axes_.y() * linear_scale_;
            twist.linear.y = -left_thumbstick_axes_.x() * linear_scale_;
        }

        const bool deflected =
            std::abs(left_thumbstick_axes_.x()) > arm_stick_deadzone_ ||
            std::abs(left_thumbstick_axes_.y()) > arm_stick_deadzone_;
        if (deflected)
        {
            enterArmStickDrive(is_left_arm);
            publishArmTwist(is_left_arm, twist);
        }
        else if (isArmPoseMuted(is_left_arm))
        {
            exitArmStickDrive(is_left_arm, /*use_left_vr=*/true);
        }
    }

    void VRInputHandler::processRightThumbstickAxes()
    {
        if (!enabled_.load() || !is_update_mode_.load())
        {
            return;
        }
        const bool is_mirror = mirror_mode_.load();
        const bool is_left_arm = is_mirror;
        const WbcToggleTarget controlledArm = is_mirror
            ? WbcToggleTarget::LEFT_ARM
            : WbcToggleTarget::RIGHT_ARM;
        if (isArmVrInputSuppressed(controlledArm))
        {
            return;
        }

        const bool alt = is_mirror ? left_grip_mode_.load() : right_grip_mode_.load();
        geometry_msgs::msg::Twist twist;
        if (alt)
        {
            twist.linear.z = -right_thumbstick_axes_.y() * linear_scale_;
            twist.angular.z = -right_thumbstick_axes_.x() * angular_scale_;
        }
        else
        {
            twist.linear.x = -right_thumbstick_axes_.y() * linear_scale_;
            twist.linear.y = -right_thumbstick_axes_.x() * linear_scale_;
        }

        const bool deflected =
            std::abs(right_thumbstick_axes_.x()) > arm_stick_deadzone_ ||
            std::abs(right_thumbstick_axes_.y()) > arm_stick_deadzone_;
        if (deflected)
        {
            enterArmStickDrive(is_left_arm);
            publishArmTwist(is_left_arm, twist);
        }
        else if (isArmPoseMuted(is_left_arm))
        {
            exitArmStickDrive(is_left_arm, /*use_left_vr=*/false);
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

        if (msg->head_state != arms_ros2_control_msgs::msg::WbcCurrentState::HEAD_TRACKING)
        {
            stopHeadTracking();
        }
        home_joint_reference_enabled_.store(msg->home_joint_reference_enabled);

        updateArmWbcVrState(
            WbcToggleTarget::LEFT_ARM, msg->left_arm_state);
        updateArmWbcVrState(
            WbcToggleTarget::RIGHT_ARM, msg->right_arm_state);
        syncBodyTrackingArmPoseHold();
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
                    return false;
                }
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
        {            prev_calculated_left_position_ = robot_current_left_position_;
            prev_calculated_left_orientation_ =
                robot_current_left_orientation_.normalized();
        }
        else
        {            prev_calculated_right_position_ = robot_current_right_position_;
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
        refreshOpticalR0Request();
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
        publishTeleopModeState();
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
        publishTeleopModeState();
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
        publishTeleopModeState();
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
        publishTeleopModeState();
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
        if (!enabled_.load() || !pub_body_twist_ || !isBodyTrackingActive())
        {
            return;
        }

        geometry_msgs::msg::Twist twist;
        if (body_left_stick_alt_mode_.load())
        {
            twist.linear.x = -left_thumbstick_axes_.y() * linear_scale_;
            twist.angular.x = -left_thumbstick_axes_.x() * angular_scale_;
        }
        else
        {
            twist.linear.x = -left_thumbstick_axes_.y() * linear_scale_;
            twist.linear.y = -left_thumbstick_axes_.x() * linear_scale_;
        }

        if (body_right_stick_alt_mode_.load())
        {
            twist.angular.y = -right_thumbstick_axes_.y() * angular_scale_;
            twist.angular.z = -right_thumbstick_axes_.x() * angular_scale_;
        }
        else
        {
            twist.linear.z = -right_thumbstick_axes_.y() * linear_scale_;
            twist.angular.z = -right_thumbstick_axes_.x() * angular_scale_;
        }

        pub_body_twist_->publish(twist);
    }

    void VRInputHandler::processChassisAxes()
    {
        const double left_x = -left_thumbstick_axes_.x();
        const double left_y = -left_thumbstick_axes_.y();
        const double right_x = -right_thumbstick_axes_.x();
        const double right_y = -right_thumbstick_axes_.y();

        auto cmd_vel = geometry_msgs::msg::Twist();
        cmd_vel.linear.x  = left_y * chassis_linear_scale_;
        cmd_vel.linear.y  = left_x * chassis_linear_scale_;
        cmd_vel.linear.z  = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;

        auto waist_lifting = std_msgs::msg::Float64();
        auto waist_turning = std_msgs::msg::Float64();

        const bool right_grip = right_grip_active_.load();
        if (right_grip)
        {
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
            cmd_vel.angular.z = right_x * chassis_angular_scale_;
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
    }

    void VRInputHandler::resetChassisAndWaistCommands()
    {
        waist_stick_axis_.store(WaistStickAxis::NONE);
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
            publishArmTwist(true, geometry_msgs::msg::Twist{});
            publishArmTwist(false, geometry_msgs::msg::Twist{});
            resetChassisAndWaistCommands();
            if (!isBodyTrackingActive())
            {
                const bool mirror = mirror_mode_.load();
                if (left_arm_pose_muted_.load())
                {
                    exitArmStickDrive(true, /*use_left_vr=*/!mirror);
                }
                if (right_arm_pose_muted_.load())
                {
                    exitArmStickDrive(false, /*use_left_vr=*/mirror);
                }
            }
            RCLCPP_INFO(node_->get_logger(),
                        "🔘 [case 20] 切换到 CHASSIS 模式（左摇杆→底盘XY，右摇杆→腰部升降/旋转）");
        }
        else
        {
            resetChassisAndWaistCommands();
            RCLCPP_INFO(node_->get_logger(),
                        "🔘 [case 20] 退出 CHASSIS 模式，恢复末端控制");
        }
        publishTeleopModeState();
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
        if (isBodyTrackingActive())
        {
            if (isLeft)
            {
                const bool newMode = !body_left_stick_alt_mode_.load();
                body_left_stick_alt_mode_.store(newMode);
                publishTeleopModeState();
                RCLCPP_INFO(node_->get_logger(),
                            "🔘 [左握把] 短按：腰部左摇杆切换为 %s",
                            newMode ? "前进+侧摆(roll)" : "XY平移");
            }
            else
            {
                const bool newMode = !body_right_stick_alt_mode_.load();
                body_right_stick_alt_mode_.store(newMode);
                publishTeleopModeState();
                RCLCPP_INFO(node_->get_logger(),
                            "🔘 [右握把] 短按：腰部右摇杆切换为 %s",
                            newMode ? "俯仰+Yaw" : "升降+Yaw");
            }
            return;
        }

        const bool isMirror = mirror_mode_.load();
        const bool toggleRightArm = isLeft ? isMirror : !isMirror;
        auto& gripMode = toggleRightArm ? right_grip_mode_ : left_grip_mode_;
        const bool newMode = !gripMode.load();
        gripMode.store(newMode);
        publishTeleopModeState();

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

        switch (msg->data)
        {
            case 1:
            {
                break;
            }
            case 2:
                break;
            case 3:
            {
                const auto topology = controlTopology();
                if (topology == ControlTopology::UNKNOWN)
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "🔘 [case 3] 忽略左Y按钮：控制拓扑尚未确认");
                    break;
                }
                if (topology == ControlTopology::FULL_BODY &&
                    !isFullBodyYbPauseMode())
                {
                    if (isYbModeConversionPending())
                    {
                        RCLCPP_WARN(node_->get_logger(),
                                    "🔘 [case 3] 忽略左Y：case 16 转换仍在 pending");
                        break;
                    }
                    const auto target = mirror_mode_.load()
                        ? WbcToggleTarget::RIGHT_ARM
                        : WbcToggleTarget::LEFT_ARM;
                    requestWbcToggle(3, "左Y按钮", target);
                    break;
                }
                {
                    const auto arm = mirror_mode_.load()
                        ? WbcToggleTarget::RIGHT_ARM
                        : WbcToggleTarget::LEFT_ARM;
                    const bool thisArmEnablePending =
                        (arm == WbcToggleTarget::LEFT_ARM)
                            ? left_pause_after_enable_.load()
                            : right_pause_after_enable_.load();
                    if (thisArmEnablePending)
                    {
                        RCLCPP_INFO(node_->get_logger(),
                                    "🔘 [case 3] 左Y：本臂 case 16 enable→pause 中，跳过重复暂停");
                        break;
                    }
                    toggleArmVrPause(arm, "左Y按钮");
                }
                break;
            }
            case 4:
            {
                if (current_fsm_state_.load() != 3)
                {
                    RCLCPP_DEBUG(node_->get_logger(), "🔘 [右摇杆按钮] 按下 - 功能: 切换UPDATE/STORAGE模式 - 操作: 忽略（当前FSM状态不是OCS2）");
                    break;
                }

                if (target_manager_ && target_manager_->getCurrentMode() != MarkerState::CONTINUOUS)
                {
                    target_manager_->togglePublishMode();
                    RCLCPP_DEBUG(node_->get_logger(), "   ArmsTargetManager已切换到CONTINUOUS模式");
                }

                if (!is_update_mode_.load())
                {
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

                    if (!full_body_yb_pause_mode_.load())
                    {
                        bool left_was_paused = left_arm_paused_.load();
                        bool right_was_paused = right_arm_paused_.load();
                        if (left_was_paused)
                        {
                            left_arm_paused_.store(false);
                            paused_left_position_ = Eigen::Vector3d::Zero();
                            paused_left_orientation_ = Eigen::Quaterniond::Identity();
                        }
                        if (right_was_paused)
                        {
                            right_arm_paused_.store(false);
                            paused_right_position_ = Eigen::Vector3d::Zero();
                            paused_right_orientation_ = Eigen::Quaterniond::Identity();
                        }
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

                    is_update_mode_.store(true);
                    resetStaleCatchUpRamp();
                    publishTeleopModeState();
                    RCLCPP_INFO(node_->get_logger(),
                                "🔘 [右摇杆按钮] 按下 - 切换到 UPDATE（已存储基准位姿）");
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
                    optical_update_enter_pending_.store(false);
                    refreshOpticalR0Request();
                    is_update_mode_.store(false);
                    body_tracking_arms_held_.store(false);
                    left_arm_pose_muted_.store(false);
                    right_arm_pose_muted_.store(false);
                    publishArmTwist(true, geometry_msgs::msg::Twist{});
                    publishArmTwist(false, geometry_msgs::msg::Twist{});
                    stopHeadTracking();
                    publishTeleopModeState();
                    RCLCPP_INFO(node_->get_logger(), "🔘 [右摇杆按钮] 按下 - 功能: 切换UPDATE/STORAGE模式 - 操作: 切换到STORAGE模式（准备存储新的基准位姿）");
                }
                break;
            }
            case 5:
                break;
            case 6:
            {
                const auto topology = controlTopology();
                if (topology == ControlTopology::UNKNOWN)
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "🔘 [case 6] 忽略右B按钮：控制拓扑尚未确认");
                    break;
                }
                if (topology == ControlTopology::FULL_BODY &&
                    !isFullBodyYbPauseMode())
                {
                    if (isYbModeConversionPending())
                    {
                        RCLCPP_WARN(node_->get_logger(),
                                    "🔘 [case 6] 忽略右B：case 16 转换仍在 pending");
                        break;
                    }
                    const auto target = mirror_mode_.load()
                        ? WbcToggleTarget::LEFT_ARM
                        : WbcToggleTarget::RIGHT_ARM;
                    requestWbcToggle(6, "右B按钮", target);
                    break;
                }
                {
                    const auto arm = mirror_mode_.load()
                        ? WbcToggleTarget::LEFT_ARM
                        : WbcToggleTarget::RIGHT_ARM;
                    const bool thisArmEnablePending =
                        (arm == WbcToggleTarget::LEFT_ARM)
                            ? left_pause_after_enable_.load()
                            : right_pause_after_enable_.load();
                    if (thisArmEnablePending)
                    {
                        RCLCPP_INFO(node_->get_logger(),
                                    "🔘 [case 6] 右B：本臂 case 16 enable→pause 中，跳过重复暂停");
                        break;
                    }
                    toggleArmVrPause(arm, "右B按钮");
                }
                break;
            }
            case 29:
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
                    stopHeadTracking();                    paused_left_position_ = Eigen::Vector3d::Zero();
                    paused_left_orientation_ = Eigen::Quaterniond::Identity();
                    paused_right_position_ = Eigen::Vector3d::Zero();
                    paused_right_orientation_ = Eigen::Quaterniond::Identity();
                    RCLCPP_WARN(node_->get_logger(),
                                "   自动切换到STORAGE模式 - 请重新进入UPDATE模式以应用镜像处理方案切换");
                }
                publishTeleopModeState();
                break;
            }
            case 7:
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
                    stopHeadTracking();                    paused_left_position_ = Eigen::Vector3d::Zero();
                    paused_left_orientation_ = Eigen::Quaterniond::Identity();
                    paused_right_position_ = Eigen::Vector3d::Zero();
                    paused_right_orientation_ = Eigen::Quaterniond::Identity();
                    RCLCPP_WARN(node_->get_logger(),
                                "   自动切换到STORAGE模式 - 请重新进入UPDATE模式以应用镜像模式更改");
                }
                publishTeleopModeState();
                break;
            }
            case 9:
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
            case 10:
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
            case 11:
            {
                const int32_t current_state = resolvedFsmState();

                if (current_state == 2)
                {
                    sendFsmCommand(3);
                    RCLCPP_INFO(node_->get_logger(), "🔘 [右A按钮] 按下 - 功能: FSM状态前进 - 操作: HOLD → OCS2");
                }
                else if (current_state == 1)
                {
                    sendFsmCommand(2);
                    RCLCPP_INFO(node_->get_logger(), "🔘 [右A按钮] 按下 - 功能: FSM状态前进 - 操作: HOME → HOLD");
                }
                else if (current_state == 3)
                {
                    RCLCPP_WARN(node_->get_logger(), "🔘 [右A按钮] 按下 - 功能: FSM状态前进 - 操作: 失败（已在OCS2状态，无法继续前进）");
                }
                else if (current_state == 4)
                {
                    RCLCPP_WARN(node_->get_logger(), "🔘 [右A按钮] 按下 - 功能: FSM状态前进 - 操作: 失败（MOVEJ 需先回 HOLD，不能直接进 OCS2）");
                }
                break;
            }
            case 12:
            {
                const int32_t current_state = resolvedFsmState();

                if (current_state == 3)
                {
                    sendFsmCommand(2);
                    RCLCPP_INFO(node_->get_logger(), "🔘 [左X按钮] 按下 - 功能: FSM状态后退/切换 - 操作: OCS2 → HOLD");
                }
                else if (current_state == 4)
                {
                    sendFsmCommand(2);
                    RCLCPP_INFO(node_->get_logger(), "🔘 [左X按钮] 按下 - 功能: FSM状态后退/切换 - 操作: MOVEJ → HOLD");
                }
                else if (current_state == 2)
                {
                    sendFsmCommand(1);
                    RCLCPP_INFO(node_->get_logger(), "🔘 [左X按钮] 按下 - 功能: FSM状态后退/切换 - 操作: HOLD → HOME");
                }
                else if (current_state == 1)
                {
                    sendFsmCommand(100);
                    RCLCPP_INFO(node_->get_logger(), "🔘 [左X按钮] 按下 - 功能: FSM状态后退/切换 - 操作: 在HOME状态切换姿态 (HOME ↔ REST)");
                }
                break;
            }
            case 13:
            {
                markGripComboConsumed(true, false);
                toggleGripperPercentMode(true);
                break;
            }
            case 14:
            {
                markGripComboConsumed(false, true);
                toggleGripperPercentMode(false);
                break;
            }
            case 15:
            {
                break;
            }
            case 16:
                markGripComboConsumed(true, true);
                handleCase16YbModeToggle();
                break;
            case 20:
            {
                if (!enabled_.load())
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "🔘 [case 20] 忽略 - VR 控制未启用（enabled_=false）");
                    break;
                }
                toggleChassisMode();
                break;
            }
            case 21:
                markGripComboConsumed(true, false);
                requestBodyMode(
                    21, "向上", "竖直", "BODY_RELATIVE",
                    arms_ros2_control_msgs::msg::WbcCurrentState::BODY_VERTICAL);
                break;
            case 22:
                markGripComboConsumed(true, false);
                requestBodyMode(
                    22, "向下", "锁定", "BODY_LOCK",
                    arms_ros2_control_msgs::msg::WbcCurrentState::BODY_LOCKED);
                break;
            case 23:
                markGripComboConsumed(true, false);
                requestBodyMode(
                    23, "向左", "跟随", "BODY_TRACKING",
                    arms_ros2_control_msgs::msg::WbcCurrentState::BODY_TRACKING);
                break;
            case 24:
                markGripComboConsumed(true, false);
                requestBodyMode(
                    24, "向右", "自定义", "BODY_CUSTOM_LOCK",
                    arms_ros2_control_msgs::msg::WbcCurrentState::BODY_CUSTOM_LOCKED);
                break;
            case 25:
                markGripComboConsumed(false, true);
                requestWbcToggle(
                    25, "右握把+右摇杆向上", WbcToggleTarget::BIMANUAL);
                break;
            case 26:
                markGripComboConsumed(false, true);
                requestWbcToggle(
                    26, "右握把+右摇杆向下", WbcToggleTarget::BASE);
                break;
            case 27:
                markGripComboConsumed(false, true);
                break;
            case 28:
                markGripComboConsumed(false, true);
                requestWbcToggle(
                    28, "右握把+右摇杆向右", WbcToggleTarget::HOME_JOINT);
                break;
            case 0:
            default:
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
                        else
                        {
                            publishTeleopModeState();
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
        
        if (command == 0)
        {
            return;
        }

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

        std::string new_name;
        if (!arms_controller_common::FSMStateTransitionValidator::validateTransition(
                current_name, command, new_name))
        {
            RCLCPP_DEBUG(node_->get_logger(),
                         "🕹️🕶️🕹️ Ignore invalid FSM transition: %s + command=%d",
                         current_name.c_str(), command);
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
            
            if (new_state != 3)
            {
                if (is_update_mode_.load())
                {
                    is_update_mode_.store(false);
                    stopHeadTracking();
                    publishTeleopModeState();
                    RCLCPP_INFO(node_->get_logger(),
                                "🕹️🕶️🕹️ FSM状态不是OCS2，自动切换到STORAGE模式 (状态=%d)",
                                new_state);
                }
            }
        }
    }
}
