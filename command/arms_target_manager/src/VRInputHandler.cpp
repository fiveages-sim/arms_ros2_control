//
// Created for Arms ROS2 Control - VRInputHandler
//

#include "arms_target_manager/VRInputHandler.h"
#include "arms_target_manager/ArmsTargetManager.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cctype>

namespace arms_ros2_control::command
{
    // 静态常量定义
    const std::string VRInputHandler::XR_NODE_NAME = "/xr_target_node";
    const double VRInputHandler::POSITION_THRESHOLD = 0.01; // 1cm threshold for position changes
    const double VRInputHandler::ORIENTATION_THRESHOLD = 0.005; // threshold for orientation changes (quaternion angle)

    VRInputHandler::VRInputHandler(
        rclcpp::Node::SharedPtr node,
        ArmsTargetManager* targetManager,
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_left_target,
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_right_target,
        const std::vector<std::string>& handControllers,
        double vr_thumbstick_linear_scale,
        double vr_thumbstick_angular_scale,
        double vr_pose_scale)
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
          , current_fsm_state_(2)  // 默认HOLD状态
          , hand_controllers_(handControllers)
          , left_gripper_open_(false)
          , right_gripper_open_(false)
          , vr_thumbstick_linear_scale_(vr_thumbstick_linear_scale)
          , vr_thumbstick_angular_scale_(vr_thumbstick_angular_scale)
          , vr_pose_scale_(vr_pose_scale)
    {
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

        // 创建 FSM 命令发布器（使用通用工具类，自动处理command=100的特殊情况）
        auto pub_fsm_command = node_->create_publisher<std_msgs::msg::Int32>("/fsm_command", 10);
        fsm_command_publisher_ = std::make_unique<arms_controller_common::FSMCommandPublisher>(
            node_, pub_fsm_command);

        // 注意：FSM命令订阅已移除，改为在 arms_target_manager_node 中统一处理
        // 这样可以避免与 ArmsTargetManager 的订阅冲突
        // FSM状态更新现在通过 fsmCommandCallback() 方法由外部调用

        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VRInputHandler created");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Subscribed to button event topic: /xr/controller_state (Int32)");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Subscribed to thumbstick axes topic: /xr/thumbstick_axes (ThumbstickAxes)");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Thumbstick scaling: linear=%.3f, angular=%.3f", vr_thumbstick_linear_scale_,
                    vr_thumbstick_angular_scale_);
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VR pose scale: %.3f", vr_pose_scale_);
        RCLCPP_INFO(node_->get_logger(),
                    "🕹️🕶️🕹️ Grip button toggles thumbstick mode: XY-translation ↔ Z-height + Yaw-rotation");
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
            // 单臂模式：如果只有一个控制器且名称中没有left/right，假设是左控制器
            else if (hand_controllers.size() == 1 && left_gripper_controller_name_.empty())
            {
                left_gripper_controller_name_ = controller_name;
                RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Detected single-arm gripper controller (assumed left): %s", controller_name.c_str());
            }
        }
    }

    void VRInputHandler::publishGripperCommand(const std::string& controller_name, int32_t command)
    {
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
    }

    void VRInputHandler::leftGripperStateCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        // 同步左夹爪状态（从 topic 中获取）
        left_gripper_open_.store(msg->data == 1);
        RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Left gripper state synced: %s", 
                     (msg->data == 1) ? "open" : "close");
    }

    void VRInputHandler::rightGripperStateCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        // 同步右夹爪状态（从 topic 中获取）
        right_gripper_open_.store(msg->data == 1);
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
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VR control DISABLED!");
    }

    void VRInputHandler::robotLeftPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // 直接使用 Pose 版本，避免代码重复
        auto pose_msg = std::make_shared<geometry_msgs::msg::Pose>(msg->pose);
        Eigen::Matrix4d pose = poseMsgToMatrix(pose_msg);
        matrixToPosOri(pose, robot_current_left_position_, robot_current_left_orientation_);
    }

    void VRInputHandler::robotRightPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // 直接使用 Pose 版本，避免代码重复
        auto pose_msg = std::make_shared<geometry_msgs::msg::Pose>(msg->pose);
        Eigen::Matrix4d pose = poseMsgToMatrix(pose_msg);
        matrixToPosOri(pose, robot_current_right_position_, robot_current_right_orientation_);
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
        matrixToPosOri(left_ee_pose_, left_position_, left_orientation_);
        // 应用VR pose位置缩放
        left_position_ *= vr_pose_scale_;

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
                    
                    // 将右摇杆累积偏移（局部坐标系）转换到世界坐标系（与手柄移动保持一致）
                    // 摇杆偏移量是在局部坐标系下的（相对于进入UPDATE时手柄的朝向）
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

                    calculatePoseFromDifference(position_with_offset, orientation_with_yaw,
                                                vr_base_left_position_, vr_base_left_orientation_,
                                                robot_base_right_position_, robot_base_right_orientation_,
                                                calculatedPos, calculatedOri);

                    // 检查计算的pose是否发生显著变化
                    if (hasPoseChanged(calculatedPos, calculatedOri, prev_calculated_right_position_,
                                       prev_calculated_right_orientation_))
                    {
                        // 调试输出
                        RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ [Mirror] Left VR → Right Arm");
                        RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Right Calculated: [%.3f, %.3f, %.3f]",
                                     calculatedPos.x(), calculatedPos.y(), calculatedPos.z());

                        // 发布到右臂
                        publishTargetPoseDirect("right", calculatedPos, calculatedOri);

                        // 更新之前计算的右臂pose
                        prev_calculated_right_position_ = calculatedPos;
                        prev_calculated_right_orientation_ = calculatedOri;
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

                    calculatePoseFromDifference(position_with_offset, orientation_with_yaw,
                                                vr_base_left_position_, vr_base_left_orientation_,
                                                robot_base_left_position_, robot_base_left_orientation_,
                                                calculatedPos, calculatedOri);

                    // 检查计算的pose是否发生显著变化
                    if (hasPoseChanged(calculatedPos, calculatedOri, prev_calculated_left_position_,
                                       prev_calculated_left_orientation_))
                    {
                        // 调试输出
                        RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Left VR Base: [%.3f, %.3f, %.3f]",
                                     vr_base_left_position_.x(), vr_base_left_position_.y(), vr_base_left_position_.z());
                        RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Left VR Current: [%.3f, %.3f, %.3f]",
                                     left_position_.x(), left_position_.y(), left_position_.z());
                        RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Left Thumbstick Offset: [%.3f, %.3f, %.3f]",
                                     left_thumbstick_offset_.x(), left_thumbstick_offset_.y(), left_thumbstick_offset_.z());
                        RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Left Robot Base: [%.3f, %.3f, %.3f]",
                                     robot_base_left_position_.x(), robot_base_left_position_.y(),
                                     robot_base_left_position_.z());
                        RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Left Calculated: [%.3f, %.3f, %.3f]",
                                     calculatedPos.x(), calculatedPos.y(), calculatedPos.z());

                        // 发布到左臂
                        publishTargetPoseDirect("left", calculatedPos, calculatedOri);

                        // 更新之前计算的左臂pose
                        prev_calculated_left_position_ = calculatedPos;
                        prev_calculated_left_orientation_ = calculatedOri;
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
        matrixToPosOri(right_ee_pose_, right_position_, right_orientation_);
        // 应用VR pose位置缩放
        right_position_ *= vr_pose_scale_;

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

                    calculatePoseFromDifference(position_with_offset, orientation_with_yaw,
                                                vr_base_right_position_, vr_base_right_orientation_,
                                                robot_base_left_position_, robot_base_left_orientation_,
                                                calculatedPos, calculatedOri);

                    // 检查计算的pose是否发生显著变化
                    if (hasPoseChanged(calculatedPos, calculatedOri, prev_calculated_left_position_,
                                       prev_calculated_left_orientation_))
                    {
                        // 调试输出
                        RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ [Mirror] Right VR → Left Arm");
                        RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Left Calculated: [%.3f, %.3f, %.3f]",
                                     calculatedPos.x(), calculatedPos.y(), calculatedPos.z());

                        // 发布到左臂
                        publishTargetPoseDirect("left", calculatedPos, calculatedOri);

                        // 更新之前计算的左臂pose
                        prev_calculated_left_position_ = calculatedPos;
                        prev_calculated_left_orientation_ = calculatedOri;
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

                    calculatePoseFromDifference(position_with_offset, orientation_with_yaw,
                                                vr_base_right_position_, vr_base_right_orientation_,
                                                robot_base_right_position_, robot_base_right_orientation_,
                                                calculatedPos, calculatedOri);

                    // 检查计算的pose是否发生显著变化
                    if (hasPoseChanged(calculatedPos, calculatedOri, prev_calculated_right_position_,
                                       prev_calculated_right_orientation_))
                    {
                        // 调试输出
                        RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Right VR Base: [%.3f, %.3f, %.3f]",
                                     vr_base_right_position_.x(), vr_base_right_position_.y(), vr_base_right_position_.z());
                        RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Right VR Current: [%.3f, %.3f, %.3f]",
                                     right_position_.x(), right_position_.y(), right_position_.z());
                        RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Right Thumbstick Offset: [%.3f, %.3f, %.3f]",
                                     right_thumbstick_offset_.x(), right_thumbstick_offset_.y(),
                                     right_thumbstick_offset_.z());
                        RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Right Robot Base: [%.3f, %.3f, %.3f]",
                                     robot_base_right_position_.x(), robot_base_right_position_.y(),
                                     robot_base_right_position_.z());
                        RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Right Calculated: [%.3f, %.3f, %.3f]",
                                     calculatedPos.x(), calculatedPos.y(), calculatedPos.z());

                        // 发布到右臂
                        publishTargetPoseDirect("right", calculatedPos, calculatedOri);

                        // 更新之前计算的右臂pose
                        prev_calculated_right_position_ = calculatedPos;
                        prev_calculated_right_orientation_ = calculatedOri;
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

    void VRInputHandler::publishTargetPoseDirect(const std::string& armType,
                                                const Eigen::Vector3d& position,
                                                const Eigen::Quaterniond& orientation)
    {
        // 转换为geometry_msgs格式
        geometry_msgs::msg::Pose pose;
        pose.position.x = position.x();
        pose.position.y = position.y();
        pose.position.z = position.z();
        pose.orientation.w = orientation.w();
        pose.orientation.x = orientation.x();
        pose.orientation.y = orientation.y();
        pose.orientation.z = orientation.z();

        // 直接发布到对应的话题（无坐标转换）
        if (armType == "left" && pub_left_target_)
        {
            pub_left_target_->publish(pose);
            RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Published left_target: [%.3f, %.3f, %.3f]",
                        pose.position.x, pose.position.y, pose.position.z);
        }
        else if (armType == "right" && pub_right_target_)
        {
            pub_right_target_->publish(pose);
            RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Published right_target: [%.3f, %.3f, %.3f]",
                        pose.position.x, pose.position.y, pose.position.z);
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "🕹️🕶️🕹️ Invalid armType or publisher not initialized: %s", armType.c_str());
        }
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
        
        // 存储右摇杆轴值（从 angular.x/y 读取）
        right_thumbstick_axes_.x() = msg->angular.x;
        right_thumbstick_axes_.y() = msg->angular.y;
        
        // 处理左摇杆轴值
        processLeftThumbstickAxes();
        
        // 处理右摇杆轴值
        processRightThumbstickAxes();
    }
    
    void VRInputHandler::processLeftThumbstickAxes()
    {
        // 在UPDATE模式下累积摇杆输入
        if (enabled_.load() && is_update_mode_.load())
        {
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


    void VRInputHandler::processButtonEvent(const std_msgs::msg::Int32::SharedPtr msg)
    {
        // 处理按钮事件（button_event: 0=无事件, 1-6=按钮按下, 7=镜像模式切换, 9=左扳机, 10=右扳机）
        // xr_target_node 已经进行上升沿检测，这里直接响应触发事件

        switch (msg->data)
        {
            case 1:  // 左摇杆按钮
            {
                // 左摇杆按钮功能已移至 xr_target_node.py（用于切换镜像模式）
                // 镜像模式的切换和相关逻辑在 case 7 中处理
                // 这里保留空处理以保持兼容性
                break;
            }
            case 2:  // 左握把按钮
            {
                // 根据镜像模式决定切换哪个臂的模式
                if (mirror_mode_.load())
                {
                    // 镜像模式：左话题数据用于右臂
                    right_grip_mode_.store(!right_grip_mode_.load());

                    if (right_grip_mode_.load())
                    {
                        RCLCPP_INFO(node_->get_logger(), "🔘 [左握把按钮] 按下 - 功能: 切换右臂摇杆控制模式 - 操作: 切换到 Z轴+Yaw旋转模式 (Y→Z, X→Yaw) [镜像模式]");
                    }
                    else
                    {
                        RCLCPP_INFO(node_->get_logger(), "🔘 [左握把按钮] 按下 - 功能: 切换右臂摇杆控制模式 - 操作: 切换到 XY平移模式 (Y→X, X→Y) [镜像模式]");
                    }
                }
                else
                {
                    // 正常模式：左话题数据用于左臂
                    left_grip_mode_.store(!left_grip_mode_.load());

                    if (left_grip_mode_.load())
                    {
                        RCLCPP_INFO(node_->get_logger(), "🔘 [左握把按钮] 按下 - 功能: 切换左臂摇杆控制模式 - 操作: 切换到 Z轴+Yaw旋转模式 (Y→Z, X→Yaw)");
                    }
                    else
                    {
                        RCLCPP_INFO(node_->get_logger(), "🔘 [左握把按钮] 按下 - 功能: 切换左臂摇杆控制模式 - 操作: 切换到 XY平移模式 (Y→X, X→Y)");
                    }
                }
                break;
            }
            case 3:  // 左Y按钮
            {
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
                        robot_base_right_position_ = robot_current_right_position_;
                        robot_base_right_orientation_ = robot_current_right_orientation_;

                        // 重置右摇杆累积偏移
                        right_thumbstick_offset_ = Eigen::Vector3d::Zero();
                        right_thumbstick_yaw_offset_ = 0.0;

                        // 切换状态为运行
                        right_arm_paused_.store(false);
                        // 清除暂停时刻的VR位姿记录（恢复后不再使用）
                        paused_left_position_ = Eigen::Vector3d::Zero();
                        paused_left_orientation_ = Eigen::Quaterniond::Identity();

                        RCLCPP_INFO(node_->get_logger(), "🔘 [左Y按钮] 按下 - 功能: 切换右臂更新状态 - 操作: 恢复右臂更新（重置基准位姿和摇杆偏移） [镜像模式]");
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
                        robot_base_left_position_ = robot_current_left_position_;
                        robot_base_left_orientation_ = robot_current_left_orientation_;

                        // 重置左摇杆累积偏移
                        left_thumbstick_offset_ = Eigen::Vector3d::Zero();
                        left_thumbstick_yaw_offset_ = 0.0;

                        // 切换状态为运行
                        left_arm_paused_.store(false);
                        // 清除暂停时刻的VR位姿记录（恢复后不再使用）
                        paused_left_position_ = Eigen::Vector3d::Zero();
                        paused_left_orientation_ = Eigen::Quaterniond::Identity();

                        RCLCPP_INFO(node_->get_logger(), "🔘 [左Y按钮] 按下 - 功能: 切换左臂更新状态 - 操作: 恢复左臂更新（重置基准位姿和摇杆偏移）");
                        RCLCPP_DEBUG(node_->get_logger(),
                                    "   VR Base Position: [%.3f, %.3f, %.3f]",
                                    vr_base_left_position_.x(), vr_base_left_position_.y(), vr_base_left_position_.z());
                        RCLCPP_DEBUG(node_->get_logger(),
                                    "   Robot Base Position: [%.3f, %.3f, %.3f]",
                                    robot_base_left_position_.x(), robot_base_left_position_.y(), robot_base_left_position_.z());
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

                    robot_base_left_position_ = robot_current_left_position_;
                    robot_base_left_orientation_ = robot_current_left_orientation_;
                    robot_base_right_position_ = robot_current_right_position_;
                    robot_base_right_orientation_ = robot_current_right_orientation_;

                    // 重置摇杆累积偏移
                    left_thumbstick_offset_ = Eigen::Vector3d::Zero();
                    right_thumbstick_offset_ = Eigen::Vector3d::Zero();
                    left_thumbstick_yaw_offset_ = 0.0;
                    right_thumbstick_yaw_offset_ = 0.0;

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

                    is_update_mode_.store(true);
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
                }
                else
                {
                    // 切换到存储模式
                    is_update_mode_.store(false);
                    RCLCPP_INFO(node_->get_logger(), "🔘 [右摇杆按钮] 按下 - 功能: 切换UPDATE/STORAGE模式 - 操作: 切换到STORAGE模式（准备存储新的基准位姿）");
                }
                break;
            }
            case 5:  // 右握把按钮
            {
                // 根据镜像模式决定切换哪个臂的模式
                if (mirror_mode_.load())
                {
                    // 镜像模式：右话题数据用于左臂
                    left_grip_mode_.store(!left_grip_mode_.load());

                    if (left_grip_mode_.load())
                    {
                        RCLCPP_INFO(node_->get_logger(), "🔘 [右握把按钮] 按下 - 功能: 切换左臂摇杆控制模式 - 操作: 切换到 Z轴+Yaw旋转模式 (Y→Z, X→Yaw) [镜像模式]");
                    }
                    else
                    {
                        RCLCPP_INFO(node_->get_logger(), "🔘 [右握把按钮] 按下 - 功能: 切换左臂摇杆控制模式 - 操作: 切换到 XY平移模式 (Y→X, X→Y) [镜像模式]");
                    }
                }
                else
                {
                    // 正常模式：右话题数据用于右臂
                    right_grip_mode_.store(!right_grip_mode_.load());

                    if (right_grip_mode_.load())
                    {
                        RCLCPP_INFO(node_->get_logger(), "🔘 [右握把按钮] 按下 - 功能: 切换右臂摇杆控制模式 - 操作: 切换到 Z轴+Yaw旋转模式 (Y→Z, X→Yaw)");
                    }
                    else
                    {
                        RCLCPP_INFO(node_->get_logger(), "🔘 [右握把按钮] 按下 - 功能: 切换右臂摇杆控制模式 - 操作: 切换到 XY平移模式 (Y→X, X→Y)");
                    }
                }
                break;
            }
            case 6:  // 右B按钮
            {
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
                        robot_base_left_position_ = robot_current_left_position_;
                        robot_base_left_orientation_ = robot_current_left_orientation_;

                        // 重置左摇杆累积偏移
                        left_thumbstick_offset_ = Eigen::Vector3d::Zero();
                        left_thumbstick_yaw_offset_ = 0.0;

                        // 切换状态为运行
                        left_arm_paused_.store(false);
                        // 清除暂停时刻的VR位姿记录（恢复后不再使用）
                        paused_right_position_ = Eigen::Vector3d::Zero();
                        paused_right_orientation_ = Eigen::Quaterniond::Identity();

                        RCLCPP_INFO(node_->get_logger(), "🔘 [右B按钮] 按下 - 功能: 切换左臂更新状态 - 操作: 恢复左臂更新（重置基准位姿和摇杆偏移） [镜像模式]");
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
                        robot_base_right_position_ = robot_current_right_position_;
                        robot_base_right_orientation_ = robot_current_right_orientation_;

                        // 重置右摇杆累积偏移
                        right_thumbstick_offset_ = Eigen::Vector3d::Zero();
                        right_thumbstick_yaw_offset_ = 0.0;

                        // 切换状态为运行
                        right_arm_paused_.store(false);
                        // 清除暂停时刻的VR位姿记录（恢复后不再使用）
                        paused_right_position_ = Eigen::Vector3d::Zero();
                        paused_right_orientation_ = Eigen::Quaterniond::Identity();

                        RCLCPP_INFO(node_->get_logger(), "🔘 [右B按钮] 按下 - 功能: 切换右臂更新状态 - 操作: 恢复右臂更新（重置基准位姿和摇杆偏移）");
                        RCLCPP_DEBUG(node_->get_logger(),
                                    "   VR Base Position: [%.3f, %.3f, %.3f]",
                                    vr_base_right_position_.x(), vr_base_right_position_.y(), vr_base_right_position_.z());
                        RCLCPP_DEBUG(node_->get_logger(),
                                    "   Robot Base Position: [%.3f, %.3f, %.3f]",
                                    robot_base_right_position_.x(), robot_base_right_position_.y(), robot_base_right_position_.z());
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
                // 根据镜像模式决定控制哪个臂
                std::string target_controller_name;
                bool is_target_left_arm; // true for left arm, false for right arm

                if (mirror_mode_.load())
                {
                    // 镜像模式：左扳机控制右臂
                    target_controller_name = right_gripper_controller_name_;
                    is_target_left_arm = false;
                }
                else
                {
                    // 正常模式：左扳机控制左臂
                    target_controller_name = left_gripper_controller_name_;
                    is_target_left_arm = true;
                }

                if (target_controller_name.empty())
                {
                    RCLCPP_WARN(node_->get_logger(), "🔘 [左扳机按钮] 按下 - 功能: 控制夹爪开合 - 操作: 失败（未检测到%s臂控制器）",
                                is_target_left_arm ? "左" : "右");
                    break;
                }

                // 获取当前夹爪状态并切换
                bool current_gripper_open = is_target_left_arm ? left_gripper_open_.load() : right_gripper_open_.load();
                int32_t command = current_gripper_open ? 0 : 1; // 0=close, 1=open

                publishGripperCommand(target_controller_name, command);

                // 状态会通过订阅器回调自动更新，无需手动更新

                RCLCPP_INFO(node_->get_logger(), "🔘 [左扳机按钮] 按下 - 功能: 控制夹爪开合 - 操作: %s夹爪已%s%s",
                            is_target_left_arm ? "左" : "右", (command == 1) ? "打开" : "关闭",
                            mirror_mode_.load() ? " [镜像模式]" : "");
                break;
            }
            case 10: // 右扳机按钮
            {
                // 根据镜像模式决定控制哪个臂
                std::string target_controller_name;
                bool is_target_left_arm; // true for left arm, false for right arm

                if (mirror_mode_.load())
                {
                    // 镜像模式：右扳机控制左臂
                    target_controller_name = left_gripper_controller_name_;
                    is_target_left_arm = true;
                }
                else
                {
                    // 正常模式：右扳机控制右臂
                    target_controller_name = right_gripper_controller_name_;
                    is_target_left_arm = false;
                }

                if (target_controller_name.empty())
                {
                    RCLCPP_WARN(node_->get_logger(), "🔘 [右扳机按钮] 按下 - 功能: 控制夹爪开合 - 操作: 失败（未检测到%s臂控制器）",
                                is_target_left_arm ? "左" : "右");
                    break;
                }

                // 获取当前夹爪状态并切换
                bool current_gripper_open = is_target_left_arm ? left_gripper_open_.load() : right_gripper_open_.load();
                int32_t command = current_gripper_open ? 0 : 1; // 0=close, 1=open

                publishGripperCommand(target_controller_name, command);

                // 状态会通过订阅器回调自动更新，无需手动更新

                RCLCPP_INFO(node_->get_logger(), "🔘 [右扳机按钮] 按下 - 功能: 控制夹爪开合 - 操作: %s夹爪已%s%s",
                            is_target_left_arm ? "左" : "右", (command == 1) ? "打开" : "关闭",
                            mirror_mode_.load() ? " [镜像模式]" : "");
                break;
            }
            case 11: // 右A按钮（FSM状态控制）
            {
                // 根据当前FSM状态发送相应的转换命令
                int32_t current_state = current_fsm_state_.load();
                
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
                    // OCS2无法继续前进
                    RCLCPP_WARN(node_->get_logger(), "🔘 [右A按钮] 按下 - 功能: FSM状态前进 - 操作: 失败（已在OCS2状态，无法继续前进）");
                }
                break;
            }
            case 12: // 左X按钮（FSM状态控制）
            {
                // 根据当前FSM状态发送相应的转换命令
                int32_t current_state = current_fsm_state_.load();
                
                if (current_state == 3)  // OCS2
                {
                    // OCS2 → HOLD
                    sendFsmCommand(2);
                    RCLCPP_INFO(node_->get_logger(), "🔘 [左X按钮] 按下 - 功能: FSM状态后退/切换 - 操作: OCS2 → HOLD");
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
            case 0:  // 无事件
            default:
                // 无按钮事件
                break;
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

        // 更新FSM状态
        int32_t old_state = current_fsm_state_.load();
        
        // 根据command推断新状态
        int32_t new_state = old_state;
        if (command == 1)
        {
            new_state = 1; // HOME
        }
        else if (command == 2)
        {
            new_state = 2; // HOLD
        }
        else if (command == 3)
        {
            new_state = 3; // OCS2
        }
        else if (command == 100)
        {
            // REST姿态切换命令（不改变状态，只切换姿态）
            // 状态保持为HOME，不需要更新状态
            return;
        }

        // 更新状态
        if (new_state != old_state)
        {
            current_fsm_state_.store(new_state);
            
            // 如果当前状态不是OCS2，自动切换到存储模式
            if (new_state != 3)  // 3 = OCS2
            {
                if (is_update_mode_.load())
                {
                    is_update_mode_.store(false);
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