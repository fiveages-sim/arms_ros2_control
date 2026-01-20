//
// Created for Arms ROS2 Control - VRInputHandler
//

#include "arms_target_manager/VRInputHandler.h"
#include "arms_target_manager/ArmsTargetManager.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <arms_ros2_control_msgs/msg/vr_controller_state.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace arms_ros2_control::command
{
    // 静态常量定义
    const std::string VRInputHandler::XR_NODE_NAME = "/xr_target_node";
    const double VRInputHandler::POSITION_THRESHOLD = 0.01; // 1cm threshold for position changes
    const double VRInputHandler::ORIENTATION_THRESHOLD = 0.005; // threshold for orientation changes (quaternion angle)
    const double VRInputHandler::LINEAR_SCALE = 0.005; // 与joystick的linear_scale一致
    const double VRInputHandler::ANGULAR_SCALE = 0.05; // 与joystick的angular_scale一致

    VRInputHandler::VRInputHandler(
        rclcpp::Node::SharedPtr node,
        ArmsTargetManager* targetManager,
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_left_target,
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_right_target,
        double updateRate)
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
          , last_update_time_(node_->now())
          , update_rate_(updateRate)
          , current_position_(0.0, 0.0, 1.0)
          , current_orientation_(1.0, 0.0, 0.0, 0.0)
    {

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

        // 创建统一控制器状态订阅器
        auto controllerStateCallback = [this](const arms_ros2_control_msgs::msg::VRControllerState::SharedPtr msg)
        {
            this->processControllerState(msg);
        };
        sub_controller_state_ = node_->create_subscription<arms_ros2_control_msgs::msg::VRControllerState>(
            "/xr/controller_state", 10, controllerStateCallback);

        // 注意：FSM命令订阅已移除，改为在 arms_target_manager_node 中统一处理
        // 这样可以避免与 ArmsTargetManager 的订阅冲突
        // FSM状态更新现在通过 fsmCommandCallback() 方法由外部调用

        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VRInputHandler created");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Subscribed to unified controller state topic: /xr/controller_state");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Thumbstick scaling: linear=%.3f, angular=%.3f", LINEAR_SCALE,
                    ANGULAR_SCALE);
        RCLCPP_INFO(node_->get_logger(),
                    "🕹️🕶️🕹️ Grip button toggles thumbstick mode: XY-translation ↔ Z-height + Yaw-rotation");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VR control is DISABLED by default.");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Right thumbstick toggles between STORAGE and UPDATE modes.");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ STORAGE mode: Store VR and robot base poses (no marker update)");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ UPDATE mode: Calculate pose differences and update markers");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ MIRROR mode: Synced from xr_target_node (left thumbstick toggles)");
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

    void VRInputHandler::rightThumbstickCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        // 只在OCS2状态下执行（状态值为3）
        if (current_fsm_state_.load() != 3)
        {
            return;
        }

        // xr_target_node 已经进行上升沿检测，这里直接响应触发事件
        if (msg->data)
        {
            // 确保切换到连续发布模式（更稳健，防止用户手动切换回单次模式）
            if (target_manager_ && target_manager_->getCurrentMode() != MarkerState::CONTINUOUS)
            {
                target_manager_->togglePublishMode();
                RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ ArmsTargetManager switched to CONTINUOUS mode for VR control");
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
                    RCLCPP_INFO(node_->get_logger(), "🟡 左臂暂停状态已重置 - 恢复更新！");
                }
                if (right_was_paused)
                {
                    right_arm_paused_.store(false);
                    RCLCPP_INFO(node_->get_logger(), "🔵 右臂暂停状态已重置 - 恢复更新！");
                }

                is_update_mode_.store(true);
                RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Switched to UPDATE mode - Base poses stored!");
                RCLCPP_INFO(node_->get_logger(),
                            "🕹️🕶️🕹️ VR Base Positions: Left [%.3f, %.3f, %.3f], Right [%.3f, %.3f, %.3f]",
                            vr_base_left_position_.x(), vr_base_left_position_.y(), vr_base_left_position_.z(),
                            vr_base_right_position_.x(), vr_base_right_position_.y(), vr_base_right_position_.z());
                RCLCPP_INFO(node_->get_logger(),
                            "🕹️🕶️🕹️ Robot Base Positions: Left [%.3f, %.3f, %.3f], Right [%.3f, %.3f, %.3f]",
                            robot_base_left_position_.x(), robot_base_left_position_.y(), robot_base_left_position_.z(),
                            robot_base_right_position_.x(), robot_base_right_position_.y(),
                            robot_base_right_position_.z());
                RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Thumbstick offsets reset!");
            }
            else
            {
                // 切换到存储模式
                is_update_mode_.store(false);
                RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Switched to STORAGE mode - Ready to store new base poses!");
            }
        }
    }

    void VRInputHandler::leftThumbstickCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        // 左摇杆按钮功能已移至 xr_target_node.py（用于切换镜像模式）
        // 镜像模式的切换和相关逻辑在 processControllerState 中通过 mirror 字段同步处理
        // 这里保留空回调函数以保持兼容性
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
        // 检查更新频率
        auto currentTime = node_->now();
        double timeSinceLastUpdate = (currentTime - last_update_time_).seconds();
        double updateInterval = 1.0 / update_rate_;

        if (timeSinceLastUpdate < updateInterval)
        {
            return;
        }
        last_update_time_ = currentTime;

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

        if (enabled_.load())
        {
            // 根据镜像模式决定使用哪个臂的状态和参数
            bool is_mirror = mirror_mode_.load();
            bool arm_paused = is_mirror ? right_arm_paused_.load() : left_arm_paused_.load();

            // 检查目标臂是否暂停更新
            if (arm_paused)
            {
                // 暂停更新：不计算和发布目标位姿，直接返回
                return;
            }

            if (is_update_mode_.load())
            {
                // 更新模式：基于差值计算pose并更新marker
                Eigen::Vector3d calculatedPos;
                Eigen::Quaterniond calculatedOri;

                if (is_mirror)
                {
                    // 镜像模式：左话题数据用于右臂
                    // 应用右摇杆累积偏移到VR当前位置
                    Eigen::Vector3d position_with_offset = left_position_ + right_thumbstick_offset_;

                    // 应用右摇杆累积Yaw旋转到VR当前姿态
                    Eigen::Quaterniond orientation_with_yaw = left_orientation_;
                    if (std::abs(right_thumbstick_yaw_offset_) > 0.001)
                    {
                        Eigen::AngleAxisd yawRotation(right_thumbstick_yaw_offset_, Eigen::Vector3d::UnitZ());
                        orientation_with_yaw = Eigen::Quaterniond(yawRotation) * left_orientation_;
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
                    // 应用左摇杆累积偏移到VR当前位置
                    Eigen::Vector3d position_with_offset = left_position_ + left_thumbstick_offset_;

                    // 应用左摇杆累积Yaw旋转到VR当前姿态
                    Eigen::Quaterniond orientation_with_yaw = left_orientation_;
                    if (std::abs(left_thumbstick_yaw_offset_) > 0.001)
                    {
                        Eigen::AngleAxisd yawRotation(left_thumbstick_yaw_offset_, Eigen::Vector3d::UnitZ());
                        orientation_with_yaw = Eigen::Quaterniond(yawRotation) * left_orientation_;
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

        if (enabled_.load())
        {
            // 根据镜像模式决定使用哪个臂的状态和参数
            bool is_mirror = mirror_mode_.load();
            bool arm_paused = is_mirror ? left_arm_paused_.load() : right_arm_paused_.load();

            // 检查目标臂是否暂停更新
            if (arm_paused)
            {
                // 暂停更新：不计算和发布目标位姿，直接返回
                return;
            }

            if (is_update_mode_.load())
            {
                // 更新模式：基于差值计算pose并更新marker
                Eigen::Vector3d calculatedPos;
                Eigen::Quaterniond calculatedOri;

                if (is_mirror)
                {
                    // 镜像模式：右话题数据用于左臂
                    // 应用左摇杆累积偏移到VR当前位置
                    Eigen::Vector3d position_with_offset = right_position_ + left_thumbstick_offset_;

                    // 应用左摇杆累积Yaw旋转到VR当前姿态
                    Eigen::Quaterniond orientation_with_yaw = right_orientation_;
                    if (std::abs(left_thumbstick_yaw_offset_) > 0.001)
                    {
                        Eigen::AngleAxisd yawRotation(left_thumbstick_yaw_offset_, Eigen::Vector3d::UnitZ());
                        orientation_with_yaw = Eigen::Quaterniond(yawRotation) * right_orientation_;
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
                    // 应用右摇杆累积偏移到VR当前位置
                    Eigen::Vector3d position_with_offset = right_position_ + right_thumbstick_offset_;

                    // 应用右摇杆累积Yaw旋转到VR当前姿态
                    Eigen::Quaterniond orientation_with_yaw = right_orientation_;
                    if (std::abs(right_thumbstick_yaw_offset_) > 0.001)
                    {
                        Eigen::AngleAxisd yawRotation(right_thumbstick_yaw_offset_, Eigen::Vector3d::UnitZ());
                        orientation_with_yaw = Eigen::Quaterniond(yawRotation) * right_orientation_;
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

    void VRInputHandler::updateMarkerPose(const std::string& armType,
                                          const Eigen::Vector3d& position,
                                          const Eigen::Quaterniond& orientation)
    {
        if (target_manager_)
        {
            // 转换为geometry_msgs格式
            geometry_msgs::msg::Point pos;
            pos.x = position.x();
            pos.y = position.y();
            pos.z = position.z();

            geometry_msgs::msg::Quaternion ori;
            ori.w = orientation.w();
            ori.x = orientation.x();
            ori.y = orientation.y();
            ori.z = orientation.z();

            // 使用ArmsTargetManager设置marker位置
            target_manager_->setMarkerPose(armType, pos, ori);

            // 输出调试信息
            RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Updated %s arm marker position: [%.3f, %.3f, %.3f]",
                        armType.c_str(), position.x(), position.y(), position.z());
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

    void VRInputHandler::leftGripCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        // xr_target_node 已经进行上升沿检测，这里直接响应触发事件
        if (msg->data)
        {
            // 根据镜像模式决定切换哪个臂的模式
            if (mirror_mode_.load())
            {
                // 镜像模式：左话题数据用于右臂
                right_grip_mode_.store(!right_grip_mode_.load());

                if (right_grip_mode_.load())
                {
                    RCLCPP_INFO(node_->get_logger(), "🟢 [Mirror] Left grip → Right arm mode: Z-height + Yaw rotation");
                }
                else
                {
                    RCLCPP_INFO(node_->get_logger(), "🟢 [Mirror] Left grip → Right arm mode: XY translation");
                }
            }
            else
            {
                // 正常模式：左话题数据用于左臂
                left_grip_mode_.store(!left_grip_mode_.load());

                if (left_grip_mode_.load())
                {
                    RCLCPP_INFO(node_->get_logger(), "🟢 Left grip mode: Z-height + Yaw rotation (Y→Z, X→Yaw)");
                }
                else
                {
                    RCLCPP_INFO(node_->get_logger(), "🟢 Left grip mode: XY translation (Y→X, X→Y)");
                }
            }
        }
    }

    void VRInputHandler::rightGripCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        // xr_target_node 已经进行上升沿检测，这里直接响应触发事件
        if (msg->data)
        {
            // 根据镜像模式决定切换哪个臂的模式
            if (mirror_mode_.load())
            {
                // 镜像模式：右话题数据用于左臂
                left_grip_mode_.store(!left_grip_mode_.load());

                if (left_grip_mode_.load())
                {
                    RCLCPP_INFO(node_->get_logger(), "🟢 [Mirror] Right grip → Left arm mode: Z-height + Yaw rotation");
                }
                else
                {
                    RCLCPP_INFO(node_->get_logger(), "🟢 [Mirror] Right grip → Left arm mode: XY translation");
                }
            }
            else
            {
                // 正常模式：右话题数据用于右臂
                right_grip_mode_.store(!right_grip_mode_.load());

                if (right_grip_mode_.load())
                {
                    RCLCPP_INFO(node_->get_logger(), "🟢 Right grip mode: Z-height + Yaw rotation (Y→Z, X→Yaw)");
                }
                else
                {
                    RCLCPP_INFO(node_->get_logger(), "🟢 Right grip mode: XY translation (Y→X, X→Y)");
                }
            }
        }
    }

    void VRInputHandler::leftYButtonCallback(std_msgs::msg::Bool::SharedPtr msg)
    {
        // 只在UPDATE模式下启用（右手摇杆按钮按下后进入UPDATE模式）
        if (!is_update_mode_.load())
        {
            return;
        }

        // xr_target_node 已经进行上升沿检测，这里直接响应触发事件
        if (msg->data)
        {
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

                    RCLCPP_INFO(node_->get_logger(), "🟡 [Mirror] 左Y按键 → 右臂更新已恢复！");
                }
                else
                {
                    // 当前是运行状态，执行暂停操作
                    right_arm_paused_.store(true);
                    RCLCPP_INFO(node_->get_logger(), "🟡 [Mirror] 左Y按键 → 右臂更新已暂停！");
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

                    RCLCPP_INFO(node_->get_logger(), "🟡 左Y按键按下 - 左臂更新已恢复！");
                    RCLCPP_INFO(node_->get_logger(),
                                "🟡 VR Base Position: [%.3f, %.3f, %.3f]",
                                vr_base_left_position_.x(), vr_base_left_position_.y(), vr_base_left_position_.z());
                    RCLCPP_INFO(node_->get_logger(),
                                "🟡 Robot Base Position: [%.3f, %.3f, %.3f]",
                                robot_base_left_position_.x(), robot_base_left_position_.y(), robot_base_left_position_.z());
                    RCLCPP_INFO(node_->get_logger(), "🟡 左摇杆偏移已重置！");
                }
                else
                {
                    // 当前是运行状态，执行暂停操作
                    left_arm_paused_.store(true);
                    RCLCPP_INFO(node_->get_logger(), "🟡 左Y按键按下 - 左臂更新已暂停！");
                }
            }
        }
    }

    void VRInputHandler::rightBButtonCallback(std_msgs::msg::Bool::SharedPtr msg)
    {
        // 只在UPDATE模式下启用（右手摇杆按钮按下后进入UPDATE模式）
        if (!is_update_mode_.load())
        {
            return;
        }

        // xr_target_node 已经进行上升沿检测，这里直接响应触发事件
        if (msg->data)
        {
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

                    RCLCPP_INFO(node_->get_logger(), "🔵 [Mirror] 右B按键 → 左臂更新已恢复！");
                }
                else
                {
                    // 当前是运行状态，执行暂停操作
                    left_arm_paused_.store(true);
                    RCLCPP_INFO(node_->get_logger(), "🔵 [Mirror] 右B按键 → 左臂更新已暂停！");
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

                    RCLCPP_INFO(node_->get_logger(), "🔵 右B按键按下 - 右臂更新已恢复！");
                    RCLCPP_INFO(node_->get_logger(),
                                "🔵 VR Base Position: [%.3f, %.3f, %.3f]",
                                vr_base_right_position_.x(), vr_base_right_position_.y(), vr_base_right_position_.z());
                    RCLCPP_INFO(node_->get_logger(),
                                "🔵 Robot Base Position: [%.3f, %.3f, %.3f]",
                                robot_base_right_position_.x(), robot_base_right_position_.y(), robot_base_right_position_.z());
                    RCLCPP_INFO(node_->get_logger(), "🔵 右摇杆偏移已重置！");
                }
                else
                {
                    // 当前是运行状态，执行暂停操作
                    right_arm_paused_.store(true);
                    RCLCPP_INFO(node_->get_logger(), "🔵 右B按键按下 - 右臂更新已暂停！");
                }
            }
        }
    }

    void VRInputHandler::leftThumbstickAxesCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        // 存储左摇杆轴值
        left_thumbstick_axes_.x() = msg->x;
        left_thumbstick_axes_.y() = msg->y;

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
                    double delta_z = left_thumbstick_axes_.y() * LINEAR_SCALE;
                    double delta_yaw = left_thumbstick_axes_.x() * ANGULAR_SCALE;

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
                    double delta_x = left_thumbstick_axes_.y() * LINEAR_SCALE;
                    double delta_y = left_thumbstick_axes_.x() * LINEAR_SCALE;

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
                    double delta_z = left_thumbstick_axes_.y() * LINEAR_SCALE;
                    double delta_yaw = left_thumbstick_axes_.x() * ANGULAR_SCALE;

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
                    double delta_x = left_thumbstick_axes_.y() * LINEAR_SCALE;
                    double delta_y = left_thumbstick_axes_.x() * LINEAR_SCALE;

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

    void VRInputHandler::rightThumbstickAxesCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        // 存储右摇杆轴值
        right_thumbstick_axes_.x() = msg->x;
        right_thumbstick_axes_.y() = msg->y;

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
                    double delta_z = right_thumbstick_axes_.y() * LINEAR_SCALE;
                    double delta_yaw = right_thumbstick_axes_.x() * ANGULAR_SCALE;

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
                    double delta_x = right_thumbstick_axes_.y() * LINEAR_SCALE;
                    double delta_y = right_thumbstick_axes_.x() * LINEAR_SCALE;

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
                    double delta_z = right_thumbstick_axes_.y() * LINEAR_SCALE;
                    double delta_yaw = right_thumbstick_axes_.x() * ANGULAR_SCALE;

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
                    double delta_x = right_thumbstick_axes_.y() * LINEAR_SCALE;
                    double delta_y = right_thumbstick_axes_.x() * LINEAR_SCALE;

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


    void VRInputHandler::processControllerState(const arms_ros2_control_msgs::msg::VRControllerState::SharedPtr msg)
    {
        // 处理所有按钮事件（触发事件，已经过上升沿检测）
        // 创建Bool消息用于调用现有回调函数

        // 更新镜像模式状态（从 xr_target_node 同步）
        bool old_mirror_mode = mirror_mode_.load();
        mirror_mode_.store(msg->mirror);

        // 如果镜像模式发生变化，记录日志并自动切换到 STORAGE 模式
        if (old_mirror_mode != msg->mirror)
        {
            if (msg->mirror)
            {
                RCLCPP_INFO(node_->get_logger(),
                            "🕹️🕶️🕹️ MIRROR mode ENABLED - Left controller controls right arm, right controller controls left arm");
            }
            else
            {
                RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ MIRROR mode DISABLED - Normal control restored");
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
                RCLCPP_WARN(node_->get_logger(),
                            "🕹️🕶️🕹️ Automatically switched to STORAGE mode - Please re-enter UPDATE mode to apply mirror changes");
                RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Thumbstick offsets reset!");
            }
        }

        // 左摇杆按钮
        if (msg->left_thumbstick_button)
        {
            auto thumbstick_msg = std::make_shared<std_msgs::msg::Bool>();
            thumbstick_msg->data = true;
            leftThumbstickCallback(thumbstick_msg);
        }

        // 右摇杆按钮
        if (msg->right_thumbstick_button)
        {
            auto thumbstick_msg = std::make_shared<std_msgs::msg::Bool>();
            thumbstick_msg->data = true;
            rightThumbstickCallback(thumbstick_msg);
        }

        // 左握把按钮
        if (msg->left_grip_button)
        {
            auto grip_msg = std::make_shared<std_msgs::msg::Bool>();
            grip_msg->data = true;
            leftGripCallback(grip_msg);
        }

        // 右握把按钮
        if (msg->right_grip_button)
        {
            auto grip_msg = std::make_shared<std_msgs::msg::Bool>();
            grip_msg->data = true;
            rightGripCallback(grip_msg);
        }

        // 左Y按钮
        if (msg->left_y_button)
        {
            auto y_button_msg = std::make_shared<std_msgs::msg::Bool>();
            y_button_msg->data = true;
            leftYButtonCallback(y_button_msg);
        }

        // 右B按钮
        if (msg->right_b_button)
        {
            auto b_button_msg = std::make_shared<std_msgs::msg::Bool>();
            b_button_msg->data = true;
            rightBButtonCallback(b_button_msg);
        }

        // 处理摇杆轴值
        auto left_axes_msg = std::make_shared<geometry_msgs::msg::Point>();
        left_axes_msg->x = msg->left_thumbstick_x;
        left_axes_msg->y = msg->left_thumbstick_y;
        left_axes_msg->z = 0.0;
        leftThumbstickAxesCallback(left_axes_msg);

        auto right_axes_msg = std::make_shared<geometry_msgs::msg::Point>();
        right_axes_msg->x = msg->right_thumbstick_x;
        right_axes_msg->y = msg->right_thumbstick_y;
        right_axes_msg->z = 0.0;
        rightThumbstickAxesCallback(right_axes_msg);
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
