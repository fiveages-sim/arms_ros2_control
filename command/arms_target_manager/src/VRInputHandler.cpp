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
        double updateRate)
        : node_(std::move(node))
          , target_manager_(targetManager)
          , enabled_(false)
          , is_update_mode_(false)
          , last_thumbstick_state_(false)
          , mirror_mode_(false)
          , last_left_thumbstick_state_(false)
          , last_left_grip_state_(false)
          , last_right_grip_state_(false)
          , last_left_y_button_state_(false)
          , last_right_b_button_state_(false)
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
        // 创建目标位姿发布器（直接发布到left_target/right_target）
        pub_left_target_ = node_->create_publisher<geometry_msgs::msg::Pose>("left_target", 10);
        pub_right_target_ = node_->create_publisher<geometry_msgs::msg::Pose>("right_target", 10);

        // 创建VR订阅器
        auto vrLeftCallback = [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            this->vrLeftCallback(msg);
        };
        sub_left_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "xr_left_ee_pose", 10, vrLeftCallback);

        auto vrRightCallback = [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            this->vrRightCallback(msg);
        };
        sub_right_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "xr_right_ee_pose", 10, vrRightCallback);

        // 创建右摇杆订阅器
        auto thumbstickCallback = [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
            this->rightThumbstickCallback(msg);
        };
        sub_right_thumbstick_ = node_->create_subscription<std_msgs::msg::Bool>(
            "xr_right_thumbstick", 10, thumbstickCallback);

        // 创建左摇杆订阅器
        auto leftThumbstickCallback = [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
            this->leftThumbstickCallback(msg);
        };
        sub_left_thumbstick_ = node_->create_subscription<std_msgs::msg::Bool>(
            "xr_left_thumbstick", 10, leftThumbstickCallback);

        // 创建机器人pose订阅器
        auto robotLeftCallback = [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            this->robotLeftPoseCallback(msg);
        };
        sub_robot_left_pose_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "left_current_pose", 10, robotLeftCallback);

        auto robotRightCallback = [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            this->robotRightPoseCallback(msg);
        };
        sub_robot_right_pose_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "right_current_pose", 10, robotRightCallback);

        // 创建摇杆轴值订阅器
        auto leftThumbstickAxesCallback = [this](const geometry_msgs::msg::Point::SharedPtr msg)
        {
            this->leftThumbstickAxesCallback(msg);
        };
        sub_left_thumbstick_axes_ = node_->create_subscription<geometry_msgs::msg::Point>(
            "xr_left_thumbstick_axes", 10, leftThumbstickAxesCallback);

        auto rightThumbstickAxesCallback = [this](const geometry_msgs::msg::Point::SharedPtr msg)
        {
            this->rightThumbstickAxesCallback(msg);
        };
        sub_right_thumbstick_axes_ = node_->create_subscription<geometry_msgs::msg::Point>(
            "xr_right_thumbstick_axes", 10, rightThumbstickAxesCallback);

        // 创建握把按钮订阅器
        auto leftGripCallback = [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
            this->leftGripCallback(msg);
        };
        sub_left_grip_ = node_->create_subscription<std_msgs::msg::Bool>(
            "xr_left_grip", 10, leftGripCallback);

        auto rightGripCallback = [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
            this->rightGripCallback(msg);
        };
        sub_right_grip_ = node_->create_subscription<std_msgs::msg::Bool>(
            "xr_right_grip", 10, rightGripCallback);

        // 创建Y按键订阅器
        auto leftYButtonCallback = [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
            this->leftYButtonCallback(msg);
        };
        sub_left_y_button_ = node_->create_subscription<std_msgs::msg::Bool>(
            "xr_left_y_button", 10, leftYButtonCallback);

        auto rightBButtonCallback = [this](const std_msgs::msg::Bool::SharedPtr msg)
        {
            this->rightBButtonCallback(msg);
        };
        sub_right_b_button_ = node_->create_subscription<std_msgs::msg::Bool>(
            "xr_right_b_button", 10, rightBButtonCallback);

        // 创建FSM命令订阅器（用于跟踪FSM状态）
        auto fsmCommandCallback = [this](const std_msgs::msg::Int32::SharedPtr msg)
        {
            this->fsmCommandCallback(msg);
        };
        sub_fsm_command_ = node_->create_subscription<std_msgs::msg::Int32>(
            "/fsm_command", 10, fsmCommandCallback);
        

        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VRInputHandler created");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Thumbstick scaling: linear=%.3f, angular=%.3f", LINEAR_SCALE,
                    ANGULAR_SCALE);
        RCLCPP_INFO(node_->get_logger(),
                    "🕹️🕶️🕹️ Grip button toggles thumbstick mode: XY-translation ↔ Z-height + Yaw-rotation");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VR control is DISABLED by default.");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Right thumbstick toggles between STORAGE and UPDATE modes.");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ STORAGE mode: Store VR and robot base poses (no marker update)");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ UPDATE mode: Calculate pose differences and update markers");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Left thumbstick toggles MIRROR mode (face-to-face control).");
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
        bool currentThumbstickState = msg->data;
        bool lastState = last_thumbstick_state_.load();

        // 检测上升沿（按钮按下）
        if (currentThumbstickState && !lastState)
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

        last_thumbstick_state_.store(currentThumbstickState);
    }

    void VRInputHandler::leftThumbstickCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        bool currentThumbstickState = msg->data;
        bool lastState = last_left_thumbstick_state_.load();

        // 检测上升沿（按钮按下）
        if (currentThumbstickState && !lastState)
        {
            // 切换镜像模式
            mirror_mode_.store(!mirror_mode_.load());

            if (mirror_mode_.load())
            {
                RCLCPP_INFO(node_->get_logger(),
                            "🕹️🕶️🕹️ MIRROR mode ENABLED - Left controller controls right arm, right controller controls left arm")
                ;
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
                            "🕹️🕶️🕹️ Automatically switched to STORAGE mode - Please re-enter UPDATE mode to apply mirror changes")
                ;
                RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Thumbstick offsets reset!");
            }
        }

        last_left_thumbstick_state_.store(currentThumbstickState);
    }

    void VRInputHandler::robotLeftPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        Eigen::Matrix4d pose = poseMsgToMatrix(msg);
        matrixToPosOri(pose, robot_current_left_position_, robot_current_left_orientation_);
    }

    void VRInputHandler::robotRightPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        Eigen::Matrix4d pose = poseMsgToMatrix(msg);
        matrixToPosOri(pose, robot_current_right_position_, robot_current_right_orientation_);
    }

    void VRInputHandler::vrLeftCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
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

        left_ee_pose_ = poseMsgToMatrix(msg);
        matrixToPosOri(left_ee_pose_, left_position_, left_orientation_);


        if (enabled_.load())
        {
            // 检查左臂是否暂停更新
            if (left_arm_paused_.load())
            {
                // 暂停更新：不计算和发布目标位姿，直接返回
                return;
            }

            if (is_update_mode_.load())
            {
                // 更新模式：基于差值计算pose并更新marker
                Eigen::Vector3d calculatedPos;
                Eigen::Quaterniond calculatedOri;

                // 应用摇杆累积偏移到VR当前位置
                Eigen::Vector3d left_position_with_offset = left_position_ + left_thumbstick_offset_;

                // 应用摇杆累积Yaw旋转到VR当前姿态
                Eigen::Quaterniond left_orientation_with_yaw = left_orientation_;
                if (std::abs(left_thumbstick_yaw_offset_) > 0.001)
                {
                    Eigen::AngleAxisd yawRotation(left_thumbstick_yaw_offset_, Eigen::Vector3d::UnitZ());
                    left_orientation_with_yaw = Eigen::Quaterniond(yawRotation) * left_orientation_;
                    left_orientation_with_yaw.normalize();
                }

                calculatePoseFromDifference(left_position_with_offset, left_orientation_with_yaw,
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

                    // 直接发布目标位姿到left_target话题（无坐标转换）
                    publishTargetPoseDirect("left", calculatedPos, calculatedOri);

                    // 更新之前计算的pose
                    prev_calculated_left_position_ = calculatedPos;
                    prev_calculated_left_orientation_ = calculatedOri;
                }
            }
            else
            {
                // 存储模式：只存储VR pose，不更新marker
                // 不计算和发布目标位姿
            }
        }
    }

    void VRInputHandler::vrRightCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        right_ee_pose_ = poseMsgToMatrix(msg);
        matrixToPosOri(right_ee_pose_, right_position_, right_orientation_);

        if (enabled_.load())
        {
            // 检查右臂是否暂停更新
            if (right_arm_paused_.load())
            {
                // 暂停更新：不计算和发布目标位姿，直接返回
                return;
            }

            if (is_update_mode_.load())
            {
                // 更新模式：基于差值计算pose并更新marker
                Eigen::Vector3d calculatedPos;
                Eigen::Quaterniond calculatedOri;

                // 应用摇杆累积偏移到VR当前位置
                Eigen::Vector3d right_position_with_offset = right_position_ + right_thumbstick_offset_;

                // 应用摇杆累积Yaw旋转到VR当前姿态
                Eigen::Quaterniond right_orientation_with_yaw = right_orientation_;
                if (std::abs(right_thumbstick_yaw_offset_) > 0.001)
                {
                    Eigen::AngleAxisd yawRotation(right_thumbstick_yaw_offset_, Eigen::Vector3d::UnitZ());
                    right_orientation_with_yaw = Eigen::Quaterniond(yawRotation) * right_orientation_;
                    right_orientation_with_yaw.normalize();
                }

                calculatePoseFromDifference(right_position_with_offset, right_orientation_with_yaw,
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

                    // 直接发布目标位姿到right_target话题（无坐标转换）
                    publishTargetPoseDirect("right", calculatedPos, calculatedOri);

                    // 更新之前计算的pose
                    prev_calculated_right_position_ = calculatedPos;
                    prev_calculated_right_orientation_ = calculatedOri;
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

    Eigen::Matrix4d VRInputHandler::poseMsgToMatrix(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose(0, 3) = msg->pose.position.x;
        pose(1, 3) = msg->pose.position.y;
        pose(2, 3) = msg->pose.position.z;

        Eigen::Quaterniond q(
            msg->pose.orientation.w,
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z);
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
        // 计算VR pose差值（从base到current的变换）
        Eigen::Vector3d vrPosDiff = vrCurrentPos - vrBasePos;
        Eigen::Quaterniond vrOriDiff = vrBaseOri.inverse() * vrCurrentOri;

        // 镜像模式：翻转x和y轴（面对面控制）
        if (mirror_mode_.load())
        {
            // 位置翻转
            vrPosDiff.x() = -vrPosDiff.x(); // 左右翻转
            vrPosDiff.y() = -vrPosDiff.y(); // 前后翻转
            // vrPosDiff.z() 保持不变（上下不翻转）

            // 旋转翻转（面对面镜像）
            // 方法：对四元数的Y和Z分量取反，实现绕Z轴的镜像
            // 这相当于对旋转进行XY平面的镜像变换
            vrOriDiff.y() = -vrOriDiff.y(); // 翻转Y分量
            // vrOriDiff.z() = -vrOriDiff.z();  // 翻转Z分量
            vrOriDiff.x() = -vrOriDiff.x(); // 翻转X分量
            vrOriDiff.normalize(); // 重新归一化
        }

        // 将相同的变换应用到机器人base pose
        resultPos = robotBasePos + vrPosDiff;
        // resultOri = robotBaseOri * vrOriDiff;
        resultOri = vrOriDiff * robotBaseOri;

        // 归一化四元数以避免漂移
        resultOri.normalize();
    }

    void VRInputHandler::leftGripCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        bool currentGripState = msg->data;
        bool lastState = last_left_grip_state_.load();

        // 检测上升沿（按钮按下）
        if (currentGripState && !lastState)
        {
            // 切换控制模式
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

        last_left_grip_state_.store(currentGripState);
    }

    void VRInputHandler::rightGripCallback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        bool currentGripState = msg->data;
        bool lastState = last_right_grip_state_.load();

        // 检测上升沿（按钮按下）
        if (currentGripState && !lastState)
        {
            // 切换控制模式
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

        last_right_grip_state_.store(currentGripState);
    }

    void VRInputHandler::leftYButtonCallback(std_msgs::msg::Bool::SharedPtr msg)
    {
        bool currentYButtonState = msg->data;
        bool lastState = last_left_y_button_state_.load();

        // 检测上升沿（按键按下）
        if (currentYButtonState && !lastState)
        {
            // 切换左臂暂停状态
            bool was_paused = left_arm_paused_.load();
            left_arm_paused_.store(!was_paused);

            if (!was_paused)
            {
                // 暂停更新：停止更新，不存储基准位姿
                RCLCPP_INFO(node_->get_logger(), "🟡 左Y按键按下 - 左臂更新已暂停！");
            }
            else
            {
                // 恢复更新：存储基准位姿并重置偏移，以便基于新基准继续计算
                vr_base_left_position_ = left_position_;
                vr_base_left_orientation_ = left_orientation_;
                robot_base_left_position_ = robot_current_left_position_;
                robot_base_left_orientation_ = robot_current_left_orientation_;

                // 重置左摇杆累积偏移
                left_thumbstick_offset_ = Eigen::Vector3d::Zero();
                left_thumbstick_yaw_offset_ = 0.0;

                RCLCPP_INFO(node_->get_logger(), "🟡 左Y按键按下 - 左臂更新已恢复！");
                RCLCPP_INFO(node_->get_logger(),
                            "🟡 VR Base Position: [%.3f, %.3f, %.3f]",
                            vr_base_left_position_.x(), vr_base_left_position_.y(), vr_base_left_position_.z());
                RCLCPP_INFO(node_->get_logger(),
                            "🟡 Robot Base Position: [%.3f, %.3f, %.3f]",
                            robot_base_left_position_.x(), robot_base_left_position_.y(), robot_base_left_position_.z());
                RCLCPP_INFO(node_->get_logger(), "🟡 左摇杆偏移已重置！");
            }
        }

        last_left_y_button_state_.store(currentYButtonState);
    }

    void VRInputHandler::rightBButtonCallback(std_msgs::msg::Bool::SharedPtr msg)
    {
        bool currentBButtonState = msg->data;
        bool lastState = last_right_b_button_state_.load();

        // 检测上升沿（按键按下）
        if (currentBButtonState && !lastState)
        {
            // 切换右臂暂停状态
            bool was_paused = right_arm_paused_.load();
            right_arm_paused_.store(!was_paused);

            if (!was_paused)
            {
                // 暂停更新：停止更新，不存储基准位姿
                RCLCPP_INFO(node_->get_logger(), "🔵 右B按键按下 - 右臂更新已暂停！");
            }
            else
            {
                // 恢复更新：存储基准位姿并重置偏移，以便基于新基准继续计算
                vr_base_right_position_ = right_position_;
                vr_base_right_orientation_ = right_orientation_;
                robot_base_right_position_ = robot_current_right_position_;
                robot_base_right_orientation_ = robot_current_right_orientation_;

                // 重置右摇杆累积偏移
                right_thumbstick_offset_ = Eigen::Vector3d::Zero();
                right_thumbstick_yaw_offset_ = 0.0;

                RCLCPP_INFO(node_->get_logger(), "🔵 右B按键按下 - 右臂更新已恢复！");
                RCLCPP_INFO(node_->get_logger(),
                            "🔵 VR Base Position: [%.3f, %.3f, %.3f]",
                            vr_base_right_position_.x(), vr_base_right_position_.y(), vr_base_right_position_.z());
                RCLCPP_INFO(node_->get_logger(),
                            "🔵 Robot Base Position: [%.3f, %.3f, %.3f]",
                            robot_base_right_position_.x(), robot_base_right_position_.y(), robot_base_right_position_.z());
                RCLCPP_INFO(node_->get_logger(), "🔵 右摇杆偏移已重置！");
            }
        }

        last_right_b_button_state_.store(currentBButtonState);
    }

    void VRInputHandler::leftThumbstickAxesCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        // 存储左摇杆轴值
        left_thumbstick_axes_.x() = msg->x;
        left_thumbstick_axes_.y() = msg->y;

        // 在UPDATE模式下累积摇杆输入
        if (enabled_.load() && is_update_mode_.load())
        {
            // 根据握把模式选择不同的控制方式
            if (left_grip_mode_.load())
            {
                // 高度旋转模式：Y轴→Z高度，X轴→Yaw旋转
                double delta_z = left_thumbstick_axes_.y() * LINEAR_SCALE; // Z轴上下
                double delta_yaw = left_thumbstick_axes_.x() * ANGULAR_SCALE; // Yaw旋转

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
                double delta_x = left_thumbstick_axes_.y() * LINEAR_SCALE; // 前后
                double delta_y = left_thumbstick_axes_.x() * LINEAR_SCALE; // 左右

                // 累积XY偏移
                left_thumbstick_offset_.x() -= delta_x;
                left_thumbstick_offset_.y() -= delta_y;

                RCLCPP_DEBUG(node_->get_logger(), "🕹️ Left thumbstick (XY): Y=%.3f→ΔX=%.4f, X=%.3f→ΔY=%.4f",
                             left_thumbstick_axes_.y(), delta_x,
                             left_thumbstick_axes_.x(), delta_y);
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
            // 根据握把模式选择不同的控制方式
            if (right_grip_mode_.load())
            {
                // 高度旋转模式：Y轴→Z高度，X轴→Yaw旋转
                double delta_z = right_thumbstick_axes_.y() * LINEAR_SCALE; // Z轴上下
                double delta_yaw = right_thumbstick_axes_.x() * ANGULAR_SCALE; // Yaw旋转

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
                double delta_x = right_thumbstick_axes_.y() * LINEAR_SCALE; // 前后
                double delta_y = right_thumbstick_axes_.x() * LINEAR_SCALE; // 左右

                // 累积XY偏移
                right_thumbstick_offset_.x() -= delta_x;
                right_thumbstick_offset_.y() -= delta_y;

                RCLCPP_DEBUG(node_->get_logger(), "🕹️ Right thumbstick (XY): Y=%.3f→ΔX=%.4f, X=%.3f→ΔY=%.4f",
                             right_thumbstick_axes_.y(), delta_x,
                             right_thumbstick_axes_.x(), delta_y);
            }
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
