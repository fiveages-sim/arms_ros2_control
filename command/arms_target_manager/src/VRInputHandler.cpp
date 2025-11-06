//
// Created for Arms ROS2 Control - VRInputHandler
//

#include "arms_target_manager/VRInputHandler.h"
#include "arms_target_manager/ArmsTargetManager.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace arms_ros2_control::command
{

    // 静态常量定义
    const std::string VRInputHandler::XR_NODE_NAME = "/xr_target_node";
    const double VRInputHandler::POSITION_THRESHOLD = 0.01;  // 1cm threshold for position changes
    const double VRInputHandler::ORIENTATION_THRESHOLD = 0.005; // threshold for orientation changes (quaternion angle)
    const double VRInputHandler::LINEAR_SCALE = 0.005;  // 与joystick的linear_scale一致
    const double VRInputHandler::ANGULAR_SCALE = 0.05;  // 与joystick的angular_scale一致

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
        , last_update_time_(node_->now())
        , update_rate_(updateRate)
        , current_position_(0.0, 0.0, 1.0)
        , current_orientation_(1.0, 0.0, 0.0, 0.0)
    {
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

        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VRInputHandler created");
        RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Thumbstick scaling: linear=%.3f, angular=%.3f", LINEAR_SCALE, ANGULAR_SCALE);
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
                
                is_update_mode_.store(true);
                RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Switched to UPDATE mode - Base poses stored!");
                RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ VR Base Positions: Left [%.3f, %.3f, %.3f], Right [%.3f, %.3f, %.3f]", 
                            vr_base_left_position_.x(), vr_base_left_position_.y(), vr_base_left_position_.z(), 
                            vr_base_right_position_.x(), vr_base_right_position_.y(), vr_base_right_position_.z());
                RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ Robot Base Positions: Left [%.3f, %.3f, %.3f], Right [%.3f, %.3f, %.3f]", 
                            robot_base_left_position_.x(), robot_base_left_position_.y(), robot_base_left_position_.z(), 
                            robot_base_right_position_.x(), robot_base_right_position_.y(), robot_base_right_position_.z());
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
                RCLCPP_INFO(node_->get_logger(), "🕹️🕶️🕹️ MIRROR mode ENABLED - Left controller controls right arm, right controller controls left arm");
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
                RCLCPP_WARN(node_->get_logger(), "🕹️🕶️🕹️ Automatically switched to STORAGE mode - Please re-enter UPDATE mode to apply mirror changes");
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
            RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "🕹️🕶️🕹️ xr_target_node found, VR control ENABLED!");
            this->enable();
        }
        else if (!checkNodeExists(node_, XR_NODE_NAME) && enabled_.load())
        {
            this->disable();
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "🕹️🕶️🕹️ xr_target_node not found, VR control DISABLED!");
            return;
        }

        left_ee_pose_ = poseMsgToMatrix(msg);
        matrixToPosOri(left_ee_pose_, left_position_, left_orientation_);
        
        
        if (enabled_.load())
        {
            if (is_update_mode_.load())
            {
                // 更新模式：基于差值计算pose并更新marker
                Eigen::Vector3d calculatedPos;
                Eigen::Quaterniond calculatedOri;
                
                // 应用摇杆累积偏移到VR当前位置
                Eigen::Vector3d left_position_with_offset = left_position_ + left_thumbstick_offset_;
                
                calculatePoseFromDifference(left_position_with_offset, left_orientation_,
                                          vr_base_left_position_, vr_base_left_orientation_,
                                          robot_base_left_position_, robot_base_left_orientation_,
                                          calculatedPos, calculatedOri);
                
                // 检查计算的pose是否发生显著变化
                if (hasPoseChanged(calculatedPos, calculatedOri, prev_calculated_left_position_, prev_calculated_left_orientation_))
                {
                    // 调试输出
                    RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Left VR Base: [%.3f, %.3f, %.3f]", 
                                vr_base_left_position_.x(), vr_base_left_position_.y(), vr_base_left_position_.z());
                    RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Left VR Current: [%.3f, %.3f, %.3f]", 
                                left_position_.x(), left_position_.y(), left_position_.z());
                    RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Left Thumbstick Offset: [%.3f, %.3f, %.3f]", 
                                left_thumbstick_offset_.x(), left_thumbstick_offset_.y(), left_thumbstick_offset_.z());
                    RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Left Robot Base: [%.3f, %.3f, %.3f]", 
                                robot_base_left_position_.x(), robot_base_left_position_.y(), robot_base_left_position_.z());
                    RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Left Calculated: [%.3f, %.3f, %.3f]", 
                                calculatedPos.x(), calculatedPos.y(), calculatedPos.z());

                    // 使用计算的pose更新左臂
                    updateMarkerPose("left", calculatedPos, calculatedOri);
                    
                    // 更新之前计算的pose
                    prev_calculated_left_position_ = calculatedPos;
                    prev_calculated_left_orientation_ = calculatedOri;
                }
            }
            else
            {
                // 存储模式：只存储VR pose，不更新marker
                // 更新之前VR pose用于变化检测（无marker更新）
                prev_vr_left_position_ = left_position_;
                prev_vr_left_orientation_ = left_orientation_;
            }
        }
    }

    void VRInputHandler::vrRightCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        right_ee_pose_ = poseMsgToMatrix(msg);
        matrixToPosOri(right_ee_pose_, right_position_, right_orientation_);
        
        if (enabled_.load())
        {
            if (is_update_mode_.load())
            {
                // 更新模式：基于差值计算pose并更新marker
                Eigen::Vector3d calculatedPos;
                Eigen::Quaterniond calculatedOri;
                
                // 应用摇杆累积偏移到VR当前位置
                Eigen::Vector3d right_position_with_offset = right_position_ + right_thumbstick_offset_;
                
                calculatePoseFromDifference(right_position_with_offset, right_orientation_,
                                          vr_base_right_position_, vr_base_right_orientation_,
                                          robot_base_right_position_, robot_base_right_orientation_,
                                          calculatedPos, calculatedOri);
                
                // 检查计算的pose是否发生显著变化
                if (hasPoseChanged(calculatedPos, calculatedOri, prev_calculated_right_position_, prev_calculated_right_orientation_))
                {

                    // 调试输出
                    RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Right VR Base: [%.3f, %.3f, %.3f]", 
                                vr_base_right_position_.x(), vr_base_right_position_.y(), vr_base_right_position_.z());
                    RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Right VR Current: [%.3f, %.3f, %.3f]", 
                                right_position_.x(), right_position_.y(), right_position_.z());
                    RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Right Thumbstick Offset: [%.3f, %.3f, %.3f]", 
                                right_thumbstick_offset_.x(), right_thumbstick_offset_.y(), right_thumbstick_offset_.z());
                    RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Right Robot Base: [%.3f, %.3f, %.3f]", 
                                robot_base_right_position_.x(), robot_base_right_position_.y(), robot_base_right_position_.z());
                    RCLCPP_DEBUG(node_->get_logger(), "🕹️🕶️🕹️ Right Calculated: [%.3f, %.3f, %.3f]", 
                                calculatedPos.x(), calculatedPos.y(), calculatedPos.z());

                    // 使用计算的pose更新右臂
                    updateMarkerPose("right", calculatedPos, calculatedOri);
                    
                    // 更新之前计算的pose
                    prev_calculated_right_position_ = calculatedPos;
                    prev_calculated_right_orientation_ = calculatedOri;
                }
            }
            else
            {
                // 存储模式：只存储VR pose，不更新marker
                // 更新之前VR pose用于变化检测（无marker更新）
                prev_vr_right_position_ = right_position_;
                prev_vr_right_orientation_ = right_orientation_;
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
            vrPosDiff.x() = -vrPosDiff.x();  // 左右翻转
            vrPosDiff.y() = -vrPosDiff.y();  // 前后翻转
            // vrPosDiff.z() 保持不变（上下不翻转）
        }
        
        // 将相同的变换应用到机器人base pose
        resultPos = robotBasePos + vrPosDiff;
        // resultOri = robotBaseOri * vrOriDiff;
        resultOri = vrOriDiff * robotBaseOri;
        
        // 归一化四元数以避免漂移
        resultOri.normalize();
    }

    void VRInputHandler::leftThumbstickAxesCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        // 存储左摇杆轴值
        left_thumbstick_axes_.x() = msg->x;
        left_thumbstick_axes_.y() = msg->y;
        
        // 在UPDATE模式下累积摇杆输入
        if (enabled_.load() && is_update_mode_.load())
        {
            // 计算位置增量（应用缩放因子）
            double delta_x = left_thumbstick_axes_.y() * LINEAR_SCALE;  // 前后
            double delta_y = left_thumbstick_axes_.x() * LINEAR_SCALE;  // 左右
            double delta_z = 0.0;
            
            // 累积到偏移量（这个偏移量会在vrLeftCallback中应用到vrCurrentPos）
            left_thumbstick_offset_.x() -= delta_x;
            left_thumbstick_offset_.y() -= delta_y;
            left_thumbstick_offset_.z() -= delta_z;
            
            RCLCPP_DEBUG(node_->get_logger(), "🕹️ Left thumbstick: X=%.3f, Y=%.3f → Offset累积: [%.4f, %.4f, %.4f]",
                        left_thumbstick_axes_.x(), left_thumbstick_axes_.y(),
                        left_thumbstick_offset_.x(), left_thumbstick_offset_.y(), left_thumbstick_offset_.z());
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
            // 计算位置增量（应用缩放因子）
            double delta_x = right_thumbstick_axes_.y() * LINEAR_SCALE;  // 前后
            double delta_y = right_thumbstick_axes_.x() * LINEAR_SCALE;  // 左右
            double delta_z = 0.0;
            
            // 累积到偏移量（这个偏移量会在vrRightCallback中应用到vrCurrentPos）
            right_thumbstick_offset_.x() -= delta_x;
            right_thumbstick_offset_.y() -= delta_y;
            right_thumbstick_offset_.z() -= delta_z;
            
            RCLCPP_DEBUG(node_->get_logger(), "🕹️ Right thumbstick: X=%.3f, Y=%.3f → Offset累积: [%.4f, %.4f, %.4f]",
                        right_thumbstick_axes_.x(), right_thumbstick_axes_.y(),
                        right_thumbstick_offset_.x(), right_thumbstick_offset_.y(), right_thumbstick_offset_.z());
        }
    }

} // namespace arms_ros2_control::command
