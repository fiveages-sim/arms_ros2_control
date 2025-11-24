//
// Created for Arms ROS2 Control - ArmsTargetManager
//

#include "arms_target_manager/ArmsTargetManager.h"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cmath>

namespace arms_ros2_control::command
{
    ArmsTargetManager::ArmsTargetManager(
        rclcpp::Node::SharedPtr node,
        bool dualArmMode,
        const std::string& frameId,
        const std::string& markerFixedFrame,
        double publishRate,
        const std::vector<int32_t>& disableAutoUpdateStates,
        double markerUpdateInterval,
        bool enableHeadControl,  
        const std::string& headControllerName,
        const std::array<double, 3>& headMarkerPosition)
        : node_(std::move(node))
          , dual_arm_mode_(dualArmMode)
          , control_base_frame_(frameId)
          , marker_fixed_frame_(markerFixedFrame)
          , publish_rate_(publishRate)
          , current_mode_(MarkerState::SINGLE_SHOT)
          , current_controller_state_(2)
          , auto_update_enabled_(true)
          , disable_auto_update_states_(disableAutoUpdateStates)
          , last_marker_update_time_(node_->now())
          , marker_update_interval_(markerUpdateInterval)
          , enable_head_control_(enableHeadControl)  
          , head_controller_name_(headControllerName)
          , head_marker_position_(headMarkerPosition)
    {
    }

    void ArmsTargetManager::initialize()
    {
        // 创建 MarkerFactory
        marker_factory_ = std::make_unique<MarkerFactory>(
            node_, marker_fixed_frame_, disable_auto_update_states_);

        // 创建 InteractiveMarkerServer
        server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
            "arms_target_manager", node_);

        // 初始化TF2 buffer和listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        setupMenu();

        // 定义所有 lambda 函数
        // 初始化 pose 的辅助函数
        auto initPose = [](geometry_msgs::msg::Pose& pose, double x, double y, double z)
        {
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = z;
            pose.orientation.w = 1.0;
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;
        };

        // 初始化 marker 的辅助函数
        auto initMarker = [this](const std::string& markerName, const std::string& markerType,
                                   std::shared_ptr<interactive_markers::MenuHandler>& menuHandler)
        {
            auto marker = buildMarker(markerName, markerType);
            server_->insert(marker);
            server_->setCallback(marker.name, [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
            {
                handleMarkerFeedback(feedback);
            });
            menuHandler->apply(*server_, marker.name);
        };

        // 左臂：初始化 pose -> 创建 marker
        initPose(left_pose_, 0.0, 0.5, 1.0);
        initMarker("left_arm_target", "left_arm", left_menu_handler_);

        // 右臂：初始化 pose -> 创建 marker（如果是双臂模式）
        if (dual_arm_mode_)
        {
            initPose(right_pose_, 0.0, -0.5, 1.0);
            initMarker("right_arm_target", "right_arm", right_menu_handler_);
        }

        // 头部：初始化 pose -> 创建 marker（如果启用头部控制）
        if (enable_head_control_)
        {
            initPose(head_pose_, head_marker_position_[0], head_marker_position_[1], head_marker_position_[2]);
            initMarker("head_target", "head", head_menu_handler_);
        }

        // 创建所有发布器和订阅器
        createPublishersAndSubscribers();

        updateMenuVisibility();

        server_->applyChanges();

        // 输出初始化信息
        RCLCPP_INFO(node_->get_logger(),
                    "ArmsTargetManager initialized. Mode: %s, Control Base Frame: %s, Marker Fixed Frame: %s, Publish Rate: %.1f Hz",
                    dual_arm_mode_ ? "dual_arm" : "single_arm",
                    control_base_frame_.c_str(),
                    marker_fixed_frame_.c_str(),
                    publish_rate_);
        RCLCPP_INFO(node_->get_logger(),
                    "📍 Markers will be created in frame: %s | "
                    "🔄 Received current_pose will be transformed to marker frame: %s | "
                    "📤 Published target poses will be transformed to control base frame: %s",
                    marker_fixed_frame_.c_str(),
                    marker_fixed_frame_.c_str(),
                    control_base_frame_.c_str());
    }

    visualization_msgs::msg::InteractiveMarker ArmsTargetManager::buildMarker(
        const std::string& name,
        const std::string& markerType) const
    {
        // 根据 marker 类型获取对应的 pose
        geometry_msgs::msg::Pose pose;
        if (markerType == "left_arm")
        {
            pose = left_pose_;
        }
        else if (markerType == "right_arm")
        {
            pose = right_pose_;
        }
        else if (markerType == "head")
        {
            pose = head_pose_;
        }
        else
        {
            // 未知类型，使用默认 pose
            pose.position.x = 0.0;
            pose.position.y = 0.0;
            pose.position.z = 0.0;
            pose.orientation.w = 1.0;
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;
        }

        // 使用 MarkerFactory 创建 marker
        return marker_factory_->createMarker(
            name,
            markerType,
            pose,
            current_mode_,
            current_controller_state_,
            auto_update_enabled_);
    }


    void ArmsTargetManager::handleMarkerFeedback(
        const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
    {
        std::string source_frame_id = feedback->header.frame_id;
        std::string marker_name = feedback->marker_name;

        // 转换pose到目标frame（配置的marker_fixed_frame_）
        geometry_msgs::msg::Pose transformed_pose = transformPose(
            feedback->pose, source_frame_id, marker_fixed_frame_);

        // 根据 marker 名称分发处理
        if (marker_name == "left_arm_target")
        {
            // 左臂 marker 处理
            left_pose_ = transformed_pose;

            if (current_mode_ == MarkerState::CONTINUOUS)
            {
                // 在连续发布模式下，只发送左臂目标位姿
                sendTargetPose("left_arm");
            }
        }
        else if (marker_name == "right_arm_target")
        {
            // 右臂 marker 处理
            right_pose_ = transformed_pose;

            if (current_mode_ == MarkerState::CONTINUOUS)
            {
                // 在连续发布模式下，只发送右臂目标位姿
                sendTargetPose("right_arm");
            }
        }
        else if (marker_name == "head_target")
        {
            // 头部 marker 处理
            head_pose_ = transformed_pose;

            if (current_mode_ == MarkerState::CONTINUOUS)
            {
                // 在连续发布模式下，发送头部目标关节位置
                sendTargetPose("head");
            }
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                       "Unknown marker name in feedback: '%s'",
                       marker_name.c_str());
        }
    }



    void ArmsTargetManager::togglePublishMode()
    {
        if (current_mode_ == MarkerState::SINGLE_SHOT)
        {
            current_mode_ = MarkerState::CONTINUOUS;
        }
        else
        {
            current_mode_ = MarkerState::SINGLE_SHOT;
        }

        updateMarkerShape();
        updateMenuVisibility();

        server_->applyChanges();
    }

    MarkerState ArmsTargetManager::getCurrentMode() const
    {
        return current_mode_;
    }

    void ArmsTargetManager::sendTargetPose(const std::string& marker_type)
    {
        // 根据marker类型执行不同的发送操作
        if (marker_type == "head")
        {
            // 发送头部目标关节位置
            if (!head_joint_publisher_)
            {
                RCLCPP_WARN(node_->get_logger(), "Head joint publisher not initialized");
                return;
            }

            // 从头部pose的orientation提取关节角度
            std::vector<double> joint_angles = quaternionToHeadJointAngles(head_pose_.orientation);

            // 创建并发布消息
            std_msgs::msg::Float64MultiArray msg;
            msg.data = joint_angles;

            head_joint_publisher_->publish(msg);

        }
        else if (marker_type == "left_arm")
        {
            // 发送左臂目标位姿
            geometry_msgs::msg::Pose transformed_left_pose = transformPose(left_pose_, marker_fixed_frame_, control_base_frame_);
            left_pose_publisher_->publish(transformed_left_pose);

            // 在双臂模式下，如果只发送左臂目标，也需要发送右臂的当前目标（保持右臂位置）
            // 因为控制器在双臂模式下需要同时收到两个目标才会执行
            if (dual_arm_mode_ && right_pose_publisher_)
            {
                geometry_msgs::msg::Pose transformed_right_pose = transformPose(right_pose_, marker_fixed_frame_, control_base_frame_);
                right_pose_publisher_->publish(transformed_right_pose);
            }
        }
        else if (marker_type == "right_arm")
        {
            // 发送右臂目标位姿
            geometry_msgs::msg::Pose transformed_right_pose = transformPose(right_pose_, marker_fixed_frame_, control_base_frame_);
            right_pose_publisher_->publish(transformed_right_pose);

            // 在双臂模式下，如果只发送右臂目标，也需要发送左臂的当前目标（保持左臂位置）
            // 因为控制器在双臂模式下需要同时收到两个目标才会执行
            if (left_pose_publisher_)
            {
                geometry_msgs::msg::Pose transformed_left_pose = transformPose(left_pose_, marker_fixed_frame_, control_base_frame_);
                left_pose_publisher_->publish(transformed_left_pose);
            }
        }
        else
        {
            // 未知的marker类型
            RCLCPP_WARN(node_->get_logger(),
                       "Unknown marker type: '%s'. Supported types: 'left_arm', 'right_arm', 'head'",
                       marker_type.c_str());
        }
    }


    void ArmsTargetManager::setupMarkerMenu(
        std::shared_ptr<interactive_markers::MenuHandler>& menu_handler,
        interactive_markers::MenuHandler::EntryHandle& send_handle,
        interactive_markers::MenuHandler::EntryHandle& toggle_handle,
        std::function<void()> sendCallback)
    {
        menu_handler = std::make_shared<interactive_markers::MenuHandler>();

        auto menuSendCallback = [sendCallback](
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& /*feedback*/)
        {
            sendCallback();
        };

        auto menuToggleCallback = [this](
            const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& /*feedback*/)
        {
            togglePublishMode();
        };

        send_handle = menu_handler->insert("发送目标", menuSendCallback);

        std::string toggleText = (current_mode_ == MarkerState::CONTINUOUS)
            ? "切换到单次发布"
            : "切换到连续发布";
        toggle_handle = menu_handler->insert(toggleText, menuToggleCallback);
    }

    void ArmsTargetManager::setupMenu()
    {
        // 为左臂设置菜单
        setupMarkerMenu(
            left_menu_handler_,
            left_send_handle_,
            left_toggle_handle_,
            [this]() { sendTargetPose("left_arm"); });

        // 为右臂设置菜单（如果是双臂模式）
        if (dual_arm_mode_)
        {
            setupMarkerMenu(
                right_menu_handler_,
                right_send_handle_,
                right_toggle_handle_,
                [this]() { sendTargetPose("right_arm"); });
        }

        // 为头部设置菜单（如果启用头部控制）
        if (enable_head_control_)
        {
            setupMarkerMenu(
                head_menu_handler_,
                head_send_handle_,
                head_toggle_handle_,
                [this]() { sendTargetPose("head"); });
        }
    }

    void ArmsTargetManager::updateMarkerShape()
    {
        // 统一的 marker 回调函数
        auto markerCallback = [this](const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
        {
            handleMarkerFeedback(feedback);
        };

        // 更新左臂 marker
        auto leftMarker = buildMarker("left_arm_target", "left_arm");
        server_->insert(leftMarker);
        server_->setCallback(leftMarker.name, markerCallback);
        left_menu_handler_->apply(*server_, leftMarker.name);

        if (dual_arm_mode_)
        {
            auto rightMarker = buildMarker("right_arm_target", "right_arm");
            server_->insert(rightMarker);
            server_->setCallback(rightMarker.name, markerCallback);
            right_menu_handler_->apply(*server_, rightMarker.name);
        }

        // 更新头部 marker（如果启用头部控制）
        if (enable_head_control_)
        {
            auto headMarker = buildMarker("head_target", "head");
            server_->insert(headMarker);
            server_->setCallback(headMarker.name, markerCallback);
            head_menu_handler_->apply(*server_, headMarker.name);
        }
    }

    void ArmsTargetManager::updateMenuVisibility()
    {
        setupMenu();

        // 应用菜单到 marker（重新创建菜单后需要先应用）
        left_menu_handler_->apply(*server_, "left_arm_target");
        if (dual_arm_mode_)
        {
            right_menu_handler_->apply(*server_, "right_arm_target");
        }
        // 应用头部菜单（如果启用头部控制）
        if (enable_head_control_)
        {
            head_menu_handler_->apply(*server_, "head_target");
        }

        // 根据当前模式设置菜单项可见性
        if (current_mode_ == MarkerState::CONTINUOUS)
        {
            left_menu_handler_->setVisible(left_send_handle_, false);
            if (dual_arm_mode_)
            {
                right_menu_handler_->setVisible(right_send_handle_, false);
            }
            // 更新头部菜单可见性
            if (enable_head_control_)
            {
                head_menu_handler_->setVisible(head_send_handle_, false);
            }
        }
        else
        {
            left_menu_handler_->setVisible(left_send_handle_, true);
            if (dual_arm_mode_)
            {
                right_menu_handler_->setVisible(right_send_handle_, true);
            }
            // 更新头部菜单可见性
            if (enable_head_control_)
            {
                head_menu_handler_->setVisible(head_send_handle_, true);
            }
        }

        // 重新应用菜单（更新可见性和菜单项文本）
        left_menu_handler_->reApply(*server_);
        if (dual_arm_mode_)
        {
            right_menu_handler_->reApply(*server_);
        }
        // 更新头部菜单
        if (enable_head_control_)
        {
            head_menu_handler_->reApply(*server_);
        }
    }

    void ArmsTargetManager::createPublishersAndSubscribers()
    {
        // 创建左臂发布器和订阅器
        left_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::Pose>("left_target", 1);
        left_end_effector_pose_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "left_current_pose", 10, [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
            {
                updateMarkerFromFeedback(msg, "left_arm");
            });

        // 创建右臂发布器和订阅器（如果是双臂模式）
        if (dual_arm_mode_)
        {
            right_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::Pose>("right_target", 1);
            right_end_effector_pose_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
                "right_current_pose", 10, [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
                {
                    updateMarkerFromFeedback(msg, "right_arm");
                });
        }

        // 创建头部发布器和订阅器（如果启用头部控制）
        if (enable_head_control_)
        {
            std::string head_topic = "/" + head_controller_name_ + "/target_joint_position";
            head_joint_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(head_topic, 1);
            head_joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, [this](const sensor_msgs::msg::JointState::ConstSharedPtr msg)
                {
                    updateMarkerFromFeedback(msg, "head");
                });
        }
    }



    void ArmsTargetManager::setAutoUpdateEnabled(bool enable)
    {
        auto_update_enabled_ = enable;
    }

    bool ArmsTargetManager::isAutoUpdateEnabled() const
    {
        return auto_update_enabled_;
    }

    void ArmsTargetManager::controlInputCallback(const arms_ros2_control_msgs::msg::Inputs::ConstSharedPtr msg)
    {
        int32_t new_state = msg->command;

        if (new_state == 0)
        {
            return;
        }

        if (new_state != current_controller_state_)
        {
            current_controller_state_ = new_state;
            
            // 处理状态切换时的特殊逻辑（根据不同 marker 类型执行相应操作）
            handleStateTransition(new_state);

            // 状态变化时重新创建marker
            updateMarkerShape();
            server_->applyChanges();
        }
    }

    void ArmsTargetManager::handleStateTransition(int32_t new_state)
    {
        // 头部状态切换处理：如果切换到 MOVE 状态（command = 3）且启用了头部控制
        // 将当前头部位置作为目标位置发布，确保头部保持当前位置
        if (new_state == 3 && enable_head_control_ && head_joint_publisher_)
        {
            // 如果有缓存的关节角度，使用缓存的；否则从 head_pose_ 提取
            if (last_head_joint_angles_.size() == 2)
            {
                std_msgs::msg::Float64MultiArray msg;
                msg.data = last_head_joint_angles_;
                head_joint_publisher_->publish(msg);
                RCLCPP_INFO(node_->get_logger(),
                           "Entered MOVE state, published current head position as target: [%.3f, %.3f]",
                           last_head_joint_angles_[0], last_head_joint_angles_[1]);
            }
            else
            {
                // 如果没有缓存，从 head_pose_ 提取（可能不是最新的，但总比没有好）
                sendTargetPose("head");
                RCLCPP_INFO(node_->get_logger(),
                           "Entered MOVE state, published head position from marker as target");
            }
        }

        // 未来可以在这里添加其他 marker 类型的状态切换处理
        // 例如：
        // if (new_state == 3 && enable_arms_control_)
        // {
        //     // 双臂的状态切换处理
        // }
    }


    void ArmsTargetManager::updateMarkerFromFeedback(
        const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg,
        const std::string& marker_type)
    {
        if (!auto_update_enabled_ || isStateDisabled(current_controller_state_))
        {
            return;
        }

        // 将接收到的pose转换到marker_fixed_frame_下
        std::string source_frame_id = pose_msg->header.frame_id;
        geometry_msgs::msg::Pose transformed_pose = transformPose(
            pose_msg->pose, source_frame_id, marker_fixed_frame_);

        // 根据marker类型更新对应的pose和marker
        std::string marker_name;
        geometry_msgs::msg::Pose* target_pose = nullptr;

        if (marker_type == "left_arm")
        {
            target_pose = &left_pose_;
            marker_name = "left_arm_target";
        }
        else if (marker_type == "right_arm")
        {
            if (!dual_arm_mode_)
            {
                return;
            }
            target_pose = &right_pose_;
            marker_name = "right_arm_target";
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                       "Unknown marker type in updateMarkerFromFeedback: '%s'",
                       marker_type.c_str());
            return;
        }

        // 更新pose和marker
        *target_pose = transformed_pose;
        server_->setPose(marker_name, *target_pose);

        if (shouldUpdateMarker())
        {
            server_->applyChanges();
        }
    }

    void ArmsTargetManager::updateMarkerFromFeedback(
        const sensor_msgs::msg::JointState::ConstSharedPtr& joint_msg,
        const std::string& marker_type)
    {
        if (marker_type != "head")
        {
            RCLCPP_WARN(node_->get_logger(),
                       "Invalid marker type for JointState callback: '%s', expected 'head'",
                       marker_type.c_str());
            return;
        }

        if (!enable_head_control_)
        {
            return;
        }

        // 头部 marker 在所有状态下都自动跟踪 head_link2 的 xyz 位置
        // 但在 MOVE 状态下不更新 orientation（orientation 由用户拖拽控制）

        // 检查是否应该更新 orientation
        // HOME (1) 和 HOLD (2) 状态：更新 position 和 orientation（自动跟踪）
        // MOVE (3) 状态：只更新 position，不更新 orientation（position 自动跟踪，orientation 用户控制）
        bool should_update_orientation = true;

        // 如果当前状态在禁用列表中（通常是 MOVE 状态），则不更新 orientation
        if (isStateDisabled(current_controller_state_))
        {
            should_update_orientation = false;
        }

        // 如果自动更新被禁用，也不更新 orientation
        if (!auto_update_enabled_)
        {
            should_update_orientation = false;
        }

        // 从 TF 获取 head_link2 的实际位置并更新（所有状态下都更新位置）
        try
        {
            // 获取 head_link2 在 marker_fixed_frame_ 中的位置（统一使用 marker_fixed_frame_）
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                marker_fixed_frame_, HEAD_LINK_NAME, tf2::TimePointZero);

            // 更新 marker 位置为 head_link2 的实际位置（所有状态下都更新）
            head_pose_.position.x = transform.transform.translation.x;
            head_pose_.position.y = transform.transform.translation.y;
            head_pose_.position.z = transform.transform.translation.z;
        }
        catch (const tf2::TransformException& ex)
        {
            // 如果 TF 转换失败，保持当前位置不变（使用固定位置或上次的位置）
            RCLCPP_DEBUG(node_->get_logger(),
                        "无法从 TF 获取头部 link %s 的位置: %s，保持当前位置",
                        HEAD_LINK_NAME, ex.what());
        }

        // 只有在需要更新 orientation 时才从 joint_states 获取并更新
        if (should_update_orientation)
        {
            // 从 joint_states 中查找 head_joint1 和 head_joint2
            double head_joint1_angle = 0.0;
            double head_joint2_angle = 0.0;
            bool found_joint1 = false;
            bool found_joint2 = false;

            for (size_t i = 0; i < joint_msg->name.size() && i < joint_msg->position.size(); ++i)
            {
                if (joint_msg->name[i] == "head_joint1")
                {
                    head_joint1_angle = joint_msg->position[i];
                    found_joint1 = true;
                }
                else if (joint_msg->name[i] == "head_joint2")
                {
                    head_joint2_angle = joint_msg->position[i];
                    found_joint2 = true;
                }

                if (found_joint1 && found_joint2)
                {
                    break;
                }
            }

            // 如果找到了两个关节，更新头部 marker 的 orientation
            if (found_joint1 && found_joint2)
            {
                // 将关节角度转换为四元数（确保顺序：head_joint1, head_joint2）
                std::vector<double> head_joint_angles = {head_joint1_angle, head_joint2_angle};

                // 缓存最新的关节角度，用于状态切换时发布
                last_head_joint_angles_ = head_joint_angles;

                geometry_msgs::msg::Quaternion quat = headJointAnglesToQuaternion(head_joint_angles);

                // 更新头部 pose 的 orientation（只在非 MOVE 状态下更新）
                head_pose_.orientation = quat;
            }
        }

        // 更新 marker（position 已更新，orientation 根据状态决定是否更新）
        server_->setPose("head_target", head_pose_);

        if (shouldUpdateMarker())
        {
            server_->applyChanges();
        }
    }


    bool ArmsTargetManager::isStateDisabled(int32_t state) const
    {
        return std::find(disable_auto_update_states_.begin(), disable_auto_update_states_.end(), state) !=
            disable_auto_update_states_.end();
    }

    bool ArmsTargetManager::shouldUpdateMarker()
    {
        auto now = node_->now();
        auto time_since_last_update = (now - last_marker_update_time_).seconds();

        if (time_since_last_update >= marker_update_interval_)
        {
            last_marker_update_time_ = now;
            return true;
        }
        return false;
    }

    geometry_msgs::msg::Pose ArmsTargetManager::transformPose(
        const geometry_msgs::msg::Pose& pose,
        const std::string& sourceFrameId,
        const std::string& targetFrameId) const
    {
        // 如果源frame和目标frame相同，不需要转换
        if (sourceFrameId == targetFrameId)
        {
            return pose;
        }

        try
        {
            // 创建PoseStamped用于转换
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = sourceFrameId;
            pose_stamped.header.stamp = rclcpp::Time(0);  // 使用Time(0)表示使用最新变换
            pose_stamped.pose = pose;

            // 获取最新的变换并使用doTransform进行转换
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                targetFrameId, sourceFrameId, tf2::TimePointZero);
            
            // 使用doTransform进行转换
            geometry_msgs::msg::PoseStamped result_stamped;
            tf2::doTransform(pose_stamped, result_stamped, transform);
            return result_stamped.pose;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "无法将pose从 %s 转换到 %s: %s，使用原始pose",
                        sourceFrameId.c_str(), targetFrameId.c_str(), ex.what());
            return pose;
        }
    }

    std::vector<double> ArmsTargetManager::quaternionToHeadJointAngles(
        const geometry_msgs::msg::Quaternion& quaternion) const
    {
        // 使用 tf2 的 getRPY 从 quaternion 提取欧拉角
        tf2::Quaternion tf_quat;
        tf2::fromMsg(quaternion, tf_quat);

        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

        // yaw (Z轴旋转) -> head_joint1
        // pitch (Y轴旋转) -> head_joint2
        // 注意：pitch 取反，使得向上转动 marker 时头部向上看
        // 忽略 roll (X轴旋转)
        return {yaw, -pitch};
    }

    geometry_msgs::msg::Quaternion ArmsTargetManager::headJointAnglesToQuaternion(
        const std::vector<double>& joint_angles) const
    {
        if (joint_angles.size() < 2)
        {
            RCLCPP_WARN(node_->get_logger(), "Invalid joint angles size, expected 2, got %zu", joint_angles.size());
            geometry_msgs::msg::Quaternion quat;
            quat.w = 1.0;
            quat.x = 0.0;
            quat.y = 0.0;
            quat.z = 0.0;
            return quat;
        }

        // head_joint1 -> yaw (Z轴旋转)
        // head_joint2 -> pitch (Y轴旋转，需要取反)
        double yaw = joint_angles[0];
        double pitch = -joint_angles[1];  // 取反，与 quaternionToHeadJointAngles 对应
        double roll = 0.0;  // 忽略 roll

        // 使用 tf2 从 RPY 创建四元数
        tf2::Quaternion tf_quat;
        tf_quat.setRPY(roll, pitch, yaw);
        tf_quat.normalize();

        geometry_msgs::msg::Quaternion quat;
        quat.w = tf_quat.w();
        quat.x = tf_quat.x();
        quat.y = tf_quat.y();
        quat.z = tf_quat.z();
        return quat;
    }

    // ============================================================================
    // 外部 API 接口函数（供 VRInputHandler、ControlInputHandler 等外部类调用）
    // ============================================================================
    // 这些函数与 marker 的创建和管理逻辑分离，专门用于外部控制接口
    // 主要包括：VR 输入、手柄控制输入等外部控制方式

    /**
     * 设置 marker 的绝对位姿（供外部调用，如 VR 输入）
     *
     * 这是一个外部 API 接口，主要用于：
     * - VRInputHandler: VR 控制器输入时调用，设置 marker 的绝对位置和方向
     * - 其他外部控制接口：需要直接设置 marker 目标位姿的场景
     *
     * 功能：
     * 1. 更新内部存储的 pose（left_pose_ 或 right_pose_）
     * 2. 更新 interactive marker 的可视化位置
     * 3. 在连续发布模式下，自动发布目标位姿到控制器
     *
     * @param armType 手臂类型："left" 或 "right"
     * @param position 目标位置（在 marker_fixed_frame_ 坐标系下）
     * @param orientation 目标方向（四元数）
     *
     * @note 如果 armType 无效或不在双臂模式下请求右臂，函数将静默返回
     * @note 在连续发布模式下，会自动转换坐标系并发布目标位姿
     *
     * @see VRInputHandler::updateMarkerPose() - VR 输入调用此函数
     */
    void ArmsTargetManager::setMarkerPose(
        const std::string& armType,
        const geometry_msgs::msg::Point& position,
        const geometry_msgs::msg::Quaternion& orientation)
    {
        geometry_msgs::msg::Pose* current_pose = nullptr;
        std::string marker_name;


        if (armType == "left")
        {
            current_pose = &left_pose_;
            marker_name = "left_arm_target";
        }
        else if (armType == "right" && dual_arm_mode_)
        {
            current_pose = &right_pose_;
            marker_name = "right_arm_target";
        }
        else
        {
            return; // 无效的手臂类型
        }

        // 更新内部 pose 存储
        current_pose->position = position;
        current_pose->orientation = orientation;

        // 更新 interactive marker 的可视化位置
        if (server_)
        {
            server_->setPose(marker_name, *current_pose);
            if (shouldUpdateMarker())
            {
                server_->applyChanges();
            }
        }

        // 在连续发布模式下，自动发送目标位姿到控制器
        if (current_mode_ == MarkerState::CONTINUOUS)
        {
            // 转换坐标系：从 marker_fixed_frame_ 转换到 control_base_frame_
            geometry_msgs::msg::Pose transformed_pose = transformPose(*current_pose, marker_fixed_frame_, control_base_frame_);
            if (armType == "left" && left_pose_publisher_)
            {
                left_pose_publisher_->publish(transformed_pose);
            }
            else if (armType == "right" && dual_arm_mode_ && right_pose_publisher_)
            {
                right_pose_publisher_->publish(transformed_pose);
            }
        }
    }

    /**
     * 增量更新 marker 的位姿（供外部调用，如手柄控制输入）
     *
     * 这是一个外部 API 接口，主要用于：
     * - ControlInputHandler: 手柄/游戏手柄输入时调用，基于增量值更新 marker 位置
     * - 其他需要相对控制的外部接口：基于增量输入调整 marker 位置的场景
     *
     * 功能：
     * 1. 检查控制器状态：只有在禁用状态（如 HOME、HOLD）下才允许增量更新
     * 2. 基于增量值更新位置（相对当前位置的偏移）
     * 3. 基于 RPY 增量更新旋转（使用全局坐标系）
     * 4. 更新 interactive marker 的可视化位置
     * 5. 在连续发布模式下，自动发布目标位姿到控制器
     *
     * @param armType 手臂类型："left" 或 "right"
     * @param positionDelta 位置增量 [x, y, z]（在 marker_fixed_frame_ 坐标系下，已缩放）
     * @param rpyDelta 旋转增量 [roll, pitch, yaw]（弧度，已缩放）
     *
     * @note 只有在禁用状态（disable_auto_update_states_）下才允许增量更新
     * @note 如果在启用状态（如 MOVE）下调用，函数会静默返回
     * @note 旋转使用全局坐标系（世界坐标系），与 marker 当前朝向无关
     * @note 在连续发布模式下，会自动转换坐标系并发布目标位姿
     *
     * @see ControlInputHandler::processControlInput() - 手柄控制调用此函数
     */
    void ArmsTargetManager::updateMarkerPoseIncremental(
        const std::string& armType,
        const std::array<double, 3>& positionDelta,
        const std::array<double, 3>& rpyDelta)
    {
        // 检查是否在禁用状态，只有在禁用状态下才允许增量更新
        // 这是为了避免在 MOVE 状态下与自动跟踪冲突
        if (!isStateDisabled(current_controller_state_))
        {
            RCLCPP_DEBUG(node_->get_logger(), "🎮 Incremental update blocked - controller state %d is not disabled",
                         current_controller_state_);
            return;
        }

        geometry_msgs::msg::Pose* current_pose = nullptr;
        std::string marker_name;

        if (armType == "left")
        {
            current_pose = &left_pose_;
            marker_name = "left_arm_target";
        }
        else if (armType == "right" && dual_arm_mode_)
        {
            current_pose = &right_pose_;
            marker_name = "right_arm_target";
        }
        else
        {
            return; // 无效的手臂类型
        }

        // 更新位置：基于当前位置添加增量
        current_pose->position.x += positionDelta[0];
        current_pose->position.y += positionDelta[1];
        current_pose->position.z += positionDelta[2];

        // 更新旋转：使用 RPY 增量（仅在增量足够大时）
        if (std::abs(rpyDelta[0]) > 0.001 || std::abs(rpyDelta[1]) > 0.001 || std::abs(rpyDelta[2]) > 0.001)
        {
            // 将当前四元数转换为 Eigen 格式
            Eigen::Quaterniond current_quat(
                current_pose->orientation.w,
                current_pose->orientation.x,
                current_pose->orientation.y,
                current_pose->orientation.z
            );

            // 创建旋转增量（RPY 欧拉角）
            Eigen::AngleAxisd rollAngle(rpyDelta[0], Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(rpyDelta[1], Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(rpyDelta[2], Eigen::Vector3d::UnitZ());

            // 组合旋转（ZYX 顺序：先 roll，再 pitch，最后 yaw）
            Eigen::Quaterniond rotationIncrement = yawAngle * pitchAngle * rollAngle;

            // 使用全局坐标系旋转（左乘）
            // 特点：旋转始终绕世界的 X/Y/Z 轴，与 marker 当前朝向无关
            // 适用场景：固定视角控制，与位置控制一致（符合用户直觉）
            current_quat = rotationIncrement * current_quat;
            current_quat.normalize();

            // 转换回 geometry_msgs 格式
            current_pose->orientation.w = current_quat.w();
            current_pose->orientation.x = current_quat.x();
            current_pose->orientation.y = current_quat.y();
            current_pose->orientation.z = current_quat.z();
        }

        // 更新 interactive marker 的可视化位置
        if (server_)
        {
            server_->setPose(marker_name, *current_pose);
            if (shouldUpdateMarker())
            {
                server_->applyChanges();
            }
        }

        // 在连续发布模式下，自动发送目标位姿到控制器
        if (current_mode_ == MarkerState::CONTINUOUS)
        {
            // 转换坐标系：从 marker_fixed_frame_ 转换到 control_base_frame_
            geometry_msgs::msg::Pose transformed_pose = transformPose(*current_pose, marker_fixed_frame_, control_base_frame_);
            if (armType == "left" && left_pose_publisher_)
            {
                left_pose_publisher_->publish(transformed_pose);
            }
            else if (armType == "right" && dual_arm_mode_ && right_pose_publisher_)
            {
                right_pose_publisher_->publish(transformed_pose);
            }
        }
    }

    /**
     * 获取 marker 的当前位姿（供外部查询调用）
     *
     * 这是一个外部 API 接口，用于：
     * - 外部系统查询当前 marker 的目标位姿
     * - 状态监控和日志记录
     * - 其他需要读取 marker 位置的场景
     *
     * 功能：
     * - 返回指定手臂的当前目标位姿（存储在 marker_fixed_frame_ 坐标系下）
     * - 如果手臂类型无效，返回零位姿（默认值）
     *
     * @param armType 手臂类型："left" 或 "right"
     * @return 当前 marker 的位姿（geometry_msgs::msg::Pose）
     *         如果 armType 无效，返回零位姿（位置为原点，方向为单位四元数）
     *
     * @note 返回的 pose 在 marker_fixed_frame_ 坐标系下
     * @note 如果请求右臂但不在双臂模式下，返回零位姿
     */
    geometry_msgs::msg::Pose ArmsTargetManager::getMarkerPose(const std::string& armType) const
    {
        if (armType == "left")
        {
            return left_pose_;
        }
        if (armType == "right" && dual_arm_mode_)
        {
            return right_pose_;
        }

        // 无效的手臂类型，返回零位姿（默认值）
        geometry_msgs::msg::Pose zero_pose;
        zero_pose.position.x = 0.0;
        zero_pose.position.y = 0.0;
        zero_pose.position.z = 0.0;
        zero_pose.orientation.w = 1.0;
        zero_pose.orientation.x = 0.0;
        zero_pose.orientation.y = 0.0;
        zero_pose.orientation.z = 0.0;
        return zero_pose;
    }




} // namespace arms_ros2_control::command
