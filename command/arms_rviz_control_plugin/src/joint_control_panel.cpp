#include "arms_rviz_control_plugin/joint_control_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <arms_controller_common/utils/FSMStateTransitionValidator.h>

namespace arms_rviz_control_plugin
{
    JointControlPanel::JointControlPanel(QWidget* parent)
        : Panel(parent)
    {
        // Create UI layout
        auto* main_layout = new QVBoxLayout(this);

        // Create category selection combo box
        category_layout_ = std::make_unique<QHBoxLayout>();
        auto* category_label = new QLabel("关节类别:", this);
        category_combo_ = std::make_unique<QComboBox>(this);
        // Options will be populated in onInitialize based on available controllers
        connect(category_combo_.get(), QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, &JointControlPanel::onCategoryChanged);
        category_layout_->addWidget(category_label);
        category_layout_->addWidget(category_combo_.get());
        category_layout_->addStretch();
        main_layout->addLayout(category_layout_.get());

        // Create scroll area for joint controls
        scroll_area_ = std::make_unique<QScrollArea>(this);
        scroll_area_->setWidgetResizable(true);
        scroll_area_->setVisible(false);

        // Create group box for joint controls (no title)
        joint_control_group_ = std::make_unique<QGroupBox>(this);
        joint_control_group_->setTitle(""); // Hide title
        joint_layout_ = std::make_unique<QVBoxLayout>(joint_control_group_.get());
        joint_layout_->setSpacing(5);

        // Add group box to scroll area
        scroll_area_->setWidget(joint_control_group_.get());
        main_layout->addWidget(scroll_area_.get());

        // Create send button
        send_button_ = std::make_unique<QPushButton>("发送关节位置", this);
        send_button_->setStyleSheet(
            "QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; }");
        send_button_->setVisible(false);
        connect(send_button_.get(), &QPushButton::clicked, this, &JointControlPanel::onSendButtonClicked);
        main_layout->addWidget(send_button_.get());

        // Status label
        status_label_ = std::make_unique<QLabel>("请切换到支持关节控制的状态", this);
        status_label_->setStyleSheet("QLabel { color: #666666; font-style: italic; padding: 5px; }");
        status_label_->setAlignment(Qt::AlignCenter);
        main_layout->addWidget(status_label_.get());
    }

    JointControlPanel::~JointControlPanel() = default;

    void JointControlPanel::onInitialize()
    {
        // Use RViz display context to get the node instead of creating a new one
        node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        // Create joint limits manager
        joint_limits_manager_ = std::make_shared<arms_controller_common::JointLimitsManager>(
            node_->get_logger());

        // Subscribe to robot_description topic to load joint limits from URDF
        // Note: joint_names_ will be initialized in onJointStateReceived, so we'll parse limits there
        robot_description_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                // Always cache robot_description
                robot_description_cache_ = msg->data;
                robot_description_received_ = true;

                // If joints are already initialized, parse limits immediately
                if (joint_limits_manager_ && joints_initialized_ && !joint_names_.empty())
                {
                    joint_limits_manager_->parseFromURDF(msg->data, joint_names_);
                    // Update spinbox ranges after parsing limits
                    updateSpinboxRanges();
                    RCLCPP_INFO(node_->get_logger(),
                                "关节限位已从 /robot_description topic 加载");
                }
                else
                {
                    RCLCPP_DEBUG(node_->get_logger(),
                                 "robot_description 已缓存，等待关节初始化");
                }
            });

        // Declare parameter with empty default
        node_->declare_parameter("joint_controllers", std::vector<std::string>());

        // Get joint controllers from parameters
        available_controllers_ = node_->get_parameter("joint_controllers").as_string_array();

        // Determine available categories and map to controllers
        available_categories_.clear();
        category_to_controller_.clear();

        std::string wbc_controller; // ocs2_wbc_controller (handles left, right, body)
        std::string arm_controller; // ocs2_arm_controller (handles left, right only)

        for (const auto& controller : available_controllers_)
        {
            std::string controller_lower = controller;
            std::transform(controller_lower.begin(), controller_lower.end(),
                           controller_lower.begin(), ::tolower);

            if (controller_lower.find("head") != std::string::npos)
            {
                available_categories_.insert("head");
                category_to_controller_["head"] = controller;
            }
            else if (controller_lower == "left_hand_controller")
            {
                available_categories_.insert("left_hand");
                category_to_controller_["left_hand"] = controller;
            }
            else if (controller_lower == "right_hand_controller")
            {
                available_categories_.insert("right_hand");
                category_to_controller_["right_hand"] = controller;
            }
            else if (controller_lower.find("body") != std::string::npos &&
                controller_lower.find("ocs2_wbc_controller") == std::string::npos &&
                controller_lower.find("ocs2_arm_controller") == std::string::npos)
            {
                // Only map body to dedicated body controller if it's not a WBC/arm controller
                available_categories_.insert("body");
                category_to_controller_["body"] = controller;
            }
            else if (controller_lower.find("ocs2_wbc_controller") != std::string::npos)
            {
                // ocs2_wbc_controller handles left, right, and body
                wbc_controller = controller;
                available_categories_.insert("left");
                available_categories_.insert("right");
                available_categories_.insert("body");
            }
            else if (controller_lower.find("ocs2_arm_controller") != std::string::npos)
            {
                // ocs2_arm_controller handles left and right only (not body)
                // Note: left/right categories will be added later based on actual joint names
                arm_controller = controller;
                // Don't add left/right categories here - wait for joint state to determine
            }
        }

        // Map left and right to WBC controller if found
        if (!wbc_controller.empty())
        {
            category_to_controller_["left"] = wbc_controller;
            category_to_controller_["right"] = wbc_controller;
            // Only map body to WBC controller if not already mapped to a dedicated body controller
            if (category_to_controller_.find("body") == category_to_controller_.end())
            {
                category_to_controller_["body"] = wbc_controller;
            }
        }

        // Map left and right to arm controller if found (and WBC controller not found)
        // Note: For ocs2_arm_controller, left/right mapping will be done after joint state is received
        // to determine if it's a single-arm or dual-arm robot
        if (!arm_controller.empty() && wbc_controller.empty())
        {
            // Store arm_controller for later use, but don't map left/right yet
            // Will be mapped in onJointStateReceived() after checking joint names
        }

        // Update category combo box options
        updateCategoryOptions();

        // Create subscribers
        // Subscribe to FSM command (dedicated topic for state transitions)
        fsm_command_subscriber_ = node_->create_subscription<std_msgs::msg::Int32>(
            "/fsm_command", 10,
            std::bind(&JointControlPanel::onFsmCommandReceived, this, std::placeholders::_1));

        joint_state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            std::bind(&JointControlPanel::onJointStateReceived, this, std::placeholders::_1));

        // Subscribe to current target topics (to update spinboxes when target changes)
        left_current_target_subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "left_current_target", 10,
            std::bind(&JointControlPanel::onLeftCurrentTargetReceived, this, std::placeholders::_1));

        right_current_target_subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "right_current_target", 10,
            std::bind(&JointControlPanel::onRightCurrentTargetReceived, this, std::placeholders::_1));

        // Initialize publisher (will be updated when category changes)
        updatePublisher();

        // Initialize visibility based on default state (command = 2, not enabled)
        updatePanelVisibility();

        RCLCPP_INFO(node_->get_logger(), "Joint Control Panel initialized with %zu controllers",
                    available_controllers_.size());
        if (!available_categories_.empty())
        {
            std::string categories_str;
            for (const auto& cat : available_categories_)
            {
                if (!categories_str.empty()) categories_str += ", ";
                categories_str += cat;
            }
            RCLCPP_INFO(node_->get_logger(), "Available categories: %s", categories_str.c_str());
        }
    }

    void JointControlPanel::onFsmCommandReceived(const std_msgs::msg::Int32::SharedPtr msg)
    {
        // 使用公共的状态转换验证工具类
        std::string new_state;
        bool valid_transition = arms_controller_common::FSMStateTransitionValidator::validateTransition(
            current_state_, msg->data, new_state);

        int32_t old_command = current_command_;

        // 只有在有效转换时才更新状态和命令
        if (valid_transition)
        {
            current_state_ = new_state;
            current_command_ = msg->data;

            // Enable joint control when command is 3 (OCS2) or 4 (MOVEJ)
            bool should_enable = (msg->data == 3 || msg->data == 4);

            if (should_enable != is_joint_control_enabled_)
            {
                is_joint_control_enabled_ = should_enable;
                // Update publisher when enabling/disabling (left/right category uses different publishers in different modes)
                if (is_joint_control_enabled_)
                {
                    updatePublisher();
                }
                updatePanelVisibility();
            }
            else if (old_command != current_command_ && is_joint_control_enabled_)
            {
                // Command changed but control is still enabled, update publisher and button text
                // Left/right category uses different publishers in OCS2 vs MOVEJ mode
                updatePublisher();
                updatePanelVisibility();
            }
        }
        else if (msg->data == 0 || msg->data == 100)
        {
            // 对于 command 0 和 100（切换姿态等特殊命令），不更新状态但允许处理
            // 这些命令不影响状态转换
            current_command_ = msg->data;
        }
        // 如果转换无效，不更新任何状态
    }

    std::string JointControlPanel::classifyJoint(const std::string& joint_name)
    {
        std::string joint_name_lower = joint_name;
        std::transform(joint_name_lower.begin(), joint_name_lower.end(),
                       joint_name_lower.begin(), ::tolower);

        // Check for head joints
        if (joint_name_lower.find("head") != std::string::npos)
        {
            return "head";
        }

        // Check for left hand joints (must check before left arm to prioritize hand)
        bool is_hand_joint = joint_name_lower.find("hand") != std::string::npos ||
                             joint_name_lower.find("finger") != std::string::npos ||
                             joint_name_lower.find("thumb") != std::string::npos ||
                             joint_name_lower.find("palm") != std::string::npos;
        
        if (is_hand_joint)
        {
            // Check for left hand
            if (joint_name_lower.find("left") != std::string::npos ||
                (joint_name_lower.length() > 0 && joint_name_lower[0] == 'l' &&
                 (joint_name_lower.find("hand") != std::string::npos ||
                  joint_name_lower.find("finger") != std::string::npos ||
                  joint_name_lower.find("thumb") != std::string::npos ||
                  joint_name_lower.find("palm") != std::string::npos)))
            {
                return "left_hand";
            }
            // Check for right hand
            if (joint_name_lower.find("right") != std::string::npos ||
                (joint_name_lower.length() > 0 && joint_name_lower[0] == 'r' &&
                 (joint_name_lower.find("hand") != std::string::npos ||
                  joint_name_lower.find("finger") != std::string::npos ||
                  joint_name_lower.find("thumb") != std::string::npos ||
                  joint_name_lower.find("palm") != std::string::npos)))
            {
                return "right_hand";
            }
        }

        // Check for left arm joints
        if (joint_name_lower.find("left") != std::string::npos ||
            joint_name_lower.find("l_") != std::string::npos ||
            (joint_name_lower.length() > 0 && joint_name_lower[0] == 'l' &&
                (joint_name_lower[1] == '_' || std::isdigit(joint_name_lower[1]))))
        {
            return "left";
        }

        // Check for right arm joints
        if (joint_name_lower.find("right") != std::string::npos ||
            joint_name_lower.find("r_") != std::string::npos ||
            (joint_name_lower.length() > 0 && joint_name_lower[0] == 'r' &&
                (joint_name_lower[1] == '_' || std::isdigit(joint_name_lower[1]))))
        {
            return "right";
        }

        // Default to body for all other joints
        return "body";
    }

    void JointControlPanel::onJointStateReceived(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Initialize joint names and positions on first message
        if (!joints_initialized_ && !msg->name.empty())
        {
            initializeJoints(msg->name);
        }

        // Update joint positions from joint state
        // Only update when NOT in joint control mode (command != 3 and != 4)
        // This allows users to modify values when in control mode
        if (joints_initialized_ && !msg->name.empty() && !msg->position.empty() && !is_joint_control_enabled_)
        {
            // Update spinbox values from joint state (only if not in control mode)
            for (size_t i = 0; i < joint_names_.size(); ++i)
            {
                const std::string& joint_name = joint_names_[i];

                // Find this joint in the message
                auto it = std::find(msg->name.begin(), msg->name.end(), joint_name);
                if (it != msg->name.end())
                {
                    size_t msg_index = std::distance(msg->name.begin(), it);
                    if (msg_index < msg->position.size())
                    {
                        double position = msg->position[msg_index];
                        joint_positions_[i] = position;

                        // Update spinbox value (block signals to avoid triggering publish)
                        if (joint_spinboxes_[i])
                        {
                            bool was_blocked = joint_spinboxes_[i]->blockSignals(true);
                            joint_spinboxes_[i]->setValue(position);
                            joint_spinboxes_[i]->blockSignals(was_blocked);
                        }
                    }
                }
            }
        }
    }

    void JointControlPanel::updateJointValuesFromState()
    {
        // This function is kept for compatibility but actual update is done in onJointStateReceived
    }

    void JointControlPanel::onLeftCurrentTargetReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // 保存 frame_id
        {
            std::lock_guard<std::mutex> lock(frame_id_mutex_);
            left_target_frame_id_ = msg->header.frame_id;
        }

        // 只要 left_arm_spinboxes_ 已初始化就更新（无论当前 category）
        if (left_arm_spinboxes_.empty() || left_arm_spinboxes_.size() < 7)
        {
            return;
        }

        // 直接使用消息中的位姿值（在 base_frame_ 坐标系下）
        // 更新 spinbox 值（阻止信号避免触发发布）
        bool was_blocked_0 = left_arm_spinboxes_[0]->blockSignals(true);
        bool was_blocked_1 = left_arm_spinboxes_[1]->blockSignals(true);
        bool was_blocked_2 = left_arm_spinboxes_[2]->blockSignals(true);
        bool was_blocked_3 = left_arm_spinboxes_[3]->blockSignals(true);
        bool was_blocked_4 = left_arm_spinboxes_[4]->blockSignals(true);
        bool was_blocked_5 = left_arm_spinboxes_[5]->blockSignals(true);
        bool was_blocked_6 = left_arm_spinboxes_[6]->blockSignals(true);

        // 更新位置 (x, y, z) - 直接使用消息中的值
        left_arm_spinboxes_[0]->setValue(msg->pose.position.x);
        left_arm_spinboxes_[1]->setValue(msg->pose.position.y);
        left_arm_spinboxes_[2]->setValue(msg->pose.position.z);

        // 更新姿态 (qx, qy, qz, qw) - 直接使用消息中的值
        left_arm_spinboxes_[3]->setValue(msg->pose.orientation.x);
        left_arm_spinboxes_[4]->setValue(msg->pose.orientation.y);
        left_arm_spinboxes_[5]->setValue(msg->pose.orientation.z);
        left_arm_spinboxes_[6]->setValue(msg->pose.orientation.w);

        // 恢复信号
        left_arm_spinboxes_[0]->blockSignals(was_blocked_0);
        left_arm_spinboxes_[1]->blockSignals(was_blocked_1);
        left_arm_spinboxes_[2]->blockSignals(was_blocked_2);
        left_arm_spinboxes_[3]->blockSignals(was_blocked_3);
        left_arm_spinboxes_[4]->blockSignals(was_blocked_4);
        left_arm_spinboxes_[5]->blockSignals(was_blocked_5);
        left_arm_spinboxes_[6]->blockSignals(was_blocked_6);
    }

    void JointControlPanel::onRightCurrentTargetReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // 保存 frame_id
        {
            std::lock_guard<std::mutex> lock(frame_id_mutex_);
            right_target_frame_id_ = msg->header.frame_id;
        }

        // 只要 right_arm_spinboxes_ 已初始化就更新（无论当前 category）
        if (right_arm_spinboxes_.empty() || right_arm_spinboxes_.size() < 7)
        {
            return;
        }

        // 直接使用消息中的位姿值（在 base_frame_ 坐标系下）
        // 更新 spinbox 值（阻止信号避免触发发布）
        bool was_blocked_0 = right_arm_spinboxes_[0]->blockSignals(true);
        bool was_blocked_1 = right_arm_spinboxes_[1]->blockSignals(true);
        bool was_blocked_2 = right_arm_spinboxes_[2]->blockSignals(true);
        bool was_blocked_3 = right_arm_spinboxes_[3]->blockSignals(true);
        bool was_blocked_4 = right_arm_spinboxes_[4]->blockSignals(true);
        bool was_blocked_5 = right_arm_spinboxes_[5]->blockSignals(true);
        bool was_blocked_6 = right_arm_spinboxes_[6]->blockSignals(true);

        // 更新位置 (x, y, z) - 直接使用消息中的值
        right_arm_spinboxes_[0]->setValue(msg->pose.position.x);
        right_arm_spinboxes_[1]->setValue(msg->pose.position.y);
        right_arm_spinboxes_[2]->setValue(msg->pose.position.z);

        // 更新姿态 (qx, qy, qz, qw) - 直接使用消息中的值
        right_arm_spinboxes_[3]->setValue(msg->pose.orientation.x);
        right_arm_spinboxes_[4]->setValue(msg->pose.orientation.y);
        right_arm_spinboxes_[5]->setValue(msg->pose.orientation.z);
        right_arm_spinboxes_[6]->setValue(msg->pose.orientation.w);

        // 恢复信号
        right_arm_spinboxes_[0]->blockSignals(was_blocked_0);
        right_arm_spinboxes_[1]->blockSignals(was_blocked_1);
        right_arm_spinboxes_[2]->blockSignals(was_blocked_2);
        right_arm_spinboxes_[3]->blockSignals(was_blocked_3);
        right_arm_spinboxes_[4]->blockSignals(was_blocked_4);
        right_arm_spinboxes_[5]->blockSignals(was_blocked_5);
        right_arm_spinboxes_[6]->blockSignals(was_blocked_6);
    }

    bool JointControlPanel::shouldShowSendButton() const
    {
        // Button only visible when controls are enabled
        if (!is_joint_control_enabled_ || !joints_initialized_)
        {
            return false;
        }

        // Hide button if "all" is selected and there are category partitions
        if (current_category_ == "all" && category_combo_ && category_combo_->count() > 1)
        {
            return false;
        }

        // When command is 3 (OCS2), hide button for body (if using ocs2_wbc_controller)
        // But show button for left/right with different text
        if (current_command_ == 3)
        {
            // Hide button for body when using ocs2_wbc_controller
            if (current_category_ == "body")
            {
                auto it = category_to_controller_.find("body");
                if (it != category_to_controller_.end())
                {
                    std::string controller_lower = it->second;
                    std::transform(controller_lower.begin(), controller_lower.end(),
                                   controller_lower.begin(), ::tolower);

                    if (controller_lower.find("ocs2_wbc_controller") != std::string::npos)
                    {
                        return false;
                    }
                }
            }
        }

        // When command is 4 (MOVEJ), hide button for left/right categories
        // NOTE: Now enabled - button will show in MOVEJ mode for left/right categories
        // if (current_command_ == 4)
        // {
        //     if (current_category_ == "left" || current_category_ == "right")
        //     {
        //         return false;
        //     }
        // }

        return true;
    }

    void JointControlPanel::updatePanelVisibility()
    {
        bool should_show_controls = is_joint_control_enabled_ && joints_initialized_;
        scroll_area_->setVisible(should_show_controls);

        // Show/hide category selection combo box
        if (category_layout_)
        {
            // Hide all widgets in the category layout when controls are not enabled
            for (int i = 0; i < category_layout_->count(); ++i)
            {
                QLayoutItem* item = category_layout_->itemAt(i);
                if (item && item->widget())
                {
                    item->widget()->setVisible(should_show_controls);
                }
            }
        }

        // Show/hide send button and update text
        if (send_button_)
        {
            bool should_show = shouldShowSendButton();
            send_button_->setVisible(should_show);

            // Update button text based on command and category
            if (should_show)
            {
                if (current_command_ == 3 && (current_category_ == "left" || current_category_ == "right"))
                {
                    send_button_->setText("发送末端位置");
                }
                else
                {
                    send_button_->setText("发送关节位置");
                }
            }
        }

        // Hide status label when joint control is enabled
        if (status_label_)
        {
            status_label_->setVisible(!should_show_controls);
        }

        if (should_show_controls)
        {
            updateJointVisibility();
        }
    }

    std::string JointControlPanel::getControllerNameForCategory(const std::string& category)
    {
        if (category == "all")
        {
            // For "all", use the first available controller or default
            if (!available_controllers_.empty())
            {
                std::string controller = available_controllers_[0];
                std::string controller_lower = controller;
                std::transform(controller_lower.begin(), controller_lower.end(),
                               controller_lower.begin(), ::tolower);

                // If it's a WBC controller, use the base topic
                if (controller_lower.find("ocs2_wbc_controller") != std::string::npos ||
                    controller_lower.find("ocs2_arm_controller") != std::string::npos)
                {
                    return "/" + controller + "/target_joint_position";
                }
                else
                {
                    return "/" + controller + "/target_joint_position";
                }
            }
            return "/ocs2_wbc_controller/target_joint_position";
        }

        // Check if we have a controller mapped for this category
        auto it = category_to_controller_.find(category);
        if (it != category_to_controller_.end())
        {
            std::string controller = it->second;
            std::string controller_lower = controller;
            std::transform(controller_lower.begin(), controller_lower.end(),
                           controller_lower.begin(), ::tolower);

            // For ocs2_wbc_controller with left/right/body, use sub-topics
            if ((category == "left" || category == "right" || category == "body") &&
                controller_lower.find("ocs2_wbc_controller") != std::string::npos)
            {
                return "/" + controller + "/target_joint_position/" + category;
            }
            // For ocs2_arm_controller with left/right, use sub-topics (but not body)
            else if ((category == "left" || category == "right") &&
                controller_lower.find("ocs2_arm_controller") != std::string::npos)
            {
                return "/" + controller + "/target_joint_position/" + category;
            }
            else
            {
                // For other controllers (head, dedicated body controller), use direct topic
                return "/" + controller + "/target_joint_position";
            }
        }

        // Fallback to default if no mapping found
        if (category == "head")
        {
            return "/head_joint_controller/target_joint_position";
        }
        else if (category == "body")
        {
            return "/body_joint_controller/target_joint_position";
        }
        else if (category == "left")
        {
            return "/ocs2_wbc_controller/target_joint_position/left";
        }
        else if (category == "right")
        {
            return "/ocs2_wbc_controller/target_joint_position/right";
        }
        else if (category == "left_hand" || category == "right_hand")
        {
            // Try to find specific left_hand_controller or right_hand_controller
            std::string target_controller;
            if (category == "left_hand")
            {
                target_controller = "left_hand_controller";
            }
            else
            {
                target_controller = "right_hand_controller";
            }
            
            // Check if the specific controller exists
            for (const auto& controller : available_controllers_)
            {
                std::string controller_lower = controller;
                std::transform(controller_lower.begin(), controller_lower.end(),
                               controller_lower.begin(), ::tolower);
                if (controller_lower == target_controller)
                {
                    return "/" + controller + "/target_joint_position";
                }
            }
            
            // Fallback: use default hand controller name
            return "/" + target_controller + "/target_joint_position";
        }

        return "/ocs2_wbc_controller/target_joint_position";
    }

    void JointControlPanel::updatePublisher()
    {
        // Reset all publishers
        joint_position_publisher_.reset();
        left_target_publisher_.reset();
        right_target_publisher_.reset();

        // For left/right category:
        // - OCS2 mode (command == 3): use PoseStamped publisher (end-effector pose)
        // - MOVEJ mode (command == 4): use joint position publisher
        if (current_category_ == "left")
        {
            if (current_command_ == 3)
            {
                // Create PoseStamped publisher for left arm (publish to left_target/stamped)
                left_target_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
                    "left_target/stamped", 10);
                RCLCPP_INFO(node_->get_logger(), "Updated publisher to topic: left_target/stamped");
            }
            else
            {
                // Use joint position publisher for MOVEJ mode
                std::string topic_name = getControllerNameForCategory(current_category_);
                joint_position_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
                    topic_name, 10);
                RCLCPP_INFO(node_->get_logger(), "Updated publisher to topic: %s", topic_name.c_str());
            }
        }
        else if (current_category_ == "right")
        {
            if (current_command_ == 3)
            {
                // Create PoseStamped publisher for right arm (publish to right_target/stamped)
                right_target_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
                    "right_target/stamped", 10);
                RCLCPP_INFO(node_->get_logger(), "Updated publisher to topic: right_target/stamped");
            }
            else
            {
                // Use joint position publisher for MOVEJ mode
                std::string topic_name = getControllerNameForCategory(current_category_);
                joint_position_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
                    topic_name, 10);
                RCLCPP_INFO(node_->get_logger(), "Updated publisher to topic: %s", topic_name.c_str());
            }
        }
        else
        {
            // For other categories, use joint position publisher
            std::string topic_name = getControllerNameForCategory(current_category_);
            joint_position_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
                topic_name, 10);
            RCLCPP_INFO(node_->get_logger(), "Updated publisher to topic: %s", topic_name.c_str());
        }
    }

    void JointControlPanel::updateCategoryOptions()
    {
        category_combo_->clear();

        // Always add "全部" option
        category_combo_->addItem("全部", "all");

        // Add available categories
        if (available_categories_.find("body") != available_categories_.end())
        {
            category_combo_->addItem("Body", "body");
        }
        if (available_categories_.find("head") != available_categories_.end())
        {
            category_combo_->addItem("Head", "head");
        }
        if (available_categories_.find("left") != available_categories_.end())
        {
            category_combo_->addItem("Left", "left");
        }
        if (available_categories_.find("right") != available_categories_.end())
        {
            category_combo_->addItem("Right", "right");
        }
        // Add left hand and right hand categories if they exist in category_to_joints_
        if (category_to_joints_.find("left_hand") != category_to_joints_.end() &&
            !category_to_joints_["left_hand"].empty())
        {
            category_combo_->addItem("Left Hand", "left_hand");
            available_categories_.insert("left_hand");
        }
        if (category_to_joints_.find("right_hand") != category_to_joints_.end() &&
            !category_to_joints_["right_hand"].empty())
        {
            category_combo_->addItem("Right Hand", "right_hand");
            available_categories_.insert("right_hand");
        }

        // Set default to first item
        if (category_combo_->count() > 0)
        {
            category_combo_->setCurrentIndex(0);
            current_category_ = category_combo_->currentData().toString().toStdString();
        }
    }

    bool JointControlPanel::hasControllerForCategory(const std::string& category)
    {
        if (category == "all")
        {
            return !available_categories_.empty();
        }
        return available_categories_.find(category) != available_categories_.end();
    }

    void JointControlPanel::onCategoryChanged()
    {
        current_category_ = category_combo_->currentData().toString().toStdString();
        updatePublisher();
        updateJointVisibility();
        // Update button visibility when category changes
        if (is_joint_control_enabled_ && joints_initialized_)
        {
            updatePanelVisibility();
        }
    }

    void JointControlPanel::updateJointVisibility()
    {
        if (!joints_initialized_)
        {
            return;
        }

        // Check if dual-arm mode
        bool has_left_joints = category_to_joints_.find("left") != category_to_joints_.end() &&
            !category_to_joints_["left"].empty();
        bool has_right_joints = category_to_joints_.find("right") != category_to_joints_.end() &&
            !category_to_joints_["right"].empty();
        bool is_dual_arm_mode = has_left_joints || has_right_joints;

        if (is_dual_arm_mode)
        {
            // Dual-arm mode: Show/hide left and right arm UI elements (7 row layouts)
            // Show xyz and quaternion only in OCS2 mode (command == 3)
            // Hide in MOVEJ mode (command == 4)
            bool show_left = (current_category_ == "left" && current_command_ == 3);
            bool show_right = (current_category_ == "right" && current_command_ == 3);

            // Show/hide left arm UI elements (7 row layouts)
            for (size_t i = 0; i < left_arm_row_layouts_.size(); ++i)
            {
                if (left_arm_row_layouts_[i])
                {
                    for (int j = 0; j < left_arm_row_layouts_[i]->count(); ++j)
                    {
                        QLayoutItem* item = left_arm_row_layouts_[i]->itemAt(j);
                        if (item && item->widget())
                        {
                            item->widget()->setVisible(show_left);
                        }
                    }
                }
            }

            // Show/hide right arm UI elements (7 row layouts)
            for (size_t i = 0; i < right_arm_row_layouts_.size(); ++i)
            {
                if (right_arm_row_layouts_[i])
                {
                    for (int j = 0; j < right_arm_row_layouts_[i]->count(); ++j)
                    {
                        QLayoutItem* item = right_arm_row_layouts_[i]->itemAt(j);
                        if (item && item->widget())
                        {
                            item->widget()->setVisible(show_right);
                        }
                    }
                }
            }
        }

        // Show/hide joint-based UI elements (for all joints, both single-arm and dual-arm mode)
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            bool visible = false;

            if (current_category_ == "all")
            {
                visible = true;
            }
            else
            {
                auto it = joint_to_category_.find(joint_names_[i]);
                if (it != joint_to_category_.end() && it->second == current_category_)
                {
                    // For left/right category:
                    // - OCS2 mode (command == 3): hide joints (show xyz/quaternion instead)
                    // - MOVEJ mode (command == 4): show joints (hide xyz/quaternion)
                    if (it->second == "left" || it->second == "right")
                    {
                        visible = (current_command_ == 4); // Show joints only in MOVEJ mode
                    }
                    else
                    {
                        visible = true;
                    }
                }
            }

            // Show/hide the row layout widgets
            if (i < joint_row_layouts_.size() && joint_row_layouts_[i])
            {
                // Show/hide all widgets in the row
                for (int j = 0; j < joint_row_layouts_[i]->count(); ++j)
                {
                    QLayoutItem* item = joint_row_layouts_[i]->itemAt(j);
                    if (item && item->widget())
                    {
                        item->widget()->setVisible(visible);
                    }
                }
            }
        }
    }

    void JointControlPanel::onSendButtonClicked()
    {
        if (is_joint_control_enabled_ && joints_initialized_)
        {
            publishJointPositions();
        }
    }

    void JointControlPanel::publishJointPositions()
    {
        if (!joints_initialized_)
        {
            return;
        }

        // Handle left arm:
        // - OCS2 mode (command == 3): publish PoseStamped with xyz and quaternion
        // - MOVEJ mode (command == 4): publish joint positions
        if (current_category_ == "left")
        {
            if (current_command_ == 3)
            {
                // OCS2 mode: publish PoseStamped
                if (!left_target_publisher_ || left_arm_spinboxes_.empty())
                {
                    return;
                }

                auto msg = geometry_msgs::msg::PoseStamped();

                // Get frame_id from saved value (use default if not set)
                {
                    std::lock_guard<std::mutex> lock(frame_id_mutex_);
                    msg.header.frame_id = left_target_frame_id_.empty() ? "base_link" : left_target_frame_id_;
                }
                msg.header.stamp = node_->get_clock()->now();

                // Set position (x, y, z)
                msg.pose.position.x = left_arm_spinboxes_[0]->value();
                msg.pose.position.y = left_arm_spinboxes_[1]->value();
                msg.pose.position.z = left_arm_spinboxes_[2]->value();

                // Set orientation (qx, qy, qz, qw)
                msg.pose.orientation.x = left_arm_spinboxes_[3]->value();
                msg.pose.orientation.y = left_arm_spinboxes_[4]->value();
                msg.pose.orientation.z = left_arm_spinboxes_[5]->value();
                msg.pose.orientation.w = left_arm_spinboxes_[6]->value();

                left_target_publisher_->publish(msg);
                return;
            }
            else
            {
                // MOVEJ mode: publish joint positions (fall through to joint position publishing logic)
            }
        }

        // Handle right arm:
        // - OCS2 mode (command == 3): publish PoseStamped with xyz and quaternion
        // - MOVEJ mode (command == 4): publish joint positions
        if (current_category_ == "right")
        {
            if (current_command_ == 3)
            {
                // OCS2 mode: publish PoseStamped
                if (!right_target_publisher_ || right_arm_spinboxes_.empty())
                {
                    return;
                }

                auto msg = geometry_msgs::msg::PoseStamped();

                // Get frame_id from saved value (use default if not set)
                {
                    std::lock_guard<std::mutex> lock(frame_id_mutex_);
                    msg.header.frame_id = right_target_frame_id_.empty() ? "base_link" : right_target_frame_id_;
                }
                msg.header.stamp = node_->get_clock()->now();

                // Set position (x, y, z)
                msg.pose.position.x = right_arm_spinboxes_[0]->value();
                msg.pose.position.y = right_arm_spinboxes_[1]->value();
                msg.pose.position.z = right_arm_spinboxes_[2]->value();

                // Set orientation (qx, qy, qz, qw)
                msg.pose.orientation.x = right_arm_spinboxes_[3]->value();
                msg.pose.orientation.y = right_arm_spinboxes_[4]->value();
                msg.pose.orientation.z = right_arm_spinboxes_[5]->value();
                msg.pose.orientation.w = right_arm_spinboxes_[6]->value();

                right_target_publisher_->publish(msg);
                return;
            }
            else
            {
                // MOVEJ mode: publish joint positions (fall through to joint position publishing logic)
            }
        }

        // Handle other categories: publish joint positions as Float64MultiArray
        if (joint_spinboxes_.empty() || !joint_position_publisher_)
        {
            return;
        }

        std::vector<double> target_positions;
        std::vector<std::string> target_joint_names;

        // Collect target positions and joint names
        if (current_category_ == "all")
        {
            target_positions.resize(joint_spinboxes_.size());
            target_joint_names = joint_names_;
            for (size_t i = 0; i < joint_spinboxes_.size(); ++i)
            {
                target_positions[i] = joint_spinboxes_[i]->value();
            }
        }
        else
        {
            // Only publish joints of the current category
            auto it = category_to_joints_.find(current_category_);
            if (it != category_to_joints_.end())
            {
                const auto& joint_indices = it->second;
                target_positions.resize(joint_indices.size());
                target_joint_names.resize(joint_indices.size());
                for (size_t i = 0; i < joint_indices.size(); ++i)
                {
                    size_t joint_idx = joint_indices[i];
                    if (joint_idx < joint_spinboxes_.size() && joint_idx < joint_names_.size())
                    {
                        target_positions[i] = joint_spinboxes_[joint_idx]->value();
                        target_joint_names[i] = joint_names_[joint_idx];
                    }
                }
            }
            else
            {
                return; // No joints in this category
            }
        }

        // Apply joint limits if available
        if (joint_limits_manager_ && joint_limits_manager_->hasAnyLimits())
        {
            target_positions = joint_limits_manager_->applyLimits(target_joint_names, target_positions);
        }

        // Create and publish message
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = target_positions;
        joint_position_publisher_->publish(msg);
    }

    void JointControlPanel::updateSpinboxRanges()
    {
        if (!joint_limits_manager_ || !joints_initialized_ || joint_spinboxes_.empty())
        {
            return;
        }

        for (size_t i = 0; i < joint_names_.size() && i < joint_spinboxes_.size(); ++i)
        {
            const std::string& joint_name = joint_names_[i];
            auto limits = joint_limits_manager_->getJointLimits(joint_name);

            if (limits.initialized)
            {
                // Set spinbox range to joint limits
                joint_spinboxes_[i]->setRange(limits.lower, limits.upper);
                RCLCPP_DEBUG(node_->get_logger(),
                             "Set range for joint %s: [%.6f, %.6f]",
                             joint_name.c_str(), limits.lower, limits.upper);
            }
            else
            {
                // Keep default range if limits not available
                joint_spinboxes_[i]->setRange(-M_PI * 2, M_PI * 2);
            }
        }
    }

    void JointControlPanel::tryParseLimitsFromCache()
    {
        // Try to parse limits from cached robot_description if available
        if (robot_description_received_ && !robot_description_cache_.empty() &&
            joint_limits_manager_ && joints_initialized_ && !joint_names_.empty())
        {
            size_t parsed_count = joint_limits_manager_->parseFromURDF(robot_description_cache_, joint_names_);
            if (parsed_count > 0)
            {
                // Update spinbox ranges after parsing limits
                updateSpinboxRanges();
                RCLCPP_INFO(node_->get_logger(),
                            "关节限位已从缓存的 robot_description 加载 (%zu 个关节)",
                            parsed_count);
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(),
                            "未能从缓存的 robot_description 中解析关节限位");
            }
        }
    }

    void JointControlPanel::initializeJoints(const std::vector<std::string>& joint_names_source)
    {
        if (joints_initialized_ || joint_names_source.empty())
        {
            return;
        }

        joint_names_.clear();
        joint_positions_.clear();
        joint_name_to_index_.clear();
        joint_to_category_.clear();
        category_to_joints_.clear();

        // 如果存在 hand / gripper 控制器，则不要过滤掉手部关节
        bool has_hand_controller = false;
        for (const auto& controller : available_controllers_)
        {
            std::string controller_lower = controller;
            std::transform(controller_lower.begin(), controller_lower.end(),
                           controller_lower.begin(), ::tolower);
            if (controller_lower.find("hand") != std::string::npos ||
                controller_lower.find("gripper") != std::string::npos)
            {
                has_hand_controller = true;
                break;
            }
        }

        // Filter out gripper joints and classify joints
        for (size_t i = 0; i < joint_names_source.size(); ++i)
        {
            const std::string& joint_name = joint_names_source[i];
            // Skip gripper joints - check for gripper, hand, finger, thumb, etc.
            std::string joint_name_lower = joint_name;
            std::transform(joint_name_lower.begin(), joint_name_lower.end(),
                           joint_name_lower.begin(), ::tolower);

            // Classify joint first to check if it's a left_hand or right_hand
            std::string category = classifyJoint(joint_name);
            
            // Filter out gripper-related joints, but keep left_hand and right_hand joints
            bool is_gripper_joint =
                joint_name_lower.find("gripper") != std::string::npos ||
                (joint_name_lower.find("hand") != std::string::npos && 
                 category != "left_hand" && category != "right_hand") ||
                (joint_name_lower.find("finger") != std::string::npos && 
                 category != "left_hand" && category != "right_hand") ||
                (joint_name_lower.find("thumb") != std::string::npos && 
                 category != "left_hand" && category != "right_hand") ||
                (joint_name_lower.find("palm") != std::string::npos && 
                 category != "left_hand" && category != "right_hand");
            // 如果有 hand/gripper 控制器，则不过滤手部关节
            if (has_hand_controller)
            {
                is_gripper_joint = false;
            }

            if (!is_gripper_joint)
            {
                size_t joint_index = joint_names_.size();
                joint_names_.push_back(joint_name);
                joint_name_to_index_[joint_name] = joint_index;

                // Use the category from classifyJoint
                joint_to_category_[joint_name] = category;
                category_to_joints_[category].push_back(joint_index);
            }
        }

        if (joint_names_.empty())
        {
            RCLCPP_WARN(node_->get_logger(), "初始化关节列表失败：未找到有效的关节名称");
            return;
        }

        // Check if dual-arm mode (has left and right joints)
        bool has_left_joints = category_to_joints_.find("left") != category_to_joints_.end() &&
            !category_to_joints_["left"].empty();
        bool has_right_joints = category_to_joints_.find("right") != category_to_joints_.end() &&
            !category_to_joints_["right"].empty();
        bool is_dual_arm_mode = has_left_joints || has_right_joints;

        // Create UI elements for each joint
        joint_row_layouts_.clear();
        joint_labels_.clear();
        joint_spinboxes_.clear();

        // Clear dual-arm mode UI elements
        left_arm_row_layouts_.clear();
        left_arm_labels_.clear();
        left_arm_spinboxes_.clear();
        right_arm_row_layouts_.clear();
        right_arm_labels_.clear();
        right_arm_spinboxes_.clear();

        joint_positions_.resize(joint_names_.size(), 0.0);

        // Dual-arm mode: Create 7 fixed row layouts for xyz and quaternion for each arm
        if (is_dual_arm_mode)
        {
            const std::vector<std::string> param_names = {"x", "y", "z", "qx", "qy", "qz", "qw"};

            // Create left arm UI elements (7 row layouts)
            if (has_left_joints)
            {
                for (size_t i = 0; i < 7; ++i)
                {
                    auto row_layout = std::make_unique<QVBoxLayout>();
                    row_layout->setSpacing(2);

                    // Create label
                    std::string label_text = "Left " + param_names[i];
                    auto label = std::make_unique<QLabel>(QString::fromStdString(label_text),
                                                          joint_control_group_.get());
                    label->setStyleSheet("QLabel { font-weight: bold; }");
                    row_layout->addWidget(label.get());
                    left_arm_labels_.push_back(std::move(label));

                    // Create spinbox
                    auto spinbox = std::make_unique<QDoubleSpinBox>(joint_control_group_.get());
                    spinbox->setRange(-1000.0, 1000.0); // Wide range for position and quaternion
                    spinbox->setSingleStep(0.01);
                    spinbox->setDecimals(6);
                    if (i < 3)
                    {
                        spinbox->setSuffix(" m"); // x, y, z in meters
                        spinbox->setValue(0.0);
                    }
                    else if (i == 6)
                    {
                        spinbox->setSuffix(""); // quaternion w
                        spinbox->setValue(1.0); // Valid quaternion: w=1.0 for no rotation
                    }
                    else
                    {
                        spinbox->setSuffix(""); // quaternion x, y, z
                        spinbox->setValue(0.0);
                    }
                    row_layout->addWidget(spinbox.get());
                    left_arm_spinboxes_.push_back(std::move(spinbox));

                    joint_layout_->addLayout(row_layout.get());
                    left_arm_row_layouts_.push_back(std::move(row_layout));
                }
            }

            // Create right arm UI elements (7 row layouts)
            if (has_right_joints)
            {
                for (size_t i = 0; i < 7; ++i)
                {
                    auto row_layout = std::make_unique<QVBoxLayout>();
                    row_layout->setSpacing(2);

                    // Create label
                    std::string label_text = "Right " + param_names[i];
                    auto label = std::make_unique<QLabel>(QString::fromStdString(label_text),
                                                          joint_control_group_.get());
                    label->setStyleSheet("QLabel { font-weight: bold; }");
                    row_layout->addWidget(label.get());
                    right_arm_labels_.push_back(std::move(label));

                    // Create spinbox
                    auto spinbox = std::make_unique<QDoubleSpinBox>(joint_control_group_.get());
                    spinbox->setRange(-1000.0, 1000.0); // Wide range for position and quaternion
                    spinbox->setSingleStep(0.01);
                    spinbox->setDecimals(6);
                    if (i < 3)
                    {
                        spinbox->setSuffix(" m"); // x, y, z in meters
                        spinbox->setValue(0.0);
                    }
                    else if (i == 6)
                    {
                        spinbox->setSuffix(""); // quaternion w
                        spinbox->setValue(1.0); // Valid quaternion: w=1.0 for no rotation
                    }
                    else
                    {
                        spinbox->setSuffix(""); // quaternion x, y, z
                        spinbox->setValue(0.0);
                    }
                    row_layout->addWidget(spinbox.get());
                    right_arm_spinboxes_.push_back(std::move(spinbox));

                    joint_layout_->addLayout(row_layout.get());
                    right_arm_row_layouts_.push_back(std::move(row_layout));
                }
            }
        }

        // Create UI elements for each joint (both single-arm and dual-arm mode)
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            // Create vertical layout for each joint (name on top, spinbox below)
            auto row_layout = std::make_unique<QVBoxLayout>();
            row_layout->setSpacing(2);

            // Create label (joint name)
            auto label = std::make_unique<QLabel>(QString::fromStdString(joint_names_[i]), joint_control_group_.get());
            label->setStyleSheet("QLabel { font-weight: bold; }");
            row_layout->addWidget(label.get());
            joint_labels_.push_back(std::move(label));

            // Create spinbox
            auto spinbox = std::make_unique<QDoubleSpinBox>(joint_control_group_.get());
            // Set default range, will be updated when limits are loaded
            spinbox->setRange(-M_PI * 2, M_PI * 2);
            spinbox->setSingleStep(0.01);
            spinbox->setDecimals(4);
            spinbox->setSuffix(" rad");
            spinbox->setValue(0.0);
            // No automatic trigger - user clicks send button instead
            row_layout->addWidget(spinbox.get());
            joint_spinboxes_.push_back(std::move(spinbox));

            joint_layout_->addLayout(row_layout.get());
            joint_row_layouts_.push_back(std::move(row_layout));
        }

        joints_initialized_ = true;

        // Set joint names in limits manager
        if (joint_limits_manager_)
        {
            joint_limits_manager_->setJointNames(joint_names_);
        }

        // Try to parse limits from cached robot_description if available
        tryParseLimitsFromCache();

        // For ocs2_arm_controller, check if we have left/right joints
        // If yes, add left/right categories; if no, it's a single-arm robot
        std::string arm_controller;
        for (const auto& controller : available_controllers_)
        {
            std::string controller_lower = controller;
            std::transform(controller_lower.begin(), controller_lower.end(),
                           controller_lower.begin(), ::tolower);
            if (controller_lower.find("ocs2_arm_controller") != std::string::npos)
            {
                arm_controller = controller;
                break;
            }
        }

        // Use the has_left_joints and has_right_joints already declared above

        // If we have ocs2_arm_controller and no left/right joints, it's a single-arm robot
        // Don't add left/right categories
        if (!arm_controller.empty() && !has_left_joints && !has_right_joints)
        {
            RCLCPP_INFO(node_->get_logger(), "Detected single-arm robot (no left/right prefixes in joint names)");
            // For single-arm, all joints go to the base topic
            // No need to add left/right categories
            // Update category options to reflect this (remove left/right if they were added earlier)
            updateCategoryOptions();
        }
        // If we have left/right joints, add categories and mappings
        else if (!arm_controller.empty() && (has_left_joints || has_right_joints))
        {
            RCLCPP_INFO(node_->get_logger(), "Detected dual-arm robot (found left/right prefixes in joint names)");
            if (has_left_joints)
            {
                available_categories_.insert("left");
                category_to_controller_["left"] = arm_controller;
            }
            if (has_right_joints)
            {
                available_categories_.insert("right");
                category_to_controller_["right"] = arm_controller;
            }
            // Update category options after adding left/right
            updateCategoryOptions();
        }
        else
        {
            // Update category options to include left_hand and right_hand if they exist
            updateCategoryOptions();
        }

        updateJointVisibility();

        // Log classification results
        RCLCPP_INFO(node_->get_logger(), "Initialized %zu joints for control", joint_names_.size());
        for (const auto& [category, indices] : category_to_joints_)
        {
            RCLCPP_INFO(node_->get_logger(), "  %s: %zu joints", category.c_str(), indices.size());
        }
    }

    void JointControlPanel::load(const rviz_common::Config& config)
    {
        // Load configuration data
        Panel::load(config);
    }

    void JointControlPanel::save(rviz_common::Config config) const
    {
        // Save configuration data
        Panel::save(config);
    }
} // namespace arms_rviz_control_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(arms_rviz_control_plugin::JointControlPanel, rviz_common::Panel)
