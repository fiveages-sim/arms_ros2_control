#include "arms_rviz_control_plugin/joint_control_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <map>
#include <set>
#include <arms_controller_common/utils/FSMStateTransitionValidator.h>

namespace
{
    // joint_states 名称常为字母序，与 linkerhand ros2_control 中关节顺序不一致；与硬件/控制器约定对齐。
    int dexterousLinkerHandSortKey(const std::string& joint_name)
    {
        std::string n = joint_name;
        std::transform(n.begin(), n.end(), n.begin(), ::tolower);
        if (n.find("thumb_joint1") != std::string::npos)
        {
            return 0;
        }
        if (n.find("thumb_joint2") != std::string::npos)
        {
            return 1;
        }
        if (n.find("thumb_joint3") != std::string::npos)
        {
            return 2;
        }
        if (n.find("thumb_joint4") != std::string::npos)
        {
            return 3;
        }
        if (n.find("rotate_joint") != std::string::npos)
        {
            return 4;
        }
        if (n.find("gripper_joint") != std::string::npos)
        {
            return 5;
        }
        if (n.find("index_joint") != std::string::npos)
        {
            return 6;
        }
        if (n.find("middle_joint") != std::string::npos)
        {
            return 7;
        }
        if (n.find("ring_joint") != std::string::npos)
        {
            return 8;
        }
        if (n.find("pinky_joint") != std::string::npos)
        {
            return 9;
        }
        return 100;
    }

    bool dexterousLinkerHandJointLess(const std::string& a, const std::string& b)
    {
        const int ka = dexterousLinkerHandSortKey(a);
        const int kb = dexterousLinkerHandSortKey(b);
        if (ka != kb)
        {
            return ka < kb;
        }
        return a < b;
    }

    std::vector<std::string> reorderJointsWithSortedDexterousHands(
        const std::vector<std::string>& old_order,
        const std::map<std::string, std::string>& joint_to_category)
    {
        std::vector<std::string> left_hand;
        std::vector<std::string> right_hand;
        for (const auto& n : old_order)
        {
            auto it = joint_to_category.find(n);
            if (it == joint_to_category.end())
            {
                continue;
            }
            if (it->second == "left_hand")
            {
                left_hand.push_back(n);
            }
            else if (it->second == "right_hand")
            {
                right_hand.push_back(n);
            }
        }
        std::sort(left_hand.begin(), left_hand.end(), dexterousLinkerHandJointLess);
        std::sort(right_hand.begin(), right_hand.end(), dexterousLinkerHandJointLess);

        std::vector<std::string> result;
        result.reserve(old_order.size());
        bool left_block_written = false;
        bool right_block_written = false;
        for (const auto& n : old_order)
        {
            auto it = joint_to_category.find(n);
            const std::string cat = (it != joint_to_category.end()) ? it->second : std::string();
            if (cat == "left_hand")
            {
                if (!left_block_written)
                {
                    for (const auto& h : left_hand)
                    {
                        result.push_back(h);
                    }
                    left_block_written = true;
                }
                continue;
            }
            if (cat == "right_hand")
            {
                if (!right_block_written)
                {
                    for (const auto& h : right_hand)
                    {
                        result.push_back(h);
                    }
                    right_block_written = true;
                }
                continue;
            }
            result.push_back(n);
        }
        return result;
    }
} // namespace

namespace arms_rviz_control_plugin
{
    JointControlPanel::JointControlPanel(QWidget* parent)
        : Panel(parent)
    {
        // Create UI layout
        auto* main_layout = new QVBoxLayout(this);

        // Create category selection combo box
        category_layout_ = std::make_unique<QHBoxLayout>();
        auto* category_label = new QLabel("类别:", this);
        category_combo_ = std::make_unique<QComboBox>(this);
        // Options will be populated in onInitialize based on available controllers
        connect(category_combo_.get(), QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, &JointControlPanel::onCategoryChanged);
        category_layout_->addWidget(category_label);
        category_layout_->addWidget(category_combo_.get());

        auto* display_unit_label = new QLabel("单位:", this);
        display_unit_combo_ = std::make_unique<QComboBox>(this);
        display_unit_combo_->addItem("米 / 弧度", "m_rad");
        display_unit_combo_->addItem("厘米 / 角度", "cm_deg");
        display_unit_combo_->setCurrentIndex(0);
        connect(display_unit_combo_.get(), QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, &JointControlPanel::onDisplayUnitChanged);
        category_layout_->addWidget(display_unit_label);
        category_layout_->addWidget(display_unit_combo_.get());
        category_layout_->addStretch();
        main_layout->addLayout(category_layout_.get());

        // Create scroll area for controls
        scroll_area_ = std::make_unique<QScrollArea>(this);
        scroll_area_->setWidgetResizable(true);
        scroll_area_->setVisible(false);

        // Create scroll content widget
        scroll_content_widget_ = std::make_unique<QWidget>(this);
        scroll_content_layout_ = std::make_unique<QVBoxLayout>(scroll_content_widget_.get());
        scroll_content_layout_->setSpacing(8);
        scroll_content_layout_->setContentsMargins(0, 0, 0, 0);

        // Create waist control group box
        waist_group_box_ = std::make_unique<QGroupBox>("腰部控制", scroll_content_widget_.get());
        waist_group_box_->setStyleSheet("QGroupBox { font-weight: bold; }");
        waist_control_layout_ = std::make_unique<QVBoxLayout>(waist_group_box_.get());
        waist_control_layout_->setSpacing(8);

        // Create original joint control group box (keep original style)
        joint_control_group_ = std::make_unique<QGroupBox>(scroll_content_widget_.get());
        joint_control_group_->setTitle(""); // keep original appearance
        joint_layout_ = std::make_unique<QVBoxLayout>(joint_control_group_.get());
        joint_layout_->setSpacing(5);

        // Lifting scale
        waist_lifting_layout_ = std::make_unique<QVBoxLayout>();
        waist_lifting_layout_->setSpacing(4);

        waist_lifting_label_ = std::make_unique<QLabel>("腰部升降速度比例:", waist_group_box_.get());
        waist_lifting_label_->setStyleSheet("QLabel { font-weight: bold; }");
        waist_lifting_layout_->addWidget(waist_lifting_label_.get());

        waist_lifting_slider_layout_ = std::make_unique<QHBoxLayout>();

        waist_lifting_slider_ = std::make_unique<QSlider>(Qt::Horizontal, waist_group_box_.get());
        waist_lifting_slider_->setRange(0, 100);
        waist_lifting_slider_->setValue(50);
        waist_lifting_slider_->setSingleStep(1);
        waist_lifting_slider_->setPageStep(5);

        waist_lifting_value_label_ = std::make_unique<QLabel>("0.50", waist_group_box_.get());
        waist_lifting_value_label_->setMinimumWidth(50);
        waist_lifting_value_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

        connect(waist_lifting_slider_.get(), &QSlider::valueChanged,
                this, &JointControlPanel::onWaistLiftingSliderChanged);

        waist_lifting_slider_layout_->addWidget(waist_lifting_slider_.get());
        waist_lifting_slider_layout_->addWidget(waist_lifting_value_label_.get());

        waist_lifting_layout_->addLayout(waist_lifting_slider_layout_.get());
        waist_control_layout_->addLayout(waist_lifting_layout_.get());

        // Turning scale
        waist_turning_layout_ = std::make_unique<QVBoxLayout>();
        waist_turning_layout_->setSpacing(4);

        waist_turning_label_ = std::make_unique<QLabel>("腰部旋转速度比例:", waist_group_box_.get());
        waist_turning_label_->setStyleSheet("QLabel { font-weight: bold; }");
        waist_turning_layout_->addWidget(waist_turning_label_.get());

        waist_turning_slider_layout_ = std::make_unique<QHBoxLayout>();

        waist_turning_slider_ = std::make_unique<QSlider>(Qt::Horizontal, waist_group_box_.get());
        waist_turning_slider_->setRange(0, 100);
        waist_turning_slider_->setValue(50);
        waist_turning_slider_->setSingleStep(1);
        waist_turning_slider_->setPageStep(5);

        waist_turning_value_label_ = std::make_unique<QLabel>("0.50", waist_group_box_.get());
        waist_turning_value_label_->setMinimumWidth(50);
        waist_turning_value_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

        connect(waist_turning_slider_.get(), &QSlider::valueChanged,
                this, &JointControlPanel::onWaistTurningSliderChanged);

        waist_turning_slider_layout_->addWidget(waist_turning_slider_.get());
        waist_turning_slider_layout_->addWidget(waist_turning_value_label_.get());

        waist_turning_layout_->addLayout(waist_turning_slider_layout_.get());
        waist_control_layout_->addLayout(waist_turning_layout_.get());

        // Buttons
        waist_button_layout_top_ = std::make_unique<QHBoxLayout>();
        waist_button_layout_bottom_ = std::make_unique<QHBoxLayout>();

        waist_up_button_ = std::make_unique<QPushButton>("上升", waist_group_box_.get());
        waist_down_button_ = std::make_unique<QPushButton>("下降", waist_group_box_.get());
        waist_left_button_ = std::make_unique<QPushButton>("左转", waist_group_box_.get());
        waist_right_button_ = std::make_unique<QPushButton>("右转", waist_group_box_.get());

        waist_up_button_->setStyleSheet(
            "QPushButton { background-color: #5cb85c; color: white; font-weight: bold; padding: 8px; }");
        waist_down_button_->setStyleSheet(
            "QPushButton { background-color: #5cb85c; color: white; font-weight: bold; padding: 8px; }");
        waist_left_button_->setStyleSheet(
            "QPushButton { background-color: #5bc0de; color: white; font-weight: bold; padding: 8px; }");
        waist_right_button_->setStyleSheet(
            "QPushButton { background-color: #5bc0de; color: white; font-weight: bold; padding: 8px; }");

        connect(waist_up_button_.get(), &QPushButton::pressed, this, &JointControlPanel::onWaistUpPressed);
        connect(waist_up_button_.get(), &QPushButton::released, this, &JointControlPanel::onWaistUpReleased);

        connect(waist_down_button_.get(), &QPushButton::pressed, this, &JointControlPanel::onWaistDownPressed);
        connect(waist_down_button_.get(), &QPushButton::released, this, &JointControlPanel::onWaistDownReleased);

        connect(waist_left_button_.get(), &QPushButton::pressed, this, &JointControlPanel::onWaistLeftPressed);
        connect(waist_left_button_.get(), &QPushButton::released, this, &JointControlPanel::onWaistLeftReleased);

        connect(waist_right_button_.get(), &QPushButton::pressed, this, &JointControlPanel::onWaistRightPressed);
        connect(waist_right_button_.get(), &QPushButton::released, this, &JointControlPanel::onWaistRightReleased);

        waist_button_layout_top_->addWidget(waist_up_button_.get());
        waist_button_layout_top_->addWidget(waist_down_button_.get());

        waist_button_layout_bottom_->addWidget(waist_left_button_.get());
        waist_button_layout_bottom_->addWidget(waist_right_button_.get());

        waist_control_layout_->addLayout(waist_button_layout_top_.get());
        waist_control_layout_->addLayout(waist_button_layout_bottom_.get());

        waist_repeat_timer_ = std::make_unique<QTimer>(this);
        waist_repeat_timer_->setInterval(100);
        connect(waist_repeat_timer_.get(), &QTimer::timeout,
                this, &JointControlPanel::onWaistRepeatTimeout);

        updateWaistScaleLabels();

        // Add both boxes into scroll content
        scroll_content_layout_->addWidget(waist_group_box_.get());
        scroll_content_layout_->addWidget(joint_control_group_.get());

        // Set scroll content
        scroll_area_->setWidget(scroll_content_widget_.get());
        main_layout->addWidget(scroll_area_.get());

        // OCS2 位姿类型 + 发送后保持输入：放在发送按钮上方
        ocs2_pose_send_layout_ = std::make_unique<QVBoxLayout>();
        ocs2_pose_send_layout_->setSpacing(6);

        pose_mode_row_layout_ = std::make_unique<QHBoxLayout>();
        pose_mode_label_ = std::make_unique<QLabel>("位姿:", this);
        pose_mode_combo_ = std::make_unique<QComboBox>(this);
        pose_mode_combo_->addItem("绝对", "absolute");
        pose_mode_combo_->addItem("相对基座", "relative_base");
        pose_mode_combo_->addItem("相对末端", "relative_ee");
        pose_mode_combo_->setCurrentIndex(0);
        pose_mode_label_->setVisible(false);
        pose_mode_combo_->setVisible(false);
        connect(pose_mode_combo_.get(), QOverload<int>::of(&QComboBox::currentIndexChanged),
                this, &JointControlPanel::onPoseModeChanged);
        pose_mode_row_layout_->addWidget(pose_mode_label_.get());
        pose_mode_row_layout_->addWidget(pose_mode_combo_.get(), 1);
        ocs2_pose_send_layout_->addLayout(pose_mode_row_layout_.get());

        relative_keep_input_checkbox_ = std::make_unique<QCheckBox>("发送后保持输入", this);
        relative_keep_input_checkbox_->setChecked(false);
        relative_keep_input_checkbox_->setVisible(false);
        relative_keep_input_checkbox_->setToolTip(
            "相对运动发送成功后：勾选则保留输入框数值，未勾选则清空（默认）");
        connect(relative_keep_input_checkbox_.get(), &QCheckBox::toggled, this,
                [this](bool checked) { relative_keep_input_ = checked; });
        ocs2_pose_send_layout_->addWidget(relative_keep_input_checkbox_.get());

        send_button_ = std::make_unique<QPushButton>("发送关节位置", this);
        send_button_->setStyleSheet(
            "QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; }");
        send_button_->setVisible(false);
        connect(send_button_.get(), &QPushButton::clicked, this, &JointControlPanel::onSendButtonClicked);
        ocs2_pose_send_layout_->addWidget(send_button_.get());

        main_layout->addLayout(ocs2_pose_send_layout_.get());

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
                joint_types_ready_ = false;

                if (joints_initialized_)
                {
                    refreshJointMetadataFromCache();
                }
                else
                {
                    RCLCPP_DEBUG(node_->get_logger(),
                                 "robot_description 已缓存，等待关节初始化");
                }
            });

        // Declare parameter with empty default
        // node_->declare_parameter("joint_controllers", std::vector<std::string>());
        if (!node_->has_parameter("joint_controllers")) {
            node_->declare_parameter<std::vector<std::string>>(
                "joint_controllers", std::vector<std::string>{});
        }
        // Get joint controllers from parameters
        available_controllers_ = node_->get_parameter("joint_controllers").as_string_array();

        // Determine available categories and map to controllers
        available_categories_.clear();
        category_to_controller_.clear();

        std::string wbc_controller; // ocs2_wbc_controller（body/head 先登记；left/right 等关节名）

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
                // ocs2_wbc_controller 可覆盖 body/head/left/right。
                // left/right 依赖关节名，等 initializeJoints() 再按实际侧别添加；
                // 这里只先登记 body/head（若尚无专用控制器）。
                wbc_controller = controller;
                if (available_categories_.find("body") == available_categories_.end())
                {
                    available_categories_.insert("body");
                }
                if (available_categories_.find("head") == available_categories_.end())
                {
                    available_categories_.insert("head");
                }
            }
            // ocs2_arm_controller：left/right 同样延迟到 initializeJoints()
        }

        // Map body/head to WBC controller when no dedicated controller already claimed them.
        // left/right are mapped later in initializeJoints() from joint-name sides.
        if (!wbc_controller.empty())
        {
            if (category_to_controller_.find("body") == category_to_controller_.end())
            {
                category_to_controller_["body"] = wbc_controller;
            }
            if (category_to_controller_.find("head") == category_to_controller_.end())
            {
                category_to_controller_["head"] = wbc_controller;
            }
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

        // Subscribe to left current target (right is created lazily when right joints exist)
        left_current_target_subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "left_current_target", 10,
            std::bind(&JointControlPanel::onLeftCurrentTargetReceived, this, std::placeholders::_1));

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

    void JointControlPanel::onBodyCurrentTargetReceived(
        const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (!joints_initialized_ || !msg)
        {
            return;
        }

        auto it = category_to_joints_.find("body");
        if (it == category_to_joints_.end() || it->second.empty())
        {
            return;
        }

        const auto& body_joint_indices = it->second;
        size_t update_count = std::min(body_joint_indices.size(), msg->data.size());

        for (size_t i = 0; i < update_count; ++i)
        {
            size_t joint_idx = body_joint_indices[i];
            double target_value = msg->data[i];

            if (joint_idx < joint_positions_.size())
            {
                joint_positions_[joint_idx] = target_value;
            }

            if (joint_types_ready_ && joint_idx < joint_spinboxes_.size() &&
                joint_spinboxes_[joint_idx])
            {
                setJointSpinboxFromSiValue(joint_idx, target_value);
            }
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
        // if (joint_name_lower.find("left") != std::string::npos ||
        //     joint_name_lower.find("l_") != std::string::npos ||
        //     (joint_name_lower.length() > 0 && joint_name_lower[0] == 'l' &&
        //         (joint_name_lower[1] == '_' || std::isdigit(joint_name_lower[1]))))
        // {
        //     return "left";
        // }
        //
        // // Check for right arm joints
        // if (joint_name_lower.find("right") != std::string::npos ||
        //     joint_name_lower.find("r_") != std::string::npos ||
        //     (joint_name_lower.length() > 0 && joint_name_lower[0] == 'r' &&
        //         (joint_name_lower[1] == '_' || std::isdigit(joint_name_lower[1]))))
        // {
        //     return "right";
        // }

        if (joint_name_lower.find("gripper") != std::string::npos)
        {
            return "";
        }

        // 3. 左臂 - 严格匹配
        // 只匹配以 "left" 或 "l_" 开头，或以 'l' + 数字开头的关节
        bool is_left = (joint_name_lower.find("left") == 0) ||
                       (joint_name_lower.find("l_") == 0) ||
                       (joint_name_lower.length() >= 2 &&
                        joint_name_lower[0] == 'l' &&
                        joint_name_lower[1] >= '0' &&
                        joint_name_lower[1] <= '9');

        if (is_left)
        {
            return "left";
        }

        // 4. 右臂 - 严格匹配
        bool is_right = (joint_name_lower.find("right") == 0) ||
                        (joint_name_lower.find("r_") == 0) ||
                        (joint_name_lower.length() >= 2 &&
                         joint_name_lower[0] == 'r' &&
                         joint_name_lower[1] >= '0' &&
                         joint_name_lower[1] <= '9');

        if (is_right)
        {
            return "right";
        }

        // Default to body. 无侧别前缀的单臂会在 initializeJoints() 中重归为 left。
        return "body";
    }

    void JointControlPanel::onJointStateReceived(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!waist_enabled_checked_)
        {
            refreshWaistEnabledState();
            waist_enabled_checked_ = true;
            if (is_waist_enabled_)
            {
                const std::string waist_controller = getWaistControllerName();

                waist_lifting_publisher_ = node_->create_publisher<std_msgs::msg::Float64>(
                    "/" + waist_controller + "/waist_lifting_command", 10);

                waist_turning_publisher_ = node_->create_publisher<std_msgs::msg::Float64>(
                    "/" + waist_controller + "/waist_turning_command", 10);

                body_current_target_subscriber_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
                    "/" + waist_controller + "/current_target_joint", 10,
                    std::bind(&JointControlPanel::onBodyCurrentTargetReceived, this, std::placeholders::_1));

                RCLCPP_INFO(node_->get_logger(),
                            "Waist control interfaces created on controller: %s",
                            waist_controller.c_str());
            }
            else
            {
                waist_lifting_publisher_.reset();
                waist_turning_publisher_.reset();
                body_current_target_subscriber_.reset();

                if (!getWaistControllerName().empty())
                {
                    RCLCPP_INFO(node_->get_logger(),
                                "Waist control interfaces not created because waist_lifting_enabled is false or unavailable");
                }
            }
        }

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

                        if (joint_types_ready_ && joint_spinboxes_[i])
                        {
                            setJointSpinboxFromSiValue(i, position);
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
        {
            std::lock_guard<std::mutex> lock(frame_id_mutex_);
            left_target_frame_id_ = msg->header.frame_id;
        }

        left_current_pose_ = msg->pose;
        left_current_pose_valid_ = true;

        // 相对模式保留用户输入的增量，不覆盖 spinbox
        if (use_relative_pose_ || left_arm_spinboxes_.size() < 7)
        {
            return;
        }

        setArmPoseSpinboxesFromPose(left_arm_spinboxes_, msg->pose);
    }

    void JointControlPanel::onRightCurrentTargetReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(frame_id_mutex_);
            right_target_frame_id_ = msg->header.frame_id;
        }

        right_current_pose_ = msg->pose;
        right_current_pose_valid_ = true;

        if (use_relative_pose_ || right_arm_spinboxes_.size() < 7)
        {
            return;
        }

        setArmPoseSpinboxesFromPose(right_arm_spinboxes_, msg->pose);
    }

    bool JointControlPanel::shouldShowSendButton() const
    {
        // Button only visible when controls are enabled
        if (!is_joint_control_enabled_ || !joints_initialized_)
        {
            return false;
        }

        // Ordinary joint commands require URDF joint types; OCS2 pose targets do not
        if (!isPoseTargetCommand() && !joint_types_ready_)
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

    void JointControlPanel::updateWaistControlsVisibility(bool visible)
    {
        if (waist_group_box_)
        {
            waist_group_box_->setVisible(visible);
        }

        if (!visible)
        {
            waist_up_pressed_ = false;
            waist_down_pressed_ = false;
            waist_left_pressed_ = false;
            waist_right_pressed_ = false;

            if (waist_repeat_timer_ && waist_repeat_timer_->isActive())
            {
                waist_repeat_timer_->stop();
            }

            stopWaistLifting();
            stopWaistTurning();
        }
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
                    send_button_->setText(use_relative_pose_
                                              ? (pose_mode_ == "relative_ee" ? "发送相对末端" : "发送相对基座")
                                              : "发送末端位姿");
                }
                else
                {
                    send_button_->setText("发送关节位置");
                }
            }
        }

        updatePoseModeControlsVisibility();

        // Hide status label when joint control is enabled, unless waiting for joint types
        if (status_label_)
        {
            const bool waiting_joint_types =
                should_show_controls && !isPoseTargetCommand() && !joint_types_ready_;
            if (waiting_joint_types)
            {
                status_label_->setText(
                    joint_metadata_status_.isEmpty()
                        ? QStringLiteral("等待 robot_description 关节类型")
                        : joint_metadata_status_);
                status_label_->setVisible(true);
            }
            else
            {
                status_label_->setVisible(!should_show_controls);
            }
        }

        if (should_show_controls)
        {
            updateJointVisibility();
        }
        updatePoseModeControlsVisibility();
    }

    std::string JointControlPanel::getControllerNameForCategory(const std::string& category)
    {
        if (category == "all")
        {
            if (!available_controllers_.empty())
            {
                return "/" + available_controllers_[0] + "/target_joint_position";
            }
            return {};
        }

        // Check if we have a controller mapped for this category
        auto it = category_to_controller_.find(category);
        if (it != category_to_controller_.end())
        {
            std::string controller = it->second;
            std::string controller_lower = controller;
            std::transform(controller_lower.begin(), controller_lower.end(),
                           controller_lower.begin(), ::tolower);

            // For ocs2_wbc_controller with left/right/body/head, use sub-topics
            if ((category == "left" || category == "right" || category == "body" || category == "head") &&
                controller_lower.find("ocs2_wbc_controller") != std::string::npos)
            {
                return "/" + controller + "/target_joint_position/" + category;
            }
            // For ocs2_arm_controller with left/right, use sub-topics (but not body)
            else if ((category == "left" || category == "right") &&
                controller_lower.find("ocs2_arm_controller") != std::string::npos)
            {
                // 单臂 MoveJ 只订阅 base topic，没有 /left|/right 子话题
                if (single_arm_mode_)
                {
                    return "/" + controller + "/target_joint_position";
                }
                return "/" + controller + "/target_joint_position/" + category;
            }
            else
            {
                // For other controllers (head, dedicated body controller), use direct topic
                return "/" + controller + "/target_joint_position";
            }
        }

        // Fallback：仅在 available_controllers_ 中查找，不硬编码不存在的控制器名
        if (category == "head")
        {
            for (const auto& controller : available_controllers_)
            {
                std::string controller_lower = controller;
                std::transform(controller_lower.begin(), controller_lower.end(),
                               controller_lower.begin(), ::tolower);
                if (controller_lower.find("head") != std::string::npos)
                {
                    return "/" + controller + "/target_joint_position";
                }
            }
        }
        else if (category == "body")
        {
            for (const auto& controller : available_controllers_)
            {
                std::string controller_lower = controller;
                std::transform(controller_lower.begin(), controller_lower.end(),
                               controller_lower.begin(), ::tolower);
                if (controller_lower.find("body") != std::string::npos &&
                    controller_lower.find("ocs2_wbc_controller") == std::string::npos &&
                    controller_lower.find("ocs2_arm_controller") == std::string::npos)
                {
                    return "/" + controller + "/target_joint_position";
                }
            }
        }
        else if (category == "left" || category == "right")
        {
            for (const auto& controller : available_controllers_)
            {
                std::string controller_lower = controller;
                std::transform(controller_lower.begin(), controller_lower.end(),
                               controller_lower.begin(), ::tolower);
                if (controller_lower.find("ocs2_wbc_controller") != std::string::npos ||
                    controller_lower.find("ocs2_arm_controller") != std::string::npos)
                {
                    if (single_arm_mode_)
                    {
                        return "/" + controller + "/target_joint_position";
                    }
                    return "/" + controller + "/target_joint_position/" + category;
                }
            }
        }
        else if (category == "left_hand" || category == "right_hand")
        {
            const std::string target_controller =
                (category == "left_hand") ? "left_hand_controller" : "right_hand_controller";

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
        }

        if (!available_controllers_.empty())
        {
            return "/" + available_controllers_[0] + "/target_joint_position";
        }
        return {};
    }

    void JointControlPanel::updatePublisher()
    {
        // Reset all publishers
        joint_position_publisher_.reset();
        left_target_publisher_.reset();
        right_target_publisher_.reset();
        left_relative_publisher_.reset();
        right_relative_publisher_.reset();

        // For left/right category:
        // - OCS2 mode (command == 3): use PoseStamped publisher (end-effector pose)
        // - MOVEJ mode (command == 4): use joint position publisher
        if (current_category_ == "left")
        {
            if (current_command_ == 3)
            {
                left_target_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
                    "left_target/stamped", 10);
                left_relative_publisher_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
                    "left_target/relative", 10);
                refreshOcs2FrameParams();
                RCLCPP_INFO(node_->get_logger(),
                            "Updated publishers: left_target/stamped + left_target/relative");
            }
            else
            {
                left_relative_publisher_.reset();
                std::string topic_name = getControllerNameForCategory(current_category_);
                if (topic_name.empty())
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "No joint-position topic for category '%s'", current_category_.c_str());
                    return;
                }
                joint_position_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
                    topic_name, 10);
                RCLCPP_INFO(node_->get_logger(), "Updated publisher to topic: %s", topic_name.c_str());
            }
        }
        else if (current_category_ == "right")
        {
            if (current_command_ == 3)
            {
                right_target_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
                    "right_target/stamped", 10);
                right_relative_publisher_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
                    "right_target/relative", 10);
                refreshOcs2FrameParams();
                RCLCPP_INFO(node_->get_logger(),
                            "Updated publishers: right_target/stamped + right_target/relative");
            }
            else
            {
                right_relative_publisher_.reset();
                std::string topic_name = getControllerNameForCategory(current_category_);
                if (topic_name.empty())
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "No joint-position topic for category '%s'", current_category_.c_str());
                    return;
                }
                joint_position_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
                    topic_name, 10);
                RCLCPP_INFO(node_->get_logger(), "Updated publisher to topic: %s", topic_name.c_str());
            }
        }
        else
        {
            std::string topic_name = getControllerNameForCategory(current_category_);
            if (topic_name.empty())
            {
                RCLCPP_WARN(node_->get_logger(),
                            "No joint-position topic for category '%s'", current_category_.c_str());
                return;
            }
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
            // OCS2：绝对 xyz+四元数（7 行）/ 相对 xyz+rpy（隐藏 qw），仅 command==3
            // MOVEJ (command==4) 隐藏位姿 UI，改用关节 spinbox
            bool show_left = (current_category_ == "left" && current_command_ == 3);
            bool show_right = (current_category_ == "right" && current_command_ == 3);

            for (size_t i = 0; i < left_arm_row_layouts_.size(); ++i)
            {
                if (left_arm_row_layouts_[i])
                {
                    const bool row_visible =
                        show_left && !(use_relative_pose_ && i == 6);
                    for (int j = 0; j < left_arm_row_layouts_[i]->count(); ++j)
                    {
                        QLayoutItem* item = left_arm_row_layouts_[i]->itemAt(j);
                        if (item && item->widget())
                        {
                            item->widget()->setVisible(row_visible);
                        }
                    }
                }
            }

            for (size_t i = 0; i < right_arm_row_layouts_.size(); ++i)
            {
                if (right_arm_row_layouts_[i])
                {
                    const bool row_visible =
                        show_right && !(use_relative_pose_ && i == 6);
                    for (int j = 0; j < right_arm_row_layouts_[i]->count(); ++j)
                    {
                        QLayoutItem* item = right_arm_row_layouts_[i]->itemAt(j);
                        if (item && item->widget())
                        {
                            item->widget()->setVisible(row_visible);
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

        // Waist controls only visible in body category
        updateWaistControlsVisibility(shouldShowWaistControls());
    }

    void JointControlPanel::onSendButtonClicked()
    {
        if (!is_joint_control_enabled_ || !joints_initialized_)
        {
            return;
        }
        if (!isPoseTargetCommand() && !joint_types_ready_)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "Joint command rejected: joint types are not ready");
            status_label_->setText("关节类型尚未加载，禁止发送");
            return;
        }
        publishJointPositions();
    }

    void JointControlPanel::publishJointPositions()
    {
        if (!joints_initialized_)
        {
            return;
        }

        if (current_category_ == "left" && current_command_ == 3)
        {
            publishOcs2ArmPose(true);
            return;
        }
        if (current_category_ == "right" && current_command_ == 3)
        {
            publishOcs2ArmPose(false);
            return;
        }

        // Handle other categories / MOVEJ: publish joint positions as Float64MultiArray
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
                target_positions[i] = getJointSpinboxSiValue(i);
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
                        target_positions[i] = getJointSpinboxSiValue(joint_idx);
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

    void JointControlPanel::onWaistLiftingSliderChanged(int)
    {
        updateWaistScaleLabels();
    }

    void JointControlPanel::onWaistTurningSliderChanged(int)
    {
        updateWaistScaleLabels();
    }

    double JointControlPanel::getWaistLiftingScale() const
    {
        if (!waist_lifting_slider_)
        {
            return 0.0;
        }
        return static_cast<double>(waist_lifting_slider_->value()) / 100.0;
    }

    double JointControlPanel::getWaistTurningScale() const
    {
        if (!waist_turning_slider_)
        {
            return 0.0;
        }
        return static_cast<double>(waist_turning_slider_->value()) / 100.0;
    }

    void JointControlPanel::updateWaistScaleLabels()
    {
        if (waist_lifting_value_label_)
        {
            waist_lifting_value_label_->setText(QString::number(getWaistLiftingScale(), 'f', 2));
        }
        if (waist_turning_value_label_)
        {
            waist_turning_value_label_->setText(QString::number(getWaistTurningScale(), 'f', 2));
        }
    }

    void JointControlPanel::publishWaistLifting(double value)
    {
        if (!waist_lifting_publisher_)
        {
            return;
        }

        std_msgs::msg::Float64 msg;
        msg.data = value;
        waist_lifting_publisher_->publish(msg);
    }

    void JointControlPanel::publishWaistTurning(double value)
    {
        if (!waist_turning_publisher_)
        {
            return;
        }

        std_msgs::msg::Float64 msg;
        msg.data = value;
        waist_turning_publisher_->publish(msg);
    }

    void JointControlPanel::stopWaistLifting()
    {
        publishWaistLifting(0.0);
    }

    void JointControlPanel::stopWaistTurning()
    {
        publishWaistTurning(0.0);
    }

    void JointControlPanel::updateWaistRepeatTimerState()
    {
        const bool any_pressed =
            waist_up_pressed_ || waist_down_pressed_ || waist_left_pressed_ || waist_right_pressed_;

        if (any_pressed)
        {
            if (waist_repeat_timer_ && !waist_repeat_timer_->isActive())
            {
                waist_repeat_timer_->start();
            }
        }
        else
        {
            if (waist_repeat_timer_ && waist_repeat_timer_->isActive())
            {
                waist_repeat_timer_->stop();
            }
        }
    }

    void JointControlPanel::onWaistUpPressed()
    {
        waist_up_pressed_ = true;
        waist_down_pressed_ = false;

        publishWaistLifting(getWaistLiftingScale());
        updateWaistRepeatTimerState();
    }

    void JointControlPanel::onWaistUpReleased()
    {
        waist_up_pressed_ = false;

        if (!waist_down_pressed_)
        {
            stopWaistLifting();
        }

        updateWaistRepeatTimerState();
    }

    void JointControlPanel::onWaistDownPressed()
    {
        waist_down_pressed_ = true;
        waist_up_pressed_ = false;

        publishWaistLifting(-getWaistLiftingScale());
        updateWaistRepeatTimerState();
    }

    void JointControlPanel::onWaistDownReleased()
    {
        waist_down_pressed_ = false;

        if (!waist_up_pressed_)
        {
            stopWaistLifting();
        }

        updateWaistRepeatTimerState();
    }

    void JointControlPanel::onWaistLeftPressed()
    {
        waist_left_pressed_ = true;
        waist_right_pressed_ = false;

        publishWaistTurning(-getWaistTurningScale());
        updateWaistRepeatTimerState();
    }

    void JointControlPanel::onWaistLeftReleased()
    {
        waist_left_pressed_ = false;

        if (!waist_right_pressed_)
        {
            stopWaistTurning();
        }

        updateWaistRepeatTimerState();
    }

    void JointControlPanel::onWaistRightPressed()
    {
        waist_right_pressed_ = true;
        waist_left_pressed_ = false;

        // 这里用你当前测试正确的符号
        publishWaistTurning(getWaistTurningScale());
        updateWaistRepeatTimerState();
    }

    void JointControlPanel::onWaistRightReleased()
    {
        waist_right_pressed_ = false;

        if (!waist_left_pressed_)
        {
            stopWaistTurning();
        }

        updateWaistRepeatTimerState();
    }

    void JointControlPanel::onWaistRepeatTimeout()
    {
        if (waist_up_pressed_ && !waist_down_pressed_)
        {
            publishWaistLifting(getWaistLiftingScale());
        }
        else if (waist_down_pressed_ && !waist_up_pressed_)
        {
            publishWaistLifting(-getWaistLiftingScale());
        }

        if (waist_left_pressed_ && !waist_right_pressed_)
        {
            publishWaistTurning(-getWaistTurningScale());
        }
        else if (waist_right_pressed_ && !waist_left_pressed_)
        {
            publishWaistTurning(getWaistTurningScale());
        }
    }

    void JointControlPanel::refreshWaistEnabledState()
    {
        is_waist_enabled_ = false;

        const std::string waist_controller = getWaistControllerName();
        if (waist_controller.empty())
        {
            updateWaistControlsVisibility(false);
            return;
        }
        const std::string remote_node = "/" + waist_controller;

        try
        {
            auto temp_node = std::make_shared<rclcpp::Node>("waist_param_checker");
            auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
                temp_node, remote_node);

            if (!param_client->wait_for_service(std::chrono::milliseconds(1000)))
            {
                RCLCPP_WARN(node_->get_logger(),
                            "Parameter service of %s is not available",
                            remote_node.c_str());
                updateWaistControlsVisibility(false);
                return;
            }

            if (!param_client->has_parameter("waist_lifting_enabled"))
            {
                RCLCPP_INFO(node_->get_logger(),
                            "Parameter %s/waist_lifting_enabled does not exist",
                            remote_node.c_str());
                updateWaistControlsVisibility(false);
                return;
            }

            is_waist_enabled_ = param_client->get_parameter<bool>("waist_lifting_enabled");

            RCLCPP_INFO(node_->get_logger(),
                        "%s/waist_lifting_enabled = %s",
                        remote_node.c_str(),
                        is_waist_enabled_ ? "true" : "false");
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Failed to query %s/waist_lifting_enabled: %s",
                        remote_node.c_str(), e.what());
            is_waist_enabled_ = false;
        }

        updateWaistControlsVisibility(shouldShowWaistControls());
    }

    std::vector<std::string> JointControlPanel::getControllerJointOrderForCategory(
        const std::string& category) const
    {
        // 面板可能早于 RViz ROS node 完成初始化。
        // 没有 node 就无法创建参数客户端，因此保留当前从 joint_states 得到的顺序。
        if (!node_)
        {
            return {};
        }

        // 只使用 category_to_controller_ 已映射的控制器；无映射则保持 joint_states 顺序。
        const auto controller_it = category_to_controller_.find(category);
        if (controller_it == category_to_controller_.end() || controller_it->second.empty())
        {
            return {};
        }
        const std::string& controller = controller_it->second;

        try
        {
            // SyncParametersClient 不能复用已加入 executor 的 RViz node_。
            auto temp_node = std::make_shared<rclcpp::Node>("joint_order_param_checker");
            auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
                temp_node, "/" + controller);

            if (!param_client->wait_for_service(std::chrono::milliseconds(100)))
            {
                RCLCPP_DEBUG(node_->get_logger(),
                             "Parameter service for %s is not available; keeping joint_states order for %s",
                             controller.c_str(), category.c_str());
                return {};
            }

            if (!param_client->has_parameter("joints"))
            {
                RCLCPP_DEBUG(node_->get_logger(),
                             "Controller %s has no joints parameter; keeping joint_states order for %s",
                             controller.c_str(), category.c_str());
                return {};
            }

            return param_client->get_parameter<std::vector<std::string>>("joints");
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Failed to read %s/joints for %s ordering: %s",
                        controller.c_str(), category.c_str(), e.what());
            return {};
        }
    }

    std::vector<std::string> JointControlPanel::reorderJointsByControllerOrder(
        const std::vector<std::string>& old_order,
        const std::map<std::string, std::string>& joint_to_category,
        const std::map<std::string, std::vector<std::string>>& category_order) const
    {
        // result 是最终面板内部顺序，后续会用于重建 category_to_joints_。
        std::vector<std::string> result;
        result.reserve(old_order.size());

        // emitted 防止同一个关节被控制器顺序和 fallback 顺序重复加入。
        std::set<std::string> emitted;

        // handled_categories 保证每个带控制器顺序的分类只处理一次。
        std::set<std::string> handled_categories;

        for (const auto& joint_name : old_order)
        {
            // 用初始化阶段已经分类好的快照查询当前关节属于 Body/Head/Left/Right 等哪一类。
            const auto cat_it = joint_to_category.find(joint_name);
            const std::string category =
                (cat_it != joint_to_category.end()) ? cat_it->second : std::string();

            // 只有 category_order 中指定的分类才按控制器顺序重排。
            const auto order_it = category_order.find(category);
            if (order_it == category_order.end() || order_it->second.empty())
            {
                // 没有控制器顺序的分类保持 old_order 中的相对顺序。
                if (emitted.insert(joint_name).second)
                {
                    result.push_back(joint_name);
                }
                continue;
            }

            if (handled_categories.insert(category).second)
            {
                // 先按控制器 joints 参数给出的权威顺序加入该分类的关节。
                for (const auto& ordered_joint : order_it->second)
                {
                    // 只接受 joint_states 中真实存在且分类匹配的关节，避免参数中多余关节污染 UI。
                    const auto ordered_cat_it = joint_to_category.find(ordered_joint);
                    if (ordered_cat_it != joint_to_category.end() &&
                        ordered_cat_it->second == category &&
                        emitted.insert(ordered_joint).second)
                    {
                        result.push_back(ordered_joint);
                    }
                }

                // 再把该分类中存在于 joint_states、但没有出现在控制器 joints 参数里的关节追加到末尾。
                // 这样不会因为配置缺项导致 UI 丢失关节。
                for (const auto& fallback_joint : old_order)
                {
                    const auto fallback_cat_it = joint_to_category.find(fallback_joint);
                    if (fallback_cat_it != joint_to_category.end() &&
                        fallback_cat_it->second == category &&
                        emitted.insert(fallback_joint).second)
                    {
                        result.push_back(fallback_joint);
                    }
                }
            }
        }

        // 返回新的全局 joint_names_ 顺序。调用方会据此重建所有 index 映射。
        return result;
    }

    void JointControlPanel::updateSpinboxRanges()
    {
        if (!joint_limits_manager_ || !joints_initialized_ || joint_spinboxes_.empty() ||
            !joint_types_ready_)
        {
            return;
        }

        for (size_t i = 0; i < joint_names_.size() && i < joint_spinboxes_.size(); ++i)
        {
            if (!joint_spinboxes_[i])
            {
                continue;
            }

            const std::string& joint_name = joint_names_[i];
            auto limits = joint_limits_manager_->getJointLimits(joint_name);

            double lower_si = 0.0;
            double upper_si = 0.0;
            if (limits.initialized)
            {
                lower_si = limits.lower;
                upper_si = limits.upper;
            }
            else if (isPrismaticJoint(i))
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "Prismatic joint %s has no position limits; skipping range update",
                             joint_name.c_str());
                continue;
            }
            else
            {
                // Revolute/continuous without explicit limits: keep [-2π, 2π]
                lower_si = -M_PI * 2;
                upper_si = M_PI * 2;
            }

            joint_spinboxes_[i]->setRange(
                toDisplayJointValue(i, lower_si),
                toDisplayJointValue(i, upper_si));
            RCLCPP_DEBUG(node_->get_logger(),
                         "Set range for joint %s: [%.6f, %.6f] (%s)",
                         joint_name.c_str(),
                         toDisplayJointValue(i, lower_si),
                         toDisplayJointValue(i, upper_si),
                         jointUnitSuffix(i).trimmed().toStdString().c_str());
        }
    }

    double JointControlPanel::toDisplayAngle(double radians) const
    {
        return use_cm_deg_ ? (radians * 180.0 / M_PI) : radians;
    }

    double JointControlPanel::fromDisplayAngle(double display_value) const
    {
        return use_cm_deg_ ? (display_value * M_PI / 180.0) : display_value;
    }

    double JointControlPanel::toDisplayLength(double meters) const
    {
        return use_cm_deg_ ? (meters * 100.0) : meters;
    }

    double JointControlPanel::fromDisplayLength(double display_value) const
    {
        return use_cm_deg_ ? (display_value / 100.0) : display_value;
    }

    bool JointControlPanel::isPrismaticJoint(size_t index) const
    {
        return index < joint_names_.size() &&
               joint_limits_manager_ &&
               joint_limits_manager_->isPrismaticJoint(joint_names_[index]);
    }

    double JointControlPanel::toDisplayJointValue(
        size_t index, double value_si) const
    {
        if (!joint_types_ready_ || index >= joint_names_.size())
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "Cannot display joint value: joint type is not ready");
            return 0.0;
        }
        return isPrismaticJoint(index)
                   ? toDisplayLength(value_si)
                   : toDisplayAngle(value_si);
    }

    double JointControlPanel::fromDisplayJointValue(
        size_t index, double display_value) const
    {
        if (!joint_types_ready_ || index >= joint_names_.size())
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "Cannot convert joint command: joint type is not ready");
            return 0.0;
        }
        return isPrismaticJoint(index)
                   ? fromDisplayLength(display_value)
                   : fromDisplayAngle(display_value);
    }

    QString JointControlPanel::jointUnitSuffix(size_t index) const
    {
        if (isPrismaticJoint(index))
        {
            return use_cm_deg_ ? " cm" : " m";
        }
        return use_cm_deg_ ? " deg" : " rad";
    }

    void JointControlPanel::setJointSpinboxFromSiValue(
        size_t index, double value_si)
    {
        if (index >= joint_spinboxes_.size() || !joint_spinboxes_[index])
        {
            return;
        }
        const bool was_blocked = joint_spinboxes_[index]->blockSignals(true);
        joint_spinboxes_[index]->setValue(
            toDisplayJointValue(index, value_si));
        joint_spinboxes_[index]->blockSignals(was_blocked);
    }

    double JointControlPanel::getJointSpinboxSiValue(size_t index) const
    {
        if (index >= joint_spinboxes_.size() || !joint_spinboxes_[index])
        {
            return 0.0;
        }
        return fromDisplayJointValue(
            index, joint_spinboxes_[index]->value());
    }

    void JointControlPanel::applyDisplayUnitToJointSpinboxes()
    {
        if (!joint_types_ready_)
        {
            return;
        }
        for (size_t i = 0; i < joint_spinboxes_.size(); ++i)
        {
            if (!joint_spinboxes_[i])
            {
                continue;
            }
            const bool was_blocked = joint_spinboxes_[i]->blockSignals(true);
            joint_spinboxes_[i]->setSuffix(jointUnitSuffix(i));
            joint_spinboxes_[i]->setDecimals(use_cm_deg_ ? 2 : 4);
            joint_spinboxes_[i]->setSingleStep(use_cm_deg_ ? 1.0 : 0.01);
            joint_spinboxes_[i]->blockSignals(was_blocked);
        }
        updateSpinboxRanges();
    }

    bool JointControlPanel::isPoseTargetCommand() const
    {
        return current_command_ == 3 &&
               (current_category_ == "left" ||
                current_category_ == "right");
    }

    void JointControlPanel::onDisplayUnitChanged()
    {
        if (!display_unit_combo_)
        {
            return;
        }

        const bool new_use_cm_deg =
            display_unit_combo_->currentData().toString() == "cm_deg";
        if (new_use_cm_deg == use_cm_deg_)
        {
            return;
        }

        std::vector<double> values_si(joint_spinboxes_.size(), 0.0);
        if (joint_types_ready_)
        {
            for (size_t i = 0; i < joint_spinboxes_.size(); ++i)
            {
                values_si[i] = getJointSpinboxSiValue(i);
            }
        }

        const geometry_msgs::msg::Pose left_abs = getArmPoseFromSpinboxes(left_arm_spinboxes_);
        const geometry_msgs::msg::Pose right_abs = getArmPoseFromSpinboxes(right_arm_spinboxes_);
        const PoseXyzRpy left_rel = getArmRelativeSpinboxesRadians(left_arm_spinboxes_);
        const PoseXyzRpy right_rel = getArmRelativeSpinboxesRadians(right_arm_spinboxes_);

        use_cm_deg_ = new_use_cm_deg;
        applyDisplayUnitToJointSpinboxes();

        if (joint_types_ready_)
        {
            for (size_t i = 0; i < joint_spinboxes_.size(); ++i)
            {
                setJointSpinboxFromSiValue(i, values_si[i]);
            }
        }

        applyDisplayUnitToPoseSpinboxes(left_arm_spinboxes_);
        applyDisplayUnitToPoseSpinboxes(right_arm_spinboxes_);
        if (use_relative_pose_)
        {
            setArmRelativeSpinboxes(left_arm_spinboxes_, left_rel);
            setArmRelativeSpinboxes(right_arm_spinboxes_, right_rel);
        }
        else
        {
            setArmPoseSpinboxesFromPose(left_arm_spinboxes_, left_abs);
            setArmPoseSpinboxesFromPose(right_arm_spinboxes_, right_abs);
        }
    }

    void JointControlPanel::setArmPoseSpinboxesFromPose(
        std::vector<std::unique_ptr<QDoubleSpinBox>>& spinboxes,
        const geometry_msgs::msg::Pose& pose)
    {
        if (spinboxes.size() < 7)
        {
            return;
        }
        const double vals[7] = {
            toDisplayLength(pose.position.x),
            toDisplayLength(pose.position.y),
            toDisplayLength(pose.position.z),
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        };
        for (size_t i = 0; i < 7; ++i)
        {
            if (!spinboxes[i])
            {
                continue;
            }
            bool blocked = spinboxes[i]->blockSignals(true);
            spinboxes[i]->setValue(vals[i]);
            spinboxes[i]->blockSignals(blocked);
        }
    }

    geometry_msgs::msg::Pose JointControlPanel::getArmPoseFromSpinboxes(
        const std::vector<std::unique_ptr<QDoubleSpinBox>>& spinboxes) const
    {
        geometry_msgs::msg::Pose pose;
        if (spinboxes.size() < 7)
        {
            return pose;
        }
        pose.position.x = fromDisplayLength(spinboxes[0] ? spinboxes[0]->value() : 0.0);
        pose.position.y = fromDisplayLength(spinboxes[1] ? spinboxes[1]->value() : 0.0);
        pose.position.z = fromDisplayLength(spinboxes[2] ? spinboxes[2]->value() : 0.0);
        pose.orientation.x = spinboxes[3] ? spinboxes[3]->value() : 0.0;
        pose.orientation.y = spinboxes[4] ? spinboxes[4]->value() : 0.0;
        pose.orientation.z = spinboxes[5] ? spinboxes[5]->value() : 0.0;
        pose.orientation.w = spinboxes[6] ? spinboxes[6]->value() : 1.0;
        return pose;
    }

    void JointControlPanel::configureArmPoseSpinbox(QDoubleSpinBox* spinbox, size_t index) const
    {
        if (!spinbox)
        {
            return;
        }
        spinbox->setRange(-1000.0, 1000.0);
        spinbox->setValue(index == 6 ? 1.0 : 0.0);  // default identity qw
        if (index < 3)
        {
            spinbox->setSuffix(use_cm_deg_ ? " cm" : " m");
            spinbox->setDecimals(use_cm_deg_ ? 2 : 4);
            spinbox->setSingleStep(use_cm_deg_ ? 1.0 : 0.01);
        }
        else
        {
            // 默认按绝对四元数配置；相对模式由 applyArmPoseUiMode 覆盖
            spinbox->setSuffix("");
            spinbox->setDecimals(4);
            spinbox->setSingleStep(0.01);
        }
    }

    void JointControlPanel::applyDisplayUnitToPoseSpinboxes(
        std::vector<std::unique_ptr<QDoubleSpinBox>>& spinboxes)
    {
        for (size_t i = 0; i < spinboxes.size() && i < 6; ++i)
        {
            if (!spinboxes[i])
            {
                continue;
            }
            bool blocked = spinboxes[i]->blockSignals(true);
            if (i < 3)
            {
                spinboxes[i]->setSuffix(use_cm_deg_ ? " cm" : " m");
                spinboxes[i]->setDecimals(use_cm_deg_ ? 2 : 4);
                spinboxes[i]->setSingleStep(use_cm_deg_ ? 1.0 : 0.01);
            }
            else if (use_relative_pose_)
            {
                spinboxes[i]->setSuffix(use_cm_deg_ ? " deg" : " rad");
                spinboxes[i]->setDecimals(use_cm_deg_ ? 2 : 4);
                spinboxes[i]->setSingleStep(use_cm_deg_ ? 1.0 : 0.01);
            }
            spinboxes[i]->blockSignals(blocked);
        }
    }

    void JointControlPanel::setArmRelativeSpinboxes(
        std::vector<std::unique_ptr<QDoubleSpinBox>>& spinboxes,
        const PoseXyzRpy& xyz_rpy)
    {
        if (spinboxes.size() < 6)
        {
            return;
        }
        const double vals[6] = {
            toDisplayLength(xyz_rpy.x),
            toDisplayLength(xyz_rpy.y),
            toDisplayLength(xyz_rpy.z),
            toDisplayAngle(xyz_rpy.roll),
            toDisplayAngle(xyz_rpy.pitch),
            toDisplayAngle(xyz_rpy.yaw)
        };
        for (size_t i = 0; i < 6; ++i)
        {
            if (!spinboxes[i])
            {
                continue;
            }
            bool blocked = spinboxes[i]->blockSignals(true);
            spinboxes[i]->setValue(vals[i]);
            spinboxes[i]->blockSignals(blocked);
        }
    }

    JointControlPanel::PoseXyzRpy JointControlPanel::getArmRelativeSpinboxesRadians(
        const std::vector<std::unique_ptr<QDoubleSpinBox>>& spinboxes) const
    {
        PoseXyzRpy out;
        if (spinboxes.size() < 6)
        {
            return out;
        }
        out.x = fromDisplayLength(spinboxes[0] ? spinboxes[0]->value() : 0.0);
        out.y = fromDisplayLength(spinboxes[1] ? spinboxes[1]->value() : 0.0);
        out.z = fromDisplayLength(spinboxes[2] ? spinboxes[2]->value() : 0.0);
        out.roll = fromDisplayAngle(spinboxes[3] ? spinboxes[3]->value() : 0.0);
        out.pitch = fromDisplayAngle(spinboxes[4] ? spinboxes[4]->value() : 0.0);
        out.yaw = fromDisplayAngle(spinboxes[5] ? spinboxes[5]->value() : 0.0);
        return out;
    }

    void JointControlPanel::applyArmPoseUiMode(
        const std::string& side_prefix,
        std::vector<std::unique_ptr<QLabel>>& labels,
        std::vector<std::unique_ptr<QDoubleSpinBox>>& spinboxes,
        std::vector<std::unique_ptr<QVBoxLayout>>& layouts)
    {
        if (labels.size() < 7 || spinboxes.size() < 7 || layouts.size() < 7)
        {
            return;
        }

        static const char* kAbsNames[] = {"x", "y", "z", "qx", "qy", "qz", "qw"};
        static const char* kRelNames[] = {"x", "y", "z", "roll", "pitch", "yaw", "qw"};
        const char** names = use_relative_pose_ ? kRelNames : kAbsNames;

        for (size_t i = 0; i < 7; ++i)
        {
            if (labels[i])
            {
                labels[i]->setText(QString::fromStdString(side_prefix + " " + names[i]));
            }
            if (!spinboxes[i])
            {
                continue;
            }
            bool blocked = spinboxes[i]->blockSignals(true);
            if (i < 3)
            {
                spinboxes[i]->setSuffix(use_cm_deg_ ? " cm" : " m");
                spinboxes[i]->setDecimals(use_cm_deg_ ? 2 : 4);
                spinboxes[i]->setSingleStep(use_cm_deg_ ? 1.0 : 0.01);
            }
            else if (use_relative_pose_ && i < 6)
            {
                spinboxes[i]->setSuffix(use_cm_deg_ ? " deg" : " rad");
                spinboxes[i]->setDecimals(use_cm_deg_ ? 2 : 4);
                spinboxes[i]->setSingleStep(use_cm_deg_ ? 1.0 : 0.01);
            }
            else
            {
                spinboxes[i]->setSuffix("");
                spinboxes[i]->setDecimals(4);
                spinboxes[i]->setSingleStep(0.01);
            }
            spinboxes[i]->blockSignals(blocked);
        }

        // 相对模式隐藏 qw 行
        const bool show_qw = !use_relative_pose_;
        if (layouts[6])
        {
            for (int j = 0; j < layouts[6]->count(); ++j)
            {
                QLayoutItem* item = layouts[6]->itemAt(j);
                if (item && item->widget())
                {
                    item->widget()->setVisible(show_qw);
                }
            }
        }
    }

    void JointControlPanel::zeroArmPoseSpinboxes(
        std::vector<std::unique_ptr<QDoubleSpinBox>>& spinboxes)
    {
        if (use_relative_pose_)
        {
            setArmRelativeSpinboxes(spinboxes, PoseXyzRpy{});
            return;
        }
        geometry_msgs::msg::Pose identity;
        identity.orientation.w = 1.0;
        setArmPoseSpinboxesFromPose(spinboxes, identity);
    }

    void JointControlPanel::updatePoseModeControlsVisibility()
    {
        const bool show_pose_mode =
            is_joint_control_enabled_ &&
            current_command_ == 3 &&
            (current_category_ == "left" || current_category_ == "right");
        if (pose_mode_label_)
        {
            pose_mode_label_->setVisible(show_pose_mode);
        }
        if (pose_mode_combo_)
        {
            pose_mode_combo_->setVisible(show_pose_mode);
        }
        if (relative_keep_input_checkbox_)
        {
            relative_keep_input_checkbox_->setVisible(show_pose_mode && use_relative_pose_);
        }
    }

    void JointControlPanel::onPoseModeChanged()
    {
        if (!pose_mode_combo_)
        {
            return;
        }
        const std::string new_mode = pose_mode_combo_->currentData().toString().toStdString();
        if (new_mode == pose_mode_)
        {
            return;
        }
        pose_mode_ = new_mode;
        use_relative_pose_ = (pose_mode_ == "relative_base" || pose_mode_ == "relative_ee");

        applyArmPoseUiMode("Left", left_arm_labels_, left_arm_spinboxes_, left_arm_row_layouts_);
        applyArmPoseUiMode("Right", right_arm_labels_, right_arm_spinboxes_, right_arm_row_layouts_);

        if (use_relative_pose_)
        {
            zeroArmPoseSpinboxes(left_arm_spinboxes_);
            zeroArmPoseSpinboxes(right_arm_spinboxes_);
            refreshOcs2FrameParams();
        }
        else
        {
            if (left_current_pose_valid_)
            {
                setArmPoseSpinboxesFromPose(left_arm_spinboxes_, left_current_pose_);
            }
            if (right_current_pose_valid_)
            {
                setArmPoseSpinboxesFromPose(right_arm_spinboxes_, right_current_pose_);
            }
        }
        updatePoseModeControlsVisibility();
        updatePanelVisibility();
    }

    std::string JointControlPanel::getOcs2ControllerName() const
    {
        auto left_it = category_to_controller_.find("left");
        if (left_it != category_to_controller_.end() && !left_it->second.empty())
        {
            return left_it->second;
        }
        auto right_it = category_to_controller_.find("right");
        if (right_it != category_to_controller_.end() && !right_it->second.empty())
        {
            return right_it->second;
        }
        return {};
    }

    void JointControlPanel::refreshOcs2FrameParams()
    {
        if (!node_)
        {
            return;
        }
        const std::string controller = getOcs2ControllerName();
        if (controller.empty())
        {
            return;
        }
        try
        {
            auto temp_node = std::make_shared<rclcpp::Node>("ocs2_frame_param_checker");
            auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
                temp_node, "/" + controller);
            if (!param_client->wait_for_service(std::chrono::milliseconds(150)))
            {
                return;
            }
            std::lock_guard<std::mutex> lock(frame_id_mutex_);
            if (param_client->has_parameter("base_frame"))
            {
                ocs2_base_frame_ = param_client->get_parameter<std::string>("base_frame");
            }
            if (param_client->has_parameter("left_ee_frame"))
            {
                left_ee_frame_ = param_client->get_parameter<std::string>("left_ee_frame");
            }
            if (param_client->has_parameter("right_ee_frame"))
            {
                right_ee_frame_ = param_client->get_parameter<std::string>("right_ee_frame");
            }
            RCLCPP_INFO(node_->get_logger(),
                        "OCS2 frames from %s: base='%s' left_ee='%s' right_ee='%s'",
                        controller.c_str(), ocs2_base_frame_.c_str(),
                        left_ee_frame_.c_str(), right_ee_frame_.c_str());
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(node_->get_logger(), "Failed to read OCS2 frame params from %s: %s",
                        controller.c_str(), e.what());
        }
    }

    bool JointControlPanel::publishOcs2ArmPose(bool is_left)
    {
        auto& spinboxes = is_left ? left_arm_spinboxes_ : right_arm_spinboxes_;

        if (use_relative_pose_)
        {
            if (spinboxes.size() < 6)
            {
                return false;
            }
            auto& pub = is_left ? left_relative_publisher_ : right_relative_publisher_;
            if (!pub)
            {
                return false;
            }

            std::string frame_id;
            {
                std::lock_guard<std::mutex> lock(frame_id_mutex_);
                if (pose_mode_ == "relative_ee")
                {
                    frame_id = is_left ? left_ee_frame_ : right_ee_frame_;
                    if (frame_id.empty())
                    {
                        RCLCPP_WARN(node_->get_logger(),
                                    "相对末端需要 left/right_ee_frame 参数，请确认 OCS2 控制器已加载");
                        return false;
                    }
                }
                else
                {
                    // relative_base: prefer current_target frame, then controller base_frame
                    const std::string& target_frame =
                        is_left ? left_target_frame_id_ : right_target_frame_id_;
                    frame_id = !target_frame.empty()
                                   ? target_frame
                                   : (!ocs2_base_frame_.empty() ? ocs2_base_frame_ : "base_link");
                }
            }

            const PoseXyzRpy values = getArmRelativeSpinboxesRadians(spinboxes);
            geometry_msgs::msg::TwistStamped msg;
            msg.header.stamp = node_->get_clock()->now();
            msg.header.frame_id = frame_id;
            msg.twist.linear.x = values.x;
            msg.twist.linear.y = values.y;
            msg.twist.linear.z = values.z;
            msg.twist.angular.x = values.roll;
            msg.twist.angular.y = values.pitch;
            msg.twist.angular.z = values.yaw;
            pub->publish(msg);
            if (!relative_keep_input_)
            {
                zeroArmPoseSpinboxes(spinboxes);
            }
            return true;
        }

        if (spinboxes.size() < 7)
        {
            return false;
        }
        auto& pub = is_left ? left_target_publisher_ : right_target_publisher_;
        if (!pub)
        {
            return false;
        }
        geometry_msgs::msg::PoseStamped msg;
        {
            std::lock_guard<std::mutex> lock(frame_id_mutex_);
            const std::string& frame =
                is_left ? left_target_frame_id_ : right_target_frame_id_;
            msg.header.frame_id = frame.empty()
                                      ? (!ocs2_base_frame_.empty() ? ocs2_base_frame_ : "base_link")
                                      : frame;
        }
        msg.header.stamp = node_->get_clock()->now();
        msg.pose = getArmPoseFromSpinboxes(spinboxes);
        pub->publish(msg);
        return true;
    }

    bool JointControlPanel::refreshJointMetadataFromCache()
    {
        joint_types_ready_ = false;
        joint_metadata_status_.clear();
        if (!robot_description_received_ || robot_description_cache_.empty() ||
            !joint_limits_manager_ || !joints_initialized_ || joint_names_.empty())
        {
            updatePanelVisibility();
            return false;
        }

        joint_limits_manager_->parseFromURDF(
            robot_description_cache_, joint_names_, false);

        for (const auto& joint_name : joint_names_)
        {
            if (!joint_limits_manager_->hasJointMotionType(joint_name))
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "Cannot initialize unit for joint %s",
                             joint_name.c_str());
                joint_metadata_status_ =
                    QString("无法识别关节单位: %1")
                        .arg(QString::fromStdString(joint_name));
                updatePanelVisibility();
                return false;
            }
            if (joint_limits_manager_->isPrismaticJoint(joint_name) &&
                !joint_limits_manager_->hasLimits(joint_name))
            {
                RCLCPP_ERROR(node_->get_logger(),
                             "Prismatic joint %s has no position limits",
                             joint_name.c_str());
                joint_metadata_status_ =
                    QString("平移关节缺少限位: %1")
                        .arg(QString::fromStdString(joint_name));
                updatePanelVisibility();
                return false;
            }
        }

        joint_types_ready_ = true;
        applyDisplayUnitToJointSpinboxes();
        for (size_t i = 0;
             i < joint_positions_.size() && i < joint_spinboxes_.size(); ++i)
        {
            setJointSpinboxFromSiValue(i, joint_positions_[i]);
        }
        updatePanelVisibility();
        RCLCPP_INFO(node_->get_logger(),
                    "Joint types and limits loaded for %zu joints",
                    joint_names_.size());
        return true;
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

        // Filter and classify joints (empty category = gripper, skip)
        for (size_t i = 0; i < joint_names_source.size(); ++i)
        {
            const std::string& joint_name = joint_names_source[i];
            std::string category = classifyJoint(joint_name);
            if (category.empty())
            {
                continue;
            }

            size_t joint_index = joint_names_.size();
            joint_names_.push_back(joint_name);
            joint_name_to_index_[joint_name] = joint_index;
            joint_to_category_[joint_name] = category;
            category_to_joints_[category].push_back(joint_index);
        }

        // 单臂：无 left/right 前缀、无 body 控制器、但有 ocs2_arm/wbc → 归为 left（对接 left_target）。
        single_arm_mode_ = false;
        {
            std::string wbc_controller;
            std::string arm_controller;
            for (const auto& controller : available_controllers_)
            {
                std::string controller_lower = controller;
                std::transform(controller_lower.begin(), controller_lower.end(),
                               controller_lower.begin(), ::tolower);
                if (controller_lower.find("ocs2_wbc_controller") != std::string::npos)
                {
                    wbc_controller = controller;
                }
                else if (controller_lower.find("ocs2_arm_controller") != std::string::npos)
                {
                    arm_controller = controller;
                }
            }
            const bool has_pose_arm_controller = !wbc_controller.empty() || !arm_controller.empty();
            const bool has_body_controller =
                category_to_controller_.find("body") != category_to_controller_.end() &&
                !category_to_controller_["body"].empty();
            const bool has_left_joints =
                category_to_joints_.find("left") != category_to_joints_.end() &&
                !category_to_joints_["left"].empty();
            const bool has_right_joints =
                category_to_joints_.find("right") != category_to_joints_.end() &&
                !category_to_joints_["right"].empty();
            const bool has_default_body_joints =
                category_to_joints_.find("body") != category_to_joints_.end() &&
                !category_to_joints_["body"].empty();

            if (has_pose_arm_controller && !has_body_controller &&
                !has_left_joints && !has_right_joints && has_default_body_joints)
            {
                single_arm_mode_ = true;
                category_to_joints_["left"] = std::move(category_to_joints_["body"]);
                category_to_joints_.erase("body");
                for (auto& [joint_name, category] : joint_to_category_)
                {
                    if (category == "body")
                    {
                        category = "left";
                    }
                }
                RCLCPP_INFO(node_->get_logger(),
                            "Single-arm mode: remapped %zu joints to left",
                            category_to_joints_["left"].size());
            }
        }

        // 分类快照（含单臂重归类），后续重排后据此重建映射，避免再次 classifyJoint。
        const auto joint_categories_snapshot = joint_to_category_;

        joint_names_ = reorderJointsWithSortedDexterousHands(joint_names_, joint_categories_snapshot);

        // 仅人体 body 控制器：按 joints 参数重排；单臂已归 left，不会进入此分支。
        std::map<std::string, std::vector<std::string>> controller_joint_order;
        const bool has_body_joints = category_to_joints_.find("body") != category_to_joints_.end() &&
            !category_to_joints_["body"].empty();
        const bool has_body_controller =
            category_to_controller_.find("body") != category_to_controller_.end() &&
            !category_to_controller_["body"].empty();
        if (has_body_joints && has_body_controller)
        {
            const auto now = std::chrono::steady_clock::now();
            if (last_body_joint_order_attempt_.time_since_epoch().count() != 0 &&
                now - last_body_joint_order_attempt_ < std::chrono::milliseconds(500))
            {
                if (status_label_)
                {
                    status_label_->setText("等待 body 控制器 joints 参数...");
                }
                return;
            }
            last_body_joint_order_attempt_ = now;

            const auto body_order = getControllerJointOrderForCategory("body");
            if (body_order.empty())
            {
                if (status_label_)
                {
                    status_label_->setText("等待 body 控制器 joints 参数...");
                }
                return;
            }
            controller_joint_order["body"] = body_order;
            RCLCPP_INFO(node_->get_logger(),
                        "Body joints will be displayed using controller joints parameter order (%zu joints)",
                        body_order.size());
        }

        joint_names_ = reorderJointsByControllerOrder(
            joint_names_, joint_categories_snapshot, controller_joint_order);
        joint_name_to_index_.clear();
        joint_to_category_.clear();
        category_to_joints_.clear();
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            const std::string& joint_name = joint_names_[i];
            joint_name_to_index_[joint_name] = i;
            const auto cat_it = joint_categories_snapshot.find(joint_name);
            const std::string category =
                (cat_it != joint_categories_snapshot.end()) ? cat_it->second : classifyJoint(joint_name);
            joint_to_category_[joint_name] = category;
            category_to_joints_[category].push_back(i);
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

        // Only advertise/subscribe right_current_target when the robot has a right arm.
        if (has_right_joints)
        {
            if (!right_current_target_subscriber_)
            {
                right_current_target_subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
                    "right_current_target", 10,
                    std::bind(&JointControlPanel::onRightCurrentTargetReceived, this, std::placeholders::_1));
                RCLCPP_INFO(node_->get_logger(),
                            "Subscribed to right_current_target (right arm joints detected)");
            }
        }
        else
        {
            right_current_target_subscriber_.reset();
        }

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

        // Dual-arm / single-arm OCS2 UI: 绝对 xyz+四元数（7）；相对复用前 6 为 xyz+rpy
        if (is_dual_arm_mode)
        {
            const std::vector<std::string> param_names = {"x", "y", "z", "qx", "qy", "qz", "qw"};

            auto create_arm_pose_ui = [&](const std::string& side_prefix,
                                         std::vector<std::unique_ptr<QLabel>>& labels,
                                         std::vector<std::unique_ptr<QDoubleSpinBox>>& spinboxes,
                                         std::vector<std::unique_ptr<QVBoxLayout>>& layouts)
            {
                for (size_t i = 0; i < 7; ++i)
                {
                    auto row_layout = std::make_unique<QVBoxLayout>();
                    row_layout->setSpacing(2);

                    std::string label_text = side_prefix + " " + param_names[i];
                    auto label = std::make_unique<QLabel>(QString::fromStdString(label_text),
                                                          joint_control_group_.get());
                    label->setStyleSheet("QLabel { font-weight: bold; }");
                    row_layout->addWidget(label.get());
                    labels.push_back(std::move(label));

                    auto spinbox = std::make_unique<QDoubleSpinBox>(joint_control_group_.get());
                    configureArmPoseSpinbox(spinbox.get(), i);
                    row_layout->addWidget(spinbox.get());
                    spinboxes.push_back(std::move(spinbox));

                    joint_layout_->addLayout(row_layout.get());
                    layouts.push_back(std::move(row_layout));
                }
            };

            if (has_left_joints)
            {
                create_arm_pose_ui("Left", left_arm_labels_, left_arm_spinboxes_, left_arm_row_layouts_);
            }
            if (has_right_joints)
            {
                create_arm_pose_ui("Right", right_arm_labels_, right_arm_spinboxes_, right_arm_row_layouts_);
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

            // Create spinbox with dimension-neutral format until joint types are ready
            auto spinbox = std::make_unique<QDoubleSpinBox>(joint_control_group_.get());
            spinbox->setRange(-1000.0, 1000.0);
            spinbox->setSingleStep(0.01);
            spinbox->setDecimals(4);
            spinbox->setSuffix("");
            spinbox->setValue(0.0);
            row_layout->addWidget(spinbox.get());
            joint_spinboxes_.push_back(std::move(spinbox));

            joint_layout_->addLayout(row_layout.get());
            joint_row_layouts_.push_back(std::move(row_layout));
        }

        joints_initialized_ = true;
        joint_types_ready_ = false;
        if (status_label_)
        {
            status_label_->setText("请切换到支持关节控制的状态");
        }

        // Set joint names in limits manager
        if (joint_limits_manager_)
        {
            joint_limits_manager_->setJointNames(joint_names_);
        }

        // Parse joint types/limits from cached robot_description if available
        refreshJointMetadataFromCache();

        // left/right 分类延迟到关节名分类后添加
        std::string wbc_controller;
        std::string arm_controller;
        for (const auto& controller : available_controllers_)
        {
            std::string controller_lower = controller;
            std::transform(controller_lower.begin(), controller_lower.end(),
                           controller_lower.begin(), ::tolower);
            if (controller_lower.find("ocs2_wbc_controller") != std::string::npos)
            {
                wbc_controller = controller;
            }
            else if (controller_lower.find("ocs2_arm_controller") != std::string::npos)
            {
                arm_controller = controller;
            }
        }

        const std::string& pose_arm_controller =
            !wbc_controller.empty() ? wbc_controller : arm_controller;

        if (!pose_arm_controller.empty() && (has_left_joints || has_right_joints))
        {
            RCLCPP_INFO(node_->get_logger(),
                        "Arm categories: left=%s right=%s single_arm=%s controller=%s",
                        has_left_joints ? "yes" : "no",
                        has_right_joints ? "yes" : "no",
                        single_arm_mode_ ? "yes" : "no",
                        pose_arm_controller.c_str());
            if (has_left_joints)
            {
                available_categories_.insert("left");
                category_to_controller_["left"] = pose_arm_controller;
            }
            if (has_right_joints)
            {
                available_categories_.insert("right");
                category_to_controller_["right"] = pose_arm_controller;
            }
            updateCategoryOptions();

            if (single_arm_mode_ && category_combo_)
            {
                for (int i = 0; i < category_combo_->count(); ++i)
                {
                    if (category_combo_->itemData(i).toString().toStdString() == "left")
                    {
                        category_combo_->setCurrentIndex(i);
                        current_category_ = "left";
                        updatePublisher();
                        break;
                    }
                }
            }
        }
        else
        {
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
        Panel::load(config);

        QString unit;
        if (config.mapGetString("DisplayUnit", &unit) && display_unit_combo_)
        {
            // 兼容旧配置 AngleUnit: rad/deg
            if (unit == "rad")
            {
                unit = "m_rad";
            }
            else if (unit == "deg")
            {
                unit = "cm_deg";
            }
            const int index = display_unit_combo_->findData(unit);
            if (index >= 0)
            {
                const bool blocked = display_unit_combo_->blockSignals(true);
                display_unit_combo_->setCurrentIndex(index);
                display_unit_combo_->blockSignals(blocked);
                use_cm_deg_ = (unit == "cm_deg");
                if (joints_initialized_ && joint_types_ready_)
                {
                    applyDisplayUnitToJointSpinboxes();
                    for (size_t i = 0; i < joint_positions_.size(); ++i)
                    {
                        setJointSpinboxFromSiValue(i, joint_positions_[i]);
                    }
                }
                applyDisplayUnitToPoseSpinboxes(left_arm_spinboxes_);
                applyDisplayUnitToPoseSpinboxes(right_arm_spinboxes_);
            }
        }
        else if (config.mapGetString("AngleUnit", &unit) && display_unit_combo_)
        {
            const QString mapped = (unit == "deg") ? "cm_deg" : "m_rad";
            const int index = display_unit_combo_->findData(mapped);
            if (index >= 0)
            {
                const bool blocked = display_unit_combo_->blockSignals(true);
                display_unit_combo_->setCurrentIndex(index);
                display_unit_combo_->blockSignals(blocked);
                use_cm_deg_ = (mapped == "cm_deg");
                if (joints_initialized_ && joint_types_ready_)
                {
                    applyDisplayUnitToJointSpinboxes();
                    for (size_t i = 0; i < joint_positions_.size(); ++i)
                    {
                        setJointSpinboxFromSiValue(i, joint_positions_[i]);
                    }
                }
                applyDisplayUnitToPoseSpinboxes(left_arm_spinboxes_);
                applyDisplayUnitToPoseSpinboxes(right_arm_spinboxes_);
            }
        }

        int keep_input = 0;
        if (config.mapGetInt("RelativeKeepInput", &keep_input) && relative_keep_input_checkbox_)
        {
            relative_keep_input_ = (keep_input != 0);
            const bool blocked = relative_keep_input_checkbox_->blockSignals(true);
            relative_keep_input_checkbox_->setChecked(relative_keep_input_);
            relative_keep_input_checkbox_->blockSignals(blocked);
        }
    }

    void JointControlPanel::save(rviz_common::Config config) const
    {
        Panel::save(config);
        config.mapSetValue("DisplayUnit", use_cm_deg_ ? "cm_deg" : "m_rad");
        config.mapSetValue("RelativeKeepInput", relative_keep_input_ ? 1 : 0);
    }

    std::string JointControlPanel::getWaistControllerName() const
    {
        auto it = category_to_controller_.find("body");
        if (it != category_to_controller_.end() && !it->second.empty())
        {
            return it->second;
        }

        for (const auto& controller : available_controllers_)
        {
            std::string controller_lower = controller;
            std::transform(controller_lower.begin(), controller_lower.end(),
                           controller_lower.begin(), ::tolower);
            if (controller_lower.find("body") != std::string::npos &&
                controller_lower.find("ocs2_wbc_controller") == std::string::npos &&
                controller_lower.find("ocs2_arm_controller") == std::string::npos)
            {
                return controller;
            }
        }
        return {};
    }

    bool JointControlPanel::shouldShowWaistControls() const
    {
        if (!is_waist_enabled_ || current_category_ != "body")
        {
            return false;
        }

        std::string waist_controller = getWaistControllerName();
        std::string waist_controller_lower = waist_controller;
        std::transform(waist_controller_lower.begin(), waist_controller_lower.end(),
                       waist_controller_lower.begin(), ::tolower);

        // Dedicated body controller: show in both OCS2(3) and MOVEJ(4)
        if (waist_controller_lower.find("body_joint_controller") != std::string::npos)
        {
            return (current_command_ == 3 || current_command_ == 4);
        }

        // WBC controller: show only in MOVEJ(4)
        if (waist_controller_lower.find("ocs2_wbc_controller") != std::string::npos)
        {
            return (current_command_ == 4);
        }

        // Fallback: keep conservative behavior
        return (current_command_ == 4);
    }
} // namespace arms_rviz_control_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(arms_rviz_control_plugin::JointControlPanel, rviz_common::Panel)
