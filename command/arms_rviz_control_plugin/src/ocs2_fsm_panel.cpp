#include "arms_rviz_control_plugin/ocs2_fsm_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <QTimer>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <memory>
#include <functional>
#include <arms_controller_common/utils/FSMStateTransitionValidator.h>

namespace arms_rviz_control_plugin
{
    OCS2FSMPanel::OCS2FSMPanel(QWidget* parent)
        : Panel(parent)
    {
        // Create main layout
        auto* main_layout = new QVBoxLayout(this);
        setFixedSize(350, 180);

        // Title with status indicator - centered
        auto* title_layout = new QHBoxLayout();
        auto* title_label = new QLabel("Current FSM", this);
        title_label->setStyleSheet("QLabel { font-weight: bold; font-size: 14px; }");

        // Small status indicator
        current_state_label_ = std::make_unique<QLabel>("HOLD", this);
        current_state_label_->setStyleSheet(
            "QLabel { padding: 1px 4px; background-color: #e3f2fd; border: 1px solid #2196F3; "
            "font-weight: bold; font-size: 9px; border-radius: 2px; margin: 0px; }");
        current_state_label_->setFixedHeight(16);
        current_state_label_->setAlignment(Qt::AlignCenter);

        title_layout->addStretch();
        title_layout->addWidget(title_label);
        title_layout->addWidget(current_state_label_.get());
        title_layout->addStretch();
        main_layout->addLayout(title_layout);

        // State transition buttons group
        button_group_ = std::make_unique<QGroupBox>(this);
        auto* button_layout = new QVBoxLayout(button_group_.get());
        button_layout->setSpacing(2);

        // HOME → HOLD (command = 2)
        home_to_hold_btn_ = std::make_unique<QPushButton>("HOLD", this);
        home_to_hold_btn_->setStyleSheet("QPushButton { background-color: #FF9800; color: white; font-weight: bold; }");
        button_layout->addWidget(home_to_hold_btn_.get());

        // Switch pose button (command = 4) - 只在HOME状态显示
        switch_pose_btn_ = std::make_unique<QPushButton>("切换姿态 (Home ↔ Rest)", this);
        switch_pose_btn_->setStyleSheet("QPushButton { background-color: #FF5722; color: white; font-weight: bold; }");
        button_layout->addWidget(switch_pose_btn_.get());

        // HOLD → OCS2 和 HOLD → MOVEJ 同一行
        auto* hold_to_row = new QHBoxLayout();
        hold_to_row->setSpacing(2);
        hold_to_ocs2_btn_ = std::make_unique<QPushButton>("OCS2", this);
        hold_to_ocs2_btn_->setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; }");
        hold_to_row->addWidget(hold_to_ocs2_btn_.get());
        hold_to_movej_btn_ = std::make_unique<QPushButton>("MOVEJ", this);
        hold_to_movej_btn_->setStyleSheet("QPushButton { background-color: #009688; color: white; font-weight: bold; }");
        hold_to_row->addWidget(hold_to_movej_btn_.get());
        button_layout->addLayout(hold_to_row);

        // OCS2 → HOLD 和 MOVEJ → HOLD 同一行
        auto* to_hold_row = new QHBoxLayout();
        to_hold_row->setSpacing(2);
        ocs2_to_hold_btn_ = std::make_unique<QPushButton>("HOLD", this);
        ocs2_to_hold_btn_->setStyleSheet("QPushButton { background-color: #FF9800; color: white; font-weight: bold; }");
        to_hold_row->addWidget(ocs2_to_hold_btn_.get());
        movej_to_hold_btn_ = std::make_unique<QPushButton>("HOLD", this);
        movej_to_hold_btn_->setStyleSheet("QPushButton { background-color: #FF9800; color: white; font-weight: bold; }");
        to_hold_row->addWidget(movej_to_hold_btn_.get());
        button_layout->addLayout(to_hold_row);

        // HOLD → HOME (command = 1)
        hold_to_home_btn_ = std::make_unique<QPushButton>("HOME", this);
        hold_to_home_btn_->setStyleSheet("QPushButton { background-color: #9C27B0; color: white; font-weight: bold; }");
        button_layout->addWidget(hold_to_home_btn_.get());

        main_layout->addWidget(button_group_.get());

        // ==================== WBC Control Section (OCS2 mode only) ====================
        wbc_container_ = std::make_unique<QWidget>(this);
        wbc_layout_ = std::make_unique<QVBoxLayout>(wbc_container_.get());
        wbc_layout_->setContentsMargins(0, 8, 0, 0);
        wbc_layout_->setSpacing(4);

        // Add separator line
        auto* separator = new QFrame(this);
        separator->setFrameShape(QFrame::HLine);
        separator->setFrameShadow(QFrame::Sunken);
        wbc_layout_->addWidget(separator);

        // WBC title
        auto* wbc_title = new QLabel("WBC 控制器设置 (OCS2模式)", this);
        wbc_title->setStyleSheet("QLabel { font-weight: bold; font-size: 12px; color: #2196F3; }");
        wbc_layout_->addWidget(wbc_title);

        // Upper button layout (grid)
        upper_button_layout_ = std::make_unique<QGridLayout>();
        upper_button_layout_->setHorizontalSpacing(12);
        upper_button_layout_->setVerticalSpacing(2);

        const int label_width = 74;
        const QString label_style = "QLabel { font-size: 13px; padding: 0px; }";

        // Base
        base_label_ = std::make_unique<QLabel>("启用底盘:", this);
        base_label_->setFixedWidth(label_width);
        base_label_->setStyleSheet(label_style);
        base_switch_ = std::make_unique<SwitchButton>(this);

        auto* base_layout = new QHBoxLayout();
        base_layout->setContentsMargins(0, 0, 0, 0);
        base_layout->setSpacing(4);
        base_layout->addWidget(base_label_.get());
        base_layout->addWidget(base_switch_.get());
        base_layout->addStretch();

        auto* base_widget = new QWidget(this);
        base_widget->setLayout(base_layout);
        upper_button_layout_->addWidget(base_widget, 0, 0);

        // Bimanual
        bimanual_label_ = std::make_unique<QLabel>("双臂耦合:", this);
        bimanual_label_->setFixedWidth(label_width);
        bimanual_label_->setStyleSheet(label_style);
        bimanual_switch_ = std::make_unique<SwitchButton>(this);

        auto* bimanual_layout = new QHBoxLayout();
        bimanual_layout->setContentsMargins(0, 0, 0, 0);
        bimanual_layout->setSpacing(4);
        bimanual_layout->addWidget(bimanual_label_.get());
        bimanual_layout->addWidget(bimanual_switch_.get());
        bimanual_layout->addStretch();

        auto* bimanual_widget = new QWidget(this);
        bimanual_widget->setLayout(bimanual_layout);
        upper_button_layout_->addWidget(bimanual_widget, 0, 1);

        // Left arm
        left_arm_label_ = std::make_unique<QLabel>("启用左臂:", this);
        left_arm_label_->setFixedWidth(label_width);
        left_arm_label_->setStyleSheet(label_style);
        left_arm_switch_ = std::make_unique<SwitchButton>(this);

        auto* left_arm_layout = new QHBoxLayout();
        left_arm_layout->setContentsMargins(0, 0, 0, 0);
        left_arm_layout->setSpacing(4);
        left_arm_layout->addWidget(left_arm_label_.get());
        left_arm_layout->addWidget(left_arm_switch_.get());
        left_arm_layout->addStretch();

        auto* left_arm_widget = new QWidget(this);
        left_arm_widget->setLayout(left_arm_layout);
        upper_button_layout_->addWidget(left_arm_widget, 1, 0);

        // Right arm
        right_arm_label_ = std::make_unique<QLabel>("启用右臂:", this);
        right_arm_label_->setFixedWidth(label_width);
        right_arm_label_->setStyleSheet(label_style);
        right_arm_switch_ = std::make_unique<SwitchButton>(this);

        auto* right_arm_layout = new QHBoxLayout();
        right_arm_layout->setContentsMargins(0, 0, 0, 0);
        right_arm_layout->setSpacing(4);
        right_arm_layout->addWidget(right_arm_label_.get());
        right_arm_layout->addWidget(right_arm_switch_.get());
        right_arm_layout->addStretch();

        auto* right_arm_widget = new QWidget(this);
        right_arm_widget->setLayout(right_arm_layout);
        upper_button_layout_->addWidget(right_arm_widget, 1, 1);

        wbc_layout_->addLayout(upper_button_layout_.get());

        // Body mode row with OCS2 to HOLD button on the right
        auto* body_row_widget = new QWidget(this);
        body_control_layout_ = std::make_unique<QHBoxLayout>(body_row_widget);
        body_control_layout_->setContentsMargins(0, 0, 0, 0);
        body_control_layout_->setSpacing(8);

        body_label_ = std::make_unique<QLabel>("身体模式:", this);
        body_label_->setFixedWidth(label_width);
        body_label_->setStyleSheet(label_style);

        body_combo_box_ = std::make_unique<QComboBox>(this);
        body_combo_box_->setFixedHeight(28);
        body_combo_box_->setMinimumWidth(92);
        body_combo_box_->setStyleSheet(R"(
            QComboBox {
                font-size: 13px;
                padding: 2px 22px 2px 8px;
                border: 1px solid #bdbdbd;
                border-radius: 6px;
                background: white;
            }
            QComboBox:hover {
                border: 1px solid #9e9e9e;
            }
            QComboBox:focus {
                border: 1px solid #66bb6a;
            }
            QComboBox::drop-down {
                subcontrol-origin: padding;
                subcontrol-position: top right;
                width: 18px;
                border: none;
            }
            QComboBox QAbstractItemView {
                font-size: 13px;
                border: 1px solid #cfcfcf;
                selection-background-color: #e8f5e9;
                selection-color: black;
                outline: none;
            }
        )");

        body_combo_box_->addItem("自由", BODY_MODE_FREE);
        body_combo_box_->addItem("竖直", BODY_MODE_VERTICAL);
        body_combo_box_->addItem("跟随", BODY_MODE_TRACKING);
        body_combo_box_->addItem("锁定", BODY_MODE_LOCKED);

        // Add OCS2 to HOLD button on the right of body mode dropdown
        auto* ocs2_to_hold_wbc_btn = new QPushButton("HOLD", this);
        ocs2_to_hold_wbc_btn->setStyleSheet("QPushButton { background-color: #FF9800; color: white; font-weight: bold; min-width: 130px; }");
        ocs2_to_hold_wbc_btn->setFixedHeight(28);
        ocs2_to_hold_wbc_btn->setMinimumWidth(130);

        body_control_layout_->addWidget(body_label_.get());
        body_control_layout_->addWidget(body_combo_box_.get());
        body_control_layout_->addWidget(ocs2_to_hold_wbc_btn);
        body_control_layout_->addStretch();

        wbc_layout_->addWidget(body_row_widget);

        main_layout->addWidget(wbc_container_.get());

        // Connect signals
        connect(home_to_hold_btn_.get(), &QPushButton::clicked, this, &OCS2FSMPanel::onHomeToHold);
        connect(hold_to_ocs2_btn_.get(), &QPushButton::clicked, this, &OCS2FSMPanel::onHoldToOCS2);
        connect(ocs2_to_hold_btn_.get(), &QPushButton::clicked, this, &OCS2FSMPanel::onOCS2ToHold);
        connect(hold_to_movej_btn_.get(), &QPushButton::clicked, this, &OCS2FSMPanel::onHoldToMoveJ);
        connect(movej_to_hold_btn_.get(), &QPushButton::clicked, this, &OCS2FSMPanel::onMoveJToHold);
        connect(hold_to_home_btn_.get(), &QPushButton::clicked, this, &OCS2FSMPanel::onHoldToHome);
        connect(switch_pose_btn_.get(), &QPushButton::clicked, this, &OCS2FSMPanel::onSwitchPose);

        // Connect WBC signals
        connect(base_switch_.get(), &SwitchButton::clicked, this, &OCS2FSMPanel::onBaseToggled);
        connect(bimanual_switch_.get(), &SwitchButton::clicked, this, &OCS2FSMPanel::onBimanualToggled);
        connect(left_arm_switch_.get(), &SwitchButton::clicked, this, &OCS2FSMPanel::onLeftArmToggled);
        connect(right_arm_switch_.get(), &SwitchButton::clicked, this, &OCS2FSMPanel::onRightArmToggled);
        connect(body_combo_box_.get(), QOverload<int>::of(&QComboBox::currentIndexChanged), this, &OCS2FSMPanel::onBodyModeChanged);
        connect(ocs2_to_hold_wbc_btn, &QPushButton::clicked, this, &OCS2FSMPanel::onOCS2ToHold);

        // Initialize button visibility (initial state is HOLD)
        updateButtonVisibility();

        // Initially hide WBC container (only shown in OCS2 mode)
        wbc_container_->setVisible(false);

        // Create reset timer
        reset_timer_ = std::make_unique<QTimer>(this);
        reset_timer_->setSingleShot(true);
        reset_timer_->setInterval(100);
        connect(reset_timer_.get(), &QTimer::timeout, this, &OCS2FSMPanel::onSwitchPoseReleased);
    }

    OCS2FSMPanel::~OCS2FSMPanel() = default;

    void OCS2FSMPanel::onInitialize()
    {
        // Use RViz display context to get the node
        node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        // Check if ocs2_wbc_controller is available
        if (!node_->has_parameter("wbc_available")) {
            node_->declare_parameter("wbc_available", false);
        }
        wbc_available_ = node_->get_parameter("wbc_available").as_bool();

        if (!wbc_available_)
        {
            RCLCPP_INFO(node_->get_logger(), "ocs2_wbc_controller not found, WBC controls will be hidden");
            // Hide WBC container if not available
            if (wbc_container_)
            {
                wbc_container_->setVisible(false);
            }
        }

        // Create FSM command publisher
        fsm_command_publisher_ = node_->create_publisher<std_msgs::msg::Int32>(
            "/fsm_command", 10);

        // Create subscriber to listen for FSM command changes
        fsm_command_subscriber_ = node_->create_subscription<std_msgs::msg::Int32>(
            "/fsm_command", 10,
            std::bind(&OCS2FSMPanel::onFsmCommandReceived, this, std::placeholders::_1));

        // Only create WBC related ROS2 interfaces if controller is available
        if (wbc_available_)
        {
            // Create WBC mode command publisher
            mode_command_pub_ = node_->create_publisher<std_msgs::msg::String>(
                "mode_command", 10);

            // Create WBC subscribers
            capability_sub_ = node_->create_subscription<arms_ros2_control_msgs::msg::WbcCapability>(
                "/ocs2_wbc_controller/wbc_capabilities",
                rclcpp::QoS(1).transient_local(),
                std::bind(&OCS2FSMPanel::onReceiveCapability, this, std::placeholders::_1));

            state_sub_ = node_->create_subscription<arms_ros2_control_msgs::msg::WbcCurrentState>(
                "/ocs2_wbc_controller/current_state",
                10,
                std::bind(&OCS2FSMPanel::onReceiveCurrentState, this, std::placeholders::_1));
    }

    RCLCPP_INFO(node_->get_logger(), "OCS2 FSM Panel initialized with /fsm_command topic");
    }

    void OCS2FSMPanel::onHomeToHold()
    {
        publishCommand(2);
        setCurrentState("HOLD");
    }

    void OCS2FSMPanel::onHoldToOCS2()
    {
        publishCommand(3);
        setCurrentState("OCS2");
    }

    void OCS2FSMPanel::onOCS2ToHold()
    {
        publishCommand(2);
        setCurrentState("HOLD");
    }

    void OCS2FSMPanel::onHoldToHome()
    {
        publishCommand(1);
        setCurrentState("HOME");
    }

    void OCS2FSMPanel::onHoldToMoveJ()
    {
        publishCommand(4);
        setCurrentState("MOVEJ");
    }

    void OCS2FSMPanel::onMoveJToHold()
    {
        publishCommand(2);
        setCurrentState("HOLD");
    }

    void OCS2FSMPanel::onSwitchPose()
    {
        if (!switch_pose_pressed_)
        {
            publishCommand(100);
            switch_pose_pressed_ = true;
            reset_timer_->start();
        }
    }

    void OCS2FSMPanel::onSwitchPoseReleased()
    {
        publishCommand(0);
        switch_pose_pressed_ = false;
    }

    void OCS2FSMPanel::publishCommand(int32_t command)
    {
        auto msg = std_msgs::msg::Int32();
        msg.data = command;
        fsm_command_publisher_->publish(msg);
        current_command_ = command;
        RCLCPP_INFO(node_->get_logger(), "Published FSM command: %d", command);
    }

    void OCS2FSMPanel::updateStatusDisplay()
    {
        updateButtonVisibility();
    }

    void OCS2FSMPanel::updateButtonVisibility()
    {
        // Update FSM buttons visibility based on current state
        if (current_state_ == "HOME")
        {
            // Show button group
            button_group_->setVisible(true);

            hold_to_ocs2_btn_->setVisible(false);
            hold_to_movej_btn_->setVisible(false);
            ocs2_to_hold_btn_->setVisible(false);
            movej_to_hold_btn_->setVisible(false);
            hold_to_home_btn_->setVisible(false);
            home_to_hold_btn_->setVisible(true);
            home_to_hold_btn_->setEnabled(true);
            switch_pose_btn_->setVisible(true);
            switch_pose_btn_->setEnabled(true);

            // Hide WBC controls in HOME mode (if available)
            if (wbc_container_)
            {
                wbc_container_->setVisible(false);
            }
        }
        else if (current_state_ == "HOLD")
        {
            // Show button group
            button_group_->setVisible(true);

            home_to_hold_btn_->setVisible(false);
            switch_pose_btn_->setVisible(false);
            ocs2_to_hold_btn_->setVisible(false);
            movej_to_hold_btn_->setVisible(false);
            hold_to_home_btn_->setVisible(true);
            hold_to_home_btn_->setEnabled(true);
            hold_to_ocs2_btn_->setVisible(true);
            hold_to_ocs2_btn_->setEnabled(true);
            hold_to_movej_btn_->setVisible(true);
            hold_to_movej_btn_->setEnabled(true);

            // Hide WBC controls in HOLD mode (if available)
            if (wbc_container_)
            {
                wbc_container_->setVisible(false);
            }
        }
        else if (current_state_ == "OCS2")
        {
            if (wbc_available_)
            {
                // WBC模式：隐藏原按钮组，显示WBC控件
                button_group_->setVisible(false);

                if (wbc_container_)
                {
                    wbc_container_->setVisible(true);
                }
                refreshWbcUi();
            }
            else
            {
                // 非WBC模式：仍显示原按钮组，但只保留 OCS2 -> HOLD 按钮
                button_group_->setVisible(true);

                home_to_hold_btn_->setVisible(false);
                switch_pose_btn_->setVisible(false);
                hold_to_ocs2_btn_->setVisible(false);
                hold_to_movej_btn_->setVisible(false);
                movej_to_hold_btn_->setVisible(false);
                hold_to_home_btn_->setVisible(false);

                ocs2_to_hold_btn_->setVisible(true);
                ocs2_to_hold_btn_->setEnabled(true);

                if (wbc_container_)
                {
                    wbc_container_->setVisible(false);
                }
            }
        }
        else if (current_state_ == "MOVEJ")
        {
            // Show button group
            button_group_->setVisible(true);

            home_to_hold_btn_->setVisible(false);
            switch_pose_btn_->setVisible(false);
            hold_to_ocs2_btn_->setVisible(false);
            hold_to_home_btn_->setVisible(false);
            ocs2_to_hold_btn_->setVisible(false);
            hold_to_movej_btn_->setVisible(false);
            movej_to_hold_btn_->setVisible(true);
            movej_to_hold_btn_->setEnabled(true);

            // Hide WBC controls in MOVEJ mode
            if (wbc_container_)
            {
                wbc_container_->setVisible(false);
            }
        }
        else
        {
            // Show button group but hide all buttons
            button_group_->setVisible(true);

            home_to_hold_btn_->setVisible(false);
            switch_pose_btn_->setVisible(false);
            hold_to_ocs2_btn_->setVisible(false);
            hold_to_movej_btn_->setVisible(false);
            ocs2_to_hold_btn_->setVisible(false);
            movej_to_hold_btn_->setVisible(false);
            hold_to_home_btn_->setVisible(false);

            if (wbc_container_)
            {
                wbc_container_->setVisible(false);
            }
        }
    }

    void OCS2FSMPanel::setCurrentState(const std::string& state)
    {
        current_state_ = state;
        current_state_label_->setText(QString::fromStdString(state));
        updateButtonVisibility();
    }

    void OCS2FSMPanel::load(const rviz_common::Config& config)
    {
        Panel::load(config);
    }

    void OCS2FSMPanel::save(rviz_common::Config config) const
    {
        Panel::save(config);
    }

    void OCS2FSMPanel::onFsmCommandReceived(const std_msgs::msg::Int32::SharedPtr msg)
    {
        std::string new_state;
        bool valid_transition = arms_controller_common::FSMStateTransitionValidator::validateTransition(
            current_state_, msg->data, new_state);

        if (valid_transition)
        {
            setCurrentState(new_state);
            current_command_ = msg->data;
        }
    }

    // ==================== WBC Control Methods ====================

    void OCS2FSMPanel::onReceiveCapability(const arms_ros2_control_msgs::msg::WbcCapability::SharedPtr msg)
    {
        if (!msg) return;

        capability_state_.has_mobile_base = msg->has_mobile_base;
        capability_state_.has_body_relative_constraint = msg->has_body_relative_constraint;
        capability_state_.has_waist_lock = msg->has_waist_lock;
        capability_state_.has_head_coupling = msg->has_head_coupling;
        capability_state_.has_bimanual_coupling = msg->has_bimanual_coupling;
        capability_state_.body_tracking_ee_enabled = msg->body_tracking_ee_enabled;

        QMetaObject::invokeMethod(this, [this]() { refreshWbcUi(); }, Qt::QueuedConnection);
    }

    void OCS2FSMPanel::onReceiveCurrentState(const arms_ros2_control_msgs::msg::WbcCurrentState::SharedPtr msg)
    {
        if (!msg) return;

        current_wbc_state_.base_state = msg->base_state;
        current_wbc_state_.body_state = msg->body_state;
        current_wbc_state_.bimanual_state = msg->bimanual_state;
        current_wbc_state_.left_arm_state = msg->left_arm_state;
        current_wbc_state_.right_arm_state = msg->right_arm_state;

        QMetaObject::invokeMethod(this, [this]() { refreshWbcUi(); }, Qt::QueuedConnection);
    }

    bool OCS2FSMPanel::isBaseLocked() const
    {
        return current_wbc_state_.base_state == 0; // BASE_LOCKED
    }

    bool OCS2FSMPanel::isBimanualCoupled() const
    {
        return current_wbc_state_.bimanual_state == 1; // BIMANUAL_COUPLED
    }

    bool OCS2FSMPanel::isLeftArmEnabled() const
    {
        return current_wbc_state_.left_arm_state == 1; // ARM_ENABLED
    }

    bool OCS2FSMPanel::isRightArmEnabled() const
    {
        return current_wbc_state_.right_arm_state == 1; // ARM_ENABLED
    }

    bool OCS2FSMPanel::isBodyFree() const
    {
        return current_wbc_state_.body_state == 0; // BODY_FREE
    }

    bool OCS2FSMPanel::isBodyVertical() const
    {
        return current_wbc_state_.body_state == 1; // BODY_VERTICAL
    }

    bool OCS2FSMPanel::isBodyTracking() const
    {
        return current_wbc_state_.body_state == 2; // BODY_TRACKING
    }

    bool OCS2FSMPanel::isBodyLocked() const
    {
        return current_wbc_state_.body_state == 3; // BODY_LOCKED
    }

    bool OCS2FSMPanel::isBodyHeadCoupled() const
    {
        return current_wbc_state_.body_state == 4; // BODY_HEAD_COUPLED
    }

    int OCS2FSMPanel::getCurrentBodyModeIndex() const
    {
        if (isBodyFree()) return BODY_MODE_FREE;
        if (isBodyVertical()) return BODY_MODE_VERTICAL;
        if (isBodyTracking()) return BODY_MODE_TRACKING;
        if (isBodyLocked()) return BODY_MODE_LOCKED;
        if (isBodyHeadCoupled()) return BODY_MODE_HEAD_COUPLED;
        return BODY_MODE_LOCKED;
    }

    QString OCS2FSMPanel::getBodyModeCommand(int modeIndex) const
    {
        switch (modeIndex)
        {
            case BODY_MODE_FREE:
                return "BODY_FREE";
            case BODY_MODE_VERTICAL:
                return "BODY_RELATIVE";
            case BODY_MODE_TRACKING:
                return "BODY_TRACKING";
            case BODY_MODE_LOCKED:
                return "BODY_LOCK";
            case BODY_MODE_HEAD_COUPLED:
                return "BODY_HEAD_COUPLED";
            default:
                return "";
        }
    }

    void OCS2FSMPanel::updateBodyComboBox()
    {
        if (!body_combo_box_) return;

        body_combo_box_->blockSignals(true);
        body_combo_box_->clear();

        body_combo_box_->addItem("自由", BODY_MODE_FREE);

        if (capability_state_.has_body_relative_constraint)
        {
            body_combo_box_->addItem("竖直", BODY_MODE_VERTICAL);
        }

        if (capability_state_.body_tracking_ee_enabled)
        {
            body_combo_box_->addItem("跟随", BODY_MODE_TRACKING);
        }

        if (capability_state_.has_waist_lock)
        {
            body_combo_box_->addItem("锁定", BODY_MODE_LOCKED);
        }

        if (capability_state_.has_head_coupling)
        {
            body_combo_box_->addItem("锁头", BODY_MODE_HEAD_COUPLED);
        }

        int current_mode = getCurrentBodyModeIndex();
        int index = body_combo_box_->findData(current_mode);
        if (index >= 0)
        {
            body_combo_box_->setCurrentIndex(index);
        }

        body_combo_box_->blockSignals(false);
        body_combo_box_->setEnabled(body_combo_box_->count() > 1);
    }

    void OCS2FSMPanel::updateSwitchVisualState(SwitchButton* sw, bool capability_available, bool logical_on)
    {
        if (!sw) return;

        if (!capability_available)
        {
            sw->setVisualState(SwitchButton::VisualState::InvalidOffGray);
            sw->setClickable(false);
            return;
        }

        sw->setClickable(true);

        if (logical_on)
        {
            sw->setVisualState(SwitchButton::VisualState::EnabledOnGreen);
        }
        else
        {
            sw->setVisualState(SwitchButton::VisualState::DisabledOffRed);
        }
    }

    void OCS2FSMPanel::refreshWbcUi()
    {
        if (!wbc_container_ || !wbc_container_->isVisible() || !wbc_available_) return;

        updateSwitchVisualState(base_switch_.get(),
                                capability_state_.has_mobile_base,
                                !isBaseLocked());

        updateSwitchVisualState(bimanual_switch_.get(),
                                capability_state_.has_bimanual_coupling,
                                isBimanualCoupled());

        updateSwitchVisualState(left_arm_switch_.get(),
                                true,
                                isLeftArmEnabled());

        updateSwitchVisualState(right_arm_switch_.get(),
                                true,
                                isRightArmEnabled());

        updateBodyComboBox();
    }

    void OCS2FSMPanel::publishModeCommand(const std::string& cmd)
    {
        if (!mode_command_pub_) return;

        std_msgs::msg::String msg;
        msg.data = cmd;
        mode_command_pub_->publish(msg);
        RCLCPP_INFO(node_->get_logger(), "Published mode command: %s", cmd.c_str());
    }

    void OCS2FSMPanel::onBaseToggled()
    {
        if (!capability_state_.has_mobile_base) return;
        publishModeCommand(isBaseLocked() ? "BASE_UNLOCK" : "BASE_LOCK");
    }

    void OCS2FSMPanel::onBimanualToggled()
    {
        if (!capability_state_.has_bimanual_coupling) return;
        publishModeCommand(isBimanualCoupled() ? "ARMS_INDEPENDENT" : "ARMS_COUPLED");
    }

    void OCS2FSMPanel::onLeftArmToggled()
    {
        publishModeCommand(isLeftArmEnabled() ? "LEFT_ARM_DISABLE" : "LEFT_ARM_ENABLE");
    }

    void OCS2FSMPanel::onRightArmToggled()
    {
        publishModeCommand(isRightArmEnabled() ? "RIGHT_ARM_DISABLE" : "RIGHT_ARM_ENABLE");
    }

    void OCS2FSMPanel::onBodyModeChanged(int index)
    {
        if (index < 0) return;

        int mode = body_combo_box_->itemData(index).toInt();
        QString command = getBodyModeCommand(mode);

        if (command.isEmpty()) return;

        int current_mode = getCurrentBodyModeIndex();
        if (mode != current_mode)
        {
            publishModeCommand(command.toStdString());
        }
    }

} // namespace arms_rviz_control_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(arms_rviz_control_plugin::OCS2FSMPanel, rviz_common::Panel)