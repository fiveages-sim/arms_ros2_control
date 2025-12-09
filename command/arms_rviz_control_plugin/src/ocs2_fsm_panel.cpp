#include "arms_rviz_control_plugin/ocs2_fsm_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <QTimer>
#include <QHBoxLayout>
#include <memory>
#include <functional>

namespace arms_rviz_control_plugin
{
    OCS2FSMPanel::OCS2FSMPanel(QWidget* parent)
        : Panel(parent)
    {
        // Create UI layout
        auto* main_layout = new QVBoxLayout(this);

        // Title with status indicator - centered
        auto* title_layout = new QHBoxLayout();
        auto* title_label = new QLabel("Current FSM", this);
        title_label->setStyleSheet("QLabel { font-weight: bold; font-size: 14px; }");
        
        // Small status indicator
        current_state_label_ = std::make_unique<QLabel>("HOLD", this);
        current_state_label_->setStyleSheet(
            "QLabel { padding: 1px 4px; background-color: #e3f2fd; border: 1px solid #2196F3; "
            "font-weight: bold; font-size: 9px; border-radius: 2px; margin: 0px; }");
        current_state_label_->setFixedHeight(16); // 设置固定高度
        current_state_label_->setAlignment(Qt::AlignCenter); // 文字居中
        
        title_layout->addStretch(); // 左侧空白
        title_layout->addWidget(title_label);
        title_layout->addWidget(current_state_label_.get());
        title_layout->addStretch(); // 右侧空白
        main_layout->addLayout(title_layout);

        // State transition buttons
        auto* button_group = new QGroupBox(this);
        auto* button_layout = new QVBoxLayout(button_group);
        button_layout->setSpacing(2); // 减少按钮间距

        // HOME → HOLD (command = 2) - 修正command值
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
        // HOLD → OCS2 (command = 3)
        hold_to_ocs2_btn_ = std::make_unique<QPushButton>("OCS2", this);
        hold_to_ocs2_btn_->setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; }");
        hold_to_row->addWidget(hold_to_ocs2_btn_.get());
        // HOLD → MOVEJ (command = 4)
        hold_to_movej_btn_ = std::make_unique<QPushButton>("MOVEJ", this);
        hold_to_movej_btn_->setStyleSheet("QPushButton { background-color: #009688; color: white; font-weight: bold; }");
        hold_to_row->addWidget(hold_to_movej_btn_.get());
        button_layout->addLayout(hold_to_row);

        // OCS2 → HOLD 和 MOVEJ → HOLD 同一行
        auto* to_hold_row = new QHBoxLayout();
        to_hold_row->setSpacing(2);
        // OCS2 → HOLD (command = 2)
        ocs2_to_hold_btn_ = std::make_unique<QPushButton>("HOLD", this);
        ocs2_to_hold_btn_->setStyleSheet("QPushButton { background-color: #FF9800; color: white; font-weight: bold; }");
        to_hold_row->addWidget(ocs2_to_hold_btn_.get());
        // MOVEJ → HOLD (command = 2)
        movej_to_hold_btn_ = std::make_unique<QPushButton>("HOLD", this);
        movej_to_hold_btn_->setStyleSheet("QPushButton { background-color: #FF9800; color: white; font-weight: bold; }");
        to_hold_row->addWidget(movej_to_hold_btn_.get());
        button_layout->addLayout(to_hold_row);

        // HOLD → HOME (command = 1)
        hold_to_home_btn_ = std::make_unique<QPushButton>("HOME", this);
        hold_to_home_btn_->setStyleSheet("QPushButton { background-color: #9C27B0; color: white; font-weight: bold; }");
        button_layout->addWidget(hold_to_home_btn_.get());

        main_layout->addWidget(button_group);

        // Connect signals
        connect(home_to_hold_btn_.get(), &QPushButton::clicked, this, &OCS2FSMPanel::onHomeToHold);
        connect(hold_to_ocs2_btn_.get(), &QPushButton::clicked, this, &OCS2FSMPanel::onHoldToOCS2);
        connect(ocs2_to_hold_btn_.get(), &QPushButton::clicked, this, &OCS2FSMPanel::onOCS2ToHold);
        connect(hold_to_movej_btn_.get(), &QPushButton::clicked, this, &OCS2FSMPanel::onHoldToMoveJ);
        connect(movej_to_hold_btn_.get(), &QPushButton::clicked, this, &OCS2FSMPanel::onMoveJToHold);
        connect(hold_to_home_btn_.get(), &QPushButton::clicked, this, &OCS2FSMPanel::onHoldToHome);
        connect(switch_pose_btn_.get(), &QPushButton::clicked, this, &OCS2FSMPanel::onSwitchPose);

        // 初始化按钮可见性（初始状态为HOLD）
        updateButtonVisibility();

        // 创建重置定时器
        reset_timer_ = std::make_unique<QTimer>(this);
        reset_timer_->setSingleShot(true);
        reset_timer_->setInterval(100); // 100ms延迟
        connect(reset_timer_.get(), &QTimer::timeout, this, &OCS2FSMPanel::onSwitchPoseReleased);
    }

    OCS2FSMPanel::~OCS2FSMPanel() = default;

    void OCS2FSMPanel::onInitialize()
    {
        // Use RViz display context to get the node instead of creating a new one
        node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        // Create publisher
        publisher_ = node_->create_publisher<arms_ros2_control_msgs::msg::Inputs>(
            "/control_input", 10);

        // Create subscriber to listen for control input changes
        subscriber_ = node_->create_subscription<arms_ros2_control_msgs::msg::Inputs>(
            "/control_input", 10,
            std::bind(&OCS2FSMPanel::onControlInputReceived, this, std::placeholders::_1));

        RCLCPP_INFO(node_->get_logger(), "OCS2 FSM Panel initialized");
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
            // 按下：发送command=4来触发姿态切换
            publishCommand(100);
            switch_pose_pressed_ = true;

            // 启动定时器，100ms后自动"释放"
            reset_timer_->start();
        }
    }

    void OCS2FSMPanel::onSwitchPoseReleased()
    {
        // 释放：发送command=0来重置防抖状态
        publishCommand(0);
        switch_pose_pressed_ = false;
    }

    void OCS2FSMPanel::publishCommand(int32_t command)
    {
        auto msg = arms_ros2_control_msgs::msg::Inputs();
        msg.command = command;
        msg.x = 0.0;
        msg.y = 0.0;
        msg.z = 0.0;
        msg.roll = 0.0;
        msg.pitch = 0.0;
        msg.yaw = 0.0;
        msg.target = 1; // 默认左臂

        publisher_->publish(msg);
        current_command_ = command;

        RCLCPP_INFO(node_->get_logger(), "Published command: %d", command);
    }

    void OCS2FSMPanel::updateStatusDisplay()
    {
        updateButtonVisibility();
    }

    void OCS2FSMPanel::updateButtonVisibility()
    {
        // 根据当前状态显示/隐藏相应的按钮
        if (current_state_ == "HOME")
        {
            // HOME状态可以转换到HOLD，也可以切换姿态
            hold_to_ocs2_btn_->setVisible(false);
            hold_to_movej_btn_->setVisible(false);
            ocs2_to_hold_btn_->setVisible(false);
            movej_to_hold_btn_->setVisible(false);
            hold_to_home_btn_->setVisible(false);
            home_to_hold_btn_->setVisible(true);
            home_to_hold_btn_->setEnabled(true);
            switch_pose_btn_->setVisible(true);
            switch_pose_btn_->setEnabled(true);
        }
        else if (current_state_ == "HOLD")
        {
            // HOLD状态可以转换到OCS2、HOME或MOVEJ
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
        }
        else if (current_state_ == "OCS2")
        {
            // OCS2状态只能转换到HOLD
            home_to_hold_btn_->setVisible(false);
            switch_pose_btn_->setVisible(false);
            hold_to_ocs2_btn_->setVisible(false);
            hold_to_movej_btn_->setVisible(false);
            hold_to_home_btn_->setVisible(false);
            movej_to_hold_btn_->setVisible(false);
            ocs2_to_hold_btn_->setVisible(true);
            ocs2_to_hold_btn_->setEnabled(true);
        }
        else if (current_state_ == "MOVEJ")
        {
            // MOVEJ状态只能转换到HOLD
            home_to_hold_btn_->setVisible(false);
            switch_pose_btn_->setVisible(false);
            hold_to_ocs2_btn_->setVisible(false);
            hold_to_home_btn_->setVisible(false);
            ocs2_to_hold_btn_->setVisible(false);
            hold_to_movej_btn_->setVisible(false);
            movej_to_hold_btn_->setVisible(true);
            movej_to_hold_btn_->setEnabled(true);
        }
        else
        {
            // 未知状态，隐藏所有按钮
            home_to_hold_btn_->setVisible(false);
            switch_pose_btn_->setVisible(false);
            hold_to_ocs2_btn_->setVisible(false);
            hold_to_movej_btn_->setVisible(false);
            ocs2_to_hold_btn_->setVisible(false);
            movej_to_hold_btn_->setVisible(false);
            hold_to_home_btn_->setVisible(false);
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
        // Load configuration data
        Panel::load(config);
    }

    void OCS2FSMPanel::save(rviz_common::Config config) const
    {
        // Save configuration data
        Panel::save(config);
    }

    void OCS2FSMPanel::onControlInputReceived(const arms_ros2_control_msgs::msg::Inputs::SharedPtr msg)
    {
        // 根据当前状态和命令，检查状态转换是否有效
        bool valid_transition = false;
        std::string new_state = current_state_;
        
        switch (msg->command)
        {
            case 1: // HOLD → HOME
                if (current_state_ == "HOLD")
                {
                    new_state = "HOME";
                    valid_transition = true;
                }
                break;
            case 2: // HOME → HOLD 或 OCS2 → HOLD 或 MOVEJ → HOLD
                if (current_state_ == "HOME" || current_state_ == "OCS2" || current_state_ == "MOVEJ")
                {
                    new_state = "HOLD";
                    valid_transition = true;
                }
                break;
            case 3: // HOLD → OCS2
                if (current_state_ == "HOLD")
                {
                    new_state = "OCS2";
                    valid_transition = true;
                }
                break;
            case 4: // HOLD → MOVEJ
                if (current_state_ == "HOLD")
                {
                    new_state = "MOVEJ";
                    valid_transition = true;
                }
                break;
            default:
                break;
        }
        
        if (valid_transition)
        {
            setCurrentState(new_state);
            current_command_ = msg->command;
        }
    }
} // namespace arms_rviz_control_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(arms_rviz_control_plugin::OCS2FSMPanel, rviz_common::Panel)
