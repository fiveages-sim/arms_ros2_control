#include "arms_rviz_control_plugin/gripper_control_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>

namespace arms_rviz_control_plugin
{
    GripperControlPanel::GripperControlPanel(QWidget* parent)
        : Panel(parent)
          , is_dual_arm_mode_(false)
          , left_gripper_open_(false)
          , right_gripper_open_(false)
    {
        // Create UI layout
        auto* main_layout = new QVBoxLayout(this);

        // Create gripper control UI
        auto* gripper_group = new QGroupBox();
        auto* gripper_layout = new QHBoxLayout(gripper_group);

        // Left arm button (disabled by default)
        left_gripper_btn_ = std::make_unique<QPushButton>("Open Left Gripper", this);
        left_gripper_btn_->setVisible(false);
        left_gripper_btn_->setStyleSheet(
            "QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }");
        gripper_layout->addWidget(left_gripper_btn_.get());

        // Right arm button (disabled by default)
        right_gripper_btn_ = std::make_unique<QPushButton>("Open Right Gripper", this);
        right_gripper_btn_->setVisible(false);
        right_gripper_btn_->setStyleSheet(
            "QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }");
        gripper_layout->addWidget(right_gripper_btn_.get());

        // No controller label
        no_controller_label_ = std::make_unique<QLabel>("No gripper controller", this);
        no_controller_label_->setStyleSheet(
            "QLabel { color: #666666; font-style: italic; padding: 10px; }");
        no_controller_label_->setAlignment(Qt::AlignCenter);
        gripper_layout->addWidget(no_controller_label_.get());

        main_layout->addWidget(gripper_group);

        // Connect signals
        connect(left_gripper_btn_.get(), &QPushButton::clicked, this, &GripperControlPanel::onLeftGripperToggle);
        connect(right_gripper_btn_.get(), &QPushButton::clicked, this, &GripperControlPanel::onRightGripperToggle);
    }

    GripperControlPanel::~GripperControlPanel() = default;

    void GripperControlPanel::onInitialize()
    {
        // Use RViz display context to get the node instead of creating a new one
        node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        // Declare parameter with empty default
        node_->declare_parameter("hand_controllers", std::vector<std::string>());
        
        // Get hand controllers from parameters
        hand_controllers_ = node_->get_parameter("hand_controllers").as_string_array();

        // First check if we have any controllers
        if (hand_controllers_.empty())
        {
            RCLCPP_INFO(node_->get_logger(), "No gripper controllers detected");
            is_dual_arm_mode_ = false;  // Not relevant when no controllers
        }
        else
        {
            // Detect if this is a dual-arm robot using controller information
            is_dual_arm_mode_ = hand_controllers_.size() > 1;
            RCLCPP_INFO(node_->get_logger(), "Detected mode: %s",
                        is_dual_arm_mode_ ? "DUAL-ARM" : "SINGLE-ARM");
        }

        // Update button visibility based on mode
        updateButtonVisibility();

        // Only create publishers and subscribers if we have controllers
        if (!hand_controllers_.empty())
        {
            // Create publishers
            gripper_publisher_ = node_->create_publisher<arms_ros2_control_msgs::msg::Gripper>(
                "/gripper_command", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

            // Create subscriber for gripper command state - this enables joystick-panel synchronization
            gripper_subscription_ = node_->create_subscription<arms_ros2_control_msgs::msg::Gripper>(
                "/gripper_command", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
                std::bind(&GripperControlPanel::onGripperCommandReceived, this, std::placeholders::_1));

            RCLCPP_INFO(node_->get_logger(), "Gripper Control Panel initialized with %zu controllers", hand_controllers_.size());
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Gripper Control Panel initialized (no controllers)");
        }
    }

    void GripperControlPanel::load(const rviz_common::Config& config)
    {
        Panel::load(config);

        // Load gripper state if available
        QVariant saved_left_state, saved_right_state;
        if (config.mapGetValue("left_gripper_open", &saved_left_state))
        {
            left_gripper_open_ = saved_left_state.toBool();
        }
        if (config.mapGetValue("right_gripper_open", &saved_right_state))
        {
            right_gripper_open_ = saved_right_state.toBool();
        }
        updateGripperDisplay();
    }

    void GripperControlPanel::save(rviz_common::Config config) const
    {
        Panel::save(config);

        // Save current gripper state
        config.mapSetValue("left_gripper_open", left_gripper_open_);
        config.mapSetValue("right_gripper_open", right_gripper_open_);
    }


    void GripperControlPanel::onLeftGripperToggle()
    {
        // Toggle based on current state
        bool should_open = !left_gripper_open_;
        publishGripperCommand(should_open, 1);
    }

    void GripperControlPanel::onRightGripperToggle()
    {
        // Toggle based on current state
        bool should_open = !right_gripper_open_;
        publishGripperCommand(should_open, 2);
    }

    void GripperControlPanel::onGripperCommandReceived(const arms_ros2_control_msgs::msg::Gripper::SharedPtr msg)
    {
        // Update gripper state based on arm_id
        if (msg->arm_id == 1)
        {
            // Left arm
            left_gripper_open_ = (msg->target == 1);
        }
        else if (msg->arm_id == 2)
        {
            // Right arm
            right_gripper_open_ = (msg->target == 1);
        }

        updateGripperDisplay();

        RCLCPP_DEBUG(node_->get_logger(), "Gripper command received for arm %d: target=%d",
                     msg->arm_id, msg->target);
    }

    void GripperControlPanel::publishGripperCommand(bool open, int32_t arm_id)
    {
        if (!gripper_publisher_)
        {
            RCLCPP_WARN(rclcpp::get_logger("gripper_control_panel"), "Gripper publisher not initialized");
            return;
        }

        auto gripper_msg = arms_ros2_control_msgs::msg::Gripper();

        if (open)
        {
            gripper_msg.target = 1; // gripper state: 1=open
            gripper_msg.direction = 1; // direction: positive
        }
        else
        {
            gripper_msg.target = 0; // gripper state: 0=close
            gripper_msg.direction = -1; // direction: negative
        }

        gripper_msg.arm_id = arm_id; // target arm: 1=left, 2=right

        gripper_publisher_->publish(gripper_msg);

        RCLCPP_DEBUG(node_->get_logger(),
                     "Published gripper command: target=%d, direction=%d, arm_id=%d",
                     gripper_msg.target, gripper_msg.direction, gripper_msg.arm_id);
    }

    void GripperControlPanel::updateGripperDisplay()
    {
        // Update left arm button
        if (left_gripper_btn_)
        {
            if (left_gripper_open_)
            {
                if (is_dual_arm_mode_)
                {
                    left_gripper_btn_->setText("Close Left Gripper");
                }
                else
                {
                    left_gripper_btn_->setText("Close Gripper");
                }
                left_gripper_btn_->setStyleSheet(
                    "QPushButton { background-color: #F44336; color: white; font-weight: bold; padding: 10px; }");
            }
            else
            {
                if (is_dual_arm_mode_)
                {
                    left_gripper_btn_->setText("Open Left Gripper");
                }
                else
                {
                    left_gripper_btn_->setText("Open Gripper");
                }
                left_gripper_btn_->setStyleSheet(
                    "QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }");
            }
        }

        // Update right arm button (only in dual arm mode)
        if (right_gripper_btn_ && is_dual_arm_mode_)
        {
            if (right_gripper_open_)
            {
                right_gripper_btn_->setText("Close Right Gripper");
                right_gripper_btn_->setStyleSheet(
                    "QPushButton { background-color: #F44336; color: white; font-weight: bold; padding: 10px; }");
            }
            else
            {
                right_gripper_btn_->setText("Open Right Gripper");
                right_gripper_btn_->setStyleSheet(
                    "QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }");
            }
        }
    }

    void GripperControlPanel::updateButtonVisibility()
    {
        // Use the hand_controllers_ member variable that was set in onInitialize()
        if (hand_controllers_.empty())
        {
            // No controllers: show label, hide and disable buttons
            left_gripper_btn_->setVisible(false);
            left_gripper_btn_->setEnabled(false);
            right_gripper_btn_->setVisible(false);
            right_gripper_btn_->setEnabled(false);
            no_controller_label_->setVisible(true);
        }
        else if (is_dual_arm_mode_)
        {
            // Dual arm mode: show and enable both buttons, hide label
            left_gripper_btn_->setVisible(true);
            left_gripper_btn_->setEnabled(true);
            right_gripper_btn_->setVisible(true);
            right_gripper_btn_->setEnabled(true);
            no_controller_label_->setVisible(false);
        }
        else
        {
            // Single arm mode: show and enable left button (rename to generic "Gripper"), hide and disable right button
            left_gripper_btn_->setVisible(true);
            left_gripper_btn_->setEnabled(true);
            left_gripper_btn_->setText("Open Gripper");
            right_gripper_btn_->setVisible(false);
            right_gripper_btn_->setEnabled(false);
            no_controller_label_->setVisible(false);
        }
    }

} // namespace arms_rviz_control_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(arms_rviz_control_plugin::GripperControlPanel, rviz_common::Panel)
