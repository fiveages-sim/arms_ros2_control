#include "arms_rviz_control_plugin/gripper_control_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cctype>

#include <QHBoxLayout>
#include <QGroupBox>

namespace arms_rviz_control_plugin
{
    GripperControlPanel::GripperControlPanel(QWidget* parent)
        : Panel(parent)
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
            is_dual_arm_mode_ = false; // Not relevant when no controllers
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
            // Determine controller mapping (left/right)
            determineControllerMapping();

            // Create target_command publishers
            createTargetCommandPublishers();

            // Create target_command subscriptions for state synchronization
            createTargetCommandSubscriptions();

            RCLCPP_INFO(node_->get_logger(), "Gripper Control Panel initialized with %zu controllers",
                        hand_controllers_.size());
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
        if (left_controller_name_.empty())
        {
            RCLCPP_WARN(node_->get_logger(), "Left controller not found");
            return;
        }
        // Toggle based on current state
        bool should_open = !left_gripper_open_;
        publishGripperCommand(should_open, left_controller_name_);
    }

    void GripperControlPanel::onRightGripperToggle()
    {
        if (right_controller_name_.empty())
        {
            RCLCPP_WARN(node_->get_logger(), "Right controller not found");
            return;
        }
        // Toggle based on current state
        bool should_open = !right_gripper_open_;
        publishGripperCommand(should_open, right_controller_name_);
    }

    void GripperControlPanel::onTargetCommandReceived(const std::string& controller_name,
                                                      const std_msgs::msg::Int32::SharedPtr msg)
    {
        // Get state pointer from controller name
        auto it = controller_to_state_.find(controller_name);
        if (it == controller_to_state_.end())
        {
            RCLCPP_WARN(node_->get_logger(), "Received target command from unknown controller: %s",
                        controller_name.c_str());
            return;
        }

        bool is_open = msg->data == 1;

        // Update gripper state directly
        *it->second = is_open;

        updateGripperDisplay();
    }

    void GripperControlPanel::publishGripperCommand(bool open, const std::string& controller_name)
    {
        auto it = target_command_publishers_.find(controller_name);
        if (it == target_command_publishers_.end() || !it->second)
        {
            RCLCPP_WARN(node_->get_logger(), "Target command publisher not found for controller: %s",
                        controller_name.c_str());
            return;
        }

        // Publish to /<controller_name>/target_command
        auto target_msg = std_msgs::msg::Int32();
        target_msg.data = open ? 1 : 0;
        it->second->publish(target_msg);

        // Update state immediately when we publish (don't wait for subscription callback)
        auto state_it = controller_to_state_.find(controller_name);
        if (state_it != controller_to_state_.end())
        {
            *(state_it->second) = open;
            updateGripperDisplay();
        }

        RCLCPP_DEBUG(node_->get_logger(),
                     "Published target command: controller=%s, target=%d (state updated immediately)",
                     controller_name.c_str(), target_msg.data);
    }

    void GripperControlPanel::updateGripperDisplay()
    {
        // Update left arm button
        if (left_gripper_btn_ && !left_display_name_.empty())
        {
            if (left_gripper_open_)
            {
                left_gripper_btn_->setText(("Close " + left_display_name_).c_str());
                left_gripper_btn_->setStyleSheet(
                    "QPushButton { background-color: #F44336; color: white; font-weight: bold; padding: 10px; }");
            }
            else
            {
                left_gripper_btn_->setText(("Open " + left_display_name_).c_str());
                left_gripper_btn_->setStyleSheet(
                    "QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }");
            }
        }

        // Update right arm button (only in dual arm mode)
        if (right_gripper_btn_ && is_dual_arm_mode_ && !right_display_name_.empty())
        {
            if (right_gripper_open_)
            {
                right_gripper_btn_->setText(("Close " + right_display_name_).c_str());
                right_gripper_btn_->setStyleSheet(
                    "QPushButton { background-color: #F44336; color: white; font-weight: bold; padding: 10px; }");
            }
            else
            {
                right_gripper_btn_->setText(("Open " + right_display_name_).c_str());
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
            // Single arm mode: show and enable left button, hide and disable right button
            left_gripper_btn_->setVisible(true);
            left_gripper_btn_->setEnabled(true);
            if (!left_display_name_.empty())
            {
                left_gripper_btn_->setText(("Open " + left_display_name_).c_str());
            }
            right_gripper_btn_->setVisible(false);
            right_gripper_btn_->setEnabled(false);
            no_controller_label_->setVisible(false);
        }
    }

    std::string GripperControlPanel::getDisplayNameFromControllerName(const std::string& controller_name)
    {
        std::string name = controller_name;

        // Remove common suffixes
        std::vector<std::string> suffixes = {"_controller", "controller", "_gripper", "gripper", "_hand", "hand"};
        for (const auto& suffix : suffixes)
        {
            if (name.length() >= suffix.length())
            {
                std::string name_lower = name;
                std::transform(name_lower.begin(), name_lower.end(), name_lower.begin(), ::tolower);
                std::string suffix_lower = suffix;
                std::transform(suffix_lower.begin(), suffix_lower.end(), suffix_lower.begin(), ::tolower);

                // Check if ends with suffix
                if (name_lower.length() >= suffix_lower.length() &&
                    name_lower.substr(name_lower.length() - suffix_lower.length()) == suffix_lower)
                {
                    name = name.substr(0, name.length() - suffix_lower.length());
                    break;
                }
            }
        }

        // Remove leading/trailing underscores
        while (!name.empty() && name.front() == '_')
        {
            name = name.substr(1);
        }
        while (!name.empty() && name.back() == '_')
        {
            name = name.substr(0, name.length() - 1);
        }

        // Replace underscores with spaces
        std::replace(name.begin(), name.end(), '_', ' ');

        // Capitalize first letter of each word
        bool capitalize_next = true;
        for (char& c : name)
        {
            if (c == ' ')
            {
                capitalize_next = true;
            }
            else if (capitalize_next)
            {
                c = std::toupper(c);
                capitalize_next = false;
            }
            else
            {
                c = std::tolower(c);
            }
        }

        return name.empty() ? "Gripper" : name;
    }

    void GripperControlPanel::determineControllerMapping()
    {
        left_controller_name_.clear();
        right_controller_name_.clear();
        left_display_name_.clear();
        right_display_name_.clear();
        controller_to_state_.clear();

        for (const auto& controller_name : hand_controllers_)
        {
            // Convert to lowercase for case-insensitive matching
            std::string name_lower = controller_name;
            std::transform(name_lower.begin(), name_lower.end(), name_lower.begin(), ::tolower);

            if (name_lower.find("left") != std::string::npos)
            {
                // Left arm controller
                left_controller_name_ = controller_name;
                left_display_name_ = getDisplayNameFromControllerName(controller_name);
                controller_to_state_[controller_name] = &left_gripper_open_;
            }
            else if (name_lower.find("right") != std::string::npos)
            {
                // Right arm controller
                right_controller_name_ = controller_name;
                right_display_name_ = getDisplayNameFromControllerName(controller_name);
                controller_to_state_[controller_name] = &right_gripper_open_;
            }
            else
            {
                // Default to left arm for single-arm robots
                if (left_controller_name_.empty())
                {
                    left_controller_name_ = controller_name;
                    left_display_name_ = getDisplayNameFromControllerName(controller_name);
                    controller_to_state_[controller_name] = &left_gripper_open_;
                }
            }
        }

        RCLCPP_INFO(node_->get_logger(),
                    "Controller mapping - Left: %s (%s), Right: %s (%s)",
                    left_controller_name_.c_str(), left_display_name_.c_str(),
                    right_controller_name_.c_str(), right_display_name_.c_str());
    }

    void GripperControlPanel::createTargetCommandPublishers()
    {
        target_command_publishers_.clear();

        for (const auto& controller_name : hand_controllers_)
        {
            // Create publisher for target_command topic
            // Use volatile QoS to allow subscribers to receive messages
            std::string topic_name = "/" + controller_name + "/target_command";
            auto publisher = node_->create_publisher<std_msgs::msg::Int32>(
                topic_name, rclcpp::QoS(10));

            target_command_publishers_[controller_name] = publisher;

            RCLCPP_INFO(node_->get_logger(),
                        "Created target_command publisher: %s",
                        topic_name.c_str());
        }
    }

    void GripperControlPanel::createTargetCommandSubscriptions()
    {
        target_command_subscriptions_.clear();

        for (const auto& controller_name : hand_controllers_)
        {
            // Create subscription for target_command topic to sync state
            // Use same QoS as publisher to ensure compatibility
            std::string topic_name = "/" + controller_name + "/target_command";

            // Use lambda to capture controller_name
            auto subscription = node_->create_subscription<std_msgs::msg::Int32>(
                topic_name, rclcpp::QoS(10),
                [this, controller_name](const std_msgs::msg::Int32::SharedPtr msg)
                {
                    onTargetCommandReceived(controller_name, msg);
                });

            target_command_subscriptions_[controller_name] = subscription;

            RCLCPP_INFO(node_->get_logger(),
                        "Created target_command subscription: %s (QoS: KeepLast(10))",
                        topic_name.c_str());
        }
    }
} // namespace arms_rviz_control_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(arms_rviz_control_plugin::GripperControlPanel, rviz_common::Panel)
