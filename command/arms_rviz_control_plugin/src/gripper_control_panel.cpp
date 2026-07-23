#include "arms_rviz_control_plugin/gripper_control_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cctype>

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QSizePolicy>
#include <QDoubleSpinBox>
#include <QString>
#include <cmath>

namespace arms_rviz_control_plugin
{
    GripperControlPanel::GripperControlPanel(QWidget* parent)
        : Panel(parent)
    {
        auto* main_layout = new QVBoxLayout(this);

        no_controller_label_ = std::make_unique<QLabel>("No gripper controller", this);
        no_controller_label_->setStyleSheet(
            "QLabel { color: #666666; font-style: italic; padding: 10px; }");
        no_controller_label_->setAlignment(Qt::AlignCenter);
        main_layout->addWidget(no_controller_label_.get());

        tool_control_group_ = new QGroupBox(QStringLiteral("位置 (0~1)"), this);
        tool_control_group_->setVisible(false);
        main_layout->addWidget(tool_control_group_);
    }

    GripperControlPanel::~GripperControlPanel() = default;

    void GripperControlPanel::onInitialize()
    {
        // Use RViz display context to get the node instead of creating a new one
        node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        // Launch may already inject hand_controllers via --params-file; only declare if missing
        // (same pattern as JointControlPanel / OCS2FSMPanel).
        if (!node_->has_parameter("hand_controllers")) {
            node_->declare_parameter("hand_controllers", std::vector<std::string>());
        }
        hand_controllers_ = node_->get_parameter("hand_controllers").as_string_array();

        if (hand_controllers_.empty())
        {
            RCLCPP_INFO(node_->get_logger(), "No gripper controllers detected");
            is_dual_arm_mode_ = false;
            updateButtonVisibility();
        }
        else
        {
            determineControllerMapping();
            is_dual_arm_mode_ = !left_controller_name_.empty() && !right_controller_name_.empty();
            RCLCPP_INFO(node_->get_logger(), "Detected mode: %s (controllers=%zu)",
                        is_dual_arm_mode_ ? "DUAL-ARM" : "SINGLE-SIDE",
                        hand_controllers_.size());
            updateButtonVisibility();

            // Create target_command publishers
            createTargetCommandPublishers();

            // Create target_command subscriptions for state synchronization
            createTargetCommandSubscriptions();
            createToolControlUi();

            RCLCPP_INFO(node_->get_logger(), "Gripper Control Panel initialized with %zu controllers",
                        hand_controllers_.size());
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
        const char* open_style =
            "QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; }";
        const char* close_style =
            "QPushButton { background-color: #F44336; color: white; font-weight: bold; padding: 8px; }";

        for (const auto& [controller_name, btn] : toggle_btn_by_controller_)
        {
            auto state_it = controller_to_state_.find(controller_name);
            if (state_it == controller_to_state_.end() || !btn)
            {
                continue;
            }
            const bool is_open = *state_it->second;
            const std::string display = getDisplayNameFromControllerName(controller_name);
            btn->setText(is_open ? QString("Close %1").arg(QString::fromStdString(display))
                                 : QString("Open %1").arg(QString::fromStdString(display)));
            btn->setStyleSheet(is_open ? close_style : open_style);
        }
    }

    void GripperControlPanel::updateButtonVisibility()
    {
        const bool has_controllers = !hand_controllers_.empty();
        no_controller_label_->setVisible(!has_controllers);
        if (tool_control_group_)
        {
            tool_control_group_->setVisible(has_controllers);
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
                // Single-side / diff-eef: first unnamed controller maps to left slot
                if (left_controller_name_.empty())
                {
                    left_controller_name_ = controller_name;
                    left_display_name_ = getDisplayNameFromControllerName(controller_name);
                    controller_to_state_[controller_name] = &left_gripper_open_;
                }
                else
                {
                    RCLCPP_WARN(node_->get_logger(),
                                "Hand controller '%s' has no left/right in name; ignored",
                                controller_name.c_str());
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
        target_percent_publishers_.clear();

        for (const auto& controller_name : hand_controllers_)
        {
            // Create publisher for target_command topic
            // Use volatile QoS to allow subscribers to receive messages
            std::string topic_name = "/" + controller_name + "/target_command";
            auto publisher = node_->create_publisher<std_msgs::msg::Int32>(
                topic_name, rclcpp::QoS(10));

            target_command_publishers_[controller_name] = publisher;
            target_percent_publishers_[controller_name] = node_->create_publisher<std_msgs::msg::Float64>(
                "/" + controller_name + "/target_percent", rclcpp::QoS(10));

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

    void GripperControlPanel::createToolControlUi()
    {
        if (!tool_control_group_ || hand_controllers_.empty())
        {
            return;
        }

        if (QLayout* old = tool_control_group_->layout())
        {
            QLayoutItem* item;
            while ((item = old->takeAt(0)) != nullptr)
            {
                if (item->widget())
                {
                    delete item->widget();
                }
                delete item;
            }
            delete old;
        }

        toggle_btn_by_controller_.clear();
        pos_spin_by_controller_.clear();

        std::vector<std::string> ordered;
        if (!left_controller_name_.empty())
        {
            ordered.push_back(left_controller_name_);
        }
        if (!right_controller_name_.empty())
        {
            ordered.push_back(right_controller_name_);
        }
        for (const auto& name : hand_controllers_)
        {
            if (name != left_controller_name_ && name != right_controller_name_)
            {
                ordered.push_back(name);
            }
        }

        auto* grid = new QGridLayout(tool_control_group_);
        grid->setHorizontalSpacing(8);
        grid->setVerticalSpacing(6);
        constexpr int kPosSpinWidth = 80;
        constexpr int kToggleBtnWidth = 156;

        const char* open_style =
            "QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; }";

        int row_idx = 0;
        for (const auto& name : ordered)
        {
            auto* pos_label = new QLabel(QStringLiteral("位置"), tool_control_group_);
            pos_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            grid->addWidget(pos_label, row_idx, 0);

            auto* pos = new QDoubleSpinBox(tool_control_group_);
            pos->setRange(0.0, 1.0);
            pos->setDecimals(3);
            pos->setSingleStep(0.05);
            pos->setFixedWidth(kPosSpinWidth);
            pos->setValue(0.0);
            grid->addWidget(pos, row_idx, 1, Qt::AlignLeft | Qt::AlignVCenter);
            pos_spin_by_controller_[name] = pos;

            auto* toggle_btn = new QPushButton(tool_control_group_);
            toggle_btn->setStyleSheet(open_style);
            toggle_btn->setFixedWidth(kToggleBtnWidth);
            toggle_btn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
            toggle_btn_by_controller_[name] = toggle_btn;
            connect(toggle_btn, &QPushButton::clicked, this,
                    [this, name]()
                    {
                        auto state_it = controller_to_state_.find(name);
                        if (state_it != controller_to_state_.end())
                        {
                            publishGripperCommand(!*state_it->second, name);
                        }
                    });
            grid->addWidget(toggle_btn, row_idx, 2, Qt::AlignLeft | Qt::AlignVCenter);
            ++row_idx;
        }

        grid->setColumnMinimumWidth(1, kPosSpinWidth);
        grid->setColumnMinimumWidth(2, kToggleBtnWidth);

        tool_control_send_btn_ = new QPushButton(QStringLiteral("发送位置"), tool_control_group_);
        connect(tool_control_send_btn_, &QPushButton::clicked, this,
                &GripperControlPanel::onPublishToolControlClicked);
        grid->addWidget(tool_control_send_btn_, row_idx, 0, 1, 3);

        updateGripperDisplay();
        updateButtonVisibility();
    }

    void GripperControlPanel::onPublishToolControlClicked()
    {
        for (const auto& name : hand_controllers_)
        {
            auto p = pos_spin_by_controller_.find(name);
            if (p == pos_spin_by_controller_.end())
            {
                continue;
            }
            const double pos = std::clamp(p->second->value(), 0.0, 1.0);
            auto pub = target_percent_publishers_.find(name);
            if (pub == target_percent_publishers_.end())
            {
                continue;
            }
            std_msgs::msg::Float64 msg;
            msg.data = pos;
            pub->second->publish(msg);
            RCLCPP_INFO(node_->get_logger(), "[%s] target_percent=%.3f", name.c_str(), pos);
        }
    }
} // namespace arms_rviz_control_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(arms_rviz_control_plugin::GripperControlPanel, rviz_common::Panel)
