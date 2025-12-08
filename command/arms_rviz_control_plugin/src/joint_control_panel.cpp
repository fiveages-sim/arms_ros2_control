#include "arms_rviz_control_plugin/joint_control_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <QFrame>
#include <algorithm>
#include <cmath>
#include <cctype>

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
        joint_control_group_->setTitle("");  // Hide title
        joint_layout_ = std::make_unique<QVBoxLayout>(joint_control_group_.get());
        joint_layout_->setSpacing(5);

        // Add group box to scroll area
        scroll_area_->setWidget(joint_control_group_.get());
        main_layout->addWidget(scroll_area_.get());

        // Create send button
        send_button_ = std::make_unique<QPushButton>("发送关节位置", this);
        send_button_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; }");
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

        // Declare parameter with empty default
        node_->declare_parameter("joint_controllers", std::vector<std::string>());
        
        // Get joint controllers from parameters
        available_controllers_ = node_->get_parameter("joint_controllers").as_string_array();

        // Determine available categories and map to controllers
        available_categories_.clear();
        category_to_controller_.clear();
        
        std::string wbc_controller;  // ocs2_wbc_controller (handles left, right, body)
        std::string arm_controller;  // ocs2_arm_controller (handles left, right only)
        
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
                arm_controller = controller;
                available_categories_.insert("left");
                available_categories_.insert("right");
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
        if (!arm_controller.empty() && wbc_controller.empty())
        {
            category_to_controller_["left"] = arm_controller;
            category_to_controller_["right"] = arm_controller;
        }

        // Update category combo box options
        updateCategoryOptions();

        // Create subscribers
        control_input_subscriber_ = node_->create_subscription<arms_ros2_control_msgs::msg::Inputs>(
            "/control_input", 10,
            std::bind(&JointControlPanel::onControlInputReceived, this, std::placeholders::_1));

        joint_state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            std::bind(&JointControlPanel::onJointStateReceived, this, std::placeholders::_1));

        // Initialize publisher (will be updated when category changes)
        updatePublisher();

        // Initialize visibility based on default state (command = 2, not enabled)
        updatePanelVisibility();

        RCLCPP_INFO(node_->get_logger(), "Joint Control Panel initialized with %zu controllers", available_controllers_.size());
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

    void JointControlPanel::onControlInputReceived(const arms_ros2_control_msgs::msg::Inputs::SharedPtr msg)
    {
        current_command_ = msg->command;
        
        // Enable joint control when command is 3 (OCS2) or 4 (MOVEJ)
        bool should_enable = (msg->command == 3 || msg->command == 4);
        
        if (should_enable != is_joint_control_enabled_)
        {
            is_joint_control_enabled_ = should_enable;
            updatePanelVisibility();
        }
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
            joint_names_.clear();
            joint_positions_.clear();
            joint_name_to_index_.clear();
            joint_to_category_.clear();
            category_to_joints_.clear();

            // Filter out gripper joints and classify joints
            for (size_t i = 0; i < msg->name.size(); ++i)
            {
                const std::string& joint_name = msg->name[i];
                // Skip gripper joints - check for gripper, hand, finger, thumb, etc.
                std::string joint_name_lower = joint_name;
                std::transform(joint_name_lower.begin(), joint_name_lower.end(), 
                              joint_name_lower.begin(), ::tolower);
                
                // Filter out gripper-related joints
                bool is_gripper_joint = 
                    joint_name_lower.find("gripper") != std::string::npos ||
                    joint_name_lower.find("hand") != std::string::npos ||
                    joint_name_lower.find("finger") != std::string::npos ||
                    joint_name_lower.find("thumb") != std::string::npos ||
                    joint_name_lower.find("palm") != std::string::npos;
                
                if (!is_gripper_joint)
                {
                    size_t joint_index = joint_names_.size();
                    joint_names_.push_back(joint_name);
                    joint_name_to_index_[joint_name] = joint_index;
                    
                    // Classify joint
                    std::string category = classifyJoint(joint_name);
                    joint_to_category_[joint_name] = category;
                    category_to_joints_[category].push_back(joint_index);
                }
            }

            if (!joint_names_.empty())
            {
                // Create UI elements for each joint
                joint_row_layouts_.clear();
                joint_labels_.clear();
                joint_spinboxes_.clear();

                joint_positions_.resize(joint_names_.size(), 0.0);

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
                updateJointVisibility();
                
                // Log classification results
                RCLCPP_INFO(node_->get_logger(), "Initialized %zu joints for control", joint_names_.size());
                for (const auto& [category, indices] : category_to_joints_)
                {
                    RCLCPP_INFO(node_->get_logger(), "  %s: %zu joints", category.c_str(), indices.size());
                }
            }
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
        
        // When command is 3 (OCS2), hide button for arm categories (left/right) and body (if using ocs2_wbc_controller)
        if (current_command_ == 3)
        {
            // Hide button for left/right when using ocs2 controllers
            if (current_category_ == "left" || current_category_ == "right")
            {
                auto it = category_to_controller_.find(current_category_);
                if (it != category_to_controller_.end())
                {
                    std::string controller_lower = it->second;
                    std::transform(controller_lower.begin(), controller_lower.end(), 
                                  controller_lower.begin(), ::tolower);
                    
                    if (controller_lower.find("ocs2_wbc_controller") != std::string::npos ||
                        controller_lower.find("ocs2_arm_controller") != std::string::npos)
                    {
                        return false;
                    }
                }
            }
            
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
        
        // Show/hide send button
        if (send_button_)
        {
            send_button_->setVisible(shouldShowSendButton());
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
        
        return "/ocs2_wbc_controller/target_joint_position";
    }

    void JointControlPanel::updatePublisher()
    {
        // getControllerNameForCategory now returns the full topic name
        std::string topic_name = getControllerNameForCategory(current_category_);
        
        // Reset publisher
        joint_position_publisher_.reset();
        
        // Create new publisher
        joint_position_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
            topic_name, 10);
        
        RCLCPP_INFO(node_->get_logger(), "Updated publisher to topic: %s", topic_name.c_str());
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
                    visible = true;
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
        if (!joints_initialized_ || joint_spinboxes_.empty() || !joint_position_publisher_)
        {
            return;
        }

        auto msg = std_msgs::msg::Float64MultiArray();
        
        // If "all" category, publish all joints
        // Otherwise, only publish joints of the current category
        if (current_category_ == "all")
        {
            msg.data.resize(joint_spinboxes_.size());
            for (size_t i = 0; i < joint_spinboxes_.size(); ++i)
            {
                msg.data[i] = joint_spinboxes_[i]->value();
            }
        }
        else
        {
            // Only publish joints of the current category
            auto it = category_to_joints_.find(current_category_);
            if (it != category_to_joints_.end())
            {
                const auto& joint_indices = it->second;
                msg.data.resize(joint_indices.size());
                for (size_t i = 0; i < joint_indices.size(); ++i)
                {
                    size_t joint_idx = joint_indices[i];
                    if (joint_idx < joint_spinboxes_.size())
                    {
                        msg.data[i] = joint_spinboxes_[joint_idx]->value();
                    }
                }
            }
            else
            {
                return;  // No joints in this category
            }
        }

        joint_position_publisher_->publish(msg);
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


