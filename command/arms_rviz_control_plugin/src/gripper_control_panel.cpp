#include "arms_rviz_control_plugin/gripper_control_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <rclcpp/rclcpp.hpp>

namespace arms_rviz_control_plugin
{

GripperControlPanel::GripperControlPanel(QWidget* parent)
  : rviz_common::Panel(parent)
  , current_gripper_target_(0)
  , gripper_command_received_(false)
  , arm_id_(1)  // Default to arm 1 (left arm/single arm)
  , is_dual_arm_mode_(false)
  , left_gripper_open_(false)
  , right_gripper_open_(false)
  , left_gripper_command_received_(false)
  , right_gripper_command_received_(false)
  , single_arm_group_(nullptr)
  , dual_arm_group_(nullptr)
  , left_gripper_btn_(nullptr)
  , right_gripper_btn_(nullptr)
  , left_gripper_state_label_(nullptr)
  , right_gripper_state_label_(nullptr)
  , left_status_label_(nullptr)
  , right_status_label_(nullptr)
{
  // Create basic UI layout - dual-arm detection will happen in onInitialize()
  auto* main_layout = new QVBoxLayout(this);

  // Title
  auto* title_label = new QLabel("Gripper Control");
  title_label->setStyleSheet("QLabel { font-weight: bold; font-size: 14px; }");
  main_layout->addWidget(title_label);

  // Placeholder - will be updated in onInitialize()
  arm_id_label_ = new QLabel("Detecting robot configuration...");
  arm_id_label_->setStyleSheet("QLabel { color: #666; font-size: 12px; }");
  main_layout->addWidget(arm_id_label_);

  // Add stretch to push content to top
  main_layout->addStretch();
}

GripperControlPanel::~GripperControlPanel() = default;

void GripperControlPanel::onInitialize()
{
  // Create ROS2 node
  node_ = rclcpp::Node::make_shared("gripper_control_panel");

  // Now detect if this is a dual-arm robot (node_ is available now)
  is_dual_arm_mode_ = detectDualArmMode();

  // Update the UI based on detection result
  if (is_dual_arm_mode_) {
    arm_id_label_->setText("Dual Arm Robot - Independent Control");
    arm_id_label_->setStyleSheet("QLabel { color: #e65100; font-size: 12px; font-weight: bold; }");

    // Create dual-arm UI
    auto* main_layout = qobject_cast<QVBoxLayout*>(this->layout());
    if (main_layout) {
      createDualArmUI(main_layout);
    }
  } else {
    arm_id_label_->setText("Single Arm Robot (ID: 1)");
    arm_id_label_->setStyleSheet("QLabel { color: #666; font-size: 12px; }");

    // Create single-arm UI
    auto* main_layout = qobject_cast<QVBoxLayout*>(this->layout());
    if (main_layout) {
      createSingleArmUI(main_layout);
    }
  }

  // Create publishers
  gripper_publisher_ = node_->create_publisher<arms_ros2_control_msgs::msg::Gripper>(
    "/gripper_command", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());

  // Create subscriber for gripper command state - this enables joystick-panel synchronization
  gripper_subscription_ = node_->create_subscription<arms_ros2_control_msgs::msg::Gripper>(
    "/gripper_command", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
    std::bind(&GripperControlPanel::onGripperCommandReceived, this, std::placeholders::_1));

  // Create timer to process ROS2 events
  ros_timer_ = new QTimer(this);
  ros_timer_->setInterval(50); // 20Hz refresh rate
  connect(ros_timer_, &QTimer::timeout, [this]() {
    rclcpp::spin_some(node_);
  });
  ros_timer_->start();

  if (is_dual_arm_mode_) {
    RCLCPP_INFO(node_->get_logger(), "Gripper Control Panel initialized for dual-arm robot");
    if (left_status_label_) left_status_label_->setText("Panel ready");
    if (right_status_label_) right_status_label_->setText("Panel ready");
  } else {
    RCLCPP_INFO(node_->get_logger(), "Gripper Control Panel initialized for single-arm robot");
    if (gripper_status_label_) gripper_status_label_->setText("Panel ready - Gripper control active");
  }
}

void GripperControlPanel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);

  // Load gripper state if available
  QVariant saved_gripper_state;
  if (config.mapGetValue("gripper_target", &saved_gripper_state)) {
    current_gripper_target_ = saved_gripper_state.toInt();
    gripper_command_received_ = true;
    updateGripperDisplay();
  }

  // Load arm ID if available (for future dual-arm support)
  QVariant saved_arm_id;
  if (config.mapGetValue("arm_id", &saved_arm_id)) {
    arm_id_ = saved_arm_id.toInt();
    arm_id_label_->setText(QString("Single Arm Robot (ID: %1)").arg(arm_id_));
  }
}

void GripperControlPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);

  // Save current gripper state
  config.mapSetValue("gripper_target", current_gripper_target_);
  config.mapSetValue("arm_id", arm_id_);
}

void GripperControlPanel::onGripperToggle()
{
  if (!gripper_command_received_) {
    // If no command state received, gripper is initially closed, so open it
    publishGripperCommand(true);
    gripper_status_label_->setText("Sent: Open Gripper (initial state: closed)");
  } else {
    // Toggle based on current state
    bool should_open = (current_gripper_target_ == 0);  // If current is close, then open
    publishGripperCommand(should_open);
    gripper_status_label_->setText(QString("Sent: %1 Gripper").arg(should_open ? "Open" : "Close"));
  }
}

void GripperControlPanel::publishGripperCommand(bool open)
{
  if (!gripper_publisher_) {
    RCLCPP_WARN(rclcpp::get_logger("gripper_control_panel"), "Gripper publisher not initialized");
    return;
  }

  auto gripper_msg = arms_ros2_control_msgs::msg::Gripper();

  if (open) {
    gripper_msg.target = 1;      // gripper state: 1=open
    gripper_msg.direction = 1;   // direction: positive
  } else {
    gripper_msg.target = 0;      // gripper state: 0=close
    gripper_msg.direction = -1;  // direction: negative
  }

  gripper_msg.arm_id = arm_id_;  // target arm: 1=left/single arm

  gripper_publisher_->publish(gripper_msg);

  RCLCPP_DEBUG(node_->get_logger(),
               "Published gripper command: target=%d, direction=%d, arm_id=%d",
               gripper_msg.target, gripper_msg.direction, gripper_msg.arm_id);
}

void GripperControlPanel::onGripperCommandReceived(const arms_ros2_control_msgs::msg::Gripper::SharedPtr msg)
{
  if (is_dual_arm_mode_) {
    // Handle dual-arm mode
    if (msg->arm_id == 1) {
      // Left arm
      left_gripper_open_ = (msg->target == 1);
      left_gripper_command_received_ = true;
    } else if (msg->arm_id == 2) {
      // Right arm
      right_gripper_open_ = (msg->target == 1);
      right_gripper_command_received_ = true;
    }

    updateDualArmGripperDisplay();

    RCLCPP_DEBUG(node_->get_logger(), "Dual-arm gripper command received for arm %d: target=%d",
                 msg->arm_id, msg->target);
  } else {
    // Handle single-arm mode - only respond to commands for this arm
    if (msg->arm_id != arm_id_) {
      return;
    }

    current_gripper_target_ = msg->target;
    gripper_command_received_ = true;

    updateGripperDisplay();

    // Update status label to show the received command (for joystick synchronization)
    gripper_status_label_->setText(QString("Received: %1 Gripper").arg(current_gripper_target_ == 1 ? "Open" : "Close"));

    RCLCPP_DEBUG(node_->get_logger(), "Gripper command received for arm %d: target=%d, updated button to show: %s",
                 msg->arm_id, msg->target, (current_gripper_target_ == 1) ? "Close Gripper" : "Open Gripper");
  }
}

void GripperControlPanel::updateGripperDisplay()
{
  if (current_gripper_target_ == 1) {
    // Currently open, button should say "Close"
    gripper_toggle_btn_->setText("Close Gripper");
    gripper_toggle_btn_->setStyleSheet("QPushButton { background-color: #F44336; color: white; font-weight: bold; padding: 10px; }");
    gripper_state_label_->setText("Gripper State: Open");
  } else {
    // Currently closed, button should say "Open"
    gripper_toggle_btn_->setText("Open Gripper");
    gripper_toggle_btn_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }");
    gripper_state_label_->setText("Gripper State: Closed");
  }
}

std::string GripperControlPanel::getCurrentRobotName()
{
  // Try to get robot name from environment variable (set by launch file)
  const char* env_robot_name = std::getenv("ROBOT_NAME");
  if (env_robot_name && strlen(env_robot_name) > 0) {
    std::string robot_name = std::string(env_robot_name);
    RCLCPP_INFO(rclcpp::get_logger("gripper_control_panel"), "Got robot name from environment: %s", robot_name.c_str());
    return robot_name;
  }

  // Fallback: use default robot name
  RCLCPP_WARN(rclcpp::get_logger("gripper_control_panel"), "No ROBOT_NAME environment variable found, using default: cr5");
  return "cr5";
}

std::string GripperControlPanel::findRobotControllerConfig(const std::string& robot_name)
{
  const auto pkg = robot_name + "_description";
  try {
    const auto share = ament_index_cpp::get_package_share_directory(pkg);
    std::filesystem::path config = std::filesystem::path(share) /
                                   "config" / "ros2_control" / "ros2_controllers.yaml";
    if (std::filesystem::is_regular_file(config)) {
      RCLCPP_INFO(rclcpp::get_logger("gripper_control_panel"),
                  "Found controller config: %s", config.string().c_str());
      return config.string();
    } else {
      RCLCPP_WARN(rclcpp::get_logger("gripper_control_panel"),
                  "Controller config file not found: %s", config.string().c_str());
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(rclcpp::get_logger("gripper_control_panel"),
                "Package '%s' not found in ament index: %s",
                pkg.c_str(), e.what());
  }

  return "";
}

bool GripperControlPanel::detectDualArmMode()
{
  std::string robot_name = getCurrentRobotName();
  RCLCPP_INFO(rclcpp::get_logger("gripper_control_panel"), "Current robot name: %s", robot_name.c_str());

  // Find the robot's controller configuration file
  std::string config_path = findRobotControllerConfig(robot_name);
  if (config_path.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("gripper_control_panel"), "No controller config found, defaulting to single arm");
    return false;
  }

  // Analyze the controller configuration to determine arm mode
  return analyzeControllerConfig(config_path);
}

bool GripperControlPanel::analyzeControllerConfig(const std::string& config_path)
{
  std::ifstream file(config_path);
  if (!file.is_open()) {
    RCLCPP_WARN(rclcpp::get_logger("gripper_control_panel"), "Cannot open config file: %s", config_path.c_str());
    return false;
  }

  std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
  file.close();

  // Remove comment lines to avoid false positives
  std::regex comment_regex(R"(#.*$)");
  std::string clean_content = std::regex_replace(content, comment_regex, "", std::regex_constants::format_default);

  // Use robust regex patterns to find controller definitions
  std::regex single_hand_regex(R"(\s+hand_controller\s*:\s*)");        // Single arm controller
  std::regex left_hand_regex(R"(\s+left_hand_controller\s*:\s*)");     // Left arm controller
  std::regex right_hand_regex(R"(\s+right_hand_controller\s*:\s*)");   // Right arm controller

  bool has_single_hand = std::regex_search(clean_content, single_hand_regex);
  bool has_left_hand = std::regex_search(clean_content, left_hand_regex);
  bool has_right_hand = std::regex_search(clean_content, right_hand_regex);

  RCLCPP_INFO(rclcpp::get_logger("gripper_control_panel"),
              "Controller analysis: hand_controller=%s, left_hand_controller=%s, right_hand_controller=%s",
              has_single_hand ? "true" : "false",
              has_left_hand ? "true" : "false",
              has_right_hand ? "true" : "false");

  // Dual arm if both left and right hand controllers exist
  if (has_left_hand && has_right_hand) {
    RCLCPP_INFO(rclcpp::get_logger("gripper_control_panel"), "Detected DUAL-ARM robot (left + right controllers)");
    return true;
  }
  // Single arm if only single hand controller exists
  else if (has_single_hand && !has_left_hand && !has_right_hand) {
    RCLCPP_INFO(rclcpp::get_logger("gripper_control_panel"), "Detected SINGLE-ARM robot (single controller)");
    return false;
  }
  // Default to single arm for unclear cases
  else {
    RCLCPP_INFO(rclcpp::get_logger("gripper_control_panel"), "Unclear controller config, defaulting to SINGLE-ARM");
    return false;
  }
}

void GripperControlPanel::createSingleArmUI(QVBoxLayout* main_layout)
{
  // Create original single-arm UI
  single_arm_group_ = new QGroupBox("Gripper Control");
  auto* gripper_layout = new QVBoxLayout(single_arm_group_);

  // Gripper toggle button - initial state shows "Open" since gripper starts closed
  gripper_toggle_btn_ = new QPushButton("Open Gripper");
  gripper_toggle_btn_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }");
  gripper_layout->addWidget(gripper_toggle_btn_);

  // Gripper state display - initial state is closed
  gripper_state_label_ = new QLabel("Gripper State: Closed");
  gripper_state_label_->setStyleSheet("QLabel { padding: 5px; background-color: #e3f2fd; border: 1px solid #2196F3; font-weight: bold; }");
  gripper_layout->addWidget(gripper_state_label_);

  // Status display
  gripper_status_label_ = new QLabel("Ready");
  gripper_status_label_->setAlignment(Qt::AlignCenter);
  gripper_status_label_->setStyleSheet("QLabel { background-color: #f0f0f0; padding: 5px; border: 1px solid #ccc; }");
  gripper_layout->addWidget(gripper_status_label_);

  main_layout->addWidget(single_arm_group_);

  // Connect signals
  connect(gripper_toggle_btn_, &QPushButton::clicked, this, &GripperControlPanel::onGripperToggle);
}

void GripperControlPanel::createDualArmUI(QVBoxLayout* main_layout)
{
  // Create dual-arm UI with two separate controls
  dual_arm_group_ = new QGroupBox("Dual Arm Gripper Control");
  auto* dual_layout = new QVBoxLayout(dual_arm_group_);

  // Left arm control
  auto* left_arm_group = new QGroupBox("Left Arm");
  auto* left_layout = new QVBoxLayout(left_arm_group);

  left_gripper_btn_ = new QPushButton("Open Left Gripper");
  left_gripper_btn_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; }");
  left_layout->addWidget(left_gripper_btn_);

  left_gripper_state_label_ = new QLabel("Left Gripper: Closed");
  left_gripper_state_label_->setStyleSheet("QLabel { padding: 3px; background-color: #e3f2fd; border: 1px solid #2196F3; font-weight: bold; }");
  left_layout->addWidget(left_gripper_state_label_);

  left_status_label_ = new QLabel("Ready");
  left_status_label_->setAlignment(Qt::AlignCenter);
  left_status_label_->setStyleSheet("QLabel { background-color: #f0f0f0; padding: 3px; border: 1px solid #ccc; }");
  left_layout->addWidget(left_status_label_);

  dual_layout->addWidget(left_arm_group);

  // Right arm control
  auto* right_arm_group = new QGroupBox("Right Arm");
  auto* right_layout = new QVBoxLayout(right_arm_group);

  right_gripper_btn_ = new QPushButton("Open Right Gripper");
  right_gripper_btn_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; }");
  right_layout->addWidget(right_gripper_btn_);

  right_gripper_state_label_ = new QLabel("Right Gripper: Closed");
  right_gripper_state_label_->setStyleSheet("QLabel { padding: 3px; background-color: #e3f2fd; border: 1px solid #2196F3; font-weight: bold; }");
  right_layout->addWidget(right_gripper_state_label_);

  right_status_label_ = new QLabel("Ready");
  right_status_label_->setAlignment(Qt::AlignCenter);
  right_status_label_->setStyleSheet("QLabel { background-color: #f0f0f0; padding: 3px; border: 1px solid #ccc; }");
  right_layout->addWidget(right_status_label_);

  dual_layout->addWidget(right_arm_group);
  main_layout->addWidget(dual_arm_group_);

  // Connect signals (placeholder - no functionality yet as requested)
  connect(left_gripper_btn_, &QPushButton::clicked, this, &GripperControlPanel::onLeftGripperToggle);
  connect(right_gripper_btn_, &QPushButton::clicked, this, &GripperControlPanel::onRightGripperToggle);
}

void GripperControlPanel::onLeftGripperToggle()
{
  if (!left_gripper_command_received_) {
    // If no command state received, gripper is initially closed, so open it
    publishGripperCommand(true, 1);
    if (left_status_label_) left_status_label_->setText("Sent: Open Left Gripper (initial state: closed)");
  } else {
    // Toggle based on current state
    bool should_open = !left_gripper_open_;
    publishGripperCommand(should_open, 1);
    if (left_status_label_) left_status_label_->setText(QString("Sent: %1 Left Gripper").arg(should_open ? "Open" : "Close"));
  }
}

void GripperControlPanel::onRightGripperToggle()
{
  if (!right_gripper_command_received_) {
    // If no command state received, gripper is initially closed, so open it
    publishGripperCommand(true, 2);
    if (right_status_label_) right_status_label_->setText("Sent: Open Right Gripper (initial state: closed)");
  } else {
    // Toggle based on current state
    bool should_open = !right_gripper_open_;
    publishGripperCommand(should_open, 2);
    if (right_status_label_) right_status_label_->setText(QString("Sent: %1 Right Gripper").arg(should_open ? "Open" : "Close"));
  }
}

void GripperControlPanel::publishGripperCommand(bool open, int32_t arm_id)
{
  if (!gripper_publisher_) {
    RCLCPP_WARN(rclcpp::get_logger("gripper_control_panel"), "Gripper publisher not initialized");
    return;
  }

  auto gripper_msg = arms_ros2_control_msgs::msg::Gripper();

  if (open) {
    gripper_msg.target = 1;      // gripper state: 1=open
    gripper_msg.direction = 1;   // direction: positive
  } else {
    gripper_msg.target = 0;      // gripper state: 0=close
    gripper_msg.direction = -1;  // direction: negative
  }

  gripper_msg.arm_id = arm_id;  // target arm: 1=left, 2=right

  gripper_publisher_->publish(gripper_msg);

  RCLCPP_DEBUG(node_->get_logger(),
               "Published gripper command: target=%d, direction=%d, arm_id=%d",
               gripper_msg.target, gripper_msg.direction, gripper_msg.arm_id);
}

void GripperControlPanel::updateDualArmGripperDisplay()
{
  // Update left arm display
  if (left_gripper_btn_ && left_gripper_state_label_ && left_status_label_) {
    if (left_gripper_open_) {
      // Currently open, button should say "Close"
      left_gripper_btn_->setText("Close Left Gripper");
      left_gripper_btn_->setStyleSheet("QPushButton { background-color: #F44336; color: white; font-weight: bold; padding: 8px; }");
      left_gripper_state_label_->setText("Left Gripper: Open");
    } else {
      // Currently closed, button should say "Open"
      left_gripper_btn_->setText("Open Left Gripper");
      left_gripper_btn_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; }");
      left_gripper_state_label_->setText("Left Gripper: Closed");
    }
    left_status_label_->setText(QString("Received: %1 Left Gripper").arg(left_gripper_open_ ? "Open" : "Close"));
  }

  // Update right arm display
  if (right_gripper_btn_ && right_gripper_state_label_ && right_status_label_) {
    if (right_gripper_open_) {
      // Currently open, button should say "Close"
      right_gripper_btn_->setText("Close Right Gripper");
      right_gripper_btn_->setStyleSheet("QPushButton { background-color: #F44336; color: white; font-weight: bold; padding: 8px; }");
      right_gripper_state_label_->setText("Right Gripper: Open");
    } else {
      // Currently closed, button should say "Open"
      right_gripper_btn_->setText("Open Right Gripper");
      right_gripper_btn_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 8px; }");
      right_gripper_state_label_->setText("Right Gripper: Closed");
    }
    right_status_label_->setText(QString("Received: %1 Right Gripper").arg(right_gripper_open_ ? "Open" : "Close"));
  }
}

} // namespace arms_rviz_control_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(arms_rviz_control_plugin::GripperControlPanel, rviz_common::Panel)