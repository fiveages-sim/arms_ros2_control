#include "arms_rviz_control_plugin/ocs2_fsm_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <QTimer>

namespace arms_rviz_control_plugin
{

OCS2FSMPanel::OCS2FSMPanel(QWidget* parent)
  : rviz_common::Panel(parent)
  , current_command_(0)
  , current_state_("HOLD")
  , switch_pose_pressed_(false)
{
  // Create UI layout
  auto* main_layout = new QVBoxLayout(this);
  
  // Title
  auto* title_label = new QLabel("OCS2 Arm Controller FSM");
  title_label->setStyleSheet("QLabel { font-weight: bold; font-size: 14px; }");
  main_layout->addWidget(title_label);
  
  // State transition buttons
  auto* button_group = new QGroupBox("State Transitions");
  auto* button_layout = new QVBoxLayout(button_group);
  
  // HOME → HOLD (command = 2) - 修正command值
  home_to_hold_btn_ = new QPushButton("HOME → HOLD");
  home_to_hold_btn_->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; }");
  button_layout->addWidget(home_to_hold_btn_);
  
  // Switch pose button (command = 4) - 只在HOME状态显示
  switch_pose_btn_ = new QPushButton("切换姿态 (Home ↔ Rest)");
  switch_pose_btn_->setStyleSheet("QPushButton { background-color: #FF5722; color: white; font-weight: bold; }");
  button_layout->addWidget(switch_pose_btn_);
  
  // HOLD → OCS2 (command = 3)
  hold_to_ocs2_btn_ = new QPushButton("HOLD → OCS2");
  hold_to_ocs2_btn_->setStyleSheet("QPushButton { background-color: #2196F3; color: white; font-weight: bold; }");
  button_layout->addWidget(hold_to_ocs2_btn_);
  
  // OCS2 → HOLD (command = 2)
  ocs2_to_hold_btn_ = new QPushButton("OCS2 → HOLD");
  ocs2_to_hold_btn_->setStyleSheet("QPushButton { background-color: #FF9800; color: white; font-weight: bold; }");
  button_layout->addWidget(ocs2_to_hold_btn_);
  
  // HOLD → HOME (command = 1)
  hold_to_home_btn_ = new QPushButton("HOLD → HOME");
  hold_to_home_btn_->setStyleSheet("QPushButton { background-color: #9C27B0; color: white; font-weight: bold; }");
  button_layout->addWidget(hold_to_home_btn_);
  
  main_layout->addWidget(button_group);
  
  // Status display
  auto* status_group = new QGroupBox("Status");
  auto* status_layout = new QVBoxLayout(status_group);
  
  // Current state display
  current_state_label_ = new QLabel("当前状态: HOLD");
  current_state_label_->setStyleSheet("QLabel { padding: 5px; background-color: #e3f2fd; border: 1px solid #2196F3; font-weight: bold; }");
  status_layout->addWidget(current_state_label_);
  
  status_label_ = new QLabel("Ready");
  status_label_->setStyleSheet("QLabel { padding: 5px; background-color: #f0f0f0; border: 1px solid #ccc; }");
  status_layout->addWidget(status_label_);
  
  main_layout->addWidget(status_group);
  
  // Connect signals
  connect(home_to_hold_btn_, &QPushButton::clicked, this, &OCS2FSMPanel::onHomeToHold);
  connect(hold_to_ocs2_btn_, &QPushButton::clicked, this, &OCS2FSMPanel::onHoldToOCS2);
  connect(ocs2_to_hold_btn_, &QPushButton::clicked, this, &OCS2FSMPanel::onOCS2ToHold);
  connect(hold_to_home_btn_, &QPushButton::clicked, this, &OCS2FSMPanel::onHoldToHome);
  connect(switch_pose_btn_, &QPushButton::clicked, this, &OCS2FSMPanel::onSwitchPose);
  
  // 初始化按钮可见性（初始状态为HOLD）
  updateButtonVisibility();
  
  // 创建重置定时器
  reset_timer_ = new QTimer(this);
  reset_timer_->setSingleShot(true);
  reset_timer_->setInterval(100); // 100ms延迟
  connect(reset_timer_, &QTimer::timeout, this, &OCS2FSMPanel::onSwitchPoseReleased);
}

OCS2FSMPanel::~OCS2FSMPanel()
{
  // 析构函数实现
}

void OCS2FSMPanel::onInitialize()
{
  // Create ROS2 node
  node_ = rclcpp::Node::make_shared("ocs2_fsm_panel");
  
  // Create publisher
  publisher_ = node_->create_publisher<arms_ros2_control_msgs::msg::Inputs>(
    "/control_input", 10);
  
  RCLCPP_INFO(node_->get_logger(), "OCS2 FSM Panel initialized");
  updateStatusDisplay();
}

void OCS2FSMPanel::onHomeToHold()
{
  publishCommand(2);
  setCurrentState("HOLD");
  status_label_->setText("Sent: HOME → HOLD (command=2)");
}

void OCS2FSMPanel::onHoldToOCS2()
{
  publishCommand(3);
  setCurrentState("OCS2");
  status_label_->setText("Sent: HOLD → OCS2 (command=3)");
}

void OCS2FSMPanel::onOCS2ToHold()
{
  publishCommand(2);
  setCurrentState("HOLD");
  status_label_->setText("Sent: OCS2 → HOLD (command=2)");
}

void OCS2FSMPanel::onHoldToHome()
{
  publishCommand(1);
  setCurrentState("HOME");
  status_label_->setText("Sent: HOLD → HOME (command=1)");
}

void OCS2FSMPanel::onSwitchPose()
{
  if (!switch_pose_pressed_) {
    // 按下：发送command=4来触发姿态切换
    publishCommand(4);
    switch_pose_pressed_ = true;
    status_label_->setText("Sent: 切换姿态 (command=4)");
    
    // 启动定时器，100ms后自动"释放"
    reset_timer_->start();
  }
}

void OCS2FSMPanel::onSwitchPoseReleased()
{
  // 释放：发送command=0来重置防抖状态
  publishCommand(0);
  switch_pose_pressed_ = false;
  status_label_->setText("姿态切换完成，可以再次切换");
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
  status_label_->setText("Panel ready - Click buttons to change FSM state");
  updateButtonVisibility();
}

void OCS2FSMPanel::updateButtonVisibility()
{
  // 根据当前状态显示/隐藏相应的按钮
  if (current_state_ == "HOME") {
    // HOME状态可以转换到HOLD，也可以切换姿态
    home_to_hold_btn_->setVisible(true);
    home_to_hold_btn_->setEnabled(true);
    switch_pose_btn_->setVisible(true);
    switch_pose_btn_->setEnabled(true);
    hold_to_ocs2_btn_->setVisible(false);
    ocs2_to_hold_btn_->setVisible(false);
    hold_to_home_btn_->setVisible(false);
  } else if (current_state_ == "HOLD") {
    // HOLD状态可以转换到OCS2或HOME
    home_to_hold_btn_->setVisible(false);
    switch_pose_btn_->setVisible(false);
    hold_to_ocs2_btn_->setVisible(true);
    hold_to_ocs2_btn_->setEnabled(true);
    ocs2_to_hold_btn_->setVisible(false);
    hold_to_home_btn_->setVisible(true);
    hold_to_home_btn_->setEnabled(true);
  } else if (current_state_ == "OCS2") {
    // OCS2状态只能转换到HOLD
    home_to_hold_btn_->setVisible(false);
    switch_pose_btn_->setVisible(false);
    hold_to_ocs2_btn_->setVisible(false);
    ocs2_to_hold_btn_->setVisible(true);
    ocs2_to_hold_btn_->setEnabled(true);
    hold_to_home_btn_->setVisible(false);
  } else {
    // 未知状态，隐藏所有按钮
    home_to_hold_btn_->setVisible(false);
    switch_pose_btn_->setVisible(false);
    hold_to_ocs2_btn_->setVisible(false);
    ocs2_to_hold_btn_->setVisible(false);
    hold_to_home_btn_->setVisible(false);
  }
}

void OCS2FSMPanel::setCurrentState(const std::string& state)
{
  current_state_ = state;
  current_state_label_->setText(QString("当前状态: %1").arg(QString::fromStdString(state)));
  updateButtonVisibility();
}

void OCS2FSMPanel::load(const rviz_common::Config& config)
{
  // Load configuration data
  rviz_common::Panel::load(config);
}

void OCS2FSMPanel::save(rviz_common::Config config) const
{
  // Save configuration data
  rviz_common::Panel::save(config);
}

} // namespace arms_rviz_control_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(arms_rviz_control_plugin::OCS2FSMPanel, rviz_common::Panel)
