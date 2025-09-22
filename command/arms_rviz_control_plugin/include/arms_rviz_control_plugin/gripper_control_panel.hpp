#ifndef ARMS_RVIZ_CONTROL_PLUGIN_GRIPPER_CONTROL_PANEL_HPP
#define ARMS_RVIZ_CONTROL_PLUGIN_GRIPPER_CONTROL_PANEL_HPP

#include <memory>
#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QTimer>
#include <string>
#include <fstream>
#include <regex>
#include <cstdlib>
#include <filesystem>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <arms_ros2_control_msgs/msg/gripper.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>

namespace arms_rviz_control_plugin
{

class GripperControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  GripperControlPanel(QWidget* parent = nullptr);
  ~GripperControlPanel() override;

  void onInitialize() override;

  // Load and save configuration data
  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onGripperToggle();
  void onLeftGripperToggle();  // For dual-arm mode
  void onRightGripperToggle(); // For dual-arm mode

private:
  void onGripperCommandReceived(const arms_ros2_control_msgs::msg::Gripper::SharedPtr msg);
  void publishGripperCommand(bool open);
  void publishGripperCommand(bool open, int32_t arm_id); // For dual-arm mode
  void updateGripperDisplay();
  void updateDualArmGripperDisplay(); // For dual-arm mode

  // Robot configuration detection
  bool detectDualArmMode();
  std::string getCurrentRobotName();
  std::string findRobotControllerConfig(const std::string& robot_name);
  bool analyzeControllerConfig(const std::string& config_path);
  std::string getRobotConfigPath();

  // UI creation methods
  void createSingleArmUI(QVBoxLayout* main_layout);
  void createDualArmUI(QVBoxLayout* main_layout);

  // ROS2
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<arms_ros2_control_msgs::msg::Gripper>::SharedPtr gripper_publisher_;
  rclcpp::Subscription<arms_ros2_control_msgs::msg::Gripper>::SharedPtr gripper_subscription_;

  // UI Elements - Single arm mode
  QPushButton* gripper_toggle_btn_;
  QLabel* gripper_status_label_;
  QLabel* gripper_state_label_;
  QLabel* arm_id_label_;

  // UI Elements - Dual arm mode
  QPushButton* left_gripper_btn_;
  QPushButton* right_gripper_btn_;
  QLabel* left_gripper_state_label_;
  QLabel* right_gripper_state_label_;
  QLabel* left_status_label_;
  QLabel* right_status_label_;

  // Layout containers
  QGroupBox* single_arm_group_;
  QGroupBox* dual_arm_group_;

  // ROS2 event processing timer
  QTimer* ros_timer_;

  // Robot configuration
  bool is_dual_arm_mode_;

  // Gripper state tracking - Single arm mode
  int32_t current_gripper_target_;  // 0=close, 1=open
  bool gripper_command_received_;
  int32_t arm_id_;  // For single-arm robot, default is 1 (left arm)

  // Gripper state tracking - Dual arm mode
  bool left_gripper_open_;   // Left arm gripper state
  bool right_gripper_open_;  // Right arm gripper state
  bool left_gripper_command_received_;
  bool right_gripper_command_received_;
};

} // namespace arms_rviz_control_plugin

#endif // ARMS_RVIZ_CONTROL_PLUGIN_GRIPPER_CONTROL_PANEL_HPP