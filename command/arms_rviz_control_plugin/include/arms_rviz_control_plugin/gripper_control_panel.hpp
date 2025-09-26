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
  void onLeftGripperToggle();
  void onRightGripperToggle();

private:
  void onGripperCommandReceived(const arms_ros2_control_msgs::msg::Gripper::SharedPtr msg);
  void publishGripperCommand(bool open, int32_t arm_id);
  void updateGripperDisplay();
  void updateButtonVisibility();



  // ROS2
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<arms_ros2_control_msgs::msg::Gripper>::SharedPtr gripper_publisher_;
  rclcpp::Subscription<arms_ros2_control_msgs::msg::Gripper>::SharedPtr gripper_subscription_;


  // UI Elements
  QPushButton* left_gripper_btn_;
  QPushButton* right_gripper_btn_;
  QLabel* no_controller_label_;

  // Robot configuration
  bool is_dual_arm_mode_;
  std::vector<std::string> hand_controllers_;

  // Gripper state tracking
  bool left_gripper_open_;   // Left arm gripper state
  bool right_gripper_open_;  // Right arm gripper state
};

} // namespace arms_rviz_control_plugin

#endif // ARMS_RVIZ_CONTROL_PLUGIN_GRIPPER_CONTROL_PANEL_HPP