#ifndef ARMS_RVIZ_CONTROL_PLUGIN_OCS2_FSM_PANEL_HPP
#define ARMS_RVIZ_CONTROL_PLUGIN_OCS2_FSM_PANEL_HPP

#include <memory>
#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <arms_ros2_control_msgs/msg/inputs.hpp>

namespace arms_rviz_control_plugin
{

class OCS2FSMPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  OCS2FSMPanel(QWidget* parent = nullptr);
  ~OCS2FSMPanel() override;

  void onInitialize() override;

  // Load and save configuration data
  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onHomeToHold();
  void onHoldToOCS2();
  void onOCS2ToHold();
  void onHoldToHome();
  void onSwitchPose();
  void onSwitchPoseReleased();

private:
  void publishCommand(int32_t command);
  void updateStatusDisplay();
  void updateButtonVisibility();
  void setCurrentState(const std::string& state);

  // ROS2
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<arms_ros2_control_msgs::msg::Inputs>::SharedPtr publisher_;

  // UI Elements
  QPushButton* home_to_hold_btn_;
  QPushButton* hold_to_ocs2_btn_;
  QPushButton* ocs2_to_hold_btn_;
  QPushButton* hold_to_home_btn_;
  QPushButton* switch_pose_btn_;
  QLabel* status_label_;
  QLabel* current_state_label_;

  // Current state tracking
  int32_t current_command_;
  std::string current_state_;
  
  // Switch pose button state
  bool switch_pose_pressed_;
  QTimer* reset_timer_;
};

} // namespace arms_rviz_control_plugin

#endif // ARMS_RVIZ_CONTROL_PLUGIN_OCS2_FSM_PANEL_HPP
