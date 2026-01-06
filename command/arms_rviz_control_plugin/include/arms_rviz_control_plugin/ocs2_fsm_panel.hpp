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
#include <std_msgs/msg/int32.hpp>

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
  void onHoldToMoveJ();
  void onMoveJToHold();
  void onSwitchPose();
  void onSwitchPoseReleased();

private:
  void publishCommand(int32_t command);
  void updateStatusDisplay();
  void updateButtonVisibility();
  void setCurrentState(const std::string& state);
  void onFsmCommandReceived(const std_msgs::msg::Int32::SharedPtr msg);

  // ROS2
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr fsm_command_publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr fsm_command_subscriber_;

  // UI Elements
  std::unique_ptr<QPushButton> home_to_hold_btn_;
  std::unique_ptr<QPushButton> hold_to_ocs2_btn_;
  std::unique_ptr<QPushButton> ocs2_to_hold_btn_;
  std::unique_ptr<QPushButton> hold_to_home_btn_;
  std::unique_ptr<QPushButton> hold_to_movej_btn_;
  std::unique_ptr<QPushButton> movej_to_hold_btn_;
  std::unique_ptr<QPushButton> switch_pose_btn_;
  std::unique_ptr<QLabel> current_state_label_;

  // Current state tracking
  int32_t current_command_ = 0;
  std::string current_state_ = "HOLD";
  
  // Switch pose button state
  bool switch_pose_pressed_ = false;
  std::unique_ptr<QTimer> reset_timer_;
};

} // namespace arms_rviz_control_plugin

#endif // ARMS_RVIZ_CONTROL_PLUGIN_OCS2_FSM_PANEL_HPP
