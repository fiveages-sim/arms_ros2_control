#ifndef ARMS_RVIZ_CONTROL_PLUGIN_JOINT_CONTROL_PANEL_HPP
#define ARMS_RVIZ_CONTROL_PLUGIN_JOINT_CONTROL_PANEL_HPP

#include <memory>
#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QDoubleSpinBox>
#include <QScrollArea>
#include <QComboBox>
#include <QPushButton>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <memory>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <arms_controller_common/utils/JointLimitsManager.h>

namespace arms_rviz_control_plugin
{

class JointControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  JointControlPanel(QWidget* parent = nullptr);
  ~JointControlPanel() override;

  void onInitialize() override;

  // Load and save configuration data
  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;

private Q_SLOTS:
  void onSendButtonClicked();
  void onCategoryChanged();

private:
  void onFsmCommandReceived(const std_msgs::msg::Int32::SharedPtr msg);
  void onJointStateReceived(const sensor_msgs::msg::JointState::SharedPtr msg);
  void updatePanelVisibility();
  void publishJointPositions();
  void updateJointValuesFromState();
  std::string classifyJoint(const std::string& joint_name);
  void updateJointVisibility();
  std::string getControllerNameForCategory(const std::string& category);
  void updatePublisher();
  void updateCategoryOptions();
  bool hasControllerForCategory(const std::string& category);
  bool shouldShowSendButton() const;
  void updateSpinboxRanges();
  void tryParseLimitsFromCache();
  void initializeJoints(const std::vector<std::string>& joint_names_source);

  // ROS2
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr fsm_command_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_position_publisher_;

  // Joint limits manager
  std::shared_ptr<arms_controller_common::JointLimitsManager> joint_limits_manager_;
  std::string robot_description_cache_;
  bool robot_description_received_ = false;

  // UI Elements
  std::unique_ptr<QGroupBox> joint_control_group_;
  std::unique_ptr<QVBoxLayout> joint_layout_;
  std::unique_ptr<QScrollArea> scroll_area_;
  std::unique_ptr<QHBoxLayout> category_layout_;
  std::unique_ptr<QComboBox> category_combo_;
  std::unique_ptr<QLabel> status_label_;
  std::unique_ptr<QPushButton> send_button_;
  std::vector<std::unique_ptr<QVBoxLayout>> joint_row_layouts_;
  std::vector<std::unique_ptr<QLabel>> joint_labels_;
  std::vector<std::unique_ptr<QDoubleSpinBox>> joint_spinboxes_;

  // Joint information
  std::vector<std::string> joint_names_;
  std::vector<double> joint_positions_;
  std::map<std::string, size_t> joint_name_to_index_;
  std::map<std::string, std::string> joint_to_category_;  // joint_name -> category
  std::map<std::string, std::vector<size_t>> category_to_joints_;  // category -> joint indices
  bool joints_initialized_ = false;
  std::string current_category_ = "all";

  // Control state
  int32_t current_command_ = 2;
  bool is_joint_control_enabled_ = false;

  // Available controllers
  std::vector<std::string> available_controllers_;
  std::set<std::string> available_categories_;
  std::map<std::string, std::string> category_to_controller_;  // category -> controller name
};

} // namespace arms_rviz_control_plugin

#endif // ARMS_RVIZ_CONTROL_PLUGIN_JOINT_CONTROL_PANEL_HPP

