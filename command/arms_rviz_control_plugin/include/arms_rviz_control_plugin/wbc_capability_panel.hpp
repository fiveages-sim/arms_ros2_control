#pragma once

#include <memory>

#include <QLabel>
#include <QVBoxLayout>
#include <QGroupBox>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>

#include "arms_ros2_control_msgs/msg/wbc_capability.hpp"

namespace arms_rviz_control_plugin
{

  class WbcCapabilityPanel : public rviz_common::Panel
  {
    Q_OBJECT

public:
    explicit WbcCapabilityPanel(QWidget* parent = nullptr);
    ~WbcCapabilityPanel() override;

    void onInitialize() override;
    void load(const rviz_common::Config& config) override;
    void save(rviz_common::Config config) const override;

  private:
    void onCapabilityReceived(
        const arms_ros2_control_msgs::msg::WbcCapability::SharedPtr msg);

    void updateCapabilityLabel(QLabel* label, const QString& name, bool enabled);

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<arms_ros2_control_msgs::msg::WbcCapability>::SharedPtr capability_subscriber_;

    std::unique_ptr<QVBoxLayout> main_layout_;
    std::unique_ptr<QGroupBox> capability_group_;

    std::unique_ptr<QLabel> status_label_;
    std::unique_ptr<QLabel> mobile_base_label_;
    std::unique_ptr<QLabel> body_relative_label_;
    std::unique_ptr<QLabel> waist_lock_label_;
    std::unique_ptr<QLabel> bimanual_label_;
    std::unique_ptr<QLabel> body_tracking_label_;
  };

}  // namespace arms_rviz_control_plugin