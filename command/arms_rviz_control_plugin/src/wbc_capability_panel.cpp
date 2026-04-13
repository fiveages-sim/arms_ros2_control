#include "arms_rviz_control_plugin/wbc_capability_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace arms_rviz_control_plugin
{

WbcCapabilityPanel::WbcCapabilityPanel(QWidget* parent)
    : rviz_common::Panel(parent)
{
    main_layout_ = std::make_unique<QVBoxLayout>(this);

    capability_group_ = std::make_unique<QGroupBox>("WBC Capabilities", this);
    auto* group_layout = new QVBoxLayout(capability_group_.get());

    status_label_ = std::make_unique<QLabel>("Waiting for capability message...", capability_group_.get());
    status_label_->setStyleSheet("QLabel { color: #666666; font-style: italic; }");
    group_layout->addWidget(status_label_.get());

    mobile_base_label_ = std::make_unique<QLabel>("Has Mobile Base: Unknown", capability_group_.get());
    body_relative_label_ = std::make_unique<QLabel>("Has Body Relative Constraint: Unknown", capability_group_.get());
    waist_lock_label_ = std::make_unique<QLabel>("Has Waist Lock: Unknown", capability_group_.get());
    bimanual_label_ = std::make_unique<QLabel>("Has Bimanual Coupling: Unknown", capability_group_.get());
    body_tracking_label_ = std::make_unique<QLabel>("Body Tracking EE Enabled: Unknown", capability_group_.get());

    group_layout->addWidget(mobile_base_label_.get());
    group_layout->addWidget(body_relative_label_.get());
    group_layout->addWidget(waist_lock_label_.get());
    group_layout->addWidget(bimanual_label_.get());
    group_layout->addWidget(body_tracking_label_.get());

    main_layout_->addWidget(capability_group_.get());
}

WbcCapabilityPanel::~WbcCapabilityPanel() = default;

void WbcCapabilityPanel::onInitialize()
{
    node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

    capability_subscriber_ =
        node_->create_subscription<arms_ros2_control_msgs::msg::WbcCapability>(
            "/ocs2_wbc_controller/wbc_capabilities",
            rclcpp::QoS(1).transient_local(),
            std::bind(&WbcCapabilityPanel::onCapabilityReceived, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(),
                "WbcCapabilityPanel initialized, subscribed to /ocs2_wbc_controller/wbc_capabilities");
}

void WbcCapabilityPanel::onCapabilityReceived(
    const arms_ros2_control_msgs::msg::WbcCapability::SharedPtr msg)
{
    if (!msg)
    {
        return;
    }

    status_label_->setText("Capability message received");
    status_label_->setStyleSheet("QLabel { color: #2e7d32; font-weight: bold; }");

    updateCapabilityLabel(mobile_base_label_.get(), "Has Mobile Base", msg->has_mobile_base);
    updateCapabilityLabel(body_relative_label_.get(), "Has Body Relative Constraint", msg->has_body_relative_constraint);
    updateCapabilityLabel(waist_lock_label_.get(), "Has Waist Lock", msg->has_waist_lock);
    updateCapabilityLabel(bimanual_label_.get(), "Has Bimanual Coupling", msg->has_bimanual_coupling);
    updateCapabilityLabel(body_tracking_label_.get(), "Body Tracking EE Enabled", msg->body_tracking_ee_enabled);
}

void WbcCapabilityPanel::updateCapabilityLabel(QLabel* label, const QString& name, bool enabled)
{
    if (!label)
    {
        return;
    }

    label->setText(QString("%1: %2").arg(name, enabled ? "Yes" : "No"));
    label->setStyleSheet(
        enabled
            ? "QLabel { color: #2e7d32; font-weight: bold; }"
            : "QLabel { color: #c62828; font-weight: bold; }");
}

void WbcCapabilityPanel::load(const rviz_common::Config& config)
{
    rviz_common::Panel::load(config);
}

void WbcCapabilityPanel::save(rviz_common::Config config) const
{
    rviz_common::Panel::save(config);
}

}  // namespace arms_rviz_control_plugin

PLUGINLIB_EXPORT_CLASS(arms_rviz_control_plugin::WbcCapabilityPanel, rviz_common::Panel)