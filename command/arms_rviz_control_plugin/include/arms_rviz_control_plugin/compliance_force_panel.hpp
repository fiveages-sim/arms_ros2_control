#ifndef ARMS_RVIZ_CONTROL_PLUGIN_COMPLIANCE_FORCE_PANEL_HPP
#define ARMS_RVIZ_CONTROL_PLUGIN_COMPLIANCE_FORCE_PANEL_HPP

#include <array>
#include <memory>
#include <string>

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>

#include <arms_ros2_control_msgs/msg/compliance_force_status.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace arms_rviz_control_plugin
{

/**
 * @brief RViz panel: live COMPLIANCE force setpoint vs measured force,
 *        and editable force-axis selection / setpoints.
 */
class ComplianceForcePanel : public rviz_common::Panel
{
    Q_OBJECT

public:
    explicit ComplianceForcePanel(QWidget* parent = nullptr);
    ~ComplianceForcePanel() override;

    void onInitialize() override;
    void load(const rviz_common::Config& config) override;
    void save(rviz_common::Config config) const override;

private Q_SLOTS:
    void onApplyClicked();
    void onSyncClicked();
    void onZeroWrenchClicked();
    void onUserEdited();

private:
    using StatusMsg = arms_ros2_control_msgs::msg::ComplianceForceStatus;

    void onStatus(const StatusMsg::SharedPtr msg);
    void applyUiToController();
    void syncUiFromStatus(const StatusMsg& msg);
    void updateMeasuredLabels(const StatusMsg& msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<StatusMsg>::SharedPtr status_sub_;
    rclcpp::AsyncParametersClient::SharedPtr param_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr zero_wrench_client_;

    QLabel* status_label_{nullptr};
    QLabel* frame_label_{nullptr};
    std::array<QCheckBox*, 6> force_axis_cb_{};
    std::array<QDoubleSpinBox*, 6> setpoint_spin_{};
    std::array<QLabel*, 6> meas_left_label_{};
    std::array<QLabel*, 6> meas_right_label_{};
    std::array<QLabel*, 6> err_left_label_{};
    QPushButton* apply_btn_{nullptr};
    QPushButton* sync_btn_{nullptr};
    QPushButton* zero_wrench_btn_{nullptr};

    bool ui_dirty_{false};
    bool have_status_{false};
    StatusMsg last_status_{};
};

}  // namespace arms_rviz_control_plugin

#endif  // ARMS_RVIZ_CONTROL_PLUGIN_COMPLIANCE_FORCE_PANEL_HPP
