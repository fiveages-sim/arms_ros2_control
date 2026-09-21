#ifndef ARMS_RVIZ_CONTROL_PLUGIN_COMPLIANCE_FORCE_PANEL_HPP
#define ARMS_RVIZ_CONTROL_PLUGIN_COMPLIANCE_FORCE_PANEL_HPP

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QProcess>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>

#include <arms_ros2_control_msgs/msg/compliance_force_status.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace arms_rviz_control_plugin
{

class PhaseIdentificationPlot : public QWidget
{
public:
    explicit PhaseIdentificationPlot(QWidget* parent = nullptr);
    void clearData();
    void appendSample(double input, double output);
    void appendResult(double frequency, double gain, double phase_deg);
    void setMode(uint8_t mode);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    uint8_t mode_{0};
    std::vector<double> input_samples_;
    std::vector<double> output_samples_;
    std::vector<double> result_frequency_;
    std::vector<double> result_gain_;
    std::vector<double> result_phase_;
};

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
    void onPositionIdentificationClicked();
    void onAllJointIdentificationClicked();
    void onContactIdentificationClicked();
    void onStopIdentificationClicked();
    void onExportIdentificationClicked();
    void onUserEdited();

private:
    using StatusMsg = arms_ros2_control_msgs::msg::ComplianceForceStatus;

    void onStatus(const StatusMsg::SharedPtr msg);
    void applyUiToController();
    void syncUiFromStatus(const StatusMsg& msg);
    void updateMeasuredLabels(const StatusMsg& msg);
    void updateAlignLabels(const StatusMsg& msg);
    QWidget* makePayloadWidget();
    void startPayloadIdentification();
    void stopPayloadIdentification();
    void readPayloadOutput();
    void setPayloadRunning(bool running);

    QProcess* payload_process_{nullptr};
    QComboBox* payload_arm_combo_{nullptr};
    QLineEdit* payload_wrench_topic_{nullptr};
    QLineEdit* payload_gravity_frame_{nullptr};
    QLineEdit* payload_joint_topic_{nullptr};
    QSpinBox* payload_pose_count_{nullptr};
    QPushButton* payload_start_btn_{nullptr};
    QPushButton* payload_sample_btn_{nullptr};
    QPushButton* payload_stop_btn_{nullptr};
    QPlainTextEdit* payload_log_{nullptr};
    QByteArray payload_stdout_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<StatusMsg>::SharedPtr status_sub_;
    rclcpp::AsyncParametersClient::SharedPtr param_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr zero_wrench_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr position_identification_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr all_joint_identification_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr contact_identification_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_identification_client_;

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

    // ── 曲面贴合（力位混合曲面追踪）：力矩驱动的目标姿态偏置外环 ──
    QCheckBox* align_enable_cb_{nullptr};
    QDoubleSpinBox* align_max_spin_{nullptr};       // UI 单位 deg，写 compliance_align_max (rad)
    QDoubleSpinBox* align_release_spin_{nullptr};   // [1/s]
    QCheckBox* align_rcc_cb_{nullptr};
    QLabel* align_left_label_{nullptr};
    QLabel* align_right_label_{nullptr};
    QLabel* align_hint_label_{nullptr};
    QPushButton* position_identification_btn_{nullptr};
    QPushButton* all_joint_identification_btn_{nullptr};
    QPushButton* contact_identification_btn_{nullptr};
    QPushButton* stop_identification_btn_{nullptr};
    QPushButton* export_identification_btn_{nullptr};
    PhaseIdentificationPlot* identification_plot_{nullptr};
    QComboBox* identification_arm_combo_{nullptr};
    QSpinBox* identification_joint_spin_{nullptr};
    QDoubleSpinBox* identification_joint_amplitude_spin_{nullptr};
    QTableWidget* identification_results_table_{nullptr};
    QTimer* status_watchdog_{nullptr};
    uint32_t last_identification_result_sequence_{0};
    std::string identification_plot_channel_;

    bool ui_dirty_{false};
    bool have_status_{false};
    StatusMsg last_status_{};
};

}  // namespace arms_rviz_control_plugin

#endif  // ARMS_RVIZ_CONTROL_PLUGIN_COMPLIANCE_FORCE_PANEL_HPP
