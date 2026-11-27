#ifndef ARMS_RVIZ_CONTROL_PLUGIN_COMPLIANCE_FORCE_PANEL_HPP
#define ARMS_RVIZ_CONTROL_PLUGIN_COMPLIANCE_FORCE_PANEL_HPP

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFont>
#include <QLabel>
#include <QPushButton>
#include <QResizeEvent>
#include <QSpinBox>
#include <QStyle>
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

#include "arms_rviz_control_plugin/compliance_path_widget.hpp"

namespace arms_rviz_control_plugin
{

/// 当前面板缩放（1.0 = 基准），由 ComplianceForcePanel::applyUiScale() 更新。
double uiScale();
/// 随 uiScale 一起缩放的等宽字体（表格/读数用）。
QFont monoUiFont();

/// 次要说明文字的配色分级；字号由面板样式表里的 @SMALL@ 统一控制，
/// 会跟着 ComplianceForcePanel::resizeEvent() 的面板宽度一起缩放。
inline void setHintRole(QLabel* label, const char* role)
{
    if (!label) return;
    label->setProperty("role", QString::fromLatin1(role));
    if (auto* style = label->style())
    {
        style->unpolish(label);
        style->polish(label);
    }
    label->update();
}

/**
 * @brief 按钮外观变体。样式规则在 ComplianceForcePanel 的样式表里按
 *        QPushButton[variant="…"] 匹配，所有按钮因此共用同一套圆角/内边距/字号，
 *        只有配色分级不同：
 *          primary  —— 主操作（应用设定 / 下发轨迹 / 开始负载辨识）
 *          danger   —— 可逆的危险操作（传感器清零 / 停止辨识 / 停止采样）
 *          critical —— 立即停止运动（停止轨迹），实心红，最醒目
 */
inline void applyButtonVariant(QPushButton* button, const char* variant)
{
    if (!button) return;
    button->setProperty("variant", QString::fromLatin1(variant));
    button->setCursor(Qt::PointingHandCursor);
    button->setMinimumHeight(26);
    if (auto* style = button->style())
    {
        style->unpolish(button);
        style->polish(button);
    }
    button->update();
}

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

protected:
    /// 面板尺寸变化时按宽度重算界面字号（夹在 85%~150%，5% 步进）。
    void resizeEvent(QResizeEvent* event) override;

private Q_SLOTS:
    void onApplyClicked();
    void onSyncClicked();    void onZeroWrenchClicked();
    void onPositionIdentificationClicked();
    void onAllJointIdentificationClicked();
    void onContactIdentificationClicked();
    void onStopIdentificationClicked();
    void onExportIdentificationClicked();
    void onUserEdited();

private:
    using StatusMsg = arms_ros2_control_msgs::msg::ComplianceForceStatus;

    /// 启动时把本面板从 rviz 默认的左侧停靠区搬到右侧（rviz2 无法用配置文件表达）。
    void dockToRightAreaOnce();
    void onStatus(const StatusMsg::SharedPtr msg);
    void applyUiToController();
    void syncUiFromStatus(const StatusMsg& msg);
    /// 按当前宽度重算字号并重新应用等宽字体；尺寸没变就什么都不做。
    void applyUiScale();
    void updateMeasuredLabels(const StatusMsg& msg);
    void updateAlignLabels(const StatusMsg& msg);
    QWidget* makePayloadWidget();
    void startPayloadIdentification();
    void stopPayloadIdentification();
    void readPayloadOutput();
    void setPayloadRunning(bool running);

    QWidget* makePathWidget();

    /// 轨迹编辑器（compliance_path）。onInitialize() 里拿到 ROS 节点后 initialize()。
    CompliancePathWidget* path_widget_{nullptr};

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
    /// 实时遥测行（20 Hz 刷新）。与 status_label_（瞬时提示浮层）分开：
    /// 以前两者共用一个 label，操作反馈 50 ms 后就被遥测刷掉，等于看不见。
    QLabel* telemetry_label_{nullptr};
    /// 状态徽尘：COMPLIANCE（绿） / 无数据（灰）。
    QLabel* state_badge_{nullptr};
    QTimer* notice_watchdog_{nullptr};
    QString last_notice_text_;
    QElapsedTimer last_notice_timer_;
    std::array<QLabel*, 6> axis_name_label_{};
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
    /// 当前界面缩放百分比（100 = 基准宽度下的原始字号）。
    int ui_scale_percent_{100};
};

}  // namespace arms_rviz_control_plugin

#endif  // ARMS_RVIZ_CONTROL_PLUGIN_COMPLIANCE_FORCE_PANEL_HPP
