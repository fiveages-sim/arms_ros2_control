#include "arms_rviz_control_plugin/compliance_force_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QGroupBox>
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QSaveFile>
#include <QSignalBlocker>
#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>

#include <algorithm>
#include <cmath>

namespace arms_rviz_control_plugin
{
    namespace
    {
        const char* kAxisNames[6] = {"Fx", "Fy", "Fz", "Mx", "My", "Mz"};
        const char* kAxisUnits[6] = {"N", "N", "N", "Nm", "Nm", "Nm"};
    }  // namespace

    PhaseIdentificationPlot::PhaseIdentificationPlot(QWidget* parent)
        : QWidget(parent)
    {
        setMinimumHeight(260);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    }

    void PhaseIdentificationPlot::clearData()
    {
        input_samples_.clear();
        output_samples_.clear();
        result_frequency_.clear();
        result_gain_.clear();
        result_phase_.clear();
        update();
    }

    void PhaseIdentificationPlot::setMode(uint8_t mode)
    {
        if (mode != 0) mode_ = mode;
        update();
    }

    void PhaseIdentificationPlot::appendSample(double input, double output)
    {
        input_samples_.push_back(input);
        output_samples_.push_back(output);
        constexpr size_t kMaxSamples = 600;
        if (input_samples_.size() > kMaxSamples)
        {
            input_samples_.erase(input_samples_.begin());
            output_samples_.erase(output_samples_.begin());
        }
        update();
    }

    void PhaseIdentificationPlot::appendResult(
        double frequency, double gain, double phase_deg)
    {
        result_frequency_.push_back(frequency);
        result_gain_.push_back(gain);
        result_phase_.push_back(phase_deg);
        update();
    }

    void PhaseIdentificationPlot::paintEvent(QPaintEvent*)
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        painter.fillRect(rect(), QColor(250, 250, 250));

        const QRectF time_rect(45, 24, width() - 60, height() * 0.48 - 30);
        const QRectF phase_rect(45, height() * 0.53, width() - 60, height() * 0.42 - 18);
        painter.setPen(QColor(90, 90, 90));
        painter.drawRect(time_rect);
        painter.drawRect(phase_rect);
        painter.drawText(6, 16, mode_ == 2
            ? "接触链路时域：实际位移(蓝) / 力(红)，各自归一化"
            : "关节位置环时域：q_cmd(蓝) / q_measured(红)，各自归一化");
        painter.drawText(6, static_cast<int>(height() * 0.53) - 5,
                         "频率响应：相位(绿，-180°..180°) / 增益(橙，归一化)");
        painter.drawText(5, static_cast<int>(phase_rect.top() + 8), "180°");
        painter.drawText(5, static_cast<int>(phase_rect.bottom()), "-180°");

        auto draw_series = [&](const std::vector<double>& values, const QColor& color) {
            if (values.size() < 2) return;
            double scale = 1e-12;
            for (double value : values) scale = std::max(scale, std::abs(value));
            QPainterPath path;
            for (size_t i = 0; i < values.size(); ++i)
            {
                const double x = time_rect.left() + time_rect.width() *
                    static_cast<double>(i) / static_cast<double>(values.size() - 1);
                const double y = time_rect.center().y() - 0.46 * time_rect.height() *
                    values[i] / scale;
                if (i == 0) path.moveTo(x, y); else path.lineTo(x, y);
            }
            painter.setPen(QPen(color, 1.5));
            painter.drawPath(path);
        };
        draw_series(input_samples_, QColor(25, 100, 210));
        draw_series(output_samples_, QColor(210, 50, 45));

        if (!result_frequency_.empty())
        {
            const auto [fmin_it, fmax_it] = std::minmax_element(
                result_frequency_.begin(), result_frequency_.end());
            const double log_min = std::log(std::max(*fmin_it, 1e-6));
            const double log_max = std::log(std::max(*fmax_it, 1e-6));
            painter.setPen(QPen(QColor(20, 145, 65), 2.0));
            QPointF previous;
            double max_gain = 1e-12;
            for (double gain : result_gain_) max_gain = std::max(max_gain, gain);
            QPainterPath gain_path;
            for (size_t i = 0; i < result_frequency_.size(); ++i)
            {
                const double ratio = std::abs(log_max - log_min) < 1e-9 ? 0.5 :
                    (std::log(std::max(result_frequency_[i], 1e-6)) - log_min) /
                    (log_max - log_min);
                const QPointF point(
                    phase_rect.left() + ratio * phase_rect.width(),
                    phase_rect.center().y() - result_phase_[i] / 360.0 * phase_rect.height());
                if (i > 0) painter.drawLine(previous, point);
                painter.drawEllipse(point, 3.0, 3.0);
                previous = point;
                const QPointF gain_point(
                    phase_rect.left() + ratio * phase_rect.width(),
                    phase_rect.bottom() - 0.9 * phase_rect.height() *
                        result_gain_[i] / max_gain);
                if (i == 0) gain_path.moveTo(gain_point); else gain_path.lineTo(gain_point);
            }
            painter.setPen(QPen(QColor(235, 135, 20), 1.5));
            painter.drawPath(gain_path);
            painter.setPen(QColor(60, 60, 60));
            painter.drawText(
                phase_rect.adjusted(5, 5, -5, -5), Qt::AlignRight | Qt::AlignTop,
                QString("最新: %1 Hz  gain=%2  phase=%3°")
                    .arg(result_frequency_.back(), 0, 'f', 2)
                    .arg(result_gain_.back(), 0, 'g', 4)
                    .arg(result_phase_.back(), 0, 'f', 1));
        }
    }

    ComplianceForcePanel::ComplianceForcePanel(QWidget* parent)
        : Panel(parent)
    {
        auto* main = new QVBoxLayout(this);
        main->setContentsMargins(6, 4, 6, 4);
        main->setSpacing(4);

        status_label_ = new QLabel("等待 COMPLIANCE 状态…", this);
        status_label_->setStyleSheet("QLabel { font-weight: bold; }");
        main->addWidget(status_label_);

        frame_label_ = new QLabel("frame: —", this);
        main->addWidget(frame_label_);

        auto* group = new QGroupBox("力控轴 / 设定力 / 实测力", this);
        auto* grid = new QGridLayout(group);
        grid->setHorizontalSpacing(6);
        grid->setVerticalSpacing(2);

        grid->addWidget(new QLabel("轴", group), 0, 0);
        grid->addWidget(new QLabel("力控", group), 0, 1);
        grid->addWidget(new QLabel("F_des", group), 0, 2);
        grid->addWidget(new QLabel("F_meas L", group), 0, 3);
        grid->addWidget(new QLabel("F_meas R", group), 0, 4);
        grid->addWidget(new QLabel("err L", group), 0, 5);

        for (int i = 0; i < 6; ++i)
        {
            auto* name = new QLabel(
                QString("%1 [%2]").arg(kAxisNames[i]).arg(kAxisUnits[i]), group);
            force_axis_cb_[i] = new QCheckBox(group);
            force_axis_cb_[i]->setToolTip("勾选 = 力控轴 (S=1)，取消 = 位控轴 (S=0)");

            setpoint_spin_[i] = new QDoubleSpinBox(group);
            setpoint_spin_[i]->setDecimals(2);
            setpoint_spin_[i]->setSingleStep(i < 3 ? 0.5 : 0.1);
            setpoint_spin_[i]->setRange(
                i < 3 ? -20.0 : -5.0, i < 3 ? 20.0 : 5.0);
            setpoint_spin_[i]->setMaximumWidth(90);

            meas_left_label_[i] = new QLabel("—", group);
            meas_right_label_[i] = new QLabel("—", group);
            err_left_label_[i] = new QLabel("—", group);
            meas_left_label_[i]->setMinimumWidth(56);
            meas_right_label_[i]->setMinimumWidth(56);
            err_left_label_[i]->setMinimumWidth(56);

            const int row = i + 1;
            grid->addWidget(name, row, 0);
            grid->addWidget(force_axis_cb_[i], row, 1, Qt::AlignCenter);
            grid->addWidget(setpoint_spin_[i], row, 2);
            grid->addWidget(meas_left_label_[i], row, 3);
            grid->addWidget(meas_right_label_[i], row, 4);
            grid->addWidget(err_left_label_[i], row, 5);

            connect(force_axis_cb_[i], &QCheckBox::toggled,
                    this, &ComplianceForcePanel::onUserEdited);
            connect(setpoint_spin_[i], QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                    this, &ComplianceForcePanel::onUserEdited);
        }

        main->addWidget(group);

        auto* btn_row = new QHBoxLayout();
        apply_btn_ = new QPushButton("应用设定", this);
        apply_btn_->setStyleSheet(
            "QPushButton { background-color: #2196F3; color: white; font-weight: bold; }");
        apply_btn_->setToolTip(
            "将力控轴选择与 F_des 写入 /ocs2_arm_controller 参数");
        sync_btn_ = new QPushButton("从控制器同步", this);
        sync_btn_->setToolTip("用最新 compliance_force_status 覆盖本面板编辑框");
        zero_wrench_btn_ = new QPushButton("传感器清零", this);
        zero_wrench_btn_->setToolTip(
            "重新标定重力补偿后的残余零偏；标定期间请保持双臂静止且末端无接触");
        btn_row->addWidget(apply_btn_);
        btn_row->addWidget(sync_btn_);
        btn_row->addWidget(zero_wrench_btn_);
        main->addLayout(btn_row);

        auto* ident_group = new QGroupBox("相位辨识", this);
        auto* ident_layout = new QVBoxLayout(ident_group);
        auto* ident_settings = new QHBoxLayout();
        identification_arm_combo_ = new QComboBox(ident_group);
        identification_arm_combo_->addItem("左臂", "left");
        identification_arm_combo_->addItem("右臂", "right");
        identification_joint_spin_ = new QSpinBox(ident_group);
        identification_joint_spin_->setRange(0, 15);
        identification_joint_amplitude_spin_ = new QDoubleSpinBox(ident_group);
        identification_joint_amplitude_spin_->setDecimals(4);
        identification_joint_amplitude_spin_->setRange(0.001, 0.05);
        identification_joint_amplitude_spin_->setSingleStep(0.002);
        identification_joint_amplitude_spin_->setValue(0.01);
        identification_joint_amplitude_spin_->setSuffix(" rad");
        ident_settings->addWidget(new QLabel("手臂", ident_group));
        ident_settings->addWidget(identification_arm_combo_);
        ident_settings->addWidget(new QLabel("关节", ident_group));
        ident_settings->addWidget(identification_joint_spin_);
        ident_settings->addWidget(new QLabel("幅值", ident_group));
        ident_settings->addWidget(identification_joint_amplitude_spin_);
        ident_layout->addLayout(ident_settings);
        auto* ident_buttons = new QHBoxLayout();
        position_identification_btn_ = new QPushButton("采集当前关节", ident_group);
        all_joint_identification_btn_ = new QPushButton("采集左右臂全部关节", ident_group);
        contact_identification_btn_ = new QPushButton("接触链路辨识", ident_group);
        stop_identification_btn_ = new QPushButton("停止辨识", ident_group);
        position_identification_btn_->setToolTip(
            "无接触条件下辨识关节位置下发 q_cmd 到实测 q_measured 的增益与相位");
        contact_identification_btn_->setToolTip(
            "保持接触条件下辨识实际位移到接触力的增益与相位");
        stop_identification_btn_->setStyleSheet(
            "QPushButton { background-color: #D9534F; color: white; }");
        ident_buttons->addWidget(position_identification_btn_);
        ident_buttons->addWidget(all_joint_identification_btn_);
        ident_buttons->addWidget(contact_identification_btn_);
        ident_buttons->addWidget(stop_identification_btn_);
        ident_layout->addLayout(ident_buttons);
        identification_plot_ = new PhaseIdentificationPlot(ident_group);
        ident_layout->addWidget(identification_plot_);
        identification_results_table_ = new QTableWidget(0, 5, ident_group);
        identification_results_table_->setHorizontalHeaderLabels(
            {"手臂", "关节", "频率[Hz]", "增益", "相位[deg]"});
        identification_results_table_->horizontalHeader()->setSectionResizeMode(
            QHeaderView::Stretch);
        identification_results_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
        identification_results_table_->setMaximumHeight(170);
        ident_layout->addWidget(identification_results_table_);
        export_identification_btn_ = new QPushButton("导出 Excel", ident_group);
        export_identification_btn_->setToolTip(
            "将当前逐轴辨识结果导出为 Excel 可直接打开的 UTF-8 CSV 文件");
        ident_layout->addWidget(export_identification_btn_);
        main->addWidget(ident_group);

        auto* hint = new QLabel(
            "仅在 COMPLIANCE 模式下有数据。F_meas 已含反馈符号约定。", this);
        hint->setWordWrap(true);
        hint->setStyleSheet("QLabel { color: #666; font-size: 11px; }");
        main->addWidget(hint);
        main->addStretch();

        connect(apply_btn_, &QPushButton::clicked, this, &ComplianceForcePanel::onApplyClicked);
        connect(sync_btn_, &QPushButton::clicked, this, &ComplianceForcePanel::onSyncClicked);
        connect(zero_wrench_btn_, &QPushButton::clicked,
                this, &ComplianceForcePanel::onZeroWrenchClicked);
        connect(position_identification_btn_, &QPushButton::clicked,
                this, &ComplianceForcePanel::onPositionIdentificationClicked);
        connect(all_joint_identification_btn_, &QPushButton::clicked,
                this, &ComplianceForcePanel::onAllJointIdentificationClicked);
        connect(contact_identification_btn_, &QPushButton::clicked,
                this, &ComplianceForcePanel::onContactIdentificationClicked);
        connect(stop_identification_btn_, &QPushButton::clicked,
                this, &ComplianceForcePanel::onStopIdentificationClicked);
        connect(export_identification_btn_, &QPushButton::clicked,
                this, &ComplianceForcePanel::onExportIdentificationClicked);

        status_watchdog_ = new QTimer(this);
        status_watchdog_->setSingleShot(true);
        status_watchdog_->setInterval(300);
        connect(status_watchdog_, &QTimer::timeout, this, [this]() {
            have_status_ = false;
            status_label_->setText(
                "COMPLIANCE 状态超时：当前不在该模式或控制器无响应");
            frame_label_->setText("frame: —");
            for (int i = 0; i < 6; ++i)
            {
                meas_left_label_[i]->setText("—");
                meas_right_label_[i]->setText("—");
                err_left_label_[i]->setText("—");
                meas_left_label_[i]->setStyleSheet("QLabel { color: #888; }");
                meas_right_label_[i]->setStyleSheet("QLabel { color: #888; }");
                err_left_label_[i]->setStyleSheet("QLabel { color: #888; }");
            }
        });
    }

    ComplianceForcePanel::~ComplianceForcePanel() = default;

    void ComplianceForcePanel::onInitialize()
    {
        node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
            node_, "/ocs2_arm_controller");
        zero_wrench_client_ = node_->create_client<std_srvs::srv::Trigger>(
            "/compliance_zero_wrench");
        position_identification_client_ = node_->create_client<std_srvs::srv::Trigger>(
            "/compliance_identify_position_loop");
        all_joint_identification_client_ = node_->create_client<std_srvs::srv::Trigger>(
            "/compliance_identify_all_joint_loops");
        contact_identification_client_ = node_->create_client<std_srvs::srv::Trigger>(
            "/compliance_identify_contact_chain");
        stop_identification_client_ = node_->create_client<std_srvs::srv::Trigger>(
            "/compliance_stop_identification");

        status_sub_ = node_->create_subscription<StatusMsg>(
            "compliance_force_status", 10,
            [this](const StatusMsg::SharedPtr msg)
            {
                // Marshal onto Qt thread
                QMetaObject::invokeMethod(
                    this,
                    [this, msg]() { onStatus(msg); },
                    Qt::QueuedConnection);
            });
    }

    void ComplianceForcePanel::load(const rviz_common::Config& config)
    {
        Panel::load(config);
    }

    void ComplianceForcePanel::save(rviz_common::Config config) const
    {
        Panel::save(config);
    }

    void ComplianceForcePanel::onUserEdited()
    {
        ui_dirty_ = true;
    }

    void ComplianceForcePanel::onApplyClicked()
    {
        applyUiToController();
    }

    void ComplianceForcePanel::onSyncClicked()
    {
        if (!have_status_)
        {
            return;
        }
        syncUiFromStatus(last_status_);
        ui_dirty_ = false;
    }

    void ComplianceForcePanel::onZeroWrenchClicked()
    {
        if (!zero_wrench_client_ || !zero_wrench_client_->service_is_ready())
        {
            status_label_->setText("清零失败: 请先进入 COMPLIANCE 模式");
            return;
        }

        (void)zero_wrench_client_->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>());
        status_label_->setText("传感器清零中…请保持机械臂静止且末端无接触");
    }

    void ComplianceForcePanel::onPositionIdentificationClicked()
    {
        if (!position_identification_client_ ||
            !position_identification_client_->service_is_ready())
        {
            status_label_->setText("位置环辨识失败: 请先进入 COMPLIANCE 模式");
            return;
        }
        identification_plot_->clearData();
        identification_results_table_->setRowCount(0);
        identification_plot_channel_.clear();
        last_identification_result_sequence_ =
            have_status_ ? last_status_.identification_result_sequence : 0;
        const std::string arm = identification_arm_combo_->currentData().toString().toStdString();
        const int joint = identification_joint_spin_->value();
        const double amplitude = identification_joint_amplitude_spin_->value();
        std::vector<rclcpp::Parameter> params = {
            rclcpp::Parameter("compliance_identification_arm", arm),
            rclcpp::Parameter("compliance_identification_joint", static_cast<int64_t>(joint)),
            rclcpp::Parameter("compliance_identification_amplitude_joint", amplitude)};
        (void)param_client_->set_parameters(
            params, [this](std::shared_future<
                              std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
                bool success = true;
                std::string reason;
                try
                {
                    for (const auto& result : future.get())
                    {
                        if (result.successful) continue;
                        success = false;
                        if (!reason.empty()) reason += "; ";
                        reason += result.reason;
                    }
                }
                catch (const std::exception& exception)
                {
                    success = false;
                    reason = exception.what();
                }
                if (success)
                {
                    (void)position_identification_client_->async_send_request(
                        std::make_shared<std_srvs::srv::Trigger::Request>());
                }
                else
                {
                    QMetaObject::invokeMethod(
                        this,
                        [this, reason]() {
                            status_label_->setText(
                                QString("位置环辨识参数写入失败: %1")
                                    .arg(QString::fromStdString(reason)));
                        },
                        Qt::QueuedConnection);
                }
            });
        status_label_->setText("正在写入当前关节辨识参数…");
    }

    void ComplianceForcePanel::onAllJointIdentificationClicked()
    {
        if (!all_joint_identification_client_ ||
            !all_joint_identification_client_->service_is_ready())
        {
            status_label_->setText("全关节采集失败: 请先进入 COMPLIANCE 模式");
            return;
        }
        identification_plot_->clearData();
        identification_results_table_->setRowCount(0);
        identification_plot_channel_.clear();
        last_identification_result_sequence_ =
            have_status_ ? last_status_.identification_result_sequence : 0;
        const double amplitude = identification_joint_amplitude_spin_->value();
        (void)param_client_->set_parameters(
            {rclcpp::Parameter("compliance_identification_amplitude_joint", amplitude)},
            [this](std::shared_future<
                       std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
                bool success = true;
                std::string reason;
                try
                {
                    for (const auto& result : future.get())
                    {
                        if (result.successful) continue;
                        success = false;
                        if (!reason.empty()) reason += "; ";
                        reason += result.reason;
                    }
                }
                catch (const std::exception& exception)
                {
                    success = false;
                    reason = exception.what();
                }
                if (success)
                {
                    (void)all_joint_identification_client_->async_send_request(
                        std::make_shared<std_srvs::srv::Trigger::Request>());
                }
                else
                {
                    QMetaObject::invokeMethod(
                        this,
                        [this, reason]() {
                            status_label_->setText(
                                QString("全关节辨识参数写入失败: %1")
                                    .arg(QString::fromStdString(reason)));
                        },
                        Qt::QueuedConnection);
                }
            });
        status_label_->setText("正在写入全关节辨识参数…");
    }

    void ComplianceForcePanel::onContactIdentificationClicked()
    {
        if (!contact_identification_client_ ||
            !contact_identification_client_->service_is_ready())
        {
            status_label_->setText("接触链路辨识失败: 请先进入 COMPLIANCE 模式");
            return;
        }
        identification_plot_->clearData();
        last_identification_result_sequence_ =
            have_status_ ? last_status_.identification_result_sequence : 0;
        (void)contact_identification_client_->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>());
        status_label_->setText("接触链路辨识请求已发送；请保持稳定接触");
    }

    void ComplianceForcePanel::onStopIdentificationClicked()
    {
        if (stop_identification_client_ && stop_identification_client_->service_is_ready())
        {
            (void)stop_identification_client_->async_send_request(
                std::make_shared<std_srvs::srv::Trigger::Request>());
            status_label_->setText("正在停止辨识并返回起始位姿…");
        }
    }

    void ComplianceForcePanel::onExportIdentificationClicked()
    {
        if (identification_results_table_->rowCount() == 0)
        {
            status_label_->setText("导出失败: 当前没有辨识结果");
            return;
        }

        const QString default_name = QDir::homePath() +
            "/compliance_phase_results_" +
            QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + ".csv";
        QString file_name = QFileDialog::getSaveFileName(
            this, "导出相位辨识结果", default_name,
            "Excel CSV (*.csv);;所有文件 (*)");
        if (file_name.isEmpty()) return;
        if (!file_name.endsWith(".csv", Qt::CaseInsensitive)) file_name += ".csv";

        QSaveFile file(file_name);
        if (!file.open(QIODevice::WriteOnly))
        {
            status_label_->setText("导出失败: 无法写入所选文件");
            return;
        }

        // UTF-8 BOM makes Excel recognize Chinese column names without an
        // import wizard. Quote every field so the CSV remains unambiguous.
        QByteArray csv("\xEF\xBB\xBF");
        csv += QStringLiteral("\"手臂\",\"关节\",\"频率[Hz]\",\"增益\",\"相位[deg]\"\r\n")
                   .toUtf8();
        for (int row = 0; row < identification_results_table_->rowCount(); ++row)
        {
            QStringList fields;
            for (int column = 0; column < identification_results_table_->columnCount(); ++column)
            {
                const auto* item = identification_results_table_->item(row, column);
                QString value = item ? item->text() : QString();
                value.replace('"', "\"\"");
                fields.push_back('"' + value + '"');
            }
            csv += (fields.join(',') + "\r\n").toUtf8();
        }

        if (file.write(csv) != csv.size() || !file.commit())
        {
            status_label_->setText("导出失败: 文件写入未完成");
            return;
        }
        status_label_->setText(QString("辨识结果已导出: %1").arg(file_name));
    }

    void ComplianceForcePanel::onStatus(const StatusMsg::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }
        last_status_ = *msg;
        have_status_ = true;
        if (status_watchdog_) status_watchdog_->start();
        const double force_limit = std::clamp(
            std::isfinite(msg->force_setpoint_limit)
                ? msg->force_setpoint_limit : 20.0,
            0.1, 20.0);
        const double torque_limit = std::clamp(
            std::isfinite(msg->torque_setpoint_limit)
                ? msg->torque_setpoint_limit : 5.0,
            0.01, 5.0);
        for (int i = 0; i < 6; ++i)
        {
            const double limit = i < 3 ? force_limit : torque_limit;
            QSignalBlocker blocker(setpoint_spin_[i]);
            setpoint_spin_[i]->setRange(-limit, limit);
        }
        const int joint_count = identification_arm_combo_->currentData().toString() == "right"
            ? msg->right_joint_count : msg->left_joint_count;
        identification_joint_spin_->setMaximum(std::max(0, joint_count - 1));

        const QString cal = QString("tare L=%1 R=%2")
                                .arg(msg->left_tare_valid ? "OK" : "--")
                                .arg(msg->right_tare_valid ? "OK" : "--");
        const QString ft = QString("FT L=%1 R=%2")
                               .arg(msg->left_ft_active ? "on" : "off")
                               .arg(msg->right_ft_active ? "on" : "off");
        const QString ready = QString("force L=%1 R=%2")
                                  .arg(msg->left_force_ready ? "ready" : "off")
                                  .arg(msg->right_force_ready ? "ready" : "off");
        QString text = QString("COMPLIANCE  |  %1  |  %2  |  %3")
                           .arg(cal, ft, ready);
        if (msg->identification_active)
        {
            const QString channel = msg->identification_mode == 1
                ? QString("J%1").arg(msg->identification_joint)
                : QString("axis%1").arg(msg->identification_axis);
            text += QString("  |  辨识 %1 %2% @ %3 Hz")
                .arg(channel)
                .arg(msg->identification_progress * 100.0, 0, 'f', 0)
                .arg(msg->identification_frequency, 0, 'f', 2);
        }
        else if (!msg->identification_message.empty() &&
                 msg->identification_message != "idle")
        {
            text += QString("  |  %1").arg(
                QString::fromStdString(msg->identification_message));
        }
        if (ui_dirty_)
        {
            text += "  |  已修改(未应用)";
        }
        status_label_->setText(text);
        frame_label_->setText(
            QString("frame: %1   sign=%2")
                .arg(QString::fromStdString(msg->header.frame_id))
                .arg(msg->force_feedback_sign, 0, 'f', 0));

        updateMeasuredLabels(*msg);

        identification_plot_->setMode(msg->identification_mode);
        if (msg->identification_active)
        {
            const std::string channel = msg->identification_arm + ":" +
                std::to_string(msg->identification_mode == 1
                    ? msg->identification_joint : msg->identification_axis);
            if (channel != identification_plot_channel_)
            {
                identification_plot_channel_ = channel;
                identification_plot_->clearData();
            }
            identification_plot_->appendSample(
                msg->identification_input, msg->identification_output);
        }
        if (msg->identification_result_valid &&
            msg->identification_result_sequence != last_identification_result_sequence_)
        {
            last_identification_result_sequence_ = msg->identification_result_sequence;
            identification_plot_->appendResult(
                msg->identification_result_frequency,
                msg->identification_gain,
                msg->identification_phase_deg);
            if (msg->identification_result_mode == 1)
            {
                const int row = identification_results_table_->rowCount();
                identification_results_table_->insertRow(row);
                identification_results_table_->setItem(row, 0, new QTableWidgetItem(
                    QString::fromStdString(msg->identification_result_arm)));
                identification_results_table_->setItem(row, 1, new QTableWidgetItem(
                    QString::number(msg->identification_result_joint)));
                identification_results_table_->setItem(row, 2, new QTableWidgetItem(
                    QString::number(msg->identification_result_frequency, 'f', 3)));
                identification_results_table_->setItem(row, 3, new QTableWidgetItem(
                    QString::number(msg->identification_gain, 'g', 5)));
                identification_results_table_->setItem(row, 4, new QTableWidgetItem(
                    QString::number(msg->identification_phase_deg, 'f', 2)));
                identification_results_table_->scrollToBottom();
            }
        }

        if (!ui_dirty_)
        {
            syncUiFromStatus(*msg);
        }
    }

    void ComplianceForcePanel::updateMeasuredLabels(const StatusMsg& msg)
    {
        for (int i = 0; i < 6; ++i)
        {
            const double des = msg.force_setpoint[i];
            const double ml = msg.force_measured_left[i];
            const double mr = msg.force_measured_right[i];
            meas_left_label_[i]->setText(QString::number(ml, 'f', 2));
            meas_right_label_[i]->setText(QString::number(mr, 'f', 2));
            err_left_label_[i]->setText(QString::number(des - ml, 'f', 2));

            const bool force_axis = msg.task_selection[i] > 0.5;
            const QString style = force_axis
                                      ? "QLabel { color: #1565C0; font-weight: bold; }"
                                      : "QLabel { color: #888; }";
            meas_left_label_[i]->setStyleSheet(style);
            meas_right_label_[i]->setStyleSheet(style);
            err_left_label_[i]->setStyleSheet(style);
        }
    }

    void ComplianceForcePanel::syncUiFromStatus(const StatusMsg& msg)
    {
        for (int i = 0; i < 6; ++i)
        {
            QSignalBlocker b1(force_axis_cb_[i]);
            QSignalBlocker b2(setpoint_spin_[i]);
            force_axis_cb_[i]->setChecked(msg.task_selection[i] > 0.5);
            setpoint_spin_[i]->setValue(msg.force_setpoint[i]);
        }
        ui_dirty_ = false;
    }

    void ComplianceForcePanel::applyUiToController()
    {
        if (!param_client_)
        {
            RCLCPP_WARN(node_->get_logger(), "ComplianceForcePanel: param client missing");
            return;
        }
        if (!param_client_->service_is_ready())
        {
            RCLCPP_WARN(node_->get_logger(),
                        "ComplianceForcePanel: /ocs2_arm_controller params not ready");
            status_label_->setText("应用失败: 控制器参数服务不可用");
            return;
        }

        std::vector<double> selection(6, 0.0);
        std::vector<double> setpoint(6, 0.0);
        for (int i = 0; i < 6; ++i)
        {
            selection[i] = force_axis_cb_[i]->isChecked() ? 1.0 : 0.0;
            setpoint[i] = setpoint_spin_[i]->value();
        }

        std::vector<rclcpp::Parameter> params = {
            rclcpp::Parameter("compliance_task_selection", selection),
            rclcpp::Parameter("compliance_force_setpoint", setpoint),
        };
        status_label_->setText("正在应用设定…");
        (void)param_client_->set_parameters(
            params,
            [this, selection, setpoint](std::shared_future<
                std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
                bool success = true;
                std::string reason;
                try
                {
                    for (const auto& result : future.get())
                    {
                        if (result.successful) continue;
                        success = false;
                        if (!reason.empty()) reason += "; ";
                        reason += result.reason;
                    }
                }
                catch (const std::exception& exception)
                {
                    success = false;
                    reason = exception.what();
                }
                QMetaObject::invokeMethod(
                    this,
                    [this, selection, setpoint, success, reason]() {
                        if (!success)
                        {
                            ui_dirty_ = true;
                            status_label_->setText(
                                QString("应用失败: %1")
                                    .arg(QString::fromStdString(reason)));
                            return;
                        }
                        ui_dirty_ = false;
                        status_label_->setText("COMPLIANCE  |  已应用设定");
                        RCLCPP_INFO(
                            node_->get_logger(),
                            "ComplianceForcePanel applied: "
                            "S=[%.0f %.0f %.0f %.0f %.0f %.0f] "
                            "F_des=[%.2f %.2f %.2f %.2f %.2f %.2f]",
                            selection[0], selection[1], selection[2],
                            selection[3], selection[4], selection[5],
                            setpoint[0], setpoint[1], setpoint[2],
                            setpoint[3], setpoint[4], setpoint[5]);
                    },
                    Qt::QueuedConnection);
            });
    }

}  // namespace arms_rviz_control_plugin

PLUGINLIB_EXPORT_CLASS(arms_rviz_control_plugin::ComplianceForcePanel, rviz_common::Panel)
