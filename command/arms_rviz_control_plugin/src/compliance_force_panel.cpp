#include "arms_rviz_control_plugin/compliance_force_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <QGridLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QSignalBlocker>

namespace arms_rviz_control_plugin
{
    namespace
    {
        const char* kAxisNames[6] = {"Fx", "Fy", "Fz", "Mx", "My", "Mz"};
        const char* kAxisUnits[6] = {"N", "N", "N", "Nm", "Nm", "Nm"};
    }  // namespace

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
            setpoint_spin_[i]->setRange(i < 3 ? -200.0 : -50.0, i < 3 ? 200.0 : 50.0);
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
        btn_row->addWidget(apply_btn_);
        btn_row->addWidget(sync_btn_);
        main->addLayout(btn_row);

        auto* hint = new QLabel(
            "仅在 COMPLIANCE 模式下有数据。F_meas 已含反馈符号约定。", this);
        hint->setWordWrap(true);
        hint->setStyleSheet("QLabel { color: #666; font-size: 11px; }");
        main->addWidget(hint);
        main->addStretch();

        connect(apply_btn_, &QPushButton::clicked, this, &ComplianceForcePanel::onApplyClicked);
        connect(sync_btn_, &QPushButton::clicked, this, &ComplianceForcePanel::onSyncClicked);
    }

    ComplianceForcePanel::~ComplianceForcePanel() = default;

    void ComplianceForcePanel::onInitialize()
    {
        node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
            node_, "/ocs2_arm_controller");

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

    void ComplianceForcePanel::onStatus(const StatusMsg::SharedPtr msg)
    {
        if (!msg)
        {
            return;
        }
        last_status_ = *msg;
        have_status_ = true;

        const QString cal = msg->zero_cal_done ? "zero_cal=OK" : "zero_cal=…";
        const QString ft = QString("FT L=%1 R=%2")
                               .arg(msg->left_ft_active ? "on" : "off")
                               .arg(msg->right_ft_active ? "on" : "off");
        QString text = QString("COMPLIANCE  |  %1  |  %2").arg(cal, ft);
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
        (void)param_client_->set_parameters(params);
        ui_dirty_ = false;

        RCLCPP_INFO(
            node_->get_logger(),
            "ComplianceForcePanel applied: S=[%.0f %.0f %.0f %.0f %.0f %.0f] "
            "F_des=[%.2f %.2f %.2f %.2f %.2f %.2f]",
            selection[0], selection[1], selection[2],
            selection[3], selection[4], selection[5],
            setpoint[0], setpoint[1], setpoint[2],
            setpoint[3], setpoint[4], setpoint[5]);

        status_label_->setText(
            status_label_->text().contains("COMPLIANCE")
                ? QString("COMPLIANCE  |  已应用设定")
                : QString("已应用设定"));
    }

}  // namespace arms_rviz_control_plugin

PLUGINLIB_EXPORT_CLASS(arms_rviz_control_plugin::ComplianceForcePanel, rviz_common::Panel)
