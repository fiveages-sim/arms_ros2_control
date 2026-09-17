#include "arms_rviz_control_plugin/compliance_force_panel.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <algorithm>
#include <cmath>

namespace arms_rviz_control_plugin
{
QWidget* ComplianceForcePanel::makePayloadWidget()
{
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);
    auto* hint = new QLabel(
        "辨识力传感器之后的末端负载：质量、重心和原始零偏。\n"
        "COMPLIANCE 中将六轴设为位控（S=0），通过目标位姿调整姿态，静止且无接触后采样。\n"
        "至少 6 个姿态，绕两个不同轴倾斜。采样器不发送运动指令。", page);
    hint->setWordWrap(true);
    layout->addWidget(hint);
    auto* form = new QFormLayout();
    payload_arm_combo_ = new QComboBox(page);
    payload_arm_combo_->addItem("左臂", "left");
    payload_arm_combo_->addItem("右臂", "right");
    payload_wrench_topic_ = new QLineEdit("/left_ft_broadcaster/wrench", page);
    payload_gravity_frame_ = new QLineEdit("world", page);
    payload_joint_topic_ = new QLineEdit("/joint_states", page);
    payload_pose_count_ = new QSpinBox(page);
    payload_pose_count_->setRange(6, 100);
    payload_pose_count_->setValue(9);
    form->addRow("手臂", payload_arm_combo_);
    form->addRow("原始 FT 话题", payload_wrench_topic_);
    form->addRow("重力参考系（-Z）", payload_gravity_frame_);
    form->addRow("关节反馈话题", payload_joint_topic_);
    form->addRow("姿态数", payload_pose_count_);
    layout->addLayout(form);
    connect(payload_arm_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int) {
                payload_wrench_topic_->setText(
                    "/" + payload_arm_combo_->currentData().toString() + "_ft_broadcaster/wrench");
            });

    auto* buttons = new QHBoxLayout();
    payload_start_btn_ = new QPushButton("开始负载辨识…", page);
    payload_sample_btn_ = new QPushButton("采集当前姿态", page);
    payload_stop_btn_ = new QPushButton("停止采样", page);
    payload_sample_btn_->setEnabled(false);
    payload_stop_btn_->setEnabled(false);
    buttons->addWidget(payload_start_btn_);
    buttons->addWidget(payload_sample_btn_);
    buttons->addWidget(payload_stop_btn_);
    layout->addLayout(buttons);
    payload_log_ = new QPlainTextEdit(page);
    payload_log_->setReadOnly(true);
    payload_log_->setMaximumBlockCount(500);
    layout->addWidget(payload_log_);

    payload_process_ = new QProcess(this);
    connect(payload_start_btn_, &QPushButton::clicked,
            this, &ComplianceForcePanel::startPayloadIdentification);
    connect(payload_sample_btn_, &QPushButton::clicked, this, [this]() {
        payload_sample_btn_->setEnabled(false);
        payload_process_->write("sample\n");
    });
    connect(payload_stop_btn_, &QPushButton::clicked,
            this, &ComplianceForcePanel::stopPayloadIdentification);
    connect(payload_process_, &QProcess::readyReadStandardOutput,
            this, &ComplianceForcePanel::readPayloadOutput);
    connect(payload_process_, &QProcess::readyReadStandardError, this, [this]() {
        payload_log_->appendPlainText(QString::fromUtf8(payload_process_->readAllStandardError()));
    });
    connect(payload_process_, &QProcess::errorOccurred, this, [this](QProcess::ProcessError) {
        payload_log_->appendPlainText(payload_process_->errorString());
        if (payload_process_->state() == QProcess::NotRunning) setPayloadRunning(false);
    });
    connect(payload_process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [this](int code, QProcess::ExitStatus status) {
                readPayloadOutput();
                setPayloadRunning(false);
                payload_log_->appendPlainText(code == 0 && status == QProcess::NormalExit
                    ? "辨识完成：结果和原始样本已保存。"
                    : "采样停止或辨识未通过；已完成姿态的 CSV 保留，请查看上方原因。");
            });
    return page;
}

void ComplianceForcePanel::setPayloadRunning(bool running)
{
    payload_start_btn_->setEnabled(!running);
    payload_stop_btn_->setEnabled(running);
    payload_sample_btn_->setEnabled(false);
    for (QWidget* widget : std::initializer_list<QWidget*>{payload_arm_combo_,
                           payload_wrench_topic_, payload_gravity_frame_, payload_joint_topic_,
                           payload_pose_count_})
        widget->setEnabled(!running);
    position_identification_btn_->setEnabled(!running);
    all_joint_identification_btn_->setEnabled(!running);
    contact_identification_btn_->setEnabled(!running);
}

void ComplianceForcePanel::startPayloadIdentification()
{
    if (payload_process_->state() != QProcess::NotRunning) return;
    if (!have_status_)
    {
        payload_log_->appendPlainText(
            "未收到有效的 COMPLIANCE 状态，或状态已超时。请进入 COMPLIANCE；若已经进入，"
            "请检查状态话题通信，并确认控制器和 RViz 均已使用更新后的消息包重启。");
        return;
    }
    if (last_status_.identification_active)
    {
        payload_log_->appendPlainText("控制器报告其他辨识正在运行，请先停止该辨识再开始负载采样。");
        return;
    }
    if (std::any_of(last_status_.task_selection.begin(), last_status_.task_selection.end(),
                    [](double v) { return !std::isfinite(v) || v != 0.; }))
    {
        QStringList selection;
        for (double value : last_status_.task_selection)
            selection.append(QString::number(value));
        payload_log_->appendPlainText(
            "控制器当前 S=[" + selection.join(", ") + "]，负载采样要求全部为 0。"
            "请取消上方六个「力控」勾选并点击「应用设定」，等待状态回读后重试。"
            "S 顺序为 Fx/Fy/Fz/Mx/My/Mz，是末端六维控制轴，与机械臂关节数无关。");
        return;
    }
    const auto& arm_joints = payload_arm_combo_->currentData().toString() == "right"
        ? last_status_.right_joint_names : last_status_.left_joint_names;
    if (arm_joints.empty())
    {
        payload_log_->appendPlainText("控制器未提供所选手臂的关节列表，请确认该臂已配置并重启更新后的控制器。");
        return;
    }
    QStringList joints;
    for (const auto& name : arm_joints) joints.append(QString::fromStdString(name));
    if (payload_wrench_topic_->text().trimmed().isEmpty() ||
        payload_gravity_frame_->text().trimmed().isEmpty() || payload_joint_topic_->text().trimmed().isEmpty())
    {
        payload_log_->appendPlainText("请填写原始 FT 话题、关节反馈话题和重力参考系。");
        return;
    }
    QString script;
    try
    {
        script = QString::fromStdString(ament_index_cpp::get_package_prefix("ocs2_arm_controller"))
            + "/lib/ocs2_arm_controller/identify_payload.py";
    }
    catch (const std::exception& e)
    {
        payload_log_->appendPlainText(QString("未找到负载辨识工具，请构建并加载 ocs2_arm_controller：%1").arg(e.what()));
        return;
    }
    const auto parent = QFileDialog::getExistingDirectory(this, "选择辨识数据保存目录", QDir::homePath());
    if (parent.isEmpty()) return;
    const auto output = QDir(parent).filePath("payload_" + payload_arm_combo_->currentData().toString()
        + "_" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz"));
    QStringList args{"-u", script, "collect", "--panel", "--joints"};
    args.append(joints);
    args << "--wrench-topic" << payload_wrench_topic_->text().trimmed()
         << "--gravity-frame" << payload_gravity_frame_->text().trimmed()
         << "--joint-state-topic" << payload_joint_topic_->text().trimmed()
         << "--poses" << QString::number(payload_pose_count_->value())
         << "--output-dir" << output;
    bool sim_time = false;
    if (node_) node_->get_parameter_or("use_sim_time", sim_time, false);
    if (sim_time) args << "--use-sim-time";
    payload_stdout_.clear();
    payload_log_->clear();
    payload_log_->appendPlainText("自动读取所选臂关节：" + joints.join(", "));
    payload_log_->appendPlainText("保存目录：" + output + "\n等待静止关节、原始 FT 和对应时间的 TF…");
    setPayloadRunning(true);
    payload_process_->start("python3", args);
}

void ComplianceForcePanel::stopPayloadIdentification()
{
    if (payload_process_->state() == QProcess::NotRunning) return;
    payload_sample_btn_->setEnabled(false);
    payload_process_->terminate();  // Python handles SIGTERM and flushes completed samples.
}

void ComplianceForcePanel::readPayloadOutput()
{
    payload_stdout_ += payload_process_->readAllStandardOutput();
    int newline;
    while ((newline = payload_stdout_.indexOf('\n')) >= 0)
    {
        const auto line = payload_stdout_.left(newline);
        payload_stdout_.remove(0, newline + 1);
        const auto object = QJsonDocument::fromJson(line).object();
        const auto event = object.value("event").toString();
        if (event == "ready")
        {
            payload_sample_btn_->setEnabled(true);
            payload_log_->appendPlainText(QString("姿态 %1/%2：调整目标位姿，静止且无接触后点击采集。")
                .arg(object.value("pose").toInt()).arg(object.value("total").toInt()));
        }
        else if (event == "sampling")
            payload_log_->appendPlainText("正在等待静止并采样…");
        else if (event == "sampled")
            payload_log_->appendPlainText(QString("姿态 %1 已保存，累计 %2 个原始样本。")
                .arg(object.value("pose").toInt()).arg(object.value("samples").toInt()));
        else if (event == "result")
        {
            const auto com = object.value("center_of_mass_mm").toArray();
            payload_log_->appendPlainText(QString("质量 %1 kg；传感器坐标系重心 [%2, %3, %4] mm。\n"
                                                   "结果未自动写入控制器；详细零偏和残差见 result.yaml。")
                .arg(object.value("mass_kg").toDouble(), 0, 'g', 6)
                .arg(com.at(0).toDouble(), 0, 'g', 6).arg(com.at(1).toDouble(), 0, 'g', 6)
                .arg(com.at(2).toDouble(), 0, 'g', 6));
        }
        else if (!line.isEmpty())
            payload_log_->appendPlainText(QString::fromUtf8(line));
    }
}
}  // namespace arms_rviz_control_plugin
