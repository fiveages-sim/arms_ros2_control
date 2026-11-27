#include "arms_rviz_control_plugin/compliance_path_widget.hpp"
// 必须在上面之后：makePathWidget() 是 ComplianceForcePanel 的成员。
#include "arms_rviz_control_plugin/compliance_force_panel.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <QGridLayout>
#include <QApplication>
#include <QFontDatabase>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QKeyEvent>
#include <QMessageBox>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QSignalBlocker>
#include <QTableWidgetItem>
#include <QTextBlock>
#include <QTextCursor>
#include <QVBoxLayout>

namespace arms_rviz_control_plugin
{
namespace
{
constexpr int kColIndex = 0;
constexpr int kColX = 1;
constexpr int kNumCols = 8;   // #, x, y, z, qx, qy, qz, qw —— 就是输入里那 7 个数

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

QString fmt(double v)
{
    // 表格只显示 4 位小数（列宽够窄才不横向滚动），完整 6 位看行提示。
    // 先把显示上会变成 -0.0000 的值归零，免得看起来像负号写错了。
    const double shown = std::abs(v) < 5e-5 ? 0.0 : v;
    return QString::number(shown, 'f', 4);
}

/**
 * @brief 把结果表上的 Delete / Backspace 转发出去。
 *
 * 结果表是只读的（NoEditTriggers），Qt 根本不处理这两个键 —— 用户"选中一行
 * 按 Backspace"就毫无反应。这里用事件过滤器而不是 QShortcut：RViz 主窗口本身
 * 有不少快捷键，QShortcut 容易被抢或跟别的上下文冲突；过滤器只在本控件有焦点
 * 时生效，行为确定。
 */
class DeleteKeyForwarder : public QObject
{
public:
    DeleteKeyForwarder(QObject* parent, std::function<void()> on_delete)
        : QObject(parent), on_delete_(std::move(on_delete))
    {
    }

protected:
    bool eventFilter(QObject* watched, QEvent* event) override
    {
        if (event->type() == QEvent::KeyPress)
        {
            const int key = static_cast<QKeyEvent*>(event)->key();
            if (key == Qt::Key_Delete || key == Qt::Key_Backspace)
            {
                on_delete_();
                return true;
            }
        }
        return QObject::eventFilter(watched, event);
    }

private:
    std::function<void()> on_delete_;
};
}  // namespace

CompliancePathWidget::CompliancePathWidget(QWidget* parent)
    : QWidget(parent)
{
    auto* main = new QVBoxLayout(this);
    main->setContentsMargins(4, 4, 4, 4);
    main->setSpacing(4);

    // ── 顶部：手臂 / 坐标系 / 总时长 / 圈数 ──
    auto* top = new QHBoxLayout();
    arm_combo_ = new QComboBox(this);
    arm_combo_->addItem("左臂", "left");
    arm_combo_->addItem("右臂", "right");
    frame_edit_ = new QLineEdit("base_link", this);
    frame_edit_->setMinimumWidth(110);
    frame_edit_->setToolTip(
        "角点坐标系。默认跟随 RViz 的 Fixed Frame（分体/全身 / 各机型自动不同）。\n"
        "不用 TF 的情形：这里填的框架与 compliance_teleop_base_frame 相同，控制器直接短路。");
    duration_spin_ = new QDoubleSpinBox(this);
    duration_spin_->setDecimals(2);
    duration_spin_->setRange(0.5, 600.0);
    duration_spin_->setSingleStep(1.0);
    duration_spin_->setValue(10.0);
    duration_spin_->setSuffix(" s");
    duration_spin_->setToolTip(
        "整条折线的总运行时间（含全部圈数，不含从当前位姿到起点的入段）。\n"
        "控制器会据此等比反解速度/加速度：时长拉长 → 加速度按平方下降 → 角点误差更小。");
    loops_spin_ = new QSpinBox(this);
    loops_spin_->setRange(1, 99);
    loops_spin_->setValue(1);
    loops_spin_->setToolTip("往返圈数；duration 是全部圈数的总时间");

    top->addWidget(new QLabel("手臂", this));
    top->addWidget(arm_combo_);
    top->addWidget(new QLabel("坐标系", this));
    top->addWidget(frame_edit_);
    top->addSpacing(8);
    top->addWidget(new QLabel("总时长", this));
    top->addWidget(duration_spin_);
    top->addWidget(new QLabel("圈数", this));
    top->addWidget(loops_spin_);
    top->addStretch();
    main->addLayout(top);

    // ── 脚点输入：每行一个点，X Y Z R P Y（位置 / 度）或 X Y Z Qx Qy Qz Qw ──
    auto* hint_row = new QHBoxLayout();
    auto* input_hint = new QLabel(
        "一个<b>矩阵</b> = 一条路径（<b>[[…], […]]</b>），可并列写多个矩阵；"
        "每行 7 个数 <b>X Y Z Qx Qy Qz Qw</b>（位置 m + 单位四元数）；"
        "<b>#</b> 注释、逗号/分号/空格都算分隔。",
        this);
    input_hint->setWordWrap(true);
    input_hint->setTextFormat(Qt::RichText);
    setHintRole(input_hint, "hint");
    hint_row->addWidget(input_hint, 1);

    auto* unit_tag = new QLabel("位置单位：<b>m</b>", this);
    unit_tag->setTextFormat(Qt::RichText);
    setHintRole(unit_tag, "hint");
    unit_tag->setToolTip(
        "X/Y/Z 一律按米解析（机器人工作空间 < 2 m）。\n"
        "若数值看着像毫米（某分量绝对值 > 20），整段输入会被判为错误、拒绝下发，\n"
        "不会自动乘 0.001 —— 单位只认一种，避免猜错量级把手臂送出去。");
    hint_row->addWidget(unit_tag);
    main->addLayout(hint_row);

    input_ = new QPlainTextEdit(this);
    input_->setPlaceholderText(
        QString::fromUtf8("# 接近\n"
                          "[\n"
                          "  [0.350000, 0.000000, 0.250000, -1.000000, 0, 0, 0],\n"
                          "  [0.350000, 0.150000, 0.250000, -1.000000, 0, 0, 0]\n"
                          "]\n"
                          "# 另一条路径（并列的第二个矩阵）\n"
                          "[[0.40, 0.00, 0.30, -1, 0, 0, 0], [0.40, 0.15, 0.30, -1, 0, 0, 0]]"));
    input_->setMinimumHeight(96);
    // 上限：约 6 行就够编辑了，否则会把下面的结果表挤没
    input_->setMaximumHeight(172);
    input_->setTabChangesFocus(true);
    input_->setFont(monoUiFont());
    input_->setProperty("mono", true);
    input_->setToolTip(
        "每行一个脚点（7 个数：x y z qx qy qz qw，位置单位 m），相邻两点走直线；\n"
        "每个中间点都会减速到 0 再转向。四元数会被归一化，长度为 0 的点被剔除。\n"
        "分组：一个矩阵 [[…], […]] = 一条独立路径；并列写多个矩阵就是多条路径，\n"
        "用下面的「组」下拉切换查看/下发。紧邻矩阵上方的注释会当作该组名字。\n"
        "只适合少量真实脚点 —— 不要把一段直线/圆弧离散成很多点发过来。");
    main->addWidget(input_, 1);

    // ── 解析结果（只读）：坐标 + 四元数 ──
    table_ = new QTableWidget(0, kNumCols, this);
    table_->setHorizontalHeaderLabels(
        {"#", "X [m]", "Y [m]", "Z [m]", "Qx", "Qy", "Qz", "Qw"});
    // 按内容定宽：挤压时会变成 "0.…"，读数就没意义了
    table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    table_->horizontalHeader()->setStretchLastSection(false);
    table_->verticalHeader()->setVisible(false);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    // 支持多选：选中几行（Shift/Ctrl）后点「删除选中」一次删掉
    table_->setSelectionMode(QAbstractItemView::ExtendedSelection);
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);   // 只读，改文本区
    table_->setMinimumHeight(150);
    table_->setFont(monoUiFont());
    table_->setProperty("mono", true);   // 面板缩放时跟着重刷
    table_->horizontalHeader()->setFont(monoUiFont());
    table_->horizontalHeader()->setProperty("mono", true);
    table_->setToolTip(
        "上面输入框的解析结果（只读）：就是格式化后的 7 个数，与下发内容一一对应。\n"
        "姿态在 RViz 里看（每个点上会画出工具坐标系三轴）；悬停某行可看该点的 RPY。\n"
        "要改数值请改输入框。");
    main->addWidget(table_, 3);

    // ── 行操作（一律作用在输入框的行上）──
    auto* row_btns = new QHBoxLayout();
    group_combo_ = new QComboBox(this);
    group_combo_->setSizeAdjustPolicy(QComboBox::AdjustToContents);
    group_combo_->setToolTip(
        "输入里解析出的路径组：一个矩阵 [[…], […]] 就是一组，并列的多个矩阵就是多组。\n"
        "切换只影响结果表 / Marker / 下发用的是哪一组，不改文本。");
    add_btn_ = new QPushButton("添加点", this);
    add_btn_->setToolTip("在末尾追加一行（复制最后一行，便于连续编辑）");
    remove_btn_ = new QPushButton("删除选中", this);
    remove_btn_->setToolTip(
        "删除选中的位姿：\n"
        "• 在下方结果表里选中若干行 → 精确删掉这几点对应的 7 个数（不动矩阵排版，也不影响同一行里的别的点）\n"
        "• 在输入框里选中几行文字 → 删掉这些整行（含注释、括号）\n"
        "• 什么都没选 → 删光标所在的那一行");
    up_btn_ = new QPushButton("上移", this);
    down_btn_ = new QPushButton("下移", this);
    capture_btn_ = new QPushButton("取当前位姿", this);
    capture_btn_->setToolTip(
        "把控制器上报的当前 TCP 位姿写成 X Y Z Qx Qy Qz Qw，替掉光标行（空文本时追加）。\n"
        "仅在 COMPLIANCE 状态下能拿到 /left_current_pose。");
    clear_btn_ = new QPushButton("清空", this);
    row_btns->addWidget(new QLabel("组", this));
    row_btns->addWidget(group_combo_);
    row_btns->addWidget(add_btn_);
    row_btns->addWidget(remove_btn_);
    row_btns->addWidget(up_btn_);
    row_btns->addWidget(down_btn_);
    row_btns->addWidget(capture_btn_);
    row_btns->addWidget(clear_btn_);
    row_btns->addStretch();
    main->addLayout(row_btns);

    // ── 摘要 + 下发 ──
    summary_label_ = new QLabel("—", this);
    summary_label_->setWordWrap(true);
    // 数值部分要能看清，只有末尾的告警用红色 —— 整个标签变红会把数字也染红
    summary_label_->setTextFormat(Qt::RichText);
    setHintRole(summary_label_, "hint");
    main->addWidget(summary_label_);

    auto* send_row = new QHBoxLayout();
    send_btn_ = new QPushButton("下发轨迹", this);
    send_btn_->setToolTip(
        "把当前脚点 + 总时长作为一条 compliance_path 下发。\n"
        "控制器会先从当前 TCP 走直线入段到第一个脚点，再按速度曲线走完全程。");
    stop_btn_ = new QPushButton("停止（停在原地）", this);
    stop_btn_->setToolTip(
        "把当前实测位姿作为单点目标下发 → 控制器立刻取消正在回放的路径并保持该位姿。");
    applyButtonVariant(send_btn_, "primary");
    applyButtonVariant(stop_btn_, "critical");
    send_row->addWidget(send_btn_);
    send_row->addWidget(stop_btn_);
    send_row->addStretch();
    main->addLayout(send_row);

    status_label_ = new QLabel("未初始化", this);
    status_label_->setWordWrap(true);
    setHintRole(status_label_, "hint");
    main->addWidget(status_label_);

    connect(add_btn_, &QPushButton::clicked, this, &CompliancePathWidget::onAddLine);
    connect(remove_btn_, &QPushButton::clicked, this, &CompliancePathWidget::onRemoveLine);
    connect(up_btn_, &QPushButton::clicked, this, &CompliancePathWidget::onMoveUp);
    connect(down_btn_, &QPushButton::clicked, this, &CompliancePathWidget::onMoveDown);
    connect(capture_btn_, &QPushButton::clicked, this, &CompliancePathWidget::onCapturePose);
    connect(clear_btn_, &QPushButton::clicked, this, &CompliancePathWidget::onClearLines);
    // 结果表只读，Qt 不处理 Delete/Backspace；用户直觉是"选中 → 按键删除"，
    // 所以把这两个键转发到和「删除选中」同一条路径上。
    {
        auto* forwarder = new DeleteKeyForwarder(this, [this]() { onRemoveLine(); });
        table_->installEventFilter(forwarder);
        table_->viewport()->installEventFilter(forwarder);
    }
    connect(send_btn_, &QPushButton::clicked, this, &CompliancePathWidget::onSend);
    connect(stop_btn_, &QPushButton::clicked, this, &CompliancePathWidget::onStop);
    // 切换路径组：只换结果表 / Marker / 下发用的那一份，不动文本
    connect(group_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            [this](int index)
            {
                const int group = group_combo_->itemData(index).toInt();
                if (group < 0 || group == active_group_)
                {
                    return;
                }
                active_group_ = group;
                loadActiveGroup();
                const auto& g = groups_[static_cast<size_t>(active_group_)];
                setStatus(QString("已切到「%1」：%2 个脚点。").arg(g.label).arg(g.poses.size()),
                          "#2196F3");
            });
    // 手臂 / 坐标系变了，Marker 的配色命名与 frame_id 都要跟着变
    connect(arm_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            [this](int) { publishMarkers(); });
    connect(frame_edit_, &QLineEdit::textChanged, this,
            [this](const QString& text)
            {
                // 内容与上次自动填的值不一致 → 用户自己改的，之后不再被 Fixed Frame 覆盖
                if (!applying_auto_frame_ && text.trimmed() != auto_frame_)
                {
                    frame_user_locked_ = true;
                }
                publishMarkers();
            });

    // 输入文本变化 → 150 ms 防抖后再解析，避免每个按键都重建表格
    parse_timer_ = new QTimer(this);
    parse_timer_->setSingleShot(true);
    parse_timer_->setInterval(150);
    connect(parse_timer_, &QTimer::timeout, this, &CompliancePathWidget::parseInput);
    connect(input_, &QPlainTextEdit::textChanged, this,
            [this]() { parse_timer_->start(); });

    capture_btn_->setEnabled(false);
    stop_btn_->setEnabled(false);
    parseInput();
}

void CompliancePathWidget::initialize(const rclcpp::Node::SharedPtr& node,
                                      const QString& default_frame)
{
    node_ = node;
    if (!node_) return;

    // 先定坐标系：后面 publishMarkers() 直接用它作 frame_id
    setDefaultFrame(default_frame);

    using PathMsg = arms_ros2_control_msgs::msg::CompliancePath;
    path_pub_left_ = node_->create_publisher<PathMsg>("/left_compliance_path", 10);
    path_pub_right_ = node_->create_publisher<PathMsg>("/right_compliance_path", 10);
    // 单点目标用 `/left_target`（geometry_msgs/Pose）抢占路径 —— 与
    // StateCompliance 的 updateTargetFromPose + cancelPath("单点目标抢占") 对应。
    single_pub_left_ = node_->create_publisher<geometry_msgs::msg::Pose>("/left_target", 10);
    single_pub_right_ = node_->create_publisher<geometry_msgs::msg::Pose>("/right_target", 10);
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/compliance_path_markers", 10);

    pose_sub_left_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/left_current_pose", rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            // 只在 ROS 线程存值：这条话题是控制周期级频率（~500 Hz），
            // 每帧 invokeMethod 到 Qt 线程会把事件队列打满。
            std::lock_guard<std::mutex> lock(pose_mutex_);
            pose_left_ = *msg;
            have_pose_left_ = true;
        });
    pose_sub_right_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/right_current_pose", rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            pose_right_ = *msg;
            have_pose_right_ = true;
        });

    // Qt 线程侧 5 Hz 轮询
    pose_timer_ = new QTimer(this);
    pose_timer_->setInterval(200);
    connect(pose_timer_, &QTimer::timeout, this, &CompliancePathWidget::refreshPoseState);
    pose_timer_->start();
    refreshPoseState();

    // 心跳：只在编辑时发的话，后加的 Marker 显示会一直是空的；2 s 重发一次很便宜。
    auto* marker_timer = new QTimer(this);
    marker_timer->setInterval(2000);
    connect(marker_timer, &QTimer::timeout, this, &CompliancePathWidget::publishMarkers);
    marker_timer->start();
    publishMarkers();

    setStatus(
        "就绪。下发前请确认：① 控制器在 COMPLIANCE；② compliance_task_selection 全为 0。",
        "#43A047");
}

void CompliancePathWidget::refreshPoseState()
{
    bool have = false;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        have = sideName() == "left" ? have_pose_left_ : have_pose_right_;
    }

    capture_btn_->setEnabled(have);
    stop_btn_->setEnabled(have);

    // 第一次拿到当前位姿时自动填第一行，省得手抄六个数
    if (have) seedFromCurrentPose();
}

geometry_msgs::msg::Pose CompliancePathWidget::currentPoseOrIdentity()
{
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    std::lock_guard<std::mutex> lock(pose_mutex_);
    const bool is_left = sideName() == "left";
    const bool have = is_left ? have_pose_left_ : have_pose_right_;
    if (have) pose = is_left ? pose_left_.pose : pose_right_.pose;
    return pose;
}

void CompliancePathWidget::seedFromCurrentPose()
{
    if (seeded_pose_row_) return;
    if (!input_->toPlainText().trimmed().isEmpty()) return;
    seeded_pose_row_ = true;
    setInputLines({poseToLine(currentPoseOrIdentity())});
}

QString CompliancePathWidget::poseToLine(const geometry_msgs::msg::Pose& pose) const
{
    // 直接写四元数，不经过 RPY：欧拉角往返会引入转换误差，且存在万向锁多解。
    // 6 位小数足够（位置 µm 级、四元数 1e-6 级），抄出去再粘回来不丢精度。
    return QString("%1  %2  %3   %4  %5  %6  %7")
        .arg(pose.position.x, 0, 'f', 6)
        .arg(pose.position.y, 0, 'f', 6)
        .arg(pose.position.z, 0, 'f', 6)
        .arg(pose.orientation.x, 0, 'f', 6)
        .arg(pose.orientation.y, 0, 'f', 6)
        .arg(pose.orientation.z, 0, 'f', 6)
        .arg(pose.orientation.w, 0, 'f', 6);
}

void CompliancePathWidget::onControllerPathState(const QString& arm, uint32_t sequence,
                                               const QString& state,
                                               const QString& message)
{
    if (sequence == controller_sequence_) return;   // 最便宜的检查放最前
    if (arm != QString::fromStdString(sideName())) return;
    controller_sequence_ = sequence;

    // 控制器侧的权威结果：颜色跟着状态走
    const char* color = "#2196F3";   // queued
    QString prefix = QString::fromUtf8("控制器：");
    if (state == "playing")
    {
        color = "#FB8C00";
    }
    else if (state == "done")
    {
        color = "#43A047";
    }
    else if (state == "cancelled" || state == "rejected")
    {
        color = "#E53935";
        prefix = state == "rejected" ? QString::fromUtf8("控制器拒绝：")
                                     : QString::fromUtf8("控制器取消：");
    }
    setStatus(prefix + message, color);
}

void CompliancePathWidget::setDefaultFrame(const QString& frame)
{
    if (frame.isEmpty() || frame_user_locked_) return;
    const QString trimmed = frame.trimmed();
    if (frame_edit_->text().trimmed() == trimmed) return;
    applying_auto_frame_ = true;
    frame_edit_->setText(trimmed);
    applying_auto_frame_ = false;
    auto_frame_ = trimmed;
}

std::string CompliancePathWidget::sideName() const
{
    return arm_combo_->currentData().toString().toStdString();
}

void CompliancePathWidget::setStatus(const QString& text, const char* color)
{
    status_label_->setText(text);
    // 字号由 hint 角色提供（面板样式表的 @SMALL@），这里只改颜色
    status_label_->setStyleSheet(QString("QLabel { color: %1; }").arg(color));
}

void CompliancePathWidget::publishMarkers()
{
    if (!marker_pub_ || !node_) return;

    const QString frame_text = frame_edit_->text().trimmed();
    const std::string frame = frame_text.isEmpty() ? "base_link" : frame_text.toStdString();
    const auto stamp = node_->now();

    visualization_msgs::msg::MarkerArray array;

    // 先清掉上一版：删行之后旧球不会自己消失
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = frame;
    clear.header.stamp = stamp;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    const std::string ns = "compliance_path_" + sideName();
    const auto make_marker = [&](int id, int type)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame;
        marker.header.stamp = stamp;
        marker.ns = ns;
        marker.id = id;
        marker.type = static_cast<int32_t>(type);
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        return marker;
    };

    // ── 其它组：灰色细线 + 组名，方便对照，但别抢当前组的视线 ──
    for (int g = 0; g < static_cast<int>(groups_.size()); ++g)
    {
        if (g == active_group_) continue;
        const auto& other = groups_[static_cast<size_t>(g)];
        if (other.poses.size() < 2) continue;

        auto line = make_marker(900 + g, visualization_msgs::msg::Marker::LINE_STRIP);
        line.scale.x = 0.002;
        line.color.r = 0.55f;
        line.color.g = 0.55f;
        line.color.b = 0.55f;
        line.color.a = 0.45f;
        for (const auto& pose : other.poses) line.points.push_back(pose.position);
        array.markers.push_back(std::move(line));

        auto label = make_marker(950 + g, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
        label.pose.position = other.poses.front().position;
        label.pose.position.z += 0.075;
        label.scale.z = 0.025;
        label.color.r = 0.55f;
        label.color.g = 0.55f;
        label.color.b = 0.55f;
        label.color.a = 0.85f;
        label.text = other.label.toStdString();
        array.markers.push_back(std::move(label));
    }

    // 与下发用的是同一套解析结果，写错的行不画
    const std::vector<geometry_msgs::msg::Pose>& poses = parsed_;
    if (poses.empty())
    {
        marker_pub_->publish(array);
        return;
    }

    // 折线：单点时画不出线，补一个重复点
    {
        auto marker = make_marker(1, visualization_msgs::msg::Marker::LINE_STRIP);
        marker.scale.x = 0.004;
        marker.color.r = 0.13f;
        marker.color.g = 0.59f;
        marker.color.b = 0.95f;
        marker.color.a = 0.75f;
        for (const auto& pose : poses) marker.points.push_back(pose.position);
        if (poses.size() == 1) marker.points.push_back(poses.front().position);
        array.markers.push_back(std::move(marker));
    }

    // 脚点：起点绿、终点红、中间蓝 —— 一眼看出走向
    {
        auto marker = make_marker(2, visualization_msgs::msg::Marker::SPHERE_LIST);
        marker.scale.x = marker.scale.y = marker.scale.z = 0.028;
        for (size_t i = 0; i < poses.size(); ++i)
        {
            marker.points.push_back(poses[i].position);
            std_msgs::msg::ColorRGBA color;
            if (i == 0)
            {
                color.r = 0.26f; color.g = 0.63f; color.b = 0.28f;
            }
            else if (i + 1 == poses.size())
            {
                color.r = 0.90f; color.g = 0.23f; color.b = 0.21f;
            }
            else
            {
                color.r = 0.13f; color.g = 0.59f; color.b = 0.95f;
            }
            color.a = 1.0f;
            marker.colors.push_back(color);
        }
        array.markers.push_back(std::move(marker));
    }

    // 每个点的工具坐标系三轴（X 红 / Y 绿 / Z 蓝，各 7 cm）：
    // 球和折线只能看出"走到哪"，姿态对不对得看这三个轴。
    {
        auto marker = make_marker(3, visualization_msgs::msg::Marker::LINE_LIST);
        marker.scale.x = 0.005;
        constexpr double kAxisLength = 0.08;
        // X 红 / Y 绿 / Z 蓝
        const std::array<std::array<float, 3>, 3> axis_color{{
            {0.90f, 0.20f, 0.20f},
            {0.20f, 0.80f, 0.25f},
            {0.20f, 0.45f, 1.00f},
        }};
        for (const auto& pose : poses)
        {
            const double x = pose.orientation.x;
            const double y = pose.orientation.y;
            const double z = pose.orientation.z;
            const double w = pose.orientation.w;
            // 四元数的旋转矩阵 R 的三列 = 工具系三个轴在 base 系中的方向。
            // 只为可视化，不引入 tf2 依赖。
            const std::array<std::array<double, 3>, 3> axes{{
                {1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + w * z), 2.0 * (x * z - w * y)},
                {2.0 * (x * y - w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z + w * x)},
                {2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y)},
            }};
            for (size_t k = 0; k < 3; ++k)
            {
                geometry_msgs::msg::Point tip = pose.position;
                tip.x += kAxisLength * axes[k][0];
                tip.y += kAxisLength * axes[k][1];
                tip.z += kAxisLength * axes[k][2];
                std_msgs::msg::ColorRGBA color;
                color.r = axis_color[k][0];
                color.g = axis_color[k][1];
                color.b = axis_color[k][2];
                color.a = 0.95f;
                marker.points.push_back(pose.position);
                marker.colors.push_back(color);
                marker.points.push_back(tip);
                marker.colors.push_back(color);
            }
        }
        array.markers.push_back(std::move(marker));
    }

    // 序号：橙色在白底/暗底都看得清
    for (size_t i = 0; i < poses.size(); ++i)
    {
        auto marker = make_marker(
            100 + static_cast<int>(i), visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
        marker.pose.position = poses[i].position;
        marker.pose.position.z += 0.045;
        marker.scale.z = 0.035;
        marker.color.r = 0.95f;
        marker.color.g = 0.55f;
        marker.color.b = 0.00f;
        marker.color.a = 1.0f;
        marker.text = std::to_string(i + 1);
        array.markers.push_back(std::move(marker));
    }

    marker_pub_->publish(array);
}

void CompliancePathWidget::parseInput()
{
    bad_lines_ = 0;
    first_error_.clear();
    groups_.clear();

    // ── 词法：把整段文本切成带行号的数，同时跟踪括号深度 ──
    // 分组规则（关键）：**一个矩阵 = 一条路径**。
    //   顶层括号里还套了一层括号（[[…], […]]）→ 一个矩阵组；
    //   裸行 / 单层 [7 个数] 行 → 全部并入同一个「松散组」。
    // 这样「每行一个点」的老粘贴方式仍然只解析出 1 组，行为完全不变。
    struct Token
    {
        QString text;
        int line{0};
        int col{0};   ///< 该行内的起始字符下标（删点时要精确挖掉）
    };
    struct Block
    {
        std::vector<Token> tokens;
        int max_depth{1};
        int start_line{0};
        QString label;
    };

    std::vector<Block> blocks;    // 矩阵组（括号套了两层）
    Block loose;                  // 松散组（裸行 / 单层方括号行）
    QString loose_label;

    QString pending_comment;      // 最近一条纯注释行 → 下一个组的名字
    int pending_comment_line = -1;

    int depth = 0;
    bool in_block = false;
    Block block;
    QString token;
    int token_line = 1;
    int token_col = 0;

    const auto flush_token = [&]()
    {
        if (token.isEmpty()) return;
        if (in_block)
        {
            block.tokens.push_back(Token{token, token_line, token_col});
        }
        else
        {
            if (loose.tokens.empty() && !pending_comment.isEmpty())
            {
                loose_label = pending_comment;
                pending_comment.clear();
                pending_comment_line = -1;
            }
            loose.tokens.push_back(Token{token, token_line, token_col});
        }
        token.clear();
    };
    const auto close_block = [&]()
    {
        if (!in_block) return;
        if (block.max_depth >= 2)
        {
            // 紧邻（≤2 行）的注释当组名，用完就清掉，免得粘到下一组
            if (!pending_comment.isEmpty() && block.start_line - pending_comment_line <= 2)
            {
                block.label = pending_comment;
                pending_comment.clear();
                pending_comment_line = -1;
            }
            blocks.push_back(std::move(block));
        }
        else
        {
            // 单层方括号行：只是"点"的包装，不是分组
            if (loose.tokens.empty() && !pending_comment.isEmpty() &&
                block.start_line - pending_comment_line <= 2)
            {
                loose_label = pending_comment;
                pending_comment.clear();
                pending_comment_line = -1;
            }
            loose.tokens.insert(loose.tokens.end(), block.tokens.begin(), block.tokens.end());
        }
        block = Block{};
        in_block = false;
    };

    const QStringList lines = inputLines();
    for (int i = 0; i < lines.size(); ++i)
    {
        const QString& raw = lines.at(i);
        bool line_has_code = false;
        for (int c = 0; c < raw.size(); ++c)
        {
            const QChar ch = raw.at(c);
            if (ch == '#') break;
            if (ch == '/' && c + 1 < raw.size() && raw.at(c + 1) == '/') break;

            if (ch == '[' || ch == '(' || ch == '{')
            {
                flush_token();
                ++depth;
                if (depth == 1 && !in_block)
                {
                    in_block = true;
                    block.max_depth = 1;
                    block.start_line = i + 1;
                }
                else if (in_block)
                {
                    block.max_depth = std::max(block.max_depth, depth);
                }
                line_has_code = true;
            }
            else if (ch == ']' || ch == ')' || ch == '}')
            {
                flush_token();
                if (depth > 0) --depth;
                if (in_block && depth == 0) close_block();
                line_has_code = true;
            }
            else if (ch == ',' || ch == ';' || ch.isSpace())
            {
                flush_token();
            }
            else
            {
                if (token.isEmpty())
                {
                    token_line = i + 1;
                    token_col = c;
                }
                token.append(ch);
                line_has_code = true;
            }
        }
        flush_token();

        // 整行只有注释 → 记下来当下一组的名字
        if (!line_has_code)
        {
            int cut = raw.indexOf('#');
            int cut_len = 1;
            const int slash = raw.indexOf("//");
            if (slash >= 0 && (cut < 0 || slash < cut))
            {
                cut = slash;
                cut_len = 2;
            }
            const QString comment = cut >= 0 ? raw.mid(cut + cut_len).trimmed() : QString();
            if (!comment.isEmpty())
            {
                pending_comment = comment;
                pending_comment_line = i + 1;
            }
        }
    }
    // 括号没闭合（少写一个 ]）也要收尾，否则整段都解析不出来
    close_block();

    // ── 每组按 7 个数切点 ──
    // 切点按**行**进行：矩阵可以写成"一行一个点"（缩进排版），也可以整条压在
    // 一行里。逐行切的好处是某一行写错了只剔除那行，同一组其它点照常可用。
    const auto fill_group = [&](const std::vector<Token>& tokens, PathGroup& group)
    {
        constexpr int kStride = 7;
        if (tokens.empty()) return;

        auto fail = [&](const QString& message)
        {
            ++bad_lines_;
            group.errors << message;
            if (first_error_.isEmpty()) first_error_ = message;
        };

        size_t begin = 0;
        while (begin < tokens.size())
        {
            size_t end = begin;
            while (end < tokens.size() && tokens[end].line == tokens[begin].line) ++end;

            bool numbers_ok = true;
            for (size_t t = begin; t < end; ++t)
            {
                bool ok = false;
                tokens[t].text.toDouble(&ok);
                if (!ok)
                {
                    numbers_ok = false;
                    fail(QString("第 %1 行含非数字「%2」").arg(tokens[t].line).arg(tokens[t].text));
                    break;
                }
            }

            const int count = static_cast<int>(end - begin);
            if (!numbers_ok)
            {
                begin = end;
                continue;
            }
            if (count % kStride != 0)
            {
                fail(QString("第 %1 行有 %2 个数（应为 7 的整数倍：x y z qx qy qz qw）")
                         .arg(tokens[begin].line)
                         .arg(count));
                begin = end;
                continue;
            }

            for (int offset = 0; offset + kStride <= count; offset += kStride)
            {
                std::array<double, 7> v{};
                for (int k = 0; k < kStride; ++k)
                {
                    v[static_cast<size_t>(k)] =
                        tokens[begin + static_cast<size_t>(offset + k)].text.toDouble();
                }
                const int line = tokens[begin].line;

                // 位置单位固定为米：只做量级检查，不做任何换算。
                // 工作空间 < 2 m，任何位置分量 > 20 一定是毫米（或别的量级）。
                bool magnitude_ok = true;
                for (int k = 0; k < 3; ++k)
                {
                    if (std::abs(v[static_cast<size_t>(k)]) > 20.0)
                    {
                        magnitude_ok = false;
                        fail(QString("第 %1 行 %2 = %3 超出 20 m，位置必须是米"
                                     "（若为毫米请先除以 1000）")
                                 .arg(line)
                                 .arg(k == 0 ? "X" : (k == 1 ? "Y" : "Z"))
                                 .arg(v[static_cast<size_t>(k)], 0, 'f', 3));
                        break;
                    }
                }
                if (!magnitude_ok) continue;

                // 只做归一化（控制器侧也会做一次）：不改变姿态，只为抵掉手抄时的
                // 有限位舍入。四元数长度为 0 说明数据坏了，整点剔除。
                const double norm =
                    std::sqrt(v[3] * v[3] + v[4] * v[4] + v[5] * v[5] + v[6] * v[6]);
                if (norm < 1e-6)
                {
                    fail(QString("第 %1 行四元数长度为 0").arg(line));
                    continue;
                }
                geometry_msgs::msg::Pose pose;
                pose.position.x = v[0];
                pose.position.y = v[1];
                pose.position.z = v[2];
                pose.orientation.x = v[3] / norm;
                pose.orientation.y = v[4] / norm;
                pose.orientation.z = v[5] / norm;
                pose.orientation.w = v[6] / norm;
                group.poses.push_back(pose);

                // 记下这 7 个数在输入框里的字符区间：「删除选中」时按它精确挖掉
                const Token& head = tokens[begin + static_cast<size_t>(offset)];
                const Token& tail = tokens[begin + static_cast<size_t>(offset + kStride - 1)];
                group.spans.push_back(PathGroup::PointSpan{
                    head.line, head.col, tail.col + static_cast<int>(tail.text.size())});
            }
            begin = end;
        }
    };

    // 松散组排在第 1 位（只有存在裸行/单层方括号行时才建）
    if (!loose.tokens.empty())
    {
        PathGroup group;
        group.label = loose_label;
        fill_group(loose.tokens, group);
        groups_.push_back(std::move(group));
    }
    for (const Block& b : blocks)
    {
        if (b.tokens.empty()) continue;   // 空矩阵 [] 不算一组
        PathGroup group;
        group.label = b.label;
        fill_group(b.tokens, group);
        groups_.push_back(std::move(group));
    }
    for (size_t i = 0; i < groups_.size(); ++i)
    {
        if (groups_[i].label.isEmpty()) groups_[i].label = QString("路径 %1").arg(i + 1);
    }

    // 尽量保持当前选中的组：删/加组后越界就夹到合法范围
    if (groups_.empty())
    {
        active_group_ = 0;
    }
    else
    {
        active_group_ = std::clamp(active_group_, 0, static_cast<int>(groups_.size()) - 1);
    }
    rebuildGroupCombo();
    loadActiveGroup();
}

void CompliancePathWidget::rebuildGroupCombo()
{
    if (!group_combo_) return;
    const QSignalBlocker blocker(group_combo_);
    group_combo_->clear();

    // 下拉里只放组名：加点数会把这一行撑宽 60px，而点数在摘要行里已经有了。
    // 组名来自用户注释，可能很长 —— 按字数截断，否则一个长注释就能把整行按钮
    // 撑出可视区。完整名字与点数都列在 tooltip 里。
    constexpr int kMaxLabelChars = 10;
    QStringList tooltip_lines;
    for (int i = 0; i < static_cast<int>(groups_.size()); ++i)
    {
        const auto& group = groups_[static_cast<size_t>(i)];
        const bool broken = !group.errors.isEmpty() || group.poses.empty();
        QString shown = group.label;
        if (shown.size() > kMaxLabelChars)
        {
            shown = shown.left(kMaxLabelChars - 1) + QString::fromUtf8("…");
        }
        group_combo_->addItem(shown + (broken ? QString::fromUtf8(" ⚠") : QString()), i);
        tooltip_lines << QString("%1. %2 — %3 个点%4")
                             .arg(i + 1)
                             .arg(group.label)
                             .arg(group.poses.size())
                             .arg(broken ? QString::fromUtf8("（解析失败）") : QString());
    }
    if (groups_.empty())
    {
        group_combo_->addItem(QString::fromUtf8("（无路径）"), -1);
        group_combo_->setCurrentIndex(0);
        group_combo_->setToolTip(
            "输入里还没解析出路径。写一个矩阵 [[…], […]] 就是一条路径；并列写多个就是多条。");
    }
    else
    {
        group_combo_->setCurrentIndex(active_group_);
        group_combo_->setToolTip(
            QString::fromUtf8("切换要查看/下发的路径（只换面板看到的东西，不改文本）：\n") +
            tooltip_lines.join('\n'));
    }
    group_combo_->setEnabled(groups_.size() > 1);
}

void CompliancePathWidget::loadActiveGroup()
{
    if (groups_.empty() || active_group_ < 0 ||
        active_group_ >= static_cast<int>(groups_.size()))
    {
        parsed_.clear();
    }
    else
    {
        parsed_ = groups_[static_cast<size_t>(active_group_)].poses;
    }
    refreshResultTable();
    refreshSummary();
    publishMarkers();
}

void CompliancePathWidget::refreshResultTable()
{
    const QSignalBlocker blocker(table_);
    table_->setRowCount(static_cast<int>(parsed_.size()));
    for (int r = 0; r < static_cast<int>(parsed_.size()); ++r)
    {
        const auto& pose = parsed_[r];
        auto put = [&](int column, const QString& text, Qt::Alignment alignment)
        {
            auto* item = new QTableWidgetItem(text);
            item->setTextAlignment(alignment);
            // 可选（为了「删除选中」），但不可编辑
            item->setFlags(item->flags() & ~Qt::ItemIsEditable);
            table_->setItem(r, column, item);
        };
        put(kColIndex, QString::number(r + 1), Qt::AlignCenter);
        // 列序与 kColX 之后的 7 个值一一对应：# X Y Z Qx Qy Qz Qw
        const std::array<double, 7> values{pose.position.x, pose.position.y, pose.position.z,
                                           pose.orientation.x, pose.orientation.y,
                                           pose.orientation.z, pose.orientation.w};
        for (int k = 0; k < 7; ++k)
        {
            put(kColX + k, fmt(values[static_cast<size_t>(k)]),
                Qt::AlignRight | Qt::AlignVCenter);
        }

        // 行提示里补一行 RPY（度），方便核对姿态；表格只放输入数据，不放派生量
        const tf2::Quaternion quaternion(pose.orientation.x, pose.orientation.y,
                                         pose.orientation.z, pose.orientation.w);
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
        const QString tip =
            poseToLine(pose) + QString("\nRPY = %1°  %2°  %3°（ZYX，仅供参考）")
                                   .arg(roll * kRadToDeg, 0, 'f', 2)
                                   .arg(pitch * kRadToDeg, 0, 'f', 2)
                                   .arg(yaw * kRadToDeg, 0, 'f', 2);
        for (int column = 0; column < kNumCols; ++column)
        {
            if (auto* item = table_->item(r, column)) item->setToolTip(tip);
        }
    }
}

QStringList CompliancePathWidget::inputLines() const
{
    return input_->toPlainText().split('\n');
}

void CompliancePathWidget::setInputLines(const QStringList& lines)
{
    input_->setPlainText(lines.join('\n'));
    // setPlainText 会触发 textChanged 启动防抖定时器；这里直接解析，操作要立刻可见
    parse_timer_->stop();
    parseInput();
}

int CompliancePathWidget::currentLine() const
{
    return input_->textCursor().blockNumber();
}

void CompliancePathWidget::setCurrentLine(int line)
{
    const QTextBlock block = input_->document()->findBlockByNumber(line);
    if (!block.isValid()) return;
    QTextCursor cursor(block);
    input_->setTextCursor(cursor);
}

void CompliancePathWidget::moveCurrentLine(int delta)
{
    QStringList lines = inputLines();
    const int index = currentLine();
    const int target = index + delta;
    if (index < 0 || target < 0 || index >= lines.size() || target >= lines.size()) return;
    // 收尾括号行不能挪：一挪括号就配错，整组都解析不出来
    const auto is_closer = [&](int i)
    {
        const QString t = lines.at(i).trimmed();
        return t == "]" || t == "]," || t == ")" || t == ")," ||
               t == "}" || t == "}," || t == "];" || t == ");";
    };
    if (is_closer(index) || is_closer(target))
    {
        setStatus("矩阵的收尾括号行不能移动；请移动具体的点所在行。", "#c62828");
        return;
    }
    lines.swapItemsAt(index, target);
    setInputLines(lines);
    setCurrentLine(target);
}

void CompliancePathWidget::onAddLine()
{
    QStringList lines = inputLines();
    // 末尾若是空行就不再追加空行
    while (!lines.isEmpty() && lines.last().trimmed().isEmpty()) lines.removeLast();

    int last = -1;
    for (int i = lines.size() - 1; i >= 0; --i)
    {
        if (!lines.at(i).trimmed().isEmpty())
        {
            last = i;
            break;
        }
    }
    const auto is_closer = [](const QString& t)
    {
        return t == "]" || t == "]," || t == ")" || t == ")," ||
               t == "}" || t == "}," || t == "];" || t == ");";
    };

    // ── 情形 1：矩阵是"缩进 + 单独一行收尾括号"的写法 ──
    //   [
    //     [a b c …],
    //     [d e f …]
    //   ]        ← 光标前的最后一行
    // 复制上一行的点，插到收尾括号之前。
    if (last >= 0 && is_closer(lines.at(last).trimmed()))
    {
        for (int i = last - 1; i >= 0; --i)
        {
            const QString t = lines.at(i).trimmed();
            if (t.isEmpty() || t.startsWith('#') || t.startsWith("//")) continue;
            if (t == "[" || t == "(" || t == "{") break;
            // 只复制"一行就是一个点"的行；一行多个点的情况没法安全复制
            int brackets = 0;
            for (const QChar ch : t)
            {
                if (ch == '[' || ch == '(' || ch == '{') ++brackets;
            }
            if (brackets > 1) break;

            QString origin = lines.at(i);
            if (!origin.trimmed().endsWith(",")) origin += ",";
            lines[i] = origin;
            lines.insert(i + 1, origin);
            setInputLines(lines);
            setCurrentLine(i + 1);
            return;
        }
    }

    // ── 情形 2：整个矩阵压在一行（…[[a…], [b…]]）──
    // 从收尾括号往前取"最后一个已闭合的行"：先找最后一个 ']'，再向前找与它配对的 '['。
    if (last >= 0)
    {
        const QString tail = lines.at(last).trimmed();
        if (tail.endsWith("]") || tail.endsWith(")"))
        {
            QString line = lines.at(last);
            const int close = std::max(line.lastIndexOf(']'), line.lastIndexOf(')'));
            const QString head = line.left(close);
            const int row_end = head.lastIndexOf(']');
            int row_start = -1;
            if (row_end > 0)
            {
                int level = 0;
                for (int i = row_end; i >= 0; --i)
                {
                    if (head.at(i) == ']')
                    {
                        ++level;
                    }
                    else if (head.at(i) == '[')
                    {
                        if (--level == 0)
                        {
                            row_start = i;
                            break;
                        }
                    }
                }
            }
            if (row_start >= 0)
            {
                const QString row = head.mid(row_start, row_end - row_start + 1);
                lines[last] = head + ", " + row + line.mid(close);
                setInputLines(lines);
                setCurrentLine(last);
                return;
            }
        }
    }

    // ── 情形 3：普通逐行写法，照旧复制最后一行 ──
    QString seed;
    for (int i = lines.size() - 1; i >= 0 && seed.isEmpty(); --i)
    {
        const QString candidate = lines.at(i).trimmed();
        if (!candidate.isEmpty() && !candidate.startsWith('#') &&
            !candidate.startsWith("//"))
        {
            seed = candidate;
        }
    }
    if (seed.isEmpty()) seed = poseToLine(currentPoseOrIdentity());

    lines.append(seed);
    setInputLines(lines);
    setCurrentLine(lines.size() - 1);
}

void CompliancePathWidget::onRemoveLine()
{
    // ① 结果表里选中了行 → 精确挖掉这些位姿的 7 个数。
    //    只删字符区间，方括号/缩进/注释、以及同一行里的别的点都不受影响。
    const QList<QTableWidgetSelectionRange> ranges = table_->selectedRanges();
    if (!ranges.isEmpty() && active_group_ >= 0 &&
        active_group_ < static_cast<int>(groups_.size()))
    {
        const auto& group = groups_[static_cast<size_t>(active_group_)];
        std::vector<int> rows;
        for (const auto& range : ranges)
        {
            for (int row = range.topRow(); row <= range.bottomRow(); ++row) rows.push_back(row);
        }
        std::sort(rows.begin(), rows.end());
        rows.erase(std::unique(rows.begin(), rows.end()), rows.end());

        QStringList lines = inputLines();
        int removed = 0;
        // 从后往前删：前面的字符下标才不会因为删掉后面的而不准
        for (auto it = rows.rbegin(); it != rows.rend(); ++it)
        {
            if (*it < 0 || *it >= static_cast<int>(group.spans.size())) continue;
            const auto& span = group.spans[static_cast<size_t>(*it)];
            if (span.line <= 0 || span.line > lines.size()) continue;
            QString& line = lines[span.line - 1];
            if (span.col_begin < 0 || span.col_end > line.size() ||
                span.col_end <= span.col_begin)
            {
                continue;
            }
            line.remove(span.col_begin, span.col_end - span.col_begin);
            ++removed;
        }
        if (removed > 0)
        {
            // 删空了只剩 [] 的行一起去掉，文本里不留空壳
            for (int i = lines.size() - 1; i >= 0; --i)
            {
                const QString t = lines.at(i).simplified().remove(' ');
                if (t.isEmpty() || t == "[]" || t == "()," || t == "[]" + QString(",") ||
                    t == "[];")
                {
                    lines.removeAt(i);
                }
            }
            table_->clearSelection();
            setInputLines(lines);
            setStatus(QString("已删除 %1 个位姿（第 %2 组「%3」）。")
                          .arg(removed)
                          .arg(active_group_ + 1)
                          .arg(group.label),
                      "#2196F3");
            return;
        }
    }

    // ② 输入框里选中了文字 → 删掉被选中的整行（可能多行）
    const QTextCursor cursor = input_->textCursor();
    if (cursor.hasSelection())
    {
        QStringList lines = inputLines();
        QTextCursor probe = cursor;
        probe.setPosition(cursor.selectionStart());
        int first = probe.blockNumber();
        probe.setPosition(cursor.selectionEnd());
        int last = probe.blockNumber();
        // 选区终点正好停在行首时不要多删下一行
        if (last > first && probe.positionInBlock() == 0) --last;
        first = std::max(0, first);
        last = std::min(last, static_cast<int>(lines.size()) - 1);
        if (last >= first)
        {
            for (int i = last; i >= first; --i) lines.removeAt(i);
            setInputLines(lines);
            setCurrentLine(std::min(first, std::max(0, static_cast<int>(lines.size()) - 1)));
            setStatus(QString("已删除第 %1~%2 行。").arg(first + 1).arg(last + 1), "#2196F3");
            return;
        }
    }

    // ③ 什么都没选 → 删光标所在行
    QStringList lines = inputLines();
    const int index = currentLine();
    if (index < 0 || index >= lines.size())
    {
        setStatus("先把光标放到要删除的那一行，或在结果表里选中要删的位姿。", "#c62828");
        return;
    }
    lines.removeAt(index);
    setInputLines(lines);
    setCurrentLine(std::min(index, std::max(0, static_cast<int>(lines.size()) - 1)));
}

void CompliancePathWidget::onMoveUp()
{
    moveCurrentLine(-1);
}

void CompliancePathWidget::onMoveDown()
{
    moveCurrentLine(1);
}

void CompliancePathWidget::onCapturePose()
{
    geometry_msgs::msg::Pose pose;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        const bool is_left = sideName() == "left";
        const bool have = is_left ? have_pose_left_ : have_pose_right_;
        if (!have)
        {
            setStatus(
                "还拿不到当前位姿：控制器不在 COMPLIANCE，或该侧 current_pose 未发布。",
                "#E53935");
            return;
        }
        pose = is_left ? pose_left_.pose : pose_right_.pose;
    }

    const QString line = poseToLine(pose);
    QStringList lines = inputLines();
    const int index = currentLine();
    // 光标在矩阵行上时保留原来的方括号/缩进/逗号写法，只换里面的 7 个数
    auto format_like = [&](const QString& original)
    {
        const QString trimmed = original.trimmed();
        if (!trimmed.startsWith('[') && !trimmed.startsWith('(')) return line;
        const QChar opener = trimmed.at(0);
        const QChar closer = opener == '[' ? QChar(']') : QChar(')');
        const QString inner = line.simplified();
        const bool trailing_comma = trimmed.endsWith(QString(closer) + ",");
        return QString("  %1%2%3%4")
            .arg(opener)
            .arg(inner)
            .arg(closer)
            .arg(trailing_comma ? "," : "");
    };

    if (lines.size() == 1 && lines.first().trimmed().isEmpty())
    {
        lines = QStringList{line};
    }
    else if (index < 0 || index >= lines.size())
    {
        lines.append(line);
    }
    else
    {
        lines[index] = format_like(lines.at(index));
    }
    setInputLines(lines);
    setCurrentLine(std::min(index, static_cast<int>(lines.size()) - 1));
}

void CompliancePathWidget::onClearLines()
{
    // 清空是明确的用户意图：别再被"首次拿到位姿就自动填一行"补回来
    seeded_pose_row_ = true;
    input_->clear();
    parse_timer_->stop();
    parseInput();
}

void CompliancePathWidget::refreshSummary()
{
    const int rows = static_cast<int>(parsed_.size());
    double length = 0.0;
    for (size_t i = 1; i < parsed_.size(); ++i)
    {
        const auto& a = parsed_[i - 1].position;
        const auto& b = parsed_[i].position;
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double dz = b.z - a.z;
        length += std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    const int loops = loops_spin_->value();
    const double duration = duration_spin_->value();
    const double total_len = length * static_cast<double>(loops);
    const double v_avg = duration > 1e-9 ? total_len / duration : 0.0;

    QString text;
    // 多组时先说清"现在看的是哪一组"，否则表格换内容会让人以为点丢了
    if (groups_.size() > 1 && active_group_ >= 0 &&
        active_group_ < static_cast<int>(groups_.size()))
    {
        text = QString("组 %1/%2（%3）　|　")
                   .arg(active_group_ + 1)
                   .arg(groups_.size())
                   .arg(groups_[static_cast<size_t>(active_group_)].label);
    }
    text += QString("%1 个脚点 → %2 段  |  单趟 %3 mm  |  %4 圈 = %5 mm  |  "
                    "总时长 %6 s  |  平均速度 %7 mm/s  |  位置单位 %8")
                .arg(rows)
                .arg(std::max(0, rows - 1))
                .arg(length * 1000.0, 0, 'f', 1)
                .arg(loops)
                .arg(total_len * 1000.0, 0, 'f', 1)
                .arg(duration, 0, 'f', 2)
                .arg(v_avg * 1000.0, 0, 'f', 1)
                .arg(QString::fromUtf8("m"));
    QString suffix;
    if (rows < 2) suffix += QString::fromUtf8("　　← 至少需要 2 个脚点");
    if (bad_lines_ > 0)
    {
        suffix += QString::fromUtf8("　　← %1 处有问题").arg(bad_lines_);
        if (!first_error_.isEmpty()) suffix += QString("（%1）").arg(first_error_);
    }
    if (!suffix.isEmpty())
    {
        suffix = QString("<span style=\"color:#E53935;font-weight:600;\">%1</span>")
                     .arg(suffix);
    }
    summary_label_->setText(text + suffix);
    setHintRole(summary_label_, "hint");
}

void CompliancePathWidget::onSend()
{
    if (!node_)
    {
        setStatus("面板尚未初始化（ROS 节点为空）。", "#c62828");
        return;
    }
    // 文本可能刚改过还没刷新，下发前再解析一次保证一致
    parse_timer_->stop();
    parseInput();

    const bool have_group = !groups_.empty() && active_group_ >= 0 &&
                            active_group_ < static_cast<int>(groups_.size());
    const QString group_label = have_group
                                    ? groups_[static_cast<size_t>(active_group_)].label
                                    : QString();

    // 只校验要下发的那一组：别的组写坏了不该拦住这一组
    if (have_group)
    {
        const auto& group = groups_[static_cast<size_t>(active_group_)];
        if (!group.errors.isEmpty())
        {
            setStatus(QString("下发取消：「%1」有 %2 处解析错误（%3）。")
                          .arg(group_label)
                          .arg(group.errors.size())
                          .arg(group.errors.first()),
                      "#c62828");
            return;
        }
    }
    if (parsed_.size() < 2)
    {
        setStatus(have_group ? QString("「%1」至少需要 2 个脚点（现在 %2 个）。")
                                   .arg(group_label)
                                   .arg(parsed_.size())
                             : QString("至少需要 2 个脚点。"),
                  "#c62828");
        return;
    }

    arms_ros2_control_msgs::msg::CompliancePath msg;
    msg.header.frame_id = frame_edit_->text().trimmed().toStdString();
    if (msg.header.frame_id.empty()) msg.header.frame_id = "base_link";

    for (const auto& pose : parsed_)
    {
        msg.points.push_back(pose.position.x);
        msg.points.push_back(pose.position.y);
        msg.points.push_back(pose.position.z);
        msg.points.push_back(pose.orientation.x);
        msg.points.push_back(pose.orientation.y);
        msg.points.push_back(pose.orientation.z);
        msg.points.push_back(pose.orientation.w);
    }

    msg.blend_radius = 0.0;   // 折线模式：每段独立加减速，角点停到 0
    msg.speed = 0.0;          // 0 = 用控制器名义值，再由 duration 等比缩放
    msg.accel = 0.0;
    msg.lat_accel = 0.0;
    msg.closed = false;
    msg.loops = static_cast<uint32_t>(std::max(1, loops_spin_->value()));
    msg.duration = duration_spin_->value();

    const std::string side = sideName();
    const QString side_cn = side == "left" ? "左臂" : "右臂";

    auto& pub = side == "left" ? path_pub_left_ : path_pub_right_;
    if (!pub)
    {
        setStatus("轨迹发布器还没建好（面板未初始化）。", "#c62828");
        return;
    }
    // 控制器的 path_sub 只在 StateCompliance::enter() 里创建、exit() 里销毁。
    // 不在 COMPLIANCE 时这条话题一个订阅者都没有，消息会被静默丢弃 ——
    // 面板只会一直停在"等待控制器确认…"，看起来像"发了但不动"。
    // 所以下发前必须先看订阅者数量，把话说清楚。
    if (pub->get_subscription_count() == 0)
    {
        setStatus(QString("下发取消：/%1_compliance_path 当前没有订阅者 —— "
                          "控制器不在 COMPLIANCE 状态（HOLD / HOME 下不接收轨迹）。"
                          "先按 FSM 切到 COMPLIANCE 再下发。")
                      .arg(side.c_str()),
                  "#c62828");
        return;
    }

    const auto answer = QMessageBox::question(
        this, "确认下发轨迹",
        QString("%1：%2\n%3 个脚点 / %4 圈 / 总时长 %5 s\n\n"
                "下发后手臂会立刻从当前位姿直线入段到第一个脚点并开始运动。\n"
                "请确认工作空间内无人、无遮挡，且已把 compliance_task_selection "
                "全设为 0。\n\n确定下发？")
            .arg(side_cn)
            .arg(have_group ? QString("路径「%1」").arg(group_label) : QString("折线"))
            .arg(parsed_.size())
            .arg(msg.loops)
            .arg(msg.duration, 0, 'f', 2),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (answer != QMessageBox::Yes) return;

    pub->publish(msg);

    setStatus(
        QString("已发送 %1%2：%3 个脚点 / %4 圈 / 总时长 %5 s，等待控制器确认…")
            .arg(side_cn)
            .arg(have_group ? QString("「%1」").arg(group_label) : QString())
            .arg(parsed_.size())
            .arg(msg.loops)
            .arg(msg.duration, 0, 'f', 2),
        "#FB8C00");
}

void CompliancePathWidget::onStop()
{
    if (!node_) return;
    const bool is_left = sideName() == "left";
    geometry_msgs::msg::Pose pose;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        const bool have = is_left ? have_pose_left_ : have_pose_right_;
        if (!have)
        {
            setStatus("拿不到当前位姿，无法停止；请改用 FSM 切回 HOLD(2)。", "#E53935");
            return;
        }
        pose = is_left ? pose_left_.pose : pose_right_.pose;
    }
    // 单点目标会抢占路径（StateCompliance: updateTargetFromPose + cancelPath），
    // 目标 = 当前实测位姿 → 就地保持。
    auto& pub = is_left ? single_pub_left_ : single_pub_right_;
    for (int i = 0; i < 3; ++i) pub->publish(pose);   // 订阅端是 SensorDataQoS，多播几次
    setStatus("已取消路径，目标保持在当前位姿。", "#FB8C00");
}

// ─────────────────────────────────────────────────────────────────────────
// 宿主面板的标签页工厂（与 makePayloadWidget 同一套约定：定义在独立 .cpp）
// ─────────────────────────────────────────────────────────────────────────
QWidget* ComplianceForcePanel::makePathWidget()
{
    auto* page = new QWidget(this);
    auto* layout = new QVBoxLayout(page);
    layout->setContentsMargins(2, 2, 2, 2);
    layout->setSpacing(4);

    auto* hint = new QLabel(
        "折线轨迹：每行一个脚点，走直线；每个中间点减速到 0 再转向。"
        "总时长由控制器反解速度/加速度（时长越长 → 加速度按平方下降 → 角点偏差越小）。",
        page);
    hint->setWordWrap(true);
    setHintRole(hint, "hint");
    layout->addWidget(hint);

    path_widget_ = new CompliancePathWidget(page);
    layout->addWidget(path_widget_, 1);
    return page;
}

}  // namespace arms_rviz_control_plugin
