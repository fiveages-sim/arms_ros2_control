#include "arms_rviz_control_plugin/compliance_force_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QGroupBox>
#include <QDockWidget>
#include <QMainWindow>
#include <QApplication>
#include <QFontDatabase>
#include <QFontMetrics>
#include <QScrollArea>
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QSaveFile>
#include <QSignalBlocker>
#include <QPainter>
#include <QPainterPath>
#include <QPaintEvent>
#include <QTabWidget>

#include <algorithm>
#include <cmath>

namespace arms_rviz_control_plugin
{
    namespace
    {
        const char* kAxisNames[6] = {"Fx", "Fy", "Fz", "Mx", "My", "Mz"};
        const char* kAxisUnits[6] = {"N", "N", "N", "Nm", "Nm", "Nm"};

        // 曲面贴合外环：UI 用角度 (deg)，参数与遥测用弧度 (rad)。
        constexpr double kRadToDeg = 57.29577951308232;
        constexpr double kDegToRad = 1.0 / kRadToDeg;

        // 面板统一外观。边框一律用半透明灰（rgba(128,128,128,a)），
        // 亮色/暗色主题下都成立；只有强调按钮才用实心底色。
        // 原则：结构必须看得见（分组框、输入框、按钮、表格都有 1px 边），
        // 但不加渐变/阴影/粗色块 —— 简约靠"细线 + 留白"，不靠"去掉线"。
        const char* kPanelStyle = R"(
QGroupBox {
    border: 1px solid rgba(128,128,128,0.85);
    border-radius: 7px;
    margin-top: 0px;
    padding: 26px 10px 10px 10px;
}
QGroupBox::title {
    subcontrol-origin: padding;
    subcontrol-position: top left;
    left: 10px;
    top: 5px;
    padding: 0px;
    color: rgba(128,128,128,1);
    font-weight: 600;
}
QDoubleSpinBox, QSpinBox, QLineEdit, QComboBox {
    padding: 5px 7px;
    border: 1px solid rgba(128,128,128,0.80);
    border-radius: 5px;
    min-height: 20px;
}
QDoubleSpinBox:hover, QSpinBox:hover, QLineEdit:hover, QComboBox:hover {
    border: 1px solid rgba(128,128,128,1.0);
}
/* 设了 border 后 spin box 的增减箭头不再由 style 画，默认是一块空白 —— 必须显式画三角 */
QDoubleSpinBox::up-button, QSpinBox::up-button {
    subcontrol-origin: border;
    subcontrol-position: top right;
    width: 20px;
    border-left: 1px solid rgba(128,128,128,0.50);
    border-top-right-radius: 4px;
    background-color: rgba(128,128,128,0.12);
}
QDoubleSpinBox::down-button, QSpinBox::down-button {
    subcontrol-origin: border;
    subcontrol-position: bottom right;
    width: 20px;
    border-left: 1px solid rgba(128,128,128,0.50);
    border-bottom-right-radius: 4px;
    background-color: rgba(128,128,128,0.12);
}
QDoubleSpinBox::up-button:hover, QSpinBox::up-button:hover,
QDoubleSpinBox::down-button:hover, QSpinBox::down-button:hover {
    background-color: rgba(128,128,128,0.28);
}
QDoubleSpinBox::up-arrow, QSpinBox::up-arrow {
    width: 0px;
    height: 0px;
    border-left: 5px solid transparent;
    border-right: 5px solid transparent;
    border-bottom: 6px solid rgba(110,110,110,1.0);
}
QDoubleSpinBox::down-arrow, QSpinBox::down-arrow {
    width: 0px;
    height: 0px;
    border-left: 5px solid transparent;
    border-right: 5px solid transparent;
    border-top: 6px solid rgba(110,110,110,1.0);
}
QDoubleSpinBox::up-arrow:disabled, QSpinBox::up-arrow:disabled,
QDoubleSpinBox::down-arrow:disabled, QSpinBox::down-arrow:disabled {
    border-bottom-color: rgba(128,128,128,0.30);
    border-top-color: rgba(128,128,128,0.30);
}
QDoubleSpinBox:focus, QSpinBox:focus, QLineEdit:focus, QComboBox:focus {
    border: 1px solid #2196F3;
}
QComboBox::drop-down {
    border: none;
    width: 18px;
}
QPlainTextEdit, QTextEdit {
    border: 1px solid rgba(128,128,128,0.80);
    border-radius: 6px;
}
QPushButton {
    padding: 6px 14px;
    border: 1px solid rgba(128,128,128,0.70);
    border-radius: 5px;
    background-color: rgba(128,128,128,0.09);
    min-height: 21px;
}
QPushButton:hover {
    border-color: rgba(128,128,128,1.0);
    background-color: rgba(128,128,128,0.18);
}
QPushButton:pressed { background-color: rgba(128,128,128,0.28); }
QPushButton:disabled {
    color: rgba(128,128,128,0.65);
    background-color: transparent;
    border-color: rgba(128,128,128,0.22);
}
QPushButton[variant="primary"] {
    background-color: #2196F3;
    border-color: #1976D2;
    color: #FFFFFF;
    font-weight: 600;
}
QPushButton[variant="primary"]:hover { background-color: #1E88E5; }
QPushButton[variant="primary"]:pressed { background-color: #1976D2; }
QPushButton[variant="primary"]:disabled {
    background-color: rgba(128,128,128,0.14);
    color: rgba(128,128,128,0.75);
}
QPushButton[variant="danger"] {
    background-color: transparent;
    border-color: rgba(229,57,53,0.55);
    color: #E53935;
    font-weight: 600;
}
QPushButton[variant="danger"]:hover {
    border-color: #E53935;
    background-color: rgba(229,57,53,0.14);
}
QPushButton[variant="danger"]:pressed { background-color: rgba(229,57,53,0.24); }
QPushButton[variant="danger"]:disabled {
    background-color: transparent;
    border-color: rgba(128,128,128,0.22);
    color: rgba(128,128,128,0.65);
}
QPushButton[variant="critical"] {
    background-color: #E53935;
    border-color: #C62828;
    color: #FFFFFF;
    font-weight: 600;
}
QPushButton[variant="critical"]:hover { background-color: #D32F2F; }
QPushButton[variant="critical"]:pressed { background-color: #C62828; }
QTableWidget {
    border: 1px solid rgba(128,128,128,0.85);
    border-radius: 6px;
    gridline-color: rgba(128,128,128,0.30);
}
QTableWidget::item:selected { background-color: rgba(33,150,243,0.18); }
QHeaderView::section {
    padding: 4px 8px;
    border: none;
    border-right: 1px solid rgba(128,128,128,0.24);
    border-bottom: 1px solid rgba(128,128,128,0.70);
    background-color: rgba(128,128,128,0.10);
    color: rgba(128,128,128,1);
}
QTableCornerButton::section {
    border: none;
    border-bottom: 1px solid rgba(128,128,128,0.70);
    background-color: rgba(128,128,128,0.10);
}
QTabWidget::pane {
    border: none;
    border-top: 1px solid rgba(128,128,128,0.30);
    top: 0px;
}
QScrollArea {
    border: none;
    background: transparent;
}
QScrollArea > QWidget > QWidget { background: transparent; }
QScrollBar:vertical {
    background: transparent;
    width: 9px;
    margin: 0px;
}
QScrollBar::handle:vertical {
    background-color: rgba(128,128,128,0.45);
    border-radius: 4px;
    min-height: 28px;
}
QScrollBar::handle:vertical:hover { background-color: rgba(128,128,128,0.70); }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0px; }
QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical { background: transparent; }
QScrollBar:horizontal {
    background: transparent;
    height: 9px;
    margin: 0px;
}
QScrollBar::handle:horizontal {
    background-color: rgba(128,128,128,0.45);
    border-radius: 4px;
    min-width: 28px;
}
QScrollBar::handle:horizontal:hover { background-color: rgba(128,128,128,0.70); }
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width: 0px; }
QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal { background: transparent; }
QTabBar::tab {
    padding: 7px 15px;
    margin-right: 4px;
    border: none;
    border-bottom: 2px solid transparent;
    background: transparent;
    color: rgba(128,128,128,1);
}
QTabBar::tab:selected {
    color: #2196F3;
    border-bottom: 2px solid #2196F3;
    font-weight: 600;
}
QLabel[role="hint"], QLabel[role="hint-ok"],
QLabel[role="hint-warn"], QLabel[role="hint-pending"] {
    font-size: @SMALL@px;
}
QLabel[role="hint"] { color: rgba(128,128,128,1); }
QLabel[role="hint-ok"] { color: #43A047; }
QLabel[role="hint-warn"] { color: #E53935; font-weight: 600; }
QLabel[role="hint-pending"] { color: #FB8C00; }
)";

        /// 全局界面缩放（1.0 = 基准）。由 ComplianceForcePanel::applyUiScale()
        /// 按面板宽度调整，等宽字体也跟着一起缩，否则数字列会和正文差一号。
        double g_ui_scale = 1.0;

        /// 把 @SMALL@ 换成当前缩放下的次要文字字号后的完整样式表。
        QString panelStyle()
        {
            QString sheet = QString::fromUtf8(kPanelStyle);
            sheet.replace("@SMALL@", QString::number(std::lround(11.0 * g_ui_scale)));
            return sheet;
        }

        /// 设置等宽字体并打标记，供 applyUiScale() 在尺寸变化后重刷。
        void setMonoFont(QWidget* widget)
        {
            if (!widget) return;
            widget->setProperty("mono", true);
            widget->setFont(monoUiFont());
        }

        /// 状态徽标：● 在线 / ○ 无数据。简约风格下不再画胶囊底，只用一个小圆点。
        void applyStateBadge(QLabel* badge, bool active)
        {
            if (!badge) return;
            const int px = static_cast<int>(std::lround(11.0 * g_ui_scale));
            badge->setText(active ? QString::fromUtf8("● COMPLIANCE")
                                  : QString::fromUtf8("○ 无数据"));
            badge->setStyleSheet(
                QString("QLabel { color: %1; font-weight: 600; font-size: %2px;"
                        " background: transparent; padding: 0px; }")
                    .arg(active ? "#43A047" : "rgba(128,128,128,1)")
                    .arg(px));
        }

        /// 操作反馈浮层的配色：按文案猜测严重程度（调用点太多，不改签名）。
        void applyNoticeStyle(QLabel* label, const QString& text)
        {
            if (!label) return;
            const char* color = "#43A047";   // 信息 / 成功
            if (text.contains("失败") || text.contains("不可用") ||
                text.contains("超时") || text.contains("拒绝") ||
                text.contains("取消") || text.contains("错误"))
            {
                color = "#E53935";
            }
            else if (text.contains("中…") || text.contains("正在") ||
                     text.contains("已修改") || text.contains("已发送"))
            {
                color = "#FB8C00";           // 进行中
            }
            label->setStyleSheet(
                QString("QLabel { color: %1; background-color: rgba(128,128,128,0.08);"
                        " border-radius: 5px; padding: 6px 9px; font-size: %2px; }")
                    .arg(color)
                    .arg(std::lround(12.0 * g_ui_scale)));
        }
    }  // namespace

    double uiScale()
    {
        return g_ui_scale;
    }

    QFont monoUiFont()
    {
        // 磅数对齐 UI 基准字体（不是系统等宽字体自己的磅数），否则两种字体差一号
        QFont font = QFontDatabase::systemFont(QFontDatabase::FixedFont);
        const double ui_size = QApplication::font().pointSizeF();
        font.setPointSizeF((ui_size > 0.0 ? ui_size : 9.0) * 0.95 * g_ui_scale);
        return font;
    }

    int minTextWidth(const QFont& font, const QString& sample, int extra)
    {
        // 写死像素在大字号主题下会把 "F_meas L"、"0.01" 这类文字裁掉，
        // 按实际字体量一遍再留余量。
        return QFontMetrics(font).horizontalAdvance(sample) + extra;
    }

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

        // 主题自适应：底色/边框/网格全部走调色板，暗色主题下不会留一块白板
        const QPalette pal = palette();
        const QColor bg = pal.color(QPalette::Base);
        const QColor fg = pal.color(QPalette::WindowText);
        // QPalette::Mid 在浅色主题下几乎是白的，画出来的框看不见；
        // 用前景色压透明度，亮色/暗色两边都有足够对比度。
        QColor border = fg;
        border.setAlpha(130);
        QColor frame = fg;
        frame.setAlpha(110);
        painter.fillRect(rect(), bg);

        const QFontMetrics metrics(font());
        const int title_h = metrics.height() + 9;   // 中文字形下沿比 descent 还低，留够再画
        // 左边距按最长的刻度文字实测，不再写死 46 px —— 字号一变 "180°" 就被裁成 "80°"
        const int left_margin = metrics.horizontalAdvance(QString::fromUtf8("−180°")) + 10;
        const int right_margin = 10;
        const int band_gap = 8;
        const int bottom_reserve = metrics.height() + 4;   // 相位区下方留给频率刻度
        const double plot_w = std::max(40, width() - left_margin - right_margin);
        // 每块 = 自己的标题带 + 绘图框；标题画在左边界右侧、刻度左侧之外，
        // 两者永远不会叠在一起（之前 180° 刻度会压掉标题开头）。
        const double block_h =
            std::max(2.0 * title_h + 80.0,
                     static_cast<double>(height() - bottom_reserve)) / 2.0;
        const double time_title_top = 4.0;
        const QRectF time_rect(left_margin, time_title_top + title_h,
                               plot_w, block_h - title_h - band_gap);
        const double phase_title_top = block_h + band_gap / 2.0;
        const QRectF phase_rect(left_margin, phase_title_top + title_h,
                                plot_w, block_h - title_h - band_gap);

        // 整体描边：白底上也能看出这是一个图区
        painter.setPen(QPen(frame, 1.0));
        painter.drawRoundedRect(QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5), 6, 6);

        painter.setPen(QPen(border, 1.0));
        painter.drawRect(time_rect);
        painter.drawRect(phase_rect);

        // 标题：从绘图区左边界开始，超宽时省略号截断，绝不越过控件边界
        const auto draw_title = [&](const QString& text, double top) {
            painter.setPen(fg);
            painter.drawText(QRectF(left_margin, top, plot_w, title_h),
                             Qt::AlignLeft | Qt::AlignVCenter,
                             metrics.elidedText(text, Qt::ElideRight, plot_w));
        };
        draw_title(mode_ == 2 ? "接触链路时域：实际位移（蓝）/ 力（红），各自归一化"
                              : "关节位置环时域：q_cmd（蓝）/ q_measured（红），各自归一化",
                   time_title_top);
        draw_title("频率响应：相位（绿，-180°..180°）/ 增益（橙，归一化）",
                   phase_title_top);

        // 相位区零线与 180°/-180° 刻度
        painter.setPen(QPen(border, 1.0, Qt::DashLine));
        painter.drawLine(QPointF(phase_rect.left(), phase_rect.center().y()),
                         QPointF(phase_rect.right(), phase_rect.center().y()));
        painter.setPen(fg);
        const auto draw_y_label = [&](const char* text, double y) {
            painter.drawText(QRectF(0, y - metrics.height() / 2.0, left_margin - 4,
                                    metrics.height()),
                             Qt::AlignRight | Qt::AlignVCenter,
                             QString::fromUtf8(text));
        };
        draw_y_label("180°", phase_rect.top() + 2);
        draw_y_label("0°", phase_rect.center().y());
        draw_y_label("-180°", phase_rect.bottom() - 2);

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
        draw_series(input_samples_, QColor(25, 110, 220));
        draw_series(output_samples_, QColor(220, 60, 55));

        if (result_frequency_.empty()) return;

        const auto [fmin_it, fmax_it] = std::minmax_element(
            result_frequency_.begin(), result_frequency_.end());
        const double log_min = std::log(std::max(*fmin_it, 1e-6));
        const double log_max = std::log(std::max(*fmax_it, 1e-6));
        const auto ratio_of = [&](double frequency) {
            return std::abs(log_max - log_min) < 1e-9
                       ? 0.5
                       : (std::log(std::max(frequency, 1e-6)) - log_min) / (log_max - log_min);
        };

        // 频率网格（对数轴）+ 端点标注
        painter.setPen(QPen(border, 1.0, Qt::DotLine));
        for (int i = 0; i <= 4; ++i)
        {
            const double x = phase_rect.left() + plot_w * i / 4.0;
            painter.drawLine(QPointF(x, phase_rect.top()), QPointF(x, phase_rect.bottom()));
        }
        painter.setPen(fg);
        painter.drawText(QRectF(phase_rect.left(), phase_rect.bottom() + 1, 70,
                                metrics.height()),
                         Qt::AlignLeft | Qt::AlignTop,
                         QString("%1 Hz").arg(*fmin_it, 0, 'f', 1));
        painter.drawText(QRectF(phase_rect.right() - 70, phase_rect.bottom() + 1, 70,
                                metrics.height()),
                         Qt::AlignRight | Qt::AlignTop,
                         QString("%1 Hz").arg(*fmax_it, 0, 'f', 1));

        double max_gain = 1e-12;
        for (double gain : result_gain_) max_gain = std::max(max_gain, gain);
        QPainterPath gain_path;
        QPointF previous;
        painter.setPen(QPen(QColor(235, 135, 20), 1.5));
        for (size_t i = 0; i < result_frequency_.size(); ++i)
        {
            const double x = phase_rect.left() + ratio_of(result_frequency_[i]) * plot_w;
            const QPointF point(
                x, phase_rect.center().y() -
                       result_phase_[i] / 360.0 * phase_rect.height());
            if (i > 0) painter.drawLine(previous, point);
            painter.setBrush(QColor(30, 150, 70));
            painter.drawEllipse(point, 3.0, 3.0);
            painter.setBrush(Qt::NoBrush);
            previous = point;
            const QPointF gain_point(
                x, phase_rect.bottom() - 0.9 * phase_rect.height() *
                       result_gain_[i] / max_gain);
            if (i == 0) gain_path.moveTo(gain_point); else gain_path.lineTo(gain_point);
        }
        painter.setPen(QPen(QColor(235, 135, 20), 1.5));
        painter.drawPath(gain_path);
        painter.setPen(fg);
        painter.drawText(
            QRectF(phase_rect.left() + 5, phase_rect.top() + 3,
                   phase_rect.width() - 10, metrics.height()),
            Qt::AlignRight | Qt::AlignTop,
            QString("最新 %1 Hz  gain=%2  phase=%3°")
                .arg(result_frequency_.back(), 0, 'f', 2)
                .arg(result_gain_.back(), 0, 'g', 4)
                .arg(result_phase_.back(), 0, 'f', 1));
    }

    ComplianceForcePanel::ComplianceForcePanel(QWidget* parent)
        : Panel(parent)
    {
        auto* main = new QVBoxLayout(this);
        main->setContentsMargins(12, 10, 12, 10);
        main->setSpacing(11);
        setStyleSheet(panelStyle());

        // ── 状态头：徽标 + 实时遥测 / frame ──
        auto* header = new QWidget(this);
        auto* header_row = new QHBoxLayout(header);
        header_row->setContentsMargins(0, 0, 0, 0);
        header_row->setSpacing(12);

        state_badge_ = new QLabel("无数据", header);
        state_badge_->setAlignment(Qt::AlignCenter);
        state_badge_->setMinimumWidth(96);
        header_row->addWidget(state_badge_, 0, Qt::AlignTop);

        auto* header_col = new QVBoxLayout();
        header_col->setContentsMargins(0, 2, 0, 0);
        header_col->setSpacing(3);
        telemetry_label_ = new QLabel("等待 COMPLIANCE 状态…", header);
        telemetry_label_->setWordWrap(true);
        setMonoFont(telemetry_label_);
        header_col->addWidget(telemetry_label_);
        frame_label_ = new QLabel("frame: —", header);
        setHintRole(frame_label_, "hint");
        header_col->addWidget(frame_label_);
        header_row->addLayout(header_col, 1);
        main->addWidget(header);

        // ── 瞬时提示浮层：所有 status_label_->setText() 都写到这里，
        //    由 notice_watchdog_ 按内容变化打样式、6 s 后自动隐藏。──
        status_label_ = new QLabel(QString(), this);
        status_label_->setWordWrap(true);
        status_label_->setVisible(false);
        main->addWidget(status_label_);

        auto* group = new QGroupBox("力控轴 / 设定力 / 实测力", this);
        auto* grid = new QGridLayout(group);
        grid->setHorizontalSpacing(14);
        grid->setVerticalSpacing(8);
        grid->setContentsMargins(2, 4, 2, 2);

        grid->addWidget(new QLabel("轴", group), 0, 0);
        grid->addWidget(new QLabel("力控", group), 0, 1);
        grid->addWidget(new QLabel("F_des", group), 0, 2);
        // 表头与值列必须同宽，否则右对齐后表头会漂在值的左边
        const int value_col_w = std::max(
            minTextWidth(monoUiFont(), QString::fromUtf8("-0.00"), 14),
            minTextWidth(QApplication::font(), QString::fromUtf8("F_meas L"), 14));
        // 表头跟值同宽同对齐，否则表头会漂在列左侧、和右对齐的数值错位
        const auto add_right_header = [&](const char* text, int column) {
            auto* label = new QLabel(QString::fromUtf8(text), group);
            label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            label->setMinimumWidth(value_col_w);
            grid->addWidget(label, 0, column);
        };
        add_right_header("F_meas L", 3);
        add_right_header("F_meas R", 4);
        add_right_header("err L", 5);
        // 尾部占位列：没有真实控件时 setColumnStretch 对不存在的列是空操作，
        // 多余宽度会被 last column 吃掉，读数列就漂到面板最右。
        grid->addWidget(new QWidget(group), 0, 6, 7, 1);
        // 列宽按内容确定；F_des 小幅伸展，多余宽度主要留在尾部。
        grid->setColumnStretch(2, 1);
        grid->setColumnStretch(6, 3);

        for (int i = 0; i < 6; ++i)
        {
            axis_name_label_[i] = new QLabel(
                QString("%1 [%2]").arg(kAxisNames[i]).arg(kAxisUnits[i]), group);
            force_axis_cb_[i] = new QCheckBox(group);
            force_axis_cb_[i]->setToolTip("勾选 = 力控轴 (S=1)，取消 = 位控轴 (S=0)");

            setpoint_spin_[i] = new QDoubleSpinBox(group);
            setpoint_spin_[i]->setDecimals(2);
            setpoint_spin_[i]->setSingleStep(i < 3 ? 0.5 : 0.1);
            setpoint_spin_[i]->setRange(
                i < 3 ? -20.0 : -5.0, i < 3 ? 20.0 : 5.0);
            setpoint_spin_[i]->setMinimumWidth(
                minTextWidth(monoUiFont(), QString::fromUtf8("-20.00"), 44));
            // 默认 Preferred 不会填满被 stretch 拉宽的单元格，会留下一条空隙
            setpoint_spin_[i]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
            setMonoFont(setpoint_spin_[i]);
            // 不设右对齐：数字会和上下箭头按钮挤在一起

            // 实测/误差列：等宽 + 右对齐 + 按字体算的实际宽度，读数才能纵向一眼比较
            const auto make_value_label = [&](const char* tip) {
                auto* label = new QLabel("—", group);
                setMonoFont(label);
                label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
                label->setMinimumWidth(value_col_w);
                label->setToolTip(tip);
                return label;
            };
            meas_left_label_[i] = make_value_label("左臂实测");
            meas_right_label_[i] = make_value_label("右臂实测");
            err_left_label_[i] = make_value_label(
                "左臂 F_des − F_meas；仅力控轴有意义（位控轴灰显）");

            const int row = i + 1;
            grid->addWidget(axis_name_label_[i], row, 0);
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

        // ── 曲面贴合：力矩驱动的目标姿态偏置外环 ──
        // 与力控轴(S=1)的区别：输出不进关节位置积分，只偏置**目标姿态**，
        // 由位控轴(S=0)跟踪。因此 rx/ry/rz 必须是位控轴，否则外环无效。
        auto* align_group = new QGroupBox("曲面贴合（力矩 → 目标姿态偏置）", this);
        auto* align_grid = new QGridLayout(align_group);
        align_grid->setHorizontalSpacing(14);
        align_grid->setVerticalSpacing(6);
        align_grid->setContentsMargins(2, 4, 2, 2);
        // 同 force 表：输入列小幅伸展，多余宽度留在尾部
        align_grid->setColumnStretch(1, 1);
        align_grid->setColumnStretch(3, 1);
        align_grid->setColumnStretch(4, 3);

        align_enable_cb_ = new QCheckBox("启用贴合外环", align_group);
        align_enable_cb_->setToolTip(
            "compliance_align_enable：唯一选中的平移力控轴作为贴合法向。\n"
            "选 Z 时只绕 X/Y 修正姿态，由旋转位控轴(S=0)跟踪。\n"
            "未选或选择多个平移力控轴时暂停贴合；失去接触后自动回中。");

        align_rcc_cb_ = new QCheckBox("绕接触点旋转 (RCC)", align_group);
        align_rcc_cb_->setChecked(true);   // 与控制器默认一致
        align_rcc_cb_->setToolTip(
            "compliance_align_rcc：转动中心取接触点估计 Δ⊥ = (f×τ)/|f|²，\n"
            "使接触点不迁移，避免越贴越「嘬入」。");

        align_max_spin_ = new QDoubleSpinBox(align_group);
        align_max_spin_->setDecimals(1);
        align_max_spin_->setRange(0.0, 90.0);
        align_max_spin_->setSingleStep(1.0);
        align_max_spin_->setValue(15.0);   // 与控制器默认 0.26 rad 一致
        align_max_spin_->setSuffix(" °");
        align_max_spin_->setMinimumWidth(
            minTextWidth(monoUiFont(), QString::fromUtf8("90.0 °"), 44));   // 带后缀，不能给固定 maxWidth
        align_max_spin_->setToolTip(
            "compliance_align_max：姿态偏置上限，决定最大可纠正的倾角（默认 15°）。");

        align_release_spin_ = new QDoubleSpinBox(align_group);
        align_release_spin_->setDecimals(2);
        align_release_spin_->setRange(0.0, 10.0);
        align_release_spin_->setSingleStep(0.1);
        align_release_spin_->setValue(1.0);   // 与控制器默认一致
        align_release_spin_->setSuffix(" 1/s");
        align_release_spin_->setMinimumWidth(
            minTextWidth(monoUiFont(), QString::fromUtf8("10.00 1/s"), 44));
        align_release_spin_->setToolTip(
            "compliance_align_release：失去接触或力矩消失后偏置的回中速率（默认 1.0）。");

        align_left_label_ = new QLabel("—", align_group);
        align_right_label_ = new QLabel("—", align_group);
        align_left_label_->setMinimumWidth(70);
        align_right_label_->setMinimumWidth(70);
        align_left_label_->setStyleSheet("QLabel { color: #888; }");
        align_right_label_->setStyleSheet("QLabel { color: #888; }");

        align_grid->addWidget(align_enable_cb_, 0, 0, 1, 2);
        align_grid->addWidget(align_rcc_cb_, 0, 2, 1, 2);
        align_grid->addWidget(new QLabel("最大偏置角", align_group), 1, 0);
        align_grid->addWidget(align_max_spin_, 1, 1);
        align_grid->addWidget(new QLabel("回中速率", align_group), 1, 2);
        align_grid->addWidget(align_release_spin_, 1, 3);
        align_grid->addWidget(new QLabel("实测偏置 L", align_group), 2, 0);
        align_grid->addWidget(align_left_label_, 2, 1);
        align_grid->addWidget(new QLabel("实测偏置 R", align_group), 2, 2);
        align_grid->addWidget(align_right_label_, 2, 3);

        align_hint_label_ = new QLabel(
            "外环偏置的是目标姿态，必须由位控轴跟踪：rx/ry/rz 需 S=0。", align_group);
        align_hint_label_->setWordWrap(true);
        align_hint_label_->setStyleSheet("QLabel { color: #666; font-size: 11px; }");
        align_grid->addWidget(align_hint_label_, 3, 0, 1, 4);

        main->addWidget(align_group);

        connect(align_enable_cb_, &QCheckBox::toggled,
                this, &ComplianceForcePanel::onUserEdited);
        connect(align_rcc_cb_, &QCheckBox::toggled,
                this, &ComplianceForcePanel::onUserEdited);
        connect(align_max_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &ComplianceForcePanel::onUserEdited);
        connect(align_release_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &ComplianceForcePanel::onUserEdited);

        auto* btn_row = new QHBoxLayout();
        btn_row->setSpacing(10);        apply_btn_ = new QPushButton("应用设定", this);
        apply_btn_->setToolTip(
            "将力控轴选择、F_des 与曲面贴合外环参数写入 /ocs2_arm_controller");
        sync_btn_ = new QPushButton("从控制器同步", this);
        sync_btn_->setToolTip("用最新 compliance_force_status 覆盖本面板编辑框");
        zero_wrench_btn_ = new QPushButton("传感器清零", this);
        zero_wrench_btn_->setToolTip(
            "所有控制模式均可清零；重新标定重力补偿后的残余零偏，期间请保持双臂静止且末端无接触");
        applyButtonVariant(apply_btn_, "primary");
        applyButtonVariant(zero_wrench_btn_, "danger");
        btn_row->addWidget(apply_btn_);
        btn_row->addWidget(sync_btn_);
        btn_row->addStretch();   // 销毁性操作推到右侧，避免顺手误按
        btn_row->addWidget(zero_wrench_btn_);
        main->addLayout(btn_row);

        auto* ident_tabs = new QTabWidget(this);
        ident_tabs->setDocumentMode(true);
        ident_tabs->setUsesScrollButtons(true);
        auto* ident_group = new QWidget(ident_tabs);
        auto* ident_layout = new QVBoxLayout(ident_group);
        ident_layout->setContentsMargins(6, 6, 6, 6);
        ident_layout->setSpacing(6);

        // 设置行：用网格 + 显式宽度，窄面板下也不会把「幅值」挤出边界
        auto* ident_settings = new QGridLayout();
        ident_settings->setHorizontalSpacing(8);
        ident_settings->setVerticalSpacing(4);
        identification_arm_combo_ = new QComboBox(ident_group);
        identification_arm_combo_->addItem("左臂", "left");
        identification_arm_combo_->addItem("右臂", "right");
        identification_arm_combo_->setMinimumWidth(
            minTextWidth(QApplication::font(), QString::fromUtf8("左臂"), 60));
        identification_joint_spin_ = new QSpinBox(ident_group);
        identification_joint_spin_->setRange(0, 15);
        identification_joint_spin_->setMinimumWidth(
            minTextWidth(monoUiFont(), QString::fromUtf8("00"), 40));
        identification_joint_amplitude_spin_ = new QDoubleSpinBox(ident_group);
        identification_joint_amplitude_spin_->setDecimals(4);
        identification_joint_amplitude_spin_->setRange(0.001, 0.05);
        identification_joint_amplitude_spin_->setSingleStep(0.002);
        identification_joint_amplitude_spin_->setValue(0.01);
        identification_joint_amplitude_spin_->setSuffix(" rad");
        // 50 = 增减按钮 20 + 左右 padding 14 + 边框 + 余量；40 会把 "rad" 裁掉一个字符
        identification_joint_amplitude_spin_->setMinimumWidth(
            minTextWidth(monoUiFont(), QString::fromUtf8("0.0100 rad"), 50));
        identification_joint_amplitude_spin_->setToolTip(
            "多正弦激励的关节幅值 RMS。\n"
            "越小越安全，但信噪比下降；辨识给出 ζ、T 随幅度的漂移用来判断摩擦影响。");
        ident_settings->addWidget(new QLabel("手臂", ident_group), 0, 0);
        ident_settings->addWidget(identification_arm_combo_, 0, 1);
        ident_settings->addWidget(new QLabel("关节", ident_group), 0, 2);
        ident_settings->addWidget(identification_joint_spin_, 0, 3);
        ident_settings->addWidget(new QLabel("幅值", ident_group), 0, 4);
        ident_settings->addWidget(identification_joint_amplitude_spin_, 0, 5);
        ident_settings->setColumnStretch(5, 1);   // 幅值输入框吃掉多余宽度
        // 不要再给不存在的第 6 列设 stretch：QGridLayout 会凭空造一列把宽度吸走，
        // 结果幅值框停在最小宽度，"rad" 被裁成 "rac"。
        ident_layout->addLayout(ident_settings);

        // 按钮：2×2 网格，窄面板也能全部露出；「停止」用描边红而不是实心红
        auto* ident_buttons = new QGridLayout();
        ident_buttons->setHorizontalSpacing(6);
        ident_buttons->setVerticalSpacing(4);
        position_identification_btn_ = new QPushButton("采集当前关节", ident_group);
        all_joint_identification_btn_ = new QPushButton("采集双臂全部关节", ident_group);
        contact_identification_btn_ = new QPushButton("接触链路辨识", ident_group);
        stop_identification_btn_ = new QPushButton("停止辨识", ident_group);
        position_identification_btn_->setToolTip(
            "无接触条件下辨识关节位置下发 q_cmd 到实测 q_measured 的增益与相位");
        all_joint_identification_btn_->setToolTip(
            "对所选手臂的全部关节依次做同样的多正弦激励与辨识");
        contact_identification_btn_->setToolTip(
            "保持接触条件下辨识实际位移到接触力的增益与相位");
        ident_buttons->addWidget(position_identification_btn_, 0, 0);
        ident_buttons->addWidget(all_joint_identification_btn_, 0, 1);
        ident_buttons->addWidget(contact_identification_btn_, 1, 0);
        ident_buttons->addWidget(stop_identification_btn_, 1, 1);
        ident_buttons->setColumnStretch(2, 1);
        applyButtonVariant(stop_identification_btn_, "danger");
        ident_layout->addLayout(ident_buttons);

        identification_plot_ = new PhaseIdentificationPlot(ident_group);
        identification_plot_->setMinimumHeight(260);
        ident_layout->addWidget(identification_plot_, 1);

        identification_results_table_ = new QTableWidget(0, 5, ident_group);
        identification_results_table_->setHorizontalHeaderLabels(
            {"手臂", "关节", "频率[Hz]", "增益", "相位[deg]"});
        identification_results_table_->horizontalHeader()->setSectionResizeMode(
            QHeaderView::Stretch);
        identification_results_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
        identification_results_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
        identification_results_table_->setFont(monoUiFont());
        identification_results_table_->setProperty("mono", true);
        // 表头不继承表格字体，必须显式设；否则会用默认字号，看起来比正文还大
        identification_results_table_->horizontalHeader()->setFont(monoUiFont());
        identification_results_table_->horizontalHeader()->setProperty("mono", true);
        identification_results_table_->setMinimumHeight(110);
        identification_results_table_->setMaximumHeight(170);
        ident_layout->addWidget(identification_results_table_);

        export_identification_btn_ = new QPushButton("导出 Excel", ident_group);
        export_identification_btn_->setToolTip(
            "将当前逐轴辨识结果导出为 Excel 可直接打开的 UTF-8 CSV 文件");
        auto* export_row = new QHBoxLayout();
        export_row->addWidget(export_identification_btn_);
        export_row->addStretch();
        ident_layout->addLayout(export_row);

        // 垂直可滚动：面板被拖窄/拖矮时不会把内容切掉
        auto* ident_scroll = new QScrollArea(ident_tabs);
        ident_scroll->setWidgetResizable(true);
        ident_scroll->setFrameShape(QFrame::NoFrame);
        ident_scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        ident_scroll->setMinimumWidth(220);   // 允许 dock 缩窄，内容横向滚动
        ident_scroll->setWidget(ident_group);

        // 标签名尽量短：tab bar 的最小宽度直接决定整个面板能缩到多窄；
        // 每页都包一层滚动区，否则页面内容会反过来把面板锁在很宽的尺寸上。
        const auto wrap_scroll = [this](QWidget* page) {
            auto* scroll = new QScrollArea(this);
            scroll->setWidgetResizable(true);
            scroll->setFrameShape(QFrame::NoFrame);
            scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
            scroll->setMinimumWidth(220);
            scroll->setWidget(page);
            return scroll;
        };
        ident_tabs->addTab(ident_scroll, "位置环 / 接触辨识");
        ident_tabs->addTab(wrap_scroll(makePayloadWidget()), "负载辨识");
        ident_tabs->addTab(wrap_scroll(makePathWidget()), "轨迹规划");
        main->addWidget(ident_tabs);

        auto* hint = new QLabel(
            "仅在 COMPLIANCE 模式下有数据。F_meas 已含反馈符号约定。", this);
        hint->setWordWrap(true);
        setHintRole(hint, "hint");
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
            applyStateBadge(state_badge_, false);
            telemetry_label_->setText("无数据（控制器不在 COMPLIANCE 或未响应）");
            frame_label_->setText("frame: —");
            for (int i = 0; i < 6; ++i)
            {
                meas_left_label_[i]->setText("—");
                meas_right_label_[i]->setText("—");
                err_left_label_[i]->setText("—");
                meas_left_label_[i]->setStyleSheet("QLabel { color: #888; }");
                meas_right_label_[i]->setStyleSheet("QLabel { color: #888; }");
                err_left_label_[i]->setStyleSheet("QLabel { color: #888; }");
                axis_name_label_[i]->setStyleSheet("QLabel { color: #888; }");
            }
            align_left_label_->setText("—");
            align_right_label_->setText("—");
            align_left_label_->setStyleSheet("QLabel { color: #888; }");
            align_right_label_->setStyleSheet("QLabel { color: #888; }");
            align_hint_label_->setText("COMPLIANCE 状态不可用：无法读取贴合外环状态。");
            setHintRole(align_hint_label_, "hint");
        });

        // ── 瞬时提示浮层看门狗 ──
        // 所有 status_label_->setText() 都是"操作反馈"；这里按文本变化贴样式
        // 并计时 6 s 后自动收起，避免和 20 Hz 的实时遥测互相刷掉。
        applyStateBadge(state_badge_, false);
        notice_watchdog_ = new QTimer(this);
        notice_watchdog_->setInterval(400);
        connect(notice_watchdog_, &QTimer::timeout, this, [this]() {
            if (!status_label_) return;
            const QString current = status_label_->text();
            if (current != last_notice_text_)
            {
                last_notice_text_ = current;
                last_notice_timer_.restart();
                applyNoticeStyle(status_label_, current);
                status_label_->setVisible(!current.isEmpty());
                return;
            }
            if (status_label_->isVisible() && last_notice_timer_.isValid() &&
                last_notice_timer_.elapsed() > 6000)
            {
                status_label_->setVisible(false);
            }
        });
        notice_watchdog_->start();

        // 没有 variant 的按钮（采集/导出/上移…）也统一成手型光标 + 同一最小高度，
        // 免得同一面板里按钮高矮不齐。
        for (auto* button : findChildren<QPushButton*>())
        {
            button->setCursor(Qt::PointingHandCursor);
            button->setMinimumHeight(26);
        }
    }

    ComplianceForcePanel::~ComplianceForcePanel()
    {
        if (payload_process_ && payload_process_->state() != QProcess::NotRunning)
        {
            payload_process_->disconnect(this);
            payload_process_->terminate();
            if (!payload_process_->waitForFinished(2000))
            {
                payload_process_->kill();
                payload_process_->waitForFinished(1000);
            }
        }
    }

    void ComplianceForcePanel::dockToRightAreaOnce()
    {
        // 沿父链找到承载本面板的 QDockWidget（PanelDockWidget 是 QDockWidget 子类），
        // 再找它的 QMainWindow。找不到就什么都不做（离屏渲染 / 单元测试场景）。
        QDockWidget* dock = nullptr;
        for (QWidget* widget = parentWidget(); widget && !dock; widget = widget->parentWidget())
        {
            dock = qobject_cast<QDockWidget*>(widget);
        }
        if (!dock) return;

        auto* window = qobject_cast<QMainWindow*>(dock->parentWidget());
        if (!window) return;

        if (window->dockWidgetArea(dock) == Qt::RightDockWidgetArea) return;
        // 已经在右侧才不搬：否则每次启动都会把用户拖好的位置重置。
        window->addDockWidget(Qt::RightDockWidgetArea, dock);
        dock->show();
    }

    void ComplianceForcePanel::onInitialize()
    {
        // 放到右侧停靠区。
        // rviz2 的 VisualizationFrame::loadPanels() 调 addPanelByName(name, class_id)，
        // 而它的默认参数是 Qt::LeftDockWidgetArea；QMainWindow State 又是在面板
        // 创建**之前** restoreState 的（loadWindowGeometry 先于 loadPanels），所以
        // 配置文件里改 QMainWindow State / Panels 顺序都不可能挪动自定义面板。
        // 只能在这里自己搬一次。
        dockToRightAreaOnce();

        node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        // 参数客户端要指向真正在控机器人的控制器：分体 /ocs2_arm_controller，
        // 全身 /ocs2_wbc_controller。launch 通过额外参数 wbc_available 告知模式。
        if (!node_->has_parameter("wbc_available"))
            node_->declare_parameter("wbc_available", false);
        param_node_ = node_->get_parameter("wbc_available").as_bool()
                          ? "/ocs2_wbc_controller" : "/ocs2_arm_controller";
        param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, param_node_);
        RCLCPP_INFO(node_->get_logger(),
                    "ComplianceForcePanel: COMPLIANCE 控制器 = %s", param_node_.c_str());

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

        // 轨迹规划标签页（compliance_path）需要节点才能建发布/订阅。
        // 脚点默认跟在 RViz 的 Fixed Frame 下：分体是 base_footprint，全身按
        // 机型/task 由 launch 改写（world 或控制系），不在面板里写死。
        if (path_widget_)
        {
            QString fixed_frame;
            if (auto* context = getDisplayContext())
            {
                if (auto* frame_manager = context->getFrameManager())
                {
                    fixed_frame = QString::fromStdString(frame_manager->getFixedFrame());
                    // 运行中在 RViz 里换 Fixed Frame 时跟着换（用户手改过就不覆盖）
                    connect(frame_manager, &rviz_common::FrameManagerIface::fixedFrameChanged,
                            path_widget_,
                            [this, frame_manager]()
                            {
                                if (!path_widget_) return;
                                path_widget_->setDefaultFrame(
                                    QString::fromStdString(frame_manager->getFixedFrame()));
                            });
                }
            }
            path_widget_->initialize(node_, fixed_frame);
        }
    }

    void ComplianceForcePanel::load(const rviz_common::Config& config)
    {
        Panel::load(config);
    }

    void ComplianceForcePanel::resizeEvent(QResizeEvent* event)
    {
        Panel::resizeEvent(event);
        applyUiScale();
    }

    void ComplianceForcePanel::applyUiScale()
    {
        // 基准宽度：dock 宽 420 px 时用原始字号。
        // 步进 5%、夹在 85%~125% —— 连续按像素缩放会让字重渲染变模糊，
        // 而且字号变化会反过来改 sizeHint，连续微调会抖动。
        // 上限刻意压在 125%：更宽的 dock 应该把空间给输入框和图表，而不是巨字。
        constexpr int kReferenceWidth = 560;
        if (width() <= 0) return;
        int percent = static_cast<int>(std::lround(100.0 * width() / kReferenceWidth / 5.0)) * 5;
        percent = std::clamp(percent, 90, 110);
        if (percent == ui_scale_percent_) return;
        ui_scale_percent_ = percent;

        g_ui_scale = percent / 100.0;
        setStyleSheet(panelStyle());   // @SMALL@ 换成新字号，整面板重新 polish

        // 正文：整块面板换字体，没有显式字体的子控件全部跟着变
        QFont base = QApplication::font();
        if (base.pointSizeF() > 0.0) base.setPointSizeF(base.pointSizeF() * g_ui_scale);
        setFont(base);

        // 等宽字体是显式设置的，必须跟着重刷（打了 "mono" 标记的那些）
        for (auto* widget : findChildren<QWidget*>())
        {
            if (!widget->property("mono").toBool()) continue;
            widget->setFont(monoUiFont());
        }

        // 徽标/提示条的音量随字号走
        applyStateBadge(state_badge_, have_status_);
        if (status_label_ && status_label_->isVisible())
            applyNoticeStyle(status_label_, status_label_->text());
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
            status_label_->setText("清零失败: 传感器清零服务不可用，请检查控制器是否已加载");
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

        // 路径回放状态（控制器侧权威结果）→ 轨迹规划页的提示行。
        // 三个字段都是定长 string[2] / uint32[2]，下标 0=左臂、1=右臂。
        if (path_widget_)
        {
            static constexpr const char* kArmNames[2] = {"left", "right"};
            for (int side = 0; side < 2; ++side)
            {
                path_widget_->onControllerPathState(
                    QString::fromLatin1(kArmNames[side]), msg->path_sequence[side],
                    QString::fromStdString(msg->path_state[side]),
                    QString::fromStdString(msg->path_message[side]));
            }
        }
        // Older controllers leave these fields at zero (unset), not a 0 N limit.
        for (int i = 0; i < 6; ++i)
        {
            const double maximum = i < 3 ? 20.0 : 5.0;
            const double reported = i < 3 ? msg->force_setpoint_limit : msg->torque_setpoint_limit;
            const double limit = std::clamp(
                std::isfinite(reported) && reported > 0.0 ? reported : maximum,
                i < 3 ? 0.1 : 0.01, maximum);
            QSignalBlocker blocker(setpoint_spin_[i]);
            setpoint_spin_[i]->setRange(-limit, limit);
        }
        const int joint_count = identification_arm_combo_->currentData().toString() == "right"
            ? msg->right_joint_count : msg->left_joint_count;
        identification_joint_spin_->setMaximum(std::max(0, joint_count - 1));

        const QString ft = QString("FT L=%1 R=%2")
                               .arg(msg->left_ft_active ? "on" : "off")
                               .arg(msg->right_ft_active ? "on" : "off");
        QString text = QString("%1  |  zero=%2")
                           .arg(ft)
                           .arg(msg->zero_cal_done ? "done" : "pending");
        if (msg->identification_active)
        {
            const QString channel = msg->identification_mode == 1
                ? QString("J%1").arg(msg->identification_joint)
                : QString("axis%1").arg(msg->identification_axis);
            text += QString("  |  辨识 %1 @ %2 Hz")
                .arg(channel)
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
        applyStateBadge(state_badge_, true);
        telemetry_label_->setText(text);
        frame_label_->setText(
            QString("frame: %1   sign=%2")
                .arg(QString::fromStdString(msg->header.frame_id))
                .arg(msg->force_feedback_sign, 0, 'f', 0));

        updateMeasuredLabels(*msg);
        updateAlignLabels(*msg);

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
            // 只有力控轴的读数有意义；位控轴灰显，避免把 F_des 当成有效目标。
            const QString readout = force_axis
                ? "QLabel { color: #1E88E5; font-weight: bold; }"
                : "QLabel { color: #888; }";
            meas_left_label_[i]->setStyleSheet(readout);
            meas_right_label_[i]->setStyleSheet(readout);
            // 误差：力控轴上超差标红、到位标绿，位控轴灰显
            const double err = des - ml;
            const double tol = i < 3 ? 1.0 : 0.2;
            if (!force_axis)
                err_left_label_[i]->setStyleSheet("QLabel { color: #888; }");
            else if (std::abs(err) > tol)
                err_left_label_[i]->setStyleSheet(
                    "QLabel { color: #E53935; font-weight: bold; }");
            else
                err_left_label_[i]->setStyleSheet(
                    "QLabel { color: #43A047; font-weight: bold; }");
            // 轴名跟着勾选走：一眼看出哪些轴在力控
            axis_name_label_[i]->setStyleSheet(
                force_axis ? "QLabel { font-weight: bold; }" : QString());
        }
    }

    void ComplianceForcePanel::updateAlignLabels(const StatusMsg& msg)
    {
        const auto paint = [&](QLabel* label, const std::array<double, 3>& bias) {
            const double angle = std::sqrt(
                bias[0] * bias[0] + bias[1] * bias[1] + bias[2] * bias[2]);
            if (!msg.align_enabled)
            {
                label->setText("—");
                label->setStyleSheet("QLabel { color: #888; }");
                label->setToolTip("贴合外环未启用 (compliance_align_enable=false)");
                return;
            }
            label->setText(QString("%1 °").arg(angle * kRadToDeg, 0, 'f', 2));
            // |bias| 顶到 align_max = 环境一直在推、贴合不到位（COMPLIANCE.md 诊断表）。
            const bool saturated = msg.align_max > 1e-9 && angle >= 0.99 * msg.align_max;
            label->setStyleSheet(saturated
                ? "QLabel { color: #C62828; font-weight: bold; }"
                : "QLabel { color: #2E7D32; font-weight: bold; }");
            label->setToolTip(QString("轴角向量 [%1 %2 %3] rad，模长 = 当前让位角%4")
                .arg(bias[0], 0, 'f', 4).arg(bias[1], 0, 'f', 4).arg(bias[2], 0, 'f', 4)
                .arg(saturated
                    ? "\n已达偏置上限：环境一直在推，贴合不到位（考虑弹性层或把 TCP 标到接触面中心）"
                    : ""));
        };
        paint(align_left_label_, msg.align_bias_left);
        paint(align_right_label_, msg.align_bias_right);

        // 偏置只能被位控轴跟踪：旋转轴仍为力控时外环不生效（控制器每 5 s 也会 WARN）。
        QString conflict;
        for (int i = 3; i < 6; ++i)
        {
            if (msg.task_selection[i] > 0.5)
                conflict += QString("%1 ").arg(kAxisNames[i]);
        }
        conflict = conflict.trimmed();

        if (!msg.align_enabled)
        {
            align_hint_label_->setText(
                "贴合外环未启用：偏置只作用于目标姿态，由位控轴 (S=0) 跟踪。");
            setHintRole(align_hint_label_, "hint");
        }
        else if (!conflict.isEmpty())
        {
            align_hint_label_->setText(QString(
                "⚠ %1 仍是力控 (S=1)，外环偏置的目标姿态没有位控轴去跟，贴合不会生效："
                "请取消这些轴的「力控」后再应用。").arg(conflict));
            setHintRole(align_hint_label_, "hint-warn");
        }
        else
        {
            align_hint_label_->setText(QString(
                "贴合外环已启用，rx/ry/rz 均为位控；偏置上限 %1°，|偏置| 贴近上限即贴合不到位。")
                .arg(msg.align_max * kRadToDeg, 0, 'f', 1));
            setHintRole(align_hint_label_, "hint-ok");
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
        {
            QSignalBlocker b1(align_enable_cb_);
            QSignalBlocker b2(align_max_spin_);
            QSignalBlocker b3(align_release_spin_);
            QSignalBlocker b4(align_rcc_cb_);
            align_enable_cb_->setChecked(msg.align_enabled);
            align_max_spin_->setValue(msg.align_max * kRadToDeg);
            align_release_spin_->setValue(msg.align_release);
            align_rcc_cb_->setChecked(msg.align_rcc);
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
                        "ComplianceForcePanel: %s params not ready", param_node_.c_str());
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
            rclcpp::Parameter("compliance_align_enable", align_enable_cb_->isChecked()),
            rclcpp::Parameter("compliance_align_max",
                              align_max_spin_->value() * kDegToRad),
            rclcpp::Parameter("compliance_align_release", align_release_spin_->value()),
            rclcpp::Parameter("compliance_align_rcc", align_rcc_cb_->isChecked()),
        };
        const bool align_enabled = align_enable_cb_->isChecked();
        const double align_max_deg = align_max_spin_->value();
        const double align_release = align_release_spin_->value();
        const bool align_rcc = align_rcc_cb_->isChecked();
        status_label_->setText("正在应用设定…");
        (void)param_client_->set_parameters(
            params,
            [this, selection, setpoint, align_enabled, align_max_deg,
             align_release, align_rcc](std::shared_future<
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
                    [this, selection, setpoint, align_enabled, align_max_deg,
                     align_release, align_rcc, success, reason]() {
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
                            "F_des=[%.2f %.2f %.2f %.2f %.2f %.2f] "
                            "align=%s max=%.1fdeg release=%.2f rcc=%s",
                            selection[0], selection[1], selection[2],
                            selection[3], selection[4], selection[5],
                            setpoint[0], setpoint[1], setpoint[2],
                            setpoint[3], setpoint[4], setpoint[5],
                            align_enabled ? "on" : "off", align_max_deg,
                            align_release, align_rcc ? "on" : "off");
                    },
                    Qt::QueuedConnection);
            });
    }

}  // namespace arms_rviz_control_plugin

PLUGINLIB_EXPORT_CLASS(arms_rviz_control_plugin::ComplianceForcePanel, rviz_common::Panel)
