#include "arms_rviz_control_plugin/switch_button.hpp"

#include <QPainter>
#include <QMouseEvent>

namespace arms_rviz_control_plugin
{

SwitchButton::SwitchButton(QWidget* parent)
    : QWidget(parent)
{
    setCursor(Qt::PointingHandCursor);
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
}

void SwitchButton::setVisualState(VisualState state)
{
    if (state_ == state) return;
    state_ = state;
    update();
}

void SwitchButton::setClickable(bool clickable)
{
    if (clickable_ == clickable) return;
    clickable_ = clickable;
    setCursor(clickable_ ? Qt::PointingHandCursor : Qt::ArrowCursor);
    update();
}

void SwitchButton::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton && clickable_)
    {
        emit clicked();
    }
    QWidget::mousePressEvent(event);
}

void SwitchButton::paintEvent(QPaintEvent* /*event*/)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    QRectF r = rect().adjusted(1, 1, -1, -1);
    const qreal radius = r.height() / 2.0;

    QColor bgColor;
    QString text;
    bool isOn = false;

    switch (state_)
    {
        case VisualState::InvalidOffGray:
            bgColor = QColor(210, 210, 210);
            text = "OFF";
            isOn = false;
            break;
        case VisualState::DisabledOffRed:
            bgColor = QColor(244, 67, 54);
            text = "OFF";
            isOn = false;
            break;
        case VisualState::EnabledOnGreen:
            bgColor = QColor(46, 204, 113);
            text = "ON";
            isOn = true;
            break;
    }

    // 背景
    p.setPen(Qt::NoPen);
    p.setBrush(bgColor);
    p.drawRoundedRect(r, radius, radius);

    // 滑块
    const qreal margin = 2.0;
    const qreal knobSize = r.height() - 2 * margin;
    QRectF knobRect;
    if (isOn)
    {
        knobRect = QRectF(r.right() - margin - knobSize, r.top() + margin, knobSize, knobSize);
    }
    else
    {
        knobRect = QRectF(r.left() + margin, r.top() + margin, knobSize, knobSize);
    }

    p.setBrush(Qt::white);
    p.drawEllipse(knobRect);

    // 文字
    QFont f = font();
    f.setBold(true);
    f.setPointSize(10);
    p.setFont(f);

    p.setPen(QColor(255, 255, 255));
    if (isOn)
    {
        QRectF textRect(r.left() + 10, r.top(), r.width() / 2.0, r.height());
        p.drawText(textRect, Qt::AlignVCenter | Qt::AlignLeft, text);
    }
    else
    {
        QRectF textRect(r.left() + 34, r.top(), 24, r.height());
        p.drawText(textRect, Qt::AlignVCenter | Qt::AlignLeft, text);
    }
}

}  // namespace arms_rviz_control_plugin