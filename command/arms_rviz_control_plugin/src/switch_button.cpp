#include "arms_rviz_control_plugin/switch_button.hpp"

#include <algorithm>
#include <QEvent>
#include <QPainter>
#include <QMouseEvent>
#include <QSizePolicy>

namespace arms_rviz_control_plugin
{

SwitchButton::SwitchButton(QWidget* parent)
    : QWidget(parent)
{
    setCursor(Qt::PointingHandCursor);
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    updateFixedSize();
}

QSize SwitchButton::sizeHint() const
{
    return QSize(72, 32);
}

QSize SwitchButton::minimumSizeHint() const
{
    return sizeHint();
}

void SwitchButton::updateFixedSize()
{
    const QSize target_size = sizeHint();
    if (size() != target_size)
    {
        setFixedSize(target_size);
    }
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

void SwitchButton::changeEvent(QEvent* event)
{
    if (event->type() == QEvent::FontChange || event->type() == QEvent::StyleChange)
    {
        updateFixedSize();
        updateGeometry();
    }
    QWidget::changeEvent(event);
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
    const qreal margin = std::max<qreal>(2.0, r.height() * 0.0625);
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
    f.setPixelSize(std::max(10, static_cast<int>(r.height() * 0.42)));
    p.setFont(f);

    p.setPen(QColor(255, 255, 255));
    const qreal textMargin = std::max<qreal>(4.0, r.height() * 0.18);
    if (isOn)
    {
        QRectF textRect(r.left() + textMargin, r.top(),
                        knobRect.left() - r.left() - 2 * textMargin, r.height());
        p.drawText(textRect, Qt::AlignCenter, text);
    }
    else
    {
        QRectF textRect(knobRect.right() + textMargin, r.top(),
                        r.right() - knobRect.right() - 2 * textMargin, r.height());
        p.drawText(textRect, Qt::AlignVCenter | Qt::AlignLeft, text);
    }
}

}  // namespace arms_rviz_control_plugin
