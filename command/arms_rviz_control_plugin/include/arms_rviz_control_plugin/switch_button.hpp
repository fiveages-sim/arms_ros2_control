#pragma once

#include <QWidget>

namespace arms_rviz_control_plugin
{

    class SwitchButton : public QWidget
    {
        Q_OBJECT

    public:
        enum class VisualState
        {
            InvalidOffGray,
            DisabledOffRed,
            EnabledOnGreen
        };

        explicit SwitchButton(QWidget* parent = nullptr);

        void setVisualState(VisualState state);
        VisualState visualState() const { return state_; }

        void setClickable(bool clickable);
        bool isClickable() const { return clickable_; }

        QSize sizeHint() const override { return QSize(72, 32); }
        QSize minimumSizeHint() const override { return QSize(72, 32); }

        Q_SIGNALS:
            void clicked();

    protected:
        void paintEvent(QPaintEvent* event) override;
        void mousePressEvent(QMouseEvent* event) override;

    private:
        VisualState state_ = VisualState::InvalidOffGray;
        bool clickable_ = true;
    };

}  // namespace arms_rviz_control_plugin