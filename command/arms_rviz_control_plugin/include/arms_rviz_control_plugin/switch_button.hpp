#pragma once

#include <QSize>
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

        QSize sizeHint() const override;
        QSize minimumSizeHint() const override;

        Q_SIGNALS:
            void clicked();

    protected:
        void changeEvent(QEvent* event) override;
        void paintEvent(QPaintEvent* event) override;
        void mousePressEvent(QMouseEvent* event) override;

    private:
        void updateFixedSize();

        VisualState state_ = VisualState::InvalidOffGray;
        bool clickable_ = true;
    };

}  // namespace arms_rviz_control_plugin
