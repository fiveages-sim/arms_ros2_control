#include "arms_rviz_control_plugin/waist_control_panel.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace arms_rviz_control_plugin
{
    WaistControlPanel::WaistControlPanel(QWidget* parent)
        : rviz_common::Panel(parent),
          up_pressed_(false),
          down_pressed_(false),
          left_pressed_(false),
          right_pressed_(false)
    {
        main_layout_ = std::make_unique<QVBoxLayout>(this);
        main_layout_->setSpacing(8);

        // =========================
        // Lifting speed scale
        // =========================
        lifting_layout_ = std::make_unique<QVBoxLayout>();
        lifting_layout_->setSpacing(4);

        lifting_label_ = std::make_unique<QLabel>("升降速度比例:", this);
        lifting_label_->setStyleSheet("QLabel { font-weight: bold; }");
        lifting_layout_->addWidget(lifting_label_.get());

        lifting_slider_layout_ = std::make_unique<QHBoxLayout>();

        lifting_slider_ = std::make_unique<QSlider>(Qt::Horizontal, this);
        lifting_slider_->setRange(0, 100);
        lifting_slider_->setValue(50);
        lifting_slider_->setSingleStep(1);
        lifting_slider_->setPageStep(5);

        lifting_value_label_ = std::make_unique<QLabel>("0.50", this);
        lifting_value_label_->setMinimumWidth(50);
        lifting_value_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

        connect(lifting_slider_.get(), &QSlider::valueChanged,
                this, &WaistControlPanel::onLiftingSliderChanged);

        lifting_slider_layout_->addWidget(lifting_slider_.get());
        lifting_slider_layout_->addWidget(lifting_value_label_.get());

        lifting_layout_->addLayout(lifting_slider_layout_.get());
        main_layout_->addLayout(lifting_layout_.get());

        // =========================
        // Turning speed scale
        // =========================
        turning_layout_ = std::make_unique<QVBoxLayout>();
        turning_layout_->setSpacing(4);

        turning_label_ = std::make_unique<QLabel>("旋转速度比例:", this);
        turning_label_->setStyleSheet("QLabel { font-weight: bold; }");
        turning_layout_->addWidget(turning_label_.get());

        turning_slider_layout_ = std::make_unique<QHBoxLayout>();

        turning_slider_ = std::make_unique<QSlider>(Qt::Horizontal, this);
        turning_slider_->setRange(0, 100);
        turning_slider_->setValue(50);
        turning_slider_->setSingleStep(1);
        turning_slider_->setPageStep(5);

        turning_value_label_ = std::make_unique<QLabel>("0.50", this);
        turning_value_label_->setMinimumWidth(50);
        turning_value_label_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

        connect(turning_slider_.get(), &QSlider::valueChanged,
                this, &WaistControlPanel::onTurningSliderChanged);

        turning_slider_layout_->addWidget(turning_slider_.get());
        turning_slider_layout_->addWidget(turning_value_label_.get());

        turning_layout_->addLayout(turning_slider_layout_.get());
        main_layout_->addLayout(turning_layout_.get());

        // =========================
        // Buttons
        // =========================
        button_layout_top_ = std::make_unique<QHBoxLayout>();
        button_layout_bottom_ = std::make_unique<QHBoxLayout>();

        up_button_ = std::make_unique<QPushButton>("上升", this);
        down_button_ = std::make_unique<QPushButton>("下降", this);
        left_button_ = std::make_unique<QPushButton>("左转", this);
        right_button_ = std::make_unique<QPushButton>("右转", this);

        up_button_->setStyleSheet(
            "QPushButton { background-color: #5cb85c; color: white; font-weight: bold; padding: 8px; }");
        down_button_->setStyleSheet(
            "QPushButton { background-color: #f0ad4e; color: white; font-weight: bold; padding: 8px; }");
        left_button_->setStyleSheet(
            "QPushButton { background-color: #5bc0de; color: white; font-weight: bold; padding: 8px; }");
        right_button_->setStyleSheet(
            "QPushButton { background-color: #337ab7; color: white; font-weight: bold; padding: 8px; }");

        connect(up_button_.get(), &QPushButton::pressed, this, &WaistControlPanel::onUpPressed);
        connect(up_button_.get(), &QPushButton::released, this, &WaistControlPanel::onUpReleased);

        connect(down_button_.get(), &QPushButton::pressed, this, &WaistControlPanel::onDownPressed);
        connect(down_button_.get(), &QPushButton::released, this, &WaistControlPanel::onDownReleased);

        connect(left_button_.get(), &QPushButton::pressed, this, &WaistControlPanel::onLeftPressed);
        connect(left_button_.get(), &QPushButton::released, this, &WaistControlPanel::onLeftReleased);

        connect(right_button_.get(), &QPushButton::pressed, this, &WaistControlPanel::onRightPressed);
        connect(right_button_.get(), &QPushButton::released, this, &WaistControlPanel::onRightReleased);

        button_layout_top_->addWidget(up_button_.get());
        button_layout_top_->addWidget(down_button_.get());

        button_layout_bottom_->addWidget(left_button_.get());
        button_layout_bottom_->addWidget(right_button_.get());

        main_layout_->addLayout(button_layout_top_.get());
        main_layout_->addLayout(button_layout_bottom_.get());
        main_layout_->addStretch();

        repeat_timer_ = std::make_unique<QTimer>(this);
        repeat_timer_->setInterval(100); // 100 ms，可按需要改成 50 ms
        connect(repeat_timer_.get(), &QTimer::timeout,
                this, &WaistControlPanel::onRepeatTimeout);

        updateScaleLabels();
    }

    WaistControlPanel::~WaistControlPanel() = default;

    void WaistControlPanel::onInitialize()
    {
        node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

        waist_lifting_publisher_ = node_->create_publisher<std_msgs::msg::Float64>(
            "/body_joint_controller/waist_lifting_command", 10);

        waist_turning_publisher_ = node_->create_publisher<std_msgs::msg::Float64>(
            "/body_joint_controller/waist_turning_command", 10);

        RCLCPP_INFO(node_->get_logger(), "Waist Control Panel initialized");
    }

    void WaistControlPanel::load(const rviz_common::Config& config)
    {
        rviz_common::Panel::load(config);

        int lifting_value = 50;
        int turning_value = 50;

        rviz_common::Config lifting_config = config.mapGetChild("lifting_slider");
        if (lifting_config.isValid())
        {
            const QVariant value = lifting_config.getValue();
            if (value.isValid())
            {
                lifting_value = value.toInt();
            }
        }

        rviz_common::Config turning_config = config.mapGetChild("turning_slider");
        if (turning_config.isValid())
        {
            const QVariant value = turning_config.getValue();
            if (value.isValid())
            {
                turning_value = value.toInt();
            }
        }

        if (lifting_slider_)
        {
            lifting_slider_->setValue(lifting_value);
        }
        if (turning_slider_)
        {
            turning_slider_->setValue(turning_value);
        }

        updateScaleLabels();
    }

    void WaistControlPanel::save(rviz_common::Config config) const
    {
        rviz_common::Panel::save(config);

        if (lifting_slider_)
        {
            config.mapMakeChild("lifting_slider").setValue(lifting_slider_->value());
        }
        if (turning_slider_)
        {
            config.mapMakeChild("turning_slider").setValue(turning_slider_->value());
        }
    }

    void WaistControlPanel::onLiftingSliderChanged(int)
    {
        updateScaleLabels();
    }

    void WaistControlPanel::onTurningSliderChanged(int)
    {
        updateScaleLabels();
    }

    double WaistControlPanel::getLiftingScale() const
    {
        if (!lifting_slider_)
        {
            return 0.0;
        }
        return static_cast<double>(lifting_slider_->value()) / 100.0;
    }

    double WaistControlPanel::getTurningScale() const
    {
        if (!turning_slider_)
        {
            return 0.0;
        }
        return static_cast<double>(turning_slider_->value()) / 100.0;
    }

    void WaistControlPanel::updateScaleLabels()
    {
        if (lifting_value_label_)
        {
            lifting_value_label_->setText(QString::number(getLiftingScale(), 'f', 2));
        }
        if (turning_value_label_)
        {
            turning_value_label_->setText(QString::number(getTurningScale(), 'f', 2));
        }
    }

    void WaistControlPanel::publishLifting(double value)
    {
        if (!waist_lifting_publisher_)
        {
            return;
        }

        std_msgs::msg::Float64 msg;
        msg.data = value;
        waist_lifting_publisher_->publish(msg);
    }

    void WaistControlPanel::publishTurning(double value)
    {
        if (!waist_turning_publisher_)
        {
            return;
        }

        std_msgs::msg::Float64 msg;
        msg.data = value;
        waist_turning_publisher_->publish(msg);
    }

    void WaistControlPanel::updateRepeatTimerState()
    {
        const bool any_pressed = up_pressed_ || down_pressed_ || left_pressed_ || right_pressed_;

        if (any_pressed)
        {
            if (repeat_timer_ && !repeat_timer_->isActive())
            {
                repeat_timer_->start();
            }
        }
        else
        {
            if (repeat_timer_ && repeat_timer_->isActive())
            {
                repeat_timer_->stop();
            }
        }
    }

    void WaistControlPanel::stopLifting()
    {
        publishLifting(0.0);
    }

    void WaistControlPanel::stopTurning()
    {
        publishTurning(0.0);
    }

    void WaistControlPanel::onUpPressed()
    {
        up_pressed_ = true;
        down_pressed_ = false;

        publishLifting(getLiftingScale());
        updateRepeatTimerState();
    }

    void WaistControlPanel::onUpReleased()
    {
        up_pressed_ = false;

        if (!down_pressed_)
        {
            stopLifting();
        }

        updateRepeatTimerState();
    }

    void WaistControlPanel::onDownPressed()
    {
        down_pressed_ = true;
        up_pressed_ = false;

        publishLifting(-getLiftingScale());
        updateRepeatTimerState();
    }

    void WaistControlPanel::onDownReleased()
    {
        down_pressed_ = false;

        if (!up_pressed_)
        {
            stopLifting();
        }

        updateRepeatTimerState();
    }

    void WaistControlPanel::onLeftPressed()
    {
        left_pressed_ = true;
        right_pressed_ = false;

        publishTurning(-getTurningScale());
        updateRepeatTimerState();
    }

    void WaistControlPanel::onLeftReleased()
    {
        left_pressed_ = false;

        if (!right_pressed_)
        {
            stopTurning();
        }

        updateRepeatTimerState();
    }

    void WaistControlPanel::onRightPressed()
    {
        right_pressed_ = true;
        left_pressed_ = false;

        publishTurning(getTurningScale());
        updateRepeatTimerState();
    }

    void WaistControlPanel::onRightReleased()
    {
        right_pressed_ = false;

        if (!left_pressed_)
        {
            stopTurning();
        }

        updateRepeatTimerState();
    }

    void WaistControlPanel::onRepeatTimeout()
    {
        if (up_pressed_ && !down_pressed_)
        {
            publishLifting(getLiftingScale());
        }
        else if (down_pressed_ && !up_pressed_)
        {
            publishLifting(-getLiftingScale());
        }

        if (left_pressed_ && !right_pressed_)
        {
            publishTurning(-getTurningScale());
        }
        else if (right_pressed_ && !left_pressed_)
        {
            publishTurning(getTurningScale());
        }
    }

} // namespace arms_rviz_control_plugin

PLUGINLIB_EXPORT_CLASS(arms_rviz_control_plugin::WaistControlPanel, rviz_common::Panel)