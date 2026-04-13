#include "arms_rviz_control_plugin/wbc_current_state_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace arms_rviz_control_plugin
{

WbcCurrentStatePanel::WbcCurrentStatePanel(QWidget* parent)
    : Panel(parent)
{
    main_layout_ = std::make_unique<QVBoxLayout>(this);
    main_layout_->setContentsMargins(8, 4, 8, 4);
    main_layout_->setSpacing(2);

    upper_button_layout_ = std::make_unique<QGridLayout>();
    upper_button_layout_->setHorizontalSpacing(12);
    upper_button_layout_->setVerticalSpacing(2);

    const int label_width = 74;
    const QString label_style = "QLabel { font-size: 13px; padding: 0px; }";

    // Base
    base_label_ = std::make_unique<QLabel>("启用底盘:", this);
    base_label_->setFixedWidth(label_width);
    base_label_->setStyleSheet(label_style);
    base_switch_ = std::make_unique<SwitchButton>(this);

    auto* base_layout = new QHBoxLayout();
    base_layout->setContentsMargins(0, 0, 0, 0);
    base_layout->setSpacing(4);
    base_layout->addWidget(base_label_.get());
    base_layout->addWidget(base_switch_.get());
    base_layout->addStretch();

    auto* base_widget = new QWidget(this);
    base_widget->setLayout(base_layout);
    upper_button_layout_->addWidget(base_widget, 0, 0);

    // Bimanual
    bimanual_label_ = std::make_unique<QLabel>("双臂耦合:", this);
    bimanual_label_->setFixedWidth(label_width);
    bimanual_label_->setStyleSheet(label_style);
    bimanual_switch_ = std::make_unique<SwitchButton>(this);

    auto* bimanual_layout = new QHBoxLayout();
    bimanual_layout->setContentsMargins(0, 0, 0, 0);
    bimanual_layout->setSpacing(4);
    bimanual_layout->addWidget(bimanual_label_.get());
    bimanual_layout->addWidget(bimanual_switch_.get());
    bimanual_layout->addStretch();

    auto* bimanual_widget = new QWidget(this);
    bimanual_widget->setLayout(bimanual_layout);
    upper_button_layout_->addWidget(bimanual_widget, 0, 1);

    // Left arm
    left_arm_label_ = std::make_unique<QLabel>("启用左臂:", this);
    left_arm_label_->setFixedWidth(label_width);
    left_arm_label_->setStyleSheet(label_style);
    left_arm_switch_ = std::make_unique<SwitchButton>(this);

    auto* left_arm_layout = new QHBoxLayout();
    left_arm_layout->setContentsMargins(0, 0, 0, 0);
    left_arm_layout->setSpacing(4);
    left_arm_layout->addWidget(left_arm_label_.get());
    left_arm_layout->addWidget(left_arm_switch_.get());
    left_arm_layout->addStretch();

    auto* left_arm_widget = new QWidget(this);
    left_arm_widget->setLayout(left_arm_layout);
    upper_button_layout_->addWidget(left_arm_widget, 1, 0);

    // Right arm
    right_arm_label_ = std::make_unique<QLabel>("启用右臂:", this);
    right_arm_label_->setFixedWidth(label_width);
    right_arm_label_->setStyleSheet(label_style);
    right_arm_switch_ = std::make_unique<SwitchButton>(this);

    auto* right_arm_layout = new QHBoxLayout();
    right_arm_layout->setContentsMargins(0, 0, 0, 0);
    right_arm_layout->setSpacing(4);
    right_arm_layout->addWidget(right_arm_label_.get());
    right_arm_layout->addWidget(right_arm_switch_.get());
    right_arm_layout->addStretch();

    auto* right_arm_widget = new QWidget(this);
    right_arm_widget->setLayout(right_arm_layout);
    upper_button_layout_->addWidget(right_arm_widget, 1, 1);

    connect(base_switch_.get(), &SwitchButton::clicked,
            this, &WbcCurrentStatePanel::onBaseToggled);
    connect(bimanual_switch_.get(), &SwitchButton::clicked,
            this, &WbcCurrentStatePanel::onBimanualToggled);
    connect(left_arm_switch_.get(), &SwitchButton::clicked,
            this, &WbcCurrentStatePanel::onLeftArmToggled);
    connect(right_arm_switch_.get(), &SwitchButton::clicked,
            this, &WbcCurrentStatePanel::onRightArmToggled);

    main_layout_->addLayout(upper_button_layout_.get());

    // Body row
    auto* body_row_widget = new QWidget(this);
    body_control_layout_ = std::make_unique<QHBoxLayout>(body_row_widget);
    body_control_layout_->setContentsMargins(0, 0, 0, 0);
    body_control_layout_->setSpacing(4);

    body_label_ = std::make_unique<QLabel>("身体模式:", this);
    body_label_->setFixedWidth(label_width);
    body_label_->setStyleSheet(label_style);

    body_combo_box_ = std::make_unique<QComboBox>(this);
    body_combo_box_->setFixedHeight(28);
    body_combo_box_->setMinimumWidth(92);
    body_combo_box_->setStyleSheet(R"(
        QComboBox {
            font-size: 13px;
            padding: 2px 22px 2px 8px;
            border: 1px solid #bdbdbd;
            border-radius: 6px;
            background: white;
        }
        QComboBox:hover {
            border: 1px solid #9e9e9e;
        }
        QComboBox:focus {
            border: 1px solid #66bb6a;
        }
        QComboBox::drop-down {
            subcontrol-origin: padding;
            subcontrol-position: top right;
            width: 18px;
            border: none;
        }
        QComboBox QAbstractItemView {
            font-size: 13px;
            border: 1px solid #cfcfcf;
            selection-background-color: #e8f5e9;
            selection-color: black;
            outline: none;
        }
    )");

    body_combo_box_->addItem("自由", BODY_MODE_FREE);
    body_combo_box_->addItem("竖直", BODY_MODE_VERTICAL);
    body_combo_box_->addItem("跟随", BODY_MODE_TRACKING);
    body_combo_box_->addItem("锁定", BODY_MODE_LOCKED);

    connect(body_combo_box_.get(),
            QOverload<int>::of(&QComboBox::currentIndexChanged),
            this,
            &WbcCurrentStatePanel::onBodyModeChanged);

    body_control_layout_->addWidget(body_label_.get());
    body_control_layout_->addWidget(body_combo_box_.get());
    body_control_layout_->addStretch();

    main_layout_->addWidget(body_row_widget);

    refreshUi();
}

WbcCurrentStatePanel::~WbcCurrentStatePanel() = default;

void WbcCurrentStatePanel::onInitialize()
{
    node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

    mode_command_pub_ = node_->create_publisher<std_msgs::msg::String>(
        "mode_command", 10);

    capability_sub_ = node_->create_subscription<WbcCapabilityMsg>(
        "/ocs2_wbc_controller/wbc_capabilities",
        rclcpp::QoS(1).transient_local(),
        std::bind(&WbcCurrentStatePanel::onReceiveCapability, this, std::placeholders::_1));

    state_sub_ = node_->create_subscription<WbcCurrentStateMsg>(
        "/ocs2_wbc_controller/current_state",
        10,
        std::bind(&WbcCurrentStatePanel::onReceiveCurrentState, this, std::placeholders::_1));
}

void WbcCurrentStatePanel::load(const rviz_common::Config& config)
{
    Panel::load(config);
}

void WbcCurrentStatePanel::save(rviz_common::Config config) const
{
    Panel::save(config);
}

void WbcCurrentStatePanel::onReceiveCapability(const WbcCapabilityMsg::SharedPtr msg)
{
    if (!msg) return;

    capability_state_.has_mobile_base = msg->has_mobile_base;
    capability_state_.has_body_relative_constraint = msg->has_body_relative_constraint;
    capability_state_.has_waist_lock = msg->has_waist_lock;
    capability_state_.has_bimanual_coupling = msg->has_bimanual_coupling;
    capability_state_.body_tracking_ee_enabled = msg->body_tracking_ee_enabled;

    QMetaObject::invokeMethod(this, [this]() { refreshUi(); }, Qt::QueuedConnection);
}

void WbcCurrentStatePanel::onReceiveCurrentState(const WbcCurrentStateMsg::SharedPtr msg)
{
    if (!msg) return;

    current_state_.base_state = msg->base_state;
    current_state_.body_state = msg->body_state;
    current_state_.bimanual_state = msg->bimanual_state;
    current_state_.left_arm_state = msg->left_arm_state;
    current_state_.right_arm_state = msg->right_arm_state;

    QMetaObject::invokeMethod(this, [this]() { refreshUi(); }, Qt::QueuedConnection);
}

bool WbcCurrentStatePanel::isBaseLocked() const
{
    return current_state_.base_state == WbcCurrentStateMsg::BASE_LOCKED;
}

bool WbcCurrentStatePanel::isBimanualCoupled() const
{
    return current_state_.bimanual_state == WbcCurrentStateMsg::BIMANUAL_COUPLED;
}

bool WbcCurrentStatePanel::isLeftArmEnabled() const
{
    return current_state_.left_arm_state == WbcCurrentStateMsg::ARM_ENABLED;
}

bool WbcCurrentStatePanel::isRightArmEnabled() const
{
    return current_state_.right_arm_state == WbcCurrentStateMsg::ARM_ENABLED;
}

bool WbcCurrentStatePanel::isBodyFree() const
{
    return current_state_.body_state == WbcCurrentStateMsg::BODY_FREE;
}

bool WbcCurrentStatePanel::isBodyVertical() const
{
    return current_state_.body_state == WbcCurrentStateMsg::BODY_VERTICAL;
}

bool WbcCurrentStatePanel::isBodyTracking() const
{
    return current_state_.body_state == WbcCurrentStateMsg::BODY_TRACKING;
}

bool WbcCurrentStatePanel::isBodyLocked() const
{
    return current_state_.body_state == WbcCurrentStateMsg::BODY_LOCKED;
}

int WbcCurrentStatePanel::getCurrentBodyModeIndex() const
{
    if (isBodyFree()) return BODY_MODE_FREE;
    if (isBodyVertical()) return BODY_MODE_VERTICAL;
    if (isBodyTracking()) return BODY_MODE_TRACKING;
    if (isBodyLocked()) return BODY_MODE_LOCKED;
    return BODY_MODE_LOCKED;
}

QString WbcCurrentStatePanel::getBodyModeCommand(int modeIndex) const
{
    switch (modeIndex)
    {
        case BODY_MODE_FREE:
            return "BODY_FREE";
        case BODY_MODE_VERTICAL:
            return "BODY_RELATIVE";
        case BODY_MODE_TRACKING:
            return "BODY_TRACKING";
        case BODY_MODE_LOCKED:
            return "BODY_LOCK";
        default:
            return "";
    }
}

void WbcCurrentStatePanel::updateBodyComboBox()
{
    if (!body_combo_box_) return;

    body_combo_box_->blockSignals(true);
    body_combo_box_->clear();

    body_combo_box_->addItem("自由", BODY_MODE_FREE);

    if (capability_state_.has_body_relative_constraint)
    {
        body_combo_box_->addItem("竖直", BODY_MODE_VERTICAL);
    }

    if (capability_state_.body_tracking_ee_enabled)
    {
        body_combo_box_->addItem("跟随", BODY_MODE_TRACKING);
    }

    if (capability_state_.has_waist_lock)
    {
        body_combo_box_->addItem("锁定", BODY_MODE_LOCKED);
    }

    int current_mode = getCurrentBodyModeIndex();
    int index = body_combo_box_->findData(current_mode);
    if (index >= 0)
    {
        body_combo_box_->setCurrentIndex(index);
    }

    body_combo_box_->blockSignals(false);
    body_combo_box_->setEnabled(body_combo_box_->count() > 1);
}

void WbcCurrentStatePanel::updateSwitchVisualState(
    SwitchButton* sw,
    bool capability_available,
    bool logical_on)
{
    if (!sw) return;

    if (!capability_available)
    {
        sw->setVisualState(SwitchButton::VisualState::InvalidOffGray);
        sw->setClickable(false);
        return;
    }

    sw->setClickable(true);

    if (logical_on)
    {
        sw->setVisualState(SwitchButton::VisualState::EnabledOnGreen);
    }
    else
    {
        sw->setVisualState(SwitchButton::VisualState::DisabledOffRed);
    }
}

void WbcCurrentStatePanel::refreshUi()
{
    base_label_->setText("启用底盘:");
    updateSwitchVisualState(base_switch_.get(),
                            capability_state_.has_mobile_base,
                            !isBaseLocked());

    bimanual_label_->setText("双臂耦合:");
    updateSwitchVisualState(bimanual_switch_.get(),
                            capability_state_.has_bimanual_coupling,
                            isBimanualCoupled());

    left_arm_label_->setText("启用左臂:");
    updateSwitchVisualState(left_arm_switch_.get(),
                            true,
                            isLeftArmEnabled());

    right_arm_label_->setText("启用右臂:");
    updateSwitchVisualState(right_arm_switch_.get(),
                            true,
                            isRightArmEnabled());

    updateBodyComboBox();
}

void WbcCurrentStatePanel::publishModeCommand(const std::string& cmd)
{
    if (!mode_command_pub_) return;

    std_msgs::msg::String msg;
    msg.data = cmd;
    mode_command_pub_->publish(msg);
}

void WbcCurrentStatePanel::onBaseToggled()
{
    if (!capability_state_.has_mobile_base) return;
    publishModeCommand(isBaseLocked() ? "BASE_UNLOCK" : "BASE_LOCK");
}

void WbcCurrentStatePanel::onBimanualToggled()
{
    if (!capability_state_.has_bimanual_coupling) return;
    publishModeCommand(isBimanualCoupled() ? "ARMS_INDEPENDENT" : "ARMS_COUPLED");
}

void WbcCurrentStatePanel::onLeftArmToggled()
{
    publishModeCommand(isLeftArmEnabled() ? "LEFT_ARM_DISABLE" : "LEFT_ARM_ENABLE");
}

void WbcCurrentStatePanel::onRightArmToggled()
{
    publishModeCommand(isRightArmEnabled() ? "RIGHT_ARM_DISABLE" : "RIGHT_ARM_ENABLE");
}

void WbcCurrentStatePanel::onBodyModeChanged(int index)
{
    if (index < 0) return;

    int mode = body_combo_box_->itemData(index).toInt();
    QString command = getBodyModeCommand(mode);

    if (command.isEmpty()) return;

    int current_mode = getCurrentBodyModeIndex();
    if (mode != current_mode)
    {
        publishModeCommand(command.toStdString());
    }
}

}  // namespace arms_rviz_control_plugin

PLUGINLIB_EXPORT_CLASS(arms_rviz_control_plugin::WbcCurrentStatePanel, rviz_common::Panel)