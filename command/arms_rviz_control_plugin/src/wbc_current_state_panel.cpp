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

    title_label_ = std::make_unique<QLabel>("WBC Current State", this);
    title_label_->setStyleSheet("QLabel { font-weight: bold; font-size: 15px; }");
    main_layout_->addWidget(title_label_.get());

    capability_label_ = std::make_unique<QLabel>("Capability: waiting...", this);
    capability_label_->setWordWrap(true);
    main_layout_->addWidget(capability_label_.get());

    state_label_ = std::make_unique<QLabel>("State: waiting...", this);
    state_label_->setWordWrap(true);
    main_layout_->addWidget(state_label_.get());

    upper_button_layout_ = std::make_unique<QGridLayout>();

    base_button_ = std::make_unique<QPushButton>("Base: 无效", this);
    bimanual_button_ = std::make_unique<QPushButton>("双臂: 无效", this);
    left_arm_button_ = std::make_unique<QPushButton>("关闭左臂", this);
    right_arm_button_ = std::make_unique<QPushButton>("关闭右臂", this);

    connect(base_button_.get(), &QPushButton::clicked,
            this, &WbcCurrentStatePanel::onBaseButtonClicked);
    connect(bimanual_button_.get(), &QPushButton::clicked,
            this, &WbcCurrentStatePanel::onBimanualButtonClicked);
    connect(left_arm_button_.get(), &QPushButton::clicked,
            this, &WbcCurrentStatePanel::onLeftArmButtonClicked);
    connect(right_arm_button_.get(), &QPushButton::clicked,
            this, &WbcCurrentStatePanel::onRightArmButtonClicked);

    upper_button_layout_->addWidget(base_button_.get(), 0, 0);
    upper_button_layout_->addWidget(bimanual_button_.get(), 0, 1);
    upper_button_layout_->addWidget(left_arm_button_.get(), 1, 0);
    upper_button_layout_->addWidget(right_arm_button_.get(), 1, 1);

    main_layout_->addLayout(upper_button_layout_.get());

    body_button_layout_ = std::make_unique<QGridLayout>();

    body_free_button_ = std::make_unique<QPushButton>("自由", this);
    body_vertical_button_ = std::make_unique<QPushButton>("竖直", this);
    body_tracking_button_ = std::make_unique<QPushButton>("跟随", this);
    body_lock_button_ = std::make_unique<QPushButton>("锁定", this);

    connect(body_free_button_.get(), &QPushButton::clicked,
            this, &WbcCurrentStatePanel::onBodyFreeButtonClicked);
    connect(body_vertical_button_.get(), &QPushButton::clicked,
            this, &WbcCurrentStatePanel::onBodyVerticalButtonClicked);
    connect(body_tracking_button_.get(), &QPushButton::clicked,
            this, &WbcCurrentStatePanel::onBodyTrackingButtonClicked);
    connect(body_lock_button_.get(), &QPushButton::clicked,
            this, &WbcCurrentStatePanel::onBodyLockButtonClicked);

    body_button_layout_->addWidget(body_free_button_.get(), 0, 0);
    body_button_layout_->addWidget(body_vertical_button_.get(), 0, 1);
    body_button_layout_->addWidget(body_tracking_button_.get(), 1, 0);
    body_button_layout_->addWidget(body_lock_button_.get(), 1, 1);

    main_layout_->addLayout(body_button_layout_.get());
    main_layout_->addStretch();

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

QString WbcCurrentStatePanel::bodyStateText() const
{
    if (isBodyFree()) return "Body=Free";
    if (isBodyVertical()) return "Body=Relative";
    if (isBodyTracking()) return "Body=Tracking";
    if (isBodyLocked()) return "Body=Locked";
    return "Body=Unknown";
}

QString WbcCurrentStatePanel::capabilityText() const
{
    return QString("Capability | mobile=%1, bimanual=%2, relative=%3, tracking=%4, waist_lock=%5")
        .arg(capability_state_.has_mobile_base ? "true" : "false")
        .arg(capability_state_.has_bimanual_coupling ? "true" : "false")
        .arg(capability_state_.has_body_relative_constraint ? "true" : "false")
        .arg(capability_state_.body_tracking_ee_enabled ? "true" : "false")
        .arg(capability_state_.has_waist_lock ? "true" : "false");
}

QString WbcCurrentStatePanel::stateText() const
{
    QString base = isBaseLocked() ? "Base=Locked" : "Base=Unlocked";
    QString bimanual = isBimanualCoupled() ? "Coupled" : "Independent";
    QString left = isLeftArmEnabled() ? "L=On" : "L=Off";
    QString right = isRightArmEnabled() ? "R=On" : "R=Off";
    QString body = bodyStateText();

    return QString("%1 | %2 | %3 | %4 | %5")
        .arg(base, bimanual, left, right, body);
}

void WbcCurrentStatePanel::refreshUi()
{
    capability_label_->setText(capabilityText());
    state_label_->setText(stateText());

    // Base
    if (!capability_state_.has_mobile_base)
    {
        base_button_->setText("Base: 无效");
        base_button_->setEnabled(false);
    }
    else
    {
        base_button_->setEnabled(true);
        base_button_->setText(isBaseLocked() ? "解锁Base" : "锁定Base");
    }

    // Bimanual
    if (!capability_state_.has_bimanual_coupling)
    {
        bimanual_button_->setText("双臂: 无效");
        bimanual_button_->setEnabled(false);
    }
    else
    {
        bimanual_button_->setEnabled(true);
        bimanual_button_->setText(isBimanualCoupled() ? "耦合->独立" : "独立->耦合");
    }

    // Arms
    left_arm_button_->setEnabled(true);
    left_arm_button_->setText(isLeftArmEnabled() ? "关闭左臂" : "启用左臂");

    right_arm_button_->setEnabled(true);
    right_arm_button_->setText(isRightArmEnabled() ? "关闭右臂" : "启用右臂");

    // Body free
    body_free_button_->setEnabled(!isBodyFree());
    body_free_button_->setText("自由");

    // Body vertical (relative)
    body_vertical_button_->setEnabled(capability_state_.has_body_relative_constraint && !isBodyVertical());
    body_vertical_button_->setText("竖直");

    // Body tracking
    body_tracking_button_->setEnabled(capability_state_.body_tracking_ee_enabled && !isBodyTracking());
    if (!capability_state_.body_tracking_ee_enabled)
    {
        body_tracking_button_->setText("跟随： 无效");
    }
    else
    {
        body_tracking_button_->setText("跟随");
    }

    // Body lock / unlock
    if (!capability_state_.has_waist_lock)
    {
        body_lock_button_->setEnabled(false);
        body_lock_button_->setText("锁定: 无效");
    }
    else
    {
        body_lock_button_->setEnabled(true);
        body_lock_button_->setText(isBodyLocked() ? "解锁躯干" : "锁定躯干");
    }
}

void WbcCurrentStatePanel::publishModeCommand(const std::string& cmd)
{
    if (!mode_command_pub_) return;

    std_msgs::msg::String msg;
    msg.data = cmd;
    mode_command_pub_->publish(msg);
}

void WbcCurrentStatePanel::onBaseButtonClicked()
{
    if (!capability_state_.has_mobile_base) return;
    publishModeCommand(isBaseLocked() ? "BASE_UNLOCK" : "BASE_LOCK");
}

void WbcCurrentStatePanel::onBimanualButtonClicked()
{
    if (!capability_state_.has_bimanual_coupling) return;
    publishModeCommand(isBimanualCoupled() ? "ARMS_INDEPENDENT" : "ARMS_COUPLED");
}

void WbcCurrentStatePanel::onLeftArmButtonClicked()
{
    publishModeCommand(isLeftArmEnabled() ? "LEFT_ARM_DISABLE" : "LEFT_ARM_ENABLE");
}

void WbcCurrentStatePanel::onRightArmButtonClicked()
{
    publishModeCommand(isRightArmEnabled() ? "RIGHT_ARM_DISABLE" : "RIGHT_ARM_ENABLE");
}

void WbcCurrentStatePanel::onBodyFreeButtonClicked()
{
    if (isBodyFree()) return;
    publishModeCommand("BODY_FREE");
}

void WbcCurrentStatePanel::onBodyVerticalButtonClicked()
{
    if (!capability_state_.has_body_relative_constraint) return;
    if (isBodyVertical()) return;
    publishModeCommand("BODY_RELATIVE");
}

void WbcCurrentStatePanel::onBodyTrackingButtonClicked()
{
    if (!capability_state_.body_tracking_ee_enabled) return;
    if (isBodyTracking()) return;
    publishModeCommand("BODY_TRACKING");
}

void WbcCurrentStatePanel::onBodyLockButtonClicked()
{
    if (!capability_state_.has_waist_lock) return;
    publishModeCommand(isBodyLocked() ? "BODY_UNLOCK" : "BODY_LOCK");
}

}  // namespace arms_rviz_control_plugin

PLUGINLIB_EXPORT_CLASS(arms_rviz_control_plugin::WbcCurrentStatePanel, rviz_common::Panel)