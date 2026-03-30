#pragma once

#include <memory>
#include <string>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <QLabel>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QComboBox>
#include <QMetaObject>

#include <arms_ros2_control_msgs/msg/wbc_capability.hpp>
#include <arms_ros2_control_msgs/msg/wbc_current_state.hpp>

#include "arms_rviz_control_plugin/switch_button.hpp"

namespace arms_rviz_control_plugin
{

class WbcCurrentStatePanel : public rviz_common::Panel
{
    Q_OBJECT

public:
    explicit WbcCurrentStatePanel(QWidget* parent = nullptr);
    ~WbcCurrentStatePanel() override;

    void onInitialize() override;
    void load(const rviz_common::Config& config) override;
    void save(rviz_common::Config config) const override;

private Q_SLOTS:
    void onBaseToggled();
    void onBimanualToggled();
    void onLeftArmToggled();
    void onRightArmToggled();
    void onBodyModeChanged(int index);

private:
    using WbcCapabilityMsg = arms_ros2_control_msgs::msg::WbcCapability;
    using WbcCurrentStateMsg = arms_ros2_control_msgs::msg::WbcCurrentState;

    enum BodyModeIndex
    {
        BODY_MODE_FREE = 0,
        BODY_MODE_VERTICAL = 1,
        BODY_MODE_TRACKING = 2,
        BODY_MODE_LOCKED = 3
    };

    struct CapabilityState
    {
        bool has_mobile_base = false;
        bool has_body_relative_constraint = false;
        bool has_waist_lock = false;
        bool has_bimanual_coupling = false;
        bool body_tracking_ee_enabled = false;
    };

    struct CurrentState
    {
        uint8_t base_state = WbcCurrentStateMsg::BASE_LOCKED;
        uint8_t body_state = WbcCurrentStateMsg::BODY_LOCKED;
        uint8_t bimanual_state = WbcCurrentStateMsg::BIMANUAL_INDEPENDENT;
        uint8_t left_arm_state = WbcCurrentStateMsg::ARM_ENABLED;
        uint8_t right_arm_state = WbcCurrentStateMsg::ARM_ENABLED;
    };

private:
    void refreshUi();
    void publishModeCommand(const std::string& cmd);

    void onReceiveCapability(const WbcCapabilityMsg::SharedPtr msg);
    void onReceiveCurrentState(const WbcCurrentStateMsg::SharedPtr msg);

    bool isBaseLocked() const;
    bool isBimanualCoupled() const;
    bool isLeftArmEnabled() const;
    bool isRightArmEnabled() const;

    bool isBodyFree() const;
    bool isBodyVertical() const;
    bool isBodyTracking() const;
    bool isBodyLocked() const;

    int getCurrentBodyModeIndex() const;
    QString getBodyModeCommand(int modeIndex) const;
    void updateBodyComboBox();

    void updateSwitchVisualState(
        SwitchButton* sw,
        bool capability_available,
        bool logical_on);

private:
    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_command_pub_;
    rclcpp::Subscription<WbcCapabilityMsg>::SharedPtr capability_sub_;
    rclcpp::Subscription<WbcCurrentStateMsg>::SharedPtr state_sub_;

    CapabilityState capability_state_;
    CurrentState current_state_;

    std::unique_ptr<QVBoxLayout> main_layout_;
    std::unique_ptr<QGridLayout> upper_button_layout_;
    std::unique_ptr<QHBoxLayout> body_control_layout_;

    std::unique_ptr<QLabel> base_label_;
    std::unique_ptr<QLabel> bimanual_label_;
    std::unique_ptr<QLabel> left_arm_label_;
    std::unique_ptr<QLabel> right_arm_label_;

    std::unique_ptr<SwitchButton> base_switch_;
    std::unique_ptr<SwitchButton> bimanual_switch_;
    std::unique_ptr<SwitchButton> left_arm_switch_;
    std::unique_ptr<SwitchButton> right_arm_switch_;

    std::unique_ptr<QLabel> body_label_;
    std::unique_ptr<QComboBox> body_combo_box_;
};

}  // namespace arms_rviz_control_plugin