#pragma once
#include <rviz_common/display.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rclcpp/rclcpp.hpp>
#include <arms_ros2_control_msgs/msg/teleop_mode_state.hpp>
#include <arms_ros2_control_msgs/msg/teleop_mode.hpp>
#include <arms_ros2_control_msgs/msg/inputs.hpp>
#include <arms_ros2_control_msgs/msg/wbc_current_state.hpp>
#include <std_msgs/msg/int32.hpp>
#include <cstdint>
#include <memory>
#include <mutex>
#include "arms_rviz_control_plugin/event_bus_teleop_dashboard.hpp"
#include "arms_rviz_control_plugin/control_input_teleop_dashboard.hpp"
// Ogre/X11 headers may define None and break subsequent Qt includes.
#ifdef None
#undef None
#endif

class QMouseEvent;

namespace rviz_common {
class RenderPanel;
}
namespace rviz_rendering {
class RenderWindow;
}

namespace arms_rviz_control_plugin {
class TeleopModeDisplay : public rviz_common::Display {
    Q_OBJECT
public:
    TeleopModeDisplay();
    ~TeleopModeDisplay() override;
    void update(float wall_dt, float ros_dt) override;
    void reset() override;
protected:
    void onInitialize() override;
    void onEnable() override;
    void onDisable() override;
private Q_SLOTS:
    void updateScale();
private:
    struct Sample {
        bool enabled{false};
        bool update_mode{false};
        bool chassis_mode{false};
        bool body_tracking{false};
        bool dex_hand_session{false};
        bool yb_pause_semantics{false};
        bool mirror_movement{false};
        bool left_arm_paused{false};
        bool right_arm_paused{false};
        bool left_gripper_percent_mode{true};
        bool right_gripper_percent_mode{true};
        uint8_t left_stick_plane{0};
        uint8_t right_stick_plane{0};
        uint8_t body_stick_plane{0};
        uint8_t body_right_stick_plane{0};
        uint8_t topology{0};
        bool valid{false};
    };
    struct JoySample {
        bool active{false};
        uint8_t control_mode{0};
        bool mirror_movement{false};
        int32_t speed_level{0};
        int32_t arm_target{1};
        bool valid{false};
    };
    struct Data {
        std::mutex mutex;
        Sample teleop{};
        JoySample joy{};
        int32_t pending_arm_event{0};
        bool awaiting_yb_pause{false};
    };

    void installMouseHook();
    void removeMouseHook();
    void onRenderWindowMouse(QMouseEvent* event);
    void forwardMouseToRviz(QMouseEvent* event, int last_x, int last_y);
    void publishEvent(int32_t code);
    void publishArmToggle(int32_t code, const Sample& teleop);
    void flushPendingArmEvent();
    int32_t eventCodeForHit(EventBusTeleopDashboard::Hit hit) const;
    std::shared_ptr<Data> data_{std::make_shared<Data>()};
    rclcpp::Subscription<arms_ros2_control_msgs::msg::TeleopModeState>::SharedPtr subscription_;
    rclcpp::Subscription<arms_ros2_control_msgs::msg::TeleopMode>::SharedPtr joy_mode_subscription_;
    rclcpp::Subscription<arms_ros2_control_msgs::msg::Inputs>::SharedPtr joy_input_subscription_;
    rclcpp::Subscription<arms_ros2_control_msgs::msg::WbcCurrentState>::SharedPtr wbc_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr caps_subscription_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr event_publisher_;
    std::unique_ptr<EventBusTeleopDashboard> event_bus_dashboard_;
    std::unique_ptr<ControlInputTeleopDashboard> control_input_dashboard_;
    rviz_common::properties::FloatProperty* scale_property_{};
    rviz_common::RenderPanel* render_panel_{};
    rviz_rendering::RenderWindow* render_window_{};
    int mouse_last_x_{0};
    int mouse_last_y_{0};
    bool mouse_hook_installed_{false};
    float elapsed_{0};
};
}
