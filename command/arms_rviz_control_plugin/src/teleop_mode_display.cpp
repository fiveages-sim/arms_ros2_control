#include <QMouseEvent>
#include <QtGlobal>
#include "arms_rviz_control_plugin/teleop_mode_display.hpp"
#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/render_system.hpp>
#include <rviz_rendering/render_window.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace arms_rviz_control_plugin {
namespace {
QPoint mouseLocalPos(const QMouseEvent* event) {
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    return event->position().toPoint();
#elif QT_VERSION >= QT_VERSION_CHECK(5, 14, 0)
    return event->localPos().toPoint();
#else
    return QPoint(event->x(), event->y());
#endif
}
}  // namespace

TeleopModeDisplay::TeleopModeDisplay() {
    scale_property_ = new rviz_common::properties::FloatProperty(
        "Scale", 1.0f, "Size of the teleop-mode overlay.", this, SLOT(updateScale()));
    scale_property_->setMin(0.5f);
    scale_property_->setMax(2.0f);
}
TeleopModeDisplay::~TeleopModeDisplay() {
    removeMouseHook();
    subscription_.reset();
    wbc_subscription_.reset();
    caps_subscription_.reset();
    event_publisher_.reset();
    dashboard_.reset();
}
void TeleopModeDisplay::onInitialize() {
    rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
    auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    event_publisher_ = node->create_publisher<std_msgs::msg::Int32>(
        "teleop/controller_state", rclcpp::QoS(10).reliable());
    subscription_ = node->create_subscription<arms_ros2_control_msgs::msg::TeleopModeState>(
        "teleop/mode_state", rclcpp::QoS(1).reliable().transient_local(),
        [data = data_](arms_ros2_control_msgs::msg::TeleopModeState::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(data->mutex);
            data->teleop.enabled = message->enabled;
            data->teleop.update_mode = message->update_mode;
            data->teleop.chassis_mode = message->chassis_mode;
            data->teleop.yb_pause_semantics = message->yb_pause_semantics;
            data->teleop.mirror_movement = message->mirror_movement;
            data->teleop.left_arm_paused = message->left_arm_paused;
            data->teleop.right_arm_paused = message->right_arm_paused;
            data->teleop.left_gripper_percent_mode = message->left_gripper_percent_mode;
            data->teleop.right_gripper_percent_mode = message->right_gripper_percent_mode;
            data->teleop.left_stick_plane = message->left_stick_plane;
            data->teleop.right_stick_plane = message->right_stick_plane;
            data->teleop.body_stick_plane = message->body_stick_plane;
            data->teleop.body_right_stick_plane = message->body_right_stick_plane;
            data->teleop.topology = message->topology;
            data->teleop.valid = true;
            // 仅记录；3/6 在 update() 里发，避免订阅回调里 publish
        });
    wbc_subscription_ = node->create_subscription<arms_ros2_control_msgs::msg::WbcCurrentState>(
        "ocs2_wbc_controller/current_state", rclcpp::QoS(1).transient_local(),
        [data = data_](arms_ros2_control_msgs::msg::WbcCurrentState::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(data->mutex);
            using State = arms_ros2_control_msgs::msg::WbcCurrentState;
            data->teleop.body_tracking = message->body_state == State::BODY_TRACKING;
        });
    caps_subscription_ = node->create_subscription<std_msgs::msg::Int32>(
        "teleop/optical_r0_caps",
        rclcpp::QoS(1).reliable().transient_local(),
        [data = data_](std_msgs::msg::Int32::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(data->mutex);
            data->teleop.dex_hand_session = message->data != 0;
        });
    installMouseHook();
}
void TeleopModeDisplay::installMouseHook() {
    if (!context_) return;
    auto* views = context_->getViewManager();
    if (!views) return;
    auto* panel = views->getRenderPanel();
    if (!panel) return;
    auto* window = panel->getRenderWindow();
    if (!window) return;
    if (mouse_hook_installed_ && panel == render_panel_ && window == render_window_) return;
    removeMouseHook();
    render_panel_ = panel;
    render_window_ = window;
    mouse_last_x_ = 0;
    mouse_last_y_ = 0;
    // RViz routes viewport mouse through this callback (not QWidget events).
    render_window_->setOnRenderWindowMouseEventsCallback(
        [this](QMouseEvent* event) { onRenderWindowMouse(event); });
    mouse_hook_installed_ = true;
}
void TeleopModeDisplay::removeMouseHook() {
    if (!mouse_hook_installed_ || !render_window_ || !render_panel_ || !context_) {
        render_window_ = nullptr;
        render_panel_ = nullptr;
        mouse_hook_installed_ = false;
        return;
    }
    // Restore a compatible forwarder (RenderPanel::onRenderWindowMouseEvents is protected).
    auto* panel = render_panel_;
    auto* ctx = context_;
    auto last_x = std::make_shared<int>(mouse_last_x_);
    auto last_y = std::make_shared<int>(mouse_last_y_);
    render_window_->setOnRenderWindowMouseEventsCallback(
        [panel, ctx, last_x, last_y](QMouseEvent* event) {
            const int prev_x = *last_x;
            const int prev_y = *last_y;
            const QPoint pos = mouseLocalPos(event);
            *last_x = pos.x();
            *last_y = pos.y();
            if (!ctx) return;
            panel->setFocus(Qt::MouseFocusReason);
            rviz_common::ViewportMouseEvent vme(panel, event, prev_x, prev_y);
            ctx->handleMouseEvent(vme);
            event->accept();
        });
    render_window_ = nullptr;
    render_panel_ = nullptr;
    mouse_hook_installed_ = false;
}
void TeleopModeDisplay::forwardMouseToRviz(QMouseEvent* event, int last_x, int last_y) {
    if (!context_ || !render_panel_) return;
    render_panel_->setFocus(Qt::MouseFocusReason);
    rviz_common::ViewportMouseEvent vme(render_panel_, event, last_x, last_y);
    context_->handleMouseEvent(vme);
    event->accept();
}
void TeleopModeDisplay::publishEvent(int32_t code) {
    if (!event_publisher_) return;
    std_msgs::msg::Int32 message;
    message.data = code;
    event_publisher_->publish(message);
}
void TeleopModeDisplay::publishArmToggle(int32_t code, const Sample& teleop) {
    using Msg = arms_ros2_control_msgs::msg::TeleopModeState;
    // FULL_BODY 默认 Y/B 是 WBC 臂开关；点 HUD L/R 先发 16，等 ATM 锁存暂停语义后再发 3/6。
    if (teleop.topology == Msg::TOPOLOGY_FULL_BODY && !teleop.yb_pause_semantics) {
        {
            std::lock_guard<std::mutex> lock(data_->mutex);
            if (data_->awaiting_yb_pause) return;
            data_->pending_arm_event = code;
            data_->awaiting_yb_pause = true;
        }
        publishEvent(16);
        return;
    }
    publishEvent(code);
}
void TeleopModeDisplay::flushPendingArmEvent() {
    int32_t code = 0;
    {
        std::lock_guard<std::mutex> lock(data_->mutex);
        if (!data_->awaiting_yb_pause || data_->pending_arm_event == 0) return;
        if (!data_->teleop.yb_pause_semantics) return;
        code = data_->pending_arm_event;
        data_->pending_arm_event = 0;
        data_->awaiting_yb_pause = false;
    }
    publishEvent(code);
}
int32_t TeleopModeDisplay::eventCodeForHit(TeleopModeDashboard::Hit hit) const {
    using Hit = TeleopModeDashboard::Hit;
    switch (hit) {
        case Hit::Teleop: return 4;
        case Hit::Mirror: return 7;
        case Hit::LeftArm: return 3;
        case Hit::RightArm: return 6;
        case Hit::NoHit: return 0;
    }
    return 0;
}
void TeleopModeDisplay::onRenderWindowMouse(QMouseEvent* event) {
    const int last_x = mouse_last_x_;
    const int last_y = mouse_last_y_;
    const QPoint pos = mouseLocalPos(event);
    mouse_last_x_ = pos.x();
    mouse_last_y_ = pos.y();

    if (isEnabled() && dashboard_ &&
        (event->type() == QEvent::MouseButtonPress ||
         event->type() == QEvent::MouseButtonRelease) &&
        event->button() == Qt::LeftButton) {
        Sample teleop;
        {
            std::lock_guard<std::mutex> lock(data_->mutex);
            teleop = data_->teleop;
        }
        if (teleop.valid && teleop.enabled) {
            const auto hit = dashboard_->hitTest(
                static_cast<float>(pos.x()), static_cast<float>(pos.y()));
            const int32_t code = eventCodeForHit(hit);
            if (code != 0) {
                if (event->type() == QEvent::MouseButtonRelease) {
                    if (hit == TeleopModeDashboard::Hit::LeftArm ||
                        hit == TeleopModeDashboard::Hit::RightArm) {
                        publishArmToggle(code, teleop);
                    } else {
                        publishEvent(code);
                    }
                }
                event->accept();
                return;
            }
        }
    }
    forwardMouseToRviz(event, last_x, last_y);
}
void TeleopModeDisplay::updateScale() {
    if (dashboard_) dashboard_->setScale(scale_property_->getFloat());
}
void TeleopModeDisplay::onEnable() {
    elapsed_ = 1;
    installMouseHook();
}
void TeleopModeDisplay::onDisable() {
    if (dashboard_) dashboard_->hide();
}
void TeleopModeDisplay::reset() {
    Display::reset();
    std::lock_guard<std::mutex> lock(data_->mutex);
    data_->teleop = {};
    data_->pending_arm_event = 0;
    data_->awaiting_yb_pause = false;
    elapsed_ = 1;
}
void TeleopModeDisplay::update(float wall_dt, float) {
    if (!isEnabled()) return;
    if (!mouse_hook_installed_) installMouseHook();
    flushPendingArmEvent();
    Sample teleop;
    {
        std::lock_guard<std::mutex> lock(data_->mutex);
        teleop = data_->teleop;
    }
    if (!teleop.valid || !teleop.enabled) {
        if (dashboard_) dashboard_->hide();
        return;
    }
    if (!dashboard_) {
        dashboard_ = std::make_unique<TeleopModeDashboard>();
        dashboard_->setScale(scale_property_->getFloat());
        dashboard_->show();
        elapsed_ = 0;
        return;
    }
    dashboard_->show();
    elapsed_ += wall_dt;
    if (elapsed_ < 0.1f) return;
    elapsed_ = 0;
    dashboard_->refreshLayout();
    dashboard_->setMode(
        teleop.update_mode, teleop.chassis_mode, teleop.body_tracking, teleop.mirror_movement,
        teleop.left_arm_paused, teleop.right_arm_paused,
        teleop.left_gripper_percent_mode, teleop.right_gripper_percent_mode,
        teleop.left_stick_plane, teleop.right_stick_plane,
        teleop.body_stick_plane, teleop.body_right_stick_plane,
        teleop.dex_hand_session);
}
}
PLUGINLIB_EXPORT_CLASS(arms_rviz_control_plugin::TeleopModeDisplay, rviz_common::Display)
