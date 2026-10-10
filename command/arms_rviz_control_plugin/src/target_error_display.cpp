#include "arms_rviz_control_plugin/target_error_display.hpp"
#include <rviz_common/display_context.hpp>
#include <rviz_rendering/render_system.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <pluginlib/class_list_macros.hpp>

namespace arms_rviz_control_plugin {
TargetErrorDisplay::TargetErrorDisplay() {
    scale_property_ = new rviz_common::properties::FloatProperty(
        "Scale", 1.0f, "Size of the tracking-error overlay.", this, SLOT(updateScale()));
    scale_property_->setMin(0.5f);
    scale_property_->setMax(2.0f);
}
TargetErrorDisplay::~TargetErrorDisplay() {
    fsm_subscription_.reset();
    mode_subscription_.reset();
    for (auto& subscription : subscriptions_) subscription.reset();
    dashboard_.reset();
}
void TargetErrorDisplay::onInitialize() {
    rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
    auto node = context_->getRosNodeAbstraction().lock()->get_raw_node();
    fsm_subscription_ = node->create_subscription<std_msgs::msg::Int32>(
        "/fsm_state", rclcpp::QoS(1).transient_local(),
        [data = data_](std_msgs::msg::Int32::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(data->mutex);
            // Actual controller state: HOME=1, HOLD=2, OCS2=3, MOVEJ=4.
            const bool active = message->data == 3;
            if (active != data->ocs2_active) {
                data->ocs2_active = active;
                ++data->fsm_generation;
                // Keep cached targets, which may only be published on change.
                for (auto& row : data->samples) row[0].valid = false;
            }
        });
    const char* topics[4][2] = {
        {"left_current_pose", "left_current_target"},
        {"right_current_pose", "right_current_target"},
        {"body_current_pose", "body_current_target"},
        {"head_current_pose", "head_current_target"}};
    for (size_t row = 0; row < 4; ++row) {
        for (size_t side = 0; side < 2; ++side) {
            subscriptions_[row*2+side] = node->create_subscription<geometry_msgs::msg::PoseStamped>(
                topics[row][side], rclcpp::SensorDataQoS().keep_last(1),
                [data = data_, row, side](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
                    std::lock_guard<std::mutex> lock(data->mutex);
                    if (side == 0 && !data->ocs2_active) return;
                    Sample sample{*message, std::chrono::steady_clock::now(), true};
                    data->samples[row][side] = std::move(sample);
                });
        }
    }
    mode_subscription_ = node->create_subscription<arms_ros2_control_msgs::msg::WbcCurrentState>(
        "/ocs2_wbc_controller/current_state", rclcpp::QoS(1).transient_local(),
        [data = data_](arms_ros2_control_msgs::msg::WbcCurrentState::ConstSharedPtr message) {
            std::lock_guard<std::mutex> lock(data->mutex);
            using State = arms_ros2_control_msgs::msg::WbcCurrentState;
            data->wbc_state_received = true;
            data->active = {message->left_arm_state == State::ARM_ENABLED,
                            message->right_arm_state == State::ARM_ENABLED,
                            message->body_state == State::BODY_TRACKING && message->head_state != State::HEAD_FORWARD,
                            message->head_state == State::HEAD_TRACKING};
        });
}
void TargetErrorDisplay::updateScale() {
    if (dashboard_) dashboard_->setScale(scale_property_->getFloat());
}
void TargetErrorDisplay::onEnable() { elapsed_ = 1; }
void TargetErrorDisplay::onDisable() { if (dashboard_) dashboard_->hide(); }
void TargetErrorDisplay::reset() {
    Display::reset();
    if (dashboard_) dashboard_->clearHistory();
    std::lock_guard<std::mutex> lock(data_->mutex);
    data_->samples = {};
    elapsed_ = 1;
}
void TargetErrorDisplay::update(float wall_dt, float) {
    if (!isEnabled()) return;
    bool ocs2_active;
    uint64_t generation;
    {
        std::lock_guard<std::mutex> lock(data_->mutex);
        ocs2_active = data_->ocs2_active;
        generation = data_->fsm_generation;
    }
    if (generation != fsm_generation_) {
        if (dashboard_) dashboard_->clearHistory();
        fsm_generation_ = generation;
        elapsed_ = 1;
    }
    if (!ocs2_active) {
        if (dashboard_) dashboard_->hide();
        return;
    }
    if (!dashboard_) {
        dashboard_ = std::make_unique<ErrorDashboard>();
        dashboard_->setScale(scale_property_->getFloat());
        dashboard_->show();
        elapsed_ = 0;
        return;  // Allow the initial text geometry to render before the layout refresh.
    }
    dashboard_->show();
    elapsed_ += wall_dt;
    if (elapsed_ < 0.1f) return;
    elapsed_ = 0;
    dashboard_->refreshLayout();
    std::array<std::array<Sample, 2>, 4> samples;
    std::array<bool, 4> active;
    {
        std::lock_guard<std::mutex> lock(data_->mutex);
        if (!data_->ocs2_active || data_->fsm_generation != generation) {
            dashboard_->hide();
            return;
        }
        samples = data_->samples;
        if (data_->wbc_state_received) {
            active = data_->active;
        } else {
            // Split / dual / single-arm ocs2_arm_controller: show ends that publish a target.
            active = {true, samples[1][1].valid, samples[2][1].valid, samples[3][1].valid};
        }
    }
    dashboard_->setActive(active);
    dashboard_->setArmTitle(active[1]);
    const auto now = std::chrono::steady_clock::now();
    for (size_t row = 0; row < 4; ++row) {
        if (!active[row]) continue;
        const auto& measured = samples[row][0];
        const auto& target = samples[row][1];
        bool valid = false;
        double mm = 0, degrees = 0;
        QString status;
        if (!measured.valid || !target.valid) status = "等待当前 / 目标位姿";
        else if (now-measured.received > std::chrono::seconds(1)) status = "当前位姿已超时";
        else if (measured.pose.header.frame_id.empty() ||
                 measured.pose.header.frame_id != target.pose.header.frame_id)
            status = "参考坐标系不一致";
        else {
            const auto& a = measured.pose.pose;
            const auto& b = target.pose.pose;
            mm = 1000*std::hypot(b.position.x-a.position.x,
                               std::hypot(b.position.y-a.position.y, b.position.z-a.position.z));
            tf2::Quaternion qa(a.orientation.x,a.orientation.y,a.orientation.z,a.orientation.w);
            tf2::Quaternion qb(b.orientation.x,b.orientation.y,b.orientation.z,b.orientation.w);
            if (!std::isfinite(mm) || !std::isfinite(qa.length2()) || !std::isfinite(qb.length2()) ||
                qa.length2() < 1e-18 || qb.length2() < 1e-18) status = "位姿数据无效";
            else {
                qa.normalize(); qb.normalize();
                degrees = 2*std::acos(std::clamp(std::abs(qa.dot(qb)),0.0,1.0))*180/std::acos(-1.0);
                valid = true;
                status = "数据有效";
            }
        }
        dashboard_->setRow(row, mm, degrees, valid, status, thresholds_);
    }
}
void TargetErrorDisplay::load(const rviz_common::Config& config) {
    Display::load(config);
    const char* keys[] = {"ErrorPositionGreenMm", "ErrorPositionRedMm",
                          "ErrorOrientationGreenDeg", "ErrorOrientationRedDeg"};
    for (size_t i = 0; i < 4; i += 2) {
        float green, red;
        if (config.mapGetFloat(keys[i], &green) && config.mapGetFloat(keys[i+1], &red) &&
            std::isfinite(green) && std::isfinite(red) && green > 0 && red > green)
            { thresholds_[i] = green; thresholds_[i+1] = red; }
    }
}
void TargetErrorDisplay::save(rviz_common::Config config) const {
    Display::save(config);
    const char* keys[] = {"ErrorPositionGreenMm", "ErrorPositionRedMm",
                          "ErrorOrientationGreenDeg", "ErrorOrientationRedDeg"};
    for (size_t i = 0; i < 4; ++i) config.mapSetValue(keys[i], thresholds_[i]);
}
}
PLUGINLIB_EXPORT_CLASS(arms_rviz_control_plugin::TargetErrorDisplay, rviz_common::Display)
