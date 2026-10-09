#pragma once
#include <rviz_common/display.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <arms_ros2_control_msgs/msg/wbc_current_state.hpp>
#include <memory>
#include <array>
#include <chrono>
#include <mutex>
#include "arms_rviz_control_plugin/error_dashboard.hpp"

namespace arms_rviz_control_plugin {
class TargetErrorDisplay : public rviz_common::Display {
    Q_OBJECT
public:
    TargetErrorDisplay() = default;
    ~TargetErrorDisplay() override;
    void update(float wall_dt, float ros_dt) override;
    void reset() override;
    void load(const rviz_common::Config& config) override;
    void save(rviz_common::Config config) const override;
protected:
    void onInitialize() override;
    void onEnable() override;
    void onDisable() override;
private:
    struct Sample {
        geometry_msgs::msg::PoseStamped pose{};
        std::chrono::steady_clock::time_point received{};
        bool valid{false};
    };
    struct Data {
        std::mutex mutex;
        std::array<std::array<Sample, 2>, 4> samples{};
        std::array<bool, 4> active{};
        bool ocs2_active{false};
        uint64_t fsm_generation{0};
    };
    std::shared_ptr<Data> data_{std::make_shared<Data>()};
    std::array<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr, 8> subscriptions_;
    rclcpp::Subscription<arms_ros2_control_msgs::msg::WbcCurrentState>::SharedPtr mode_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr fsm_subscription_;
    std::unique_ptr<ErrorDashboard> dashboard_;
    uint64_t fsm_generation_{0};
    std::array<double, 4> thresholds_{5, 20, 1, 5};
    float elapsed_{0};
};
}
