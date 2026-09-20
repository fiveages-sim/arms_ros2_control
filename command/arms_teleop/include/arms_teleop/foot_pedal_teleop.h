#ifndef ARMS_TELEOP_FOOT_PEDAL_TELEOP_H
#define ARMS_TELEOP_FOOT_PEDAL_TELEOP_H

#include <linux/input.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

class FootPedalTeleop final : public rclcpp::Node {
public:
    FootPedalTeleop();
    ~FootPedalTeleop() override;

    FootPedalTeleop(const FootPedalTeleop &) = delete;
    FootPedalTeleop & operator=(const FootPedalTeleop &) = delete;

private:
    struct MultiActionPedalState {
        bool pressed{false};
        bool long_press_fired{false};
        std::chrono::steady_clock::time_point pressed_at{};
    };

    void validateParameters() const;
    std::string resolveDevicePath() const;
    void openDevice();
    void closeDevice();
    std::int32_t eventForKey(std::uint16_t key) const;
    void publishEvent(std::int32_t xr_event);
    void updateLongPress(
        MultiActionPedalState & state,
        std::int64_t threshold_ms,
        std::int32_t long_press_event);
    void handleMultiActionPedal(
        const input_event & event,
        MultiActionPedalState & state,
        std::int32_t short_press_event,
        std::int32_t long_press_event,
        std::int64_t threshold_ms);
    void handleKeyEvent(const input_event & event);
    void publishIdleState();
    void readEvents();
    void handleDeviceFailure(const std::string & reason);

    std::string device_path_;
    std::string device_serial_;
    bool grab_device_{true};
    double publish_rate_hz_{30.0};
    std::string controller_state_topic_{"/xr/controller_state"};
    std::int64_t whole_teleop_long_press_ms_{800};
    std::int64_t mirror_long_press_ms_{800};
    int key_fsm_down_{KEY_F13};
    int key_fsm_up_{KEY_F14};
    int key_left_arm_toggle_{KEY_F15};
    int key_right_arm_toggle_{KEY_F16};
    MultiActionPedalState left_pedal_state_;
    MultiActionPedalState right_pedal_state_;
    int device_fd_{-1};
    bool grabbed_{false};
    bool device_failed_{false};
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr controller_state_publisher_;
    rclcpp::TimerBase::SharedPtr input_timer_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr shutdown_timer_;
};

#endif  // ARMS_TELEOP_FOOT_PEDAL_TELEOP_H
