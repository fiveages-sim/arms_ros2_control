//
// Keyboard teleop: publishes same topics as joystick_teleop (except /fsm_command).
//

#ifndef ARMS_TELEOP_KEYBOARD_TELEOP_H
#define ARMS_TELEOP_KEYBOARD_TELEOP_H

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <termios.h>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <arms_ros2_control_msgs/msg/inputs.hpp>

class KeyboardTeleop final : public rclcpp::Node {
public:
    KeyboardTeleop();
    ~KeyboardTeleop() override;

    KeyboardTeleop(const KeyboardTeleop &) = delete;
    KeyboardTeleop &operator=(const KeyboardTeleop &) = delete;

private:
    enum class ControlMode : std::uint8_t { Arm = 0, Chassis = 1 };

    void timerCallback();
    void stdinThreadFunc();

    bool isTtyOk() const;
    void restoreTerminal();
    void publishWaistZero();

    double speedMultiplier() const;

    void refreshStatusLine();

    bool isActive(std::uint16_t code, std::chrono::steady_clock::time_point now,
                  std::chrono::milliseconds stale) const;

    rclcpp::Publisher<arms_ros2_control_msgs::msg::Inputs>::SharedPtr inputs_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr chassis_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr waist_lift_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr waist_turn_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::thread stdin_thread_;
    std::atomic<bool> stdin_running_{true};

    mutable std::mutex key_mutex_;
    std::unordered_map<std::uint16_t, std::chrono::steady_clock::time_point> last_key_time_;
    std::chrono::milliseconds movement_key_stale_ms_{120};
    std::chrono::milliseconds discrete_key_stale_ms_{320};

    struct termios orig_termios_{};
    bool tty_saved_{false};

    double update_hz_{20.0};
    double chassis_linear_scale_{0.25};
    double chassis_angular_scale_{0.5};
    double arm_activation_threshold_{0.5};
    bool show_status_line_{true};

    int speed_level_{3};
    bool mirror_movement_{false};
    ControlMode mode_{ControlMode::Arm};

    std::int32_t target_arm_{1};
    bool left_gripper_open_{false};
    bool right_gripper_open_{false};

    arms_ros2_control_msgs::msg::Inputs inputs_{};
    geometry_msgs::msg::Twist chassis_cmd_{};

    bool prev_active_n_{false};
    bool prev_active_m_{false};
    bool prev_active_space_{false};
    bool prev_active_enter_{false};
    bool prev_active_minus_{false};
    bool prev_active_plus_{false};
};

#endif
