//
// Created by biao on 24-9-11.
//

#ifndef KEYBOARD_TELEOP_H
#define KEYBOARD_TELEOP_H

#include <rclcpp/rclcpp.hpp>
#include <arms_ros2_control_msgs/msg/inputs.hpp>
#include <arms_ros2_control_msgs/msg/gripper.hpp>
#include <std_msgs/msg/int32.hpp>
#include <termios.h>

template <typename T1, typename T2>
T1 max(const T1 a, const T2 b) {
    return (a > b ? a : b);
}

template <typename T1, typename T2>
T1 min(const T1 a, const T2 b) {
    return (a < b ? a : b);
}

class KeyboardTeleop final : public rclcpp::Node {
public:
    KeyboardTeleop();

    ~KeyboardTeleop() override {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
    }

private:
    void timer_callback();

    void check_command(char key);
    void check_value(char key);
    void sendGripperCommand(bool open);

    static bool kbhit();

    arms_ros2_control_msgs::msg::Inputs inputs_;
    rclcpp::Publisher<arms_ros2_control_msgs::msg::Inputs>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr fsm_command_publisher_;
    rclcpp::Publisher<arms_ros2_control_msgs::msg::Gripper>::SharedPtr gripper_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State management
    int32_t currentTarget_;

    bool just_published_ = false;
    int reset_count_ = 0;

    float sensitivity_left_ = 0.05;
    float sensitivity_right_ = 0.05;
    termios old_tio_{}, new_tio_{};
};

#endif //KEYBOARD_TELEOP_H 