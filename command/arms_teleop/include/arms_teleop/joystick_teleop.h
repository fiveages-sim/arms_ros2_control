//
// Created by tlab-uav on 24-9-13.
//

#ifndef JOYSTICK_TELEOP_H
#define JOYSTICK_TELEOP_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <arms_ros2_control_msgs/msg/inputs.hpp>
#include <arms_ros2_control_msgs/msg/gripper.hpp>

class JoystickTeleop final : public rclcpp::Node {
public:
    JoystickTeleop();

    ~JoystickTeleop() override = default;

private:
    void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg);
    void processButtons(const sensor_msgs::msg::Joy::SharedPtr msg);
    void processAxes(const sensor_msgs::msg::Joy::SharedPtr msg);
    void sendGripperCommand(bool open);
    void gripper_command_callback(arms_ros2_control_msgs::msg::Gripper::SharedPtr msg);
    double applyDeadzone(double value, double deadzone = 0.1) const;

    arms_ros2_control_msgs::msg::Inputs inputs_;
    rclcpp::Publisher<arms_ros2_control_msgs::msg::Inputs>::SharedPtr publisher_;
    rclcpp::Publisher<arms_ros2_control_msgs::msg::Gripper>::SharedPtr gripper_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Subscription<arms_ros2_control_msgs::msg::Gripper>::SharedPtr gripper_command_subscription_;
    
    // Control parameters
    double updateRate_;
    
    // State management
    bool enabled_;
    rclcpp::Time lastUpdateTime_;
    
    // Button state tracking for edge detection
    bool last_x_pressed_;
    bool last_a_pressed_;
    bool last_b_pressed_;
    bool last_y_pressed_;
    bool last_lb_pressed_;
    bool last_rb_pressed_;
    bool last_back_pressed_;
    bool last_start_pressed_;
    bool last_left_stick_pressed_;
    bool last_right_stick_pressed_;
    
    // Target arm selection (1=left, 2=right)
    int32_t currentTarget_;

    // Gripper command state tracking (for synchronization with panel)
    int32_t current_gripper_target_;  // 0=close, 1=open
    bool gripper_command_received_;

    // Separate gripper states for left and right arms (like task3)
    bool left_gripper_open_;   // Left arm gripper state
    bool right_gripper_open_;  // Right arm gripper state
};

#endif //JOYSTICK_TELEOP_H 