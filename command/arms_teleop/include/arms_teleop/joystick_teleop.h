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
    void sendGripperCommand(bool open);

    arms_ros2_control_msgs::msg::Inputs inputs_;
    rclcpp::Publisher<arms_ros2_control_msgs::msg::Inputs>::SharedPtr publisher_;
    rclcpp::Publisher<arms_ros2_control_msgs::msg::Gripper>::SharedPtr gripper_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    
    // Gripper control
    bool last_x_pressed_;
};

#endif //JOYSTICK_TELEOP_H 