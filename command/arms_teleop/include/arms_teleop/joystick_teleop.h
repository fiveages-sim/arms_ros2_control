//
// Created by tlab-uav on 24-9-13.
//

#ifndef JOYSTICK_TELEOP_H
#define JOYSTICK_TELEOP_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <arms_ros2_control_msgs/msg/inputs.hpp>
#include <control_msgs/action/parallel_gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class JoystickTeleop final : public rclcpp::Node {
public:
    JoystickTeleop();

    ~JoystickTeleop() override = default;

private:
    void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg);
    bool sendGripperCommand(double position);

    arms_ros2_control_msgs::msg::Inputs inputs_;
    rclcpp::Publisher<arms_ros2_control_msgs::msg::Inputs>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    
    // Action client for gripper control
    rclcpp_action::Client<control_msgs::action::ParallelGripperCommand>::SharedPtr gripper_action_client_;
    bool gripper_open_;
    bool last_x_pressed_;
    double gripper_open_position_;
    double gripper_closed_position_;
    double gripper_max_effort_;
};

#endif //JOYSTICK_TELEOP_H 