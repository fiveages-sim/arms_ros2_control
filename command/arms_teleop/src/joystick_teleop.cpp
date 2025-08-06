//
// Created by tlab-uav on 24-9-13.
//

#include "arms_teleop/joystick_teleop.h"

using std::placeholders::_1;

JoystickTeleop::JoystickTeleop() : Node("joystick_teleop_node") {
    publisher_ = create_publisher<arms_ros2_control_msgs::msg::Inputs>("control_input", 10);
    subscription_ = create_subscription<
        sensor_msgs::msg::Joy>("joy", 10, std::bind(&JoystickTeleop::joy_callback, this, _1));
}

void JoystickTeleop::joy_callback(sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->buttons[1] && msg->buttons[4]) {
        inputs_.command = 1; // LB + B
    } else if (msg->buttons[0] && msg->buttons[4]) {
        inputs_.command = 2; // LB + A
    } else if (msg->buttons[2] && msg->buttons[4]) {
        inputs_.command = 3; // LB + X
    } else if (msg->buttons[3] && msg->buttons[4]) {
        inputs_.command = 4; // LB + Y
    } else if (msg->axes[2] != 1 && msg->buttons[1]) {
        inputs_.command = 5; // LT + B
    } else if (msg->axes[2] != 1 && msg->buttons[0]) {
        inputs_.command = 6; // LT + A
    } else if (msg->axes[2] != 1 && msg->buttons[2]) {
        inputs_.command = 7; // LT + X
    } else if (msg->axes[2] != 1 && msg->buttons[3]) {
        inputs_.command = 8; // LT + Y
    } else if (msg->buttons[7]) {
        inputs_.command = 9; // START
    } else {
        inputs_.command = 0;
        inputs_.lx = -msg->axes[0];
        inputs_.ly = msg->axes[1];
        inputs_.rx = -msg->axes[3];
        inputs_.ry = msg->axes[4];
    }
    publisher_->publish(inputs_);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickTeleop>();
    spin(node);
    rclcpp::shutdown();
    return 0;
}
