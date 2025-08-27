//
// Created by tlab-uav on 24-9-13.
//

#include "arms_teleop/joystick_teleop.h"

using std::placeholders::_1;

JoystickTeleop::JoystickTeleop() : Node("joystick_teleop_node") {
    publisher_ = create_publisher<arms_ros2_control_msgs::msg::Inputs>("control_input", 10);
    gripper_publisher_ = create_publisher<arms_ros2_control_msgs::msg::Gripper>("/gripper_command", 10);
    subscription_ = create_subscription<
        sensor_msgs::msg::Joy>("joy", 10, std::bind(&JoystickTeleop::joy_callback, this, _1));
    
    // Initialize gripper control
    last_x_pressed_ = false;
}

void JoystickTeleop::joy_callback(sensor_msgs::msg::Joy::SharedPtr msg) {
    // Check if X button is pressed
    bool x_pressed = msg->buttons[2];
    
    // Original command logic
    if (msg->buttons[1] && msg->buttons[4]) {
        inputs_.command = 1; // LB + B
    } else if (msg->buttons[0] && msg->buttons[4]) {
        inputs_.command = 2; // LB + A
    } else if (msg->buttons[7]) {
        inputs_.command = 3; // START
    } else if (msg->buttons[3] && msg->buttons[4]) {
        inputs_.command = 4; // LB + Y
    } else {
        inputs_.command = 0;
        inputs_.lx = -msg->axes[0];
        inputs_.ly = msg->axes[1];
        inputs_.rx = -msg->axes[3];
        inputs_.ry = msg->axes[4];
        
        // If no command is set and X is pressed alone, toggle gripper
        if (x_pressed && !last_x_pressed_) {
            // Send gripper command (alternate between open and close)
            static bool gripper_open = false;
            gripper_open = !gripper_open;
            
            sendGripperCommand(gripper_open);
            
            if (gripper_open) {
                RCLCPP_INFO(this->get_logger(), "Gripper opened");
            } else {
                RCLCPP_INFO(this->get_logger(), "Gripper closed");
            }
        }
    }
    
    // Update last X button state
    last_x_pressed_ = x_pressed;
    
    publisher_->publish(inputs_);
}

void JoystickTeleop::sendGripperCommand(bool open)
{
    auto gripper_msg = arms_ros2_control_msgs::msg::Gripper();
    
    if (open) {
        gripper_msg.target = 1;  // 打开夹爪
        gripper_msg.direction = 1;
    } else {
        gripper_msg.target = 0;  // 关闭夹爪
        gripper_msg.direction = -1;
    }
    
    gripper_publisher_->publish(gripper_msg);
    RCLCPP_DEBUG(this->get_logger(), "Sent gripper command: target=%d, direction=%d", 
                  gripper_msg.target, gripper_msg.direction);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickTeleop>();
    spin(node);
    rclcpp::shutdown();
    return 0;
}
