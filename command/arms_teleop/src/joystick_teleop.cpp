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
    
    // Initialize control parameters
    updateRate_ = 20.0;
    
    // Initialize state
    enabled_ = false;
    lastUpdateTime_ = now();
    currentTarget_ = 1; // Start with left arm
    
    // Initialize button states
    last_x_pressed_ = false;
    last_a_pressed_ = false;
    last_b_pressed_ = false;
    last_y_pressed_ = false;
    last_lb_pressed_ = false;
    last_rb_pressed_ = false;
    last_back_pressed_ = false;
    last_start_pressed_ = false;
    last_left_stick_pressed_ = false;
    last_right_stick_pressed_ = false;
    
    // Initialize inputs message
    inputs_.command = 0;
    inputs_.x = 0.0;
    inputs_.y = 0.0;
    inputs_.z = 0.0;
    inputs_.roll = 0.0;
    inputs_.pitch = 0.0;
    inputs_.yaw = 0.0;
    inputs_.target = currentTarget_;
    
    RCLCPP_INFO(get_logger(), "🎮 JoystickTeleop created");
    RCLCPP_INFO(get_logger(), "🎮 Joystick control is DISABLED by default. Press right stick to enable.");
    RCLCPP_INFO(get_logger(), "🎮 Controls: Right stick=toggle control, A=switch target arm, B=send command, X=toggle gripper");
}

void JoystickTeleop::joy_callback(sensor_msgs::msg::Joy::SharedPtr msg) {
    // Check update frequency
    auto currentTime = now();
    double timeSinceLastUpdate = (currentTime - lastUpdateTime_).seconds();
    double updateInterval = 1.0 / updateRate_;

    if (timeSinceLastUpdate < updateInterval) {
        return;
    }
    lastUpdateTime_ = currentTime;

    // Process buttons first (always process for enable/disable toggle)
    processButtons(msg);

    // Only process other logic if joystick is enabled
    if (enabled_) {
        // Original command logic - keep existing behavior
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
            
            // Process axes if no command is active
            processAxes(msg);
        }
        
        // Publish the current state only when enabled
        publisher_->publish(inputs_);
    }
}

void JoystickTeleop::processButtons(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->buttons.size() <= 10) {
        return;
    }

    // Check button states
    bool x_pressed = msg->buttons[2];
    bool a_pressed = msg->buttons[0];
    bool b_pressed = msg->buttons[1];
    bool y_pressed = msg->buttons[3];
    bool lb_pressed = msg->buttons[4];
    bool rb_pressed = msg->buttons[5];
    bool back_pressed = msg->buttons[6];
    bool start_pressed = msg->buttons[7];
    bool left_stick_pressed = msg->buttons[9];
    bool right_stick_pressed = msg->buttons[10];

    // Detect button press events (rising edge)
    bool right_stick_just_pressed = right_stick_pressed && !last_right_stick_pressed_;
    bool a_just_pressed = a_pressed && !last_a_pressed_;
    bool b_just_pressed = b_pressed && !last_b_pressed_;
    bool x_just_pressed = x_pressed && !last_x_pressed_;

    // Update last button states
    last_x_pressed_ = x_pressed;
    last_a_pressed_ = a_pressed;
    last_b_pressed_ = b_pressed;
    last_y_pressed_ = y_pressed;
    last_lb_pressed_ = lb_pressed;
    last_rb_pressed_ = rb_pressed;
    last_back_pressed_ = back_pressed;
    last_start_pressed_ = start_pressed;
    last_left_stick_pressed_ = left_stick_pressed;
    last_right_stick_pressed_ = right_stick_pressed;

    // Process button events
    if (right_stick_just_pressed) {
        // Right stick press: toggle control
        enabled_ = !enabled_;
        if (enabled_) {
            RCLCPP_INFO(get_logger(), "🎮 Joystick control ENABLED!");
        } else {
            RCLCPP_INFO(get_logger(), "🎮 Joystick control DISABLED!");
        }
    }

    if (a_just_pressed && enabled_) {
        // A button: switch target arm
        currentTarget_ = (currentTarget_ == 1) ? 2 : 1;
        inputs_.target = currentTarget_;
        RCLCPP_INFO(get_logger(), "🎮 Switched target arm to: %s", 
                    (currentTarget_ == 1) ? "LEFT" : "RIGHT");
    }

    if (x_just_pressed && enabled_) {
        // X button: toggle gripper
        static bool gripper_open = false;
        gripper_open = !gripper_open;
        sendGripperCommand(gripper_open);
        
        if (gripper_open) {
            RCLCPP_INFO(get_logger(), "🎮 Gripper opened");
        } else {
            RCLCPP_INFO(get_logger(), "🎮 Gripper closed");
        }
    }
}

void JoystickTeleop::processAxes(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (msg->axes.size() <= 7) {
        return;
    }

    // Left stick controls position (x, y)
    double left_stick_x = applyDeadzone(msg->axes[0]);
    double left_stick_y = applyDeadzone(msg->axes[1]);

    // Right stick controls position (z) and rotation (yaw)
    double right_stick_x = applyDeadzone(msg->axes[3]);
    double right_stick_y = applyDeadzone(msg->axes[4]);

    // D-pad controls roll and pitch rotation
    double dpad_x = 0.0;
    double dpad_y = 0.0;
    if (msg->axes.size() > 6 && msg->axes.size() > 7) {
        dpad_x = applyDeadzone(msg->axes[6], 0.5);
        dpad_y = applyDeadzone(msg->axes[7], 0.5);
    }

    // Update position (x, y, z)
    inputs_.x = left_stick_y;   // Left stick Y: forward/backward
    inputs_.y = left_stick_x;   // Left stick X: left/right
    inputs_.z = right_stick_y;  // Right stick Y: up/down

    // Update rotation (roll, pitch, yaw)
    inputs_.roll = dpad_x;         // D-pad X: roll
    inputs_.pitch = -dpad_y;       // D-pad Y: pitch (inverted)
    inputs_.yaw = right_stick_x;   // Right stick X: yaw
}

double JoystickTeleop::applyDeadzone(double value, double deadzone) const {
    if (std::abs(value) < deadzone) {
        return 0.0;
    }
    return value;
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
