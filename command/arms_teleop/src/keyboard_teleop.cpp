//
// Created by biao on 24-9-11.
//

#include "arms_teleop/keyboard_teleop.h"

KeyboardTeleop::KeyboardTeleop() : Node("keyboard_teleop_node") {
    publisher_ = create_publisher<arms_ros2_control_msgs::msg::Inputs>("control_input", 10);
    gripper_publisher_ = create_publisher<arms_ros2_control_msgs::msg::Gripper>("/gripper_command", 10);
    timer_ = create_wall_timer(std::chrono::microseconds(100), std::bind(&KeyboardTeleop::timer_callback, this));
    
    // Initialize state
    currentTarget_ = 1; // Start with left arm
    
    // Initialize inputs message
    inputs_.command = 0;
    inputs_.x = 0.0;
    inputs_.y = 0.0;
    inputs_.z = 0.0;
    inputs_.roll = 0.0;
    inputs_.pitch = 0.0;
    inputs_.yaw = 0.0;
    inputs_.target = currentTarget_;

    tcgetattr(STDIN_FILENO, &old_tio_);
    new_tio_ = old_tio_;
    new_tio_.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio_);
    RCLCPP_INFO(get_logger(), "⌨️ Keyboard input node started.");
    RCLCPP_INFO(get_logger(), "⌨️ Controls: t=switch target arm, 1-0=commands, SPACE=gripper");
    RCLCPP_INFO(get_logger(), "⌨️ Movement: WASD=xy, IJKL=z+yaw, QE=roll, RF=pitch");
    RCLCPP_INFO(get_logger(), "Please input keys, press Ctrl+C to quit.");
}

void KeyboardTeleop::timer_callback() {
    if (kbhit()) {
        char key = getchar();
        check_command(key);
        
        // Process movement if no command is active
        if (inputs_.command == 0) {
            check_value(key);
        } else {
            // Clear movement when command is active
            inputs_.x = 0;
            inputs_.y = 0;
            inputs_.z = 0;
            inputs_.roll = 0;
            inputs_.pitch = 0;
            inputs_.yaw = 0;
            reset_count_ = 100;
        }
        
        // Always publish when there's input
        publisher_->publish(inputs_);
        just_published_ = true;
    } else {
        if (just_published_) {
            reset_count_ -= 1;
            if (reset_count_ == 0) {
                just_published_ = false;
                if (inputs_.command != 0) {
                    inputs_.command = 0;
                    publisher_->publish(inputs_);
                }
            }
        }
    }
}

void KeyboardTeleop::check_command(const char key) {
    switch (key) {
        case 't':
        case 'T':
            // Switch target arm
            currentTarget_ = (currentTarget_ == 1) ? 2 : 1;
            inputs_.target = currentTarget_;
            RCLCPP_INFO(get_logger(), "⌨️ Switched target arm to: %s", 
                        (currentTarget_ == 1) ? "LEFT" : "RIGHT");
            break;
        case '1':
            inputs_.command = 1; // L2_B
            break;
        case '2':
            inputs_.command = 2; // L2_A
            break;
        case '3':
            inputs_.command = 3; // L2_X
            break;
        case '4':
            inputs_.command = 4; // L2_Y
            break;
        case '5':
            inputs_.command = 5; // L1_A
            break;
        case '6':
            inputs_.command = 6; // L1_B
            break;
        case '7':
            inputs_.command = 7; // L1_X
            break;
        case '8':
            inputs_.command = 8; // L1_Y
            break;
        case '9':
            inputs_.command = 9;
            break;
        case '0':
            inputs_.command = 10;
            break;
        case ' ':
            // Toggle gripper open/close
            static bool gripper_open = false;
            gripper_open = !gripper_open;
            
            sendGripperCommand(gripper_open);
            
            if (gripper_open) {
                RCLCPP_INFO(this->get_logger(), "⌨️ Gripper opened");
            } else {
                RCLCPP_INFO(this->get_logger(), "⌨️ Gripper closed");
            }
            break;
        default:
            inputs_.command = 0;
            break;
    }
}

void KeyboardTeleop::check_value(char key) {
    switch (key) {
        // Position control (x, y)
        case 'w':
        case 'W':
            inputs_.x = min<float>(inputs_.x + sensitivity_left_, 1.0);
            break;
        case 's':
        case 'S':
            inputs_.x = max<float>(inputs_.x - sensitivity_left_, -1.0);
            break;
        case 'd':
        case 'D':
            inputs_.y = min<float>(inputs_.y + sensitivity_left_, 1.0);
            break;
        case 'a':
        case 'A':
            inputs_.y = max<float>(inputs_.y - sensitivity_left_, -1.0);
            break;

        // Position control (z) and rotation (yaw)
        case 'i':
        case 'I':
            inputs_.z = min<float>(inputs_.z + sensitivity_right_, 1.0);
            break;
        case 'k':
        case 'K':
            inputs_.z = max<float>(inputs_.z - sensitivity_right_, -1.0);
            break;
        case 'l':
        case 'L':
            inputs_.yaw = min<float>(inputs_.yaw + sensitivity_right_, 1.0);
            break;
        case 'j':
        case 'J':
            inputs_.yaw = max<float>(inputs_.yaw - sensitivity_right_, -1.0);
            break;
            
        // Rotation control (roll)
        case 'q':
        case 'Q':
            inputs_.roll = max<float>(inputs_.roll - sensitivity_right_, -1.0);
            break;
        case 'e':
        case 'E':
            inputs_.roll = min<float>(inputs_.roll + sensitivity_right_, 1.0);
            break;
            
        // Rotation control (pitch)
        case 'r':
        case 'R':
            inputs_.pitch = min<float>(inputs_.pitch + sensitivity_right_, 1.0);
            break;
        case 'f':
        case 'F':
            inputs_.pitch = max<float>(inputs_.pitch - sensitivity_right_, -1.0);
            break;
        default:
            break;
    }
}

bool KeyboardTeleop::kbhit() {
    timeval tv = {0L, 0L};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
}

void KeyboardTeleop::sendGripperCommand(bool open)
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
    RCLCPP_DEBUG(get_logger(), "Sent gripper command: target=%d, direction=%d", 
                  gripper_msg.target, gripper_msg.direction);
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardTeleop>();
    spin(node);
    rclcpp::shutdown();
    return 0;
}
