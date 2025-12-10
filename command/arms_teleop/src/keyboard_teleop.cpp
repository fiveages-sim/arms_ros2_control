//
// Created by biao on 24-9-11.
//

#include "arms_teleop/keyboard_teleop.h"

KeyboardTeleop::KeyboardTeleop() : Node("keyboard_teleop_node") {
    publisher_ = create_publisher<arms_ros2_control_msgs::msg::Inputs>("control_input", 10);
    fsm_command_publisher_ = create_publisher<std_msgs::msg::Int32>("/fsm_command", 10);
    gripper_publisher_ = create_publisher<arms_ros2_control_msgs::msg::Gripper>("/gripper_command", 10);
    timer_ = create_wall_timer(std::chrono::microseconds(100), std::bind(&KeyboardTeleop::timer_callback, this));
    
    // Initialize state
    currentTarget_ = 1; // Start with left arm
    
    // Initialize inputs message (command field is no longer used, only for incremental control)
    inputs_.command = 0;  // Always 0, FSM commands go to /fsm_command topic
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
        
        // Check for FSM command first
        int32_t fsm_command = 0;
        switch (key) {
            case '1':
                fsm_command = 1; // HOME
                break;
            case '2':
                fsm_command = 2; // HOLD
                break;
            case '3':
                fsm_command = 3; // OCS2/MOVE
                break;
            case '4':
                fsm_command = 4; // MOVEJ
                break;
            case '5':
                fsm_command = 5;
                break;
            case '6':
                fsm_command = 6;
                break;
            case '7':
                fsm_command = 7;
                break;
            case '8':
                fsm_command = 8;
                break;
            case '9':
                fsm_command = 9;
                break;
            case '0':
                fsm_command = 10;
                break;
            default:
                break;
        }
        
        // Publish FSM command if there's a command
        if (fsm_command != 0) {
            auto fsm_cmd = std_msgs::msg::Int32();
            fsm_cmd.data = fsm_command;
            fsm_command_publisher_->publish(fsm_cmd);
            reset_count_ = 100;
        } else {
            // Process other keys (movement, target arm switch, gripper)
            check_command(key);
            
            // Process movement for incremental control
            check_value(key);
            
            // Publish incremental control to /control_input (command is always 0)
            inputs_.command = 0;
            publisher_->publish(inputs_);
        }
        
        just_published_ = true;
    } else {
        if (just_published_) {
            reset_count_ -= 1;
            if (reset_count_ == 0) {
                just_published_ = false;
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
            // FSM commands (1-0) are handled in timer_callback, not here
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
