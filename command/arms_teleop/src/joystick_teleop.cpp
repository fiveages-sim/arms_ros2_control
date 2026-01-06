//
// Created by tlab-uav on 24-9-13.
//

#include "arms_teleop/joystick_teleop.h"

using std::placeholders::_1;

JoystickTeleop::JoystickTeleop() : Node("joystick_teleop_node") {
    publisher_ = create_publisher<arms_ros2_control_msgs::msg::Inputs>("control_input", 10);
    fsm_command_publisher_ = create_publisher<std_msgs::msg::Int32>("/fsm_command", 10);
    left_target_command_publisher_ = create_publisher<std_msgs::msg::Int32>("/left_hand_controller/target_command", 10);
    right_target_command_publisher_ = create_publisher<std_msgs::msg::Int32>("/right_hand_controller/target_command", 10);
    subscription_ = create_subscription<
        sensor_msgs::msg::Joy>("joy", 10, std::bind(&JoystickTeleop::joy_callback, this, _1));
    left_target_command_subscription_ = create_subscription<std_msgs::msg::Int32>(
        "/left_hand_controller/target_command", 10,
        std::bind(&JoystickTeleop::left_target_command_callback, this, _1));
    right_target_command_subscription_ = create_subscription<std_msgs::msg::Int32>(
        "/right_hand_controller/target_command", 10,
        std::bind(&JoystickTeleop::right_target_command_callback, this, _1));
    
    // Load button and axes mapping from parameters
    loadButtonMapping();
    
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

    // Initialize separate gripper states for left and right arms
    left_gripper_open_ = false;   // Left arm gripper initially closed
    right_gripper_open_ = false;  // Right arm gripper initially closed
    
    // Initialize speed mode (start with low speed)
    high_speed_mode_ = false;
    
    // Load speed scaling parameters
    this->declare_parameter("speed.low_scale", 0.3);
    this->declare_parameter("speed.high_scale", 1.0);
    low_speed_scale_ = this->get_parameter("speed.low_scale").as_double();
    high_speed_scale_ = this->get_parameter("speed.high_scale").as_double();
    
    // Initialize inputs message (command field is no longer used, only for incremental control)
    inputs_.command = 0;  // Always 0, FSM commands go to /fsm_command topic
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
    RCLCPP_INFO(get_logger(), "🎮 Speed mode: Left stick button=toggle high/low speed (Current: %s)", 
                high_speed_mode_ ? "HIGH" : "LOW");
    
    // Print button mapping configuration
    printButtonMapping();
}

void JoystickTeleop::loadButtonMapping() {
    // Declare and get button mapping parameters with Xbox controller defaults
    this->declare_parameter("button_mapping.x_button", 2);
    this->declare_parameter("button_mapping.a_button", 0);
    this->declare_parameter("button_mapping.b_button", 1);
    this->declare_parameter("button_mapping.y_button", 3);
    this->declare_parameter("button_mapping.lb_button", 4);
    this->declare_parameter("button_mapping.rb_button", 5);
    this->declare_parameter("button_mapping.back_button", 6);
    this->declare_parameter("button_mapping.start_button", 7);
    this->declare_parameter("button_mapping.left_stick_button", 9);
    this->declare_parameter("button_mapping.right_stick_button", 10);
    
    button_map_.x_button = this->get_parameter("button_mapping.x_button").as_int();
    button_map_.a_button = this->get_parameter("button_mapping.a_button").as_int();
    button_map_.b_button = this->get_parameter("button_mapping.b_button").as_int();
    button_map_.y_button = this->get_parameter("button_mapping.y_button").as_int();
    button_map_.lb_button = this->get_parameter("button_mapping.lb_button").as_int();
    button_map_.rb_button = this->get_parameter("button_mapping.rb_button").as_int();
    button_map_.back_button = this->get_parameter("button_mapping.back_button").as_int();
    button_map_.start_button = this->get_parameter("button_mapping.start_button").as_int();
    button_map_.left_stick_button = this->get_parameter("button_mapping.left_stick_button").as_int();
    button_map_.right_stick_button = this->get_parameter("button_mapping.right_stick_button").as_int();
    
    // Declare and get axes mapping parameters with Xbox controller defaults
    this->declare_parameter("axes_mapping.left_stick_x", 0);
    this->declare_parameter("axes_mapping.left_stick_y", 1);
    this->declare_parameter("axes_mapping.right_stick_x", 3);
    this->declare_parameter("axes_mapping.right_stick_y", 4);
    this->declare_parameter("axes_mapping.dpad_x", 6);
    this->declare_parameter("axes_mapping.dpad_y", 7);
    this->declare_parameter("axes_mapping.deadzone", 0.1);
    this->declare_parameter("axes_mapping.dpad_deadzone", 0.5);
    
    axes_map_.left_stick_x = this->get_parameter("axes_mapping.left_stick_x").as_int();
    axes_map_.left_stick_y = this->get_parameter("axes_mapping.left_stick_y").as_int();
    axes_map_.right_stick_x = this->get_parameter("axes_mapping.right_stick_x").as_int();
    axes_map_.right_stick_y = this->get_parameter("axes_mapping.right_stick_y").as_int();
    axes_map_.dpad_x = this->get_parameter("axes_mapping.dpad_x").as_int();
    axes_map_.dpad_y = this->get_parameter("axes_mapping.dpad_y").as_int();
    axes_map_.deadzone = this->get_parameter("axes_mapping.deadzone").as_double();
    axes_map_.dpad_deadzone = this->get_parameter("axes_mapping.dpad_deadzone").as_double();
    
    // Declare and get mirror movement parameter
    this->declare_parameter("mirror_movement", true);
    mirror_movement_ = this->get_parameter("mirror_movement").as_bool();
}

void JoystickTeleop::printButtonMapping() {
    RCLCPP_INFO(get_logger(), "📋 Button Mapping Configuration:");
    RCLCPP_INFO(get_logger(), "   X Button:           %d", button_map_.x_button);
    RCLCPP_INFO(get_logger(), "   A Button:           %d", button_map_.a_button);
    RCLCPP_INFO(get_logger(), "   B Button:           %d", button_map_.b_button);
    RCLCPP_INFO(get_logger(), "   Y Button:           %d", button_map_.y_button);
    RCLCPP_INFO(get_logger(), "   LB Button:          %d", button_map_.lb_button);
    RCLCPP_INFO(get_logger(), "   RB Button:          %d", button_map_.rb_button);
    RCLCPP_INFO(get_logger(), "   Back Button:        %d", button_map_.back_button);
    RCLCPP_INFO(get_logger(), "   Start Button:       %d", button_map_.start_button);
    RCLCPP_INFO(get_logger(), "   Left Stick Button:  %d", button_map_.left_stick_button);
    RCLCPP_INFO(get_logger(), "   Right Stick Button: %d", button_map_.right_stick_button);
    RCLCPP_INFO(get_logger(), "📋 Axes Mapping Configuration:");
    RCLCPP_INFO(get_logger(), "   Left Stick X:       %d", axes_map_.left_stick_x);
    RCLCPP_INFO(get_logger(), "   Left Stick Y:       %d", axes_map_.left_stick_y);
    RCLCPP_INFO(get_logger(), "   Right Stick X:      %d", axes_map_.right_stick_x);
    RCLCPP_INFO(get_logger(), "   Right Stick Y:      %d", axes_map_.right_stick_y);
    RCLCPP_INFO(get_logger(), "   D-Pad X:            %d", axes_map_.dpad_x);
    RCLCPP_INFO(get_logger(), "   D-Pad Y:            %d", axes_map_.dpad_y);
    RCLCPP_INFO(get_logger(), "   Deadzone:           %.2f", axes_map_.deadzone);
    RCLCPP_INFO(get_logger(), "   D-Pad Deadzone:     %.2f", axes_map_.dpad_deadzone);
    RCLCPP_INFO(get_logger(), "📋 Mirror Movement:     %s", mirror_movement_ ? "ENABLED" : "DISABLED");
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
        // FSM command logic - publish directly to /fsm_command topic
        int32_t fsm_command = 0;
        if (msg->buttons[button_map_.b_button] && msg->buttons[button_map_.lb_button]) {
            fsm_command = 1; // LB + B -> HOME
        } else if (msg->buttons[button_map_.a_button] && msg->buttons[button_map_.lb_button]) {
            fsm_command = 2; // LB + A -> HOLD
        } else if (msg->buttons[button_map_.start_button]) {
            fsm_command = 3; // START -> OCS2/MOVE
        } else if (msg->buttons[button_map_.y_button] && msg->buttons[button_map_.lb_button]) {
            fsm_command = 4; // LB + Y -> MOVEJ
        }
        
        // Publish FSM command if there's a command
        if (fsm_command != 0) {
            auto fsm_cmd = std_msgs::msg::Int32();
            fsm_cmd.data = fsm_command;
            fsm_command_publisher_->publish(fsm_cmd);
        } else {
            // Process axes for incremental control when no FSM command is active
            processAxes(msg);
            
            // Publish incremental control to /control_input (command is always 0)
            inputs_.command = 0;
            publisher_->publish(inputs_);
        }
    }
}

void JoystickTeleop::processButtons(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Check if we have enough buttons
    size_t max_button_index = std::max({
        button_map_.x_button, button_map_.a_button, button_map_.b_button,
        button_map_.y_button, button_map_.lb_button, button_map_.rb_button,
        button_map_.back_button, button_map_.start_button,
        button_map_.left_stick_button, button_map_.right_stick_button
    });
    
    if (msg->buttons.size() <= max_button_index) {
        return;
    }

    // Check button states using configured mapping
    bool x_pressed = msg->buttons[button_map_.x_button];
    bool a_pressed = msg->buttons[button_map_.a_button];
    bool b_pressed = msg->buttons[button_map_.b_button];
    bool y_pressed = msg->buttons[button_map_.y_button];
    bool lb_pressed = msg->buttons[button_map_.lb_button];
    bool rb_pressed = msg->buttons[button_map_.rb_button];
    bool back_pressed = msg->buttons[button_map_.back_button];
    bool start_pressed = msg->buttons[button_map_.start_button];
    bool left_stick_pressed = msg->buttons[button_map_.left_stick_button];
    bool right_stick_pressed = msg->buttons[button_map_.right_stick_button];

    // Detect button press events (rising edge)
    bool right_stick_just_pressed = right_stick_pressed && !last_right_stick_pressed_;
    bool a_just_pressed = a_pressed && !last_a_pressed_;
    bool x_just_pressed = x_pressed && !last_x_pressed_;
    bool left_stick_just_pressed = left_stick_pressed && !last_left_stick_pressed_;

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
        currentTarget_ = currentTarget_ == 1 ? 2 : 1;
        inputs_.target = currentTarget_;
        RCLCPP_INFO(get_logger(), "🎮 Switched target arm to: %s", 
                    (currentTarget_ == 1) ? "LEFT" : "RIGHT");
    }

    if (x_just_pressed && enabled_) {
        // X button: toggle gripper - 像VR一样，基于当前状态切换（不立即更新状态）
        bool current_gripper_state = (currentTarget_ == 1) ? left_gripper_open_ : right_gripper_open_;
        bool should_open = !current_gripper_state;  // 计算目标状态

        sendGripperCommand(should_open);  // 发布命令

        // 不立即更新状态，等待订阅器回调来更新（与VR逻辑一致）
        std::string arm_name = (currentTarget_ == 1) ? "LEFT" : "RIGHT";
        RCLCPP_INFO(get_logger(), "🎮 %s gripper command sent: %s",
                   arm_name.c_str(), should_open ? "OPEN" : "CLOSE");
    }

    if (left_stick_just_pressed && enabled_) {
        // Left stick button: toggle speed mode
        high_speed_mode_ = !high_speed_mode_;
        RCLCPP_INFO(get_logger(), "🎮 Speed mode switched to: %s (Scale: %.2f)",
                   high_speed_mode_ ? "HIGH" : "LOW",
                   high_speed_mode_ ? high_speed_scale_ : low_speed_scale_);
    }
}

void JoystickTeleop::processAxes(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Check if we have enough axes
    size_t max_axis_index = std::max({
        axes_map_.left_stick_x, axes_map_.left_stick_y,
        axes_map_.right_stick_x, axes_map_.right_stick_y,
        axes_map_.dpad_x, axes_map_.dpad_y
    });
    
    if (msg->axes.size() <= max_axis_index) {
        return;
    }

    // Left stick controls position (x, y) - use configured mapping
    double left_stick_x = applyDeadzone(msg->axes[axes_map_.left_stick_x], axes_map_.deadzone);
    double left_stick_y = applyDeadzone(msg->axes[axes_map_.left_stick_y], axes_map_.deadzone);

    // Apply mirror movement if enabled (invert both X and Y)
    if (mirror_movement_) {
        left_stick_x = -left_stick_x;
        left_stick_y = -left_stick_y;
    }

    // Right stick controls position (z) and rotation (yaw) - use configured mapping
    double right_stick_x = applyDeadzone(msg->axes[axes_map_.right_stick_x], axes_map_.deadzone);
    double right_stick_y = applyDeadzone(msg->axes[axes_map_.right_stick_y], axes_map_.deadzone);

    // D-pad controls roll and pitch rotation - use configured mapping
    double dpad_x = applyDeadzone(msg->axes[axes_map_.dpad_x], axes_map_.dpad_deadzone);
    double dpad_y = applyDeadzone(msg->axes[axes_map_.dpad_y], axes_map_.dpad_deadzone);

    // Apply speed scaling based on current mode
    double speed_scale = high_speed_mode_ ? high_speed_scale_ : low_speed_scale_;

    // Update position (x, y, z) with speed scaling
    inputs_.x = left_stick_y * speed_scale;   // Left stick Y: forward/backward
    inputs_.y = left_stick_x * speed_scale;   // Left stick X: left/right
    inputs_.z = right_stick_y * speed_scale;  // Right stick Y: up/down

    // Update rotation (roll, pitch, yaw) with speed scaling
    inputs_.roll = dpad_x * speed_scale;         // D-pad X: roll
    inputs_.pitch = -dpad_y * speed_scale;       // D-pad Y: pitch (inverted)
    inputs_.yaw = right_stick_x * speed_scale;   // Right stick X: yaw
}

double JoystickTeleop::applyDeadzone(double value, double deadzone) const {
    if (std::abs(value) < deadzone) {
        return 0.0;
    }
    return value;
}

void JoystickTeleop::sendGripperCommand(bool open)
{
    auto target_msg = std_msgs::msg::Int32();
    target_msg.data = open ? 1 : 0;

    // 根据当前目标手臂发布到对应的话题
    if (currentTarget_ == 1) {
        // 左臂
        left_target_command_publisher_->publish(target_msg);
        RCLCPP_DEBUG(this->get_logger(), "Sent left gripper command: target=%d", target_msg.data);
    } else if (currentTarget_ == 2) {
        // 右臂
        right_target_command_publisher_->publish(target_msg);
        RCLCPP_DEBUG(this->get_logger(), "Sent right gripper command: target=%d", target_msg.data);
    }
}

void JoystickTeleop::left_target_command_callback(std_msgs::msg::Int32::SharedPtr msg)
{
    // 更新左夹爪状态
    left_gripper_open_ = (msg->data == 1);
    RCLCPP_DEBUG(this->get_logger(), "Received left target command: target=%d, state=%s",
                msg->data, left_gripper_open_ ? "open" : "closed");
}

void JoystickTeleop::right_target_command_callback(std_msgs::msg::Int32::SharedPtr msg)
{
    // 更新右夹爪状态
    right_gripper_open_ = (msg->data == 1);
    RCLCPP_DEBUG(this->get_logger(), "Received right target command: target=%d, state=%s",
                msg->data, right_gripper_open_ ? "open" : "closed");
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickTeleop>();
    spin(node);
    rclcpp::shutdown();
    return 0;
}
