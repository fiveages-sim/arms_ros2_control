//
// Created by tlab-uav on 24-9-13.
//

#include "arms_teleop/joystick_teleop.h"

using std::placeholders::_1;

JoystickTeleop::JoystickTeleop() : Node("joystick_teleop_node") {
    publisher_ = create_publisher<arms_ros2_control_msgs::msg::Inputs>("control_input", 10);
    fsm_command_publisher_ = create_publisher<std_msgs::msg::Int32>("/fsm_command", 10);
    chassis_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    subscription_ = create_subscription<
        sensor_msgs::msg::Joy>("joy", 10, std::bind(&JoystickTeleop::joy_callback, this, _1));
    
    // Load button and axes mapping from parameters
    loadButtonMapping();
    
    // Initialize control parameters
    updateRate_ = 20.0;
    
    // Initialize state
    enabled_ = false;
    lastUpdateTime_ = now();
    currentTarget_ = 1; // Start with left arm
    current_mode_ = ARM_MODE; // Start with arm control mode
    
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

    // Initialize separate gripper states for left and right arms (for display purposes only)
    left_gripper_open_ = false;   // Left arm gripper initially closed
    right_gripper_open_ = false;  // Right arm gripper initially closed
    
    // Initialize speed mode (start with low speed)
    high_speed_mode_ = false;
    
    // Initialize last FSM command
    last_fsm_command_ = 0;
    
    // Load speed scaling parameters
    this->declare_parameter("speed.low_scale", 0.3);
    this->declare_parameter("speed.high_scale", 1.0);
    low_speed_scale_ = this->get_parameter("speed.low_scale").as_double();
    high_speed_scale_ = this->get_parameter("speed.high_scale").as_double();
    
    // Load chassis speed scaling parameters
    this->declare_parameter("chassis.linear_scale", 0.25);
    this->declare_parameter("chassis.angular_scale", 0.5);
    chassis_linear_scale_ = this->get_parameter("chassis.linear_scale").as_double();
    chassis_angular_scale_ = this->get_parameter("chassis.angular_scale").as_double();
    
    // Initialize inputs message (only for incremental control)
    inputs_.x = 0.0;
    inputs_.y = 0.0;
    inputs_.z = 0.0;
    inputs_.roll = 0.0;
    inputs_.pitch = 0.0;
    inputs_.yaw = 0.0;
    inputs_.target = currentTarget_;
    inputs_.hand_command = -1;  // -1 means no hand command (will be set when X button is pressed)
    
    // Initialize chassis command
    chassis_cmd_.linear.x = 0.0;
    chassis_cmd_.linear.y = 0.0;
    chassis_cmd_.linear.z = 0.0;
    chassis_cmd_.angular.x = 0.0;
    chassis_cmd_.angular.y = 0.0;
    chassis_cmd_.angular.z = 0.0;
    
    RCLCPP_INFO(get_logger(), "🎮 JoystickTeleop created");
    RCLCPP_INFO(get_logger(), "🎮 Joystick control is DISABLED by default. Press right stick to enable.");
    RCLCPP_INFO(get_logger(), "🎮 Control Mode: RB=switch between ARM and CHASSIS (Current: %s)", 
                current_mode_ == ARM_MODE ? "ARM" : "CHASSIS");
    RCLCPP_INFO(get_logger(), "🎮 Normal Mode: A=switch arm, X=toggle gripper, Left stick=switch speed");
    RCLCPP_INFO(get_logger(), "🎮 FSM Mode (LB pressed): LB+A=HOME, LB+B=HOLD, LB+Y=MOVEJ, LB+START=OCS2");
    RCLCPP_INFO(get_logger(), "🎮 Speed mode: Left stick button=toggle high/low speed (Current: %s)", 
                high_speed_mode_ ? "HIGH" : "LOW");
    RCLCPP_INFO(get_logger(), "🎮 Mirror mode: Back button=toggle mirror movement (Current: %s)", 
                mirror_movement_ ? "ENABLED" : "DISABLED");
    
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

    // Process buttons first (handles both FSM and normal functions)
    processButtons(msg);

    // Only process axes if joystick is enabled and not in FSM mode
    if (enabled_) {
        // Check if LB is pressed (FSM mode - only for arm control)
        bool lb_pressed = (msg->buttons.size() > button_map_.lb_button) ? 
                         msg->buttons[button_map_.lb_button] : false;
        
        if (current_mode_ == ARM_MODE) {
            // Arm control mode
            if (!lb_pressed) {
                // Normal mode: Process axes for incremental control
                processAxes(msg);
                
                // Publish incremental control to /control_input (includes hand_command if set)
                publisher_->publish(inputs_);
                
                // Reset hand_command after publishing (only send once per button press)
                if (inputs_.hand_command != -1) {
                    inputs_.hand_command = -1;
                }
            }
        } else {
            // Chassis control mode
            processChassisAxes(msg);
            
            // Publish chassis velocity command
            chassis_publisher_->publish(chassis_cmd_);
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

    // Detect button press events (rising edge) - BEFORE updating last states
    bool right_stick_just_pressed = right_stick_pressed && !last_right_stick_pressed_;
    bool a_just_pressed = a_pressed && !last_a_pressed_;
    bool b_just_pressed = b_pressed && !last_b_pressed_;
    bool y_just_pressed = y_pressed && !last_y_pressed_;
    bool x_just_pressed = x_pressed && !last_x_pressed_;
    bool start_just_pressed = start_pressed && !last_start_pressed_;
    bool left_stick_just_pressed = left_stick_pressed && !last_left_stick_pressed_;
    bool rb_just_pressed = rb_pressed && !last_rb_pressed_;
    bool back_just_pressed = back_pressed && !last_back_pressed_;

    // Process button events
    // Right stick press: toggle control (always processed, regardless of LB)
    if (right_stick_just_pressed) {
        enabled_ = !enabled_;
        if (enabled_) {
            RCLCPP_INFO(get_logger(), "🎮 Joystick control ENABLED!");
        } else {
            RCLCPP_INFO(get_logger(), "🎮 Joystick control DISABLED!");
            // Reset commands when disabling
            if (current_mode_ == ARM_MODE) {
                inputs_.x = 0.0;
                inputs_.y = 0.0;
                inputs_.z = 0.0;
                inputs_.roll = 0.0;
                inputs_.pitch = 0.0;
                inputs_.yaw = 0.0;
                publisher_->publish(inputs_);
            } else {
                chassis_cmd_.linear.x = 0.0;
                chassis_cmd_.linear.y = 0.0;
                chassis_cmd_.angular.z = 0.0;
                chassis_publisher_->publish(chassis_cmd_);
            }
        }
    }
    
    // RB button: switch between ARM and CHASSIS control modes
    if (rb_just_pressed) {
        current_mode_ = (current_mode_ == ARM_MODE) ? CHASSIS_MODE : ARM_MODE;
        RCLCPP_INFO(get_logger(), "🎮 Control mode switched to: %s", 
                    current_mode_ == ARM_MODE ? "ARM" : "CHASSIS");
        
        // Reset commands when switching modes
        if (current_mode_ == ARM_MODE) {
            chassis_cmd_.linear.x = 0.0;
            chassis_cmd_.linear.y = 0.0;
            chassis_cmd_.angular.z = 0.0;
            chassis_publisher_->publish(chassis_cmd_);
        } else {
            inputs_.x = 0.0;
            inputs_.y = 0.0;
            inputs_.z = 0.0;
            inputs_.roll = 0.0;
            inputs_.pitch = 0.0;
            inputs_.yaw = 0.0;
            publisher_->publish(inputs_);
        }
    }
    
    // Back button: toggle mirror movement mode (works in both ARM and CHASSIS modes)
    if (back_just_pressed) {
        mirror_movement_ = !mirror_movement_;
        RCLCPP_INFO(get_logger(), "🎮 Mirror movement mode: %s", 
                    mirror_movement_ ? "ENABLED" : "DISABLED");
    }

    // Only process other functions if joystick is enabled
    if (enabled_) {
        // FSM mode: LB button acts as "FSM mode" modifier
        if (lb_pressed) {
            // FSM mode: LB + button combinations for state transitions
            int32_t fsm_command = 0;
            
            if (a_just_pressed) {
                // LB + A: HOLD → HOME (command=1)
                fsm_command = 1;
                RCLCPP_INFO(get_logger(), "🎮 FSM: HOLD → HOME");
            } else if (b_just_pressed) {
                // LB + B: Any state → HOLD (command=2, safe return to HOLD)
                fsm_command = 2;
                RCLCPP_INFO(get_logger(), "🎮 FSM: → HOLD (safe return)");
            } else if (y_just_pressed) {
                // LB + Y: HOLD → MOVEJ (command=4)
                fsm_command = 4;
                RCLCPP_INFO(get_logger(), "🎮 FSM: HOLD → MOVEJ");
            } else if (start_just_pressed) {
                // LB + START: HOLD → OCS2 (command=3)
                fsm_command = 3;
                RCLCPP_INFO(get_logger(), "🎮 FSM: HOLD → OCS2");
            } else if (x_just_pressed) {
                // LB + X: HOME State Switch
                fsm_command = 100;
                RCLCPP_INFO(get_logger(), "🎮 FSM: HOME POSE Switch");
            }
            
            // Publish FSM command if there's a command
            if (fsm_command != 0) {
                auto fsm_cmd = std_msgs::msg::Int32();
                fsm_cmd.data = fsm_command;
                fsm_command_publisher_->publish(fsm_cmd);
                last_fsm_command_ = fsm_command;
            }
        } else {
            // Normal mode: Check if we need to send 0 when transitioning from non-zero FSM command
            if (last_fsm_command_ != 0) {
                // Last time we sent a non-zero command, now we're in normal mode, send 0 once
                auto fsm_cmd = std_msgs::msg::Int32();
                fsm_cmd.data = 0;
                fsm_command_publisher_->publish(fsm_cmd);
                last_fsm_command_ = 0;
            }
            
            // Normal mode: Process normal button functions (only in ARM mode)
            if (current_mode_ == ARM_MODE) {
                if (a_just_pressed) {
                    // A button: switch target arm
                    currentTarget_ = currentTarget_ == 1 ? 2 : 1;
                    inputs_.target = currentTarget_;
                    RCLCPP_INFO(get_logger(), "🎮 Switched target arm to: %s", 
                                (currentTarget_ == 1) ? "LEFT" : "RIGHT");
                }

                if (x_just_pressed) {
                    // X button: toggle gripper
                    bool current_gripper_state = (currentTarget_ == 1) ? left_gripper_open_ : right_gripper_open_;
                    bool should_open = !current_gripper_state;

                    // Set hand_command in inputs message (will be sent with next control_input message)
                    inputs_.hand_command = should_open ? 1 : 0;
                    
                    // Update local state for display
                    if (currentTarget_ == 1) {
                        left_gripper_open_ = should_open;
                    } else {
                        right_gripper_open_ = should_open;
                    }

                    std::string arm_name = (currentTarget_ == 1) ? "LEFT" : "RIGHT";
                    RCLCPP_INFO(get_logger(), "🎮 %s gripper command: %s (will be sent with control_input)",
                               arm_name.c_str(), should_open ? "OPEN" : "CLOSE");
                }
            }
            
            // Speed mode toggle (works in both modes)
            if (left_stick_just_pressed) {
                // Left stick button: toggle speed mode
                high_speed_mode_ = !high_speed_mode_;
                RCLCPP_INFO(get_logger(), "🎮 Speed mode switched to: %s (Scale: %.2f)",
                           high_speed_mode_ ? "HIGH" : "LOW",
                           high_speed_mode_ ? high_speed_scale_ : low_speed_scale_);
            }
        }
    }

    // Update last button states AFTER processing
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

    // Apply mirror movement to D-pad if enabled (invert both X and Y)
    if (!mirror_movement_) {
        dpad_x = -dpad_x;
        dpad_y = -dpad_y;
    }

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

void JoystickTeleop::processChassisAxes(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Check if we have enough axes
    size_t max_axis_index = std::max({
        axes_map_.left_stick_x, axes_map_.left_stick_y,
        axes_map_.right_stick_x, axes_map_.right_stick_y
    });
    
    if (msg->axes.size() <= max_axis_index) {
        return;
    }

    // Left stick controls linear velocity
    // Left stick Y: forward/backward (linear.x)
    // Left stick X: left/right (linear.y, for omnidirectional chassis)
    // Note: Mirror movement is NOT applied to chassis control
    double left_stick_x = applyDeadzone(msg->axes[axes_map_.left_stick_x], axes_map_.deadzone);
    double left_stick_y = applyDeadzone(msg->axes[axes_map_.left_stick_y], axes_map_.deadzone);

    // Right stick controls angular velocity
    // Right stick X: rotation (angular.z)
    double right_stick_x = applyDeadzone(msg->axes[axes_map_.right_stick_x], axes_map_.deadzone);

    // Apply speed scaling based on current mode
    double speed_scale = high_speed_mode_ ? high_speed_scale_ : low_speed_scale_;

    // Update chassis velocity command
    chassis_cmd_.linear.x = left_stick_y * chassis_linear_scale_ * speed_scale;   // Forward/backward
    chassis_cmd_.linear.y = left_stick_x * chassis_linear_scale_ * speed_scale;   // Left/right (for omnidirectional)
    chassis_cmd_.linear.z = 0.0;                                                  // No vertical movement
    chassis_cmd_.angular.x = 0.0;                                                 // No roll
    chassis_cmd_.angular.y = 0.0;                                                 // No pitch
    chassis_cmd_.angular.z = right_stick_x * chassis_angular_scale_ * speed_scale; // Rotation
}

// sendGripperCommand, left_target_command_callback, and right_target_command_callback
// are no longer needed - hand commands are now sent via Inputs message

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickTeleop>();
    spin(node);
    rclcpp::shutdown();
    return 0;
}
