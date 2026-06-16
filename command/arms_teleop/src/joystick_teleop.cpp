//
// Created by tlab-uav on 24-9-13.
//

#include "arms_teleop/joystick_teleop.h"

#include <algorithm>
#include <cmath>
#include <limits>

using std::placeholders::_1;

namespace {

constexpr float kNoHandCommand = std::numeric_limits<float>::quiet_NaN();

int param_int(rclcpp::Node * node, const char * name, int default_value)
{
    node->declare_parameter(name, default_value);
    return node->get_parameter(name).as_int();
}

double param_double(rclcpp::Node * node, const char * name, double default_value)
{
    node->declare_parameter(name, default_value);
    return node->get_parameter(name).as_double();
}

bool param_bool(rclcpp::Node * node, const char * name, bool default_value)
{
    node->declare_parameter(name, default_value);
    return node->get_parameter(name).as_bool();
}

}  // namespace

JoystickTeleop::JoystickTeleop() : Node("joystick_teleop_node") {
    publisher_ = create_publisher<arms_ros2_control_msgs::msg::Inputs>("control_input", 10);
    fsm_command_publisher_ = create_publisher<std_msgs::msg::Int32>("/fsm_command", 10);
    chassis_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    subscription_ = create_subscription<
        sensor_msgs::msg::Joy>("joy", 10, std::bind(&JoystickTeleop::joy_callback, this, _1));
    waist_lifting_publisher_ = create_publisher<std_msgs::msg::Float64>("/body_joint_controller/waist_lifting_command", 10);
    waist_turning_publisher_ = create_publisher<std_msgs::msg::Float64>("/body_joint_controller/waist_turning_command", 10);
    
    loadParameters();

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

    left_gripper_ratio_ = 0.0;
    right_gripper_ratio_ = 0.0;
    
    // Initialize speed mode (start with low speed)
    high_speed_mode_ = false;
    
    // Initialize last FSM command
    last_fsm_command_ = 0;

    // Initialize inputs message (only for incremental control)
    inputs_.x = 0.0;
    inputs_.y = 0.0;
    inputs_.z = 0.0;
    inputs_.roll = 0.0;
    inputs_.pitch = 0.0;
    inputs_.yaw = 0.0;
    inputs_.target = currentTarget_;
    inputs_.hand_command = kNoHandCommand;
    
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
    RCLCPP_INFO(get_logger(), "🎮 Normal Mode: A=switch arm, X=toggle gripper, LT/RT=gripper ratio, Left stick=switch speed");
    RCLCPP_INFO(get_logger(), "🎮 FSM Mode (LB pressed): LB+A=HOME, LB+B=HOLD, LB+Y=MOVEJ, LB+START=OCS2");
    RCLCPP_INFO(get_logger(), "🎮 Speed mode: Left stick button=toggle high/low speed (Current: %s)", 
                high_speed_mode_ ? "HIGH" : "LOW");
    RCLCPP_INFO(get_logger(), "🎮 Mirror mode: Back button=toggle mirror movement (Current: %s)", 
                mirror_movement_ ? "ENABLED" : "DISABLED");
    
    // Print button mapping configuration
    printButtonMapping();
}

void JoystickTeleop::loadParameters() {
    // Mapping defaults are fallbacks only; launch loads config/joy_mapping/*.yaml.

    button_map_.x_button = param_int(this, "button_mapping.x_button", 0);
    button_map_.a_button = param_int(this, "button_mapping.a_button", 0);
    button_map_.b_button = param_int(this, "button_mapping.b_button", 0);
    button_map_.y_button = param_int(this, "button_mapping.y_button", 0);
    button_map_.lb_button = param_int(this, "button_mapping.lb_button", 0);
    button_map_.rb_button = param_int(this, "button_mapping.rb_button", 0);
    button_map_.back_button = param_int(this, "button_mapping.back_button", 0);
    button_map_.start_button = param_int(this, "button_mapping.start_button", 0);
    button_map_.left_stick_button = param_int(this, "button_mapping.left_stick_button", 0);
    button_map_.right_stick_button = param_int(this, "button_mapping.right_stick_button", 0);
    button_map_.dpad_up = param_int(this, "button_mapping.dpad_up_button", -1);
    button_map_.dpad_down = param_int(this, "button_mapping.dpad_down_button", -1);
    button_map_.dpad_left = param_int(this, "button_mapping.dpad_left_button", -1);
    button_map_.dpad_right = param_int(this, "button_mapping.dpad_right_button", -1);

    axes_map_.left_stick_x = param_int(this, "axes_mapping.left_stick_x", 0);
    axes_map_.left_stick_y = param_int(this, "axes_mapping.left_stick_y", 1);
    axes_map_.right_stick_x = param_int(this, "axes_mapping.right_stick_x", 2);
    axes_map_.right_stick_y = param_int(this, "axes_mapping.right_stick_y", 3);
    axes_map_.deadzone = param_double(this, "axes_mapping.deadzone", 0.1);
    arm_axes_activation_threshold_ =
        param_double(this, "axes_mapping.arm_activation_threshold", 0.5);

    if (button_map_.dpad_up >= 0) {
        axes_map_.dpad_x = 0;
        axes_map_.dpad_y = 0;
        axes_map_.dpad_deadzone = 0.0;
    } else {
        axes_map_.dpad_x = param_int(this, "axes_mapping.dpad_x", 0);
        axes_map_.dpad_y = param_int(this, "axes_mapping.dpad_y", 0);
        axes_map_.dpad_deadzone = param_double(this, "axes_mapping.dpad_deadzone", 0.5);
    }

    axes_map_.left_trigger = param_int(this, "axes_mapping.left_trigger", -1);
    axes_map_.right_trigger = param_int(this, "axes_mapping.right_trigger", -1);
    axes_map_.trigger_deadzone = param_double(this, "axes_mapping.trigger_deadzone", 0.05);
    gripper_step_per_tick_ = param_double(this, "gripper.step_per_tick", 0.05);

    mirror_movement_ = param_bool(this, "mirror_movement", true);
    low_speed_scale_ = param_double(this, "speed.low_scale", 0.3);
    high_speed_scale_ = param_double(this, "speed.high_scale", 1.0);
    chassis_linear_scale_ = param_double(this, "chassis.linear_scale", 0.25);
    chassis_angular_scale_ = param_double(this, "chassis.angular_scale", 0.5);
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
    if (button_map_.dpad_up >= 0) {
        RCLCPP_INFO(get_logger(), "   D-Pad (buttons):    up=%d down=%d left=%d right=%d",
                    button_map_.dpad_up, button_map_.dpad_down,
                    button_map_.dpad_left, button_map_.dpad_right);
    }
    RCLCPP_INFO(get_logger(), "📋 Axes Mapping Configuration:");
    RCLCPP_INFO(get_logger(), "   Left Stick X:       %d", axes_map_.left_stick_x);
    RCLCPP_INFO(get_logger(), "   Left Stick Y:       %d", axes_map_.left_stick_y);
    RCLCPP_INFO(get_logger(), "   Right Stick X:      %d", axes_map_.right_stick_x);
    RCLCPP_INFO(get_logger(), "   Right Stick Y:      %d", axes_map_.right_stick_y);
    if (button_map_.dpad_up < 0) {
        RCLCPP_INFO(get_logger(), "   D-Pad X:            %d", axes_map_.dpad_x);
        RCLCPP_INFO(get_logger(), "   D-Pad Y:            %d", axes_map_.dpad_y);
    }
    RCLCPP_INFO(get_logger(), "   Deadzone:           %.2f", axes_map_.deadzone);
    if (button_map_.dpad_up < 0) {
        RCLCPP_INFO(get_logger(), "   D-Pad Deadzone:     %.2f", axes_map_.dpad_deadzone);
    }
    RCLCPP_INFO(get_logger(), "   Arm Activation Thr: %.2f", arm_axes_activation_threshold_);
    if (axes_map_.left_trigger >= 0 || axes_map_.right_trigger >= 0) {
        RCLCPP_INFO(get_logger(), "   LT / RT Axes:       %d / %d (deadzone %.2f, step %.3f)",
                    axes_map_.left_trigger, axes_map_.right_trigger,
                    axes_map_.trigger_deadzone, gripper_step_per_tick_);
    }
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
    inputs_.hand_command = kNoHandCommand;

    processButtons(msg);

    // Only process axes if joystick is enabled and not in FSM mode
    if (enabled_) {
        // Check if LB is pressed (FSM mode - only for arm control)
        bool lb_pressed = (static_cast<size_t>(button_map_.lb_button) < msg->buttons.size()) ? 
                         msg->buttons[button_map_.lb_button] : false;
        
        if (current_mode_ == ARM_MODE) {
            // Arm control mode
            if (!lb_pressed) {
                // Normal mode: Process axes for incremental control
                processAxes(msg);
                processGripperTriggers(msg);

                publisher_->publish(inputs_);
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
                    double & ratio = (currentTarget_ == 1) ? left_gripper_ratio_ : right_gripper_ratio_;
                    ratio = (ratio > 0.5) ? 0.0 : 1.0;
                    inputs_.hand_command = static_cast<float>(ratio);

                    const std::string arm_name = (currentTarget_ == 1) ? "LEFT" : "RIGHT";
                    RCLCPP_INFO(get_logger(), "🎮 %s gripper ratio set to: %.0f%%",
                                arm_name.c_str(), ratio * 100.0);
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

bool JoystickTeleop::isButtonPressed(const sensor_msgs::msg::Joy::SharedPtr msg, int index) const {
    return index >= 0 &&
           static_cast<size_t>(index) < msg->buttons.size() &&
           msg->buttons[static_cast<size_t>(index)] != 0;
}

JoystickTeleop::DpadAxes JoystickTeleop::readDpad(
    const sensor_msgs::msg::Joy::SharedPtr msg) const
{
    if (button_map_.dpad_up >= 0) {
        DpadAxes dpad;
        if (isButtonPressed(msg, button_map_.dpad_left)) {
            dpad.x -= 1.0;
        }
        if (isButtonPressed(msg, button_map_.dpad_right)) {
            dpad.x += 1.0;
        }
        if (isButtonPressed(msg, button_map_.dpad_up)) {
            dpad.y -= 1.0;
        }
        if (isButtonPressed(msg, button_map_.dpad_down)) {
            dpad.y += 1.0;
        }
        dpad.x = -dpad.x;
        dpad.y = -dpad.y;
        return dpad;
    }

    if (msg->axes.size() <= static_cast<size_t>(std::max(axes_map_.dpad_x, axes_map_.dpad_y))) {
        return {};
    }

    DpadAxes dpad;
    dpad.x = applyDeadzone(msg->axes[axes_map_.dpad_x], axes_map_.dpad_deadzone);
    dpad.y = applyDeadzone(msg->axes[axes_map_.dpad_y], axes_map_.dpad_deadzone);
    return dpad;
}

void JoystickTeleop::processAxes(const sensor_msgs::msg::Joy::SharedPtr msg) {
    size_t max_axis_index = std::max({
        axes_map_.left_stick_x, axes_map_.left_stick_y,
        axes_map_.right_stick_x, axes_map_.right_stick_y
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
    // Apply mirror movement to yaw when mirror mode is enabled
    if (mirror_movement_) {
        right_stick_x = -right_stick_x;
    }

    auto dpad = readDpad(msg);
    double dpad_x = dpad.x;
    double dpad_y = dpad.y;

    // Apply mirror movement to D-pad if enabled (invert both X and Y)
    if (!mirror_movement_) {
        dpad_x = -dpad_x;
        dpad_y = -dpad_y;
    }

    // Apply activation threshold in ARM mode to avoid accidental micro movements.
    auto applyActivationThreshold = [this](double value) {
        return std::abs(value) >= arm_axes_activation_threshold_ ? value : 0.0;
    };
    left_stick_x = applyActivationThreshold(left_stick_x);
    left_stick_y = applyActivationThreshold(left_stick_y);
    right_stick_x = applyActivationThreshold(right_stick_x);
    right_stick_y = applyActivationThreshold(right_stick_y);

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

double JoystickTeleop::readTriggerAxis(
    const sensor_msgs::msg::Joy::SharedPtr msg, int axis_index) const
{
    if (axis_index < 0 || static_cast<size_t>(axis_index) >= msg->axes.size()) {
        return 0.0;
    }

    const double axis_value = msg->axes[static_cast<size_t>(axis_index)];
    // SDL/game_controller triggers: idle ~1.0, pressed toward -1.0
    double pressed = (1.0 - axis_value) * 0.5;
    pressed = std::clamp(pressed, 0.0, 1.0);
    if (pressed < axes_map_.trigger_deadzone) {
        return 0.0;
    }
    return pressed;
}

void JoystickTeleop::processGripperTriggers(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (axes_map_.left_trigger < 0 && axes_map_.right_trigger < 0) {
        return;
    }

    const double lt = readTriggerAxis(msg, axes_map_.left_trigger);
    const double rt = readTriggerAxis(msg, axes_map_.right_trigger);
    if (lt <= 0.0 && rt <= 0.0) {
        return;
    }

    double & ratio = (currentTarget_ == 1) ? left_gripper_ratio_ : right_gripper_ratio_;
    const double prev_ratio = ratio;
    const double speed_scale = high_speed_mode_ ? high_speed_scale_ : low_speed_scale_;
    ratio += (rt - lt) * gripper_step_per_tick_ * speed_scale;
    ratio = std::clamp(ratio, 0.0, 1.0);

    if (std::abs(ratio - prev_ratio) < 1e-6) {
        return;
    }

    inputs_.hand_command = static_cast<float>(ratio);
}

void JoystickTeleop::processChassisAxes(const sensor_msgs::msg::Joy::SharedPtr msg) {
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

    // Right stick X: chassis rotation (angular.z)
    double right_stick_x = applyDeadzone(msg->axes[axes_map_.right_stick_x], axes_map_.deadzone);
    const auto dpad = readDpad(msg);
    const double dpad_x = dpad.x;
    const double dpad_y = dpad.y;

    auto waist_cmd = std_msgs::msg::Float64();
    auto waist_turn_cmd = std_msgs::msg::Float64();
    double speed_scale = high_speed_mode_ ? high_speed_scale_ : low_speed_scale_;

    const bool lifting_active = dpad_y > 0.5 || dpad_y < -0.5;
    const bool turning_active = dpad_x > 0.5 || dpad_x < -0.5;
    const double turning_direction_scale = mirror_movement_ ? -1.0 : 1.0;

    if (lifting_active) {
        waist_cmd.data = (dpad_y > 0.0) ? speed_scale : -speed_scale;
    }
    if (turning_active) {
        waist_turn_cmd.data = ((dpad_x > 0.0) ? -speed_scale : speed_scale) * turning_direction_scale;
    }

    waist_lifting_publisher_->publish(waist_cmd);
    waist_turning_publisher_->publish(waist_turn_cmd);

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
