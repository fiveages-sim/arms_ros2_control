//
// Created by tlab-uav on 24-9-13.
//

#include "arms_teleop/joystick_teleop.h"

using std::placeholders::_1;

JoystickTeleop::JoystickTeleop() : Node("joystick_teleop_node") {
    publisher_ = create_publisher<arms_ros2_control_msgs::msg::Inputs>("control_input", 10);
    subscription_ = create_subscription<
        sensor_msgs::msg::Joy>("joy", 10, std::bind(&JoystickTeleop::joy_callback, this, _1));
    
    // Initialize gripper action client
    gripper_action_client_ = rclcpp_action::create_client<control_msgs::action::ParallelGripperCommand>(
        this, "/hand_controller/gripper_cmd");
    
    // Declare gripper parameters with default values
    this->declare_parameter("gripper.open_position", 1.0);
    this->declare_parameter("gripper.closed_position", 0.0);
    this->declare_parameter("gripper.max_effort", 10.0);
    
    // Get parameter values
    gripper_open_position_ = this->get_parameter("gripper.open_position").as_double();
    gripper_closed_position_ = this->get_parameter("gripper.closed_position").as_double();
    gripper_max_effort_ = this->get_parameter("gripper.max_effort").as_double();
    
    // Initialize gripper state
    gripper_open_ = false;
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
        
        // If no command is set and X is pressed alone, toggle gripper
        if (x_pressed && !last_x_pressed_) {
            // Toggle gripper state
            gripper_open_ = !gripper_open_;
            
            // Send gripper command
            if (gripper_open_) {
                if (sendGripperCommand(gripper_open_position_)) {
                    RCLCPP_INFO(this->get_logger(), "Gripper opened");
                }
            } else {
                if (sendGripperCommand(gripper_closed_position_)) {
                    RCLCPP_INFO(this->get_logger(), "Gripper closed");
                }
            }
        }
    }
    
    // Update last X button state
    last_x_pressed_ = x_pressed;
    
    publisher_->publish(inputs_);
}

bool JoystickTeleop::sendGripperCommand(double position)
{
    if (!gripper_action_client_->action_server_is_ready())
    {
        RCLCPP_WARN(this->get_logger(), "Gripper action server not ready");
        return false;
    }

    auto goal_msg = control_msgs::action::ParallelGripperCommand::Goal();
    goal_msg.command.position = {position}; // 将单个位置值包装成vector
    goal_msg.command.effort = {gripper_max_effort_}; // 将单个力度值包装成vector

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::ParallelGripperCommand>::SendGoalOptions();
    send_goal_options.result_callback = [this](const auto &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_DEBUG(this->get_logger(), "Gripper command succeeded");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Gripper command failed");
        }
    };

    gripper_action_client_->async_send_goal(goal_msg, send_goal_options);
    return true;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickTeleop>();
    spin(node);
    rclcpp::shutdown();
    return 0;
}
