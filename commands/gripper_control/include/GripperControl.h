#ifndef GRIPPER_CONTROL_H
#define GRIPPER_CONTROL_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <sensor_msgs/msg/joy.hpp>

class GripperControl : public rclcpp::Node
{
public:
    GripperControl();

private:
    void open_gripper();
    void close_gripper();
    void send_gripper_command(double position);
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_action_client_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int trigger_button_;
    double open_position_;
    double debounce_time_;
    double max_effort_;
    bool gripper_open_;
    rclcpp::Time last_toggle_time_;
    std::string controller_name_;  // 控制器名称
};

#endif // GRIPPER_CONTROL_H 