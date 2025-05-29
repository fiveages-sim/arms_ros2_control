//
// Created by fiveages on 25-5-23.
//

#include "GripperControl.h"


GripperControl::GripperControl() : Node("gripper_control")
{
    // 声明并获取参数
    this->declare_parameter<int>("trigger_button", 9);
    this->declare_parameter<double>("open_position", 0.08);
    this->declare_parameter<double>("debounce_time", 1.0);
    this->declare_parameter<double>("max_effort", -1.0);
    this->declare_parameter<std::string>("controller_name", "gripper_controller");
    
    this->get_parameter("trigger_button", trigger_button_);
    this->get_parameter("open_position", open_position_);
    this->get_parameter("debounce_time", debounce_time_);
    this->get_parameter("max_effort", max_effort_);
    this->get_parameter("controller_name", controller_name_);
    
    last_toggle_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

    // 构建完整的话题名
    std::string action_topic = "/" + controller_name_ + "/gripper_cmd";

    // 创建夹爪控制动作客户端
    gripper_action_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
        this, action_topic);

    // 订阅手柄/joy话题
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&GripperControl::joy_callback, this, std::placeholders::_1));

    // 等待动作服务器启动
    if (!gripper_action_client_->wait_for_action_server(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(this->get_logger(), "Gripper action server not available after waiting");
        rclcpp::shutdown();
    }

    // 节点启动时用一次性定时器自动打开夹爪
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        [this]() {
            open_gripper();
            gripper_open_ = true;
            timer_->cancel();
        }
    );
}

void GripperControl::open_gripper()
{
    send_gripper_command(open_position_); // 打开夹爪，使用参数设置的开口大小
}

void GripperControl::close_gripper()
{
    send_gripper_command(0.0); // 关闭夹爪，设置开口大小为 0.0m
}

void GripperControl::send_gripper_command(double position)
{
    auto goal_msg = control_msgs::action::GripperCommand::Goal();
    goal_msg.command.position = position;
    goal_msg.command.max_effort = max_effort_; // 使用参数化的最大力

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();
    send_goal_options.result_callback = [this](const auto &result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "Gripper command succeeded");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Gripper command failed");
        }
    };

    gripper_action_client_->async_send_goal(goal_msg, send_goal_options);
}

void GripperControl::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    auto now = this->now();
    // 检查trigger_button_对应的按钮是否被按下，防抖
    if (trigger_button_ >= 0 && msg->buttons.size() > static_cast<size_t>(trigger_button_) && msg->buttons[trigger_button_] == 1)
    {
        if ((now - last_toggle_time_).seconds() >= debounce_time_)
        {
            // 切换夹爪状态
            if (gripper_open_)
            {
                close_gripper();
                gripper_open_ = false;
                RCLCPP_INFO(this->get_logger(), "Gripper closed by joystick button %d", trigger_button_ + 1);
            }
            else
            {
                open_gripper();
                gripper_open_ = true;
                RCLCPP_INFO(this->get_logger(), "Gripper opened by joystick button %d", trigger_button_ + 1);
            }
            last_toggle_time_ = now;
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}