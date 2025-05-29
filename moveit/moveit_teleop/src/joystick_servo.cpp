#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>

// Constants for topics and frames
const std::string DEFAULT_JOY_TOPIC = "/joy";
const std::string DEFAULT_TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string DEFAULT_EEF_FRAME_ID = "panda_hand";
const std::string DEFAULT_BASE_FRAME_ID = "panda_link0";
const double DEFAULT_VELOCITY_SCALE = 1.0;  // Add default velocity scale

// Enums for button names -> axis/button array index
// For XBOX 1 controller
enum Axis
{
  LEFT_STICK_X = 0,
  LEFT_STICK_Y = 1,
  LEFT_TRIGGER = 2,
  RIGHT_STICK_X = 3,
  RIGHT_STICK_Y = 4,
  RIGHT_TRIGGER = 5
};

enum Button
{
  LEFT_BUMPER = 4,
  RIGHT_BUMPER = 5,
  CHANGE_VIEW = 0,
  MENU = 7
};

// Default values for axes
std::map<Axis, double> AXIS_DEFAULTS = { { LEFT_TRIGGER, 1.0 }, { RIGHT_TRIGGER, 1.0 } };

bool convertJoyToCmd(const std::vector<float>& axes, const std::vector<int>& buttons,
                     std::unique_ptr<geometry_msgs::msg::TwistStamped>& twist,
                     double velocity_scale)  // Add velocity_scale parameter
{
  // Map buttons to twist commands
  twist->twist.linear.z = axes[RIGHT_STICK_Y] * velocity_scale;
  twist->twist.linear.y = axes[RIGHT_STICK_X] * velocity_scale;

  double lin_x_right = -0.5 * (axes[RIGHT_TRIGGER] - AXIS_DEFAULTS.at(RIGHT_TRIGGER));
  double lin_x_left = 0.5 * (axes[LEFT_TRIGGER] - AXIS_DEFAULTS.at(LEFT_TRIGGER));
  twist->twist.linear.x = (lin_x_right + lin_x_left) * velocity_scale;

  twist->twist.angular.y = axes[LEFT_STICK_Y] * velocity_scale;
  twist->twist.angular.x = axes[LEFT_STICK_X] * velocity_scale;

  double roll_positive = buttons[RIGHT_BUMPER];
  double roll_negative = -1 * buttons[LEFT_BUMPER];
  twist->twist.angular.z = (roll_positive + roll_negative) * velocity_scale;

  return true;
}

namespace moveit_teleop
{
class JoyToServoPub : public rclcpp::Node
{
public:
  JoyToServoPub(const rclcpp::NodeOptions& options)
    : Node("joystick_servo", options)
  {
    // Declare parameters with default values
    this->declare_parameter("joy_topic", DEFAULT_JOY_TOPIC);
    this->declare_parameter("twist_topic", DEFAULT_TWIST_TOPIC);
    this->declare_parameter("eef_frame_id", DEFAULT_EEF_FRAME_ID);
    this->declare_parameter("base_frame_id", DEFAULT_BASE_FRAME_ID);
    this->declare_parameter("frame_switch_cooldown", 0.5);  // 0.5 seconds cooldown
    this->declare_parameter("velocity_scale", DEFAULT_VELOCITY_SCALE);  // Add velocity scale parameter

    // Get parameter values
    std::string joy_topic = this->get_parameter("joy_topic").as_string();
    std::string twist_topic = this->get_parameter("twist_topic").as_string();
    eef_frame_id_ = this->get_parameter("eef_frame_id").as_string();
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    frame_switch_cooldown_ = this->get_parameter("frame_switch_cooldown").as_double();
    velocity_scale_ = this->get_parameter("velocity_scale").as_double();  // Get velocity scale value
    frame_to_publish_ = base_frame_id_;
    last_frame_switch_time_ = this->now();

    // Setup pub/sub
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) { return joyCB(msg); });

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic, rclcpp::SystemDefaultsQoS());

    // Create a service client to start the ServoNode
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  }

  void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
  {
    // Create the twist message
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();

    // This call updates the frame for twist commands
    updateCmdFrame(frame_to_publish_, msg->buttons);
    // Convert the joystick message to Twist and publish
    if (convertJoyToCmd(msg->axes, msg->buttons, twist_msg, velocity_scale_))  // Pass velocity_scale_
    {
      twist_msg->header.frame_id = frame_to_publish_;
      twist_msg->header.stamp = this->now();
      twist_pub_->publish(std::move(twist_msg));
    }
  }

private:
  void updateCmdFrame(std::string& frame_name, const std::vector<int>& buttons) const
  {
    auto current_time = this->now();
    if (buttons[CHANGE_VIEW])
    {
      // Check if enough time has passed since last switch
      if ((current_time - last_frame_switch_time_).seconds() >= frame_switch_cooldown_)
      {
        RCLCPP_INFO(this->get_logger(), "changing view");
        if (frame_name == eef_frame_id_)
        {
          RCLCPP_INFO(this->get_logger(), "Switching frame from %s to %s", frame_name.c_str(), base_frame_id_.c_str());
          frame_name = base_frame_id_;
          last_frame_switch_time_ = current_time;
        }
        else if (frame_name == base_frame_id_)
        {
          RCLCPP_INFO(this->get_logger(), "Switching frame from %s to %s", frame_name.c_str(), eef_frame_id_.c_str());
          frame_name = eef_frame_id_;
          last_frame_switch_time_ = current_time;
        }
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

  std::string frame_to_publish_;
  std::string eef_frame_id_;
  std::string base_frame_id_;
  double frame_switch_cooldown_;
  double velocity_scale_;  // Add velocity scale member variable
  mutable rclcpp::Time last_frame_switch_time_;
};  // class JoyToServoPub

}  // namespace moveit_teleop

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_teleop::JoyToServoPub)
