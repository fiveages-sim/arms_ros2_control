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
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <string>
#include <vector>
#include <map>
#include <memory>

// Constants for topics and frames
const std::string DEFAULT_JOY_TOPIC = "/joy";
const std::string DEFAULT_TWIST_TOPIC = "/servo_node/delta_twist_cmds";
const std::string DEFAULT_EEF_FRAME_ID = "panda_hand";
const std::string DEFAULT_BASE_FRAME_ID = "panda_link0";
const double DEFAULT_VELOCITY_SCALE = 1.0;  // Add default velocity scale

// Gripper control constants
const int DEFAULT_GRIPPER_TRIGGER_BUTTON = 2;
const double DEFAULT_GRIPPER_DEBOUNCE_TIME = 1.0;
const double DEFAULT_GRIPPER_MAX_EFFORT = -1.0;
const std::string DEFAULT_GRIPPER_CONTROLLER_NAME = "hand_controller";

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
    this->declare_parameter("frame_switch_cooldown", 0.5);  // 0.5 seconds cooldown
    this->declare_parameter("velocity_scale", DEFAULT_VELOCITY_SCALE);  // Add velocity scale parameter

    // Gripper control parameters
    this->declare_parameter("gripper_trigger_button", DEFAULT_GRIPPER_TRIGGER_BUTTON);
    this->declare_parameter("gripper_debounce_time", DEFAULT_GRIPPER_DEBOUNCE_TIME);
    this->declare_parameter("gripper_max_effort", DEFAULT_GRIPPER_MAX_EFFORT);
    this->declare_parameter("gripper_controller_name", DEFAULT_GRIPPER_CONTROLLER_NAME);

    // Get parameter values
    std::string joy_topic = this->get_parameter("joy_topic").as_string();
    std::string twist_topic = this->get_parameter("twist_topic").as_string();
    frame_switch_cooldown_ = this->get_parameter("frame_switch_cooldown").as_double();
    velocity_scale_ = this->get_parameter("velocity_scale").as_double();  // Get velocity scale value

    // Get gripper parameters
    gripper_trigger_button_ = this->get_parameter("gripper_trigger_button").as_int();
    gripper_debounce_time_ = this->get_parameter("gripper_debounce_time").as_double();
    gripper_max_effort_ = this->get_parameter("gripper_max_effort").as_double();
    gripper_controller_name_ = this->get_parameter("gripper_controller_name").as_string();
    gripper_open_ = true;  // Start with gripper open
    last_gripper_toggle_time_ = this->now();

    // Try to read frame configurations from servo parameters
    // First try to read from moveit_servo namespace
    std::string servo_namespace = "moveit_servo";

    this->declare_parameter(servo_namespace + ".ee_frame_name", DEFAULT_EEF_FRAME_ID);
    this->declare_parameter(servo_namespace + ".robot_link_command_frame", DEFAULT_BASE_FRAME_ID);
    eef_frame_id_ = this->get_parameter(servo_namespace + ".ee_frame_name").as_string();
    base_frame_id_ = this->get_parameter(servo_namespace + ".robot_link_command_frame").as_string();

    this->declare_parameter(servo_namespace + ".gripper_open", 0.08);
    this->declare_parameter(servo_namespace + ".gripper_close", 0.0);
    gripper_open_position_ = this->get_parameter(servo_namespace + ".gripper_open").as_double();
    gripper_close_position_ = this->get_parameter(servo_namespace + ".gripper_close").as_double();

    // Initialize frame_to_publish_ to base frame
    frame_to_publish_ = base_frame_id_;
    last_frame_switch_time_ = this->now();

    // Setup pub/sub
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        joy_topic, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) { return joyCB(msg); });

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic, rclcpp::SystemDefaultsQoS());

    // Setup gripper action client
    std::string gripper_action_topic = "/" + gripper_controller_name_ + "/gripper_cmd";
    gripper_action_client_ = rclcpp_action::create_client<control_msgs::action::GripperCommand>(
        this, gripper_action_topic);

    // Create a service client to start the ServoNode
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

    // Initialize gripper (open it at startup)
    if (gripper_action_client_->wait_for_action_server(std::chrono::seconds(5)))
    {
      openGripper();
      RCLCPP_INFO(this->get_logger(), "Gripper initialized and opened");
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Gripper action server not available, gripper control disabled");
    }

    // Log the final configuration
    RCLCPP_INFO(this->get_logger(), "MoveIt Teleop initialized with:");
    RCLCPP_INFO(this->get_logger(), "  - EEF Frame: %s", eef_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Base Frame: %s", base_frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Gripper Open Position: %.2f", gripper_open_position_);
    RCLCPP_INFO(this->get_logger(), "  - Gripper Close Position: %.2f", gripper_close_position_);
    RCLCPP_INFO(this->get_logger(), "  - Gripper Controller: %s", gripper_controller_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "  - Gripper Trigger Button: %d", gripper_trigger_button_);
  }

  void joyCB(const sensor_msgs::msg::Joy::ConstSharedPtr& msg)
  {
    // Handle gripper control
    handleGripperControl(msg->buttons);

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
  void handleGripperControl(const std::vector<int>& buttons)
  {
    auto now = this->now();
    
    // Check if gripper trigger button is pressed
    if (gripper_trigger_button_ >= 0 && 
        buttons.size() > static_cast<size_t>(gripper_trigger_button_) && 
        buttons[gripper_trigger_button_] == 1)
    {
      // Check debounce time
      if ((now - last_gripper_toggle_time_).seconds() >= gripper_debounce_time_)
      {
        // Toggle gripper state
        if (gripper_open_)
        {
          closeGripper();
          gripper_open_ = false;
          RCLCPP_INFO(this->get_logger(), "Gripper closed by joystick button %d", gripper_trigger_button_ + 1);
        }
        else
        {
          openGripper();
          gripper_open_ = true;
          RCLCPP_INFO(this->get_logger(), "Gripper opened by joystick button %d", gripper_trigger_button_ + 1);
        }
        last_gripper_toggle_time_ = now;
      }
    }
  }

  void openGripper()
  {
    sendGripperCommand(gripper_open_position_);
  }

  void closeGripper()
  {
    sendGripperCommand(gripper_close_position_);
  }

  void sendGripperCommand(double position)
  {
    if (!gripper_action_client_->action_server_is_ready())
    {
      RCLCPP_WARN(this->get_logger(), "Gripper action server not ready");
      return;
    }

    auto goal_msg = control_msgs::action::GripperCommand::Goal();
    goal_msg.command.position = position;
    goal_msg.command.max_effort = gripper_max_effort_;

    auto send_goal_options = rclcpp_action::Client<control_msgs::action::GripperCommand>::SendGoalOptions();
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
  }

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
  rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr gripper_action_client_;

  std::string frame_to_publish_;
  std::string eef_frame_id_;
  std::string base_frame_id_;
  double frame_switch_cooldown_;
  double velocity_scale_;  // Add velocity scale member variable
  mutable rclcpp::Time last_frame_switch_time_;

  // Gripper control members
  int gripper_trigger_button_;
  double gripper_open_position_;
  double gripper_close_position_;
  double gripper_debounce_time_;
  double gripper_max_effort_;
  std::string gripper_controller_name_;
  bool gripper_open_;
  rclcpp::Time last_gripper_toggle_time_;
};  // class JoyToServoPub

}  // namespace moveit_teleop

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(moveit_teleop::JoyToServoPub)
