#include "aubo_ros2_control/bluedot_force_sensor.h"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace aubo_ros2_control
{
class BlueDotForceSensorNode : public rclcpp::Node
{
public:
  BlueDotForceSensorNode()
  : Node("bluedot_force_sensor_node")
  {
    const auto sensor_ip = declare_parameter<std::string>("sensor_ip", "192.168.0.20");
    const auto sensor_port = static_cast<uint16_t>(declare_parameter<int>("sensor_port", 49152));
    const auto command = static_cast<uint16_t>(declare_parameter<int>("command", 2));
    const auto num_samples = static_cast<uint32_t>(declare_parameter<int>("num_samples", 1));
    const auto zero_sample_count = declare_parameter<int>("zero_sample_count", 10);
    const auto scale = declare_parameter<double>("scale", 1000000.0);
    const auto publish_topic = declare_parameter<std::string>("publish_topic", "/ft_sensor_wrench");
    const auto frame_id = declare_parameter<std::string>("frame_id", "ft_sensor");
    const int poll_period_ms = declare_parameter<int>("poll_period_ms", 10);

    frame_id_ = frame_id;
    sensor_ = std::make_unique<BlueDotForceSensor>(
      sensor_ip, sensor_port, command, num_samples, zero_sample_count, scale);
    publisher_ = create_publisher<geometry_msgs::msg::WrenchStamped>(publish_topic, 10);

    std::string error_message;
    if (!sensor_->connect(&error_message)) {
      RCLCPP_ERROR(get_logger(), "%s", error_message.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "BlueDot force sensor configured for %s:%d, publishing to %s",
        sensor_ip.c_str(),
        static_cast<int>(sensor_port),
        publish_topic.c_str());
    }

    timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max(1, poll_period_ms)),
      std::bind(&BlueDotForceSensorNode::pollSensor, this));
  }

private:
  void pollSensor()
  {
    if (!sensor_) {
      return;
    }

    BlueDotForceData data;
    bool zero_ready = false;
    std::string error_message;
    if (!sensor_->read(data, zero_ready, &error_message)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "BlueDot read failed: %s",
        error_message.c_str());
      return;
    }

    if (!zero_ready) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "BlueDot zero calibration in progress...");
      return;
    }

    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.wrench.force.x = data.fx;
    msg.wrench.force.y = data.fy;
    msg.wrench.force.z = data.fz;
    msg.wrench.torque.x = data.tx;
    msg.wrench.torque.y = data.ty;
    msg.wrench.torque.z = data.tz;
    publisher_->publish(msg);
  }

  std::string frame_id_;
  std::unique_ptr<BlueDotForceSensor> sensor_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace aubo_ros2_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<aubo_ros2_control::BlueDotForceSensorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
