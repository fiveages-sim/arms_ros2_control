//
// HandCommandBus 实现。
//
#include "arms_target_manager/core/HandCommandBus.h"

#include <algorithm>
#include <utility>

namespace arms_ros2_control::command
{
    HandCommandBus::HandCommandBus(
        rclcpp::Node::SharedPtr node, std::vector<std::string> controllers)
        : node_(std::move(node))
          , controllers_(std::move(controllers))
    {
    }

    std::string HandCommandBus::controllerFor(int32_t target) const
    {
        if (controllers_.empty())
        {
            return "";
        }
        if (controllers_.size() == 1)
        {
            return controllers_.front();
        }
        const auto index = static_cast<size_t>(std::max(1, target) - 1);
        return index < controllers_.size() ? controllers_[index] : "";
    }

    std::string HandCommandBus::controllerForSide(Side side) const
    {
        if (controllers_.empty())
        {
            return "";
        }
        const size_t index = side == Side::Left ? 0 : 1;
        if (index < controllers_.size())
        {
            return controllers_[index];
        }
        return controllers_.front();
    }

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr HandCommandBus::switchPublisher(
        const std::string& controller)
    {
        auto& publisher = switch_publishers_[controller];
        if (!publisher)
        {
            publisher = node_->create_publisher<std_msgs::msg::Int32>(
                controller + "/target_command", 10);
        }
        return publisher;
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr HandCommandBus::percentPublisher(
        const std::string& controller)
    {
        auto& publisher = percent_publishers_[controller];
        if (!publisher)
        {
            publisher = node_->create_publisher<std_msgs::msg::Float64>(
                controller + "/target_percent", 10);
        }
        return publisher;
    }

    void HandCommandBus::publishSwitch(const std::string& controller, bool open)
    {
        if (controller.empty())
        {
            return;
        }
        std_msgs::msg::Int32 command;
        command.data = open ? 1 : 0;
        switchPublisher(controller)->publish(command);
    }

    void HandCommandBus::publishPercent(const std::string& controller, double percent)
    {
        if (controller.empty())
        {
            return;
        }
        std_msgs::msg::Float64 command;
        command.data = std::clamp(percent, 0.0, 1.0);
        percentPublisher(controller)->publish(command);
    }

} // namespace arms_ros2_control::command
