//
// HandCommandBus 实现。
//
// 关键点：
//   * 发布器按需创建：某个控制器从未被命令过，就不会广告对应话题；
//   * 名字解析失败（控制器列表为空或编号越界）时静默丢弃，由调用方决定是否提示。
//
// 注意：
//   百分比命令会先夹到 [0, 1]；不要在这里加去抖或接管逻辑，
//   那是各输入源（例如 VR 的三段式接管）自己的职责。
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

    /**
     * @brief target 编号 → 控制器名
     *
     * 只配了一个控制器时不分左右；多于一个时按 1 为起始编号。
     * 编号越界返回空字符串，调用方应直接丢弃该命令。
     */
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

    /**
     * @brief 左右侧 → 控制器名（第 0 个为左）
     *
     * 少于两个控制器时两侧都回退到第 0 个，保证单手配置仍能响应命令。
     */
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

    /**
     * @brief 取（必要时创建）某控制器的开关命令发布器
     *
     * 按需创建，避免未接入的手部也广告一个话题。
     */
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

    /**
     * @brief 取（必要时创建）某控制器的百分比发布器
     */
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

    /**
     * @brief 发送开关命令
     * @param controller 控制器名；为空时丢弃
     * @param open true=张开，false=闭合
     */
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

    /**
     * @brief 发送百分比命令
     * @param controller 控制器名；为空时丢弃
     * @param percent 0=完全闭合，1=完全张开，越界值会被夹到合法区间
     */
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
