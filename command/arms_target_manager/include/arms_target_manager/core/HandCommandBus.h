//
// HandCommandBus - 手部（夹爪/灵巧手）命令的唯一出口。
//
// VR 和摇杆/键盘以前各自维护一套“控制器名 → 发布器”的映射，
// 现在统一放这里，避免同一条手被两套逻辑并发写入。
//
#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>

#include "arms_target_manager/core/TeleopTypes.h"

namespace arms_ros2_control::command
{
    class HandCommandBus
    {
    public:
        HandCommandBus(rclcpp::Node::SharedPtr node,
                       std::vector<std::string> controllers);

        [[nodiscard]] bool empty() const { return controllers_.empty(); }
        [[nodiscard]] const std::vector<std::string>& controllers() const
        {
            return controllers_;
        }

        /** control_input 里的 target 编号 → 控制器名（多于一个时按 1 开始编号）。 */
        [[nodiscard]] std::string controllerFor(int32_t target) const;

        /** 左右侧 → 控制器名（按列表顺序，第 0 个为左）。 */
        [[nodiscard]] std::string controllerForSide(Side side) const;

        /** 开关命令：open=true 张开，false 闭合。 */
        void publishSwitch(const std::string& controller, bool open);

        /** 百分比命令：0=完全闭合，1=完全张开。 */
        void publishPercent(const std::string& controller, double percent);

    private:
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr switchPublisher(
            const std::string& controller);
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr percentPublisher(
            const std::string& controller);

        rclcpp::Node::SharedPtr node_;
        std::vector<std::string> controllers_;
        std::map<std::string, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr>
            switch_publishers_;
        std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>
            percent_publishers_;
    };

} // namespace arms_ros2_control::command
