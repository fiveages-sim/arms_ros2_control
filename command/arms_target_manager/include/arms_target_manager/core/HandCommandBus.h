//
// HandCommandBus - 手部（夹爪/灵巧手）命令的唯一出口。
//
// 作用：
//   把“控制器名 → 发布器”的映射收拢到一处，对外只给四个入口：
//     controllerFor / controllerForSide  名字解析；
//     publishSwitch / publishPercent     真正发命令。
//
// 输入：控制器名列表（来自 hand_controllers 参数，第 0 个为左）。
// 输出：<controller>/target_command（开关）与 <controller>/target_percent（百分比）。
//
// 注意：
//   以前 VR 和摇杆/键盘各维护一套映射，同一条手可能被两套逻辑并发写入；
//   新增手部输入请调用这里的方法，不要再自己 create_publisher。
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
        /**
         * @brief 构造手部命令出口
         * @param node ROS 节点，用于按需创建发布器
         * @param controllers 手部/夹爪控制器名列表；第 0 个为左，第 1 个为右
         */
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
