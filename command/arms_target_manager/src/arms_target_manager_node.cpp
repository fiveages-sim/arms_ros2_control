//
// Created for Arms ROS2 Control - ArmsTargetManager Node
//

#include <rclcpp/rclcpp.hpp>
#include "arms_target_manager/ArmsTargetManager.h"

using namespace arms_ros2_control::command;

int main(int argc, char** argv)
{
    // 初始化ROS2
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = rclcpp::Node::make_shared("arms_target_manager_node");

    // 获取参数
    std::string topic_prefix = node->declare_parameter("topic_prefix", "arm_controller");
    bool dual_arm_mode = node->declare_parameter("dual_arm_mode", false);
    std::string frame_id = node->declare_parameter("frame_id", "world");

    RCLCPP_INFO(node->get_logger(), 
               "Starting ArmsTargetManager with topic_prefix: %s, dual_arm_mode: %s, frame_id: %s",
               topic_prefix.c_str(), 
               dual_arm_mode ? "true" : "false",
               frame_id.c_str());

    try
    {
        // 创建ArmsTargetManager
        auto target_manager = std::make_unique<ArmsTargetManager>(
            node, topic_prefix, dual_arm_mode, frame_id);

        // 初始化
        target_manager->initialize();

        RCLCPP_INFO(node->get_logger(), "ArmsTargetManager is ready!");
        RCLCPP_INFO(node->get_logger(), "Use RViz to interact with the markers.");

        // 保持节点运行
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "Exception in ArmsTargetManager: %s", e.what());
        return 1;
    }

    // 清理
    rclcpp::shutdown();
    return 0;
}
