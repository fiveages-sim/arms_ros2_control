//
// Created for Arms ROS2 Control - ArmsTargetManager Node
//

#include <rclcpp/rclcpp.hpp>
#include "arms_target_manager/ArmsTargetManager.h"
#include "arms_target_manager/ControlInputHandler.h"
#include "arms_target_manager/VRInputHandler.h"

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
    std::string control_base_frame = node->declare_parameter("control_base_frame", "world");
    std::string marker_fixed_frame = node->declare_parameter("marker_fixed_frame", "base_link");
    double linear_scale = node->declare_parameter("linear_scale", 0.005);
    double angular_scale = node->declare_parameter("angular_scale", 0.05);

    double vr_update_rate = node->declare_parameter("vr_update_rate", 500.0);
    bool enable_vr = node->declare_parameter("enable_vr", true);

    RCLCPP_INFO(node->get_logger(), 
               "Starting ArmsTargetManager with topic_prefix: %s, dual_arm_mode: %s, control_base_frame: %s, marker_fixed_frame: %s",
               topic_prefix.c_str(), 
               dual_arm_mode ? "true" : "false",
               control_base_frame.c_str(),
               marker_fixed_frame.c_str());
    RCLCPP_INFO(node->get_logger(), 
               "Control scales: linear=%.3f, angular=%.3f (deadzone handled at input source)",
               linear_scale, angular_scale);
               
    RCLCPP_INFO(node->get_logger(), 
               "VR control: enabled=%s, update_rate=%.1f Hz",
               enable_vr ? "true" : "false", vr_update_rate);

    try
    {
        // 创建ArmsTargetManager
        auto target_manager = std::make_unique<ArmsTargetManager>(
            node, topic_prefix, dual_arm_mode, control_base_frame, marker_fixed_frame);

        // 初始化
        target_manager->initialize();

        // 创建ControlInputHandler
        auto control_handler = std::make_unique<ControlInputHandler>(
            node, target_manager.get(), linear_scale, angular_scale);

        // 创建VRInputHandler（如果启用）
        std::unique_ptr<VRInputHandler> vr_handler = nullptr;
        if (enable_vr) {
            vr_handler = std::make_unique<VRInputHandler>(
                node, target_manager.get(), vr_update_rate);
        }

        // 创建control input订阅器，同时处理两个回调
        auto control_subscription = node->create_subscription<arms_ros2_control_msgs::msg::Inputs>(
            "control_input", 10, 
            [control_handler_ptr = control_handler.get(), target_manager_ptr = target_manager.get()](const arms_ros2_control_msgs::msg::Inputs::ConstSharedPtr msg) {
                // 先处理ControlInputHandler
                if (control_handler_ptr) {
                    control_handler_ptr->processControlInput(msg);
                }
                // 再处理ArmsTargetManager的状态更新
                if (target_manager_ptr) {
                    target_manager_ptr->controlInputCallback(msg);
                }
            });

        RCLCPP_INFO(node->get_logger(), "ArmsTargetManager is ready!");
        RCLCPP_INFO(node->get_logger(), "Use RViz to interact with the markers.");
        RCLCPP_INFO(node->get_logger(), "Control input handler is ready for joystick/keyboard control!");
        if (enable_vr) {
            RCLCPP_INFO(node->get_logger(), "VR input handler is ready for VR control!");
        }

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
