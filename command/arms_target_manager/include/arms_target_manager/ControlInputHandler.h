//
// Created for Arms ROS2 Control - ControlInputHandler
//

#ifndef CONTROL_INPUT_HANDLER_H
#define CONTROL_INPUT_HANDLER_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <arms_ros2_control_msgs/msg/inputs.hpp>

namespace arms_ros2_control::command
{
    // 前向声明
    class ArmsTargetManager;

    /**
     * ControlInputHandler - 控制输入处理器Wrapper
     * 
     * 接收control input数据，累积位置和旋转变化，通过ArmsTargetManager更新marker位置
     * 支持线性缩放和角度缩放，死区处理在输入源层面完成
     */
    class ControlInputHandler
    {
    public:
        /**
         * 构造函数
         * @param node ROS节点指针
         * @param targetManager ArmsTargetManager指针
         * @param linearScale 线性移动缩放因子，默认为0.005
         * @param angularScale 角度移动缩放因子，默认为0.05
         */
        ControlInputHandler(
            rclcpp::Node::SharedPtr node,
            ArmsTargetManager* targetManager,
            double linearScale = 0.005,
            double angularScale = 0.05);

        ~ControlInputHandler() = default;

        /**
         * 处理控制输入并更新ArmsTargetManager
         * @param msg 控制输入消息
         */
        void processControlInput(arms_ros2_control_msgs::msg::Inputs::ConstSharedPtr msg);



    private:
        // ROS节点和TargetManager
        rclcpp::Node::SharedPtr node_;
        ArmsTargetManager* target_manager_;

        // 控制参数
        double linear_scale_;
        double angular_scale_;
    };

} // namespace arms_ros2_control::command

#endif // CONTROL_INPUT_HANDLER_H
