//
// Created for Arms ROS2 Control - ControlInputHandler
//

#ifndef CONTROL_INPUT_HANDLER_H
#define CONTROL_INPUT_HANDLER_H

#include <memory>
#include <string>
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
     * 支持线性缩放和角度缩放，以及死区处理
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
         * @param deadzone 死区阈值，默认为0.1
         */
        ControlInputHandler(
            rclcpp::Node::SharedPtr node,
            ArmsTargetManager* targetManager,
            double linearScale = 0.005,
            double angularScale = 0.05,
            double deadzone = 0.1);

        ~ControlInputHandler() = default;

        /**
         * 处理控制输入并更新ArmsTargetManager
         * @param msg 控制输入消息
         */
        void processControlInput(const arms_ros2_control_msgs::msg::Inputs::ConstSharedPtr msg);


        /**
         * 设置线性缩放因子
         * @param scale 缩放因子
         */
        void setLinearScale(double scale);

        /**
         * 设置角度缩放因子
         * @param scale 缩放因子
         */
        void setAngularScale(double scale);

        /**
         * 设置死区阈值
         * @param deadzone 死区阈值
         */
        void setDeadzone(double deadzone);


    private:
        /**
         * 应用死区过滤
         * @param value 输入值
         * @param deadzone 死区阈值
         * @return 过滤后的值
         */
        double applyDeadzone(double value, double deadzone) const;


        // ROS节点和TargetManager
        rclcpp::Node::SharedPtr node_;
        ArmsTargetManager* target_manager_;

        // 控制参数
        double linear_scale_;
        double angular_scale_;
        double deadzone_;
    };

} // namespace arms_ros2_control::command

#endif // CONTROL_INPUT_HANDLER_H
