//
// ControlInputHandler - 手柄 / 键盘输入的适配层。
//
// 作用：
//   把 control_input（arms_ros2_control_msgs/Inputs）里的归一化摇杆量
//   （-1~1）换成带单位的机器人命令，并把手部命令分流出去。
//   设备侧不需要知道 m/s、rad/s，也不需要知道该手臂此刻能不能接受速度。
//
// 输入：control_input
//   x / y / z         归一化平移量
//   roll / pitch / yaw 归一化旋转量
//   target            1=左臂，2=右臂
//   hand_command      手部命令（0/1 为开关，其它为百分比）
//
// 输出：
//   <side>_target/twist  速度命令，已乘上 linear_scale / angular_scale
//   <controller>/target_command、/target_percent  经由 HandCommandBus
//
// 注意：
//   本类只发速度，不积分位置、不发布绝对位姿；
//   速度不参与 TargetRouter 的控制权仲裁，但被 VR / 拖动占用时会整条丢弃。
//
#pragma once


#include <memory>
#include <vector>
#include <string>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <arms_ros2_control_msgs/msg/inputs.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include "arms_target_manager/core/HandCommandBus.h"

namespace arms_ros2_control::command
{
    // 前向声明
    class ArmsTargetManager;

    /**
     * ControlInputHandler - 控制输入处理器
     *
     * 接收 control_input (Inputs)，适配为 left/right_target/twist 速度命令；
     * hand_command 仍转发到夹爪控制器。Marker 仅做可视化，不再发绝对 Pose。
     *
     * linear_scale / angular_scale 单位为最大速度：m/s、rad/s（满杆）。
     */
    class ControlInputHandler
    {
    public:
        /**
         * @brief 构造控制输入处理器
         * @param node ROS 节点，用于建发布器与打日志
         * @param targetManager 目标管理器，用于询问某臂能否接受速度命令
         * @param linearScale 摇杆满杆对应的线速度 [m/s]
         * @param angularScale 摇杆满杆对应的角速度 [rad/s]
         * @param controlInputRate 输入标称频率 [Hz]，仅用于日志与限幅打印
         * @param handControllers 手部/夹爪控制器名列表，交给 HandCommandBus 解析左右
         */
        ControlInputHandler(
            rclcpp::Node::SharedPtr node,
            ArmsTargetManager* targetManager,
            double linearScale = 0.1,
            double angularScale = 0.25,
            double controlInputRate = 50.0,
            const std::vector<std::string>& handControllers = {});

        ~ControlInputHandler() = default;

        /**
         * @brief 处理一帧 control_input 消息
         *
         * 输入是归一化摇杆量，按比例换算成 Twist 发到对应手臂的速度话题；
         * 若该臂正被 VR 或手动拖动占用，则丢弃速度但照常转发手部命令。
         *
         * @param msg 摇杆/键盘输入（x/y/z 平移，roll/pitch/yaw 旋转，hand_command 手部命令）
         */
        void processControlInput(arms_ros2_control_msgs::msg::Inputs::ConstSharedPtr msg);

    private:
        /**
         * @brief 把手部命令转发给对应控制器
         *
         * 0/1 精确值走开关接口，其余走百分比接口，避免中途值被当成开关。
         *
         * @param target 目标编号（1=左手，2=右手，其它按左手处理）
         * @param hand_command 手部命令值，0~1；非有限值时忽略
         */
        void processHandCommand(int32_t target, float hand_command);

        /**
         * @brief 按目标编号解析手部控制器名
         * @param target 目标编号（1=左手，2=右手）
         * @return 控制器名；未配置时返回空字符串
         */
        std::string resolveHandControllerName(int32_t target) const;

        rclcpp::Node::SharedPtr node_;
        ArmsTargetManager* target_manager_;

        double linear_scale_;
        double angular_scale_;
        double control_input_rate_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr left_twist_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr right_twist_publisher_;

        std::vector<std::string> hand_controllers_;
        HandCommandBus hand_bus_;
    };
} // namespace arms_ros2_control::command
