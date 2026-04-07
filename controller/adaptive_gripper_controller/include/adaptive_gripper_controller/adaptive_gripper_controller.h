// C++ standard
#pragma once
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

namespace adaptive_gripper_controller
{
    /**
     * \brief 简单的夹爪控制器，实现基本的位置读取和输出功能
     */
    class AdaptiveGripperController : public controller_interface::ControllerInterface
    {
    public:
        AdaptiveGripperController();

        /**
         * @brief 命令接口配置
         */
        [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        /**
         * @brief 状态接口配置
         */
        [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        /**
         * @brief 控制器更新函数
         */
        controller_interface::return_type update(
            const rclcpp::Time& time, const rclcpp::Duration& period) override;

        /**
         * @brief 控制器初始化
         */
        controller_interface::CallbackReturn on_init() override;

        /**
         * @brief 控制器配置
         */
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State& previous_state) override;

        /**
         * @brief 控制器激活
         */
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& previous_state) override;

        /**
         * @brief 控制器停用
         */
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State& previous_state) override;

    private:
        // 夹爪关节名称
        std::string joint_name_ = "gripper_joint";
        
        // 控制器名称（用于话题命名）
        std::string controller_name_;

        // 目标位置
        double target_position_ = 0.0;

        // 夹爪状态
        int32_t gripper_target_ = 0;

        // 关节限制
        double joint_upper_limit_ = 0.1;
        double joint_lower_limit_ = 0.0;
        bool limits_initialized_ = false;

        // 配置的初始值（从 robot_description 读取，默认为 0.0）
        double config_initial_position_ = 0.0;
        
        // 夹爪的关闭和打开位置（初始化时计算）
        double closed_position_ = 0.0;
        double open_position_ = 0.0;

        // 是否使用力反馈接口
        bool use_effort_interface_ = true;
        
        // 力反馈阈值
        double force_threshold_ = 0.1;

        // 力反馈触发后的移动距离比例 (0.0-1.0)
        double force_feedback_ratio_ = 0.5;

        // 夹取任务状态跟踪
        bool force_threshold_triggered_ = false;
        
        // 是否有 effort 接口（运行时自动检测）
        bool has_effort_interface_ = false;
        
        // 控制模式：true=直接位置控制（无力反馈），false=开关/百分比控制（有力反馈）
        bool direct_position_mode_ = false;

        // ---------------------------------------------------------------
        // 跨线程命令收件箱（原子变量）
        //
        // ros2_control 中 update() 运行在 RT 线程，订阅回调运行在 executor
        // 线程。所有命令通道只写各自的原子"收件箱"，update() 用 exchange()
        // 原子消费，从而使 target_position_ / direct_position_mode_ 等状态
        // 变量完全归 update() 独占，消除数据竞争。
        //
        // 哨兵值含义：
        //   pending_direct_position_  : quiet_NaN = 无待处理命令
        //   pending_target_switch_    : -1         = 无待处理命令 (0=close, 1=open)
        //   pending_percent_command_  : -1.0       = 无待处理命令
        // ---------------------------------------------------------------
        std::atomic<double>  pending_direct_position_{std::numeric_limits<double>::quiet_NaN()};
        std::atomic<int32_t> pending_target_switch_{-1};
        std::atomic<double>  pending_percent_command_{-1.0};

        // 需要的状态接口类型列表（从 hardware 查询）
        std::vector<std::string> available_state_interface_types_;

        // 硬件接口结构体
        struct GripperInterfaces
        {
            // 命令接口
            std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
            position_command_interface_;

            // 状态接口
            std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
            position_state_interface_;
            std::optional<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
            effort_state_interface_;

            void clear()
            {
                position_command_interface_ = std::nullopt;
                position_state_interface_ = std::nullopt;
                effort_state_interface_ = std::nullopt;
            }
        };

        GripperInterfaces gripper_interfaces_;

        // ROS订阅器
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr direct_position_subscription_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr target_command_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_percent_subscription_;

        // 解析robot description获取关节限制
        void parse_joint_limits(const std::string& robot_description);

        // 解析robot description获取初始值，并计算夹爪位置
        void parse_initial_value(const std::string& robot_description);
    };
} // namespace adaptive_gripper_controller
