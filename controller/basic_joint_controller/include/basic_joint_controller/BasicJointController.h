//
// Created for Basic Joint Controller
//

#pragma once

#include <memory>
#include <vector>
#include <string>
#include <unordered_map>
#include <limits>
#include <algorithm>

#include <controller_interface/controller_interface.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

// Use common FSM and interfaces from arms_controller_common
#include <arms_controller_common/FSM/FSMState.h>
#include <arms_controller_common/FSM/StateHome.h>
#include <arms_controller_common/FSM/StateHold.h>
#include <arms_controller_common/CtrlInterfaces.h>

namespace basic_joint_controller
{
    // Use FSM types from arms_controller_common
    using FSMState = arms_controller_common::FSMState;
    using FSMStateName = arms_controller_common::FSMStateName;
    using FSMMode = arms_controller_common::FSMMode;
    using CtrlInterfaces = arms_controller_common::CtrlInterfaces;

    // Forward declarations
    class StateHold; // Extended StateHold with MOVEJ transition
    class StateMoveJ; // Extended StateMoveJ for basic_joint_controller

    struct FSMStateList
    {
        std::shared_ptr<FSMState> invalid;
        std::shared_ptr<arms_controller_common::StateHome> home; // Use common StateHome
        std::shared_ptr<StateHold> hold; // Extended StateHold for basic_joint_controller
        std::shared_ptr<StateMoveJ> movej; // Extended StateMoveJ for basic_joint_controller
    };

    class BasicJointController final : public controller_interface::ControllerInterface
    {
    public:
        BasicJointController() = default;
        ~BasicJointController() override = default;

        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
        controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
        controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
        controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    private:
        std::shared_ptr<FSMState> getNextState(FSMStateName stateName) const;

        // Hardware parameters
        std::string command_prefix_;
        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        // Control interfaces
        CtrlInterfaces ctrl_interfaces_;

        // State machine
        FSMStateList state_list_;
        std::shared_ptr<FSMState> current_state_;
        std::shared_ptr<FSMState> next_state_;
        FSMStateName next_state_name_{FSMStateName::INVALID};
        FSMMode mode_{FSMMode::NORMAL};

        // Interface mapping
        // ⚠️ command_interface_map_[name] 用的是 operator[] ⇒ yaml 的 `command_interfaces`
        //    里写了某类口、而这里没有对应项时，会插入 nullptr 再被解引用（段错误）。
        //    两边必须同步改。
        std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
                           *>
        command_interface_map_ = {
            {"position", &ctrl_interfaces_.joint_position_command_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_command_interface_}
        };

        // ── 参考速度输出（可选）：给驱动内部阻抗层（电流环 / CSV）当 q̇_d 用 ─────────
        // 用**位置命令的差分**当参考速度写进 <joint>/velocity 命令口。
        // 只有在 yaml 的 command_interfaces 里含 velocity（真的 claim 到了该口）时才写；
        // 不含就完全不碰，行为与从前逐位一致。
        // 为什么要它：分体模式（split_body）下腰部/头部归 body_joint_controller /
        //   head_joint_controller，**没有 ocs2_wbc_controller** 去写 <joint>/velocity，
        //   而驱动内部阻抗律的 Kd·(q̇_d−q̇) 与摩擦前馈都要 q̇_d。
        //   没有 q̇_d 时 Kd 项退化成纯阻尼 ⇒ 跟轨迹时有 (Kd/Kp)·q̇ 的稳态滞后
        //   （本机 Kd/Kp=15000/60000 ⇒ 0.2 rad/s 时约 2.9°）。
        void publishReferenceVelocities(const rclcpp::Duration& period);
        std::vector<double> prev_position_command_;
        double velocity_command_max_rad_s_{2.0};

        std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>*>
        state_interface_map_ = {
            {"position", &ctrl_interfaces_.joint_position_state_interface_},
            {"velocity", &ctrl_interfaces_.joint_velocity_state_interface_}
        };

        // Controller name
        std::string controller_name_;

        // Target command parameters
        bool target_command_enabled_{false};
        int32_t target_command_close_config_{1};
        int32_t target_command_open_config_{0};
        std::vector<std::string> target_command_hold_joint_names_;
        std::vector<bool> target_command_hold_mask_;

        void buildTargetCommandHoldMask();
        void applyTargetCommandHoldOverrides(std::vector<double>& target) const;

        // ROS subscriptions
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr fsm_command_subscription_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_joint_position_subscription_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr target_command_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_percent_subscription_;

        // body motion
        bool waist_lifting_enabled_{false};
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr waist_lifting_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr waist_lifting_pose_relative_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr waist_lifting_pose_absolute_subscription_;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::string waist_absolute_source_frame_{"base_footprint"};
        std::string waist_absolute_target_frame_{"body_base"};

        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr waist_lifting_command_subscription_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr waist_turning_command_subscription_;
    };
}
