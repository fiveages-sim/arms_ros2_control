//
// Created for OCS2 Arm Controller - StateOCS2 Implementation
//

#include "ocs2_arm_controller/FSM/StateOCS2.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <pinocchio/algorithm/kinematics.hpp>

namespace ocs2::mobile_manipulator
{
    StateOCS2::StateOCS2(CtrlInterfaces& ctrl_interfaces,
                         const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                         const std::shared_ptr<CtrlComponent>& ctrl_comp)
        : FSMState(FSMStateName::OCS2, "OCS2"), ctrl_comp_(ctrl_comp), ctrl_interfaces_(ctrl_interfaces), node_(node)
    {
        // 获取关节名称
        joint_names_ = node_->get_parameter("joints").as_string_array();

        // 获取MPC更新频率
        double mpc_frequency = 0.0;
        const double controller_frequency = ctrl_interfaces_.frequency_;

        // 尝试从参数获取MPC频率
        if (node_->has_parameter("mpc_frequency"))
        {
            mpc_frequency = node_->get_parameter("mpc_frequency").as_double();
            RCLCPP_INFO(node_->get_logger(), "MPC frequency from parameter: %.1f Hz", mpc_frequency);
        }

        // 综合判断MPC频率是否有效

        if (const bool mpc_frequency_valid = mpc_frequency > 0.0 && mpc_frequency <= controller_frequency; !
            mpc_frequency_valid)
        {
            // 使用控制器频率的一半作为默认值
            const double original_frequency = mpc_frequency;
            mpc_frequency = controller_frequency / 2.0;

            if (original_frequency <= 0.0)
            {
                RCLCPP_WARN(node_->get_logger(),
                            "Invalid MPC frequency (%.1f Hz), using half of controller frequency: %.1f Hz (controller: %.1f Hz)",
                            original_frequency, mpc_frequency, controller_frequency);
            }
            else if (original_frequency > controller_frequency)
            {
                RCLCPP_WARN(node_->get_logger(),
                            "MPC frequency (%.1f Hz) exceeds controller frequency (%.1f Hz), using half of controller frequency: %.1f Hz",
                            original_frequency, mpc_frequency, controller_frequency);
            }
            else
            {
                RCLCPP_INFO(node_->get_logger(),
                            "MPC frequency not set, using half of controller frequency: %.1f Hz (controller: %.1f Hz)",
                            mpc_frequency, controller_frequency);
            }
        }

        mpc_period_ = 1.0 / mpc_frequency;

        // 使用CtrlComponent的接口
        RCLCPP_INFO(node_->get_logger(), "Using CtrlComponent interface for StateOCS2");

        RCLCPP_INFO(node_->get_logger(), "StateOCS2 initialized successfully");
    }

    void StateOCS2::enter()
    {
        RCLCPP_INFO(node_->get_logger(), "Entering OCS2 state");

        // 重置MPC
        ctrl_comp_->resetMpc();

        // 重置时间
        last_mpc_time_ = node_->now();

        RCLCPP_INFO(node_->get_logger(), "OCS2 state entered successfully");
    }

    void StateOCS2::run(const rclcpp::Time& time, const rclcpp::Duration& period)
    {
        // 检查是否需要更新MPC
        if ((time - last_mpc_time_).seconds() >= mpc_period_)
        {
            // 使用CtrlComponent评估策略
            ctrl_comp_->advanceMpc();

            last_mpc_time_ = time;
        }

        ctrl_comp_->evaluatePolicy(time);
    }

    void StateOCS2::exit()
    {
        RCLCPP_INFO(node_->get_logger(), "Exiting OCS2 state");
        // 清理工作（如果需要）
    }

    FSMStateName StateOCS2::checkChange()
    {
        // 检查控制输入进行状态切换
        switch (ctrl_interfaces_.control_inputs_.command)
        {
        case 2: return FSMStateName::HOLD;
        default: return FSMStateName::OCS2;
        }
    }
} // namespace ocs2::mobile_manipulator
