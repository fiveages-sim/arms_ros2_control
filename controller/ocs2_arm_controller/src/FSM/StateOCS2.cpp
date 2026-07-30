//
// Created for OCS2 Arm Controller - StateOCS2 Implementation
//

#include "ocs2_arm_controller/FSM/StateOCS2.h"

#include <ocs2_controller_common/mpc/MpcExecutionParams.hpp>
#include <ocs2_core/misc/LoadData.h>
#include <pinocchio/algorithm/kinematics.hpp>

namespace ocs2::mobile_manipulator
{
    StateOCS2::StateOCS2(CtrlInterfaces& ctrl_interfaces,
                         const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                         const std::shared_ptr<CtrlComponent>& ctrl_comp)
        : FSMState(FSMStateName::OCS2, "ocs2", ctrl_interfaces), ctrl_comp_(ctrl_comp), ctrl_interfaces_(ctrl_interfaces), node_(node)
    {
        joint_names_ = node_->get_parameter("joints").as_string_array();

        const double controller_frequency = ctrl_interfaces_.frequency_;
        const auto mpc_params = controller_common::computeMpcExecutionParams(
            controller_frequency, node_->get_parameter("mpc_frequency").as_int(), node_->get_logger());
        mpc_period_ = mpc_params.mpc_period_sec;
        thread_sleep_duration_ms_ = mpc_params.thread_sleep_ms;

        const bool selfCollisionEnabled = ctrl_comp_->interface_->isSelfCollisionEnabled();
        if (selfCollisionEnabled)
        {
            const scalar_t minimumDistance = ctrl_comp_->interface_->getSelfCollisionMinimumDistance();
            RCLCPP_INFO(node_->get_logger(),
                "Self-collision enabled: will switch to HOLD when distance <= %.4f m (minimumDistance from config)",
                minimumDistance);
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "Self-collision disabled in config, collision detection not active");
        }

        RCLCPP_INFO(node_->get_logger(), "StateOCS2 initialized successfully");
    }

    StateOCS2::~StateOCS2()
    {
        stopMpcThread();
    }

    void StateOCS2::enter()
    {
        if (ctrl_interfaces_.control_mode_ == ControlMode::MIX)
        {
            if (ctrl_interfaces_.pd_gains_.size() >= 2)
            {
                double kp = ctrl_interfaces_.pd_gains_[0];
                double kd = ctrl_interfaces_.pd_gains_[1];

                RCLCPP_INFO(node_->get_logger(), "Setting OCS2 gains: kp=%.2f, kd=%.2f", kp, kd);

                for (size_t i = 0; i < ctrl_interfaces_.joint_kp_command_interface_.size(); ++i)
                {
                    std::ignore = ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(kp);
                }
                for (size_t i = 0; i < ctrl_interfaces_.joint_kd_command_interface_.size(); ++i)
                {
                    std::ignore = ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(kd);
                }
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "OCS2 gains not configured, using default gains");
            }
        }

        // Ensure previous MPC worker is fully stopped before starting a new one.
        if (!mpc_thread_finished_.load() || mpc_thread_.joinable())
        {
            stopMpcThread();
        }

        ctrl_comp_->resetMpc();
        collision_detected_ = false;
        last_mpc_time_ = node_->now();

        // Kick viz snapshot after reset (viz thread owned by controller activate lifetime).
        ctrl_comp_->requestVisualizationUpdate();
        mpc_thread_finished_ = false;
        mpc_running_ = true;
        mpc_update_requested_ = true;
        mpc_thread_ = std::thread(&StateOCS2::mpcUpdateThread, this);
    }

    void StateOCS2::run(const rclcpp::Time& time, const rclcpp::Duration& /* period */)
    {
        if (!ctrl_comp_->initialPolicyReceived())
        {
            mpc_update_requested_ = true;
            ctrl_comp_->holdLastSentPositions();
            return;
        }

        if (ctrl_comp_->interface_->isSelfCollisionEnabled() && !collision_detected_)
        {
            const scalar_t minimumDistance = ctrl_comp_->interface_->getSelfCollisionMinimumDistance();
            if (ctrl_comp_->checkSelfCollisionOnObservation(minimumDistance))
            {
                collision_detected_ = true;
                RCLCPP_WARN(node_->get_logger(),
                    "Collision detected! Distance <= minimumDistance: %.4f m. Will switch to HOLD state.",
                    minimumDistance);
            }
        }

        if ((time - last_mpc_time_).seconds() >= mpc_period_)
        {
            mpc_update_requested_ = true;
            last_mpc_time_ = time;
        }

        ctrl_comp_->evaluatePolicy(time);
    }

    void StateOCS2::beginExit()
    {
        // RT-safe: only signal MPC worker to stop. Join happens in tryFinishExit().
        // Visualization thread is owned by the controller activate lifetime — do not stop here.
        mpc_update_requested_ = false;
        mpc_running_ = false;
    }

    bool StateOCS2::tryFinishExit()
    {
        // After beginExit, mpc_running_ is already false — wait only on finished_.
        if (!mpc_thread_finished_.load())
        {
            return false;
        }

        if (mpc_thread_.joinable())
        {
            mpc_thread_.join();
        }
        mpc_thread_finished_ = true;

        if (ctrl_comp_)
        {
            ctrl_comp_->clearTrajectoryVisualization();
        }
        RCLCPP_INFO(node_->get_logger(), "OCS2 state exited successfully, MPC thread stopped");
        return true;
    }

    void StateOCS2::exit()
    {
        // Synchronous fallback (destructor / callers that don't use async CHANGE).
        beginExit();
        while (!tryFinishExit())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void StateOCS2::mpcUpdateThread()
    {
        RCLCPP_DEBUG(node_->get_logger(), "MPC update thread started");

        while (mpc_running_.load())
        {
            if (mpc_update_requested_.load())
            {
                try
                {
                    // Only advanceMpc here. Never call getPolicy()/updatePolicy() on this
                    // thread — MRT active policy is not thread-safe (see MRT_BASE.h).
                    ctrl_comp_->advanceMpc();
                    mpc_update_requested_ = false;
                }
                catch (const std::exception& e)
                {
                    RCLCPP_ERROR(node_->get_logger(), "Error in MPC update: %s", e.what());
                    mpc_update_requested_ = false;
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(thread_sleep_duration_ms_));
        }

        mpc_thread_finished_ = true;
        RCLCPP_DEBUG(node_->get_logger(), "MPC update thread stopped");
    }

    void StateOCS2::stopMpcThread()
    {
        mpc_running_ = false;
        mpc_update_requested_ = false;
        if (mpc_thread_.joinable())
        {
            mpc_thread_.join();
        }
        mpc_thread_finished_ = true;
    }

    FSMStateName StateOCS2::checkChange()
    {
        if (collision_detected_)
        {
            ctrl_comp_->publishFsmCommand(2);
            RCLCPP_WARN(node_->get_logger(), "Published fsm_command=2 to stop all controllers");
            return FSMStateName::HOLD;
        }

        switch (ctrl_interfaces_.fsm_command_)
        {
        case 2: return FSMStateName::HOLD;
        default: return FSMStateName::OCS2;
        }
    }
} // namespace ocs2::mobile_manipulator
