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
        // Get joint names
        joint_names_ = node_->get_parameter("joints").as_string_array();

        // Get MPC update frequency
        double mpc_frequency = 0.0;
        const double controller_frequency = ctrl_interfaces_.frequency_;

        // Try to get MPC frequency from parameters
        if (node_->has_parameter("mpc_frequency"))
        {
            mpc_frequency = node_->get_parameter("mpc_frequency").as_double();
            RCLCPP_INFO(node_->get_logger(), "MPC frequency from parameter: %.1f Hz", mpc_frequency);
        }

        // Validate MPC frequency
        if (const bool mpc_frequency_valid = mpc_frequency > 0.0 && mpc_frequency <= controller_frequency; !
            mpc_frequency_valid)
        {
            // Use 1/4 of controller frequency as default to ensure MPC updates are frequent enough
            const double original_frequency = mpc_frequency;
            mpc_frequency = controller_frequency / 4.0;

            if (original_frequency <= 0.0)
            {
                RCLCPP_WARN(node_->get_logger(),
                            "Invalid MPC frequency (%.1f Hz), using 1/4 of controller frequency: %.1f Hz (controller: %.1f Hz)",
                            original_frequency, mpc_frequency, controller_frequency);
            }
            else if (original_frequency > controller_frequency)
            {
                RCLCPP_WARN(node_->get_logger(),
                            "MPC frequency (%.1f Hz) exceeds controller frequency (%.1f Hz), using 1/4 of controller frequency: %.1f Hz",
                            original_frequency, mpc_frequency, controller_frequency);
            }
            else
            {
                RCLCPP_INFO(node_->get_logger(),
                            "MPC frequency not set, using 1/4 of controller frequency: %.1f Hz (controller: %.1f Hz)",
                            mpc_frequency, controller_frequency);
            }
        }

        // Ensure MPC frequency is not too low, minimum frequency is 10Hz
        if (mpc_frequency < 10.0)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "MPC frequency (%.1f Hz) is too low, setting minimum frequency to 10.0 Hz", mpc_frequency);
            mpc_frequency = 10.0;
        }

        mpc_period_ = 1.0 / mpc_frequency;

        // Calculate thread sleep duration based on controller frequency
        // Sleep time is set to 2x controller period to ensure MPC update requests are not missed
        thread_sleep_duration_ms_ = static_cast<int>((2.0 / controller_frequency) * 1000.0);

        // Use CtrlComponent interface
        RCLCPP_INFO(node_->get_logger(), "Using CtrlComponent interface for StateOCS2");
        RCLCPP_INFO(node_->get_logger(), "Thread sleep duration: %d ms (based on controller frequency: %.1f Hz)", 
                    thread_sleep_duration_ms_, controller_frequency);

        RCLCPP_INFO(node_->get_logger(), "StateOCS2 initialized successfully");
    }

    StateOCS2::~StateOCS2()
    {
        // Ensure thread is properly stopped when destructing
        stopMpcThread();
    }

    void StateOCS2::enter()
    {
        RCLCPP_INFO(node_->get_logger(), "Entering OCS2 state");

        // Reset MPC
        ctrl_comp_->resetMpc();

        // Reset time
        last_mpc_time_ = node_->now();

        // Start MPC update thread
        mpc_thread_should_stop_ = false;
        mpc_running_ = true;
        mpc_thread_ = std::thread(&StateOCS2::mpcUpdateThread, this);

        RCLCPP_INFO(node_->get_logger(), "OCS2 state entered successfully, MPC update thread started");
    }

    void StateOCS2::run(const rclcpp::Time& time, const rclcpp::Duration& /* period */)
    {
        // Check if MPC update is needed
        if ((time - last_mpc_time_).seconds() >= mpc_period_)
        {
            // Set MPC update flag
            mpc_update_requested_ = true;
            last_mpc_time_ = time;
        }

        // Execute policy evaluation
        ctrl_comp_->evaluatePolicy(time);
    }

    void StateOCS2::exit()
    {
        RCLCPP_INFO(node_->get_logger(), "Exiting OCS2 state");
        
        // Stop MPC update thread
        stopMpcThread();
        
        // Clear trajectory visualization
        ctrl_comp_->clearTrajectoryVisualization();
        
        RCLCPP_INFO(node_->get_logger(), "OCS2 state exited successfully, MPC update thread stopped");
    }

    void StateOCS2::mpcUpdateThread()
    {
        RCLCPP_INFO(node_->get_logger(), "MPC update thread started");
        
        while (!mpc_thread_should_stop_.load())
        {
            // Check if MPC update is needed
            if (mpc_update_requested_.load())
            {
                try
                {
                    ctrl_comp_->advanceMpc();
                    mpc_update_requested_ = false; // Clear flag
                }
                catch (const std::exception& e)
                {
                    RCLCPP_ERROR(node_->get_logger(), "Error in MPC update: %s", e.what());
                    mpc_update_requested_ = false; // Clear flag even on error
                }
            }
            
            // Brief sleep to avoid busy waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(thread_sleep_duration_ms_));
        }
        
        RCLCPP_INFO(node_->get_logger(), "MPC update thread stopped");
    }

    void StateOCS2::stopMpcThread()
    {
        if (mpc_running_)
        {
            // Set stop flag
            mpc_thread_should_stop_ = true;
            
            // Wait for thread to finish
            if (mpc_thread_.joinable())
            {
                mpc_thread_.join();
            }
            
            mpc_running_ = false;
            RCLCPP_INFO(node_->get_logger(), "MPC update thread stopped successfully");
        }
    }

    FSMStateName StateOCS2::checkChange()
    {
        // Check control inputs for state transition
        switch (ctrl_interfaces_.control_inputs_.command)
        {
        case 2: return FSMStateName::HOLD;
        default: return FSMStateName::OCS2;
        }
    }
} // namespace ocs2::mobile_manipulator
