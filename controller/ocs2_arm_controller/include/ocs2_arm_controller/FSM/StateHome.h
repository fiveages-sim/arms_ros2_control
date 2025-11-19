//
// Created for OCS2 Arm Controller - StateHome
//

#ifndef STATEHOME_H
#define STATEHOME_H

#include "ocs2_arm_controller/Ocs2ArmController.h"
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace ocs2::mobile_manipulator
{
    class StateHome final : public FSMState
    {
    public:
        explicit StateHome(CtrlInterfaces& ctrl_interfaces, const std::vector<double>& target_pos, 
                          const std::shared_ptr<CtrlComponent>& ctrl_comp = nullptr,
                          double duration = 3.0);

        void enter() override;
        void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        void exit() override;
        FSMStateName checkChange() override;

        // Set rest pose configuration
        void setRestPose(const std::vector<double>& rest_pos);
        
        // Check if rest pose is configured
        bool hasRestPose() const { return has_rest_pose_; }

    private:
        // Switch between home pose and rest pose
        void switchPose();
        
        // Start interpolation to new target pose
        void startInterpolation();

        CtrlInterfaces& ctrl_interfaces_;
        std::shared_ptr<CtrlComponent> ctrl_comp_;  // CtrlComponent reference for torque calculation
        std::vector<double> target_pos_;  // Home pose
        std::vector<double> rest_pos_;    // Rest pose (if configured)
        std::vector<double> start_pos_;   // Starting position when entering state
        std::vector<double> current_target_; // Current target pose (home or rest)
        double duration_;                 // Interpolation duration in seconds
        double percent_;                  // Interpolation progress (0.0 to 1.0)
        bool has_rest_pose_;             // Whether rest pose is configured
        bool is_rest_pose_;              // Current pose state (false: home, true: rest)
        
        // Pose switching debounce
        bool last_switch_command_{false};  // Last switch command state
        bool switch_debounced_{false};     // Debounced switch command
    };
} // namespace ocs2::mobile_manipulator

#endif //STATEHOME_H
