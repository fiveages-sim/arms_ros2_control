//
// Created for OCS2 Arm Controller - StateHome
//

#ifndef STATEHOME_H
#define STATEHOME_H

#include "ocs2_arm_controller/Ocs2ArmController.h"
#include <rclcpp/rclcpp.hpp>

namespace ocs2_arm_controller {

class StateHome final : public FSMState {
public:
    explicit StateHome(CtrlInterfaces& ctrl_interfaces, const std::vector<double>& target_pos);

    void enter() override;
    void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    void exit() override;
    FSMStateName checkChange() override;

private:
    CtrlInterfaces& ctrl_interfaces_;
    std::vector<double> target_pos_;
};

} // namespace ocs2_arm_controller

#endif //STATEHOME_H 