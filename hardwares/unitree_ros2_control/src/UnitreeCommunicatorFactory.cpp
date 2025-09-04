#include "unitree_ros2_control/UnitreeCommunicatorFactory.h"
#include "unitree_ros2_control/QuadrupedCommunicator.h"
#include "unitree_ros2_control/HumanoidCommunicator.h"
#include <rclcpp/rclcpp.hpp>

std::unique_ptr<UnitreeCommunicator> UnitreeCommunicatorFactory::createCommunicator(const std::string& robot_type) {
    if (robot_type == "quadruped" || robot_type == "go2") {
        return std::make_unique<QuadrupedCommunicator>();
    } else if (robot_type == "humanoid" || robot_type == "g1") {
        return std::make_unique<HumanoidCommunicator>();
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("UnitreeCommunicatorFactory"), 
                    "Unsupported robot type: %s", robot_type.c_str());
        return nullptr;
    }
}

std::vector<std::string> UnitreeCommunicatorFactory::getSupportedRobotTypes() {
    return {"quadruped", "humanoid", "go2", "g1"}; // 支持新旧命名
}

bool UnitreeCommunicatorFactory::isRobotTypeSupported(const std::string& robot_type) {
    return robot_type == "quadruped" || robot_type == "humanoid" || 
           robot_type == "go2" || robot_type == "g1";
}
