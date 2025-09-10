// include necessary headers
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

// 自定义控制器类，继承自 ControllerInterface
class MyRobotController : public controller_interface::ControllerInterface
{
public:
  // 控制器接口配置：指定需要声明的命令接口
  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    // 明确列出所有需要声明的命令接口名称
    std::vector<std::string> command_interface_names;
    // 示例：声明左髋关节俯仰关节的力矩控制接口
    command_interface_names.push_back("left_hip_pitch_joint/effort");
    // 你可以根据需要添加更多接口，例如：
    // command_interface_names.push_back("right_hip_pitch_joint/effort");
    // command_interface_names.push_back("left_knee_joint/position");

    return {controller_interface::interface_configuration_type::INDIVIDUAL, command_interface_names};
  }

  // 状态接口配置（本例暂不关注状态接口，如需读取传感器数据可在此配置）
  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    return {controller_interface::interface_configuration_type::NONE, {}};
  }

  // 生命周期：控制器启动时的回调
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    // 基类方法会帮我们“claim”在 command_interface_configuration 中指定的接口
    // 这些接口会存储在 command_interfaces_ 向量中
    return controller_interface::CallbackReturn::SUCCESS;
  }

  // 生命周期：控制器停用时的回调
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    // 控制器停用时，基类会自动释放所有已声明的接口
    return controller_interface::CallbackReturn::SUCCESS;
  }

  // 核心更新函数，在每个控制周期被调用 OLD
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    // 检查是否成功获取到接口
    if (command_interfaces_.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "No command interfaces claimed!");
      return controller_interface::return_type::ERROR;
    }

    // 假设我们只声明了一个接口，直接访问第一个
    auto & hip_effort_interface = command_interfaces_[0];
    // 为你想要控制的关节设置命令值（例如：发送 5.0 Nm 的力矩）
    double target_effort = 5.0;
    hip_effort_interface.set_value(target_effort);
    // RCLCPP_INFO(get_node()->get_logger(), "Setting effort to: %f", target_effort);

    // 如果你声明了多个接口，可以遍历 command_interfaces_ 并为每个接口设置值
    // for (auto & interface : command_interfaces_) {
    //   if (interface.get_interface_name() == "left_hip_pitch_joint/effort") {
    //     interface.set_value(5.0);
    //   } else if (interface.get_interface_name() == "some_other_interface") {
    //     interface.set_value(...);
    //   }
    // }

    return controller_interface::return_type::OK;
  }


  // 控制器初始化（可选）
  controller_interface::CallbackReturn on_init() override
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }
};

// 将控制器注册为插件
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(MyRobotController, controller_interface::ControllerInterface)