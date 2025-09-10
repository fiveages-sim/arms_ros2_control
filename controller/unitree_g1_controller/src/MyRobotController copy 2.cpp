// include necessary headers
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
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
    // command_interface_names.push_back("left_hip_pitch_joint/effort");
    // 你可以根据需要添加更多接口，例如：
    // command_interface_names.push_back("right_hip_pitch_joint/effort");
    // command_interface_names.push_back("left_knee_joint/position");

    // 下肢关键关节 - effort 接口
    command_interface_names.push_back("left_hip_pitch_joint/effort");
    command_interface_names.push_back("left_knee_joint/effort");
    command_interface_names.push_back("left_ankle_pitch_joint/effort");
    command_interface_names.push_back("right_hip_pitch_joint/effort");
    command_interface_names.push_back("right_knee_joint/effort");
    command_interface_names.push_back("right_ankle_pitch_joint/effort");
    // 腰部关节 - effort 接口
    command_interface_names.push_back("waist_pitch_joint/effort");
    command_interface_names.push_back("waist_roll_joint/effort");


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
    phase_ = 3.0;
    cycle_duration_ = 1.0;
    return controller_interface::CallbackReturn::SUCCESS;
  }

  // 生命周期：控制器停用时的回调
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    // 控制器停用时，基类会自动释放所有已声明的接口
    return controller_interface::CallbackReturn::SUCCESS;
  }

  // 核心更新函数，在每个控制周期被调用
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &period) override
  {
    // 1. 更新跑步相位 (0~2PI)
    phase_ += (2 * M_PI) * (period.nanoseconds() / 1e9) / cycle_duration_;
    if (phase_ > 2 * M_PI) phase_ -= 2 * M_PI;

    // 2. 为每个关节计算并设置力矩命令
    for (auto & interface : command_interfaces_) {
      std::string iface_name = interface.get_interface_name();

      std::cout << "[DEBUG] Interface: " << iface_name << std::endl;

      double effort_cmd = 0.0;

      // 根据关节名称和相位分配不同的控制信号
      if (iface_name.find("hip_pitch_joint/effort") != std::string::npos) {
        // 髋关节：提供主要的前后摆动动力，相位差PI使左右交替
        double phase_offset = (iface_name.find("left_") != std::string::npos) ? 0.0 : M_PI;
        effort_cmd = 15.0 * sin(phase_ + phase_offset); // 幅度15Nm
      }
      else if (iface_name.find("knee_joint/effort") != std::string::npos) {
        // 膝关节：在摆动相屈膝，支撑相伸膝并提供缓冲
        double phase_offset = (iface_name.find("left_") != std::string::npos) ? 0.0 : M_PI;
        // 使用绝对值sin模拟蹬地时的爆发和摆动时的放松
        effort_cmd = 10.0 * fabs(sin(phase_ + phase_offset)); // 幅度10Nm
      }
      else if (iface_name.find("ankle_pitch_joint/effort") != std::string::npos) {
        // 踝关节：配合膝关节，在蹬离地面时提供额外力矩
        double phase_offset = (iface_name.find("left_") != std::string::npos) ? 0.0 : M_PI;
        effort_cmd = 5.0 * sin(phase_ + phase_offset + 0.2); // 幅度5Nm，稍有相位领先
      }
      else if (iface_name.find("waist_pitch_joint/effort") != std::string::npos) {
        // 腰部俯仰：协调身体前后平衡，对抗跑步时的前后倾
        effort_cmd = 8.0 * sin(phase_); // 幅度8Nm
      }
      else if (iface_name.find("waist_roll_joint/effort") != std::string::npos) {
        // 腰部侧滚：协调身体左右平衡，重心转移时起作用
        effort_cmd = 3.0 * sin(2 * phase_); // 幅度3Nm，频率是两倍
      }

      interface.set_value(effort_cmd);
    }

    return controller_interface::return_type::OK;
  }


  private:
    double phase_;
    double cycle_duration_;

  // 控制器初始化（可选）
  controller_interface::CallbackReturn on_init() override
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }
};

// 将控制器注册为插件
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(MyRobotController, controller_interface::ControllerInterface)