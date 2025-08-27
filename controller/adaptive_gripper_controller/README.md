# 简单夹爪控制器 (Simple Gripper Controller)

## 概述

简单夹爪控制器是一个基础的ROS2控制器，实现夹爪的基本位置读取和输出功能。这个控制器专注于核心功能，易于理解和扩展。

## 主要功能

- **位置读取**：从硬件接口读取夹爪的当前位置
- **位置输出**：向硬件接口输出目标位置命令
- **基本控制**：实现简单的夹爪位置控制

## 安装和编译

### 依赖项
```bash
# ROS2 Humble或更新版本
# 以下ROS2包：
- controller_interface
- hardware_interface
- rclcpp
- rclcpp_lifecycle
```

### 编译
```bash
cd ~/ros2_ws
colcon build --packages-select adaptive_gripper_controller
```

## 配置

### 基本配置
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    simple_gripper_controller:
      type: adaptive_gripper_controller/AdaptiveGripperActionController

simple_gripper_controller:
  ros__parameters:
    joint: "gripper_joint"
```

## 使用方法

### 1. 启动控制器
```bash
# 启动控制器管理器
ros2 launch controller_manager spawner.launch.py

# 或者使用launch文件
ros2 launch your_robot_bringup robot_bringup.launch.py
```

### 2. 监控夹爪状态
```bash
# 查看关节状态
ros2 topic echo /joint_states

# 查看控制器状态
ros2 control list_controllers
```

## 接口

### 命令接口
- `{joint}/position`: 位置命令接口

### 状态接口
- `{joint}/position`: 位置状态接口

## 扩展建议

这个简单的控制器可以作为基础，后续可以添加以下功能：

1. **参数配置**：添加可配置的参数
2. **话题接口**：添加ROS话题来接收位置命令
3. **服务接口**：添加ROS服务来控制夹爪
4. **状态监控**：添加夹爪状态监控
5. **安全功能**：添加位置限制和安全检查

## 故障排除

### 常见问题

1. **控制器无法启动**
   - 检查硬件接口配置
   - 确认关节名称正确
   - 检查依赖包是否安装

2. **位置命令不生效**
   - 检查硬件接口是否提供位置数据
   - 确认命令接口配置正确

### 调试命令
```bash
# 查看控制器状态
ros2 control list_controllers

# 查看硬件接口
ros2 control list_hardware_interfaces

# 查看关节状态
ros2 topic echo /joint_states
```

## 许可证

Apache License 2.0

## 联系方式

如有问题，请通过GitHub Issues联系我们。
