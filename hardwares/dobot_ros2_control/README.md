# Dobot ROS2 Control Hardware Interface

## 1. Interfaces

## 2. Build

Tested environment:
* Ubuntu 24.04
   * ROS2 Jazzy
* Ubuntu 22.04
   * ROS2 Humble

Build Command:
```bash
cd ~/ros2_ws
colcon build --packages-up-to dobot_ros2_control --symlink-install
```

```
Dobot CR机器人 (192.168.5.38)
    ↑ TCP连接 (端口30004实时数据, 29999控制命令)
dobot_ros2_control (硬件接口)
    ↓ 提供 hardware_interface
ros2_control_node (控制器管理器)
    ↓ 加载控制器
joint_trajectory_controller / position_controller
    ↓
MoveIt / 其他高层应用
```

**注意**: 本实现直接通过TCP与机器人通信，无需启动cr_robot_ros2_node底层驱动节点。

## 配置参数

在 URDF 或 ros2_control 配置文件中可以设置以下参数：

| 参数名称 | 类型 | 默认值 | 说明 |
|---------|------|--------|------|
| `robot_ip` | string | `192.168.5.38` | 机器人 IP 地址 |
| `servo_time` | double | `0.03` | ServoJ 执行时间（秒），建议为 controller_manager update_rate 的倒数 |
| `aheadtime` | double | `20.0` | 提前量，用于轨迹平滑预测（范围 20-100） |
| `gain` | double | `500.0` | 比例增益，用于轨迹跟踪误差补偿（范围 200-1000） |
| `speed_factor` | int | `5` | 全局速度比例（范围 1-100，百分比） |
| `verbose` | bool | `false` | 是否显示详细日志（包括 TCP 接收和命令发送频率） |

**注意**: 
- 硬件接口的更新频率由 `controller_manager` 的 `update_rate` 参数控制（在 `config/dobot_ros2_control.yaml` 中配置）
- `servo_time` 应该与控制周期匹配，例如 100Hz 控制频率对应 0.01 秒
- `aheadtime` 和 `gain` 参数用于改善轨迹跟踪性能，建议保持默认值

## URDF 配置示例

```xml
<ros2_control name="dobot_system" type="system">
  <hardware>
    <plugin>dobot_ros2_control/DobotHardware</plugin>
    <param name="robot_ip">192.168.5.38</param>
    <param name="servo_time">0.01</param>
    <param name="aheadtime">20.0</param>
    <param name="gain">500.0</param>
    <param name="speed_factor">5</param>
    <param name="verbose">false</param>
  </hardware>
  
  <!-- 6个关节 -->
  <joint name="joint1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  
  <joint name="joint2">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  
  <joint name="joint3">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  
  <joint name="joint4">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  
  <joint name="joint5">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  
  <joint name="joint6">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

## 使用方法

### 1. 编译

```bash
cd ~/ros2_ws
colcon build --packages-select dobot_ros2_control --symlink-install
source install/setup.bash
```

### 2. 确保机器人网络连接

确保你的电脑能够访问机器人IP地址：

```bash
ping 192.168.5.38
```

### 3. 启动 ros2_control

在你的 launch 文件中添加 ros2_control_node：

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, ros2_control_params],
            output='screen',
        ),
        # ... 加载控制器 ...
    ])
```

### 4. 加载控制器

```bash
# 加载关节状态广播器
ros2 control load_controller joint_state_broadcaster

# 加载位置控制器（例如）
ros2 control load_controller position_controller

# 配置并启动控制器
ros2 control set_controller_state joint_state_broadcaster start
ros2 control set_controller_state position_controller start
```

## 测试

### 查看硬件接口状态

```bash
ros2 control list_hardware_interfaces
```

应该看到类似输出：
```
command interfaces
    joint1/position [available] [unclaimed]
    joint2/position [available] [unclaimed]
    ...
state interfaces
    joint1/position
    joint1/velocity
    joint1/effort
    ...
```

### 查看控制器状态

```bash
ros2 control list_controllers
```

## 依赖项

- ROS2 Humble 或更高版本
- ros2_control
- ros2_controllers
- sensor_msgs
- dobot_msgs_v4 (Dobot 底层驱动包)

## 技术细节

### 数据流

1. **状态读取**：
   - `CRCommanderRos2` 通过TCP连接（端口30004）接收实时数据
   - 独立线程持续接收和解析 `RealTimeData` 结构体（1440字节）
   - `read()` 函数调用 `getCurrentJointStatus()` 获取最新关节位置
   - 角度自动转换为弧度
   - 同时读取速度和力矩数据

2. **命令发送**（待实现）：
   - ros2_control 调用 `write()` 函数
   - `write()` 通过TCP连接（端口29999）发送控制命令
   - 支持ServoJ等实时控制指令

### 线程安全

- 使用 `std::mutex` 保护共享数据
- 回调函数和 `read()` 函数可能在不同线程执行
- 所有数据访问都有互斥锁保护

### 生命周期管理

- `on_init()`: 读取配置参数，初始化数据结构
- `on_activate()`: 创建订阅器，等待首次数据
- `read()`: 返回最新的关节状态
- `write()`: 发送控制命令（当前为空实现）
- `on_deactivate()`: 清理资源

## 开发计划

- [ ] 实现位置命令发送（通过 ServoJ 服务）
- [ ] 添加速度控制模式
- [ ] 添加力矩控制模式
- [ ] 添加碰撞检测接口
- [ ] 优化实时性能
- [ ] 添加错误恢复机制

## 参考资料

- [ros2_control 文档](https://control.ros.org/)
- [hardware_interface API](https://control.ros.org/master/doc/api/hardware_interface/html/)
- Dobot CR 系列编程手册

## 许可证

Apache-2.0

## 贡献

欢迎提交 Issue 和 Pull Request！

