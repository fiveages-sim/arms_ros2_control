# Rokae ROS2 Control Hardware Interface

基于wheeled_robot_msgs的Rokae双臂机器人ROS2控制硬件接口。

## 功能特性

- 支持双臂7自由度机械臂控制
- 基于wheeled_robot_msgs消息通信
- 纯位置控制接口
- 安全初始化机制
- 实时状态监控

## 消息接口

### 状态消息 (ArmState)
- 左臂/右臂关节位置 (7个关节)
- 左臂/右臂移动状态
- 左臂/右臂笛卡尔位姿

### 命令消息 (ArmMove)
- 左臂/右臂目标位置
- 运动速度比例

## 配置参数

- `arm_state_topic`: 状态话题名称 (默认: "/arm_state")
- `arm_move_topic`: 命令话题名称 (默认: "/arm_move")
- `update_rate`: 更新频率 (默认: 100 Hz)

## URDF配置示例

```xml
<ros2_control name="rokae_system" type="system">
  <hardware>
    <plugin>rokae_ros2_control/RokaeHardware</plugin>
    <param name="arm_state_topic">/arm_state</param>
    <param name="arm_move_topic">/arm_move</param>
    <param name="update_rate">100</param>
  </hardware>
  
  <!-- 左臂关节 -->
  <joint name="left_joint1">
    <command_interface name="position"/>
    <state_interface name="position"/>
  </joint>
  <!-- ... 其他左臂关节 ... -->
  
  <!-- 右臂关节 -->
  <joint name="right_joint1">
    <command_interface name="position"/>
    <state_interface name="position"/>
  </joint>
  <!-- ... 其他右臂关节 ... -->
</ros2_control>
```

## 使用

### 1. 编译
```bash
cd ~/ros2_ws
colcon build --packages-up-to rokae_ros2_control --symlink-install
```

### 2. 启动可视化
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rokae_ros2_control visualize.launch.py
```

### 3. 手动启动
1. 确保wheeled_robot_msgs包已编译
2. 在URDF中配置RokaeHardware插件
3. 启动控制器管理器
4. 加载相应的控制器

## 注意事项

- 只支持位置控制接口
- 需要底层驱动发布ArmState消息
- 命令通过ArmMove消息发送给底层驱动
