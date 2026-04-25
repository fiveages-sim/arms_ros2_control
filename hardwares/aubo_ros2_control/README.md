# AUBO ROS2 Control 硬件接口

这个包提供 AUBO 真机的 `ros2_control` 硬件插件，以及可选的 BlueDot 六维力传感器节点。

## 1. 目录依赖

当前 AUBO 运行时最小文件集合放在工作区根目录下：

```text
~/ros2_ws/src/aubo/
├── include/
│   ├── serviceinterface.h
│   ├── AuboRobotMetaType.h
│   └── robotiomatetype.h
└── lib/
    ├── libauborobotcontroller.so.1.3.2
    └── liblog4cplus-1.2.so.5
```

`aubo_ros2_control` 会优先从这个扁平目录读取 AUBO 运行时文件。

## 2. 编译

```bash
# 编译 AUBO 真机硬件接口
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select aubo_ros2_control --symlink-install
source install/setup.bash
```

## 3. 真机硬件参数

这些参数由 `aubo_i16_description/xacro/ros2_control/robot.xacro` 写入到硬件插件。

| 参数名 | 默认值 | 说明 |
| --- | --- | --- |
| `robot_ip` | `192.168.1.107` | AUBO 控制柜 IP |
| `robot_port` | `8899` | SDK 登录端口 |
| `username` | `aubo` | 登录用户名 |
| `password` | `123456` | 登录密码 |
| `collision_class` | `6` | 启动时碰撞等级 |
| `max_joint_acc_deg` | `50.0` | 全局最大关节加速度，单位 deg/s^2 |
| `max_joint_vel_deg` | `50.0` | 全局最大关节速度，单位 deg/s |
| `command_mode` | `follow_mode` | 推荐给 OCS2 使用的连续关节命令模式 |
| `move_blocking` | `false` | `movej` 模式时是否阻塞 |
| `shutdown_on_disconnect` | `false` | 断开时是否下发关机 |
| `command_period_ms` | `20` | 连续下发命令周期 |
| `position_command_threshold` | `0.0002` | 小于这个阈值的关节修正不再重复下发 |
| `ft_topic` | `/ft_sensor_wrench` | 外部六维力话题 |

## 4. 常用真机启动命令

```bash
# 真机，无夹爪
ros2 launch ocs2_arm_controller demo.launch.py \
  robot:=aubo_i16 \
  hardware:=real \
  robot_ip:=192.168.1.107 \
  ft_topic:=/ft_sensor_wrench \
  launch_mode:=control_only
```

```bash
# 真机，带 AG2F90-C 夹爪
ros2 launch ocs2_arm_controller demo.launch.py \
  robot:=aubo_i16 \
  type:=AG2F90-C \
  hardware:=real \
  robot_ip:=192.168.1.107 \
  ft_topic:=/ft_sensor_wrench \
  launch_mode:=control_only
```

## 5. BlueDot 六维力传感器

如果真机使用 BlueDot 力传感器，可先单独启动：

```bash
# 将 BlueDot 数据发布到 /ft_sensor_wrench
ros2 run aubo_ros2_control bluedot_force_sensor_node --ros-args \
  -p sensor_ip:=192.168.0.20 \
  -p sensor_port:=49152 \
  -p publish_topic:=/ft_sensor_wrench \
  -p frame_id:=ft_sensor \
  -p poll_period_ms:=10
```

然后再启动真机控制：

```bash
# 让 AUBO 硬件接口订阅同一个力话题
ros2 launch ocs2_arm_controller demo.launch.py \
  robot:=aubo_i16 \
  type:=AG2F90-C \
  hardware:=real \
  robot_ip:=192.168.1.107 \
  ft_topic:=/ft_sensor_wrench \
  launch_mode:=control_only
```

## 6. 说明

- 这套实现走 AUBO 官方 SDK 动态库，不是自己重写的底层通信协议。
- OCS2 场景下默认使用 `follow_mode`，比重复 `movej` 更适合连续控制。
- 如果你只做仿真测试，直接用 `hardware:=mock_components` 即可，不需要真机和控制柜在线。
