# Arms RViz Control Plugin

这是一个用于OCS2 Arm Controller的RViz插件，提供智能的FSM状态切换功能。

## 功能

- **智能状态显示**: 显示当前FSM状态
- **动态按钮控制**: 根据当前状态只显示可用的转换按钮，隐藏不可用的按钮
- **正确的状态转换**: 遵循OCS2 Arm Controller的FSM规则
- **HOLD初始状态**: 插件启动时默认处于HOLD状态
- **姿态切换功能**: 在HOME状态下可以多次切换Home姿态和Rest姿态
  - **HOME → HOLD**: 从归位状态切换到保持状态 (command=2)
  - **HOLD → OCS2**: 从保持状态切换到MPC控制状态 (command=3)  
  - **OCS2 → HOLD**: 从MPC控制状态切换到保持状态 (command=2)
  - **HOLD → HOME**: 从保持状态切换到归位状态 (command=1)
  - **切换姿态**: 在HOME状态下切换Home和Rest姿态 (command=4)

## 安装

```bash
cd /home/fiveages/ros2_ws
colcon build --packages-select arms_rviz_control_plugin
source install/setup.bash
```

## 使用方法

1. 启动RViz2:
```bash
ros2 run rviz2 rviz2
```

2. 在RViz2中添加Panel:
   - 点击 `Panels` → `Add New Panel`
   - 选择 `arms_rviz_control_plugin/OCS2FSMPanel`

3. 确保OCS2 Arm Controller正在运行并订阅 `/control_input` 话题

4. 点击相应的按钮来切换FSM状态

## 状态说明

- **HOME**: 机器人归位到预设位置，支持在Home姿态和Rest姿态之间切换
- **HOLD**: 保持当前位置不动
- **OCS2**: 使用MPC进行轨迹跟踪控制

## FSM状态转换规则

根据OCS2 Arm Controller的实际实现，状态转换遵循以下规则：

1. **HOME状态**: 可以转换到HOLD状态，也可以切换Home/Rest姿态
2. **HOLD状态**: 可以转换到OCS2状态或HOME状态
3. **OCS2状态**: 只能转换到HOLD状态

插件会根据当前状态智能显示可用的转换按钮，完全隐藏不可用的按钮，避免用户尝试无效的状态转换。插件启动时默认处于HOLD状态，显示"HOLD → OCS2"和"HOLD → HOME"两个可用转换按钮。当处于HOME状态时，还会显示"切换姿态"按钮用于在Home和Rest姿态之间多次切换。

## 话题

- **发布**: `/control_input` (arms_ros2_control_msgs/msg/Inputs)
- **订阅**: 无

## 依赖

- ROS2 Humble
- rviz_common
- arms_ros2_control_msgs
