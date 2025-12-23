# Basic Joint Controller

一个基本的关节控制器，包含三个状态机：Home、Hold 和 Move。

## 1. 编译

```bash
cd ~/ros2_ws
colcon build --packages-up-to basic_joint_controller --symlink-install
```

## 2. 功能特性

- **StateHome**: 从当前位置平滑插值到预设的 home 位置
- **StateHold**: 保持当前关节位置不变
- **StateMove**: 接收目标关节位置，从当前位置平滑插值到目标位置

## 3. 配置参数

在控制器配置文件中需要设置以下参数：

```yaml
basic_joint_controller:
  ros__parameters:
    update_rate: 1000  # 控制器更新频率 (Hz)
    joints: ["joint1", "joint2", "joint3", ...]  # 关节名称列表
    command_interfaces: ["position"]  # 命令接口类型
    state_interfaces: ["position", "velocity"]  # 状态接口类型
    home_1: [0.0, 0.0, 0.0, ...]  # Home 配置 1 的关节角度（弧度）
    home_2: [0.5, 0.5, 0.5, ...]  # Home 配置 2 的关节角度（可选）
    home_3: [-0.5, -0.5, -0.5, ...]  # Home 配置 3 的关节角度（可选）
    # 可以继续添加 home_4, home_5, ... 最多支持 10 个配置
    home_duration: 3.0  # Home 状态插值持续时间（秒）
    move_duration: 3.0  # Move 状态插值持续时间（秒）
    home_interpolation_type: "tanh"  # "tanh"（默认）或 "linear"
    home_tanh_scale: 3.0  # 仅在 home_interpolation_type == "tanh" 时生效
    movej_interpolation_type: "tanh"  # "tanh"（默认）或 "linear"
    movej_tanh_scale: 3.0  # 仅在 movej_interpolation_type == "tanh" 时生效
    hold_position_threshold: 0.1  # Hold 状态位置阈值（弧度）
    switch_command_base: 100  # 配置切换命令的基础值（默认 100，可设置为 4 以兼容旧代码）
```

## 4. 状态转换

状态转换通过 `/control_input` 话题的 `command` 字段控制：

- `command = 1`: 切换到 HOME 状态
- `command = 2`: 切换到 HOLD 状态
- `command = 3`: 切换到 MOVE 状态

### 4.1 Home 状态配置切换

在 HOME 状态下，可以通过以下命令切换不同的 home 配置：

- `command = switch_command_base`（默认 100）: 循环切换到下一个配置
- `command = switch_command_base + 1`（默认 101）: 切换到配置 0
- `command = switch_command_base + 2`（默认 102）: 切换到配置 1
- `command = switch_command_base + 3`（默认 103）: 切换到配置 2
- ... 以此类推

例如，如果 `switch_command_base = 4`（兼容旧代码）：
- `command = 4`: 循环切换
- `command = 5`: 切换到配置 0
- `command = 6`: 切换到配置 1

## 5. 使用方法

### 5.1 Demo Launch 文件

使用 demo launch 文件可以方便地启动头部控制器或身体控制器：

```bash
# 启动头部和身体控制器（默认）
source ~/ros2_ws/install/setup.bash
ros2 launch basic_joint_controller demo.launch.py robot:=fiveages_w1

# 只启动头部控制器
ros2 launch basic_joint_controller demo.launch.py robot:=fiveages_w1 enable_body:=false

# 只启动身体控制器
ros2 launch basic_joint_controller demo.launch.py robot:=fiveages_w1 enable_head:=false
```

Launch 文件参数：
- `robot`: 机器人名称（默认：fiveages_w1）
- `type`: 机器人类型（可选）
- `hardware`: 硬件类型，'gz' 用于 Gazebo 仿真，'isaac' 用于 Isaac 仿真，'mock_components' 用于模拟组件（默认：mock_components）
- `world`: Gazebo 世界名称（仅在 hardware=gz 时使用，默认：dart）
- `enable_head`: 是否启用头部控制器（默认：true）
- `enable_body`: 是否启用身体控制器（默认：true）
- `use_rviz`: 是否启动 RViz 可视化（默认：true）

### 5.2 状态切换

通过发布 `/control_input` 话题来切换状态：

```bash
ros2 topic pub /control_input arms_ros2_control_msgs/msg/Inputs "{command: 1}"  # 切换到 HOME
ros2 topic pub /control_input arms_ros2_control_msgs/msg/Inputs "{command: 2}"  # 切换到 HOLD
ros2 topic pub /control_input arms_ros2_control_msgs/msg/Inputs "{command: 3}"  # 切换到 MOVE
```

### 5.3 设置目标位置（Move 状态）

在 MOVE 状态下，通过发布 `target_joint_position` 话题（相对于控制器命名空间）来设置目标关节位置：

```bash
# 对于名为 "head_joint_controller" 的控制器：
ros2 topic pub /head_joint_controller/target_joint_position std_msgs/msg/Float64MultiArray "{data: [1.0, 0.5, -0.5, ...]}"  # 设置目标关节角度（弧度）

# 对于名为 "waist_joint_controller" 的控制器：
ros2 topic pub /waist_joint_controller/target_joint_position std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, ...]}"  # 设置目标关节角度（弧度）
```

**注意**：话题名称相对于控制器的命名空间，允许多个控制器实例（例如一个控制头部，一个控制腰部）同时运行而不会冲突。

## 6. 实现说明

### 6.1 状态机架构

- 所有状态都继承自 `FSMState` 基类
- 每个状态实现 `enter()`, `run()`, `exit()`, `checkChange()` 方法
- 状态转换通过 `checkChange()` 方法检查控制输入来决定

### 6.2 插值算法

Home 状态使用 `tanh` 函数进行平滑插值。MoveJ 状态支持可配置插值方式。

```cpp
double phase = std::tanh(percent_ * 3.0);
double interpolated_value = phase * target_pos[i] + (1.0 - phase) * start_pos[i];
```

这样可以确保平滑的加速和减速。

#### MoveJ 插值参数

- **`movej_interpolation_type`**：`"tanh"`（默认）或 `"linear"`
- **`movej_tanh_scale`**：tanh 缩放系数（默认 `3.0`），仅在 `movej_interpolation_type="tanh"` 时生效

### 6.3 线程安全

StateMove 使用互斥锁保护目标位置的更新，确保在多线程环境下的安全性。

