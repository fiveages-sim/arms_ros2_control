# Basic Joint Controller

一个关节位置控制器，包含三状态 FSM（Home / Hold / MoveJ），支持可选的灵巧手开关/比例控制以及腰部升降/转向控制。

## 1. 编译

```bash
cd ~/ros2_ws
colcon build --packages-up-to basic_joint_controller --symlink-install
```

---

## 2. 功能特性

- **StateHome**：平滑插值到最多 10 个预设关节构型之一
- **StateHold**：保持所有关节当前位置
- **StateMoveJ**：接收目标关节位置并平滑插值；支持关节轨迹和基于前缀的局部控制
- **灵巧手控制**（`target_command_enabled`）：开关控制或在两个预设构型间按比例插值
- **腰部升降/转向控制**（`waist_lifting_enabled`）：基于速度系数的腰部运动

---

## 3. 配置参数

```yaml
my_controller:
  ros__parameters:
    update_rate: 1000           # 控制器更新频率 (Hz)
    joints: ["j1", "j2", ...]   # 受控关节名称列表
    command_interfaces: ["position"]
    state_interfaces:  ["position", "velocity"]

    # --- Home 构型（最多 10 个）---
    home_1: [0.0, 0.0, ...]     # 必须
    home_2: [0.5, 0.5, ...]     # 可选
    home_3: [-0.5, -0.5, ...]   # 可选
    home_duration: 3.0
    home_interpolation_type: "tanh"   # "tanh" | "linear"
    home_tanh_scale: 3.0
    switch_command_base: 100    # 切换构型的 FSM 命令基础值

    # --- MoveJ ---
    movej_duration: 3.0
    movej_interpolation_type: "tanh"  # "tanh" | "linear"
    movej_tanh_scale: 3.0
    movej_trajectory_duration: 3.0
    movej_trajectory_blend_ratio: 0.0

    # --- 灵巧手 / 末端执行器开关与比例控制 ---
    # 需要 target_command_enabled: true
    # 使用 StateHome 的构型作为开/关目标位置
    target_command_enabled: false
    target_command_close_config: 1  # "关闭"构型的索引（0-based）
    target_command_open_config: 0   # "打开"构型的索引（0-based）

    # --- 腰部升降 ---
    waist_lifting_enabled: false
    waist_lifting_type: "three_joint"   # "three_joint" | "single_joint"
    waist_lifting_duration: 3.0
    waist_default_parameter: [0.25, 1.0, 5.0]  # [最大速度, 加速度, 减速度]
    # (three_joint 模式专用)
    waist_l1: 0.322
    waist_l2: 0.355
    waist_rotation_direction: [1.0, 1.0, 1.0]
    waist_angle_offset: [0.0, 0.0, 0.0]
```

---

## 4. FSM 状态转换

通过向 `/fsm_command`（`std_msgs/Int32`）发布消息触发状态转换：

| 值 | 动作 |
|---|---|
| `1` | 切换到 **HOME** |
| `2` | 切换到 **HOLD** |
| `3` | 切换到 **MOVEJ** |
| `switch_command_base`（默认 `100`） | (HOME) 循环切换到下一个构型 |
| `switch_command_base + 1`（默认 `101`） | (HOME) 切换到构型 0 |
| `switch_command_base + 2`（默认 `102`） | (HOME) 切换到构型 1 |
| … | … |

```bash
ros2 topic pub --once /fsm_command std_msgs/msg/Int32 "data: 1"   # → HOME
ros2 topic pub --once /fsm_command std_msgs/msg/Int32 "data: 3"   # → MOVEJ
```

---

## 5. 话题接口

以下所有话题均以控制器名称为命名空间（例如 `/left_hand_controller/...`）。

### 5.1 MoveJ：目标关节位置

**话题：** `/{controller_name}/target_joint_position`  
**消息类型：** `std_msgs/Float64MultiArray`  
**生效状态：** MOVEJ

```bash
ros2 topic pub --once /my_controller/target_joint_position \
  std_msgs/msg/Float64MultiArray "{data: [0.1, 0.2, 0.3]}"
```

### 5.2 MoveJ：关节轨迹

**话题：** `/{controller_name}/target_joint_trajectory`  
**消息类型：** `trajectory_msgs/JointTrajectory`  
**生效状态：** MOVEJ

多路点轨迹，按 `movej_trajectory_blend_ratio` 进行路点融合。

### 5.3 灵巧手 — 开关控制

**话题：** `/{controller_name}/target_command`  
**消息类型：** `std_msgs/Int32`  
**前提：** `target_command_enabled: true`，当前状态 MOVEJ

| 值 | 动作 |
|---|---|
| `0` | 移动到**关闭**构型（`home_configs[target_command_close_config]`） |
| `1` | 移动到**打开**构型（`home_configs[target_command_open_config]`）  |

```bash
ros2 topic pub --once /left_hand_controller/target_command std_msgs/msg/Int32 "data: 0"  # 关闭
ros2 topic pub --once /left_hand_controller/target_command std_msgs/msg/Int32 "data: 1"  # 打开
```

### 5.4 灵巧手 — 比例控制

**话题：** `/{controller_name}/target_percent`  
**消息类型：** `std_msgs/Float64`（范围：`0.0` ~ `1.0`）  
**前提：** `target_command_enabled: true`，当前状态 MOVEJ

在关闭构型和打开构型之间**逐关节线性插值**：

```
target[i] = close_config[i] + percent × (open_config[i] − close_config[i])
```

| 值 | 结果 |
|---|---|
| `0.0` | 完全关闭（等同于 `target_command = 0`） |
| `1.0` | 完全打开（等同于 `target_command = 1`） |
| `0.0~1.0` | 两个构型之间的比例混合 |

超出 `[0.0, 1.0]` 的值会被截断并输出警告日志。

```bash
ros2 topic pub --once /left_hand_controller/target_percent \
  std_msgs/msg/Float64 "data: 0.6"   # 打开至 60%
```

### 5.5 腰部升降 — 位置控制

**话题：** `/{controller_name}/waist_lifting`  
**消息类型：** `std_msgs/Float64`  
**前提：** `waist_lifting_enabled: true`，当前状态 MOVEJ

相对当前位置移动指定距离（米）。

```bash
ros2 topic pub --once /body_controller/waist_lifting std_msgs/msg/Float64 "data: 0.05"
```

### 5.6 腰部升降 — 速度系数

**话题：** `/{controller_name}/waist_lifting_command`  
**消息类型：** `std_msgs/Float64`（系数范围：`[-1.0, 1.0]`）  
**前提：** `waist_lifting_enabled: true`，当前状态 MOVEJ

持续速度控制。实际速度 = `factor × waist_default_parameter[0]`。发送 `0.0` 停止。

```bash
ros2 topic pub /body_controller/waist_lifting_command std_msgs/msg/Float64 "data: 0.5"   # 上升
ros2 topic pub --once /body_controller/waist_lifting_command std_msgs/msg/Float64 "data: 0.0"  # 停止
```

### 5.7 腰部转向 — 速度系数

**话题：** `/{controller_name}/waist_turning_command`  
**消息类型：** `std_msgs/Float64`（系数范围：`[-1.0, 1.0]`）  
**前提：** `waist_lifting_enabled: true`，当前状态 MOVEJ

```bash
ros2 topic pub /body_controller/waist_turning_command std_msgs/msg/Float64 "data: -0.3"  # 左转
```

---

## 6. 话题汇总

以控制器名称 `my_controller` 为例：

| 话题 | 消息类型 | 生效状态 | 说明 |
|---|---|---|---|
| `/fsm_command` | `Int32` | 任意 | FSM 状态/构型切换 |
| `/my_controller/target_joint_position` | `Float64MultiArray` | MOVEJ | 直接关节目标 |
| `/my_controller/target_joint_trajectory` | `JointTrajectory` | MOVEJ | 多路点轨迹 |
| `/my_controller/target_command` | `Int32` (0/1) | MOVEJ | 灵巧手开关控制 |
| `/my_controller/target_percent` | `Float64` (0~1) | MOVEJ | 灵巧手比例控制 |
| `/my_controller/waist_lifting` | `Float64` | MOVEJ | 腰部升降距离 |
| `/my_controller/waist_lifting_command` | `Float64` | MOVEJ | 腰部升降速度系数 |
| `/my_controller/waist_turning_command` | `Float64` | MOVEJ | 腰部转向速度系数 |

---

## 7. Demo Launch 文件

```bash
source ~/ros2_ws/install/setup.bash

# 启动头部和身体控制器（默认）
ros2 launch basic_joint_controller demo.launch.py robot:=fiveages_w1

# 只启动头部控制器
ros2 launch basic_joint_controller demo.launch.py robot:=fiveages_w1 enable_body:=false

# 只启动身体控制器
ros2 launch basic_joint_controller demo.launch.py robot:=fiveages_w1 enable_head:=false
```

| 参数 | 默认值 | 说明 |
|---|---|---|
| `robot` | `fiveages_w1` | 机器人名称 |
| `type` | — | 机器人类型（可选） |
| `hardware` | `mock_components` | `gz` / `isaac` / `mock_components` |
| `world` | `dart` | Gazebo 世界名称（仅 `hardware=gz` 时生效） |
| `enable_head` | `true` | 是否启用头部控制器 |
| `enable_body` | `true` | 是否启用身体控制器 |
| `use_rviz` | `true` | 是否启动 RViz |

---

## 8. 实现说明

### 插值算法

Home 和 MoveJ 状态均支持可配置插值方式：

- **`tanh`**（默认）：S 形曲线，平滑加减速
  ```
  phase = tanh(t/T × scale)
  pos   = start + phase × (target − start)
  ```
- **`linear`**：匀速插值

### 线程安全

`StateMoveJ::setTargetPosition()` 内部使用互斥锁保护，可从任意 ROS 回调线程安全调用。
