# Adaptive Gripper Controller

## 概述

自适应夹爪控制器（`adaptive_gripper_controller`）是一个基于 ros2_control 框架的夹爪位置控制器。支持三种独立的控制通道，并在向关闭方向运动时提供基于力/力矩反馈的自适应抓取检测，可在接触到物体时自动停止。

关节限位和初始值从 `/robot_description` 自动解析，无需在控制器参数中手动指定开合位置。

---

## 编译

```bash
cd ~/ros2_ws
colcon build --packages-select adaptive_gripper_controller
```

---

## 配置参数

在控制器的 `ros__parameters` 下声明：

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `joint` | string | `"gripper_joint"` | 受控关节名称 |
| `use_effort_interface` | bool | `true` | 是否启用力/力矩状态接口用于力反馈 |
| `force_threshold` | double | `0.1` | 触发力反馈的力/力矩阈值（单位与硬件一致） |
| `force_feedback_ratio` | double | `0.5` | 力反馈触发后继续推进的比例（0.0=停在当前位置，1.0=继续到原始目标） |

### 配置示例

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz
    left_gripper_controller:
      type: adaptive_gripper_controller/AdaptiveGripperController

left_gripper_controller:
  ros__parameters:
    joint: "left_gripper_joint"
    use_effort_interface: true
    force_threshold: 0.15
    force_feedback_ratio: 0.3
```

---

## 硬件接口

### 命令接口（必须）

| 接口 | 说明 |
|---|---|
| `{joint}/position` | 位置命令输出 |

### 状态接口

| 接口 | 必须 | 说明 |
|---|---|---|
| `{joint}/position` | 是 | 当前关节位置 |
| `{joint}/effort` | 否（由 `use_effort_interface` 控制） | 当前关节力/力矩，用于力反馈检测 |

---

## 话题接口

控制器提供三条独立的控制通道，可按需选择使用。每条通道的收到命令后立即生效。

### 通道 1：直接位置控制

**话题：** `/{joint_name}/position_command`  
**消息类型：** `std_msgs/Float64`  
**力反馈：** 禁用（纯位置控制）

发布关节位置目标值（单位：弧度或米，与关节类型一致）。数值会被自动截断到关节限位范围内。

```bash
# 示例：发布位置命令
ros2 topic pub --once /left_gripper_joint/position_command std_msgs/msg/Float64 "data: 0.05"
```

---

### 通道 2：开关控制

**话题：** `/{controller_name}/target_command`  
**消息类型：** `std_msgs/Int32`  
**力反馈：** 关闭（`0`）时启用

| 值 | 行为 |
|---|---|
| `1` | 移动到完全打开位置，无力反馈 |
| `0` | 移动到完全关闭位置，启用力反馈（检测到接触时停止） |

```bash
# 关闭夹爪（启用力反馈）
ros2 topic pub --once /left_gripper_controller/target_command std_msgs/msg/Int32 "data: 0"

# 打开夹爪
ros2 topic pub --once /left_gripper_controller/target_command std_msgs/msg/Int32 "data: 1"
```

---

### 通道 3：比例控制

**话题：** `/{controller_name}/target_percent`  
**消息类型：** `std_msgs/Float64`（范围：`0.0` ~ `1.0`）  
**力反馈：** 根据目标方向**自动选择**

| 值 | 行为 |
|---|---|
| `0.0` | 完全关闭（`closed_position`） |
| `1.0` | 完全打开（`open_position`） |
| `0.0~1.0` | 在关闭/打开位置之间线性插值 |

控制器会在 `update()` 内根据目标位置与当前位置的关系自动判断运动方向：
- **向关闭方向运动** → 启用力反馈，检测到抓取接触时按 `force_feedback_ratio` 停止
- **向打开方向运动** → 纯位置控制，无力反馈

超出 `[0.0, 1.0]` 的值会被自动截断并打印警告。

```bash
# 打开至 60%
ros2 topic pub --once /left_gripper_controller/target_percent std_msgs/msg/Float64 "data: 0.6"

# 完全关闭（力反馈抓取）
ros2 topic pub --once /left_gripper_controller/target_percent std_msgs/msg/Float64 "data: 0.0"
```

---

## 力反馈逻辑

力反馈仅在以下条件同时满足时生效：
1. 当前处于**开关控制或比例控制**模式（非直接位置控制）
2. `use_effort_interface: true` 且硬件 effort 接口已绑定
3. 当前运动方向为**关闭方向**（`gripper_target_ == 0`）
4. 尚未触发过力反馈（每次新命令会重置此标志）

触发条件：`|current_effort| > force_threshold`

触发后的目标位置计算：
```
新目标位置 = 当前位置 + (原始目标 - 当前位置) × force_feedback_ratio
```

- `force_feedback_ratio = 0.0`：立即停在当前位置（最柔性）
- `force_feedback_ratio = 1.0`：继续推进到原始目标（最刚性）
- `force_feedback_ratio = 0.5`（默认）：推进剩余距离的一半

---

## 位置初始化机制

控制器从 `/robot_description` 话题（`transient_local` QoS）自动解析：

1. **关节限位**（`<limit lower="..." upper="..."/>`）：确定位置合法范围
2. **初始值**（`<state_interface name="position"><param name="initial_value">`）：
   - 距离初始值**更近的限位**被判定为关闭位置（`closed_position`）
   - 另一侧限位为打开位置（`open_position`）

---

## 话题汇总

以下假设控制器名称为 `left_gripper_controller`，关节名称为 `left_gripper_joint`：

| 话题 | 消息类型 | 方向 | 说明 |
|---|---|---|---|
| `/left_gripper_joint/position_command` | `Float64` | 订阅 | 直接位置控制（无力反馈） |
| `/left_gripper_controller/target_command` | `Int32` | 订阅 | 开关控制（0=关/力反馈，1=开） |
| `/left_gripper_controller/target_percent` | `Float64` | 订阅 | 比例控制（0.0~1.0，自动力反馈） |
| `/robot_description` | `String` | 订阅 | 自动解析限位和初始值 |

---

## 调试命令

```bash
# 查看控制器状态
ros2 control list_controllers

# 查看硬件接口
ros2 control list_hardware_interfaces

# 查看关节状态（确认位置和力矩）
ros2 topic echo /joint_states

# 监控控制器日志（包含力反馈触发信息）
ros2 topic echo /rosout | grep gripper
```

---

## 故障排除

| 现象 | 可能原因 | 解决方法 |
|---|---|---|
| 命令被忽略并提示 "limits not initialized" | `/robot_description` 未发布或解析失败 | 确认机器人描述已加载，检查关节名称是否匹配 |
| 力反馈从不触发 | `use_effort_interface: false` 或硬件未提供 effort 接口 | 检查参数配置和 `ros2 control list_hardware_interfaces` |
| 力反馈触发过早 | `force_threshold` 太小 | 适当增大阈值 |
| 夹爪不能完全抓紧 | `force_feedback_ratio` 过小 | 适当增大比例，或使用直接位置控制通道 |
| 控制器激活失败 | 关节名称与 URDF 中不匹配 | 确认 `joint` 参数与 URDF `<joint name="...">` 一致 |

---

## 许可证

Apache License 2.0
