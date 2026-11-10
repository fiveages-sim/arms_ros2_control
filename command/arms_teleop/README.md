# Arms Teleop

统一遥操包：手柄（`joystick_teleop`）、键盘（`keyboard_teleop`）、USB 脚踏板（`foot_pedal_teleop`）。

不要同时运行会抢同一批话题的手柄与键盘节点（如 `control_input`、`/cmd_vel`）。

## 目录

- [1. 功能概览](#1-功能概览)
- [2. 编译](#2-编译)
- [3. 手柄遥操](#3-手柄遥操)
- [4. 键盘遥操](#4-键盘遥操)
- [5. 脚踏板遥操](#5-脚踏板遥操)
- [6. 话题](#6-话题)
- [7. 依赖](#7-依赖)

## 1. 功能概览

| 节点 | 输入 | 主要输出 | 说明 |
|------|------|----------|------|
| `joystick_teleop` | `sensor_msgs/Joy` | `control_input`、`teleop_mode`、`/cmd_vel`、腰部指令、`/fsm_command` | 经 launch 选 `game_controller_node` 或 `joy_node` |
| `keyboard_teleop` | 终端 stdin（TTY） | 与手柄相同，**不发布** `/fsm_command` | 独立终端运行；参数可调 |
| `foot_pedal_teleop` | Linux evdev 键盘设备 | `/teleop/controller_state`（`std_msgs/Int32`） | 事件流给 `VRInputHandler`；不发 `/fsm_command` |

## 2. 编译

```bash
cd ~/ros2_ws
colcon build --packages-up-to arms_teleop --symlink-install
```

## 3. 手柄遥操

### 3.1 启动

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch arms_teleop joystick_teleop.launch.py
```

默认 `joy_driver:=auto`：若 `joy_enumerate_devices` 报告 `Mapped: true`，用 [`game_controller_node`](https://docs.ros.org/en/ros2_packages/jazzy/api/joy/index.html) + [`config/joy_mapping/generic_gamepad.yaml`](config/joy_mapping/generic_gamepad.yaml)（SDL 标准布局，F710 / PS4 / Xbox / Switch Pro / GameSir 等大多走此路径）；否则回退 `joy_node` + `default.yaml`。

```bash
# 强制原始 joy 驱动
ros2 launch arms_teleop joystick_teleop.launch.py joy_driver:=joy

# 指定设备（默认 /dev/input/js0）；多手柄时只传 joy_dev，launch 会按设备 ID 再枚举
ros2 launch arms_teleop joystick_teleop.launch.py joy_dev:=/dev/input/js1

# 列出已连接手柄（ID / GUID / GamePad / Mapped / 名称）
ros2 run joy joy_enumerate_devices
```

`quick_start.sh`：仅 1 个手柄时自动启动；多个时选手柄 ID，只传 `joy_dev`。

### 3.2 Launch 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `joy_driver` | `auto` | `auto` / `game_controller` / `joy` |
| `joy_dev` | `/dev/input/js0` | 手柄设备路径（`/dev/input/jsN`） |
| `config` | `auto` | `config/joy_mapping/` 下 YAML 文件名（无后缀），或 `auto` |

### 3.3 映射文件

路径：`config/joy_mapping/`。

| 文件 | 使用时机 |
|------|----------|
| `generic_gamepad.yaml` | `game_controller_node` + `config:=auto`（Mapped 手柄） |
| `default.yaml` | `joy_node` + `config:=auto`（未 Mapped） |

`config:=auto`：`Mapped: true` → `generic_gamepad`；否则 → `default`。

```bash
ros2 launch arms_teleop joystick_teleop.launch.py config:=default joy_driver:=joy
```

**F710 X/D：** 默认 `game_controller_node` 下 X/D 共用 `generic_gamepad.yaml`；若强制 `joy_driver:=joy`，用 `default.yaml`，切换模式后需重启 launch。

### 3.4 新设备校准

仅针对未 Mapped 设备，或强制 `joy_driver:=joy` 时：

1. `ros2 topic echo /joy`，记录按键 / 摇杆下标。
2. 在 `config/joy_mapping/` 新建或修改 YAML，启动时用 `config:=<name>`。

### 3.5 常用操作（节点启动日志摘要）

- 默认**禁用**遥操；右摇杆按下启用 / 关闭。
- `RB`：ARM ↔ CHASSIS；`Back`：镜像；左摇杆按下：高低速。
- 普通模式：`A` 切臂，`X` 夹爪开关，`LT`/`RT` 夹爪开合比例。
- FSM（按住 `LB`）：`LB+A` HOME，`LB+B` HOLD，`LB+Y` MOVEJ，`LB+START` OCS2。

节点侧还有 `mirror_movement`、`speed.*`、`chassis.*`、`button_mapping.*`、`axes_mapping.*` 等 ROS 参数，一般由映射 YAML 提供。

## 4. 键盘遥操

### 4.1 启动

必须在**独立交互终端**运行（stdin 需为真实 TTY），不要嵌进会抢走 stdin 的大 launch：

```bash
source ~/ros2_ws/install/setup.bash
ros2 run arms_teleop keyboard_teleop

# 可选参数
ros2 run arms_teleop keyboard_teleop --ros-args -p discrete_key_stale_ms:=350
```

键盘节点不发布 `/fsm_command`；腰部 lift/turn 话题固定发 0。

### 4.2 按键

| 按键 | 作用 |
|------|------|
| WASD | 平移（ARM：`Inputs` x/y；CHASSIS：`/cmd_vel` 线速度） |
| I / K | z（仅 ARM） |
| J / L | yaw（ARM）或 yaw 角速度（CHASSIS） |
| 方向键 | roll / pitch（仅 ARM） |
| N | 松开时切换 ARM / CHASSIS |
| M | 松开时切换镜像（底盘速度不受镜像影响） |
| `-` / `+` 或 `=` | 速度档 1–10（×0.1），默认档 3 |
| Space | 切换活动臂（仅 ARM） |
| Enter | 夹爪开合（仅 ARM） |

状态行用 `\033[2K\r` 原地刷新（`MODE`、速度、镜像、臂/夹爪）。节点日志级别为 **WARN**，避免 `[INFO]` 顶掉状态行。

### 4.3 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `update_rate` | `20.0` | 定时发布频率（Hz） |
| `movement_key_stale_ms` | `120` | WASD / IK / JL / 方向键无刷新即视为松开；越小抬键停得越快 |
| `discrete_key_stale_ms` | `320` | 离散键（N、M、Space、Enter、+/-）同理；须**大于**内核按键重复延迟（约 250 ms），否则会在首次 repeat 前误判松开导致开关连击 |
| `chassis.linear_scale` | `0.25` | 底盘线速度比例 |
| `chassis.angular_scale` | `0.5` | 底盘角速度比例 |
| `arm_axes_activation_threshold` | `0.5` | 与手柄同名参数同义 |
| `mirror_movement` | `false` | 与手柄同名；ARM 侧实际应用 `mir_apply = !mirror_movement`，使 WASD 与手柄一致；状态行显示该标志（`false` 时为 `OFF`）；运行中用 **M** 切换 |
| `show_status_line` | `true` | `false` 关闭状态行 |

## 5. 脚踏板遥操

`foot_pedal_teleop` 读取一个 Linux evdev 键盘设备，向 `/teleop/controller_state` 发布与 VR 面键相同的 `std_msgs/Int32` 事件流。FSM / 遥操开关由 `VRInputHandler` 统一处理；本节点不发 `/fsm_command`。

默认独占设备（`grab_device:=true`），事件不会进桌面；忽略内核 key-repeat；无事件时仍以 30 Hz 发布 `0`。不使用双击手势。

### 5.1 启动与设备选择

仅连接一台受支持脚踏板时自动发现：

```bash
source ~/ros2_ws/install/setup.bash
ros2 run arms_teleop foot_pedal_teleop
```

多台 CM6K 时会列出序列号并退出，避免误选；按序列号指定：

```bash
ros2 run arms_teleop foot_pedal_teleop --ros-args \
  -p device_serial:=03CDAB4B081EBD3671FFFFFFFF0300
```

也可显式传稳定 `device` 路径（覆盖自动发现，便于排查）。

### 5.2 按键事件

| 按键 | 行为 | 事件值 |
|------|------|--------|
| F13 | FSM down | `12` |
| F14 | FSM up | `11` |
| F15 短按 | 切换左臂遥操 | `3` |
| F15 长按（默认 800 ms） | 切换镜像 | `7` |
| F16 短按 | 切换右臂遥操 | `6` |
| F16 长按（默认 800 ms） | 整体遥操 STORAGE ↔ UPDATE | `4` |

F16 长按只发 `4`，不会再附带 `6`。

### 5.3 参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `device` | `auto` | 自动找 CM6K interface-00，或显式 evdev 路径 |
| `device_serial` | 空 | auto 模式下区分多台 CM6K |
| `grab_device` | `true` | 独占该输入设备 |
| `teleop.publish_rate_hz` | `30.0` | `/teleop/controller_state` 发布频率 |
| `teleop.controller_state_topic` | `/teleop/controller_state` | 事件话题 |
| `whole_teleop_long_press_ms` | `800` | F16 触发事件 `4` 的按住时长 |
| `mirror_long_press_ms` | `800` | F15 触发事件 `7` 的按住时长 |
| `keys.fsm_up` / `fsm_down` / `left_arm_toggle` / `right_arm_toggle` | F13–F16（`183`–`186`） | Linux input key code |

### 5.4 udev 权限

若打开设备报 `Permission denied`，配置 udev，或使用有权读该 evdev 的用户。不要对普通键盘设备开启独占 grab。

SayoDevice CM6K（VID `8089`，PID `000c`，interface `00`）可一次性安装包内规则：

```bash
sudo install -m 0644 \
  src/arms_ros2_control/command/arms_teleop/config/udev/70-sayo-cm6k-foot-pedal.rules \
  /etc/udev/rules.d/70-sayo-cm6k-foot-pedal.rules
sudo udevadm control --reload-rules
```

重载后重新插拔脚踏板。`TAG+="uaccess"` 会在匹配的 event 设备创建时给当前本地登录会话读权限，无需每次 `setfacl`。规则只匹配 USB interface `00`，不含辅助的 `if01`。

## 6. 话题

### 6.1 发布

| 话题 | 类型 | 谁发 | 说明 |
|------|------|------|------|
| `control_input` | `arms_ros2_control_msgs/Inputs` | 手柄 / 键盘 | 高频轴；`target` 为活动臂（1=左，2=右） |
| `teleop_mode` | `arms_ros2_control_msgs/TeleopMode` | 手柄 / 键盘 | 锁存模式快照（`Reliable` + `TransientLocal`）：`active`、`control_mode`、`mirror_movement`、`speed_level`、`speed_scale`、`stamp`；启动发一次，字段变化再发（切臂不触发，看 `Inputs.target`）。手柄默认 `active=false` 直至右摇杆启用；键盘始终 `active=true` |
| `/cmd_vel` | `geometry_msgs/Twist` | 手柄 / 键盘 | 见 [6.3](#63-cmd_vel-与-wbc) |
| `/body_joint_controller/waist_lifting_command` | `std_msgs/Float64` | 手柄 / 键盘 | 键盘固定发 0 |
| `/body_joint_controller/waist_turning_command` | `std_msgs/Float64` | 手柄 / 键盘 | 键盘固定发 0 |
| `/fsm_command` | `std_msgs/Int32` | **仅手柄** | 键盘 / 脚踏板不发 |
| `/teleop/controller_state` | `std_msgs/Int32` | **仅脚踏板** | VR 面键事件流 |

速度字段：键盘 `speed_level` 为 1–10，`speed_scale = level * 0.1`；手柄 `0`=LOW / `1`=HIGH，比例由配置的 low/high scale 决定。

### 6.2 订阅

| 话题 | 类型 | 谁订 | 说明 |
|------|------|------|------|
| `joy` | `sensor_msgs/Joy` | 仅手柄 | |
| `/fsm_state` | `std_msgs/Int32`（锁存） | 手柄 / 键盘 | |
| `/ocs2_wbc_controller/current_state` | `arms_ros2_control_msgs/WbcCurrentState` | 手柄 / 键盘 | 未收到时按底盘锁定处理 |

### 6.3 `/cmd_vel` 与 WBC

仅当 **FSM=OCS2 且 `BASE_UNLOCKED`** 时，遥操节点向 WBC 让出 `/cmd_vel`。HOLD、底盘锁定的 OCS2、分体（无 WBC 状态）仍由本节点发布。离开 OCS2 时由 WBC `stopBase()` 清零，而不是本节点。键盘在 ARM 模式下发的零速度，在 WBC 占用话题时也会跳过。

## 7. 依赖

- `rclcpp`、`sensor_msgs`、`geometry_msgs`、`std_msgs`、`control_msgs`、`rclcpp_action`、`rcutils`
- `arms_ros2_control_msgs`
- 运行时：`joy`（`joy_node` / `game_controller_node` / `joy_enumerate_devices`）
