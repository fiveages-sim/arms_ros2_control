# Arms RViz Control Plugin

面向 OCS2 / WBC 机械臂控制的 RViz2 面板集合：FSM 切换、关节与末端目标、夹爪、以及 WBC 能力与状态。

在 RViz 中通过 `Panels` → `Add New Panel` 添加，类名前缀均为 `arms_rviz_control_plugin/`。

| Panel 类名 | 用途 |
|---|---|
| `OCS2FSMPanel` | FSM 状态切换；可选 WBC 模式控制 |
| `JointControlPanel` | MOVEJ 关节目标 / OCS2 末端绝对·相对目标；腰部点动 |
| `GripperControlPanel` | 夹爪开关与位置百分比 |
| `WbcCapabilityPanel` | 只读：WBC 能力标志 |
| `WbcCurrentStatePanel` | WBC 当前状态显示与模式切换 |

## 目录

- [1. 安装](#1-安装)
- [2. OCS2FSMPanel](#2-ocs2fsmpanel)
  - [2.1 FSM 命令](#21-fsm-命令fsm_commandstd_msgsint32)
  - [2.2 转换规则](#22-转换规则)
  - [2.3 WBC（可选）](#23-wbc可选)
  - [2.4 话题](#24-话题)
- [3. JointControlPanel](#3-jointcontrolpanel)
  - [3.1 分类](#31-分类)
  - [3.2 显示单位](#32-显示单位)
  - [3.3 OCS2 末端位姿](#33-ocs2-末端位姿left--rightcommand3)
  - [3.4 MOVEJ 关节目标](#34-movej-关节目标command4)
  - [3.5 腰部（Body）](#35-腰部body)
  - [3.6 主要话题](#36-主要话题)
- [4. GripperControlPanel](#4-grippercontrolpanel)
- [5. WbcCapabilityPanel](#5-wbccapabilitypanel)
- [6. WbcCurrentStatePanel](#6-wbccurrentstatepanel)
- [7. 使用步骤](#7-使用步骤)
- [8. 说明](#8-说明)

---

## 1. 安装

```bash
cd ~/ros2_ws
colcon build --packages-up-to arms_rviz_control_plugin --symlink-install
source install/setup.bash
```

依赖：`rclcpp`、`rviz_common`、`sensor_msgs`、`geometry_msgs`、`tf2`、`arms_controller_common`、`arms_ros2_control_msgs`（以及对应 Qt）。

---

## 2. OCS2FSMPanel

智能 FSM 切换面板。启动默认 **HOLD**，仅显示当前状态允许的转换按钮。

### 2.1 FSM 命令（`/fsm_command`，`std_msgs/Int32`）

| command | 含义 | 典型按钮 |
|---:|---|---|
| 1 | HOME | HOLD → HOME |
| 2 | HOLD | HOME/OCS2/MOVEJ → HOLD |
| 3 | OCS2（MPC） | HOLD → OCS2 |
| 4 | MOVEJ / HOME 下切换 Home↔Rest 姿态 | HOLD → MOVEJ；HOME 下「切换姿态」 |

### 2.2 转换规则

1. **HOME**：可到 HOLD；可多次切换 Home/Rest 姿态（command=4）
2. **HOLD**：可到 OCS2、MOVEJ 或 HOME
3. **OCS2 / MOVEJ**：只能回到 HOLD

### 2.3 WBC（可选）

当节点参数 `wbc_available:=true`（检测到 `ocs2_wbc_controller`）时，在 OCS2 状态下显示 WBC 控件：底盘、双臂耦合、左右臂使能、身体模式等，经 `mode_command` 下发。

### 2.4 话题

| 方向 | 话题 | 类型 |
|---|---|---|
| 发布 / 订阅 | `/fsm_command` | `std_msgs/Int32` |
| 发布 | `mode_command` | `std_msgs/String`（WBC） |
| 订阅 | `/ocs2_wbc_controller/wbc_capabilities` | `WbcCapability` |
| 订阅 | `/ocs2_wbc_controller/current_state` | `WbcCurrentState` |

---

## 3. JointControlPanel

在 **OCS2（command=3）** 或 **MOVEJ（command=4）** 下发送目标；订阅 `/fsm_command` 与 `/joint_states`，按当前状态与分类显示控件。

### 3.1 分类

根据控制器与关节名自动出现：`全部` / `Body` / `Head` / `Left` / `Right` / `Left Hand` / `Right Hand`。  
`ocs2_wbc_controller` 可覆盖 body/head/left/right；单臂无 left/right 前缀时归到 **Left**，对接 `left_target`。

### 3.2 显示单位

| 选项 | 长度 | 角度 |
|---|---|---|
| **米 / 弧度** | m | rad |
| **厘米 / 角度** | cm | deg |

内部与 ROS 消息始终为 **m / rad**；面板只做显示换算。配置键 `DisplayUnit`（兼容旧键 `AngleUnit`）。

### 3.3 OCS2 末端位姿（Left / Right，command=3）

位姿类型：

| 模式 | 含义 | 发送话题 |
|---|---|---|
| **绝对** | 世界/基座系下目标位姿 | `left_target/stamped` / `right_target/stamped`（`PoseStamped`） |
| **相对基座** | 相对基座（或 `current_target` 坐标系）增量 | `…/relative`（`TwistStamped`） |
| **相对末端** | 相对 EE 坐标系增量（需控制器 `left/right_ee_frame`） | 同上 |

**绝对目标的姿态 UI 随显示单位变化：**

| 显示单位 | 绝对姿态输入 | 线上消息 |
|---|---|---|
| 米 / 弧度 | `qx, qy, qz, qw`（四元数） | 直接写入 `PoseStamped.orientation` |
| 厘米 / 角度 | `roll, pitch, yaw`（度） | 经 **OrientZYX**（`Rz·Ry·Rx`，同 ABB / `tf2::setRPY`）转为四元数再发送 |

回填当前目标时：厘米/角度用 **EulerZYX**（`tf2::Matrix3x3::getRPY`）把四元数拆成 RPY。

相对模式 UI 始终为 `x, y, z, roll, pitch, yaw`（单位随显示模式）。勾选「发送后保持输入」则相对发送后不清空数值。

绝对模式会订阅 `left_current_target` / `right_current_target` 回填；相对模式不覆盖用户输入。

### 3.4 MOVEJ 关节目标（command=4）

按分类向对应控制器发布 `Float64MultiArray`，例如：

- `/<controller>/target_joint_position`
- `/ocs2_wbc_controller/target_joint_position/{left\|right\|body\|head}`
- 单臂 MoveJ：`/ocs2_arm_controller/target_joint_position`（无 `/left` 子话题）

关节顺序优先取控制器 `joints` 参数；并做 URDF 限位裁剪。

### 3.5 腰部（Body）

**MOVEJ / 非追踪**：Body 分类下可点动升降/旋转（长按连发），发布 `waist_lifting/turning_command`；也可发送关节目标。

**OCS2 + BODY_TRACKING（全身跟随）**：与手臂同构的笛卡尔位姿 UI（共用「位姿」下拉）：

| 模式 | 行为 | 话题 |
|---|---|---|
| 绝对 | 身体目标位姿 | `body_target/stamped` |
| 相对基座 | 一次 SE(3) 增量 | `body_target/relative`（`frame_id`=base） |
| 相对身体 | 相对 `body_frame` 增量 | `body_target/relative`（`frame_id`=`body_frame`） |

显示条件：`FSM=OCS2` 且 `WbcCurrentState.body_state=BODY_TRACKING(2)`（订阅 `/ocs2_wbc_controller/current_state`）。  
绝对姿态单位约定与手臂相同（米/弧度→四元数；厘米/角度→RPY / OrientZYX）。回填订阅 `body_current_target`。  
此时隐藏 Body 关节行与腰部点动，避免与笛卡尔跟踪冲突。

### 3.6 主要话题

| 方向 | 话题 | 类型 | 场景 |
|---|---|---|---|
| 订阅 | `/fsm_command` | `Int32` | 显隐与模式 |
| 订阅 | `/ocs2_wbc_controller/current_state` | `WbcCurrentState` | Body TRACKING 门控 |
| 订阅 | `/joint_states` | `JointState` | 关节初值 |
| 订阅 | `robot_description` | `String` | 关节类型 / 限位 |
| 订阅 | `left_current_target` / `right_current_target` | `PoseStamped` | 手臂绝对回填 |
| 订阅 | `body_current_target` | `PoseStamped` | 身体绝对回填（TRACKING） |
| 发布 | `left_target/stamped` / `right_target/stamped` | `PoseStamped` | OCS2 手臂绝对 |
| 发布 | `left_target/relative` / `right_target/relative` | `TwistStamped` | OCS2 手臂相对 |
| 发布 | `body_target/stamped` | `PoseStamped` | OCS2 身体绝对（TRACKING） |
| 发布 | `body_target/relative` | `TwistStamped` | OCS2 身体相对（TRACKING） |
| 发布 | `/<controller>/target_joint_position[…]` | `Float64MultiArray` | MOVEJ / 其它分类 |
| 发布 | 腰部 lifting / turning | `Float64` | Body 点动（非 TRACKING） |

---

## 4. GripperControlPanel

自动发现手部/夹爪控制器，提供开关与位置（0~1）发送。

| 方向 | 话题 | 类型 |
|---|---|---|
| 发布 / 订阅 | `/<controller>/target_command` | `Int32`（开合同步） |
| 发布 | `/<controller>/target_percent` | `Float64`（0~1） |

---

## 5. WbcCapabilityPanel

只读显示 WBC 能力（移动底盘、身体相对约束、腰锁、自定义关节锁、双臂耦合、身体跟踪等）。

- 订阅：`/ocs2_wbc_controller/wbc_capabilities`（`WbcCapability`，transient_local）

---

## 6. WbcCurrentStatePanel

显示并切换 WBC 当前模式（底盘、双臂、左右臂、身体模式等）。

| 方向 | 话题 | 类型 |
|---|---|---|
| 发布 | `mode_command` | `String` |
| 订阅 | `/ocs2_wbc_controller/wbc_capabilities` | `WbcCapability` |
| 订阅 | `/ocs2_wbc_controller/current_state` | `WbcCurrentState` |

> `OCS2FSMPanel` 在 WBC 可用时也内嵌了同类控件；本面板可单独放置以便观察/操作。

---

## 7. 使用步骤

1. 启动带 OCS2/WBC 的机器人 bringup 与 RViz2。
2. 添加需要的 Panel（至少 `OCS2FSMPanel`；末端/关节调试再加 `JointControlPanel`）。
3. 用 FSM 面板进入 **OCS2** 或 **MOVEJ**。
4. 在关节面板选择分类与显示单位，编辑目标后点击发送。

可选：`ros2 launch arms_rviz_control_plugin test_panel.launch.py`（包内测试 launch）。

---

## 8. 说明

- Joint 面板的绝对目标在 **厘米/角度** 下使用 ZYX 欧拉角（ABB `OrientZYX` / `EulerZYX`）；**米/弧度** 下保持原生四元数输入。
- 相对增量的角速度分量约定为 RPY（与控制器 `PoseBasedReferenceManager` 一致：`dq = Rz·Ry·Rx`）。
- Body TRACKING 笛卡尔目标与腰部 Float64 点动分离：追踪时用 `body_target/*`，点动仅用于 MOVEJ / 非追踪场景。
- 旧 README 中的 `/control_input` 已废弃，现统一使用 `/fsm_command`。
