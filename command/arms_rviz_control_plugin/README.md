# Arms RViz Control Plugin

面向 OCS2 / WBC 机械臂控制的 RViz2 面板集合：FSM 切换、关节与末端目标、夹爪、以及 WBC 能力。

在 RViz 中通过 `Panels` → `Add New Panel` 添加，类名前缀均为 `arms_rviz_control_plugin/`。

| Panel 类名 | 用途 |
|---|---|
| `OCS2FSMPanel` | FSM 状态切换；可选 WBC 模式控制 |
| `JointControlPanel` | MOVEJ 关节目标 / OCS2 末端绝对·相对目标；腰部点动 |
| `GripperControlPanel` | 夹爪开关与位置百分比 |
| `WbcCapabilityPanel` | 只读：WBC 能力标志 |

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
- [6. 使用步骤](#6-使用步骤)
- [7. 说明](#7-说明)

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

当节点参数 `wbc_available:=true`（检测到 `ocs2_wbc_controller`）时，在 OCS2 状态下显示 WBC 控件：底盘、双臂耦合、左右臂使能、HOME 关节参考、身体模式等，经 `mode_command` 下发。

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

## 6. 使用步骤

1. 启动带 OCS2/WBC 的机器人 bringup 与 RViz2。
2. 添加需要的 Panel（至少 `OCS2FSMPanel`；末端/关节调试再加 `JointControlPanel`）。
3. 用 FSM 面板进入 **OCS2** 或 **MOVEJ**。
4. 在关节面板选择分类与显示单位，编辑目标后点击发送。

可选：`ros2 launch arms_rviz_control_plugin test_panel.launch.py`（包内测试 launch）。

---

## 7. 说明

- Joint 面板的绝对目标在 **厘米/角度** 下使用 ZYX 欧拉角（ABB `OrientZYX` / `EulerZYX`）；**米/弧度** 下保持原生四元数输入。
- 相对增量的角速度分量约定为 RPY（与控制器 `PoseBasedReferenceManager` 一致：`dq = Rz·Ry·Rx`）。
- Body TRACKING 笛卡尔目标与腰部 Float64 点动分离：追踪时用 `body_target/*`，点动仅用于 MOVEJ / 非追踪场景。
- 旧 README 中的 `/control_input` 已废弃，现统一使用 `/fsm_command`。

### 三维视窗中的目标跟踪误差 HUD

HUD 作为 `arms_rviz_control_plugin/TargetErrorDisplay` 显示在 RViz 网格视图右上角，
不再位于 JointControlPanel，也不依赖该控制面板存在。
W2 的 fullbody/splitbody 和通用 humanoid 配置已启用。
已有用户配置可在 Displays → Add → TargetErrorDisplay 添加，用 Displays 复选框开关。

独立订阅 left/right/body/head_current_pose 与对应的 current_target，以及
`/ocs2_wbc_controller/current_state`。没有命令发布器，不影响机器人控制。
以最高 10 Hz 更新显示内容，仅在文字或颜色变化时更新 Ogre 文字元素、实测 1 秒超时，目标保留最后一帧；不同 frame 或无效位姿不计算误差。
只显示已启用的手臂（ARM_ENABLED）、位姿跟踪中的 BODY（BODY_TRACKING，且非 HEAD_FORWARD）和 HEAD（HEAD_TRACKING）。未启用部分隐藏并收拢空位，切换时清空对应曲线；未收到模式状态时全部隐藏。模式订阅使用 transient-local，支持 RViz 晚于控制器启动。

位置 ≤5 mm 为绿色、≥20 mm 为红色；姿态 ≤1°为绿色、≥5°为红色，中间黄色。
无效或超时数值灰色，数值显示破折号。颜色阈值调整入口隐藏，Display 配置中仍保存阈值。
使用 RViz 底层 Ogre Overlay 的原生 BorderPanel 与 TextArea 元素，每个末端使用独立细边框、完全透明且无填充的背景、大数字和单位标签，无 QWidget 叠加；文字使用原生 Ogre 元素，趋势线使用小尺寸透明纹理。LEFT ARM / RIGHT ARM / BODY / HEAD 分别表示左臂、右臂、躯干和头部；POS (mm) / ANG (deg) 分别为位置和姿态误差。使用 RViz 自带 Liberation Sans Bold 字体并一次性生成高清字形纹理，不需要额外插件。有效误差仅通过数值颜色表示等级，不显示等级文案；无效数据保留具体原因，未启用跟踪时隐藏整个框。固定标签为黑色粗体。数值不截断。此显示为屏幕叠加，不随 Grid 的相机缩放或旋转。

每个框的位置、姿态数字背后各叠加最近 5 秒的半透明误差曲线，最多 5 Hz 更新。横轴按实际时间滚动；纵轴固定为 0 到对应红色阈值的两倍，红色虚线表示红色阈值，超出上限时在顶部显示短标记。无效数据或采样间隔超过 0.6 秒时断线，RViz Reset 清空历史。不显示时间窗口标注，曲线不另占高度；数字保持不透明并在曲线上层显示，单位沿用下方标签。

HUD 以控制器实际反馈 `/fsm_state`（transient-local）为总开关，仅状态 3（OCS2）计算和显示误差。HOME、HOLD、MOVEJ、未知状态或尚未收到状态时隐藏并跳过误差/曲线更新；状态切换清空曲线并等待新的实际位姿。保留目标缓存以兼容仅在目标变化时发布的话题；非 OCS2 模式直接丢弃实际位姿回调数据。
