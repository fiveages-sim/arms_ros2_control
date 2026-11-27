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
- [8. ComplianceForcePanel — 轨迹规划标签页](#8-complianceforcepanel--轨迹规划折线标签页)

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
已有用户配置可在 Displays → Add → TargetErrorDisplay 添加，用 Displays 复选框开关。
该 Display 的 `Scale` 属性（0.5–2.0，默认 1.0）调节叠加层大小，并随 RViz 配置保存。

独立订阅 left/right/body/head_current_pose 与对应的 current_target，以及
`/ocs2_wbc_controller/current_state`。没有命令发布器，不影响机器人控制。
以最高 10 Hz 更新显示内容，仅在文字或颜色变化时更新 Ogre 文字元素、实测 1 秒超时，目标保留最后一帧；不同 frame 或无效位姿不计算误差。
全身 WBC 收到 `/ocs2_wbc_controller/current_state` 后，只显示已启用的手臂（ARM_ENABLED）、位姿跟踪中的 BODY（BODY_TRACKING，且非 HEAD_FORWARD）和 HEAD（HEAD_TRACKING）。分体、双臂和单臂走 `ocs2_arm_controller`、没有这条状态时，OCS2 期间显示左臂；右臂、身体、头仅在收到对应 `current_target` 后显示。未启用部分隐藏并收拢空位，切换时清空对应曲线。模式订阅使用 transient-local，支持 RViz 晚于控制器启动。

位置 ≤5 mm 为绿色、≥20 mm 为红色；姿态 ≤1°为绿色、≥5°为红色，中间黄色。
无效或超时数值灰色，数值显示破折号。颜色阈值调整入口隐藏，Display 配置中仍保存阈值。
使用 RViz 底层 Ogre Overlay 的原生 BorderPanel 与 TextArea 元素，每个末端使用独立细边框、完全透明且无填充的背景、大数字和单位标签，无 QWidget 叠加；文字使用原生 Ogre 元素，趋势线使用小尺寸透明纹理。仅左臂时标题为 ARM，出现右臂后为 LEFT ARM / RIGHT ARM；BODY / HEAD 分别表示躯干和头部；POS (mm) / ANG (deg) 分别为位置和姿态误差。使用 RViz 自带 Liberation Sans Bold 字体并一次性生成高清字形纹理，不需要额外插件。有效误差仅通过数值颜色表示等级，不显示等级文案；无效数据保留具体原因，未启用跟踪时隐藏整个框。固定标签为黑色粗体。数值不截断。此显示为屏幕叠加，不随 Grid 的相机缩放或旋转。

每个框的位置、姿态数字背后各叠加最近 5 秒的半透明误差曲线，最多 5 Hz 更新。横轴按实际时间滚动；纵轴固定为 0 到对应红色阈值的两倍，红色虚线表示红色阈值，超出上限时在顶部显示短标记。无效数据或采样间隔超过 0.6 秒时断线，RViz Reset 清空历史。不显示时间窗口标注，曲线不另占高度；数字保持不透明并在曲线上层显示，单位沿用下方标签。

HUD 以控制器实际反馈 `/fsm_state`（transient-local）为总开关，仅状态 3（OCS2）计算和显示误差。HOME、HOLD、MOVEJ、未知状态或尚未收到状态时隐藏并跳过误差/曲线更新；状态切换清空曲线并等待新的实际位姿。保留目标缓存以兼容仅在目标变化时发布的话题；非 OCS2 模式直接丢弃实际位姿回调数据。

---

## 8. ComplianceForcePanel — 轨迹规划（折线）标签页

`arms_rviz_control_plugin/ComplianceForcePanel` 的第三个标签页，把多个脚点 + 一个
总运行时间打包成一条 `arms_ros2_control_msgs/CompliancePath` 一次性下发。

### 8.1 前置条件

1. 控制器处于 **COMPLIANCE**（`/fsm_command` = `5`）。面板的顺序不能反：控制器的
   `StateCompliance::enter()` 会清掉已排队的路径，所以必须**先进 COMPLIANCE 再下发**。
2. `compliance_task_selection` 必须 **全为 0**（纯位置控制）。机型 YAML 默认是
   `[1,0,0,0,0,0]`，第 0 轴是力控（导纳），那一轴不会跟随轨迹。

#### 8.1.1 脚点坐标系 = RViz 的 Fixed Frame

脚点默认**不写死在 `base_link`**，而是启动时取 RViz 当前的 Fixed Frame：

| 启动方式 | Fixed Frame | 来源 |
|---|---|---|
| `split_body.launch.py` | `base_footprint`（或 task 配置的控制系） | `splitbody.rviz:667`，启动时由 `patch_rviz_fixed_frame()` 改写 |
| `full_body.launch.py` | `world`（WBC）或 task 的 `control_base_frame` | `fullbody.rviz:773`，启动时被 `full_body.launch.py:141-157` 改写 |

所以**不管分体还是全身、不管哪个机型，面板里的坐标系和三维视图看到的是同一个系**，
不用手动改。运行中在 RViz 里换 Fixed Frame 也会跟着换；一旦手动编辑过这个输入框，
就不再自动覆盖。

下发时控制器会把 `header.frame_id` 转到 `compliance_teleop_base_frame`（默认
`base_link`）：两者相同时**直接短路、不查 TF**；不同时查 `lookupTransform`，查不到
则**整条路径被拒绝**（`COMPLIANCE path: <side> rejected (第 N 个角点 TF … → base 失败)`）。

### 8.2 界面

| 控件 | 说明 |
|------|------|
| 手臂 / 坐标系 | `左臂`\|`右臂`；坐标系**默认跟随 RViz 的 Fixed Frame**（见 8.1.1），也可手动改写 |
| 脚点输入框 | 多行文本，**一个矩阵 = 一条路径**，格式见 8.2.2；边打字边解析 |
| 组 | 下拉框列出本次输入里的所有路径，切换它只换「看/发」的是哪一组，不改文本；只有一组时置灰。组名取矩阵上方紧邻的注释，没写就叫 `路径 N` |
| 解析结果表 | 只读，8 列：`#`、`X/Y/Z [m]`、`Qx/Qy/Qz/Qw`（4 位小数）—— 就是格式化后的输入数据，与下发内容一一对应。姿态在 RViz 里看（每个点画工具坐标系三轴）；悬停某行可看该点的 RPY |
| 总时长 | 整条折线的总运行时间（含全部圈数，**不含**从当前位姿到起点的入段） |
| 圈数 | `loops`，`duration` 是全部圈数的总时间。**开链折线按往返执行**：2 圈 = 去 + 回，3 圈停在远端 |
| 添加点 | 追加一个点：矩阵排版时插在收尾括号之前并补好逗号，单行矩阵时插在最后一个点后面 |
| 删除选中 | 三种都支持：① **结果表里选中若干行**（Shift/Ctrl 多选）→ 精确删掉这几点对应的 7 个数，矩阵排版和同一行里的其它点都不动；② **输入框里选中几行文字** → 删掉这些整行（含注释、括号）；③ 什么都没选 → 删光标所在行。结果表选中后**直接按 `Delete` / `Backspace` 也一样**（表格只读，Qt 默认不处理这两个键，面板自己接上了） |
| 上移 / 下移 | 作用在**光标所在行**上（不能移动矩阵的收尾括号行） |
| 取当前位姿 | 把当前 TCP 位姿写成 `X Y Z Qx Qy Qz Qw` 替掉光标行（文本为空时追加） |
| 清空 | 清空输入框 |
| 下发轨迹 | **先检查 `/…_compliance_path` 有没有订阅者** —— 控制器的订阅只在 COMPLIANCE 状态下存在，HOLD / HOME 下消息会被静默丢弃，所以无订阅者时直接拒绝并提示（不会假装发送成功）。有订阅者才二次确认后发布；提示行随后显示**控制器侧的真实结果**（排队 → 入段 → 回放中 → 已完成，或被拒绝/取消的原因） |
| 停止（停在原地） | 把当前实测位姿发到 `/left_target`，抢占并取消正在回放的路径 |

摘要行实时显示：当前组、脚点数、段数、单趟长度、圈数、总长度、总时长、平均速度；
有解析失败的行时会在末尾标红并给出第一个错误。

> **总时长不能压得太短。** 控制器按 `总长度 / 时长` 反解速度，超过它的平移速度
> 上限（默认 0.15 m/s）会拒绝执行并在日志里给出最短可行时长，例如：
> `duration 2.00 s 太短：这条路径最短需要 3.51 s（速度上限 150 mm/s，按此时长反解需 263 mm/s）`

### 8.2.2 脚点文本格式（矩阵 = 路径）

**一个矩阵 = 一条路径。** 一条路径写成「外层方括号里套着一行行方括号」，
并列写多个矩阵就是多条路径，用「组」下拉切换下发哪一条：

```
# 接近                      ← 紧邻矩阵上方的注释会成为这一组的名字
[
  [X Y Z Qx Qy Qz Qw],
  [X Y Z Qx Qy Qz Qw]
]

# 扫查一圈
[[0.712401, -0.549088, 0.949855, 0.704416, 0.061629, -0.704416, -0.061629],
 [0.594506, -0.299951, 0.959314, 0.706093, 0.056995, -0.705031, -0.053765],
 [0.667822, -0.138390, 0.965221, 0.706360, 0.054103, -0.705454, -0.049896]]
```

每个点 7 个数：`X Y Z` 位置 [m] + `Qx Qy Qz Qw` 单位四元数。

**分组只看括号套了几层**，所以几种排版都能吃：

| 输入 | 结果 |
|------|------|
| `[[…], […]]` | 1 组 |
| 两个并列矩阵（中间空行/注释都行） | 2 组 |
| 每行一个 `[…]`（单层），或裸的 7 个数 | 1 组（旧的粘贴方式，行为不变） |
| 裸行 + 矩阵混写 | 2 组：裸行是第 1 组，矩阵是第 2 组 |
| 少写一个 `]` | 仍然按"到文本结尾"收尾，能解析 |

**只接受四元数，不接受 RPY 欧拉角。** 欧拉角 → 四元数 → 欧拉角 的往返本身
就带转换误差，而且在 pitch = ±90°（下探姿态）附近有万向锁多解：同一个位姿能
写出无数组 `R P Y`，抄错一个数看不出来。四元数是位姿的唯一表示，写什么就是
什么。

**解析是宽松的**：

- 括号 `[]` `()` `{}` 全部忽略，逗号 / 分号 / 空格 / Tab 都算分隔符
- 行尾多余的 `,` 无所谓，矩阵末尾的尾逗号也无所谓
- **一行里可以放多个点**（该行数量是 **7 的整数倍**就行）
- 四元数自动归一化；长度为 0 的点被剔除
- 6 个数（旧的 RPY 写法）会报错 `第 N 行有 6 个数（应为 7 的整数倍：x y z qx qy qz qw）`

**错误只影响所在的那一组。** 写坏一组的某个点时，该组仍能下发的其它点照常
保留，坏组在下拉框里标 `⚠`（悬停看具体原因）；下发时只校验当前选中的那一组，
别的组写坏了不拦你。

`取当前位姿` 也是按 7 个数写出（6 位小数，位置 µm 级、四元数 1e-6 级），
取出来后可以直接再粘回去，数值不损失。

#### 位置单位固定为 m

X/Y/Z 一律按**米**解析，没有单位下拉、也不做任何自动换算。

唯一的保护是一条量级检查：任何位置分量 `|x| > 20 m` 就判定这一行不是米制
（机器人工作空间 < 2 m），该行被剔除、摘要标红并拒绝下发：

```
← 1 处有问题（第 1 行 X = 712.401 超出 20 m，位置必须是米（若为毫米请先除以 1000））
```

这样既不会因为"猜量级"把手臂送到 712 米外，也不会悄悄乘个 0.001 让人以为单位对了。

解析结果（坐标 + 四元数）显示在下方的只读表里，同一份结果同时驱动
RViz 里的 Marker 和真正下发的 `compliance_path`，三处不会不一致。

摘要行实时显示：脚点数、段数、单趟长度、圈数、总长度、总时长、平均速度。

### 8.2.1 三维视图同步显示
面板会把脚点表实时画到 RViz 三维视图，话题 `/compliance_path_markers`
（`visualization_msgs/MarkerArray`）：

| 元素 | 含义 |
|------|------|
| 球 | 脚点。**起点绿、终点红、中间蓝** |
| 折线 | 相邻脚点之间的直线段 |
| 三轴短线 | 该点的**工具坐标系**（X 红 / Y 绿 / Z 蓝，各 8 cm）—— 球只能看出“走到哪”，姿态对不对得看这三个轴 |
| 橙色数字 | 脚点序号，与表格的 `#` 列一致 |
| 灰色细线 + 灰字 | **其它未选中的路径组**：只画轮廓和组名，方便对照位置；切到哪一组就画哪一组的完整符号 |

任何编辑（增删行、改数值、上移下移、取当前位姿、清空、切手臂、改坐标系）都会
立刻重发；另有 2 s 心跳，保证后加的 Marker 显示也能拿到。

`fullbody.rviz` / `splitbody.rviz` 里已经加了名为 `CompliancePath` 的
MarkerArray 显示，加载即生效。自己配 RViz 时：**Add → By topic →
`/compliance_path_markers` → MarkerArray**。

注意 Marker 用的是表格里的 `坐标系` 字段作 `frame_id`，填了不存在的框架会
导致整个 MarkerArray 被 RViz 丢在 TF 错误里（显示不出来）。

### 8.3 下发的字段

面板固定发：

```
blend_radius = 0        # 折线模式：每段各自一条梯形曲线，角点停到 0 再转向
speed = 0, accel = 0    # 交给控制器名义值，再由 duration 等比缩放
lat_accel = 0
closed = false
loops = 界面圈数
duration = 界面总时长
```

控制器侧的语义见 `arms_ros2_control_msgs/msg/CompliancePath.msg`：

- `blend_radius = 0` 时**每段独立加减速**，段末速度归零 → 角点停到 0 再转向，
  速度方向不突变，法向加速度恒为 0，没有切角误差。代价是每个中间角点多花 `v/a` 秒。
- `duration > 0` 时按 `v→v/k`、`a→a/k²` 等比缩放命中总时长（梯形形状不变，所以
  时长严格 `k` 倍）。时长拉长 → 加速度按平方下降 → 角点冲过头更小
  （`e ≈ a·τe/K_eff`，辨识值 `τe=38.3 ms`、`K_eff=4.71 1/s`）。

### 8.4 观察

- 规划结果：控制器日志 `COMPLIANCE path: <side> queued — …（圆角 0）… / 每段停车`
- 规划图形：RViz 三维视图里的球/折线/序号（见 8.2.1）
- 进度：`/left_current_target`、`/right_current_target`（20 Hz）
- 完成：日志 `COMPLIANCE path: <side> done`，之后目标保持在终点
- 中止：日志 `COMPLIANCE path: <side> cancelled (跟不上（跟踪误差超限）)` 等

### 8.5 限制

- **只适合少量真实脚点**：每个中间点都会真的停住，把一段直线/圆弧离散成很多点发过来
  会变成一步一顿。
- 停到 0 会让部分关节**换向**，背隙/摩擦会在角点体现为横向偏差。这一点当前辨识没有
  建模，需要实测确认。
- 停稳后约 3~4 个周期（≈0.4 s）残余振动才衰减完；紧接着反向时会叠在起步上。
