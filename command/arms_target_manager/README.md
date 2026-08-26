# ArmsTargetManager

机械臂目标管理器 - 提供 3D 交互式 marker、手柄适配与 VR 输入，用于设置末端目标。

## 功能特点

- **3D 交互式 marker**：在 RViz 中拖拽设置绝对目标 pose
- **单臂/双臂支持**
- **手柄适配**：订阅 `control_input`（`Inputs`），按 `linear_scale`/`angular_scale` 放大为 `*/twist` 速度（m/s、rad/s）；marker 仅可视化
- **VR**：仍发绝对 `left_target` / `right_target`（本阶段不迁相对话题）

## 使用方法

### 1. 编译包

```bash
cd ~/ros2_ws
colcon build --packages-up-to arms_target_manager
```

### 2. 启动节点

通过 OCS2 控制器 launch 文件启动（会自动包含 ArmsTargetManager）：

```bash
ros2 launch ocs2_arm_controller demo.launch.py robot:=cr5
```

### 3. 在 RViz 中查看

1. 启动 RViz
2. 添加 InteractiveMarkers 显示
3. 设置 Fixed Frame 为配置的 frame_id
4. 拖拽 marker 设置目标 pose

## PoseBasedReferenceManager 入站话题（左臂；右臂对称）

| Topic | 消息 | 行为 | 主要客户端 |
|-------|------|------|------------|
| `left_target` | `geometry_msgs/Pose` | 立即绝对 | VR / marker 连续 |
| `left_target/stamped` | `geometry_msgs/PoseStamped` | 绝对 **moveL**；`frame_id` 非 base 则 TF→base | RViz 绝对 |
| `left_target/twist` | `geometry_msgs/Twist` | **速度流**（m/s、rad/s），latch + `dt` 积分 | **手柄** |
| `left_target/relative` | `geometry_msgs/TwistStamped` | **一次相对位移**（m、rad）+ moveL；`frame_id` 选坐标系 | **RViz 相对基座/末端** |
| `left_current_target` | `PoseStamped` | 当前指令目标反馈（`frame_id`=base） | RViz / 可视化 |
| `body_target` | `geometry_msgs/Pose` | 立即绝对（需 `body_target_enabled`） | VR / marker |
| `body_target/stamped` | `geometry_msgs/PoseStamped` | 绝对 **moveL** | RViz 身体绝对（TRACKING） |
| `body_target/relative` | `geometry_msgs/TwistStamped` | **一次相对位移** + moveL；`frame_id`=base/`body_frame` | **RViz 相对基座/身体** |
| `body_current_target` | `PoseStamped` | 身体当前指令目标反馈 | RViz / marker |

**MOVEJ 下同一 stamped 话题**：`PoseBasedReferenceManager` 是 `left/right/dual_target/stamped` 与 TF buffer 的唯一持有者。FSM 为 OCS2 时写入参考缓冲；非 OCS2 时转给 `StateMoveJ` 走 `startLinearTrajectory`（lina MoveL + 逐点 IK），语义同 `execute_linear`。vel/acc/jerk 留空则用控制器 `cartesian_defaults`（默认 `max_linear_velocity=0.25`，`ik_type=AUTO`，`time_mode=false`）。无 `lina_planning` 时 MOVEJ 侧为 no-op。

### MOVEJ 手臂 Marker（需 `lina_planning`）

有 `lina_planning` 且 **MOVEJ 接了 IK MoveL**（`ocs2_arm_controller`）时，默认 `disable_auto_update_states = {3, 4}`。切到 MOVEJ（`/fsm_command=4`）后：

1. 左右臂 marker 可拖，只改可视化。
2. 菜单「发送目标」发一次 `*/stamped` → StateMoveJ IK MoveL。「发送双臂」走 `dual_target/stamped`（3 个 pose 时忽略 body）。
3. 菜单不显示「切换到连续发布」；进入 MOVEJ 时若已是连续模式，按单次处理。
4. Python `send_target_stamped` 发同一 stamped 话题；须保持 FSM=MOVEJ（关闭位姿类自动切 OCS2），才会走 IK MoveL。RViz Joint 面板 MOVEJ 仍只发关节数组，不走 stamped。

`ocs2_wbc_controller` 的 MOVEJ **没有** IK MoveL（只走关节）。全身 launch 会设 `enable_movej_cartesian_markers:=false`，MOVEJ 下手臂 marker 不显示、跟实际位姿，与改前相同。无 `lina_planning` 时默认同样不显示。

要点：

- `relative` 用 **TwistStamped**：`twist` 为位移增量；`header.frame_id` 为表达该增量的坐标系（base / EE link / 其它 TF）。非 base 时控制器将 `linear`/`angular` TF 旋到 base 再合成。
- `twist` 话题仍为裸 `Twist`（速度语义，仅基座）。
- `angular.{x,y,z}` 对应 roll / pitch / yaw（合成：`R' = RΔ(yaw)·RΔ(pitch)·RΔ(roll)·R`）。
- `left_current_pose` / `left_current_target` 的 `frame_id` 是 **base（或 WBC 轮式 world）**，不是 EE link 名。

### 控制器 frame 参数（加载时一次生效）

`ocs2_arm_controller` / `ocs2_wbc_controller`：

| 参数 | 默认 | 含义 |
|------|------|------|
| `base_frame` | task.info `baseFrame` | 模型/参考基座（可被 YAML 覆盖） |
| `left_ee_frame` | task.info `eeFrame` | 左末端 tip（YAML 可改为 `left_tcp` 等） |
| `right_ee_frame` | task.info `eeFrame1` | 右末端 tip |
| `body_frame`（仅 WBC） | `bodyRelative.bodyLinkName` | 身体 link |

- YAML 已配 → 用配置；未配 → 用 info，并写入 param 供外界读取。
- 覆盖在 **Interface 构造时内存注入** `createManipulatorModelInfo`（不写临时 info），MPC 真正跟踪新 tip。
- **不支持**像 `movel_duration` 那样运行时热改；改 tip 需重启控制器。

RViz Joint Panel：绝对 / 相对基座 / 相对末端（手臂）或相对身体（Body TRACKING）；相对基座填 `current_target`/`base_frame`；相对末端填 `left/right_ee_frame`；相对身体填 `body_frame`。

## 参数说明

### 节点参数
- `dual_arm_mode`、`control_base_frame`、`hand_controllers`：由 launch / task.info 自动配置
- `marker_fixed_frame`：Marker 固定坐标系，默认 `base_link`
- `enable_movej_cartesian_markers`：MOVEJ 下是否显示可拖手臂 marker（发 stamped → IK MoveL）。有 `lina_planning` 时默认 `true`；`full_body.launch.py` 在 `ocs2_wbc_controller` 下设为 `false`

### YAML 配置（`config/default.yaml` 或 task 旁 `target_manager.yaml`）

| 参数 | 含义 | 默认 |
|------|------|------|
| `linear_scale` | 映射到 `*/twist` 的线速度上限 **m/s**（满杆） | `0.1` |
| `angular_scale` | 映射到 `*/twist` 的角速度上限 **rad/s**（满杆） | `0.25` |
| `control_input_rate` | 上游 `control_input` 典型频率 Hz（仅手感估算，不参与实时换算） | `50.0` |
| `vr_thumbstick_linear_scale` | VR 摇杆线位移步进 m/step | `0.005` |
| `vr_thumbstick_angular_scale` | VR 摇杆角位移步进 rad/step | `0.05` |
| `enable_vr` | 是否启用 VR | `false` |

### 手柄 / 键盘 scale（速度，不是每帧位移）

`ControlInputHandler` 将 `control_input`（`Inputs`）适配为 `left_target/twist` / `right_target/twist`：

```text
twist.linear.{x,y,z}  = Inputs.{x,y,z}     * linear_scale     # 单位：m/s
twist.angular.{x,y,z} = Inputs.{roll,pitch,yaw} * angular_scale  # 单位：rad/s
```

- **`linear_scale` / `angular_scale` 的单位就是 Twist 速度单位**（m/s、rad/s），不是旧版「每条消息位移步进」的 m/step、rad/step。
- `Inputs` 轴量大致在 `[-1, 1]`（再乘 teleop 侧 `speed_scale`）；满杆时末端指令速度约为上述 scale。
- 各机器人可在自身 `target_manager.yaml` 覆盖默认值。
- OCS2 对 `*/twist` 做 latch + `dt` 积分；需要更快/更慢时直接调大/调小这两个 scale。

## VR 面键的单臂控制

| 控制拓扑 | 非镜像：左 Y | 非镜像：右 B | 镜像：左 Y | 镜像：右 B |
|---|---|---|---|---|
| FULL_BODY（默认） | 左臂 ENABLE/DISABLE | 右臂 ENABLE/DISABLE | 右臂 ENABLE/DISABLE | 左臂 ENABLE/DISABLE |
| FULL_BODY（事件 16 后） | 左臂 VR 暂停/恢复 | 右臂 VR 暂停/恢复 | 右臂 VR 暂停/恢复 | 左臂 VR 暂停/恢复 |
| SPLIT_BODY | 左臂 VR 暂停/恢复 | 右臂 VR 暂停/恢复 | 右臂 VR 暂停/恢复 | 左臂 VR 暂停/恢复 |
| UNKNOWN | 忽略 | 忽略 | 忽略 | 忽略 |

- FULL_BODY 默认 Y/B 为 ENABLE/DISABLE，不依赖 VR UPDATE/STORAGE，但要求 VR enabled、FSM=OCS2。
- 双臂耦合时禁止通过 Y/B 单独切换任一臂。
- 事件 16（左右握把 + 左 Y + 右 B）在全身下锁存切换为暂停语义（需 UPDATE）；再按 16 切回禁用。SPLIT_BODY 忽略 16。
- 切换时：已禁用臂 → ENABLE 后暂停；已暂停臂 → DISABLE 后清暂停；未动过的臂不变。双臂耦合时不能把暂停语义切回禁用。
- 暂停语义下右摇杆进 UPDATE 不再清暂停。
- 禁用语义下 Y/B 与 case 25–26、28 复用现有 WBC pending、2 秒超时和右摇杆回中安全门；case 27 预留，当前不处理。

### FULL_BODY 单臂禁用后的 VR 恢复

- WBC 确认某臂为 `ARM_DISABLED` 后，VR 停止发布映射到该臂的 pose 目标，也停止累积该臂摇杆 offset；另一臂保持可控。
- 禁用期间可自由移动对应 VR 手柄。重新启用确认后，机器人基准取 `observation_.state` 对应的 `left/right_current_pose`，VR 基准取手柄当前位置，并清零该臂摇杆 offset。
- rebase 成功前该臂保持抑制；current pose 或 TF 暂不可用时不会回退到禁用前 target。
- `left_current_target` / `right_current_target` 可能因参考管理器同步发布而刷新时间戳；禁用臂 pose 数值应保持不受 VR 输入影响。
- 镜像模式只交换物理手柄到实际机械臂的映射，WBC enabled/disabled 和 rebase 状态始终按实际机械臂保存。

## 本包发布 / 订阅摘要

**发布**

- Marker 单次「发送目标」：`left_target/stamped`、`right_target/stamped`（`PoseStamped`）；双臂耦合：`dual_target/stamped`
- Marker 连续 / VR：`left_target`、`right_target`（绝对 Pose；MOVEJ 下不发）
- 手柄适配：`left_target/twist`、`right_target/twist`

**订阅**

- `control_input`（`arms_ros2_control_msgs/Inputs`）
- `left_current_pose` / `right_current_pose`（更新 marker；frame_id=base）
- VR 相关话题（启用时）

## 依赖

- rclcpp
- geometry_msgs
- visualization_msgs
- interactive_markers
