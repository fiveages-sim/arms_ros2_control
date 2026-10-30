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
| `head_target` | `geometry_msgs/Pose` | WBC Head 6D 最终目标连续更新 | full-body Head marker |
| `head_target/stamped` | `geometry_msgs/PoseStamped` | WBC Head 6D 插值目标 | full-body Head marker 单次发送 |
| `head_current_target` | `geometry_msgs/PoseStamped` | WBC Head 最终目标回写；本节点只订阅 | Head marker |

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

这些参数属于 `ocs2_arm_controller` / `ocs2_wbc_controller`（**仅启动**），不是本节点参数。`ocs2_wbc_controller` 为私有子模块，此处仅转述已有说明。

| 参数 | 默认 | 生效时机 | 含义 |
|------|------|------|------|
| `base_frame` | task.info `baseFrame` | 仅启动 | 模型/参考基座（可被 YAML 覆盖） |
| `left_ee_frame` | task.info `eeFrame` | 仅启动 | 左末端 tip（YAML 可改为 `left_tcp` 等） |
| `right_ee_frame` | task.info `eeFrame1` | 仅启动 | 右末端 tip |
| `body_frame`（仅 WBC） | `bodyRelative.bodyLinkName` | 仅启动 | 身体 link |

- YAML 已配 → 用配置；未配 → 用 info，并写入 param 供外界读取。
- 覆盖在 **Interface 构造时内存注入** `createManipulatorModelInfo`（不写临时 info），MPC 真正跟踪新 tip。
- **不支持**像 `movel_duration` 那样运行时热改；改 tip 需重启控制器。`movel_*` 由控制器在下一次插值 / stamped 命令时 `get_parameter` 重读（**运行时**）。MOVEJ 侧 `cartesian_defaults.*` 同样在下一条 MOVEJ 命令时重读（**运行时**）。

RViz Joint Panel：绝对 / 相对基座 / 相对末端（手臂）或相对身体（Body TRACKING）；相对基座填 `current_target`/`base_frame`；相对末端填 `left/right_ee_frame`；相对身体填 `body_frame`。

### Full-body WBC Head 6D Marker

`enable_wbc_head_tracking_marker`（本节点参数，**仅启动**，默认 `false`）打开 WBC Head XYZ+RPY 六轴 Marker。共享 YAML 中保持关闭；`full_body.launch.py` 只在实际选择 `ocs2_wbc_controller` 时覆盖，`split_body` 不覆盖。Marker 还受运行态门控：只有 FSM=OCS2 且 `WbcCurrentState.head_state=HEAD_TRACKING` 时插入，离开模式立即移除并停止发布。头部相机注视双手中点激活时（`head_midpoint_gaze_active=true`）`head_state` 会随之变为 `HEAD_GAZE`，Marker 走同一门控自动隐藏，VR 头部输入同样停止，无需单独处理。

每次进入该模式，Marker 先从 `marker_fixed_frame -> head_link_name` TF 同步当前完整位姿；模式内 Marker 表示最终目标，不持续跟随实际 Head 或 MPC 中间轨迹。自身发布后的短暂回显冷却避免 `head_target -> head_current_target -> Marker` 抖动，其他最终目标随后可通过 `head_current_target` 回写 Marker。

## 参数说明

本节点在 `main` 里 `declare_parameter` 后把值传入构造函数，**没有** `add_on_set_parameters_callback`，之后也不再 `get_parameter`。下表全部为 **仅启动**（改参数需重启节点）。无 **运行时** / **未核实** 项。

### 节点参数

| 参数 | 含义 | 默认 | 生效时机 |
|------|------|------|------|
| `dual_arm_mode` | 双臂模式；由 launch / task.info 自动配置 | `false` | 仅启动 |
| `control_base_frame` | 控制基座坐标系 | `"world"` | 仅启动 |
| `hand_controllers` | 手部控制器名列表（用于检测/校验） | `[]` | 仅启动 |
| `marker_fixed_frame` | Marker 固定坐标系 | `"base_link"` | 仅启动 |
| `enable_movej_cartesian_markers` | MOVEJ 下是否显示可拖手臂 marker（发 stamped → IK MoveL）。`full_body.launch.py` 在 `ocs2_wbc_controller` 下设为 `false` | 有 `lina_planning` 时 `true`，否则 `false` | 仅启动 |
| `enable_wbc_head_tracking_marker` | 是否具备 full-body WBC Head 6D Marker 启动能力；仍需 OCS2 + `HEAD_TRACKING` | `false` | 仅启动 |

### YAML 配置（`config/default.yaml` 打底 + task 旁 `target_manager.yaml` 覆盖）

启动参数由 `robot_common_launch` 的 `prepare_arms_target_manager_parameters()` 组装，
**按以下顺序**（ROS 2 后加载的覆盖先加载的）：

1. `arms_target_manager/config/default.yaml` —— **基底**，所有可调项的默认值都写在这里；
2. task `.info` 同目录下的 `target_manager.yaml`（如 `w2_description/config/ocs2/target_manager.yaml`）
   —— 只写该机型要改的项；也可用 `config_file_path` 显式指定替代它；
3. launch 侧强制覆盖的字典（`dual_arm_mode` / `control_base_frame` / `marker_fixed_frame` /
   `hand_controllers` / `enable_movej_cartesian_markers` / `enable_wbc_head_tracking_marker` / `use_sim_time`）。

> ⚠️ 因此：
> - 改**所有机型通用**的默认值 → 改 `config/default.yaml`；
> - 改**某个机型**的值 → 改该机型的 `target_manager.yaml`（它会覆盖基底）；
> - 如果某机型**没有** task 旁的 `target_manager.yaml`，则只用基底文件。
> - 用 `ros2 param get /arms_target_manager <name>` 可以确认实际生效值（避免改错文件）。

| 参数 | 含义 | 默认 | 生效时机 |
|------|------|------|------|
| `linear_scale` | 映射到 `*/twist` 的线速度上限 **m/s**（满杆） | `0.1` | 仅启动 |
| `angular_scale` | 映射到 `*/twist` 的角速度上限 **rad/s**（满杆） | `0.25` | 仅启动 |
| `control_input_rate` | 上游 `control_input` 典型频率 Hz（仅手感估算，不参与实时换算） | `50.0` | 仅启动 |
| `vr_thumbstick_linear_scale` | VR 摇杆线位移步进 m/step | `0.005` | 仅启动 |
| `vr_thumbstick_angular_scale` | VR 摇杆角位移步进 rad/step | `0.05` | 仅启动 |
| `vr_pose_scale` | VR 手柄位姿缩放（可由组合键 15 校准） | `1.0` | 仅启动 |
| `vr_follow_frame` | 相对增量路径冻结旋转所用的跟随系 | `"base_footprint"` | 仅启动 |
| `body_ee_source` | 全身绝对跟踪的末端数据源：`hand` / `wrist` | `"hand"` | 仅启动 |
| `body_length_scale` | 身长尺度：`pelvis→spine3`（并同尺度推 `spine3→head`） | `1.0` | 仅启动 |
| `shoulder_width_scale` | 肩宽尺度：`spine3→shoulder(L/R)` | `1.0` | 仅启动 |
| `arm_length_scale` | 臂长尺度：`shoulder→末端(hand/wrist)` | `1.0` | 仅启动 |
| ~~`absolute_map_yaw_deg`~~ | **已删除**：映射按「人体系」锚定——人体各量先表达在人体根自身坐标系里，人体朝向 ≡ 机器人机头朝向，无需人工偏航常数 | — | — |
| `hand_to_ee_rpy_deg_left` / `_right` | 手部坐标系 → 末端坐标系的固定偏置（固定轴 RPY，度） | `[-90,0,0]` / `[90,0,0]` | 仅启动 |
| `body_src_to_ee_rpy_deg` / `head_src_to_ee_rpy_deg` | 胸部/头部数据源 → 目标坐标系的固定偏置（度） | `[0,0,0]` | 仅启动 |
| `absolute_filter_alpha` | 绝对目标一阶低通系数（0=不滤波） | `0.3` | 仅启动 |
| `capture_settle_ms` | 标定前等待姿态稳定时长 (ms) | `200.0` | 仅启动 |
| `skeleton_timeout` | 骨架单关节数据看门狗 (s)，超时只冻结对应任务 | `0.2` | 仅启动 |
| ~~`human_root_filter_alpha`~~ / ~~`human_root_offset`~~ | **已迁移**到 `vr_pose_publisher`：`XR_HUMAN_FOOTPRINT_FILTER_ALPHA` / `XR_HUMAN_FOOTPRINT_OFFSET`（或 `XRTargetNode` 构造参数） | — | — |
| `body_length_scale` | 身长尺度：`pelvis→spine3`（并同尺度推 `spine3→head`） | `1.0` | 仅启动 |
| `shoulder_width_scale` | 肩宽尺度：`spine3→shoulder(L/R)` | `1.0` | 仅启动 |
| `arm_length_scale` | 臂长尺度：`shoulder→末端(hand/wrist)` | `1.0` | 仅启动 |
| `skeleton_geometry_tolerance` | 手–脊柱距离相对参考臂长的允许偏差；`<=0` 关闭 | `0.2` | 仅启动 |
| `absolute_ramp_enable` | 接管渐入总开关；关掉后行为与旧版完全一致 | `true` | 仅启动 |
| `absolute_ramp_max_linear_velocity` | 过渡段线速度上限 (m/s) | `0.15` | 仅启动 |
| `absolute_ramp_max_angular_velocity` | 过渡段角速度上限 (rad/s) | `0.4` | 仅启动 |
| `absolute_ramp_min_duration` | 最短过渡时长 (s) | `0.2` | 仅启动 |
| `absolute_ramp_max_duration` | 最长过渡时长 (s)；触顶时实际峰值速度会高于上限 | `3.0` | 仅启动 |
| `absolute_ramp_resume_gap` | 目标输出中断超过该时长后重新渐入 (s) | `0.3` | 仅启动 |
| `base_follow_enable` | 底盘跟踪总开关（左摇杆按键仍需在全身模式内） | `true` | 仅启动 |
| `base_follow_rate` | 底盘位置外环频率 (Hz) | `50.0` | 仅启动 |
| `base_follow_tau` | 软参考「跑步机」时间常数 (s)；稳态增益 ≈ `kp*tau` | `2.0` | 仅启动 |
| `base_follow_kp` | 位置增益（默认 0.5×2.0 = 稳态 1:1） | `0.5` | 仅启动 |
| `base_follow_kpsi` | 偏航增益（人体转身 1:1） | `1.0` | 仅启动 |
| `base_follow_deadband` / `base_follow_yaw_deadband` | 位置死区 (m) / 偏航死区 (rad) | `0.02` / `0.05` | 仅启动 |
| `base_vmax` / `base_wmax` | 线速度上限 (m/s) / 角速度上限 (rad/s) | `0.4` / `0.8` | 仅启动 |
| `base_acc_max` / `base_alpha_max` | 线加速度上限 (m/s²) / 角加速度上限 (rad/s²) | `1.0` / `2.0` | 仅启动 |
| `base_cmd_tau` | 底盘指令一阶低通 (s) | `0.2` | 仅启动 |
| `base_follow_data_timeout` | 人体 footprint 数据看门狗 (s) | `0.2` | 仅启动 |
| `enable_vr` | 是否启用 VR | `false` | 仅启动 |

> `base_follow_tau_yaw` 目前为预留参数（偏航采用绝对跟踪，不使用软参考）。

### 全身遥操作模式（组合键 30）内的允许操作

| 操作 | 事件 | 行为 |
|------|------|------|
| 左右握把 + 右摇杆按下 | `30` | 进入/退出全身遥操作模式（发 `BODY_TRACKING` + `HEAD_TRACKING`） |
| 右摇杆短按 | `4` | 开/关「全部跟踪」：胸/头/双臂末端改为 `/teleop/body_*` 绝对位姿映射 |
| 左摇杆按键 | `7` | 开/关「底盘跟踪」：人体 footprint → 位置外环 → `/cmd_vel` |
| 右 A / 左 X | `11` / `12` | FSM 前进 / 后退（与普通模式共享） |
| 扳机 / 握把+扳机 | `9` / `10` / `13` / `14` | 夹爪控制（与普通模式共享） |
| FSM 离开 OCS2 / 拓扑离开 FULL_BODY | — | **自动退出整个模式**（「全部跟踪」「底盘跟踪」一并关闭），回到 OCS2 后需重新按组合键开启 |

**普通模式的其他组合键在全身模式下全部不可用**（含 `3`/`6` 单臂、`16` Y/B 语义、`20` 底盘遥控、`21–28` 身体/WBC 开关）。需要这些功能时先按 `30` 退出全身模式。

非全身模式下 `4` 仍是 UPDATE/STORAGE 切换、`7` 仍是镜像切换；`31` 已随上游长按功能删除。
「底盘跟踪」独立于「全部跟踪」，`BASE_UNLOCKED` 时冻结跟随并把 `/cmd_vel` 让给 WBC。
自动退出与普通遥操作「离开 OCS2 自动回 STORAGE」保持一致。

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

> 本节仅适用于**普通 VR 遥操作模式**。进入全身遥操作（组合键 30）后，`3`/`6`/`16`/`20`/`21–28` 全部不可用，只剩 FSM、夹爪、`4`、`7`。

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
- WBC Head Marker：`head_target`（连续）或 `head_target/stamped`（单次）
- 手柄适配：`left_target/twist`、`right_target/twist`

**订阅**

- `control_input`（`arms_ros2_control_msgs/Inputs`）
- `left_current_pose` / `right_current_pose`（更新 marker；frame_id=base）
- `head_current_target`（仅 WBC Head 6D 最终目标回写）
- VR 相关话题（启用时）

## 依赖

- rclcpp
- geometry_msgs
- visualization_msgs
- interactive_markers
