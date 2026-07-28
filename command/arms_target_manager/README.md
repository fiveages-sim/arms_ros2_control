# ArmsTargetManager

机械臂目标管理器 - 提供 3D 交互式 marker、手柄适配与 VR 输入，用于设置末端目标。

## 功能特点

- **3D 交互式 marker**：在 RViz 中拖拽设置绝对目标 pose
- **单臂/双臂支持**
- **手柄适配**：订阅 `control_input`（`Inputs`），发布 `*/twist` 速度流；marker 仅可视化
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

RViz Joint Panel：绝对 / 相对基座 / 相对末端；相对基座填 `current_target`/`base_frame`；相对末端填 `left/right_ee_frame`。

## 参数说明

### 节点参数
- `dual_arm_mode`、`control_base_frame`、`hand_controllers`：由 launch / task.info 自动配置
- `marker_fixed_frame`：Marker 固定坐标系，默认 `base_link`

### YAML 配置（`config/default.yaml` 或 task 旁 `target_manager.yaml`）

| 参数 | 含义 | 默认 |
|------|------|------|
| `linear_scale` | 手柄满杆最大线速度 **m/s** | `0.25` |
| `angular_scale` | 手柄满杆最大角速度 **rad/s** | `2.5` |
| `control_input_rate` | 上游 `control_input` 典型频率 Hz（手感对齐估算） | `50.0` |
| `vr_thumbstick_linear_scale` | VR 摇杆线位移步进 m/step | `0.005` |
| `vr_thumbstick_angular_scale` | VR 摇杆角位移步进 rad/step | `0.05` |
| `enable_vr` | 是否启用 VR | `false` |

### 手柄 scale 与现网手感对齐

```text
scale_vel ≈ scale_old * f
```

例：`scale_old = 0.005`、`f = 50` → `linear_scale = 0.25`。

## 本包发布 / 订阅摘要

**发布**

- Marker / VR：`left_target`、`right_target`（绝对 Pose）
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
