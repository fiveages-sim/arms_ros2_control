# base_pose_sim

把 `cmd_vel` 积分成底盘平面位姿，向 TF 树补一条 `world→base` 变换。

## 为什么需要它

无物理仿真（`mock_components`）链路里没有任何节点发布 `world→base_footprint`，
而依赖底盘自由度的规划/控制链通常只在启动后探测一次这条 TF，探测失败就会把底盘
当固定基座处理并永久锁死。本节点用一个纯运动学积分补上这条变换，让整条链路可用。

**它不是什么**：没有质量、惯性、摩擦、碰撞，也不解算轮系（转向角、轮速、打滑）。
底盘会穿墙、轮子不会转。低速卡滞、舵轮对齐死区、轮胎打滑这类问题**无法**用它复现，
必须用带物理的仿真。

## 话题

| 方向 | 话题 | 说明 |
|---|---|---|
| 订阅 | `cmd_vel`（`geometry_msgs/Twist`） | 机体系速度，`linear.x/y`、`angular.z` |
| 发布 | `/tf` | `world→base`；`publish_odom=true` 时改为 `odom→base` |
| 发布 | `/tf_static` | 仅 `publish_odom=true`：一次性锁存 `world→odom` |
| 发布 | `odom`（`nav_msgs/Odometry`） | 仅 `publish_odom=true` |

TF 话题始终是绝对 `/tf`：所有广播器都发布在绝对名上，本节点不参与命名空间隔离。
也不发 `base→link` 的固定关节，那是 `robot_state_publisher` 的职责。

## 支持的底盘形式

两种模型共用同一份积分公式，只是可用自由度不同：

$$
\begin{aligned}
x &\mathrel{+}= (v_x\cos\psi - v_y\sin\psi)\,\Delta t\\
y &\mathrel{+}= (v_x\sin\psi + v_y\cos\psi)\,\Delta t\\
\psi &\mathrel{+}= \omega\,\Delta t
\end{aligned}
$$

| `base_model` | 使用的分量 | 典型底盘 |
|---|---|---|
| `omni`（默认） | $v_x, v_y, \omega$ | 全向轮、麦克纳姆、四轮独立转向（舵轮） |
| `differential` | $v_x, \omega$（$v_y$ 强制为 0） | 二轮差速、履带、单舵轮 |

差速底盘天然只收到 $v_x, \omega$，积分结果与 `omni` 完全一致；把 `base_model`
配成 `differential` 的额外价值是**收到非零 $v_y$ 时给出限频告警**，用来暴露"模型选错"
或"上游算错"，而不是静默吞掉。

对应关系参考规划配置里的底盘类型：差速式 → `differential`，全向式 → `omni`，
固定基座不需要本节点。

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `cmd_vel_topic` | `cmd_vel` | 速度指令话题（相对名） |
| `base_model` | `omni` | `omni` / `differential`；非法值回退 `omni` 并报错 |
| `world_frame` | `world` | 世界系 |
| `odom_frame` | `odom` | 仅 `publish_odom=true` 时有意义 |
| `base_frame` | `base_footprint` | 底盘参考系 |
| `base_z` | `0.0` | `base_frame` 离地高度；参考系取 `base_link` 时填台面高度 |
| `publish_odom` | `false` | `true` 时额外发 `world→odom`（静态）+ `odom→base` + `/odom` |
| `publish_rate` | `50.0` | 积分与发布频率 (Hz) |
| `cmd_timeout` | `0.25` | 指令中断判定 (s)，超时按零速处理 |
| `max_linear_velocity` | `0.0` | 线速度上限 (m/s)，`0` = 不限 |
| `max_angular_velocity` | `0.0` | 角速度上限 (rad/s)，`0` = 不限 |
| `initial_x` / `initial_y` / `initial_yaw` | `0.0` | 启动位姿，用于与场景摆放对齐 |

`publish_odom=true` 时 `odom` 原点锁存在启动位姿上，`/odom` 从零开始，
与常见仿真器的行为一致。

## 运行

```bash
ros2 run base_pose_sim base_pose_sim_node

# 差速底盘
ros2 run base_pose_sim base_pose_sim_node --ros-args -p base_model:=differential

# 需要 nav2 时
ros2 run base_pose_sim base_pose_sim_node --ros-args -p publish_odom:=true
```

通常由上层 launch 在**控制器之前**拉起：控制器对这条 TF 的探测窗口有限
（`mobile_base_tf_timeout_sec`），先起节点才能被探测到。

## 注意

- 与真实位姿源（物理仿真、定位）**不能同时运行**，两者发同一父子对会让 TF 来回跳。
- 真机上不要启用：会与定位节点抢同一条变换。
- 时钟跳变时单步积分被钳在 0.5 s，避免重置仿真时间后位姿瞬移。
