# VRInputHandler - VR输入处理器

## 概述

VRInputHandler是基于VRMarkerWrapper功能开发的VR输入处理器，专门为arms_target_manager设计。它实现了VR pose订阅、状态切换机制和marker更新功能。

### 普通模式 / 全身模式分离

从本版起，VR 遥操作拆成两个**物理独立**的组件：

- `VRInputHandler`：普通 VR 遥操作 + 共享的 FSM（11/12）与夹爪（9/10/13/14 + 扳机模拟量），并负责按键/摇杆事件的路由；
- `WholeBodyVrTeleop`（`WholeBodyVrTeleop.h/.cpp`）：全身骨架遥操作（case 30），独占 `/teleop/body_*` 与 `/teleop/human_footprint_pose` 订阅，只提供事件 4（全部跟踪）与事件 7（底盘跟踪）。

进入全身模式后，普通模式的组合键（`3`/`6`/`16`/`20`/`21–28`）全部不可用，必须先退出全身模式。两者只共享 FSM、夹爪与发布基础设施。

当 `ocs2_wbc_controller` 处于 active 时，拓扑判定为 **FULL_BODY（全身控制）**，普通模式下可在 VR 手柄上切换身体模式、底盘锁、双臂耦合和单臂启用。当 `ocs2_arm_controller` 处于 active 时为 **SPLIT_BODY（分体/双臂）**，普通模式的全身组合键无效。

## 功能特性

### 核心功能
- **VR Pose订阅**: 订阅VR左右臂的末端执行器pose
- **状态切换机制**: 通过右摇杆点击在存储模式和更新模式之间切换
- **连续目标**: 每个有效VR pose回调均发布目标；变化阈值代码暂时保留但不生效
- **差值计算**: 基于存储的base pose计算相对变化并应用到机器人
- **控制拓扑检测**: 启动时查询 `/controller_manager/list_controllers`，区分 FULL_BODY / SPLIT_BODY
- **全身模式命令**: FULL_BODY 下通过 `/mode_command` 请求 WBC 身体/底盘/双臂/单臂状态

### 工作模式

#### 存储模式 (Storage Mode)
- 默认模式，不更新marker
- 存储VR和机器人的当前pose作为base pose
- 用于设置参考点

#### 更新模式 (Update Mode)
- 基于存储的base pose计算VR pose的变化
- 将相同的变化应用到机器人base pose
- 更新marker位置
- 仅在 FSM = OCS2 时可由右摇杆切入

## 订阅的Topic

### 普通模式（`VRInputHandler`）
- `/teleop/left_ee_pose` (`geometry_msgs/msg/Pose`): VR左臂末端执行器pose
- `/teleop/right_ee_pose` (`geometry_msgs/msg/Pose`): VR右臂末端执行器pose
- `/teleop/head_pose` (`geometry_msgs/msg/Pose`): VR头显pose
- `/teleop/controller_state` (`std_msgs/msg/Int32`): 按钮/组合键事件码（单帧）
- `/teleop/thumbstick_axes` (`geometry_msgs/msg/Twist`): 左右摇杆轴值；`linear.z` / `angular.z` 携带左右握把按下状态
- `/teleop/trigger_values` (`geometry_msgs/msg/Twist`): 左右扳机模拟量

### 全身模式（`WholeBodyVrTeleop`）
- `/teleop/body_pelvis_pose` (`geometry_msgs/msg/Pose`): 骨架骨盆（胸部锚点 / 绝对映射）
- `/teleop/body_spine3_pose` (`geometry_msgs/msg/Pose`): 骨架胸椎上段（胸部绝对目标）
- `/teleop/body_head_pose` (`geometry_msgs/msg/Pose`): 骨架头部（头部绝对目标）
- `/teleop/body_left_hand_pose` / `body_right_hand_pose` (`geometry_msgs/msg/Pose`):
  骨架手部，双臂末端绝对目标（`body_ee_source=wrist` 时订阅 `*_wrist_pose`）
- `/teleop/body_left_shoulder_pose` / `body_right_shoulder_pose`: 肩宽/臂长尺度链起点
- `/teleop/human_footprint_pose`: 人体基坐标系 H

> 所有 `/teleop/body_*` 与 `/teleop/human_footprint_pose` 都是**动态话题**（头显开启全身追踪后才出现），`WholeBodyVrTeleop` 常驻订阅。

### 机器人状态
- `left_current_pose` (`geometry_msgs/msg/PoseStamped`): 机器人左臂当前pose
- `right_current_pose` (`geometry_msgs/msg/PoseStamped`): 机器人右臂当前pose
- `head_current_pose` (`geometry_msgs/msg/PoseStamped`): 机器人头部当前pose（头部跟踪的门控条件）

## 参数配置

### 构造函数参数
- `node`: ROS节点指针
- `targetManager`: ArmsTargetManager指针
- `updateRate`: 更新频率，默认500Hz

### 启动参数
- `enable_vr`: 是否启用VR控制，默认true
- `vr_update_rate`: VR更新频率，默认500.0Hz
- `vr_follow_frame`: VR 增量参考轴系，恢复时冻结 F → C 旋转，FULL_BODY / SPLIT_BODY 均生效，默认 `base_footprint`
- 全身绝对映射 / 底盘跟踪参数（`body_ee_source`、`absolute_*`、`capture_settle_ms`、`skeleton_*`、
  `base_follow_*`）见 `README.md` 的参数表，均只影响全身遥操作模式内部行为

### VR 目标坐标系

`vr_follow_frame` 指定 VR 增量的参考轴系 F；控制器坐标系 C 来自
current_pose 的 `header.frame_id`。进入 UPDATE、暂停恢复或单臂重新接管时，
每臂独立查询并冻结 F → C 的旋转 `R_C_F`。机器人锚点和目标缓存始终保存在 C 中。

```text
delta_p_F = inverse(R_v0) * (p_v - p_v0)
delta_R_F = inverse(R_v0) * R_v
p_target_C = p_anchor_C + R_C_F * delta_p_F
R_target_C = R_C_F * delta_R_F * inverse(R_C_F) * R_anchor_C
```

镜像、缩放和摇杆仍按原规则作用于 VR/F 中的增量。平移增量只旋转，
不叠加 TF 平移；连续目标在 C 中平滑并发布，不再每帧查询底盘 TF。
因此 base_footprint 移动或转向不会带动静止 VR 对应的 world target。

- 普通恢复优先使用最后发布的 C 中目标作为锚点，无历史目标时使用实际末端。
- WBC 手臂重新启用从实际末端重新接管；另一臂的冻结旋转不变。
- 普通暂停冻结 VR 输入，保持本轮旋转；摇杆仍可调整目标，恢复时才更新旋转。
- TF 不可用时基准无效，停止该臂发布并重试；成功时重新对齐 VR 起点、清零摇杆偏移，下一帧恢复发布。
- F 与 C 相同时使用单位旋转；split-body 同样遵循冻结增量轴系的规则。
- 校准和头部跟踪保留各自的实时 TF 语义；校准写入手臂缓存的是 C 中目标。
- `robot.local.yaml` 只选择 `info_file_name`，不配置本参数。

## 全身控制操作说明

VR 操作分为两套**互斥**模式：

- **普通 VR 遥操作**：UPDATE/STORAGE、镜像、单臂启用/暂停、身体模式、WBC 开关、CHASSIS 遥控等；
- **全身遥操作**（组合键 30）：只保留 FSM（11/12）、夹爪（9/10/13/14）、事件 4（全部跟踪）、事件 7（底盘跟踪）。

以下普通模式组合键仅在控制拓扑为 **FULL_BODY**（`ocs2_wbc_controller` active）、VR 已启用、且 FSM = **OCS2** 时生效。SPLIT_BODY 下按这些组合会被忽略；**进入全身遥操作后也全部不可用**，需先按组合键 30 退出。

方向组合由 `xr_target_node` 在握把按住且摇杆偏置超过 **0.7** 时发出；松开握把后，对应摇杆必须回到 **0.3** 以内，才会解除抑制，避免「先松握把、后松摇杆」误驱动末端或底盘。

### 推荐流程

1. 启动全身控制器（`ocs2_wbc_controller`）和 `arms_target_manager`（`enable_vr:=true`），并确认 `xr_target_node` 在线。
2. 用 **右 A** 把 FSM 推到 OCS2（HOME → HOLD → OCS2）；**左 X** 后退。
3. **（普通模式相对增量路径）**把双手柄放到期望起始姿态，点 **右摇杆** 进入 UPDATE，双臂开始跟随。
4. **（全身绝对映射路径）**按 **左右握把 + 右摇杆按下**（30）进入全身遥操作模式，再按 **右摇杆短按**（4）开启「全部跟踪」（骨架绝对位姿驱动胸/头/双臂）；需要底盘跟随时按 **左摇杆按键**（7）开关「底盘跟踪」。
5. **（普通模式）**用 **左握把 + 左摇杆四方向** 切换身体模式；用 **右握把 + 右摇杆上/下/右** 切换双臂耦合 / 底盘锁 / 参考关节。单臂启用默认用左 Y / 右 B；**左右握把 + 左 Y + 右 B**（事件 16）可把 Y/B 切到 VR 暂停语义。
6. **（普通模式）**进入 **跟随（BODY_TRACKING）** 后，左摇杆改控腰部，右摇杆仍控右臂。
7. **（普通模式）**需要底盘遥控时，同时按下左右摇杆进入 CHASSIS 模式。全身模式下没有手动底盘遥控，只有事件 7 的底盘跟踪。

> 以下“身体模式 / 腰部跟随 / WBC 开关 / 单臂启用 / 底盘遥控”各节均为**普通模式**行为，全身模式下不可用。

### 身体模式（左握把 + 左摇杆）

按住左握把，再把左摇杆推到对应方向（阈值 0.7）。请求发出后进入最多 **2 秒** 的 pending：左摇杆此时既不驱动腰部也不驱动左臂，直到 WBC 确认目标 `body_state` **并且** 左摇杆回中。超时且已回中则按 WBC 实际状态恢复路由。

| 操作 | 事件 | `/mode_command` | WBC `body_state` | 效果 |
|------|------|-----------------|------------------|------|
| 左握把 + 左摇杆**上** | 21 | `BODY_RELATIVE` | `BODY_VERTICAL` | 竖直：躯干相对底盘保持竖直软约束 |
| 左握把 + 左摇杆**下** | 22 | `BODY_LOCK` | `BODY_LOCKED` | 锁定：腰部关节速度锁死 |
| 左握把 + 左摇杆**左** | 23 | `BODY_TRACKING` | `BODY_TRACKING` | 跟随：腰部跟踪 `body_target`，可用左摇杆微调 |
| 左握把 + 左摇杆**右** | 24 | `BODY_CUSTOM_LOCK` | `BODY_CUSTOM_LOCKED` | 自定义锁定：按 task.info 中的自定义关节锁 |

切到跟随时，若 marker 还不是连续发布，会自动切到 CONTINUOUS。

### 腰部跟随（BODY_TRACKING）

WBC 确认 `body_state = BODY_TRACKING` 且 FSM = OCS2 后：

- **左摇杆** 增量改 BodyMarker（不依赖双臂 UPDATE/STORAGE）。
- **右摇杆** 仍控制右臂。
- 默认 XY：摇杆 Y → 前后（X），摇杆 X → 左右（Y）。
- 点一下 **左握把**（短按松开，未超过 400 ms，且未被组合键消费）在 **XY** 与 **Z+Yaw** 之间切换：Y → 升降，X → 偏航。长按（包括组合键没发出去）不会切模式。
- 离开跟随后，腰部摇杆平面自动回到 XY；此时左握把短按恢复为「切换左臂摇杆 XY / Z+Yaw」。右握把短按同样切换对应臂的摇杆平面。

腰部控制不要求当前处于 UPDATE 模式。

### WBC 开关（右握把 + 右摇杆）

按住右握把，再推右摇杆。同样有 **2 秒 pending + 右摇杆回中** 安全门；pending 期间右摇杆轴值被清零。同一时间只允许一条右侧开关请求。

| 操作 | 事件 | 切换命令 | 效果 |
|------|------|----------|------|
| 右握把 + 右摇杆**上** | 25 | `ARMS_COUPLED` / `ARMS_INDEPENDENT` | 双臂耦合开/关 |
| 右握把 + 右摇杆**下** | 26 | `BASE_UNLOCK` / `BASE_LOCK` | 底盘解锁 / 锁定（WBC 层） |
| 右握把 + 右摇杆**左** | 27 | — | 预留，当前不处理 |
| 右握把 + 右摇杆**右** | 28 | `HOME_JOINT_ON` / `HOME_JOINT_OFF` | 参考关节追踪开/关 |

约束：

- 双臂已耦合时，禁止通过左 Y / 右 B 单独切换任一臂。
- 任意一臂禁用时，禁止开启双臂耦合。
- 无 `has_home_joint_reference` 时忽略事件 28。
- 左 Y / 右 B（禁用语义）与事件 25–26、28 复用同一套 pending / 超时 / 回中逻辑。事件 16 的状态对翻不走这条 pending。

### 单臂启用 / 禁用与 Y/B 语义锁存（左 Y / 右 B / 事件 16）

默认 FULL_BODY 下面键向 WBC 请求该臂 ENABLE/DISABLE，不依赖 UPDATE/STORAGE，但仍要求 VR enabled、FSM = OCS2。

**事件 16**（左右握把 + 左 Y + 右 B）在 FULL_BODY + OCS2 + VR enabled 下翻转锁存：Y/B 在「WBC 单臂禁用」与「VR 暂停/恢复」之间切换。SPLIT_BODY 忽略 16，Y/B 仍为暂停。

| 控制拓扑 | 非镜像：左 Y | 非镜像：右 B | 镜像：左 Y | 镜像：右 B |
|----------|--------------|--------------|------------|------------|
| FULL_BODY（默认，禁用语义） | 左臂 ENABLE/DISABLE | 右臂 ENABLE/DISABLE | 右臂 ENABLE/DISABLE | 左臂 ENABLE/DISABLE |
| FULL_BODY（事件 16 后，暂停语义） | 左臂 VR 暂停/恢复（需 UPDATE） | 右臂 VR 暂停/恢复（需 UPDATE） | 右臂 VR 暂停/恢复（需 UPDATE） | 左臂 VR 暂停/恢复（需 UPDATE） |
| SPLIT_BODY | 左臂 VR 暂停/恢复 | 右臂 VR 暂停/恢复 | 右臂 VR 暂停/恢复 | 左臂 VR 暂停/恢复 |

锁存切换时对翻已改变的臂，未动过的臂不变：

- 切到暂停语义：机械臂若为 `ARM_DISABLED`，发 `*_ARM_ENABLE`，rebase 成功后再冻结为暂停。
- 切回禁用语义：机械臂若已 VR 暂停，发 `*_ARM_DISABLE`，WBC 确认禁用后再清暂停。
- 双臂耦合时禁止 16 把暂停语义切回禁用（锁存不变）。WBC 开关或 16 转换仍在 pending 时，16 与 Y/B 均忽略。
- 暂停语义下右摇杆进入 UPDATE **不再清暂停**；已暂停臂会按当前手柄重新冻结，避免 `vr_base` 重写造成跳变。
- `disable()` 或拓扑离开 FULL_BODY 时锁存回到默认禁用语义。

WBC 确认某臂为 `ARM_DISABLED` 后：

- VR 停止向该臂发布 pose，也不再累积该臂摇杆 offset；另一臂仍可控制。
- 禁用期间可自由移动对应手柄。重新启用并确认后，机器人基准取当前 `left/right_current_pose`，VR 基准取手柄当前位置，并清零该臂摇杆 offset。
- rebase 成功前该臂保持抑制；current pose 或 TF 暂不可用时不会回退到禁用前的 target。
- 镜像只交换物理手柄到机械臂的映射；WBC enabled/disabled 与 rebase 始终按实际机械臂保存。

### 底盘遥控（左右摇杆同时按下）

同时按下左右摇杆帽（事件 20）在 **CHASSIS** 与末端控制之间切换。只要求 VR 已启用，不要求 FULL_BODY。进入时清零末端摇杆累积；退出时向底盘/腰部话题各发一次 0，防止残留运动。

CHASSIS 模式下摇杆不再驱动末端：

| 输入 | 默认 | 按住右握把（仅 SPLIT_BODY） |
|------|------|------------------------------|
| 左摇杆 Y / X | 底盘前后 / 左右（`/cmd_vel` linear.x / linear.y） | 同左 |
| 右摇杆 X | 底盘转向（`/cmd_vel` angular.z） | 腰部旋转（`waist_turning_command`） |
| 右摇杆 Y | 忽略 | 腰部升降（`waist_lifting_command`） |

FULL_BODY 下右握把+右摇杆方向组合仍会锁存并清零右轴（含预留的 27/28），因此 **CHASSIS 里不能再用右握把当腰部修饰符**。全身场景下请用：

- 事件 26：WBC 底盘解锁 / 锁定；
- 事件 23 + 左摇杆：腰部跟随微调；
- 事件 20 + 左摇杆 / 右摇杆 X：VR 直发 `/cmd_vel`。

这与事件 26 的 `BASE_UNLOCK` 不是同一层：前者是手柄遥控速度，后者是 MPC 是否允许底盘自由度。

`/cmd_vel` 交接：仅在 **FULL_BODY + FSM=OCS2 + `BASE_UNLOCKED`** 时 VR 不发 `/cmd_vel`（让给 WBC）。HOLD / HOME / MOVEJ，或 OCS2 里手动 `BASE_LOCK`，手柄底盘照常可用。SPLIT_BODY 不让路。退出 OCS2 的零速由 WBC `stopBase()` 负责；退出 CHASSIS / `disable()` 仍由 VR 清零。

### 全身控制速查（普通模式）

```
左握把 + 左摇杆上/下/左/右  →  竖直 / 锁定 / 跟随 / 自定义
右握把 + 右摇杆上/下        →  双臂耦合 / 底盘锁（左/右预留）
左 Y / 右 B                 →  默认对应臂 ENABLE/DISABLE；事件 16 后为 VR 暂停/恢复
左右握把 + 左 Y + 右 B      →  切换 Y/B 禁用 ↔ 暂停语义（SPLIT_BODY 忽略）
左右摇杆同时按下            →  底盘遥控 ↔ 末端控制
跟随模式下短按左握把        →  腰部摇杆 XY ↔ Z+Yaw
短按左/右握把               →  对应臂摇杆 XY ↔ Z+Yaw（长按或组合键不切换）
右摇杆点击                  →  STORAGE ↔ UPDATE（需 OCS2；全身模式内改为「全部跟踪」开关）
右 A / 左 X                 →  FSM 前进 / 后退
```

### 全身模式速查

```
左右握把 + 右摇杆按下       →  进入/退出全身遥操作模式
右摇杆短按（事件 4）        →  开/关「全部跟踪」（胸/头/双臂 = /teleop/body_* 绝对映射）
左摇杆按键（事件 7）        →  开/关「底盘跟踪」（人体 footprint → 位置外环 → /cmd_vel）
右 A / 左 X                 →  FSM 前进 / 后退（共享）
扳机 / 握把+扳机            →  夹爪控制（共享）
其他组合键                  →  不可用，需先退出全身模式
```

## 全身遥操作（全身骨骼数据驱动，组合键 case 30）

**组合键：左右握把 + 右摇杆按下**（事件码 30）。在 VR enabled + FULL_BODY（`ocs2_wbc_controller` active）+ FSM=OCS2 下开关：

- **开启（进入模式）**：自动请求身体模式「跟随」（`BODY_TRACKING`）与头部模式「跟踪」（`HEAD_TRACKING`）；
  此时**尚未下发胸/头/双臂目标**，需再按右摇杆（4）开启「全部跟踪」；
- **手动关闭**：再次按组合键退出模式，同时关掉「全部跟踪」与「底盘跟踪」（WBC 保持当前目标）。
- **自动退出**：权威 FSM 一离开 OCS2（HOLD / HOME / MOVEJ，含控制器侧异常自动回 `HOLD`）就退出整个模式，
  **「全部跟踪」与「底盘跟踪」一并关闭**，回到 OCS2 后需重新按组合键开启；
  判定同时放在 FSM 命令回调、按键/摇杆入口与 50 Hz 底盘外环里，因此 FSM 由任何来源切换
  （VR 按键、RViz OCS2 面板、手柄 `/fsm_command`、控制器自动回退）都能退出。
  控制拓扑不再为 FULL_BODY（如切成 `split_body`）时同样自动退出。

**模式内允许的用户操作只有四项**：FSM（11/12）、夹爪（9/10/13/14）、事件 4、事件 7。
`3`/`6`/`16`/`20`/`21–28` 等其他组合键在全身模式下**全部不可用**，必须先按 30 退出。

### 模式内开关：全部跟踪（右摇杆短按，事件 4）

右摇杆松开沿（上游发事件 `4`）在全身模式内改为 **开/关「全部跟踪」**
（toggle，与双臂末端同步启动）：

> ⚠️ **该模式内只有绝对映射，没有“相对运动模式”**。早期版本在这里还留着一套
> “开启时记录基准 → 跟踪相对变化量（冻结转态 / 冻结旋转 / 骨盆→spine3 偏移变化量）”
> 的分支，现**已删除**：按下右摇杆开跟踪后，胸/头/双臂直接采当前人体位姿映射，
> 不再依据按下那一刻的快照。未开跟踪时不下发任何目标（WBC 保持当前目标）。
> （底座上的相对增量（冻结基准）只保留给**非全身模式**：手柄驱动双臂、头显驱动 WBC 头部目标。）

### 目标独占（谁在跟踪期间会让路）

「全部跟踪」开启后，**只对"运行期输入"让路**——RViz 交互 marker 是调试通道，**不门控**。

| 输入路径 | 是否让路 | 说明 |
|---|---|---|
| 手柄/键盘 **`control_input`** | ✅ 让路 | `ControlInputHandler::processControlInput` 按 `isArmVrActive` 跳过 `*/twist` 发布与 marker 增量（夹爪命令仍处理）。它的 `*/twist` 会被参考管理器持续积分，与 VR 抢同一目标 |
| **左右摇杆微调** | ✅ 让路 | `thumbstickAxesCallback` 在全身模式下直接早退（见上文） |
| RViz **交互 marker 拖拽** | ❌ **不门控** | 调试通道：随时可手工干预。跟踪中拖拽会被 VR 下一帧覆盖（不影响 VR 控制） |
| marker 右键菜单 **「发送目标」/「发送双臂」** | ❌ **不门控** | 同上；跟踪中发送即被覆盖 |
| marker **增量更新**（`updateMarkerPoseIncremental` / `updateBodyMarkerPoseIncremental`） | ❌ **不门控** | 手柄走 `control_input` 的入口已在上游让路，这里不再重复拦 |
| RViz **joint_control_panel「发送姿态」** | ❌ **不门控** | 面板是调试工具 |
| **头部 marker 拖拽/菜单**（`head_vr_active_`） | ✅ 让路 | **既有设计、与本次无关**：头显驱动头部目标期间，头部 marker 保留目标显示但**拖动与菜单发送均被禁用**，释放后恢复交互（见本文档"头显驱动 WBC Head 6D 目标"） |

- 让路标志在 `ArmsTargetManager` 内部（`setArmVrActive`，仅左/右臂），不额外发话题。
- 退出跟踪 / 失能 / 退出全身模式 → 自动交还。
- **完全不受门控**：RViz marker/面板、`ros2 topic pub`、`ros2_robot_interface` 的 Python API、
  `record/record_playback.py` 回放等直接向 `left/right/body/head_target[*]` 发布的外部脚本。

- **开启时不做任何“按下瞬间的快照”**，而是逐帧按下面三步求值：

  ```
  ① 人体基坐标系 H（由 vr_pose_publisher 统一计算并发布到
       /teleop/human_footprint_pose）：
       位置 xy = **骨盆**在地面的投影（骨盆 xy）
       位置 z  = 两**足** z 的较低者（行走时支撑脚即地面）
       朝向   = 两**踝** x 轴在地平面投影的平均（直立、非内八外八时即人体前向）
       俯仰/滚转恒为 0， 即 H 永远是竖直系（与 base_footprint 同构）
       xy/z 一阶低通，最后加人体基坐标系自身系下的原点偏置
       （发布端参数 human_footprint_filter_alpha / human_footprint_offset）
  ② 各部位换算到 H： p_H = Rz(ψ_H)ᵀ·(p − p_H)，R_H = Rz(ψ_H)ᵀ·R
  ③ **骨架分段尺度**（在 H 里只缩放「段向量」，姿态完全不变）：
       p'spine3   = p_pelvis   + s_body    ·(p_spine3  − p_pelvis)
       p'head     = p'spine3   + s_body    ·(p_head    − p_spine3)
       p'shoulder = p'spine3   + s_shoulder·(p_shoulder− p_spine3)
       p'hand     = p'shoulder + s_arm     ·(p_hand    − p_shoulder)
  ④ 直接作为机器人对应部位「相对底座」的目标：
       位置_base = R_align · p'
       姿态_base = R_align · R_H · C_src→ee
     再乘底座位姿 → world：T_world = T_base_world · T_base
  ```

  胸部位置另有一个**锚点比例**：`p_H` 取「沿脊柱轴从 spine3 退到解剖对应点」，
  `body_anchor_lambda`（0 = spine3 位置，1 = 骨盆位置，默认 1）；**姿态始终由 spine3 决定**。

  三条关键语义：

  - **人体自身的行走与整体转身被 H 完全吸收**，只用来驱动底盘速度指令：
    底盘锁定时人走动 / 整体转身都不会拖动手臂、腰、头的目标；
  - **姿态与位置都是“当前值”而不是“相对按下时刻的增量”**：机器人头摆正、人体头侧向 30° 时，
    按下右摇杆开启跟踪后头目标**立即**就是侧向 30°；
  - **方向由“人自己的坐标系”给出**：人体手沿人体前向前伸→末端目标沿**底盘前向**移动，
    与机器人腕部当前姿态无关。
  （H 的原点在地面、轴向与 `base_footprint` 一致，所以人与机器人的目标可直接对应，位移偏置默认 0。）

- **骨架分段尺度（代替早期的单一绝对映射尺度）**：只对「段向量」缩放，
  **关节姿态完全不变、人体根与地面不动**。三个参数：

  | 参数 | 缩放对象 | >1 的含义与效果 |
  |---|---|---|
  | `body_length_scale` | `pelvis → spine3`（并同尺度推 `spine3 → head`） | 机器人更高/躯干更长。注意 **λ=1 时胸部位置取骨盆，所以它不影响胸目标**，只把头部与手臂根抬高/降低 |
  | `shoulder_width_scale` | `spine3 → shoulder(L/R)` | 机器人肩更宽。人双手前伸（臂向量只有前后分量）时，**末端间距 = s_shoulder × 人体肩宽** |
  | `arm_length_scale` | `shoulder → 末端(hand/wrist)` | 机器人臂更长。人臂完全伸直时机器人臂也完全伸直（取 机器人臂长/人体臂长） |

  段向量必须用**原始**端点作差、再挂到**变形后**的父节点上；
  变形顺序为 pelvis → spine3/head → shoulder → hand。
  某段数据缺失时该段尺度退回 1.0（节流告警），不影响其它段。
  肩部话题 `/teleop/body_{left,right}_shoulder_pose` 是标定的必要条件。

- **人体基坐标系为什么需要骨盆 + 踝 + 足**：VR 世界系原点是头显启动时的**头部**
  （z=0 在头高、地面在负侧），而又需要一个「永远竖直、在地面」的参考系才能与
  `base_footprint` 对应。骨盆提供水平位置，两踝提供朝向，两足提供地面高度（取较低者）。
  计算、低通与原点偏置都在 `vr_pose_publisher` 内完成；本节点只订阅
  `/teleop/human_footprint_pose`，话题断流/超时即冻结对应全身目标。

- **`T_base_world`（底座位姿）怎么来**：优先查 TF `vr_follow_frame → ee_frame_id`
  （移动底盘下即 `base_footprint → world`）；无 TF 时回退到**对自己下发 `/cmd_vel` 的积分估计**。
  回退路径下会打印节流告警；此时若底盘被打滑或外部（导航 / 手柄）驱动，手臂会有偏差。
  为什么需要它：底盘跟随前进时，目标必须跟着底座一起走，否则手臂会被向后拖长。

- 数据源（`/teleop/body_*` + `/teleop/human_footprint_pose`）：

  | 机器人目标 | 人体数据源（默认） | 固定偏置参数 |
  |---|---|---|
  | `left_tcp` / `right_tcp` | `body_left_hand_pose` / `body_right_hand_pose`（`body_ee_source=wrist` 时用 `*_wrist_pose`） | `hand_to_ee_rpy_deg_left` / `_right` |
  | `body_link4`（胸） | `body_spine3_pose` | `body_src_to_ee_rpy_deg` |
  | `head_link2` | `body_head_pose` | `head_src_to_ee_rpy_deg` |
  | （人体根 H） | `human_footprint_pose`（骨盆 xy + 双踝 yaw + 双足 z，由 vr_pose_publisher 计算） | — |
  | （胸部 λ 锚点） | `body_pelvis_pose` | — |

- **`C_src→ee` 为什么必需**：人体骨架坐标系与机器人目标坐标系（tcp/body_link4/head_link2）
  的轴向定义不同。例如手柄面朝上时，人手 x 轴向上、左手 z 轴向左 / 右手 z 轴向右；
  而 `left/right_tcp` 是 x 轴向上、z 轴向前（见 `m6_ccs_description/xacro/arm.xacro` 的
  `tcp_joint rpy="π/2 -π/2 0"`）。因此需要绕手部 x 轴 ∓90°：
  `hand_to_ee_rpy_deg_left=[-90,0,0]`、`hand_to_ee_rpy_deg_right=[90,0,0]`（默认值）。
  **换 `body_ee_source=wrist`、换机型（`eeFrame` 为 `*_eef` 等）时必须重调这三个角。**
  现场校准办法：人体保持标准姿态，看 RViz 里末端目标的坐标轴与人体朝向差多少，
  把差值填进对应的 `*_rpy_deg`（固定轴 RPY，度，`R = Rz(yaw)·Ry(pitch)·Rx(roll)`）。
- **无需偏航对齐参数**（早期 `absolute_map_yaw_deg` 已删除）：映射把人体各量先表达在人体根
  自身坐标系里，所以「人体朝向 ≡ 机器人机头朝向」是**构造成立**的——人在模式内转身只会
  去驱动底盘偏航，不会给上肢目标叠加固定角度偏差。

- **人体上半身姿态直接映射为机器人姿态**：胸转时手/头自动跟随（骨架绝对位姿已包含胸的运动），
  不需要额外的「胸转手补」处理；非全身模式的相对增量（冻结基准）路径完全不受影响；
- **接管瞬间不再阶跃**：开启「全部跟踪」后，胸/头/双臂目标会从机器人当前保持的位姿
  （优先取 1 s 内最后一次下发的目标，其次实测位姿），按下面的速度上限缓慢过渡到实时
  VR 期望位姿；过渡结束后恢复逐帧直通，稳态跟随手感不变。
  过渡时长由距离自动反算：`T = 1.875 · max(|Δp|/v, Δθ/ω)`，再钳到 `[min, max]`；
  进度用五次多项式（两端速度、加速度为 0），因此起手与交接都没有速度阶跃。
  过渡期间目标持续跟随**实时**期望位姿，不会“追一个冻结的旧终点”。
  骨架超时冻结后恢复（输出中断 > `absolute_ramp_resume_gap`）也会重新渐入。

  | 参数 | 默认 | 说明 |
  |---|---|---|
  | `absolute_ramp_enable` | `true` | 总开关（关掉即回到旧行为） |
  | `absolute_ramp_max_linear_velocity` | `0.15` m/s | 过渡段线速度上限 |
  | `absolute_ramp_max_angular_velocity` | `0.4` rad/s | 过渡段角速度上限 |
  | `absolute_ramp_min_duration` | `0.2` s | 最短过渡时长 |
  | `absolute_ramp_max_duration` | `3.0` s | 最长过渡时长；触顶时峰值速度会超上限（节流告警） |
  | `absolute_ramp_resume_gap` | `0.3` s | 输出中断超过该时长后重新渐入 |

- 可达自由度投影：胸部/头部目标按 ZYX 分解**丢弃 roll**（W2 腰部 3×pitch + 1×yaw、无 roll；
  头部仅 yaw/pitch）；
- **摇杆微调**：全身模式下不可用（摇杆完全让给骨架绝对映射）；普通模式下仍按原规则叠加在相对增量路径上。
- 数据质量：逐关节看门狗（`skeleton_timeout`）只冻结对应任务；一阶低通（`absolute_filter_alpha`）
  抑制骨架抖动；手–脊柱距离超出 `skeleton_geometry_tolerance` 的帧丢弃（冻结该臂）；
- 标定需要全部 5 个数据源就位（3 个机器人位姿 + pelvis/spine3/head/hand×2），
  未就位时按 2 s 节流日志列出缺失项；标定完成后才真正下发目标。

### 模式内开关：底盘跟踪（左摇杆按键，事件 7）

左摇杆按键（松开沿，上游发事件 `7`）在全身模式内 **开/关「底盘跟踪」**，
**独立于「全部跟踪」**（不要求后者已开）。普通模式下同一个 `7` 仍是镜像切换。

- 数据源：`/teleop/human_footprint_pose` 的水平位置 + 偏航（上游已完成低通与原点偏置）；
- 外环（默认 50 Hz）：

  ```
  Δp_human → R_z(-ψ_pelvis0) · Δp_human            # 转到接管时刻的机器人本体系 F
  p_ref    += (p_F - p_ref) · dt/τ                 # 软参考「跑步机」
  e        = p_F - p_ref
  v_F      = |e| > deadband ? clamp(kp·e, vmax) : 0
  ψ_err    = wrap(Δyaw_human - ψ_offset)           # 绝对偏航跟踪，ψ_offset 由自下发 ω 积分
  ω        = |ψ_err| > deadband ? clamp(kpsi·ψ_err, wmax) : 0
  → 一阶低通 + 速率限幅 → v_B = R_z(-ψ_offset)·v_F
  ```

- 稳态速度增益 ≈ `base_follow_kp * base_follow_tau`（默认 0.5×2.0 = 1:1）；
- 输出全向 `/cmd_vel`：`linear.x`、`linear.y`、`angular.z`；
- 零速退出：关闭开关、人体 footprint 数据超时（`base_follow_data_timeout`）、离开 OCS2 / FULL_BODY、
  `disable()` 各发一次零 Twist；
- 与普通模式的手动底盘遥控（case 20）**不会同时生效**：进入全身模式时 CHASSIS 会被强制退出并零速；全身模式内不提供手动底盘遥控；
- **`BASE_UNLOCKED` 时让路给 WBC**：不发 `/cmd_vel`，但把 `p_ref` 与人体同步，
  回到 `BASE_LOCKED` 时不产生冲击。

> 已知限制：`ψ_offset` 靠积分自己下发的 ω，打滑或限速饱和时会漂移；
> 底盘跟踪关闭时**上肢目标仍然以底座为基准**，所以人走动不会再拖动手臂；但若底盘被锁定且人走得
> 太远，手的目标仍会超出机械臂工作空间（此时应开启底盘跟踪）。

### 其余行为

- 全身遥操开启期间：
  - **只保留 FSM、夹爪、事件 4、事件 7**；`3`/`6`/`16`/`20`/`21–28` 等其他组合键不可用；
  - **左右摇杆都不再微调末端/腰部**（thumbstickAxesCallback 在全身模式下直接早退）：
    本模式下双臂末端与腰部目标全部由骨架绝对映射给出，若容许摇杆累加偏移，
    会表现为“拨右摇杆就推着右臂走”。进入模式时会把已有摇杆偏移清零；
  - 进入模式时强制关闭镜像并退出 CHASSIS（零速），退出后普通模式从 STORAGE 开始，
    镜像保持关闭，需重新按事件 7 开启。
- 数据失效（骨架断流/关节非法）时目标保持不动（冻结，不清零）；
  离开 OCS2（或拓扑变为 SPLIT_BODY）时**整个全身遥操作模式自动退出**（「全部跟踪」「底盘跟踪」一并关闭），
  回到 OCS2 后需重新按组合键恢复。
- 依赖头显端开启全身追踪（Whole Body Motion Tracking + 足部追踪器）：
  `/teleop/body_*` 话题不存在时开关仍可切换，但不会有任何目标更新（日志提示）。

## 使用方法

### 1. 基本使用

```cpp
#include "arms_target_manager/VRInputHandler.h"

// 创建VRInputHandler
auto vr_handler = std::make_unique<VRInputHandler>(
    node, target_manager.get(), 500.0);
```

### 2. 启动节点

```bash
# 使用launch文件启动
ros2 launch arms_target_manager vr_test.launch.py

# 或者直接启动节点
ros2 run arms_target_manager arms_target_manager_node --ros-args -p enable_vr:=true -p vr_update_rate:=500.0
```

全身控制请用 WBC launch（会加载 `ocs2_wbc_controller` 并带上 ArmsTargetManager），例如：

```bash
ros2 launch ocs2_arm_controller full_body.launch.py \
    robot_name:=fiveages_w2 \
    robot_type:=rg75
```

### 3. 操作流程

1. **启动系统**: 确保VR设备、`xr_target_node` 和机器人系统都在运行
2. **进入 OCS2**: 右 A 前进到 OCS2；全身控制的模式组合键只在此状态下生效
3. **存储模式**: 默认处于存储模式，移动VR设备到期望的起始位置
4. **切换到更新模式**: 点按右摇杆，系统会存储当前VR和机器人的pose作为base pose
5. **VR控制**: 移动VR设备，机器人marker会跟随VR设备的变化
6. **切换回存储模式**: 再次点按右摇杆，可以重新设置base pose
7. **全身模式**: 见上一节「全身控制操作说明」

## 技术细节

### 变化检测阈值（暂时禁用）
- `POSITION_THRESHOLD`: 原位置变化阈值1cm，代码保留用于A/B回退，当前不参与发布判断
- `ORIENTATION_THRESHOLD`: 原方向变化阈值0.005弧度，代码保留用于A/B回退，当前不参与发布判断

### 差值计算算法
```cpp
// 计算VR pose差值
Eigen::Vector3d vrPosDiff = vrCurrentPos - vrBasePos;
Eigen::Quaterniond vrOriDiff = vrBaseOri.inverse() * vrCurrentOri;

// 应用到机器人base pose
resultPos = robotBasePos + vrPosDiff;
resultOri = robotBaseOri * vrOriDiff;
```

### 状态管理
- 使用原子变量确保线程安全
- 支持多线程环境下的状态切换
- 自动检测XR节点的存在性
- 控制拓扑在启动期每秒重试，确认 FULL_BODY 或 SPLIT_BODY 后停止；两者同时 active 则保持 UNKNOWN

## 调试信息

VRInputHandler会输出详细的调试信息：

- 控制拓扑切换（`VR control topology changed to FULL_BODY`）
- 模式切换信息
- 身体模式 / WBC 开关的请求、确认、2 秒超时
- VR pose变化记录（存储模式）
- 机器人pose变化记录（存储模式）
- 计算的pose信息（更新模式）
- Marker更新确认

## 注意事项

1. **坐标系一致性**: 手臂锚点与发布目标均在 `ee_frame_id_`，仅用恢复时冻结的旋转转换 `vr_follow_frame` 中的增量
2. **频率匹配**: VR更新频率应与系统处理能力匹配
3. **阈值调整**: 根据实际需求调整变化检测阈值
4. **线程安全**: 所有状态变量都使用原子操作确保线程安全
5. **TF 故障**: 跟随 frame 与控制 frame 间 TF 缺失时不发布错误目标，后续 VR 回调自动重试
6. **组合键抑制**: FULL_BODY 下按住握把会锁存对应摇杆，松手后须回中才恢复末端/底盘轴值。握把切换摇杆 XY/Z+Yaw 只认短按（松开且 ≤400 ms）；长按或已发出组合键（13/14/16/21–28）松开时不切模式。
7. **能力门控**: `/mode_command` 是否被接受取决于 WBC task.info（如 `bodyTrackingEE`、`waistLock`、`bimanualCoupling`）。被拒绝时 VR 侧 2 秒超时后按实际状态恢复

## 故障排除

### 常见问题

1. **VR控制不响应**
   - 检查VR设备是否连接、`/xr_target_node` 是否存在
   - 确认topic名称是否为 `/teleop/left_ee_pose` 等
   - 检查是否处于更新模式，以及 FSM 是否为 OCS2

2. **Marker不更新**
   - 确认已点按右摇杆切换到更新模式
   - 确认ArmsTargetManager正常工作
   - FULL_BODY 下确认该臂未被 WBC 禁用，且不在身体模式 pending 中

3. **全身组合键无反应**
   - 日志若出现「当前不是 FULL_BODY」，说明还在跑 `ocs2_arm_controller`
   - 日志若出现「当前 FSM 不是 OCS2」，先用右 A 进入 OCS2
   - 握把+摇杆偏置需超过 0.7；pending 未结束或摇杆未回中时后续请求会被忽略
   - 双臂耦合开启时不能用 Y/B 单独切臂
   - 事件 16 要求 VR enabled、FULL_BODY、OCS2；耦合时不能把暂停语义切回禁用

4. **频闪问题**
   - 调整位置和方向变化阈值
   - 检查VR设备的数据质量
   - 确认更新频率设置合理

## 与VRMarkerWrapper的对比

| 特性 | VRMarkerWrapper | VRInputHandler |
|------|----------------|----------------|
| 目标系统 | ocs2_ros_interfaces | arms_target_manager |
| Marker控制 | IMarkerControl接口 | ArmsTargetManager |
| 状态管理 | 内置状态机 | 简化状态切换 |
| 集成方式 | 独立组件 | 集成到target manager |
| 配置方式 | 构造函数参数 | 启动参数 |

## 开发历史

基于VRMarkerWrapper的功能，适配arms_target_manager的架构，保持了核心的VR控制逻辑，同时简化了与目标管理器的集成。FULL_BODY 下增加身体模式、WBC 四开关、单臂禁用 rebase 与底盘遥控。

### 头显驱动 WBC Head 6D 目标

启用现有 `enable_vr` 后，头部跟踪直接使用现有状态：控制拓扑为 FULL_BODY、VR enabled、UPDATE、FSM=OCS2、head_state=HEAD_TRACKING。没有新增头部使能或缩放配置。

- 输入 `/xr/head_pose`（Pose）和 `head_current_pose`（PoseStamped），输出相对话题 `head_target`（Pose）。头部实际位姿订阅独立于左右手缓存。
- 头显上游 TeleVuer/XRoboToolkit wrapper 已完成 OpenXR → 机器人轴转换。头显增量按机器人轴与 `vr_follow_frame` 对齐的约定计算，平移为 **1:1**，不受双手校准比例和镜像模式影响。
- 在 `vr_follow_frame` 中使用 `p = p_robot_base + p_head - p_head_base`、`q = q_head * inverse(q_head_base) * q_robot_base`；最后通过 TF 转到 `head_current_pose.header.frame_id` 发布，与双臂从 current pose 获取发布坐标系的方式一致。
- 每次接管重新记录头显基准。机器人基准优先使用最后发布的目标（保存于明确的控制坐标系，再转换到计算坐标系）；只有没有历史目标时才用当前实际位姿。暂停、退出模式、断流不清除历史目标，避免柔顺控制的实际偏差累积到目标中。
- 每条输入均检查有限值和四元数有效性，不做位姿超时检测。头显消息停止时不更新目标；消息恢复且控制状态未改变时，继续使用原基准，不因消息间隔自动重新对齐。机器人实际位姿只缓存，不按接收时间判定过期。
- VR 实际接管期间，头部 marker 保留最终目标显示，但拖动和菜单发送均被禁用；释放后恢复交互。停止发布不取消 WBC 已收到的目标。

恢复时沿用 VR 自己的历史目标：即使 WBC 模式切换初始化了目标，或释放期间 marker 设置了其他目标，VR 恢复仍会回到自身最后目标。本接口不检测设备断流或上游重发的旧位姿。

运行验证应在仿真中进行：首次接管检查目标等于实际位姿；三轴移动 0.1 m 检查目标对应变化 0.1 m；改变手臂缩放及镜像确认头部不受影响；制造实际位姿偏差后重复 STORAGE/UPDATE，确认目标无累计漂移；暂停头显消息再恢复，确认不因消息间隔重设基准；暂停机器人实际位姿消息，确认仍按已有缓存处理；检查非单位 TF、四元数正负等价、无效位姿、模式切换以及 marker 菜单/拖动互斥。

头部开关独立于身体模式，使用 WBC 的 `head_state` 反馈。头腰耦合模式禁止独立头部跟踪，进入耦合后 VR 释放头部接管但保留最后目标；退出耦合不自动启用头部。RViz 面板按“左臂/右臂、底盘/头部、双臂耦合/参考关节、身体模式/HOLD”排列，头部按钮由实际反馈驱动。
