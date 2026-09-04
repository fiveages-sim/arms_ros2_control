# VRInputHandler - VR输入处理器

## 概述

VRInputHandler是基于VRMarkerWrapper功能开发的VR输入处理器，专门为arms_target_manager设计。它实现了VR pose订阅、状态切换机制和marker更新功能。

当 `ocs2_wbc_controller` 处于 active 时，拓扑判定为 **FULL_BODY（全身控制）**，可在 VR 手柄上切换身体模式、底盘锁、双臂耦合和单臂启用。当 `ocs2_arm_controller` 处于 active 时为 **SPLIT_BODY（分体/双臂）**，全身组合键无效。

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

### VR输入
- `/xr/left_ee_pose` (`geometry_msgs/msg/Pose`): VR左臂末端执行器pose
- `/xr/right_ee_pose` (`geometry_msgs/msg/Pose`): VR右臂末端执行器pose
- `/xr/head_pose` (`geometry_msgs/msg/Pose`): VR头显pose
- `/xr/controller_state` (`std_msgs/msg/Int32`): 按钮/组合键事件码（单帧）
- `/xr/thumbstick_axes` (`geometry_msgs/msg/Twist`): 左右摇杆轴值；`linear.z` / `angular.z` 携带左右握把按下状态
- `/xr/trigger_values` (`geometry_msgs/msg/Twist`): 左右扳机模拟量

### 机器人状态
- `left_current_pose` (`geometry_msgs/msg/PoseStamped`): 机器人左臂当前pose
- `right_current_pose` (`geometry_msgs/msg/PoseStamped`): 机器人右臂当前pose

## 参数配置

### 构造函数参数
- `node`: ROS节点指针
- `targetManager`: ArmsTargetManager指针
- `updateRate`: 更新频率，默认500Hz

### 启动参数
- `enable_vr`: 是否启用VR控制，默认true
- `vr_update_rate`: VR更新频率，默认500.0Hz
- `vr_follow_frame`: FULL_BODY 下 VR 末端目标计算坐标系，默认 `base_footprint`

### FULL_BODY 目标坐标系

`vr_follow_frame` 默认是 `base_footprint`。进入 UPDATE/rebase 时，
`VRInputHandler` 将 current pose 从其 `header.frame_id` 转换到
`vr_follow_frame` 并锁存；每条高频 VR 输入在该 frame 中计算目标，
再转换回 current-pose frame 后发布普通 `Pose`。

- mobile-base：通常为 `world -> base_footprint` 锁存，
  `base_footprint -> world` 发布；
- fixed-base：通常两个 frame 都是 `base_footprint`，直接复制；
- TF 不可用：跳过本周期，不更新缓存，下一条 VR 输入自动重试。
- `robot.local.yaml` 只选择 `info_file_name`，不配置本参数。

## 全身控制操作说明

以下操作仅在控制拓扑为 **FULL_BODY**（`ocs2_wbc_controller` active）、VR 已启用、且 FSM = **OCS2** 时生效。SPLIT_BODY 下按这些组合会被忽略。

方向组合由 `xr_target_node` 在握把按住且摇杆偏置超过 **0.7** 时发出；松开握把后，对应摇杆必须回到 **0.3** 以内，才会解除抑制，避免「先松握把、后松摇杆」误驱动末端或底盘。

### 推荐流程

1. 启动全身控制器（`ocs2_wbc_controller`）和 `arms_target_manager`（`enable_vr:=true`），并确认 `xr_target_node` 在线。
2. 用 **右 A** 把 FSM 推到 OCS2（HOME → HOLD → OCS2）；**左 X** 后退。
3. 把双手柄放到期望起始姿态，点 **右摇杆** 进入 UPDATE，双臂开始跟随。
4. 用 **左握把 + 左摇杆四方向** 切换身体模式；用 **右握把 + 右摇杆上/下/右** 切换双臂耦合 / 底盘锁 / 参考关节。单臂启用默认用左 Y / 右 B；**左右握把 + 左 Y + 右 B**（事件 16）可把 Y/B 切到 VR 暂停语义。
5. 进入 **跟随（BODY_TRACKING）** 后，左摇杆改控腰部，右摇杆仍控右臂。
6. 需要底盘遥控时，再同时按下左右摇杆进入 CHASSIS 模式。

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

### 全身控制速查

```
左握把 + 左摇杆上/下/左/右  →  竖直 / 锁定 / 跟随 / 自定义
右握把 + 右摇杆上/下        →  双臂耦合 / 底盘锁（左/右预留）
左 Y / 右 B                 →  默认对应臂 ENABLE/DISABLE；事件 16 后为 VR 暂停/恢复
左右握把 + 左 Y + 右 B      →  切换 Y/B 禁用 ↔ 暂停语义（SPLIT_BODY 忽略）
左右摇杆同时按下            →  底盘遥控 ↔ 末端控制
跟随模式下短按左握把      →  腰部摇杆 XY ↔ Z+Yaw
短按左/右握把              →  对应臂摇杆 XY ↔ Z+Yaw（长按或组合键不切换）
右摇杆点击                  →  STORAGE ↔ UPDATE（需 OCS2）
右 A / 左 X                 →  FSM 前进 / 后退
```

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

1. **坐标系一致性**: FULL_BODY 下计算在 `vr_follow_frame`，发布前转换到 `ee_frame_id_`；非全身控制保持旧语义
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
   - 确认topic名称是否为 `/xr/left_ee_pose` 等
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
