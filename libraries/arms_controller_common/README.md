# Arms Controller Common Library

为机械臂控制器提供共享 FSM（有限状态机）机制的通用库，包含 StateMoveJ、StateHome、StateHold、腰部运动规划、关节限位管理、插值、重力补偿等基础组件。

---

## 目录结构

```
arms_controller_common/
├── include/arms_controller_common/
│   ├── FSM/
│   │   ├── FSMState.h                  # FSM 基类
│   │   ├── StateHome.h                 # Home 状态（多构型）
│   │   ├── StateHold.h                 # Hold 状态（位置保持）
│   │   └── StateMoveJ.h                # MoveJ 状态（关节空间运动）
│   ├── CtrlInterfaces.h                # 控制接口聚合结构体
│   └── utils/
│       ├── GravityCompensation.h       # 基于 Pinocchio 的静态重力补偿
│       ├── Interpolation.h             # tanh/linear 插值
│       ├── JointTrajectoryManager.h    # 多路点轨迹管理
│       ├── JointLimitsManager.h        # URDF 关节限位解析与检查
│       ├── WaistLiftingPlaner.h        # 腰部升降/转向运动规划
│       ├── FSMCommandPublisher.h       # FSM 命令发布工具
│       ├── FSMStateTransitionValidator.h
│       ├── SharedPublishers.h
│       └── AngleUtils.h
├── src/
│   ├── FSM/
│   │   ├── StateHome.cpp
│   │   ├── StateHold.cpp
│   │   └── StateMoveJ.cpp
│   └── utils/
│       ├── GravityCompensation.cpp
│       ├── JointTrajectoryManager.cpp
│       ├── JointLimitsManager.cpp
│       └── WaistLiftingPlaner.cpp
├── CMakeLists.txt
└── package.xml
```

---

## 功能特性

| 组件 | 功能 |
|---|---|
| `FSMState` | 统一的有限状态机基类接口 |
| `StateHome` | 支持最多 10 个预设构型，平滑插值到目标构型 |
| `StateHold` | 保持关节当前位置，可选重力补偿 |
| `StateMoveJ` | 关节空间运动，支持单目标、轨迹、关节前缀局部控制 |
| `GravityCompensation` | 基于 Pinocchio 的静态力矩计算（MIX 控制模式） |
| `JointTrajectoryManager` | 多路点轨迹的时间分配与插值执行 |
| `JointLimitsManager` | 从 URDF 解析关节限位，自动截断越界目标 |
| `WaistLiftingPlaner` | 腰部升降（位置和速度）、转向运动规划 |

---

## StateHome

StateHome 支持最多 10 个预设构型（`home_1`、`home_2`、…），通过 FSM 命令动态切换。

### 从参数自动加载（推荐）

```cpp
// 在 on_init() 中调用，需要 auto_declare 支持
auto state_home = std::make_shared<arms_controller_common::StateHome>(
    ctrl_interfaces_, gravity_comp, node_);

state_home->init([this](const std::string& name, const std::vector<double>& default_val) {
    return this->auto_declare<std::vector<double>>(name, default_val);
});
```

### 手动设置

```cpp
state_home->setHomePosition({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

// 或设置多个构型
state_home->setHomeConfigurations({
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},   // index 0
    {0.5, 0.5, 0.5, 0.5, 0.5, 0.5},   // index 1
});
```

### 访问构型（供其他模块使用）

```cpp
// 获取指定索引的构型（用于 target_command/target_percent）
std::vector<double> config = state_home->getConfiguration(0);  // 0-based
```

### 构型切换命令

通过 FSM 命令切换（`switch_command_base` 默认为 100）：

| 命令值 | 效果 |
|---|---|
| `switch_command_base` | 循环到下一个构型 |
| `switch_command_base + 1` | 切换到构型 0 |
| `switch_command_base + 2` | 切换到构型 1 |
| … | … |

---

## StateHold

保持关节当前位置。可选接入 `GravityCompensation` 以在 MIX 控制模式下输出补偿力矩。

```cpp
auto state_hold = std::make_shared<arms_controller_common::StateHold>(
    ctrl_interfaces_, gravity_comp, node_);
```

---

## StateMoveJ

关节空间运动状态，是控制器中最常用的状态。

### 创建

```cpp
auto state_movej = std::make_shared<arms_controller_common::StateMoveJ>(
    ctrl_interfaces_, node_, joint_names_, gravity_comp);
```

### 订阅关节目标话题

```cpp
// 订阅单一话题（适用于单关节组控制器）
state_movej->setupSubscriptions("target_joint_position");

// 订阅带前缀的话题（left/right/body），适用于全身控制器
state_movej->setupSubscriptions("target_joint_position", /*enable_prefix_topics=*/true);
// 自动创建：/target_joint_position
//           /left/target_joint_position
//           /right/target_joint_position
//           /body/target_joint_position
```

### 订阅关节轨迹话题

```cpp
state_movej->setupTrajectorySubscription("target_joint_trajectory");
```

### 注册关节轨迹服务

```cpp
state_movej->setupJointTrajectoryService("joint_trajectory_with_para");
```

### 编程方式设置目标（线程安全）

```cpp
// 设置所有关节目标
state_movej->setTargetPosition({0.1, 0.2, 0.3, 0.0, 0.0, 0.0});

// 只设置指定前缀的关节（其余关节保持当前位置）
state_movej->setTargetPosition("left", {0.1, 0.2, 0.3});
```

### 关节限位

```cpp
// 从 URDF 自动解析限位（推荐在 robot_description 回调中调用）
state_movej->updateJointLimitsFromURDF(robot_description_xml);

// 或提供自定义限位检查回调（e.g. 从 Pinocchio 模型获取）
state_movej->setJointLimitChecker(
    [this](const std::vector<double>& pos) -> std::vector<double> {
        return my_pinocchio_model_.clampJointLimits(pos);
    });
```

### 腰部运动（需先初始化 WaistLiftingPlaner）

```cpp
// 位置控制：腰部局部坐标系相对移动 [dx, dz]
state_movej->moveWaistLifting(Eigen::Vector3d(0.0, 0.05, 0.0));

// 速度系数控制（-1.0~1.0，0.0 停止）
state_movej->setWaistLiftingFactor(0.5);   // 上升
state_movej->setWaistTurningFactor(-0.3);  // 左转

// 开关命令（0=停止, 1=上升, 2=下降）
state_movej->setWaistLiftingCommand(1);
```

---

## GravityCompensation（重力补偿）

仅在 MIX 控制模式（`position + velocity + effort + kp + kd` 接口）下生效。

### 从 URDF 文件创建

```cpp
auto gravity_comp = std::make_shared<arms_controller_common::GravityCompensation>(
    "/path/to/robot.urdf");
```

### 从已有 Pinocchio 模型创建（避免重复加载）

```cpp
// 适用于 ocs2_arm_controller，复用 OCS2 已加载的模型
const auto& model = ctrl_comp_->interface_->getPinocchioInterface().getModel();
auto gravity_comp = std::make_shared<arms_controller_common::GravityCompensation>(model);
```

---

## 控制模式自动检测

库根据硬件接口自动选择控制模式：

| 模式 | 接口要求 | 重力补偿 |
|---|---|---|
| **POSITION** | 仅 `position` 命令接口 | 不生效 |
| **MIX** | `position + velocity + effort + kp + kd` | 自动输出补偿力矩 |

---

## FSM 状态转换命令

| 命令值 | 效果 |
|---|---|
| `1` | 切换到 HOME |
| `2` | 切换到 HOLD |
| `3` | 切换到 MOVEJ（或具体控制器定义的运动状态） |
| `>= switch_command_base` | HOME 状态内切换构型 |

---

## 集成示例

### Basic Joint Controller（位置控制）

```cpp
#include "arms_controller_common/FSM/StateHome.h"
#include "arms_controller_common/FSM/StateHold.h"
#include "arms_controller_common/FSM/StateMoveJ.h"

// on_init()
state_home_  = std::make_shared<StateHome>(ctrl_interfaces_, nullptr, node_);
state_hold_  = std::make_shared<StateHold>(ctrl_interfaces_, nullptr, node_);
state_movej_ = std::make_shared<StateMoveJ>(ctrl_interfaces_, node_, joint_names_);
state_home_->init([this](auto name, auto def){ return auto_declare<std::vector<double>>(name, def); });

// on_configure()
state_movej_->setupSubscriptions("target_joint_position");
state_movej_->setupTrajectorySubscription();
state_movej_->setupJointTrajectoryService("joint_trajectory_with_para");
```

### OCS2 Arm Controller（MIX 控制 + 重力补偿）

```cpp
#include "arms_controller_common/FSM/StateHome.h"
#include "arms_controller_common/FSM/StateHold.h"
#include "arms_controller_common/utils/GravityCompensation.h"

// 复用 OCS2 的 Pinocchio 模型，避免重复加载 URDF
const auto& model = ctrl_comp_->interface_->getPinocchioInterface().getModel();
auto gravity_comp = std::make_shared<GravityCompensation>(model);

auto state_home = std::make_shared<StateHome>(ctrl_interfaces_, gravity_comp, node_);
auto state_hold = std::make_shared<StateHold>(ctrl_interfaces_, gravity_comp, node_);
```

---

## 依赖

- `hardware_interface`
- `controller_interface`
- `rclcpp` / `rclcpp_lifecycle`
- `std_msgs`, `trajectory_msgs`, `arms_ros2_control_msgs`
- `pinocchio`（重力补偿）
- `urdf`（关节限位解析）
- `eigen3`

---

## 构建

```bash
cd ~/ros2_ws
colcon build --packages-select arms_controller_common
```


## 运动中的目标替换与同步停车

HOME 配置切换和 MOVEJ 关节目标替换先校验新目标。数量错误、非有限值、缺失限位或超出
URDF 位置范围的目标直接拒绝，不改变当前运动及已有待执行目标。MOVEJ 关节目标不再自动
裁剪到边界。减速期间收到新的有效目标，只更新待执行目标，不重新启动停车；所有关节停止
后执行最新目标。

`JointSpeedStopPlanner` 使用公共分段常 jerk 减速进度，按初始速度比例映射各关节，
同时停止，避免腰部等耦合关节各自减速导致姿态变化。继承命令侧位置、速度、加速度；
当初始加速度与速度方向成比例时直接接入公共曲线。比例不兼容时，先以统一时长将
各关节加速度平滑降为零，再按过渡后的速度比例同步停车。这一过渡保持初始速度、
加速度均满足的线性关节约束，但不能保证任意非线性笛卡尔路径不变。
doubleS 输出优先使用解析速度和加速度；其他输出使用控制周期的命令差分估计。

普通单目标运动的限制按控制器配置，默认值如下（旋转关节分别为 rad/s、rad/s²、rad/s³）：

```yaml
ros__parameters:
  home_max_velocity: 2.0
  home_max_acceleration: 4.0
  home_max_jerk: 20.0
  movej_max_velocity: 2.0
  movej_max_acceleration: 4.0
  movej_max_jerk: 20.0
```

停车使用当前轨迹保存的限制，不会因接收新目标时读取参数而改变旧轨迹的停车限制。
HOME 的 doubleS 自动延长过短时长以满足这些限制；MOVEJ 的 doubleS 若关闭
`movej_auto_extend_duration` 且时长不足，会拒绝规划，不再通过压缩时间突破限制。

停车方案准备成功后才清除原轨迹。doubleS 停车若不满足位置/运动约束，则拒绝此次切换并
保留原运动。linear 等非 doubleS 路径按允许的最大 jerk 制动；若先触及位置限位，则在
首次触限时整组一起截停，并将所有关节内部速度、加速度置零，避免单关节停止后其他关节
继续运动破坏配合关系。初始加速度已超出制动限制时，会先以
最大 jerk 恢复到允许范围。触限截断允许命令导数不连续，只保证命令位置受限，不保证机械
关节没有跟踪误差或惯性超调。停车规划不再依赖是否编译了 lina_planning。
