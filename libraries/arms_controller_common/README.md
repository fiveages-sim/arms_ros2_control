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
// 位置控制：腰部相对当前位置移动 distance 米
state_movej->moveWaistLifting(0.05);

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
