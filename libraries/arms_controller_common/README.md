# Arms Controller Common Library

通用库，为机械臂控制器提供共享的 FSM（有限状态机）机制和重力补偿功能。

## 功能特性

- ✅ **通用 FSM 基类**：统一的有限状态机接口
- ✅ **StateHome**：支持多配置和单配置的 home 位置控制
- ✅ **StateHold**：位置保持状态
- ✅ **重力补偿**：基于 Pinocchio 和 URDF 的静态力矩计算
- ✅ **自动控制模式检测**：根据硬件接口自动检测 position 或 mixed 控制模式
- ✅ **可选功能**：力控模式作为可选功能，根据硬件支持自动启用

## 目录结构

```
arms_controller_common/
├── include/arms_controller_common/
│   ├── FSM/
│   │   ├── FSMState.h          # FSM 基类
│   │   ├── StateHome.h         # Home 状态
│   │   └── StateHold.h          # Hold 状态
│   ├── CtrlInterfaces.h         # 控制接口结构
│   └── utils/
│       └── GravityCompensation.h # 重力补偿工具类
├── src/
│   ├── FSM/
│   │   ├── StateHome.cpp
│   │   └── StateHold.cpp
│   └── utils/
│       └── GravityCompensation.cpp
├── CMakeLists.txt
└── package.xml
```

## 使用示例

### 1. 基本使用（仅位置控制）

#### 方式一：手动设置配置

```cpp
#include "arms_controller_common/FSM/StateHome.h"
#include "arms_controller_common/FSM/StateHold.h"

// 创建 StateHome
auto state_home = std::make_shared<arms_controller_common::StateHome>(
    ctrl_interfaces_, logger_, 3.0);

// 设置单个 home 位置
std::vector<double> home_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
state_home->setHomePosition(home_pos);

// 或设置多个配置
std::vector<std::vector<double>> home_configs = {
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.5, 0.5, 0.5, 0.5, 0.5, 0.5}
};
state_home->setHomeConfigurations(home_configs);

// 创建 StateHold
auto state_hold = std::make_shared<arms_controller_common::StateHold>(
    ctrl_interfaces_, logger_, 0.1);
```

#### 方式二：从参数自动加载（推荐）

```cpp
#include "arms_controller_common/FSM/StateHome.h"

// 创建 StateHome
auto state_home = std::make_shared<arms_controller_common::StateHome>(
    ctrl_interfaces_, logger_, 3.0);

// 使用 init() 方法自动从参数加载配置（home_1, home_2, home_3, ...）
// 在控制器的 on_init() 中调用
state_home->init([this](const std::string& name, const std::vector<double>& default_value)
{
    return this->auto_declare<std::vector<double>>(name, default_value);
});

// 设置配置切换命令基值
state_home->setSwitchCommandBase(100);
```

这种方式会自动从 ROS 参数中读取 `home_1`, `home_2`, `home_3` 等配置，非常方便！

### 2. 使用重力补偿（力控模式）

#### 方式一：从 URDF 文件创建（适用于 basic_joint_controller）

```cpp
#include "arms_controller_common/utils/GravityCompensation.h"

// 创建重力补偿工具（从 URDF 文件路径）
std::string urdf_path = "/path/to/robot.urdf";
auto gravity_comp = std::make_shared<arms_controller_common::GravityCompensation>(urdf_path);

// 创建带重力补偿的 StateHome
auto state_home = std::make_shared<arms_controller_common::StateHome>(
    ctrl_interfaces_, logger_, 3.0, gravity_comp);

// 创建带重力补偿的 StateHold
auto state_hold = std::make_shared<arms_controller_common::StateHold>(
    ctrl_interfaces_, logger_, 0.1, gravity_comp);
```

#### 方式二：从现有 Pinocchio 模型创建（适用于 ocs2_arm_controller，避免重复加载）

```cpp
#include "arms_controller_common/utils/GravityCompensation.h"
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>

// 从 OCS2 interface 获取 Pinocchio 模型（避免重复加载 URDF）
const auto& pinocchio_model = ctrl_comp_->interface_->getPinocchioInterface().getModel();
auto gravity_comp = std::make_shared<arms_controller_common::GravityCompensation>(pinocchio_model);

// 创建带重力补偿的 StateHome
auto state_home = std::make_shared<arms_controller_common::StateHome>(
    ctrl_interfaces_, logger_, 3.0, gravity_comp);

// 创建带重力补偿的 StateHold
auto state_hold = std::make_shared<arms_controller_common::StateHold>(
    ctrl_interfaces_, logger_, 0.1, gravity_comp);
```

**优势**：方式二避免了重复加载 URDF 文件，直接复用 OCS2 已经加载的 Pinocchio 模型，更高效。

### 3. 多配置支持

```cpp
// 设置多个 home 配置
std::vector<std::vector<double>> home_configs = {
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},  // home_1
    {0.5, 0.5, 0.5, 0.5, 0.5, 0.5},  // home_2
    {-0.5, -0.5, -0.5, -0.5, -0.5, -0.5}  // home_3
};
state_home->setHomeConfigurations(home_configs);

// 设置配置切换命令基值（默认 100）
state_home->setSwitchCommandBase(100);
// command = 100: 循环切换到下一个配置
// command = 101: 切换到配置 0
// command = 102: 切换到配置 1
// ...
```

### 4. Home/Rest Pose 切换

```cpp
// 设置 rest pose（将作为第二个配置，index 1）
std::vector<double> rest_pos = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
state_home->setRestPose(rest_pos);

// Rest pose 现在是 home_configs_ 的第二个配置（index 1）
// 可以通过配置切换命令访问：
// command = switch_command_base + 2 (默认 102): 切换到 rest pose (配置 1)
```

## 控制模式

库会自动检测硬件接口并设置控制模式：

- **POSITION 模式**：只有 position 命令接口
- **MIX 模式**：有 position, velocity, effort, kp, kd 命令接口

在 MIX 模式下，如果提供了 `GravityCompensation`，会自动计算并发送静态力矩。

## 状态转换命令

- `command = 1`: 切换到 HOME 状态
- `command = 2`: 切换到 HOLD 状态
- `command = 3`: 切换到 MOVE/OCS2 状态（由具体控制器决定）
- `command = 4`: 在 HOME 状态下切换 home/rest pose
- `command >= 100`: 在 HOME 状态下切换配置（可配置）

## 依赖

- `hardware_interface`
- `arms_ros2_control_msgs`
- `rclcpp`
- `pinocchio`（用于重力补偿）
- `urdf`（用于加载机器人模型）
- `ament_index_cpp`

## 构建

```bash
cd ~/ros2_ws
colcon build --packages-select arms_controller_common
```

## 集成到控制器

### Basic Joint Controller

```cpp
#include "arms_controller_common/FSM/StateHome.h"
#include "arms_controller_common/FSM/StateHold.h"

// 使用通用库的 StateHome 和 StateHold
```

### OCS2 Arm Controller

```cpp
#include "arms_controller_common/FSM/StateHome.h"
#include "arms_controller_common/FSM/StateHold.h"
#include "arms_controller_common/utils/GravityCompensation.h"

// 从 OCS2 interface 获取 Pinocchio 模型（避免重复加载）
const auto& pinocchio_model = ctrl_comp_->interface_->getPinocchioInterface().getModel();
auto gravity_comp = std::make_shared<arms_controller_common::GravityCompensation>(pinocchio_model);

// 使用通用库，并添加重力补偿支持
auto state_home = std::make_shared<arms_controller_common::StateHome>(
    ctrl_interfaces_, logger_, 3.0, gravity_comp);
auto state_hold = std::make_shared<arms_controller_common::StateHold>(
    ctrl_interfaces_, logger_, 0.1, gravity_comp);
```

## 优势

1. **代码复用**：两个控制器共享相同的 FSM 实现
2. **功能统一**：力控模式作为可选功能，两个控制器都可以使用
3. **易于维护**：统一的代码库，减少重复代码
4. **灵活扩展**：支持多配置、pose 切换等高级功能
5. **自动适配**：根据硬件接口自动选择控制模式

