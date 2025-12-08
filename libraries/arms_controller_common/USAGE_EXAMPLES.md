# GravityCompensation 使用示例

## 两种构造方式

### 方式一：从 URDF 文件创建

适用于 `basic_joint_controller` 等不依赖 OCS2 的控制器。

```cpp
#include "arms_controller_common/utils/GravityCompensation.h"

// 从 URDF 文件路径创建
std::string urdf_path = ament_index_cpp::get_package_share_directory("robot_description") 
                       + "/urdf/robot.urdf";
auto gravity_comp = std::make_shared<arms_controller_common::GravityCompensation>(urdf_path);
```

### 方式二：从现有 Pinocchio 模型创建（推荐用于 OCS2）

适用于 `ocs2_arm_controller`，直接复用 OCS2 已经加载的 Pinocchio 模型，避免重复加载 URDF。

```cpp
#include "arms_controller_common/utils/GravityCompensation.h"
#include <ocs2_mobile_manipulator/MobileManipulatorInterface.h>

// 从 OCS2 interface 获取 Pinocchio 模型
// ctrl_comp_ 是 CtrlComponent 的实例，已经包含了加载好的 interface_
const auto& pinocchio_model = ctrl_comp_->interface_->getPinocchioInterface().getModel();

// 使用现有模型创建重力补偿工具（避免重复加载）
auto gravity_comp = std::make_shared<arms_controller_common::GravityCompensation>(pinocchio_model);
```

## 在 OCS2 控制器中的完整示例

```cpp
// 在 Ocs2ArmController::on_init() 中

// 1. 创建 CtrlComponent（已经加载了 Pinocchio 模型）
ctrl_comp_ = std::make_shared<CtrlComponent>(get_node(), ctrl_interfaces_);

// 2. 从 CtrlComponent 获取 Pinocchio 模型并创建重力补偿工具
const auto& pinocchio_model = ctrl_comp_->interface_->getPinocchioInterface().getModel();
auto gravity_comp = std::make_shared<arms_controller_common::GravityCompensation>(pinocchio_model);

// 3. 创建带重力补偿的 StateHome
state_list_.home = std::make_shared<arms_controller_common::StateHome>(
    ctrl_interfaces_, get_node()->get_logger(), home_duration, gravity_comp);
state_list_.home->setHomePosition(home_pos_);

// 4. 创建带重力补偿的 StateHold
state_list_.hold = std::make_shared<arms_controller_common::StateHold>(
    ctrl_interfaces_, get_node()->get_logger(), hold_threshold, gravity_comp);
```

## 优势对比

| 方式 | 优点 | 缺点 | 适用场景 |
|------|------|------|----------|
| 从 URDF 创建 | 独立，不依赖其他组件 | 需要重复加载 URDF | basic_joint_controller |
| 从模型创建 | 复用已有模型，高效 | 需要已有 Pinocchio 模型 | ocs2_arm_controller |

## 注意事项

1. **模型一致性**：确保传入的 Pinocchio 模型与控制器使用的机器人模型一致
2. **线程安全**：每个 `GravityCompensation` 实例维护自己的 `Data` 对象，可以安全地在多线程环境中使用
3. **性能**：方式二（从模型创建）避免了重复加载 URDF，启动更快，内存占用更少

