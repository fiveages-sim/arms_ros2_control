# 统一关节轨迹管理器分析与设计方案

## 一、现状分析

### 1.1 当前实现情况

#### StateHome 和 StateMoveJ 的插值实现
- **单节点插值**：两个状态都实现了从起点到终点的单段插值
- **支持的插值类型**：
  - `TANH`: 使用 tanh 函数进行平滑插值
  - `LINEAR`: 线性插值
  - `DOUBLES`: 使用 lina planning 的 `moveJ` planner（单段轨迹）
  - `NONE`: 直接跳转到目标位置

#### 代码重复问题
在 `StateHome.cpp` 和 `StateMoveJ.cpp` 中存在大量重复的插值逻辑：

1. **进度更新逻辑**（`updateInterpolationProgress`）：
   ```cpp
   percent_ += 1.0 / (duration_ * controller_frequency);
   ```

2. **相位计算逻辑**（`calculateInterpolationPhase`）：
   - TANH: `phase = tanh(percent_ * scale)`
   - LINEAR: `phase = percent_`
   - NONE: `phase = 1.0`

3. **位置应用逻辑**（`applyInterpolatedJointPositions`）：
   ```cpp
   interpolated_value = phase * target_pos[i] + (1.0 - phase) * start_pos[i];
   ```

4. **DOUBLES 插值初始化**（`initMoveJPlanner`）：
   - 两个状态都有相同的 `moveJ` planner 初始化代码

### 1.2 Lina Planning 多节点轨迹支持

#### 已实现的功能
- **`SmoothCurveOfMultiJointsUsingBlending`**: 支持多节点轨迹规划
  - 输入：多个轨迹点（`std::vector<TrajectPoint>`）
  - 输出：平滑的多段轨迹（movej + bezier 混合）
  - 接口：继承自 `Planner`，提供统一的 `run()` 方法

#### 当前使用限制
- `StateHome` 和 `StateMoveJ` 中的 `DOUBLES` 类型只使用单段 `moveJ`
- 没有利用多节点轨迹规划能力

### 1.3 设计机会

✅ **可行性高**：
1. lina planning 已经提供了完整的多节点轨迹规划接口
2. `Planner` 基类提供了统一的 `run()` 接口
3. 单节点和多节点轨迹可以统一封装

## 二、设计方案

### 2.1 模块化设计：基础插值 + 可选进阶模块

#### 设计原则
1. **基础插值模块**（不依赖 lina planning）：
   - 支持 `TANH`, `LINEAR`, `NONE` 插值类型
   - 支持单节点和多节点轨迹
   - 多节点轨迹使用分段插值（每段独立使用基础插值）

2. **进阶插值模块**（可选，依赖 lina planning）：
   - 支持 `DOUBLES` 插值类型
   - 单节点轨迹使用 `moveJ` planner
   - 多节点轨迹使用 `SmoothCurveOfMultiJointsUsingBlending`

3. **自动检测**：
   - 编译时检测 lina planning 是否可用
   - 运行时根据插值类型和可用模块选择实现方式

### 2.2 统一关节轨迹管理器（JointTrajectoryManager）

#### 核心职责
1. **统一接口**：为单节点和多节点轨迹提供统一的接口
2. **自动选择**：根据插值类型、节点数量和可用模块自动选择计算方式
3. **状态管理**：管理轨迹执行状态（进度、完成状态等）
4. **模块化**：基础功能不依赖 lina planning，进阶功能可选

#### 类设计

```cpp
class JointTrajectoryManager
{
public:
    // 初始化单节点轨迹
    bool initSingleNode(
        const std::vector<double>& start_pos,
        const std::vector<double>& target_pos,
        double duration,
        InterpolationType type,
        double controller_frequency,
        double tanh_scale = 3.0
    );

    // 初始化多节点轨迹
    // 基础模式：使用分段插值（每段用 TANH/LINEAR）
    // 进阶模式（DOUBLES + lina planning 可用）：使用 SmoothCurveOfMultiJointsUsingBlending
    bool initMultiNode(
        const std::vector<std::vector<double>>& waypoints,
        const std::vector<double>& durations,  // 每段轨迹的持续时间
        InterpolationType type,  // 基础插值类型（TANH/LINEAR）或 DOUBLES
        double controller_frequency,
        double tanh_scale = 3.0
    );

    // 获取下一个轨迹点（统一接口）
    std::vector<double> getNextPoint();

    // 检查是否完成
    bool isCompleted() const;

    // 重置轨迹
    void reset();

    // 获取当前进度 [0, 1]
    double getProgress() const;

    // 检查 DOUBLES 插值是否可用（lina planning 是否可用）
    static bool isDoublesAvailable();

private:
    // 单节点插值实现
    std::vector<double> computeSingleNodePoint();
    
    // 多节点轨迹实现 - 基础模式（分段插值）
    std::vector<double> computeMultiNodeBasic();
    
    // 多节点轨迹实现 - 进阶模式（lina planning）
    std::vector<double> computeMultiNodeAdvanced();
    
    // 根据插值类型计算相位
    double calculatePhase(InterpolationType type, double percent);
    
    // 检查是否使用进阶模式
    bool useAdvancedMode() const;
};
```

### 2.3 实现策略

#### 策略 1：单节点轨迹

**基础插值**（TANH/LINEAR/NONE）：
- 使用现有的简单插值逻辑
- 不依赖 lina planning

**进阶插值**（DOUBLES，需要 lina planning）：
- 使用 `moveJ` planner（单段）
- 如果 lina planning 不可用，返回错误或降级到 LINEAR

#### 策略 2：多节点轨迹

**基础模式**（TANH/LINEAR，不依赖 lina planning）：
- 使用分段插值：将多节点轨迹分解为多个单段
- 每段独立使用基础插值（TANH/LINEAR）
- 示例：3个节点 → 2段轨迹，每段独立插值

**进阶模式**（DOUBLES + lina planning 可用）：
- 使用 `SmoothCurveOfMultiJointsUsingBlending`
- 提供平滑的多段轨迹（movej + bezier 混合）
- 如果 lina planning 不可用，自动降级到基础模式

#### 策略 3：模块检测

**编译时检测**：
```cmake
# CMakeLists.txt
find_package(lina_planning QUIET)  # 改为 QUIET，不强制要求
if(lina_planning_FOUND)
    target_compile_definitions(${PROJECT_NAME} PUBLIC HAS_LINA_PLANNING)
    target_link_libraries(${PROJECT_NAME} PUBLIC lina_planning::lina_planning)
endif()
```

**运行时检测**：
```cpp
// 在 JointTrajectoryManager 中
#ifdef HAS_LINA_PLANNING
    bool JointTrajectoryManager::isDoublesAvailable() { return true; }
#else
    bool JointTrajectoryManager::isDoublesAvailable() { return false; }
#endif
```

### 2.3 集成方案

#### 在 StateHome 和 StateMoveJ 中的使用

**重构前**（当前实现）：
```cpp
// StateHome::run()
if (interpolation_type_ == InterpolationType::DOUBLES) {
    planning::TrajectPoint movej_point = movej_planner.run();
    // 应用位置...
} else {
    updateInterpolationProgress();
    double phase = calculateInterpolationPhase();
    applyInterpolatedJointPositions(phase);
}
```

**重构后**（使用统一管理器）：
```cpp
// StateHome::run()
std::vector<double> next_pos = trajectory_manager_.getNextPoint();
applyJointPositions(next_pos);
```

### 2.4 多节点轨迹初始化示例

#### 基础模式（不依赖 lina planning）
```cpp
// 示例：3个节点的轨迹，使用 TANH 插值
std::vector<std::vector<double>> waypoints = {
    start_pos,         // 起点
    intermediate_pos,  // 中间点
    target_pos         // 终点
};

std::vector<double> durations = {2.0, 2.0};  // 两段轨迹，每段2秒

// 使用基础插值（分段 TANH）
trajectory_manager_.initMultiNode(
    waypoints, 
    durations, 
    InterpolationType::TANH,  // 基础插值类型
    controller_frequency
);
```

#### 进阶模式（需要 lina planning）
```cpp
// 示例：3个节点的轨迹，使用 DOUBLES 插值
std::vector<std::vector<double>> waypoints = {
    start_pos,         // 起点
    intermediate_pos,  // 中间点
    target_pos         // 终点
};

std::vector<double> durations = {2.0, 2.0};  // 两段轨迹，每段2秒

// 使用进阶插值（lina planning 的平滑轨迹）
if (JointTrajectoryManager::isDoublesAvailable()) {
    trajectory_manager_.initMultiNode(
        waypoints, 
        durations, 
        InterpolationType::DOUBLES,  // 进阶插值类型
        controller_frequency
    );
} else {
    // 降级到基础模式
    RCLCPP_WARN(logger_, "DOUBLES interpolation not available, falling back to LINEAR");
    trajectory_manager_.initMultiNode(
        waypoints, 
        durations, 
        InterpolationType::LINEAR,
        controller_frequency
    );
}
```

### 2.5 基础多节点轨迹实现细节

#### 分段插值算法
```cpp
// 伪代码：基础多节点轨迹计算
std::vector<double> computeMultiNodeBasic() {
    // 1. 确定当前处于哪一段轨迹
    size_t current_segment = findCurrentSegment();
    
    // 2. 计算当前段内的进度
    double segment_progress = calculateSegmentProgress(current_segment);
    
    // 3. 根据插值类型计算相位
    double phase = calculatePhase(interpolation_type_, segment_progress);
    
    // 4. 在当前段的起点和终点之间插值
    const auto& start = waypoints_[current_segment];
    const auto& end = waypoints_[current_segment + 1];
    
    std::vector<double> result;
    for (size_t i = 0; i < start.size(); ++i) {
        result.push_back(phase * end[i] + (1.0 - phase) * start[i]);
    }
    
    return result;
}
```

#### 优势
- ✅ 不依赖外部库，纯 C++ 实现
- ✅ 支持任意数量的节点
- ✅ 每段可以使用不同的插值类型（未来扩展）
- ✅ 性能开销小

## 三、优势分析

### 3.1 代码复用
- ✅ 消除 `StateHome` 和 `StateMoveJ` 中的重复代码
- ✅ 统一的插值逻辑，便于维护和扩展

### 3.2 功能扩展
- ✅ 为 `StateHome` 和 `StateMoveJ` 提供多节点轨迹支持
- ✅ 为未来的 `HomeJ` 状态提供多节点轨迹功能
- ✅ 统一的接口，便于添加新的插值类型

### 3.3 灵活性
- ✅ 支持单节点和多节点轨迹的统一管理
- ✅ 根据插值类型自动选择最优计算方式
- ✅ 易于扩展新的插值算法

## 四、实现建议

### 4.1 文件结构
```
arms_controller_common/
├── include/arms_controller_common/utils/
│   ├── Interpolation.h (已存在)
│   ├── JointLimitsManager.h (已存在)
│   └── JointTrajectoryManager.h (新增)
└── src/utils/
    └── JointTrajectoryManager.cpp (新增)
```

### 4.1.1 CMakeLists.txt 修改

需要将 lina_planning 从 REQUIRED 改为可选：

```cmake
# 修改前
find_package(lina_planning REQUIRED)

# 修改后
find_package(lina_planning QUIET)  # 可选依赖

if(lina_planning_FOUND)
    # 定义编译宏，表示 lina planning 可用
    target_compile_definitions(${PROJECT_NAME} PUBLIC HAS_LINA_PLANNING)
    target_link_libraries(${PROJECT_NAME} PUBLIC lina_planning::lina_planning)
    message(STATUS "lina_planning found - DOUBLES interpolation enabled")
else()
    message(STATUS "lina_planning not found - DOUBLES interpolation disabled")
endif()
```

### 4.2 实现步骤

1. **创建 JointTrajectoryManager 类**
   - 实现单节点轨迹管理
   - 实现多节点轨迹管理（封装 lina planning）
   - 提供统一的 `getNextPoint()` 接口

2. **重构 StateHome**
   - 替换现有插值逻辑为 `JointTrajectoryManager`
   - 保持现有 API 不变

3. **重构 StateMoveJ**
   - 替换现有插值逻辑为 `JointTrajectoryManager`
   - 保持现有 API 不变

4. **扩展功能**
   - 为 `StateHome` 添加多节点轨迹支持（可选）
   - 为未来的 `HomeJ` 状态提供多节点轨迹功能

### 4.3 注意事项

1. **向后兼容**：保持现有 API 不变，内部实现使用统一管理器
2. **可选依赖**：lina planning 改为可选，基础功能不依赖它
3. **降级策略**：当 DOUBLES 不可用时，自动降级到 LINEAR 或给出警告
4. **错误处理**：添加适当的错误检查和日志记录
5. **测试**：确保重构后功能与现有实现一致
6. **性能考虑**：
   - 基础多节点轨迹：使用分段插值，性能开销小
   - 进阶多节点轨迹：使用 lina planning，性能已经优化

## 五、总结

✅ **可行性结论**：**高度可行**

统一关节轨迹管理器可以：
1. ✅ 统一 `StateHome` 和 `StateMoveJ` 的插值逻辑
2. ✅ 支持单节点和多节点轨迹的统一管理
3. ✅ **模块化设计**：基础功能不依赖 lina planning，进阶功能可选
4. ✅ **基础多节点轨迹**：使用分段插值，支持 TANH/LINEAR/NONE
5. ✅ **进阶多节点轨迹**：利用 lina planning 的平滑轨迹规划能力（可选）
6. ✅ 为未来的 `HomeJ` 状态提供多节点轨迹功能
7. ✅ 提高代码可维护性和可扩展性

### 5.1 功能对比表

| 功能 | 基础模式（无 lina planning） | 进阶模式（有 lina planning） |
|------|---------------------------|---------------------------|
| 单节点 TANH | ✅ 支持 | ✅ 支持 |
| 单节点 LINEAR | ✅ 支持 | ✅ 支持 |
| 单节点 NONE | ✅ 支持 | ✅ 支持 |
| 单节点 DOUBLES | ❌ 不支持 | ✅ 支持（moveJ） |
| 多节点 TANH | ✅ 支持（分段插值） | ✅ 支持（分段插值） |
| 多节点 LINEAR | ✅ 支持（分段插值） | ✅ 支持（分段插值） |
| 多节点 DOUBLES | ❌ 不支持 | ✅ 支持（SmoothCurveOfMultiJointsUsingBlending） |

### 5.2 实现建议

建议按照以下顺序逐步实现：

1. **第一阶段**：实现基础单节点轨迹管理
   - 支持 TANH/LINEAR/NONE
   - 不依赖 lina planning

2. **第二阶段**：实现基础多节点轨迹管理
   - 使用分段插值算法
   - 支持 TANH/LINEAR/NONE 的多节点轨迹

3. **第三阶段**：集成 lina planning（可选）
   - 添加 DOUBLES 支持
   - 单节点使用 moveJ
   - 多节点使用 SmoothCurveOfMultiJointsUsingBlending

4. **第四阶段**：重构 StateHome 和 StateMoveJ
   - 使用统一管理器替换现有插值逻辑
   - 保持 API 向后兼容

