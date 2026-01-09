# StateMoveJ Trajectory Msgs 接口设计

## 一、需求分析

### 1.1 目标
为 `StateMoveJ` 添加基于 `trajectory_msgs::msg::JointTrajectory` 的接口，支持：
- 接收标准 ROS2 轨迹消息
- 提取位置信息用于多节点轨迹规划
- 自动计算每段轨迹的持续时间
- 与现有的单节点接口兼容
- **完全兼容 lina planning 的多节点规划接口**

### 1.1.1 与 lina planning 的兼容性

**lina planning 的 `SmoothCurveOfMultiJointsUsingBlending` 要求**：
- ✅ `multiple_points.size() >= 3`（至少3个轨迹点）
- ✅ `multiple_parameters.size() + 1 == multiple_points.size()`（参数数量 = 点数 - 1）

**我们的设计**：
- ✅ 输入轨迹至少2个点 + 当前位置 = 至少3个点（满足要求）
- ✅ 段数 = 点数 - 1，参数数量 = 段数 = 点数 - 1（满足要求）

**结论**：✅ **完全匹配，可以直接使用 lina planning 的多节点规划功能**

### 1.2 ROS2 JointTrajectory 消息格式

```cpp
trajectory_msgs::msg::JointTrajectory
{
    std_msgs::msg::Header header;
    string[] joint_names;                    // 关节名称列表
    JointTrajectoryPoint[] points;            // 轨迹点列表
}

trajectory_msgs::msg::JointTrajectoryPoint
{
    float64[] positions;                      // 关节位置（必需）
    float64[] velocities;                     // 关节速度（可选）
    float64[] accelerations;                  // 关节加速度（可选）
    builtin_interfaces::Duration time_from_start;  // 从开始的时间偏移
}
```

### 1.3 关键信息提取

对于只考虑位置信息的情况：
- **位置信息**：从每个 `point.positions` 提取
- **时间信息**：从每个 `point.time_from_start` 计算每段持续时间
- **关节映射**：通过 `joint_names` 匹配到控制器的关节顺序

## 二、设计方案

### 2.1 接口设计

#### 方法 1：直接设置轨迹消息
```cpp
void setTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory);
```

**注意**：`trajectory_duration` 作为 `JointTrajectoryManager` 的类成员变量，可以通过 setter 方法设置：
```cpp
void setTrajectoryDuration(double duration);
```

#### 方法 2：通过订阅接收轨迹消息（可选）
```cpp
void setupTrajectorySubscription(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    const std::string& topic_name = "joint_trajectory"
);
```

**注意**：`trajectory_duration` 作为 `JointTrajectoryManager` 的类成员变量，在初始化时设置一次即可。

### 2.2 数据转换流程

```
JointTrajectory 消息
    ↓
1. 验证消息有效性（关节名称匹配、输入点数 >= 2）
    ↓
2. 获取当前位置作为起点（第一个点）
   - 从 joint_position_state_interface_ 读取当前关节位置
   - 作为轨迹的第一个 waypoint
    ↓
3. 提取输入轨迹的位置信息
   - 遍历 trajectory.points（输入的点，至少2个）
   - 提取每个 point.positions → 后续 waypoints
   - 最终 waypoints = [当前位置] + [输入的点]
   - 总点数 >= 3（当前位置 + 输入的2个点）
    ↓
4. 计算每段持续时间
   - 使用 trajectory_duration 参数作为总时长
   - 按路径长度比例分配到各段
   - → segment_durations（段数 = waypoints.size() - 1）
    ↓
5. 关节名称映射（如果需要）
   - 匹配 trajectory.joint_names 与控制器关节顺序
   - 重新排列位置数据（仅对输入的点进行映射）
   - 当前位置保持原样
    ↓
6. 应用关节限制（如果设置了 limit_checker）
   - 对每个 waypoint 应用限制
    ↓
7. 调用 JointTrajectoryManager::initMultiNode()
```

**关键设计决策**：
- ✅ **当前位置作为起点**：确保轨迹总是从当前状态开始，避免跳跃
- ✅ **输入要求**：输入轨迹至少2个点，加上当前位置，总轨迹至少3个点
- ✅ **简化逻辑**：不需要处理"是否使用轨迹第一个点作为起点"的复杂情况
- ✅ **lina planning 兼容**：总轨迹至少3个点，满足 `SmoothCurveOfMultiJointsUsingBlending` 的要求

**数据流验证**：
```
输入：JointTrajectory（2个点）
  ↓
+ 当前位置（1个点）
  ↓
= 总轨迹（3个点）✅ >= 3
  ↓
段数 = 3 - 1 = 2
参数数量 = 2 ✅ = 点数 - 1
  ↓
满足 lina planning 要求：multiple_parameters.size() + 1 == multiple_points.size() ✅
```

### 2.3 关节名称映射策略

**情况 1：完全匹配**
- `trajectory.joint_names` 与控制器关节名称完全一致
- 直接使用位置数据

**情况 2：部分匹配**
- 只匹配部分关节（例如：只控制左臂或右臂）
- 未匹配的关节保持当前位置

**情况 3：顺序不同**
- 关节名称相同但顺序不同
- 需要重新排列位置数据

**情况 4：不匹配**
- 无法匹配任何关节
- 返回错误，不执行轨迹

### 2.4.1 NONE 插值类型的处理

**NONE 类型的特殊行为**：

当插值类型为 `NONE` 时，`calculatePhase()` 函数直接返回 `phase = 1.0`：

```cpp
double JointTrajectoryManager::calculatePhase(InterpolationType type, double percent) const
{
    if (type == InterpolationType::NONE)
    {
        return 1.0;  // 直接返回 1.0，表示立即到达目标
    }
    // ... 其他插值类型的处理
}
```

**单节点轨迹（NONE）**：
- 每个时间步都输出目标位置：`interpolated_value = 1.0 * target_pos[i] + 0.0 * start_pos[i] = target_pos[i]`
- 仍然会按照设定的 `duration` 时间执行（`updateProgress()` 会更新进度）
- 但每个时间步都直接输出目标位置，实际上就是"立即"到达目标

**多节点轨迹（NONE）**：
- 每个段都立即跳转到下一个 waypoint：`interpolated_value = 1.0 * end_waypoint[i] + 0.0 * start_waypoint[i] = end_waypoint[i]`
- 仍然会按照设定的各段 `duration` 时间执行
- 每个段都会立即跳转到目标，但仍然会等待设定的时间才进入下一段

**关键点**：
- ✅ NONE 类型仍然会按照设定的时间执行（保持时间同步）
- ✅ 但每个时间步都输出目标位置，实现"立即"到达的效果
- ✅ 对于多节点轨迹，每个段都会立即跳转到下一个 waypoint
- ⚠️ **注意**：NONE 类型不会使用 lina planning（DOUBLES 模式），因为 NONE 是基础插值类型

### 2.4 时间处理策略

**设计决策**：暂时不考虑从消息中提取时间信息，使用 `trajectory_duration` 参数设定总时长。

#### 策略 1：基础模式（TANH/LINEAR/NONE）- 手动计算各段时长

对于基础插值模式，需要手动计算各段时长并按路径长度比例分配：

```cpp
// 使用 trajectory_duration 作为总时长
// 按路径长度比例分配到各段
double total_duration = trajectory_duration;  // 从参数获取
std::vector<double> segment_durations;

// 计算每段的路径长度（欧氏距离）
std::vector<double> segment_lengths;
for (size_t i = 0; i < waypoints.size() - 1; ++i) {
    double length = 0.0;
    for (size_t j = 0; j < waypoints[i].size(); ++j) {
        double diff = waypoints[i+1][j] - waypoints[i][j];
        length += diff * diff;  // 欧氏距离的平方
    }
    segment_lengths.push_back(std::sqrt(length));
}

// 计算总长度并按比例分配时间
double total_length = std::accumulate(segment_lengths.begin(), segment_lengths.end(), 0.0);
if (total_length > 1e-6) {
    for (double len : segment_lengths) {
        segment_durations.push_back(total_duration * (len / total_length));
    }
} else {
    // 如果所有段长度都为0，平均分配
    double avg_duration = total_duration / segment_lengths.size();
    segment_durations.assign(segment_lengths.size(), avg_duration);
}
```

#### 策略 2：lina planning 模式（DOUBLES）- 自动计算 ⭐

**重要发现**：lina planning 支持自动计算各段时长，不需要手动计算！

lina planning 的 `TrajectoryParameter` 有一个 `time_mode` 标志：
- 当 `time_mode = true` 时，会根据 `total_time` 和 `joint_max_vel` **自动计算**每段的时间
- 使用 `calculate_norm_time_of_joint()` 计算每段的基础时间，然后按比例缩放以匹配 `total_time`

**实现方式**：
```cpp
// 对于 DOUBLES 模式，使用 lina planning 的自动时间计算
if (type == InterpolationType::DOUBLES && isDoublesAvailable()) {
    // 创建单个 TrajectoryParameter（不是每段一个）
    planning::TrajectoryParameter param(trajectory_duration, nr_of_joints);
    param.time_mode = true;  // 启用自动时间计算
    
    // 设置关节最大速度（可以从控制器获取或使用默认值）
    // param.joint_max_vel = ...;  // 如果需要自定义
    
    // 使用单个参数，lina planning 会自动为每段计算时间
    planning::TrajectoryInitParameters init_params(
        trajectory_points, param, period_);  // 注意：传入单个 param，不是 vector
    
    multi_node_planner_->init(init_params);
}
```

**优势**：
- ✅ **自动计算**：lina planning 根据路径长度和最大速度自动计算各段时间
- ✅ **更精确**：考虑了关节速度限制，比简单的路径长度比例更合理
- ✅ **简化实现**：不需要手动计算各段时长
- ✅ **统一参数**：只需要设置 `trajectory_duration` 总时长

**注意**：
- 基础模式（TANH/LINEAR/NONE）仍需要手动计算各段时长
- lina planning 模式（DOUBLES）使用自动计算，不需要手动计算

### 2.5 错误处理

1. **空轨迹**：输入轨迹点数量 < 2（加上当前位置后总点数 < 3）
2. **关节数量不匹配**：位置数量与关节数量不一致
3. **关节名称不匹配**：无法匹配任何关节
4. **trajectory_duration 无效**：必须 > 0
5. **关节限制违反**：目标位置超出限制（会应用限制，但记录警告）

## 三、实现细节

### 3.1 头文件修改

```cpp
// StateMoveJ.h
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class StateMoveJ {
public:
    // 新增方法
    void setTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory);
    
    // 可选：订阅接口
    void setupTrajectorySubscription(
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
        const std::string& topic_name = "joint_trajectory"
    );

private:
    // 辅助方法
    bool validateTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory);
    std::vector<size_t> mapJointNames(const std::vector<std::string>& trajectory_joint_names);
    std::vector<double> calculateSegmentDurations(
        const std::vector<std::vector<double>>& waypoints,
        double total_duration);
    
    // 订阅相关
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_subscription_;
    bool trajectory_subscription_setup_{false};
};
```

**注意**：`trajectory_duration` 作为 `JointTrajectoryManager` 的类成员变量，可以通过以下方式设置：
```cpp
// 在 StateMoveJ 构造函数或初始化时
trajectory_manager_.setTrajectoryDuration(trajectory_duration);  // 从参数获取
```

### 3.2 实现逻辑

```cpp
void StateMoveJ::setTrajectory(const trajectory_msgs::msg::JointTrajectory& trajectory)
{
    std::lock_guard lock(target_mutex_);
    
    // 1. 验证消息
    if (!validateTrajectory(trajectory)) {
        return;
    }
    
    // 获取 trajectory_duration 从 JointTrajectoryManager
    double trajectory_duration = trajectory_manager_.getTrajectoryDuration();
    if (trajectory_duration <= 0.0) {
        RCLCPP_ERROR(logger_,
                    "Invalid trajectory_duration: %.3f, must be positive. "
                    "Please set it using setTrajectoryDuration() first.",
                    trajectory_duration);
        return;
    }
    
    // 2. 映射关节名称
    std::vector<size_t> joint_indices = mapJointNames(trajectory.joint_names);
    if (joint_indices.empty()) {
        RCLCPP_ERROR(logger_, "Cannot map trajectory joint names to controller joints");
        return;
    }
    
    // 3. 获取当前位置作为起点（第一个 waypoint）
    std::vector<double> current_positions;
    current_positions.reserve(joint_names_.size());
    for (auto i : ctrl_interfaces_.joint_position_state_interface_) {
        auto value = i.get().get_optional();
        current_positions.push_back(value.value_or(0.0));
    }
    
    // 应用关节限制到当前位置
    current_positions = applyJointLimits(current_positions, "current position");
    
    // 4. 提取输入轨迹的位置信息（按控制器关节顺序）
    std::vector<std::vector<double>> waypoints;
    waypoints.reserve(trajectory.points.size() + 1);  // +1 为当前位置
    
    // 先添加当前位置作为起点
    waypoints.push_back(current_positions);
    
    // 然后添加输入轨迹的点
    for (const auto& point : trajectory.points) {
        if (point.positions.size() != trajectory.joint_names.size()) {
            RCLCPP_ERROR(logger_, "Position size mismatch in trajectory point");
            return;
        }
        
        std::vector<double> mapped_positions(joint_names_.size());
        // 初始化：使用当前位置（对于未匹配的关节）
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            if (i < current_positions.size()) {
                mapped_positions[i] = current_positions[i];
            }
        }
        
        // 映射轨迹中的位置
        for (size_t i = 0; i < joint_indices.size(); ++i) {
            size_t controller_idx = joint_indices[i];
            if (controller_idx < mapped_positions.size() && i < point.positions.size()) {
                mapped_positions[controller_idx] = point.positions[i];
            }
        }
        
        // 应用关节限制
        mapped_positions = applyJointLimits(mapped_positions, "trajectory waypoint");
        
        waypoints.push_back(mapped_positions);
    }
    
    // 5. 计算每段持续时间（仅用于基础模式）
    // 注意：对于 DOUBLES 模式，lina planning 会自动计算，不需要手动计算
    std::vector<double> durations;
    double trajectory_duration = trajectory_manager_.getTrajectoryDuration();
    
    if (interpolation_type_ != InterpolationType::DOUBLES || 
        !JointTrajectoryManager::isDoublesAvailable()) {
        // 基础模式：手动计算各段时长（按路径长度比例分配）
        durations = calculateSegmentDurations(waypoints, trajectory_duration);
    }
    // DOUBLES 模式：durations 保持为空，lina planning 会自动计算
    
    // 6. 初始化多节点轨迹
        // waypoints 现在包含：当前位置 + 输入的轨迹点（至少2个）= 至少3个点
        // 这满足 lina planning 的 SmoothCurveOfMultiJointsUsingBlending 要求（至少3个点）
        if (waypoints.size() >= 3) {
            size_t num_segments = waypoints.size() - 1;
            
            trajectory_manager_.initMultiNode(
                waypoints,
                durations,  // 基础模式：包含计算的各段时长；DOUBLES 模式：为空，自动计算
                interpolation_type_,
                ctrl_interfaces_.frequency_,
                trajectory_duration / num_segments,  // 默认持续时间（平均时间，作为后备）
                tanh_scale_
            );
            
            has_target_ = true;
            interpolation_active_ = false;  // 将在 run() 中激活
            
            RCLCPP_INFO(logger_,
                       "Trajectory loaded: %zu waypoints (1 current + %zu input), %zu segments. "
                       "Compatible with lina planning multi-node planner.",
                       waypoints.size(), trajectory.points.size(), num_segments);
        }
        else
        {
            RCLCPP_ERROR(logger_,
                        "Invalid trajectory: need at least 2 input points (total >= 3 with current position), got %zu",
                        trajectory.points.size());
        }
}
```

### 3.3 关节名称映射实现

```cpp
std::vector<size_t> StateMoveJ::mapJointNames(
    const std::vector<std::string>& trajectory_joint_names)
{
    std::vector<size_t> indices;
    indices.reserve(trajectory_joint_names.size());
    
    for (const auto& traj_joint_name : trajectory_joint_names) {
        bool found = false;
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            if (joint_names_[i] == traj_joint_name) {
                indices.push_back(i);
                found = true;
                break;
            }
        }
        if (!found) {
            RCLCPP_WARN(logger_,
                       "Trajectory joint '%s' not found in controller joints",
                       traj_joint_name.c_str());
            // 可以选择：返回空（严格匹配）或跳过（部分匹配）
            // 这里选择严格匹配
            return std::vector<size_t>();
        }
    }
    
    return indices;
}
```

### 3.4 时间提取实现

```cpp
std::vector<double> StateMoveJ::calculateSegmentDurations(
    const std::vector<std::vector<double>>& waypoints,
    double total_duration)
{
    std::vector<double> durations;
    
    if (waypoints.size() < 2) {
        return durations;  // 空向量，将使用默认时间
    }
    
    size_t num_segments = waypoints.size() - 1;
    durations.reserve(num_segments);
    
    // 计算每段的路径长度（欧氏距离）
    std::vector<double> segment_lengths;
    segment_lengths.reserve(num_segments);
    
    for (size_t i = 0; i < num_segments; ++i) {
        double length = 0.0;
        const auto& start = waypoints[i];
        const auto& end = waypoints[i + 1];
        
        if (start.size() != end.size()) {
            RCLCPP_ERROR(logger_,
                        "Waypoint size mismatch: waypoint %zu has %zu joints, waypoint %zu has %zu joints",
                        i, start.size(), i + 1, end.size());
            durations.clear();
            return durations;
        }
        
        for (size_t j = 0; j < start.size(); ++j) {
            double diff = end[j] - start[j];
            length += diff * diff;  // 欧氏距离的平方
        }
        segment_lengths.push_back(std::sqrt(length));
    }
    
    // 计算总长度
    double total_length = 0.0;
    for (double len : segment_lengths) {
        total_length += len;
    }
    
    // 按路径长度比例分配时间
    if (total_length > 1e-6) {  // 避免除零
        for (double len : segment_lengths) {
            double segment_duration = total_duration * (len / total_length);
            durations.push_back(segment_duration);
        }
    }
    else {
        // 如果所有段长度都为0（所有点相同），平均分配
        double avg_duration = total_duration / num_segments;
        durations.assign(num_segments, avg_duration);
        RCLCPP_WARN(logger_,
                   "All waypoints are identical, evenly distributing duration %.3f across %zu segments",
                   total_duration, num_segments);
    }
    
    return durations;
}
```

## 四、使用示例

### 4.1 编程接口

```cpp
// 创建轨迹消息
// 注意：只需要提供目标点，当前位置会自动作为起点
// 时间信息暂时不使用，通过 trajectory_duration 参数设定总时长
trajectory_msgs::msg::JointTrajectory trajectory;
trajectory.joint_names = {"joint1", "joint2", "joint3"};

// 添加第一个目标点（至少需要2个点）
trajectory_msgs::msg::JointTrajectoryPoint point1;
point1.positions = {1.0, 0.5, 0.3};
// time_from_start 暂时不使用，可以设置为任意值
trajectory.points.push_back(point1);

// 添加第二个目标点
trajectory_msgs::msg::JointTrajectoryPoint point2;
point2.positions = {2.0, 1.0, 0.6};
trajectory.points.push_back(point2);

// 设置轨迹总时长（在初始化时设置一次）
double trajectory_duration = 5.0;  // 从参数获取或直接设置
trajectory_manager_.setTrajectoryDuration(trajectory_duration);

// 设置轨迹
// 实际轨迹：当前位置 → point1 → point2（3个点，2段）
// 时间分配：按路径长度比例分配总时长
state_movej.setTrajectory(trajectory);
```

**时间说明**：
- `trajectory_duration`：总轨迹时长（秒）
- 时间分配：按各段路径长度比例自动分配
  - 例如：总时长5秒，第一段路径长度是第二段的2倍
  - 第一段：5.0 * (2/3) ≈ 3.33秒
  - 第二段：5.0 * (1/3) ≈ 1.67秒
- `time_from_start`：暂时不使用，未来可能支持

### 4.2 订阅接口

```cpp
// 在控制器初始化时设置 trajectory_duration（从参数获取）
double trajectory_duration = auto_declare("trajectory_duration", 2.0);
trajectory_manager_.setTrajectoryDuration(trajectory_duration);

// 设置订阅
state_movej.setupTrajectorySubscription(node, "joint_trajectory");

// 通过话题发布轨迹
// ros2 topic pub /controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{...}"
// 注意：消息中的 time_from_start 暂时不使用，总时长由 trajectory_duration 成员变量控制
```

## 五、优势分析

### 5.1 标准化
- ✅ 使用标准 ROS2 消息格式
- ✅ 与其他 ROS2 工具兼容（MoveIt、trajectory planners 等）

### 5.2 灵活性
- ✅ 支持任意数量的轨迹点（至少2个输入点）
- ✅ 自动处理时间信息
- ✅ 支持部分关节控制

### 5.3 兼容性
- ✅ 与现有单节点接口共存
- ✅ 使用统一的 JointTrajectoryManager
- ✅ 支持所有插值类型（TANH/LINEAR/DOUBLES/NONE）

### 5.4 lina planning 兼容性 ⭐
- ✅ **完全兼容**：总轨迹至少3个点，满足 `SmoothCurveOfMultiJointsUsingBlending` 的要求
- ✅ **参数匹配**：参数数量 = 点数 - 1，满足 `multiple_parameters.size() + 1 == multiple_points.size()`
- ✅ **自动选择**：当使用 DOUBLES 插值类型时，自动使用 lina planning 的多节点规划
- ✅ **平滑轨迹**：利用 lina planning 的 movej + bezier 混合，生成平滑的多段轨迹

## 六、实现建议

### 6.1 实现步骤

1. **添加依赖**
   - 在 CMakeLists.txt 中添加 `trajectory_msgs` 依赖
   - 确保 `trajectory_duration` 参数可以从控制器参数中获取

2. **修改 JointTrajectoryManager**
   - 在头文件中添加 `trajectory_duration_` 成员变量（类似 `duration_`）
   - 添加 `setTrajectoryDuration(double duration)` 和 `getTrajectoryDuration() const` 方法
   - 在 `reset()` 方法中重置 `trajectory_duration_` 为默认值（例如 3.0）
   
   ```cpp
   // JointTrajectoryManager.h
   class JointTrajectoryManager {
   public:
       void setTrajectoryDuration(double duration);
       double getTrajectoryDuration() const;
   
   private:
       double trajectory_duration_{3.0};  // 默认3秒
   };
   ```

3. **修改 StateMoveJ 头文件**
   - 添加 `#include <trajectory_msgs/msg/joint_trajectory.hpp>`
   - 添加 `setTrajectory()` 方法声明
   - 添加辅助方法声明

4. **实现核心逻辑**
   - 实现 `setTrajectory()` 方法
   - 实现关节名称映射
   - 实现时间计算（使用 `trajectory_manager_.getTrajectoryDuration()`）

5. **可选：添加订阅**
   - 实现 `setupTrajectorySubscription()` 方法

6. **测试**
   - 测试单点轨迹（2个点）
   - 测试多点轨迹（3+个点）
   - 测试关节名称匹配
   - 测试时间计算

### 6.2 注意事项

1. **线程安全**：`setTrajectory()` 需要使用 `target_mutex_` 保护
2. **状态管理**：设置轨迹后，需要在 `run()` 中正确激活
3. **关节限制**：每个轨迹点（包括当前位置）都需要应用关节限制
4. **起点处理**：**总是使用当前位置作为起点**，输入轨迹的点作为后续路径点
5. **输入要求**：输入轨迹至少2个点，加上当前位置后总轨迹至少3个点
6. **时间处理**：
   - **基础模式（TANH/LINEAR/NONE）**：手动计算各段时长，按路径长度比例分配
   - **lina planning 模式（DOUBLES）**：使用 `time_mode = true`，lina planning 自动计算各段时长
   - 使用 `trajectory_duration` 成员变量设定总时长
   - 暂时不考虑消息中的 `time_from_start` 信息
7. **参数来源**：
   - `trajectory_duration` 作为 `JointTrajectoryManager` 的类成员变量
   - 可以通过 `setTrajectoryDuration()` 方法设置
   - 在控制器初始化时从参数获取（与 ocs2_arm_controller 一致）
   - 设置一次后，后续所有轨迹都使用该值

## 七、总结

通过添加 `trajectory_msgs::msg::JointTrajectory` 接口，`StateMoveJ` 可以：
1. ✅ 接收标准 ROS2 轨迹消息
2. ✅ 自动提取位置信息
3. ✅ 使用 `JointTrajectoryManager::trajectory_duration_` 成员变量控制总时长
   - **基础模式**：按路径长度比例手动分配各段时长
   - **lina planning 模式**：自动计算各段时长（更精确，考虑速度限制）
4. ✅ 使用多节点轨迹规划（基础或进阶）
5. ✅ **完全兼容 lina planning 的多节点规划接口**
6. ✅ 与现有接口兼容
7. ✅ 支持部分关节控制

**设计优势**：
- ✅ `trajectory_duration` 作为类成员变量，设置一次后所有轨迹都使用该值
- ✅ 简化接口：`setTrajectory()` 不需要额外参数
- ✅ 统一管理：与 `duration_` 类似的设计模式
- ✅ 易于配置：在控制器初始化时从参数获取并设置

### 7.1 lina planning 兼容性确认

**接口匹配验证**：

| 要求 | lina planning | 我们的设计 | 状态 |
|------|--------------|-----------|------|
| 最小点数 | `>= 3` | 当前位置(1) + 输入(2) = 3 | ✅ |
| 参数数量 | `点数 - 1` | 段数 = 点数 - 1 | ✅ |
| 数据格式 | `TrajectPoint[]` | 自动转换 | ✅ |
| 时间参数 | `TrajectoryParameter[]` | 自动创建 | ✅ |

**结论**：✅ **完全匹配，可以直接使用 lina planning 的 `SmoothCurveOfMultiJointsUsingBlending` 进行多节点轨迹规划**

当使用 `DOUBLES` 插值类型时：
- 输入轨迹（至少2个点）+ 当前位置 = 至少3个点
- 自动转换为 `TrajectPoint` 数组
- 创建单个 `TrajectoryParameter`，设置 `time_mode = true`
- lina planning **自动计算**各段时长（根据路径长度和最大速度）
- 调用 `SmoothCurveOfMultiJointsUsingBlending::init()` 进行平滑轨迹规划
- 生成 movej + bezier 混合的平滑多段轨迹

**关键优势**：lina planning 的自动时间计算比手动按路径长度比例分配更精确，因为它考虑了关节速度限制。

这个设计充分利用了现有的 `JointTrajectoryManager`，实现了代码复用和功能扩展，同时完全兼容 lina planning 的高级轨迹规划能力。

