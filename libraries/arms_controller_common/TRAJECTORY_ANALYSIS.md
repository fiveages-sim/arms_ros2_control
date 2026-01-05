# StateMoveJ 轨迹插值功能实现分析

## 设计理念：统一轨迹插值

### 核心设计原则
**所有运动都视为轨迹插值，统一从当前位置开始**

1. **单点目标** = 仅包含目标点的轨迹（起始点自动为当前位置）
2. **多点轨迹** = 当前位置（自动） -> 轨迹点1 -> 轨迹点2 -> ... -> 轨迹点N

这样的设计优势：
- 代码逻辑统一，无需区分两种模式
- 单点和多点使用相同的插值逻辑
- 更容易维护和扩展

## 当前实现分析

### 1. 现有功能
- `StateMoveJ` 当前支持从起始位置到目标位置的单点插值
- 使用 `duration_` 参数控制插值总时长
- 支持线性（LINEAR）和双曲正切（TANH）两种插值类型
- 通过 `percent_` 变量跟踪插值进度（0.0 到 1.0）
- 在 `run()` 方法中根据控制器频率更新 `percent_`

### 2. 关键代码位置

#### 当前插值逻辑（StateMoveJ.cpp:72-190）
```cpp
void StateMoveJ::run(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    // 1. 检查是否有目标位置
    // 2. 更新插值进度 percent_ += 1.0 / (duration_ * controller_frequency)
    // 3. 计算插值相位 phase
    // 4. 应用插值: interpolated_value = phase * target_pos_[i] + (1.0 - phase) * start_pos_[i]
}
```

## 统一实现方案

### 方案概述
将所有运动统一为轨迹插值模式：
- 单点目标：自动构建为 [当前位置, 目标位置] 的轨迹
- 多点轨迹：自动构建为 [当前位置, 轨迹点1, 轨迹点2, ..., 轨迹点N] 的轨迹
- 所有轨迹都从当前位置开始，使用统一的插值逻辑

### 核心设计思路

1. **统一轨迹模式**：
   - 所有运动都使用轨迹数据结构
   - 单点目标自动转换为两点轨迹（当前位置 -> 目标位置）
   - 多点轨迹自动添加当前位置作为起始点

2. **轨迹数据结构**：
   - 存储轨迹点列表，每个点包含归一化时间和关节位置
   - 时间归一化：将轨迹中的时间映射到 `duration_` 参数指定的总时长
   - 第一个点始终是当前位置（时间 0.0）

3. **插值策略**：
   - 根据当前时间找到相邻的两个轨迹点
   - 在这两个点之间进行线性插值
   - 支持关节名称匹配和前缀过滤

## 详细实现步骤

### 步骤 1: 修改头文件 (StateMoveJ.h)

在 `private` 部分添加轨迹相关成员：

```cpp
// 轨迹点结构
struct TrajectoryPoint {
    double normalized_time;  // 归一化时间 [0.0, 1.0]，相对于 duration_
    std::vector<double> positions;  // 关节位置
};

// 轨迹相关成员变量
std::vector<TrajectoryPoint> trajectory_points_;  // 存储轨迹点（第一个点始终是当前位置）
rclcpp::Time trajectory_start_time_;  // 轨迹开始执行的时间
bool has_trajectory_{false};  // 是否有有效轨迹
bool trajectory_active_{false};  // 轨迹是否正在执行
rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_subscription_;
```

在 `private` 部分添加内部方法：

```cpp
/**
 * @brief 构建轨迹（内部方法，统一处理单点和多点）
 * @param points 目标点列表（不包含起始点，起始点自动为当前位置）
 * @param times 每个点的时间（相对于轨迹开始，如果为空则均匀分配）
 */
void buildTrajectory(const std::vector<std::vector<double>>& points,
                     const std::vector<double>& times = {});
```

在 `public` 部分添加方法声明：

```cpp
/**
 * @brief 设置轨迹（从 JointTrajectory 消息）
 * @param msg JointTrajectory 消息
 */
void setTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr& msg);
```

### 步骤 2: 实现统一的轨迹构建方法

关键实现逻辑：**所有目标都转换为轨迹，第一个点始终是当前位置**

```cpp
/**
 * @brief 构建轨迹（内部方法，统一处理单点和多点）
 * @param points 目标点列表（不包含起始点，起始点自动为当前位置）
 * @param times 每个点的时间（相对于轨迹开始，如果为空则均匀分配）
 */
void StateMoveJ::buildTrajectory(const std::vector<std::vector<double>>& points,
                                 const std::vector<double>& times = {})
{
    std::lock_guard lock(target_mutex_);
    
    // 1. 获取当前位置作为起始点
    std::vector<double> current_positions;
    for (auto i : ctrl_interfaces_.joint_position_state_interface_)
    {
        auto value = i.get().get_optional();
        current_positions.push_back(value.value_or(0.0));
    }
    
    // 2. 清空并重建轨迹点
    trajectory_points_.clear();
    
    // 3. 添加起始点（当前位置，时间 0.0）
    TrajectoryPoint start_point;
    start_point.normalized_time = 0.0;
    start_point.positions = current_positions;
    trajectory_points_.push_back(start_point);
    
    // 4. 处理时间分配
    double max_time = 0.0;
    if (!times.empty() && times.size() == points.size())
    {
        // 使用提供的时间
        max_time = *std::max_element(times.begin(), times.end());
    }
    else if (points.size() > 1)
    {
        // 均匀分配时间
        max_time = duration_;
    }
    else
    {
        max_time = duration_;
    }
    
    // 5. 添加目标点
    for (size_t i = 0; i < points.size(); ++i)
    {
        TrajectoryPoint traj_point;
        
        // 计算归一化时间
        if (!times.empty() && i < times.size())
        {
            traj_point.normalized_time = times[i] / max_time;
        }
        else
        {
            // 均匀分配：第一个目标点在 1/(N+1)，最后一个在 1.0
            traj_point.normalized_time = static_cast<double>(i + 1) / points.size();
        }
        
        // 确保时间在 [0.0, 1.0] 范围内
        traj_point.normalized_time = std::clamp(traj_point.normalized_time, 0.0, 1.0);
        
        // 设置关节位置
        traj_point.positions.resize(joint_names_.size());
        if (i < points.size() && points[i].size() == joint_names_.size())
        {
            traj_point.positions = points[i];
        }
        else
        {
            // 如果大小不匹配，使用当前位置并警告
            RCLCPP_WARN(logger_, 
                       "Point %zu size (%zu) doesn't match joint count (%zu), using current position",
                       i, points[i].size(), joint_names_.size());
            traj_point.positions = current_positions;
        }
        
        trajectory_points_.push_back(traj_point);
    }
    
    // 6. 应用关节限制检查
    if (joint_limit_checker_)
    {
        for (auto& point : trajectory_points_)
        {
            point.positions = joint_limit_checker_(point.positions);
        }
    }
    
    // 7. 设置标志
    has_trajectory_ = true;
    trajectory_active_ = false;  // 将在 enter() 或 run() 中激活
    
    RCLCPP_INFO(logger_, "Built trajectory with %zu points (including start), duration: %.2f seconds",
               trajectory_points_.size(), duration_);
}

/**
 * @brief 设置轨迹（从 JointTrajectory 消息）
 * @param msg JointTrajectory 消息
 */
void StateMoveJ::setTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr& msg)
{
    std::lock_guard lock(target_mutex_);
    
    // 1. 验证消息有效性
    if (!msg || msg->points.empty() || msg->joint_names.empty())
    {
        RCLCPP_WARN(logger_, "Invalid trajectory message received");
        return;
    }
    
    // 2. 初始化关节名称（如果尚未初始化）
    if (joint_names_.empty())
    {
        initializeJointNames();
    }
    
    // 3. 建立关节名称映射
    std::vector<size_t> joint_indices;
    joint_indices.reserve(msg->joint_names.size());
    
    for (const auto& traj_joint_name : msg->joint_names)
    {
        bool found = false;
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            if (joint_names_[i] == traj_joint_name)
            {
                joint_indices.push_back(i);
                found = true;
                break;
            }
        }
        if (!found)
        {
            RCLCPP_WARN(logger_, "Joint '%s' from trajectory not found in controller", 
                       traj_joint_name.c_str());
            joint_indices.push_back(SIZE_MAX);  // 标记为无效
        }
    }
    
    // 4. 提取轨迹点的时间信息
    std::vector<double> times;
    double max_time = 0.0;
    
    for (const auto& point : msg->points)
    {
        double time_from_start = point.time_from_start.sec + 
                                 point.time_from_start.nanosec * 1e-9;
        times.push_back(time_from_start);
        if (time_from_start > max_time)
        {
            max_time = time_from_start;
        }
    }
    
    // 如果轨迹中没有时间信息，均匀分配时间
    if (max_time <= 0.0 && msg->points.size() > 1)
    {
        max_time = duration_;  // 使用默认 duration
        times.clear();
        for (size_t i = 0; i < msg->points.size(); ++i)
        {
            times.push_back(static_cast<double>(i + 1) / msg->points.size() * duration_);
        }
    }
    else if (max_time <= 0.0)
    {
        max_time = duration_;
        times = {duration_};
    }
    
    // 5. 提取轨迹点的位置信息
    std::vector<std::vector<double>> points;
    std::vector<double> current_positions;
    
    // 获取当前位置
    for (auto i : ctrl_interfaces_.joint_position_state_interface_)
    {
        auto value = i.get().get_optional();
        current_positions.push_back(value.value_or(0.0));
    }
    
    for (const auto& point : msg->points)
    {
        std::vector<double> positions(joint_names_.size());
        
        // 初始化所有关节为当前位置
        positions = current_positions;
        
        // 填充匹配的关节位置
        for (size_t i = 0; i < joint_indices.size() && i < point.positions.size(); ++i)
        {
            if (joint_indices[i] != SIZE_MAX && joint_indices[i] < joint_names_.size())
            {
                positions[joint_indices[i]] = point.positions[i];
            }
        }
        
        points.push_back(positions);
    }
    
    // 6. 调用统一的轨迹构建方法
    buildTrajectory(points, times);
}
```

### 步骤 3: 修改 setTargetPosition 方法

将单点目标转换为轨迹（仅包含目标点的轨迹）：

```cpp
void StateMoveJ::setTargetPosition(const std::vector<double>& target_pos)
{
    std::lock_guard lock(target_mutex_);
    
    // 检查状态是否激活
    if (!state_active_)
    {
        RCLCPP_WARN(logger_, "Cannot set target position: StateMoveJ is not active. Please enter MOVEJ state first.");
        return;
    }
    
    // 禁用前缀过滤
    use_prefix_filter_ = false;
    active_prefix_.clear();
    joint_mask_.clear();
    if (!joint_names_.empty())
    {
        joint_mask_.resize(joint_names_.size(), true);
    }
    
    // 验证目标位置大小
    if (target_pos.size() != joint_names_.size())
    {
        RCLCPP_WARN(logger_, 
                   "Target position size (%zu) doesn't match joint count (%zu)",
                   target_pos.size(), joint_names_.size());
        return;
    }
    
    // 检查是否与当前目标相同
    if (has_trajectory_ && !trajectory_points_.empty())
    {
        const auto& last_point = trajectory_points_.back();
        bool is_same = true;
        for (size_t i = 0; i < target_pos.size() && i < last_point.positions.size(); ++i)
        {
            if (std::abs(target_pos[i] - last_point.positions[i]) > TARGET_EPSILON)
            {
                is_same = false;
                break;
            }
        }
        if (is_same && trajectory_active_)
        {
            RCLCPP_DEBUG(logger_, "Received same target position, skipping re-interpolation");
            return;
        }
    }
    
    // 将单点目标转换为轨迹（仅包含目标点）
    std::vector<std::vector<double>> points = {target_pos};
    buildTrajectory(points);  // 会自动添加当前位置作为起始点
}
```

### 步骤 4: 修改 run() 方法

统一使用轨迹插值逻辑（不再区分单点和多点）：

```cpp
void StateMoveJ::run(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    std::lock_guard lock(target_mutex_);
    
    // === 统一轨迹插值模式 ===
    if (!has_trajectory_ || trajectory_points_.empty())
    {
        // 没有轨迹，保持当前位置
        std::vector<double> current_positions;
        for (auto i : ctrl_interfaces_.joint_position_state_interface_)
        {
            auto value = i.get().get_optional();
            current_positions.push_back(value.value_or(0.0));
        }
        
        for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size() && 
             i < current_positions.size(); ++i)
        {
            std::ignore = ctrl_interfaces_.joint_position_command_interface_[i]
                .get().set_value(current_positions[i]);
        }
        return;
    }
    
    // 如果轨迹刚激活，记录开始时间并更新起始点
    if (!trajectory_active_)
    {
        trajectory_start_time_ = time;
        trajectory_active_ = true;
        
        // 更新起始点为当前位置（确保从当前位置开始）
        std::vector<double> current_positions;
        for (auto i : ctrl_interfaces_.joint_position_state_interface_)
        {
            auto value = i.get().get_optional();
            current_positions.push_back(value.value_or(0.0));
        }
        
        if (!trajectory_points_.empty())
        {
            trajectory_points_[0].positions = current_positions;
            trajectory_points_[0].normalized_time = 0.0;
        }
        
        RCLCPP_INFO(logger_, "Starting trajectory execution with %zu points", 
                   trajectory_points_.size());
    }
    
    // 计算从轨迹开始到当前的时间差
    rclcpp::Duration elapsed = time - trajectory_start_time_;
    double elapsed_seconds = elapsed.seconds();
    
    // 计算归一化进度 [0.0, 1.0]
    double normalized_progress = elapsed_seconds / duration_;
    
    // 限制在 [0.0, 1.0] 范围内
    normalized_progress = std::clamp(normalized_progress, 0.0, 1.0);
    
    // 找到当前时间对应的两个相邻轨迹点
    size_t point_idx = 0;
    bool found_segment = false;
    
    for (size_t i = 0; i < trajectory_points_.size() - 1; ++i)
    {
        if (normalized_progress >= trajectory_points_[i].normalized_time &&
            normalized_progress <= trajectory_points_[i + 1].normalized_time)
        {
            point_idx = i;
            found_segment = true;
            break;
        }
    }
    
    // 如果超出最后一个点，使用最后一个点
    if (!found_segment || normalized_progress >= trajectory_points_.back().normalized_time)
    {
        point_idx = trajectory_points_.size() - 1;
    }
    
    // 在两个相邻点之间进行插值
    std::vector<double> interpolated_positions;
    
    if (point_idx >= trajectory_points_.size() - 1)
    {
        // 使用最后一个点
        interpolated_positions = trajectory_points_.back().positions;
    }
    else
    {
        // 在两个点之间插值
        const auto& point1 = trajectory_points_[point_idx];
        const auto& point2 = trajectory_points_[point_idx + 1];
        
        double t1 = point1.normalized_time;
        double t2 = point2.normalized_time;
        double alpha = 0.0;
        
        if (std::abs(t2 - t1) > 1e-6)
        {
            alpha = (normalized_progress - t1) / (t2 - t1);
        }
        alpha = std::clamp(alpha, 0.0, 1.0);
        
        // 应用插值类型（LINEAR 或 TANH）
        double phase = alpha;
        if (interpolation_type_ == InterpolationType::TANH)
        {
            const double scale = (tanh_scale_ > 0.0) ? tanh_scale_ : 3.0;
            phase = std::tanh(alpha * scale) / std::tanh(scale);
        }
        phase = std::clamp(phase, 0.0, 1.0);
        
        // 线性插值
        interpolated_positions.resize(joint_names_.size());
        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            if (i < point1.positions.size() && i < point2.positions.size())
            {
                interpolated_positions[i] = (1.0 - phase) * point1.positions[i] + 
                                           phase * point2.positions[i];
            }
            else if (i < point1.positions.size())
            {
                interpolated_positions[i] = point1.positions[i];
            }
            else
            {
                interpolated_positions[i] = 0.0;
            }
        }
    }
    
    // 应用关节掩码（前缀过滤）
    if (use_prefix_filter_ && joint_mask_.size() == interpolated_positions.size())
    {
        std::vector<double> current_positions;
        for (auto i : ctrl_interfaces_.joint_position_state_interface_)
        {
            auto value = i.get().get_optional();
            current_positions.push_back(value.value_or(0.0));
        }
        
        for (size_t i = 0; i < interpolated_positions.size(); ++i)
        {
            if (!joint_mask_[i] && i < current_positions.size())
            {
                // 保持当前关节位置
                interpolated_positions[i] = current_positions[i];
            }
        }
    }
    
    // 应用插值位置到关节
    for (size_t i = 0; i < ctrl_interfaces_.joint_position_command_interface_.size() &&
         i < interpolated_positions.size(); ++i)
    {
        std::ignore = ctrl_interfaces_.joint_position_command_interface_[i]
            .get().set_value(interpolated_positions[i]);
    }
    
    // 重力补偿（如果启用）
    if (ctrl_interfaces_.control_mode_ == ControlMode::MIX && gravity_compensation_)
    {
        std::vector<double> static_torques = 
            gravity_compensation_->calculateStaticTorques(interpolated_positions);
        
        for (size_t i = 0; i < ctrl_interfaces_.joint_force_command_interface_.size() && 
             i < static_torques.size(); ++i)
        {
            std::ignore = ctrl_interfaces_.joint_force_command_interface_[i]
                .get().set_value(static_torques[i]);
        }
    }
}
```

### 步骤 4: 修改 setupSubscriptions 方法

在 `setupSubscriptions` 中添加轨迹订阅：

```cpp
void StateMoveJ::setupSubscriptions(...)
{
    // ... 现有代码 ...
    
    // 添加轨迹订阅
    trajectory_subscription_ = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        base_topic + "/trajectory", 10,
        [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
        {
            setTrajectory(msg);
        });
    RCLCPP_INFO(logger_, "Subscribed to %s/trajectory for joint trajectory", 
               base_topic.c_str());
    
    // ... 其余代码 ...
}
```

### 步骤 5: 修改 enter() 和 exit() 方法

在 `enter()` 中重置轨迹状态（但保留轨迹数据，等待执行）：

```cpp
void StateMoveJ::enter()
{
    // ... 现有代码 ...
    
    // 重置轨迹执行状态（但保留轨迹数据）
    trajectory_active_ = false;
    // 注意：不清除 trajectory_points_，因为可能已经设置了轨迹
    
    // 如果已有轨迹，将在 run() 中激活
    if (has_trajectory_ && !trajectory_points_.empty())
    {
        RCLCPP_INFO(logger_, "Entering MOVEJ state with existing trajectory (%zu points)",
                   trajectory_points_.size());
    }
    
    // ... 其余代码 ...
}
```

在 `exit()` 中清理轨迹状态：

```cpp
void StateMoveJ::exit()
{
    std::lock_guard lock(target_mutex_);
    
    // ... 现有代码 ...
    
    // 清理轨迹状态
    has_trajectory_ = false;
    trajectory_active_ = false;
    trajectory_points_.clear();
    
    // ... 其余代码 ...
}
```

## 关键设计决策

### 1. 统一轨迹设计
- **所有运动都是轨迹**：单点目标自动转换为两点轨迹（当前位置 -> 目标位置）
- **起始点自动添加**：所有轨迹的第一个点始终是当前位置（时间 0.0）
- **统一插值逻辑**：单点和多点使用完全相同的插值代码

### 2. 时间处理
- **归一化时间**：将轨迹中的时间归一化到 [0.0, 1.0]，然后映射到 `duration_`
- **时间戳使用**：使用 ROS 时间戳 `rclcpp::Time` 来跟踪轨迹开始时间
- **时间同步**：在 `run()` 中首次执行时记录开始时间，并更新起始点为当前位置
- **时间分配**：如果没有提供时间信息，均匀分配时间

### 3. 关节匹配
- **名称匹配**：通过关节名称字符串匹配（仅用于 JointTrajectory 消息）
- **位置映射**：对于 JointTrajectory，建立从轨迹关节索引到控制接口索引的映射
- **缺失处理**：如果轨迹中的关节在控制器中不存在，使用当前位置值

### 4. 插值策略
- **线性插值**：在相邻轨迹点之间使用线性插值
- **插值类型**：支持 LINEAR 和 TANH 两种类型（在段内应用）
- **边界处理**：超出轨迹范围时使用最后一个点
- **单点处理**：如果轨迹只有起始点和目标点（单点模式），正常插值

### 5. 状态管理
- **统一标志**：只使用 `has_trajectory_` 标志，不再需要 `has_target_`
- **执行状态**：`trajectory_active_` 表示轨迹是否正在执行
- **状态转换**：进入状态时保留轨迹数据，执行时激活；退出时清理所有数据

## 使用示例

### 发布轨迹消息

```cpp
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// 创建轨迹消息
auto traj = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
traj->joint_names = {"left_shoulder", "left_elbow", "left_wrist"};
traj->points.resize(3);

// 点1：起始位置
traj->points[0].positions = {0.0, 0.0, 0.0};
traj->points[0].time_from_start.sec = 0;
traj->points[0].time_from_start.nanosec = 0;

// 点2：中间位置
traj->points[1].positions = {1.0, 1.0, 1.0};
traj->points[1].time_from_start.sec = 1;
traj->points[1].time_from_start.nanosec = 0;

// 点3：目标位置
traj->points[2].positions = {2.0, 2.0, 2.0};
traj->points[2].time_from_start.sec = 2;
traj->points[2].time_from_start.nanosec = 0;

// 发布轨迹
trajectory_publisher_->publish(*traj);
```

### 配置说明

- **Topic 名称**：`{node_name}/target_joint_position/trajectory`
- **Duration 参数**：通过构造函数传入，控制轨迹执行总时长
- **时间归一化**：如果轨迹中的总时间为 T，duration_ 为 D，则时间会被缩放为 D/T

## 注意事项

1. **线程安全**：所有轨迹操作都在 `target_mutex_` 保护下
2. **起始点一致性**：每次执行轨迹时，起始点都会更新为当前位置，确保从当前位置开始
3. **时间精度**：使用 `rclcpp::Time` 和 `rclcpp::Duration` 保证时间精度
4. **边界情况**：
   - 空轨迹：保持当前位置
   - 单点轨迹（仅目标点）：正常插值（当前位置 -> 目标位置）
   - 时间超出：使用最后一个点
   - 关节不匹配：使用当前位置值并警告
5. **向后兼容**：`setTargetPosition()` 方法仍然可用，内部会转换为轨迹
6. **前缀过滤**：支持关节掩码，可以只控制部分关节

## 测试建议

1. **基本功能**：测试单点、多点轨迹
2. **时间处理**：测试不同时间分布的轨迹
3. **关节匹配**：测试部分关节匹配的情况
4. **边界情况**：测试空轨迹、单点轨迹等
5. **模式切换**：测试从单点模式切换到轨迹模式

