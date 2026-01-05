# StateMoveJ 轨迹插值实现方案

## 需求分析

为 `StateMoveJ` 添加 `trajectory_msgs/JointTrajectory` topic 订阅功能，实现：
1. 订阅 `trajectory_msgs/JointTrajectory` 类型的 topic
2. 对轨迹中的多个点进行根据总时长的插值
3. 支持关节名称匹配
4. 线程安全

## 实现方案

### 1. 数据结构设计

需要添加以下成员变量来存储轨迹信息：

```cpp
// 轨迹点结构
struct TrajectoryPoint {
    double time_from_start;  // 从轨迹开始的时间（秒）
    std::vector<double> positions;  // 关节位置
};

// 轨迹相关成员变量
std::vector<TrajectoryPoint> trajectory_points_;  // 存储轨迹点
rclcpp::Time trajectory_start_time_;  // 轨迹开始执行的时间
bool has_trajectory_{false};  // 是否有有效轨迹
bool trajectory_active_{false};  // 轨迹是否正在执行
rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_subscription_;
```

### 2. 实现步骤

#### 步骤 1: 修改头文件 (StateMoveJ.h)

添加必要的包含和成员变量：

```cpp
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// 在 private 部分添加：
struct TrajectoryPoint {
    double time_from_start;
    std::vector<double> positions;
};

std::vector<TrajectoryPoint> trajectory_points_;
rclcpp::Time trajectory_start_time_;
bool has_trajectory_{false};
bool trajectory_active_{false};
rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_subscription_;
```

#### 步骤 2: 实现轨迹订阅回调

在 `setupSubscriptions` 方法中添加轨迹订阅：

```cpp
// 订阅 JointTrajectory topic
trajectory_subscription_ = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
    base_topic + "/trajectory", 10,
    [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        setTrajectory(msg);
    });
```

#### 步骤 3: 实现 setTrajectory 方法

处理接收到的轨迹消息：
- 解析关节名称并匹配到控制接口
- 提取轨迹点（位置和时间）
- 根据总时长重新归一化时间
- 存储轨迹点

#### 步骤 4: 修改 run() 方法

在 `run()` 中添加轨迹插值逻辑：
- 如果 `has_trajectory_` 为 true，使用轨迹插值
- 否则，使用原有的单点插值逻辑
- 根据当前时间计算在轨迹中的位置
- 在相邻轨迹点之间进行线性插值

### 3. 关键实现细节

#### 3.1 关节名称匹配

`JointTrajectory` 消息包含关节名称列表，需要匹配到控制接口中的关节：
- 遍历 `msg->joint_names`
- 在 `joint_names_` 中查找匹配的索引
- 建立映射关系

#### 3.2 时间归一化

`JointTrajectory` 中的时间是相对于轨迹开始的时间。需要：
- 如果消息中有 `time_from_start`，使用该时间
- 如果消息中没有时间信息，根据总点数均匀分配时间
- 根据 `duration_` 参数重新归一化时间

#### 3.3 轨迹插值算法

在 `run()` 中：
1. 计算从轨迹开始到当前的时间差
2. 找到当前时间对应的两个相邻轨迹点
3. 在这两个点之间进行线性插值
4. 应用关节掩码（如果使用前缀过滤）

### 4. 代码结构

```
StateMoveJ::setTrajectory()
  ├── 验证消息有效性
  ├── 匹配关节名称
  ├── 提取轨迹点
  ├── 时间归一化（根据 duration_）
  └── 存储轨迹点

StateMoveJ::run()
  ├── 检查是否有轨迹
  ├── 如果有轨迹：
  │   ├── 计算当前时间在轨迹中的位置
  │   ├── 找到相邻轨迹点
  │   └── 线性插值
  └── 否则：
      └── 使用原有单点插值逻辑
```

### 5. 注意事项

1. **线程安全**: 轨迹数据需要加锁保护
2. **状态管理**: 进入状态时重置轨迹，退出时清理
3. **关节数量匹配**: 确保轨迹中的关节数量与控制接口匹配
4. **时间处理**: 处理 ROS 时间戳和相对时间的转换
5. **边界情况**: 
   - 轨迹为空
   - 轨迹只有一个点
   - 时间超出轨迹范围

### 6. 使用示例

```cpp
// 在控制器中设置订阅
state_movej_->setupSubscriptions(node, "target_joint_position", false);

// 发布轨迹
trajectory_msgs::msg::JointTrajectory traj;
traj.joint_names = {"joint1", "joint2", "joint3"};
traj.points.resize(3);
traj.points[0].positions = {0.0, 0.0, 0.0};
traj.points[0].time_from_start.sec = 0;
traj.points[1].positions = {1.0, 1.0, 1.0};
traj.points[1].time_from_start.sec = 1;
traj.points[2].positions = {2.0, 2.0, 2.0};
traj.points[2].time_from_start.sec = 2;
trajectory_publisher_->publish(traj);
```

## 实现优先级

1. **高优先级**: 基本轨迹订阅和插值功能
2. **中优先级**: 关节名称匹配和验证
3. **低优先级**: 高级特性（如速度/加速度插值）

