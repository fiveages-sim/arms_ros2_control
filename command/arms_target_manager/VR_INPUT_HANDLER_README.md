# VRInputHandler - VR输入处理器

## 概述

VRInputHandler是基于VRMarkerWrapper功能开发的VR输入处理器，专门为arms_target_manager设计。它实现了VR pose订阅、状态切换机制和marker更新功能。

## 功能特性

### 核心功能
- **VR Pose订阅**: 订阅VR左右臂的末端执行器pose
- **状态切换机制**: 通过left_thumbstick在存储模式和更新模式之间切换
- **频闪优化**: 基于阈值检测pose变化，避免不必要的marker更新
- **差值计算**: 基于存储的base pose计算相对变化并应用到机器人

### 工作模式

#### 存储模式 (Storage Mode)
- 默认模式，不更新marker
- 存储VR和机器人的当前pose作为base pose
- 用于设置参考点

#### 更新模式 (Update Mode)
- 基于存储的base pose计算VR pose的变化
- 将相同的变化应用到机器人base pose
- 更新marker位置

## 订阅的Topic

### VR输入
- `xr_left_ee_pose` (geometry_msgs/msg/PoseStamped): VR左臂末端执行器pose
- `xr_right_ee_pose` (geometry_msgs/msg/PoseStamped): VR右臂末端执行器pose
- `xr_right_thumbstick` (std_msgs/msg/Bool): 右摇杆按下状态

### 机器人状态
- `left_current_pose` (geometry_msgs/msg/PoseStamped): 机器人左臂当前pose
- `right_current_pose` (geometry_msgs/msg/PoseStamped): 机器人右臂当前pose

## 参数配置

### 构造函数参数
- `node`: ROS节点指针
- `targetManager`: ArmsTargetManager指针
- `updateRate`: 更新频率，默认500Hz

### 启动参数
- `enable_vr`: 是否启用VR控制，默认true
- `vr_update_rate`: VR更新频率，默认500.0Hz

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

### 3. 操作流程

1. **启动系统**: 确保VR设备和机器人系统都在运行
2. **存储模式**: 默认处于存储模式，移动VR设备到期望的起始位置
3. **切换到更新模式**: 按下right_thumbstick，系统会存储当前VR和机器人的pose作为base pose
4. **VR控制**: 移动VR设备，机器人marker会跟随VR设备的变化
5. **切换回存储模式**: 再次按下right_thumbstick，可以重新设置base pose

## 技术细节

### 变化检测阈值
- `POSITION_THRESHOLD`: 位置变化阈值，默认1cm
- `ORIENTATION_THRESHOLD`: 方向变化阈值，默认0.005弧度

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

## 调试信息

VRInputHandler会输出详细的调试信息：

- 模式切换信息
- VR pose变化记录（存储模式）
- 机器人pose变化记录（存储模式）
- 计算的pose信息（更新模式）
- Marker更新确认

## 注意事项

1. **坐标系一致性**: 确保VR和机器人使用相同的坐标系
2. **频率匹配**: VR更新频率应与系统处理能力匹配
3. **阈值调整**: 根据实际需求调整变化检测阈值
4. **线程安全**: 所有状态变量都使用原子操作确保线程安全

## 故障排除

### 常见问题

1. **VR控制不响应**
   - 检查VR设备是否连接
   - 确认topic名称是否正确
   - 检查是否处于更新模式

2. **Marker不更新**
   - 确认已按下left_thumbstick切换到更新模式
   - 检查pose变化是否超过阈值
   - 确认ArmsTargetManager正常工作

3. **频闪问题**
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

基于VRMarkerWrapper的功能，适配arms_target_manager的架构，保持了核心的VR控制逻辑，同时简化了与目标管理器的集成。
