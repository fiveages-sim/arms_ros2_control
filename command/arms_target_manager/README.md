# ArmsTargetManager

机械臂目标管理器 - 提供3D交互式marker用于设置机械臂末端执行器的目标pose。

## 功能特点

- **3D交互式marker**：在RViz中显示可拖拽的marker
- **单臂/双臂支持**：支持单臂和双臂模式
- **实时pose发布**：拖拽marker时实时发布pose消息
- **简单易用**：无需复杂配置，开箱即用

## 使用方法

### 1. 编译包

```bash
cd ~/ros2_ws
colcon build --packages-up-to arms_target_manager
```

### 2. 启动节点

通过 OCS2 控制器 launch 文件启动（会自动包含 ArmsTargetManager）：

```bash
ros2 launch ocs2_arm_controller demo.launch.py robot:=cr5
```

ArmsTargetManager 的参数处理逻辑已集成到 `robot_common_launch` 中，会在启动时自动：
- 解析 task.info 文件检测 dual_arm_mode 和 control_base_frame
- 自动检测 hand_controllers（如果启用 gripper）
- 查找并加载配置文件

### 3. 在RViz中查看

1. 启动RViz
2. 添加InteractiveMarkers显示
3. 设置Fixed Frame为配置的frame_id
4. 拖拽marker设置目标pose

## 参数说明

### 节点参数
参数通过 OCS2 控制器 launch 文件自动配置，包括：
- `dual_arm_mode`：是否双臂模式（从 task.info 自动检测）
- `control_base_frame`：控制基坐标系（从 task.info 自动检测）
- `marker_fixed_frame`：Marker 固定坐标系，默认为 "base_link"
- `hand_controllers`：手部/夹爪控制器名称列表（从控制器配置自动检测）

### 配置文件查找逻辑
配置文件会自动按以下优先级查找：
1. **task_file 同目录下的 `target_manager.yaml`**（如果存在）
2. **默认配置文件** `arms_target_manager/config/default.yaml`

### YAML 配置参数
- `linear_scale`：线性控制缩放因子，默认 0.005
- `angular_scale`：角度控制缩放因子，默认 0.05
- `vr_update_rate`：VR 更新频率（Hz），默认 500.0
- `enable_vr`：是否启用 VR 输入处理，默认 true

## 发布主题

- `left_target`：左臂目标pose
- `right_target`：右臂目标pose（仅双臂模式）

## 订阅主题

- `left_current_pose`：左臂当前pose（用于自动更新marker）
- `right_current_pose`：右臂当前pose（仅双臂模式）

## 与PoseBasedReferenceManager配合使用

这个包专门设计用于与您的`PoseBasedReferenceManager`配合使用：

1. 启动`ArmsTargetManager`节点
2. 启动您的`PoseBasedReferenceManager`
3. 在RViz中拖拽marker设置目标pose
4. `ArmsTargetManager`发布pose消息
5. `PoseBasedReferenceManager`接收消息并生成轨迹

## 示例配置

### YAML 配置文件示例 (config/default.yaml)
```yaml
/**:
  ros__parameters:
    # 线性控制缩放因子（用于手柄/键盘控制）
    linear_scale: 0.005
    
    # 角度控制缩放因子（用于手柄/键盘控制）
    angular_scale: 0.05
    
    # VR更新频率（Hz）
    vr_update_rate: 500.0
    
    # 是否启用VR输入处理
    enable_vr: true
```

注意：`dual_arm_mode` 和 `control_base_frame` 会自动从 `task_file` 中解析，不需要在 YAML 中配置。

## 依赖

- rclcpp
- geometry_msgs
- visualization_msgs
- interactive_markers
