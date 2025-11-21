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

通常通过 OCS2 控制器 launch 文件启动（会自动包含 ArmsTargetManager）：

```bash
ros2 launch ocs2_arm_controller full_body.launch.py robot:=cr5
```

或者直接使用 OCS2 专用的 launch 文件：

```bash
ros2 launch arms_target_manager ocs2_arm_target_manager.launch.py \
    task_file:=/path/to/task.info
```

### 3. 在RViz中查看

1. 启动RViz
2. 添加InteractiveMarkers显示
3. 设置Fixed Frame为配置的frame_id
4. 拖拽marker设置目标pose

## 参数说明

### Launch 参数
- `task_file`：task.info 文件路径（必需，用于自动检测 dual_arm_mode 和 control_base_frame）
- `marker_fixed_frame`：Marker 固定坐标系，默认为 "base_link"

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
