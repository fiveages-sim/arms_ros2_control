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

#### 单臂模式
```bash
ros2 launch arms_target_manager single_arm.launch.py
```

#### 双臂模式
```bash
ros2 launch arms_target_manager dual_arm.launch.py
```

#### 自定义参数
```bash
ros2 launch arms_target_manager arms_target_manager.launch.py \
    topic_prefix:=my_arm \
    dual_arm_mode:=true \
    frame_id:=base_link
```

### 3. 在RViz中查看

1. 启动RViz
2. 添加InteractiveMarkers显示
3. 设置Fixed Frame为配置的frame_id
4. 拖拽marker设置目标pose

## 参数说明

- `topic_prefix`：主题前缀，默认为"arm_controller"
- `dual_arm_mode`：是否为双臂模式，默认为false
- `frame_id`：坐标系ID，默认为"world"

## 发布主题

- `{topic_prefix}_left_pose_target`：左臂目标pose
- `{topic_prefix}_right_pose_target`：右臂目标pose（仅双臂模式）

## 与PoseBasedReferenceManager配合使用

这个包专门设计用于与您的`PoseBasedReferenceManager`配合使用：

1. 启动`ArmsTargetManager`节点
2. 启动您的`PoseBasedReferenceManager`
3. 在RViz中拖拽marker设置目标pose
4. `ArmsTargetManager`发布pose消息
5. `PoseBasedReferenceManager`接收消息并生成轨迹

## 示例配置

### 单臂机器人
```yaml
topic_prefix: "arm_controller"
dual_arm_mode: false
frame_id: "world"
```

### 双臂机器人
```yaml
topic_prefix: "arm_controller"
dual_arm_mode: true
frame_id: "world"
```

## 依赖

- rclcpp
- geometry_msgs
- visualization_msgs
- interactive_markers
