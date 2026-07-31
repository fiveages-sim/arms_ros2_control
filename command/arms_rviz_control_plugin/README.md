# Arms RViz Control Plugin

OCS2 Arm Controller 的 RViz 面板插件。

## 面板

| Panel | 功能 | 主要接口 |
|-------|------|----------|
| **OCS2FSMPanel** | FSM 切换、COMPLIANCE 刚度预设 | 发布 `/fsm_command` |
| **ComplianceForcePanel** | 力控轴 / 设定力 / 实测力 | 订阅 `/compliance_force_status`；写 `compliance_*` 参数 |
| **GripperControlPanel** | 夹爪开合 | — |
| **JointControlPanel** | 关节微调 | 订阅 `/fsm_command` |
| **WbcCapabilityPanel** / **WbcCurrentStatePanel** | 全身 WBC（fullbody） | — |

COMPLIANCE 完整说明：[COMPLIANCE.md](../../controller/ocs2_arm_controller/COMPLIANCE.md)

## 安装

```bash
colcon build --packages-select arms_rviz_control_plugin --symlink-install
source install/setup.bash
```

启动 OCS2 launch 后 RViz 会自动加载面板；手动添加：**Panels → Add Panel → `arms_rviz_control_plugin/…`**

## OCS2FSMPanel

发布 **`/fsm_command`**（`std_msgs/Int32`），并订阅同一话题同步当前状态。

| command | 转换 |
|---------|------|
| 1 | → HOME |
| 2 | → HOLD |
| 3 | → OCS2 |
| 4 | HOME 内切换姿态 |
| 5 | → COMPLIANCE |

可用转换由当前状态决定；不可用的按钮会自动隐藏。

## 依赖

- ROS 2 Jazzy+
- rviz_common
- arms_ros2_control_msgs
- arms_controller_common
