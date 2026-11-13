# FSM 与 WBC 模式

这一篇讲两件容易被混在一起的事：**整机现在归谁控制**，和**控制器内部怎么控**。
它们是两套独立的状态，各有自己的话题和代码位置。

## 一、两层状态

| | 第一层：FSM 状态 | 第二层：WBC 模式 |
|---|---|---|
| 回答的问题 | 整机现在归谁控制 | OCS2 内部怎么控 |
| 取值 | HOME / HOLD / MOVEJ / OCS2（4 个） | 一个 9 位整数：身体 / 双臂 / 耦合 / 底盘锁定 / 头部 |
| 存放位置 | 控制器里的当前状态对象 | OCS2 的 `ModeSchedule` |
| HOLD 下能改吗 | 不能（改就是切状态） | **能**（注释明确写了 any FSM state including HOLD） |
| 命令话题 | `fsm_command` | `mode_command` |
| 状态话题 | `fsm_state` | `<controller>/current_state` |
| 包内访问点 | `core/FsmClient.h` | `core/HumanoidModeClient.h` |

不要把两者合并。它们正交：可以在 HOLD 状态下把身体模式改成 `BODY_LOCK`，
也可以在 OCS2 状态下保持底盘锁定。

## 二、话题约定

### 命名

所有相关话题都用**相对名**，由节点命名空间隔离多机器人：

```
/robot_a/fsm_command        /robot_b/fsm_command
/robot_a/fsm_state          /robot_b/fsm_state
/robot_a/mode_command       /robot_b/mode_command
```

控制器侧的 `<controller>/current_state`、`<controller>/wbc_capabilities` 也是相对名，
会落到 `/<controller_manager 命名空间>/<controller_name>/...`。

**不要写绝对名 `/fsm_command`。** 单机器人时看不出问题，双机器人时会互相打架。

如果同一个命名空间里确实要跑两个控制器，可以用参数分开：

| 参数 | 默认值 | 归属 |
|---|---|---|
| `fsm_state_topic` | `fsm_state` | `ocs2_arm_controller` / `ocs2_wbc_controller` / `arms_target_manager_node` |
| `fsm_command_topic` | `fsm_command` | 同上 |
| `wbc_controller_name` | `ocs2_wbc_controller` | `arms_target_manager_node` |
| `mode_command_topic` | `mode_command` | `arms_target_manager_node` |

### 为什么用 `transient_local`

控制器**只在状态变化时**发一帧（`on_activate` 会发一次初始的 HOLD），不做周期发布。

因此订阅端必须用 `rclcpp::QoS(1).reliable().transient_local()`：

```cpp
node->create_subscription<std_msgs::msg::Int32>(
    "fsm_state", rclcpp::QoS(1).reliable().transient_local(), callback);
```

这样无论订阅者什么时候加入，都能立刻拿到最后一帧。**不需要也不建议改成周期发布**：
周期发布会让“收到消息 = 状态刚变化”这个有用信号失效。

如果某个消费者用了 `10`（默认 volatile），它就只能收到加入之后的变化，
在没人切模式的情况下会一直用构造时的默认值。

## 三、FSM 层

### 谁写

只有三类来源：

1. 遥操作设备：VR 组合键 11/12、手柄 LB+A/B/Y/START、RViz 面板按钮；
2. 控制器自己：碰撞检测触发时主动发 `fsm_command=2` 退回 HOLD；
3. 外部脚本。

包内统一走 `FsmClient::request(意图)`，不再直接拼命令号：

```cpp
FsmClient fsm(node);                 // 默认订阅 fsm_state、发布 fsm_command
fsm.request(FsmIntent::Next);        // HOME → HOLD → OCS2
fsm.request(FsmIntent::Back);        // OCS2/MOVEJ → HOLD → HOME
```

意图到命令号的映射（唯一一份，`FsmClient::plan()`）：

| 意图 | 当前 | 命令 | 效果 |
|---|---|---|---|
| Next | HOME | 2 | HOME → HOLD |
| Next | HOLD | 3 | HOLD → OCS2 |
| Next | OCS2 / MOVEJ | — | 不合法，打 WARN 不发 |
| Back | OCS2 / MOVEJ | 2 | → HOLD |
| Back | HOLD | 1 | → HOME |
| Back | HOME | 100 | 切换预设姿态（状态不变） |

### 谁读

消费者**一律订阅 `fsm_state`**，不要再用 `FSMStateTransitionValidator` 推断。
历史上那张规则表和控制器实际行为有 4 处不一致（例如基础控制器把命令 3 当作 MOVEJ，
`StateHome` 收到命令 3/4 也会回 HOLD），推断必然出错。

### 默认状态

`FsmClient` 构造即 **HOLD**，与控制器 `on_activate()` 的初始状态一致，
所以不存在“未知状态”窗口。未接入 `FsmClient` 的模块也按 HOLD 处理。

## 四、WBC 模式层

### 谁写

`mode_command` 是**局部修改**语义的字符串命令 —— 只改指定的那一项，其余保持：

| 类别 | 命令 |
|---|---|
| 身体 | `BODY_FREE` / `BODY_RELATIVE` / `BODY_TRACKING` / `BODY_LOCK` / `BODY_CUSTOM_LOCK` / `BODY_UNLOCK` |
| 头部 | `HEAD_DISABLE` / `HEAD_TRACKING` / `HEAD_GAZE` / `HEAD_FORWARD` |
| 底盘 | `BASE_LOCK` / `BASE_UNLOCK` |
| 双臂 | `ARMS_COUPLED` / `ARMS_INDEPENDENT` |
| 单臂 | `LEFT_ARM_ENABLE` / `LEFT_ARM_DISABLE` / `RIGHT_ARM_ENABLE` / `RIGHT_ARM_DISABLE` |
| 参考关节 | `HOME_JOINT_ON` / `HOME_JOINT_OFF` |
| 整机预设 | `PRESET:HM_DUAL_BODY_TRACKING` 等 |

包内统一走 `HumanoidModeClient` 的意图方法，不再四处拼字符串：

```cpp
HumanoidModeClient mode(node);
mode.setBodyMode(BodyMode::Tracking);
mode.setHeadMode(HeadMode::Gaze);
mode.setBaseLocked(false);
mode.setArmEnabled(/*left=*/true, true);
mode.setBodyAndHeadTracking();       // 全身遥操进入
```

### 谁读

订阅 `<controller>/current_state`（`WbcCurrentState`），**直接用报文里的值**。

三个必须注意的点：

1. **不要用模式位自己推手臂状态。** 控制器要等内部使能等待相位结束才报
   `ARM_ENABLED`，自己推会得到“已使能”的假象。用 `armEnabled(left)`。
2. **`base_state` 可能被控制器强制锁定。** 没有外部世界坐标时，控制器会把
   `baseLocked` 强制置真，用户请求解锁会静默失败。用 `expect()` 登记期望，
   超时会打 WARN。
3. `<controller>/wbc_capabilities` 是**静态能力声明**（有没有移动底盘、腰锁、
   双臂耦合、头部跟踪），不是运行状态，不要混用。

### 能力声明

```cpp
if (mode.capability().has_mobile_base) { ... }
```

未收到时为默认值（全 false），所以依赖能力的功能在启动早期会保守关闭。

## 五、包内的接线

`arms_target_manager_node.cpp` 只创建一次，然后注入给两个使用方：

```cpp
auto fsm  = std::make_shared<FsmClient>(node);
auto mode = std::make_shared<HumanoidModeClient>(node);

target_manager->attachFsmClient(fsm.get());
target_manager->attachHumanoidModeClient(mode.get());
vr_handler->attachFsmClient(fsm.get());
vr_handler->attachHumanoidModeClient(mode.get());
```

因此一个进程里只有：

- 1 个 `fsm_state` 订阅、1 个 `fsm_command` 发布器；
- 1 个 `current_state` 订阅、1 个 `wbc_capabilities` 订阅、1 个 `mode_command` 发布器。

`ArmsTargetManager` 里的 `current_fsm_state_` / `current_controller_state_`
现在只是**兼容缓存**，由 `FsmClient` 的观察者刷新，不再自行推断。

## 六、排查

| 现象 | 看什么 |
|---|---|
| 界面显示的状态和机器人不一致 | `ros2 topic echo /<ns>/fsm_state`；若没有输出说明控制器没激活 |
| 按了按钮没反应 | 日志里找“FSM 请求被忽略”或“WBC 模式未生效” |
| 全身模式开了但机器人不动 | 看 `current_state.body_state` 是否真的变成 `BODY_TRACKING` |
| 双机器人互相干扰 | `ros2 topic info -v` 确认话题是否带命名空间前缀 |
| 后启动的节点状态是默认值 | 检查该订阅的 QoS 是否为 `transient_local` |
