# Arms ROS2 Control

本仓库包含用于机械臂和机器人手臂的ros2-control文件。它为ROS2环境中的各种机器人机械臂提供控制器和硬件接口。

## 目录

- [项目结构](#项目结构)
- [依赖项](#依赖项)
- [测试环境](#测试环境)
- [快速开始](#快速开始)
- [组件](#组件)
- [配置](#配置)
- [腰部电流环与两层重力补偿](#腰部电流环与两层重力补偿)
- [开发](#开发)
- [故障排除](#故障排除)
- [许可证](#许可证)

## 项目结构

项目组织如下：

```
arms_ros2_control/
├── controller/                    # 控制器实现
│   ├── ocs2_arm_controller/      # 基于OCS2的机械臂控制器
│   └── adaptive_gripper_controller/ # 自适应夹爪控制器
├── hardwares/                    # 硬件接口实现
│   ├── topic_based_ros2_control/ # 基于话题的硬件接口
│   └── unitree_ros2_control/    # Unitree机器人硬件接口
├── command/                      # 命令输入实现
│   ├── arms_ros2_control_msgs/  # 控制输入消息定义
│   ├── arms_rviz_control_plugin/ # RViz控制插件
│   ├── arms_target_manager/     # 目标管理系统
│   └── arms_teleop/             # 统一遥操作包
│       └── joystick_teleop      # 基于手柄的控制
└── README.md
```

## 依赖项

本包依赖于：

- [`robot_descriptions`](https://github.com/fiveages-sim/robot_descriptions) - 机器人描述文件（URDF、XACRO）
- [`ocs2_ros2`](https://github.com/legubiao/ocs2_ros2) - OCS2 ROS2集成（`ocs2_arm_controller`需要）

**包放置位置**：`robot_descriptions`和`ocs2_ros2`都应该放在ROS2工作空间的`src`目录中，与`arms_ros2_control`并列：

```
ros2_ws/
├── src/
│   ├── robot_descriptions/        # 机器人描述文件
│   ├── ocs2_ros2/                # OCS2 ROS2集成
│   └── arms_ros2_control/        # 本包
├── install/
└── log/
```

## 测试环境

本包已在以下ROS2发行版中测试并验证可用：

- **ROS2 Jazzy** (Ubuntu 24.04)



## 快速开始

<details>
<summary><strong>📥 Git克隆项目及依赖项（点击展开）</strong></summary>

要获取此项目及其依赖项，请在ROS2工作空间中克隆以下仓库：

```bash
# 导航到ROS2工作空间
cd ~/ros2_ws/src

git clone https://github.com/fiveages-sim/arms_ros2_control
git clone https://github.com/fiveages-sim/robot_descriptions
git clone https://github.com/legubiao/ocs2_ros2

# 初始化robot_descriptions中必需的子模块
cd robot_descriptions
git submodule update --init common manipulator/Dobot

# 使用rosdep安装依赖项
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

**注意**：
- `rosdep install` 命令将自动安装所有必需的系统依赖项

</details>

<details>
<summary><strong>🔧 验证OCS2设置（可选）</strong></summary>

如果您是第一次接触OCS2，请先验证OCS2环境是否正确配置。通过运行移动机械臂演示之一来验证：

```bash
# 初始化ocs2 robotic assets子模块
cd ~/ros2_ws/src/ocs2_ros2
git submodule update --init submodules/ocs2_robotic_assets

# 构建移动机械臂包
cd ~/ros2_ws
colcon build --packages-up-to ocs2_mobile_manipulator_ros --symlink-install

# 尝试可用的演示之一：
source ~/ros2_ws/install/setup.bash

# Franka Panda
ros2 launch ocs2_mobile_manipulator_ros franka.launch.py

# 或 Mabi-Mobile
ros2 launch ocs2_mobile_manipulator_ros manipulator_mabi_mobile.launch.py
```

如果这些演示中的任何一个成功运行，则您的OCS2环境配置正确。有关更多可用演示，请参阅[ocs2_mobile_manipulator_ros README](https://github.com/legubiao/ocs2_ros2/tree/ros2/basic%20examples/ocs2_mobile_manipulator_ros)。

![ocs2_franka](.images/ocs2_franka.png)

</details>

### 1. 构建包

```bash
cd ~/ros2_ws
colcon build --packages-up-to ocs2_arm_controller cr5_description arms_teleop adaptive_gripper_controller --symlink-install
```

### 2. 使用模拟硬件启动

* OCS2机械臂控制器
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch ocs2_arm_controller demo.launch.py type:=AG2F90-C
  ```

> **交互式控制：**
> - 按按钮在OCS2控制器FSM之间切换并切换夹爪。
> - 在RViz中，拖拽交互式标记设置目标位置，然后右键单击发送轨迹命令
>
> ![ocs2_dobot](.images/ocs2%20dobot.png)

### 3. 使用Gazebo仿真启动

* 安装Gazebo Harmonic
    ```bash
    sudo apt-get install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control
    ```

#### 启动控制器：
这里使用了Agibot G1作为其他机器人的示例。
* 编译机器人描述
  ```bash
  cd ~/ros2_ws
  colcon build --packages-up-to agibot_g1_description --symlink-install
  ```
* 你可以使用`world`来选择gazebo世界
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch ocs2_arm_controller demo.launch.py robot:=agibot_g1 hardware:=gz world:=warehouse
  ```
  ![ocs2_dobot_gazebo](.images/ocs2%20agibot.png)

### 4. 使用Isaac Sim仿真启动

* 编译增强的topic_based_ros2_control包
  ```bash
  cd ~/ros2_ws
  colcon build --packages-up-to topic_based_ros2_control --symlink-install
  ```
* 启动控制器（在此步骤之前启动Isaac Sim）
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch ocs2_arm_controller demo.launch.py hardware:=isaac type:=AG2F90-C
  ```
  ![ocs2_isaac](.images/ocs2%20isaac.png)

## 组件

### 控制器

#### OCS2机械臂控制器

`ocs2_arm_controller`使用OCS2框架为机械臂提供基于MPC的控制。

**特性：**

- 用于轨迹跟踪的模型预测控制（MPC）
- 实时优化
- 支持各种机器人配置

#### 自适应夹爪控制器

`adaptive_gripper_controller`提供基础夹爪控制功能，具有位置读取和输出能力。

**特性：**

- 从硬件接口读取位置信息
- 向硬件接口输出位置命令
- 基础夹爪位置控制
- 易于理解和扩展

### 硬件接口

#### 基于话题的硬件接口

`topic_based_ros2_control`包提供通过ROS2话题通信的通用硬件接口。原始版本可在[topic based ros2 control](https://github.com/PickNikRobotics/topic_based_ros2_control)找到。

**特性：**

- 适用于任何硬件的通用接口
- 基于话题的通信
- 易于与自定义硬件集成

#### Unitree硬件接口

`unitree_ros2_control`包基于unitree_sdk2为Unitree机器人提供硬件接口。

**特性：**

- 支持Unitree G1和其他sdk2兼容机器人
- Mujoco仿真集成
- 真实机器人支持

### 命令输入系统

#### Arms RViz控制插件

`arms_rviz_control_plugin`提供智能RViz插件，结合了OCS2机械臂控制器和自适应夹爪控制器的控制功能。

**特性：**

- **OCS2机械臂控制器控制：**
  - 智能FSM状态显示和切换
  - 动态按钮控制，只显示可用的状态转换
  - 遵循OCS2机械臂控制器FSM规则的正确状态转换
  - HOLD初始状态和正确的状态流程
  - HOME状态下的姿态切换功能
- **夹爪控制器控制：**
  - 左右夹爪控制按钮
  - 实时夹爪状态显示
  - 单臂/双臂夹爪支持
  - 自动控制器检测和UI适配

#### Arms目标管理器

`arms_target_manager`为设置机械臂末端执行器目标姿态提供3D交互式标记。

**特性：**

- RViz中的3D交互式标记
- 单臂/双臂支持
- VR遥操作
- 手柄遥操作

#### Arms遥操作

`arms_teleop`包为机械臂提供统一的遥操作功能。

**特性：**

- 基于手柄的控制
- 基于键盘的控制
- 不同输入方法的统一接口

## 配置

### 机器人配置

机器人特定配置存储在`robot_descriptions`包中。每个机器人都有自己的描述包，包含：

- URDF/XACRO文件
- 配置文件
- 网格文件

### 控制器配置

控制器配置存储在相应的控制器包中：

- `ocs2_arm_controller/config/` - OCS2控制器配置
- 硬件特定配置在硬件接口包中

## 腰部电流环与两层重力补偿

### 为什么需要它（2026-10-13 真机事故）

手臂 `M6_CCS` 报错 → `controller_manager` 把 M6_CCS **和** `juxie_body` 一起停掉 →
`ocs2_wbc_controller` 需要 claim 全部 20 个关节，**激活失败** →
腰部 1/2/3 已经在电流环里，但**没有任何人在补重力** → `body_joint2` 下沉 8.1°。

根因：重力补偿只存在于控制器里，而控制器**可以整个消失**。

### 架构：两层，取其一不叠加

| 层级 | 实现位置 | 输入 | 依赖 | 精度 |
| --- | --- | --- | --- | --- |
| ① **保命层** | `juxie_ros2_control` 驱动内部 | 实测 `(q1,q2,q3)` 查 13³ 表 | **无**（只需文件 + CAN 反馈） | ≤ 0.86°(手臂姿态) + 0.86°(忽略 q4) + 0.32°(插值) |
| ② **精度层** | 全身 = `ocs2_wbc_controller`、分体 = `ocs2_arm_controller` 代发同一话题 | 真实 20 关节 RNEA | 控制器存活 | ≈ 0 |

- 驱动**优先用话题值**；话题超过 `gravity_ff_topic_timeout_s` 没更新就**退回查表**。
- **取其一，不叠加** —— 叠加会重复补偿。
- **精度层零配置**：两个控制器共用的 `arms_controller_common::WaistGravityFeedforward` 从 xacro
  产物 `/robot_description` 里自己派生关节/电机名单（读 `current_motor_ids`）、话题
  （`gravity_ff_topic`）与 τ→电流系数（`gravity_ff_ma_per_nm`），并用同一份 URDF 建模型
  （WBC 用自己的全机模型，分体用 `robot_description` 的全机模型）⇒ **launch / profile 都不用写**，
  仿真里没有 juxie 段则自动关闭。
- ⚠️ **不能用 `/joint_states` 当保命层**：`joint_state_broadcaster` 要 claim 20 个关节，
  M6_CCS 一挂它自己也挂。

### 接口：上层只写位置 + 速度，电流全在驱动内部

上层（OCS2/WBC）**照旧只写两个普通命令口**，与其它关节完全同构，**没有 effort / 电流口**：

| 命令口 | 值 |
| --- | --- |
| `<joint>/position` | 目标 $q_d$ |
| `<joint>/velocity` | 目标 $\dot q_d$ |

电流是**驱动内部**算出来的：驱动拿 $(q_d, \dot q_d)$ 和实测 $(q, \dot q)$ 按下面的阻抗律算出 $I$，
再用 0x05 帧把电流下发给电机。**上层不碰电流。**

⚠️ 不要给这些关节导出 `effort` 命令口：`CtrlInterfaces` 的 MIX 自动检测会把整机切到
`position + velocity + effort + kp + kd` 分支，反而改变上层行为。

唯一的差别只是驱动**怎么消费** `velocity` 口：CSV 关节直接转发给电机速度环；
电流环关节不转发，而是代进阻抗律当速度参考。

### 阻抗律与下发节奏（驱动内的 2 ms 控制线程）

> 电流换算跑在驱动自己的控制线程 `JxHardware::controlLoop()` 里，周期 = `control_period_ms`
> （xacro 里配 **2 ms = 500 Hz**），**与 CM 的 `update_rate` 无关**。
> 不在 `write()` 里算的原因：CM 的 `read()/write()` 用 `pthread_mutex_trylock` 抢 RM 的锁，
> 别的组件做生命周期切换时会**整拍跳过** `write()`（实测 M6_CCS 激活占 1.78 s）——
> 电流下发不能停，所以它必须有自己的固定节拍；`write()` 只做校验 + 更新快照。
> ⚠️ 下面的增益全部按 **2 ms** 标定 ⇒ 改 `control_period_ms` 必须重新标定。

```
I = Kp·(q_d − q) + Kd·(q̇_d − q̇) + g_ff + g_auto + f_off
  + (|v_ff| < v_eps ? 0 : sign(v_ff)·min(f_c + f_v·|v_ff|, ff_max))
```

- `Kp/Kd/f_*` 单位都含 mA，**不需要 Kt**（mA 域直接标定）。
- `friction_ff_velocity: reference` ⇒ `|q̇_d| ≥ v_eps` 时前馈符号取**参考速度**（换向抢先一步）。
- 输出按 `max_current_ma` 限幅，`current_slew_rate_ma_s` 变化率限制。

### 关键配置

数值默认值的**唯一来源** = 描述包里的 `config/ros2_control/juxie_joint_impedance.yaml`
（`xacro/ros2_control/robot.xacro` 用 `xacro.load_yaml` **直接读**它 ⇒ launch / Python 不参与）；
`config/robot.local.yaml` 的 `hardware:` 段只写**这台机器的接法与开关**，需要改某项时写同名键即覆盖默认值。

| 参数 | 作用 |
| --- | --- |
| `body_current_motor_ids` | 哪些电机进电流环（1~4 = 腰）；也是精度层名单的来源 |
| `impedance_kp_ma_rad` / `impedance_kd_ma_s_rad` | 阻抗增益（顺序 = `motor_ids`） |
| `max_current_ma` | 全局电流限幅 |
| `gravity_ff_table` | ① 保命层：离线生成的 13³ 重力表 |
| `gravity_ff_ma_per_nm` | ② 精度层的 τ→电流系数（与查表标定同源，驱动不读，只随 URDF 传给控制器） |
| `gravity_ff_topic` / `gravity_ff_topic_timeout_s` | ② 精度层：话题名与新鲜度阈值 |
| `feedback_timeout_cycles` | 反馈丢失多少拍后锁存（见下） |

⚠️ `body_current_motor_ids` 必须与 `control.patch.ocs2_wbc_controller.torque_command_joints`
**同名同名单**，否则那些关节的 `position` 口会被改写成实测位置 ⇒ `Kp(q_d−q) ≡ 0`，弹簧项死掉。
（不一致时会在 configure 阶段直接报错，不会静默失败。）

生成重力表：

```bash
python3 scripts/identify_juxie_friction_gravity.py table   # 自动找 planning URDF，输出 config/waist_gravity_ff_table.txt
```

### 反馈丢失保护（逐电机）

`readMotorStates()` 每拍对每个电机计数：收到正常帧清零，**没回帧**或**电机自称未使能**则累加。
连续超过 `feedback_timeout_cycles` 就把该电机**锁存为 CSP 保持最后一帧位置**，并忽略上层命令；
全部电机都被锁存时 `read()` 返回 `ERROR`，由 `controller_manager` 停掉硬件，
`on_deactivate()` 会把所有电机切回 CSP 再关 CAN。

三条硬规则：

1. ⚠️ **绝不对重力关节下发 0 电流** —— 那是自由落体，比"慢慢下沉"危险得多。
   所以非有限命令路径**刻意不清零** `raw_current_commands_`，只上报错误。
2. 一旦不信任某条反馈通道就**不自动恢复**，让模块自己抱住，直到重启驱动。
3. `feedback_timeout_cycles` 不能设 1~2：反馈是模块收到控制帧后才回的，与 CM 的 `read()` 天然异步，
   单拍缺失是正常现象。只有"**使能且无错**"的帧才把计数清零（异常帧持续累计，否则永远到不了阈值）。

锁存会把 `motor_control_modes_[i]` 改成 CSP（运行模式），所以 `on_deactivate()` **不做任何模式判断**：

1. 先 `join()` 控制线程，然后**无条件**把全部电机切回 CSP（target = 当前位置）并 `resetPositionTarget()`；
2. 发 **3 帧**（模式切换是异步的，实测滞后 2~3 帧），每帧间隔 10 ms，并**检查返回值** ——
   3 帧全提交失败就返回 `ERROR`，不宣称“已进入位置保持”；
3. 最后恢复 `motor_control_modes_`（已被锁存的保持 CSP，其余还原）。

⚠️ 为什么不能“只给配了 CSV/电流环的电机切”：锁存会把模式改成 CSP，用模式去判断就会**漏切**，
而模块其实还停在电流环上执行最后一条电流命令（2026-10-13 事故的形态）。
对本来就是 CSP 的配置，多切一次是零成本空操作。

### 首次上电验收标准

整套两层重力 + 阻抗栈**尚未在真机上跑过**。第一次上电时：

- ✅ **腰部应当完全不动**（`on_activate` 用实测位置初始化快照 ⇒ 首帧 `q_d == q` ⇒ 弹簧项为 0，重力全部前馈）。
- ❌ 如果腰部朝**一个方向漂 8~15° 后停住** ⇒ `ma` 符号反了 ⇒ 取反
  `juxie_joint_impedance.yaml` 里的 `gravity_ff_ma_per_nm`（精度层）与那张查表的标定（保命层），
  并重新生成重力表。

## 开发

### 添加新机器人

1. 将机器人描述添加到`robot_descriptions/manipulator/`
2. 在相应的控制器包中创建配置文件
3. 更新启动文件以包含新机器人
4. 在转移到真实机器人之前，使用模拟和仿真器硬件进行测试

### 添加新控制器

1. 在`controller/`目录中创建新包
2. 实现控制器接口
3. 添加配置和启动文件
4. 使用使用说明更新此README

## 故障排除

### 常见问题

1. **构建错误**：确保所有依赖项都已安装和构建
2. **启动错误**：检查机器人描述是否正确安装
3. **硬件连接**：验证硬件接口配置

### 获取帮助

- 查看各个包的README文件以获取具体说明
- 查看`ocs2_ros2`文档以了解OCS2特定问题
- 查看`robot_descriptions`包以了解机器人特定配置

## 许可证

本包根据Apache License 2.0许可。详情请参阅[LICENSE](LICENSE)文件。 
