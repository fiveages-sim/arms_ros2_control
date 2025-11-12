# Eyou ROS2 Control Hardware Interface

ROS2 Control硬件接口，用于通过CANopen协议控制Eyou Harmonic电机。

## 功能特性

- 支持同步位置模式（CSP - Cyclic Synchronous Position）
- 独立控制线程，4ms周期实时控制
- 多电机同步控制
- 自动单位转换（弧度 ↔ 编码器计数）
- 支持位置、速度、力矩状态反馈

## 配置参数

### 基本配置

```xml
<ros2_control name="eyou_body" type="system">
  <hardware>
    <plugin>eyou_ros2_control/EyouHardware</plugin>
    <param name="device_type">Canable</param>
    <param name="device_index">0</param>
    <param name="baudrate">1000</param>
    <param name="motor_ids">[5, 6]</param>
    <param name="control_period_ms">4</param>
    <param name="report_period_ms">100</param>
    <param name="use_sync">true</param>
    
    <!-- 减速比配置（推荐使用数组方式） -->
    <param name="reduction_ratios">[50.0, 30.0]</param>
    
    <!-- 或者使用单独配置（向后兼容） -->
    <!-- <param name="motor_5.reduction_ratio">50.0</param> -->
    <!-- <param name="motor_6.reduction_ratio">30.0</param> -->
  </hardware>
  
  <joint name="joint1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  
  <joint name="joint2">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

### 参数说明

| 参数名称 | 类型 | 默认值 | 说明 |
|---------|------|--------|------|
| `device_type` | string | `Canable` | CAN设备类型（Canable/USB2CAN） |
| `device_index` | int | `0` | CAN设备索引 |
| `baudrate` | int | `1000` | CAN波特率（kbps） |
| `motor_ids` | string | - | 电机节点ID列表，格式：`[5, 6]` 或 `5,6` |
| `control_period_ms` | int | `4` | 控制周期（毫秒） |
| `report_period_ms` | int | `100` | 状态上报周期（毫秒） |
| `use_sync` | bool | `true` | 是否使用同步模式 |
| `reduction_ratios` | string | - | 减速比数组，格式：`[50.0, 30.0]` 或 `50.0,30.0`（推荐） |
| `motor_X.reduction_ratio` | double | `1.0` | 电机X的减速比（如果未提供reduction_ratios数组时使用） |

## 依赖

### 系统依赖

编译和运行本包需要安装 `patchelf` 工具，用于设置共享库的 RPATH：

```bash
sudo apt-get install patchelf
```

`patchelf` 用于在安装后自动为 Eyou SDK 的共享库设置 RPATH，确保运行时能正确找到依赖库。

## CAN 接口配置

Eyou SDK 在初始化时会自动配置 CAN 接口（`can0`），这需要 sudo 权限。有两种配置方式：

### 方法一：配置 sudo 免密码（推荐）

如果希望程序能够自动配置 CAN 接口，需要配置 sudo 免密码执行特定的命令：

1. 编辑 sudoers 文件：
```bash
sudo visudo
```

2. 添加以下行（将 `your_username` 替换为你的用户名，例如 `fa`）：
```
your_username ALL=(ALL) NOPASSWD: /sbin/ip link set can0 *
```

3. 保存并退出（在 visudo 中：按 ESC，然后输入 `:wq`）

### 方法二：手动配置 CAN 接口

在启动 ROS2 节点之前，手动配置 CAN 接口：

```bash
# 需要 root 权限
sudo ip link set can0 down  # 先关闭（如果已启动）
sudo ip link set can0 type can bitrate 1000000 sample-point 0.875 restart-ms 0
sudo ip link set can0 up
```

**注意**：如果使用手动配置，SDK 初始化时可能仍会尝试重新配置接口，建议使用方法一。

### 验证 CAN 接口状态

```bash
# 检查接口状态
ip link show can0

# 应该显示 `state UP` 而不是 `state DOWN`
```

## 编译

```bash
cd ~/ros2_ws
colcon build --packages-up-to eyou_ros2_control --symlink-install
```

## 架构设计

- **独立CSP控制线程**：4ms周期发送位置指令和SYNC帧
- **无锁设计**：单写入者模式，无需mutex保护
- **状态读取**：在read()中主动读取本地字典
- **单位转换**：自动处理弧度与编码器计数之间的转换

## 库文件管理

本包采用与rokae_ros2_control相同的方式管理预编译库：

```
eyou_ros2_control/
├── external/
│   └── libeyou/
│       ├── include/
│       │   └── eu_harmonic.h
│       └── lib/
│           └── Linux/
│               ├── x86_64/
│               │   ├── libeu_harmonic.so
│               │   ├── libcontrolcan.so
│               │   ├── libcyhcs_log.so
│               │   ├── libeu_canable.so
│               │   ├── libeu_candrv.so
│               │   └── libusbcanfd.so
│               └── aarch64/  (如果支持ARM架构)
```

库文件已包含在包中，无需额外配置。

编译安装时，CMake 会自动将所有 Eyou SDK 库文件安装到 `install/eyou_ros2_control/lib/` 目录，并使用 `patchelf` 为这些库设置 RPATH（`$ORIGIN`），确保运行时能正确找到同目录下的依赖库。

## 注意事项

1. **编码器分辨率**：
   - 电机轴转一圈 = 65536 脉冲（编码器原始值）
   - 关节轴转一圈 = 65536 × 减速比 脉冲
   - 转换公式：`编码器计数 = 弧度 × 减速比 × 65536 / (2π)`
   - 可在代码中修改 `ENCODER_RESOLUTION`（默认65536）
2. **力矩转换**：使用固定系数0.001（可在代码中修改 `EFFORT_SCALE`）
3. **电机数量**：必须与URDF中定义的关节数量一致
4. **架构支持**：支持x86_64和aarch64架构（自动检测）

