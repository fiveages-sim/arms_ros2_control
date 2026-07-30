# arxlift2s_ros2_control

ROS 2 Control hardware interfaces for **ARX LIFT2S**.

| Subsystem | Plugin | Stack |
|-----------|--------|-------|
| Dual X5 arms | `arxlift2s_ros2_control/ArxX5Hardware` | **Stanford** `arx5-sdk` (`Arx5JointController`) |
| Lift column | `arxlift2s_ros2_control/ArxLiftHardware` | Official `.so` + **soft_p** / **hybrid** (`setHeight` or `sendLiftHybrid`) |

Official **InterfacesThread** arm path (`libarx_x5_src`) is **retired** from this package.
Layout follows [ht-ros2-control](https://github.com/fiveages-sim/ht-ros2-control) (`external/` SDK)
and [modbus-ros2-control](https://github.com/fiveages-sim/modbus-ros2-control) (multi-plugin XML).

## Product boundary

| Layer | Role |
|-------|------|
| This package | SystemInterface plugins + vendored/linked SDKs only |
| `arx_lift2s_description` | URDF / xacro wiring / controller yaml |
| Controllers | OCS2 / adaptive_gripper / basic_joint |

## Plugins

| Plugin | CAN | Notes |
|--------|-----|-------|
| `ArxX5Hardware` | `can1` / `can3` (param `can_interface`, alias `can_name`) | Instantiate ×2; `full_control` \| `position` (live) |
| `ArxLiftHardware` | `can5` | soft_p / hybrid; live `arx_lift.motor_mode` + gains |

Does **not** export waist / head / chassis (lift activate still parks chassis).

### ArxX5Hardware (Stanford; panthera-ht-aligned)

URDF **always** declares MIX `position/velocity/effort/kp/kd`. `control_mode` only
changes `write()` (same contract as [panthera-ht](https://github.com/fiveages-sim/open-deploy-ws/tree/panthera-ht) / HT):

| Mode | `set_joint_cmd` | MIT kp/kd |
|------|-----------------|-----------|
| `full_control` | pos + vel + effort (OCS2 MIX) | HI `joint_k/d_gains` |
| `position` | pos only (vel=0, torque=0 by default; 可选 position_forward_effort=true 保留 effort 前馈) | HI `joint_k/d_gains` |

`control_mode` 与增益都在**臂 HI 节点**上调（与 `control_mode` 同一套 rqt / `ros2 param`）：

```bash
# Nodes: /arx_lift2s_left_system  /arx_lift2s_right_system
ros2 param set /arx_lift2s_left_system control_mode position
ros2 param set /arx_lift2s_left_system joint_k_gains "[80.0, 70.0, 70.0, 30.0, 30.0, 20.0]"
ros2 param set /arx_lift2s_left_system joint_d_gains "[2.0, 2.0, 2.0, 1.0, 1.0, 0.7]"

# Periodic status log (default off)
ros2 param set /arx_lift2s_left_system status_debug true
ros2 param set /arx_lift2s_right_system status_debug true
```

启动默认：改 xacro `joint_k_gains` / `control_mode`（或 launch `xacro_control_mode:=...`）后重启。

| Param | Default | Notes |
|-------|---------|-------|
| `robot_model` | `X5` | |
| `can_interface` | `can0` | LIFT2S: `can1` / `can3`; `can_name` accepted as alias |
| `control_mode` | `full_control` | Live on arm HI node |
| `joint_k_gains` / `joint_d_gains` | mode-dependent xacro | Live on arm HI node |
| `gripper_kp` / `gripper_kd` | `5.0` / `0.2` | Gripper position-only |
| `joint_k_gains_full_control` / `joint_d_gains_full_control` | mode dependent | full_control 时使用的 MIT kp/kd（control_mode 切换会自动应用） |
| `joint_k_gains_position` / `joint_d_gains_position` | mode dependent | position 时使用的 MIT kp/kd（control_mode 切换会自动应用） |
| `joint_k_gains` / `joint_d_gains` | backward-compatible | 旧参数名：只覆盖“当前 control_mode”的那套 gains |
| `position_forward_effort` | `false` | `position` 下是否把 effort 转成 torque |
| `status_debug` | `false` | 2 Hz write() status + gain log; live on arm HI node |

**Default MIT gains (xacro):**

| Mode | `joint_k_gains` | `joint_d_gains` |
|------|-----------------|-----------------|
| `full_control` | `[20, 20, 20, 20, 10, 10]` | `[3.5, 3.5, 3.5, 3.5, 1.0, 1.0]` |
| `position` | `[80, 70, 70, 30, 30, 20]` | `[2.0, 2.0, 2.0, 1.0, 1.0, 0.7]` (= arx-ros2-control) |

### ArxLiftHardware (soft_p/position | hybrid)

Background thread (~400 Hz). Calibrate / park always uses Soft-P `loop()`.
After activate, mode selects the drive path:

| Mode | Drive | Gains |
|------|-------|-------|
| `soft_p` / `position` | `setHeight` + `loop()`；**只用 position**（忽略 vel/effort） | `arx_lift.soft_p_kp` |
| `hybrid` | `sendLiftHybrid`；position 斜坡 + **仅 HI 重力**（**忽略上层 effort**） | `arx_lift.hybrid_kp` / `hybrid_kd` (kd pack ≤5) |

分体/全身 quick_start 均可选；xacro 默认 **`hybrid`**。导出 MIX IF 供 `ocs2_wbc` claim；
soft_p/hybrid 下上层写的 vel/effort 均不下发电机。

| Param | Default | Notes |
|-------|---------|-------|
| `can_name` | `can5` | |
| `robot_type` | `2` | **`2=LIFTS` for LIFT2S** |
| `lift_motor_mode` | `hybrid` | `soft_p`/`position` \| `hybrid`; live: `arx_lift.motor_mode` |
| `soft_p_kp` | `8.0` | Aliases: `lift_kp`, `kp`; live: `arx_lift.soft_p_kp` |
| `lift_max_vel` | `0.20` | Soft-P SDK ramp limit |
| `hybrid_kp` | `5.0` | Live: `arx_lift.hybrid_kp` |
| `hybrid_kd` | `2.0` | Live: `arx_lift.hybrid_kd` (pack clamp ≤5) |
| `gravity_compensation_torque` | `-1.8` | Hybrid τ_ff **only**; live: `arx_lift.gravity_compensation_torque` |
| `lift_max_torque` | `15.0` | τ clamp |
| `cmd_ramp_vel` | `0.12` (Lift2S xacro; code default `0.04`) | Command ramp [m/s] |
| `max_height_m` / `height_span_m` | `0.48` | Command clamp |
| `height_rad_per_meter` | `41.54` | meters ↔ SDK motor rad (`height_to_motor` alias) |
| `sdk_max_rad` | `20.0` | |
| `status_debug` | `false` | 2 Hz write() status log; live on lift HI node |

```bash
# Switch mode / hot-tune (node: ros2 param list | grep arx_lift)
ros2 param set /controller_manager arx_lift.motor_mode soft_p
ros2 param set /controller_manager arx_lift.soft_p_kp 10.0
ros2 param set /controller_manager arx_lift.motor_mode hybrid
ros2 param set /controller_manager arx_lift.hybrid_kp 50.0
ros2 param set /controller_manager arx_lift.hybrid_kd 1.0
ros2 param set /controller_manager arx_lift.gravity_compensation_torque -1.8

# Periodic status log (default off; 2 s throttle when on)
ros2 param set /arx_lift2s_lift_system status_debug true
```

## Layout

```text
arxlift2s_ros2_control/
├── include/arxlift2s_ros2_control/
├── src/                    # ArxX5Hardware (Stanford) + ArxLiftHardware
├── external/
│   ├── arx5-sdk/           # symlink or copy of Stanford SDK (required)
│   └── SOEM/lib/<arch>/    # prebuilt libsoem.so (SOEM v1.4.0)
├── third_party/arx_lift_src/   # official lift .so + headers
├── arxlift2s_ros2_control.xml
├── CMakeLists.txt
└── package.xml
```

`third_party/arx_x5_src` is unused (legacy); safe to delete from checkouts later.

## External setup

```bash
# From this package directory
mkdir -p external
# Stanford SDK (example: workspace sibling)
ln -sfn ../../../../arx-ros2-control/external/arx5-sdk external/arx5-sdk
# SOEM: vendored prebuilt only (external/SOEM/lib/x86_64/libsoem.so)
```

No conda / no SOEM source tree required.

## Mutual exclusion

Do **not** share the same CAN with:

- Official nodes: `X5Controller` / `lift_controller`
- A second Stanford instance on the same interface (`arx_ros2_control` on can1/can3)

## Build

```bash
colcon build --packages-select arxlift2s_ros2_control --symlink-install
```
