# arxlift2s_ros2_control

ROS 2 Control hardware interfaces for **ARX LIFT2S** (official closed-source SDK).

Designed to migrate as a submodule under
[arms_ros2_control/hardwares](https://github.com/fiveages-sim/arms_ros2_control/tree/main/hardwares)
alongside packages such as `dobot_ros2_control` and `arx_ros2_control`.

> **Note:** This package uses the **LIFT official** `InterfacesThread` /
> `LiftHeadControlLoop` path (`libarx_x5_src.so` / `libarx_lift_src.so`).
> It is **not** the open-source [arx-ros2-control](https://github.com/fiveages-sim/arx-ros2-control)
> (`Arx5JointController`) stack.

## Product boundary

| Layer | Location | Role |
|-------|----------|------|
| Hardware plugins | this package | SystemInterface only; no whole-robot config/xacro |
| Robot description + controller yaml | [`offical/description/arx_lift2s_description`](../../description/arx_lift2s_description) | `hardware:=real` wires the three systems below |
| Controllers | arms_ros2_control (OCS2 / adaptive_gripper / basic_joint) | Unchanged; description supplies joints / params |

## Plugins

| Plugin | Class | Typical CAN | Role |
|--------|-------|-------------|------|
| `arxlift2s_ros2_control/ArxX5Hardware` | single X5 arm | `can1` / `can3` | Instantiate twice for dual arm |
| `arxlift2s_ros2_control/ArxLiftHardware` | lift column | `can5` | One `lift_joint` (height, m) |

Exports: dual-arm joints (+ optional gripper) + one lift joint.  
Does **not** export waist / head / chassis (activate still parks chassis via `setChassisCmd(0,0,0,2)`).

### Parameter contract

**ArxX5Hardware**

| Param | Required | Default | Notes |
|-------|----------|---------|-------|
| `can_name` | yes | — | LIFT2S left `can1`, right `can3` |
| `urdf_path` | yes | — | Official single-arm URDF for `InterfacesThread` (from description: `.../urdf/x5_sdk.urdf`) |
| `end_type` | no | `0` | SDK end-effector type |

Joints: exactly 6 non-gripper arm joints; optional one joint whose name contains `gripper` or `hand`.  
Command: `position`. State: `position`, `velocity`, `effort`.  
Gripper ↔ `setCatch`: ROS **m** (URDF `0～0.044`) ↔ SDK **catch `0～5`** (official: 0–5 ↔ 0–80 mm; see LIFT `03-ROS2-单臂X5-SDK.pdf`). HI scales `cmd_m/0.044*5` on write and `catch/5*0.044` on read. `setCatch` is sent only when a controller writes a finite gripper command (NaN = read-only). Arm joints: **rad** 1:1. Lift (other plugin): **meters** 1:1.

**ArxLiftHardware**

| Param | Required | Default | Notes |
|-------|----------|---------|-------|
| `can_name` | no | `can5` | |
| `robot_type` | no | `0` | `0=LIFT`, `1=X7S`, **`2=LIFTS` (LIFT2S)** |

Joints: exactly one (name typically `lift_joint`). Height unit: **meters**.

## Layout

```text
arxlift2s_ros2_control/
├── include/arxlift2s_ros2_control/
├── src/                              # ArxX5Hardware + ArxLiftHardware
├── third_party/                      # vendored official SDK headers + .so
├── arxlift2s_ros2_control.xml
├── CMakeLists.txt
└── package.xml
```

## SDK (`third_party/`)

Synced from the official LIFT controller packages:

- Arm: `arx_x5_controller/lib/arx_x5_src` (+ `arx_hardware_interface` headers)
- Lift: `arx_lift_controller/lib/arx_lift_src` (+ `arx_hardware_interface` headers)

## Status

Plugins implemented and buildable (`libarxlift2s_ros2_control.so`).  
Wire via description: `ros2 launch arx_lift2s_description ocs2_real.launch.py hardware:=real`.

## Mutual exclusion

Do **not** run on the same CAN while these plugins are active:

- Official nodes: `X5Controller` / `lift_controller`
- Open-source: `arx_ros2_control` (`ArxX5Hardware` / `HardwareArxBody`)

## Arms bring-up checklist

| Item | Requirement |
|------|-------------|
| Systems | Left arm / right arm / lift — three `ros2_control` systems |
| Naming | Arm joints use `left_` / `right_` prefixes (match controller yaml) |
| Gripper HI | `position` command + state; `effort` available for `adaptive_gripper_controller` |
| Lift | Height in **meters** (~0–0.48) |
| Mutex | Same CAN not shared with official nodes or open-source arx HI |

## Build

```bash
# From a ROS2 workspace that contains this package under src/
colcon build --packages-select arxlift2s_ros2_control --symlink-install
```
