# arxlift2s_ros2_control

ROS 2 Control hardware interfaces for **ARX LIFT2S**.

| Subsystem | Plugin | Stack |
|-----------|--------|-------|
| Dual X5 arms | `arxlift2s_ros2_control/ArxX5Hardware` | **Stanford** `arx5-sdk` (`Arx5JointController`) |
| Lift column | `arxlift2s_ros2_control/ArxLiftHardware` | Official `LiftHeadControlLoop` (`libarx_lift_src.so`) |

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
| `ArxX5Hardware` | `can1` / `can3` (param `can_interface`, alias `can_name`) | Instantiate ×2; **full_control only** |
| `ArxLiftHardware` | `can5` | `lift_joint` MIX IFs; `write()` → `setHeight`; `robot_type:=2` |

Does **not** export waist / head / chassis (lift activate still parks chassis).

### ArxX5Hardware (Stanford)

URDF **always** declares MIX `position/velocity/effort/kp/kd`. Lift2S forces **`full_control`**
(pos+vel+effort → `set_joint_cmd`; MIT kp/kd from HI `joint_k/d_gains`). `position` / `pd_control` are not supported here.

| Param | Default | Notes |
|-------|---------|-------|
| `robot_model` | `X5` | |
| `can_interface` | `can0` | LIFT2S: `can1` / `can3`; `can_name` accepted as alias |
| `control_mode` | `full_control` | Other values warned and forced to `full_control` |
| `joint_k_gains` / `joint_d_gains` | `[20×4, 10×2]` / `[3.5×4, 1×2]` | Field-tuned Lift2S; dynamic via ros2 param / rqt |
| `gripper_kp` / `gripper_kd` | `5.0` / `0.2` | Gripper position-only |

### ArxLiftHardware (official lift, full_control MIX)

Exports MIX command IFs so `ocs2_wbc_controller` can claim full_control. SDK path remains height position.

| Param | Default | Notes |
|-------|---------|-------|
| `can_name` | `can5` | |
| `robot_type` | `0` | **`2=LIFTS` for LIFT2S** |
| `control_mode` | `full_control` | Informational; write always uses `setHeight` |
| `lift_kp` | `8.0` | **Required** — SDK default is 0 (no motion) |
| `lift_max_vel` | `0.10` | Ramp per ~400 Hz loop (official) |
| `gravity_compensation_torque` | `-1.8` (LIFTS) | Hold bias; tune like official launch |
| `lift_max_torque` | `15.0` | |
| `max_height_m` | `0.48` | Command clamp; URDF prismatic upper ~0.46 m |
| `height_to_motor` | `41.54` | **Required** — `setHeight`/`getHeight` are motor rad, not meters |

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
