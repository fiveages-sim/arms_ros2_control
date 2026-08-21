# Arms Teleop Package

This package provides joystick and keyboard teleoperation for robotic arms and chassis.

## Features

- **Joystick teleop**: game controllers via `sensor_msgs/Joy` and the `joystick_teleop` node
- **Keyboard teleop**: terminal-based `keyboard_teleop` (stdin); publishes the same command topics as the joystick path except it does not publish `/fsm_command`
- **Configurable** mappings and scaling (joystick YAML; keyboard ROS parameters)

## Building

```bash
cd ~/ros2_ws
colcon build --packages-up-to arms_teleop --symlink-install
```

## Usage

### Joystick teleop

Launch the joystick teleop stack (`game_controller_node` or `joy_node` + `joystick_teleop`):

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch arms_teleop joystick_teleop.launch.py
```

By default `joy_driver:=auto` uses [`game_controller_node`](https://docs.ros.org/en/ros2_packages/jazzy/api/joy/index.html) when `joy_enumerate_devices` reports `Mapped: true`, with [`config/joy_mapping/generic_gamepad.yaml`](config/joy_mapping/generic_gamepad.yaml) (SDL standard layout). Most gamepads (F710, PS4, Xbox, Switch Pro, GameSir, etc.) take this path. Unmapped devices fall back to `joy_node` + `default.yaml`.

Force raw joystick driver:

```bash
ros2 launch arms_teleop joystick_teleop.launch.py joy_driver:=joy
```

Use a different joystick device (default is `/dev/input/js0`):

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch arms_teleop joystick_teleop.launch.py joy_dev:=/dev/input/js1
```

List connected controllers (ID, GUID, GamePad, Mapped, name):

```bash
ros2 run joy joy_enumerate_devices
```

When multiple controllers are connected, pass `joy_dev` only; launch re-enumerates by device ID:

```bash
ros2 launch arms_teleop joystick_teleop.launch.py joy_dev:=/dev/input/js1
```

#### Mapping files (`config/joy_mapping/`)

| File | When used |
|------|-----------|
| `generic_gamepad.yaml` | `game_controller_node` + `config:=auto`（Mapped 手柄） |
| `default.yaml` | `joy_node` + `config:=auto`（未 Mapped 设备） |

`config:=auto`：`Mapped: true` → `generic_gamepad`；否则 `default`。

Override:

```bash
ros2 launch arms_teleop joystick_teleop.launch.py config:=default joy_driver:=joy
```

#### 新设备校准（仅未 Mapped 设备或强制 `joy_driver:=joy`）

1. `ros2 topic echo /joy` 记录按键/摇杆下标。
2. 在 `config/joy_mapping/` 新建或修改 yaml，并用 `config:=<name>` 指定。

**F710 X/D：** 默认 `game_controller_node` 下 X/D 共用 `generic_gamepad.yaml`；若强制 `joy_driver:=joy`，用 `default.yaml` 并需在切换模式后重启 launch。

### Keyboard teleop

Run in a **separate interactive terminal** so stdin is a real TTY (do not rely on embedding this node in a large launch file that steals stdin). Typical workflow: start your stack with `ros2 launch ...` in one terminal; in another:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run arms_teleop keyboard_teleop
```

Do not run `joystick_teleop` and `keyboard_teleop` at the same time if both publish to the same topics (`control_input`, `/cmd_vel`, etc.).

Optional parameters example:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run arms_teleop keyboard_teleop --ros-args -p discrete_key_stale_ms:=350
```

### USB foot-pedal FSM control

`foot_pedal_teleop` reads one Linux evdev keyboard device directly, subscribes to `/fsm_state`,
and publishes `/fsm_command`. Default mapping: F13=HOLD, F14=HOME / HOME pose switch,
F15=OCS2, F16=MOVEJ.

With exactly one supported pedal connected, device discovery is automatic:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run arms_teleop foot_pedal_teleop
```

If multiple CM6K pedals are connected, the node lists their serial numbers and exits instead of
silently choosing the wrong device. Select the intended unit by serial:

```bash
ros2 run arms_teleop foot_pedal_teleop --ros-args \
  -p device_serial:=03CDAB4B081EBD3671FFFFFFFF0300
```

An explicit stable `device` path remains available for diagnostics and overrides auto-discovery.

By default the node exclusively grabs that device (`grab_device:=true`), so pedal events do not
reach the desktop or other applications. F13 always requests HOLD immediately. F14-F16 use clicks
for teleoperation events and long presses for FSM commands:

- F14 single-click: publish `/xr/controller_state=50` (start).
- F14 double-click: publish `/xr/controller_state=51` (end).
- F14 long-press: request HOME; while already HOME, request HOME pose switch (`command=100`).
- F15 long-press: request OCS2.
- F15 single/double-click while in OCS2: toggle left/right-arm control.
- F16 single-click: publish `/xr/controller_state=52` (manual intervention).
- F16 double-click: publish `/xr/controller_state=53` (delete).
- F16 long-press: request MOVEJ.

The node reads the actual arm states from `/ocs2_wbc_controller/current_state` and publishes the
toggle to `mode_command`. Per-arm toggles are rejected while bimanual coupling is enabled, matching
the WBC RViz panel behavior. Like `xr_target_node.py`, the node publishes `std_msgs/Int32` on
`/xr/controller_state` continuously at 30 Hz: an XR event occupies one frame and subsequent frames
contain `0`. Events are available in HOME, HOLD, OCS2, and MOVEJ. Single-click actions are emitted
after the double-click window expires. HOME, OCS2, and MOVEJ long-press commands are
accepted only from HOLD, except the HOME pose switch. F13 cancels pending click actions. Linux
key-repeat events are ignored.

Parameters:

- `device` (default `auto`): automatically find the CM6K interface-00 keyboard, or use an explicit
  stable evdev path.
- `device_serial` (default empty): disambiguate multiple connected CM6K pedals in auto mode.
- `grab_device` (default `true`): exclusively consume this input device.
- `hold_on_disconnect` (default `true`): publish HOLD if the device read fails or disconnects.
- `gesture.double_click_ms` (default `300`): shared F14-F16 double-click window, valid range
  150-1000 ms.
- `gesture.long_press_ms` (default `700`): F14-F16 FSM long-press threshold, valid range
  400-2000 ms.
- `xr.publish_rate_hz` (default `30.0`): `/xr/controller_state` state-stream frequency.
- `mode_command_topic` (default `mode_command`): WBC humanoid-mode command topic.
- `keys.hold/home/ocs2/movej`: Linux input key codes, default F13-F16 (`183`-`186`).

If opening the device fails with `Permission denied`, configure an input-device udev rule or run
the node as a user with permission to read that evdev device. Do not use the ordinary keyboard's
device path with exclusive grabbing enabled.

For the SayoDevice CM6K pedal (VID `8089`, PID `000c`, interface `00`), install the packaged udev
rule once:

```bash
sudo install -m 0644 \
  src/arms_ros2_control/command/arms_teleop/config/udev/70-sayo-cm6k-foot-pedal.rules \
  /etc/udev/rules.d/70-sayo-cm6k-foot-pedal.rules
sudo udevadm control --reload-rules
```

Unplug and reconnect the pedal after reloading. `TAG+="uaccess"` grants the active local login
session read access whenever the matching event device is created, so per-plug `setfacl` commands
are no longer needed. The rule deliberately matches only USB interface `00`, not the auxiliary
`if01` input interface.

### Keyboard parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `update_rate` | 20.0 | Timer publish rate (Hz) |
| `movement_key_stale_ms` | 120 | How long WASD / I K / J L / arrows count as held without a refresh (release inference). Lower = faster stop when you lift movement keys. |
| `discrete_key_stale_ms` | 320 | Same for **discrete** keys (N, M, Space, Enter, +/-). Must be **greater than** the kernel key-repeat delay (~250 ms); otherwise the gap before the first repeat looks like a “release” and **toggle keys fire twice** (once falsely, once on real release). |
| `chassis.linear_scale` | 0.25 | Chassis linear velocity scale |
| `chassis.angular_scale` | 0.5 | Chassis angular velocity scale |
| `arm_axes_activation_threshold` | 0.5 | Same role as in `joystick_teleop` |
| `mirror_movement` | `false` | Stored toggle / ROS param (same name as `joystick_teleop`). ARM kinematics apply **`mir_apply = !mirror_movement`** so WASD maps like the joystick; the **status line shows this flag** (`OFF` when `false`). Toggle with **M** while running. |
| `show_status_line` | `true` | When `true`, refresh one ANSI status line on stdout (`MODE`, speed, mirror, arm/gripper). Set `false` to disable. |

Runtime status uses `\033[2K\r` so the same row updates in place. This node sets its logger severity to **WARN** so routine `[INFO]` lines do not push the status row aside.

### Keyboard key map (summary)

| Keys | Action |
|------|--------|
| WASD | Translation (ARM: `Inputs` x/y; CHASSIS: `cmd_vel` linear) |
| I / K | z (ARM only) |
| J / L | yaw (ARM) or yaw rate (CHASSIS) |
| Arrow keys | roll / pitch (ARM only) |
| N | Toggle ARM / CHASSIS (on key release) |
| M | Toggle mirror (on release; chassis velocities ignore mirror) |
| `-` / `+` or `=` | Speed level 1–10 (×0.1), default level 3 |
| Space | Cycle active arm (ARM only) |
| Enter | Gripper open/close (ARM only) |

Waist lift/turn topics are published as zero from the keyboard node. Keyboard teleop does not publish `/fsm_command`.

## Topics

### Published topics

- `control_input` (`arms_ros2_control_msgs/msg/Inputs`) — high-rate axes; `target` is the active arm (1=left, 2=right)
- `teleop_mode` (`arms_ros2_control_msgs/msg/TeleopMode`) — latched mode snapshot (`Reliable` + `TransientLocal`): `active`, `control_mode`, `mirror_movement`, `speed_level`, `speed_scale`, with `stamp`. Published once at startup and again when those fields change (not on target-arm switch; use `Inputs.target` for that). Joystick starts with `active=false` until right-stick enable; keyboard always `active=true`.
- `/cmd_vel` (`geometry_msgs/msg/Twist`)
- `/body_joint_controller/waist_lifting_command` (`std_msgs/msg/Float64`)
- `/body_joint_controller/waist_turning_command` (`std_msgs/msg/Float64`)

The joystick stack also publishes `/fsm_command` (`std_msgs/msg/Int32`). The keyboard node does not.

`TeleopMode.speed_level` / `speed_scale`: keyboard uses levels 1–10 with `speed_scale = level * 0.1`; joystick uses `0`=LOW / `1`=HIGH with the configured low/high scales.

### Subscribed topics (joystick only)

- `joy` (`sensor_msgs/msg/Joy`)

## Parameters (joystick launch)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `joy_driver` | `auto` | `auto` / `game_controller` / `joy` — input node selection |
| `joy_dev` | `/dev/input/js0` | Joystick device path (`/dev/input/jsN`) |
| `config` | `auto` | Mapping YAML stem under `config/joy_mapping/`, or `auto` |

`quick_start.sh`：仅 1 个手柄时自动启动；多个时选手柄 ID，只传 `joy_dev`。

## Dependencies

- `rclcpp`: ROS 2 C++ client library
- `sensor_msgs`: sensor message types
- `geometry_msgs`, `std_msgs`
- `arms_ros2_control_msgs`: control input message definitions
