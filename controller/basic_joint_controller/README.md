# Basic Joint Controller

A joint position controller with a three-state FSM (Home / Hold / MoveJ), optional dexterous hand switch/percent control, and optional waist lifting/turning control.

## 1. Building

```bash
cd ~/ros2_ws
colcon build --packages-up-to basic_joint_controller --symlink-install
```

---

## 2. Features

- **StateHome**: Smoothly interpolates to one of up to 10 preset joint configurations
- **StateHold**: Holds all joints at their current positions
- **StateMoveJ**: Accepts target joint positions and interpolates smoothly; supports joint trajectories and prefix-based partial control
- **Dexterous hand control** (`target_command_enabled`): Switch a hand open/close or move it to an arbitrary proportion between the two poses
- **Waist lifting/turning control** (`waist_lifting_enabled`): Velocity-based waist lifting and turning

---

## 3. Configuration Parameters

```yaml
my_controller:
  ros__parameters:
    update_rate: 1000           # Controller update rate (Hz)
    joints: ["j1", "j2", ...]   # Controlled joint names
    command_interfaces: ["position"]
    state_interfaces:  ["position", "velocity"]

    # --- Home configurations (up to 10) ---
    home_1: [0.0, 0.0, ...]     # Required
    home_2: [0.5, 0.5, ...]     # Optional
    home_3: [-0.5, -0.5, ...]   # Optional
    home_duration: 3.0
    home_interpolation_type: "tanh"   # "tanh" | "linear"
    home_tanh_scale: 3.0
    switch_command_base: 100    # Base FSM command for config switching

    # --- MoveJ ---
    movej_duration: 3.0
    movej_interpolation_type: "tanh"  # "tanh" | "linear"
    movej_tanh_scale: 3.0
    movej_trajectory_duration: 3.0
    movej_trajectory_blend_ratio: 0.0

    # --- Dexterous hand / end-effector switch & percent control ---
    # Requires target_command_enabled: true
    # Uses StateHome configurations as open/close poses
    target_command_enabled: false
    target_command_close_config: 1  # Index of "close" pose in home configs (0-based)
    target_command_open_config: 0   # Index of "open"  pose in home configs (0-based)

    # --- Waist lifting ---
    waist_lifting_enabled: false
    waist_lifting_type: "three_joint"   # "three_joint" | "single_joint"
    waist_lifting_duration: 3.0
    waist_lifting_default_parameter: [0.25, 1.0, 5.0]  # [max_speed, accel, decel]
    waist_turning_default_parameter: [0.25, 1.0, 5.0]  # [max_speed, accel, decel]
    # (three_joint only)
    waist_l1: 0.322
    waist_l2: 0.355
    waist_rotation_direction: [1.0, 1.0, 1.0]
    waist_angle_offset: [0.0, 0.0, 0.0]
```

---

## 4. FSM State Transitions

State transitions are triggered by publishing to `/fsm_command` (`std_msgs/Int32`):

| Value | Action |
|---|---|
| `1` | Switch to **HOME** |
| `2` | Switch to **HOLD** |
| `3` | Switch to **MOVEJ** |
| `switch_command_base` (default `100`) | (HOME) Cycle to next configuration |
| `switch_command_base + 1` (default `101`) | (HOME) Switch to configuration 0 |
| `switch_command_base + 2` (default `102`) | (HOME) Switch to configuration 1 |
| … | … |

```bash
ros2 topic pub --once /fsm_command std_msgs/msg/Int32 "data: 1"   # → HOME
ros2 topic pub --once /fsm_command std_msgs/msg/Int32 "data: 3"   # → MOVEJ
```

---

## 5. Topics

All topics below are namespaced to the controller name (e.g. `/left_hand_controller/...`).

### 5.1 MoveJ: target joint position

**Topic:** `/{controller_name}/target_joint_position`  
**Type:** `std_msgs/Float64MultiArray`  
**Active state:** MOVEJ

```bash
ros2 topic pub --once /my_controller/target_joint_position \
  std_msgs/msg/Float64MultiArray "{data: [0.1, 0.2, 0.3]}"
```

### 5.2 MoveJ: joint trajectory

**Topic:** `/{controller_name}/target_joint_trajectory`  
**Type:** `trajectory_msgs/JointTrajectory`  
**Active state:** MOVEJ

Multi-waypoint trajectory; waypoints are blended according to `movej_trajectory_blend_ratio`.

### 5.3 Dexterous hand — switch control

**Topic:** `/{controller_name}/target_command`  
**Type:** `std_msgs/Int32`  
**Requires:** `target_command_enabled: true`, active state: MOVEJ

| Value | Action |
|---|---|
| `0` | Move to **close** pose (`home_configs[target_command_close_config]`) |
| `1` | Move to **open** pose  (`home_configs[target_command_open_config]`)  |

```bash
ros2 topic pub --once /left_hand_controller/target_command std_msgs/msg/Int32 "data: 0"  # close
ros2 topic pub --once /left_hand_controller/target_command std_msgs/msg/Int32 "data: 1"  # open
```

### 5.4 Dexterous hand — percent control

**Topic:** `/{controller_name}/target_percent`  
**Type:** `std_msgs/Float64` (range: `0.0` ~ `1.0`)  
**Requires:** `target_command_enabled: true`, active state: MOVEJ

Linearly interpolates between the close pose and open pose **per joint**:

```
target[i] = close_config[i] + percent × (open_config[i] − close_config[i])
```

| Value | Result |
|---|---|
| `0.0` | Fully closed (same as `target_command = 0`) |
| `1.0` | Fully open  (same as `target_command = 1`) |
| `0.0~1.0` | Proportional blend between the two poses |

Values outside `[0.0, 1.0]` are clamped with a warning.

```bash
ros2 topic pub --once /left_hand_controller/target_percent \
  std_msgs/msg/Float64 "data: 0.6"   # 60% open
```

### 5.5 Waist lifting — position

**Topic:** `/{controller_name}/waist_lifting`  
**Type:** `std_msgs/Float64`  
**Requires:** `waist_lifting_enabled: true`, active state: MOVEJ

Moves the waist by the specified distance (meters) from its current position.

```bash
ros2 topic pub --once /body_controller/waist_lifting std_msgs/msg/Float64 "data: 0.05"
```

### 5.6 Waist lifting — velocity factor

**Topic:** `/{controller_name}/waist_lifting_command`  
**Type:** `std_msgs/Float64` (factor range: `[-1.0, 1.0]`)  
**Requires:** `waist_lifting_enabled: true`, active state: MOVEJ

Continuous velocity control. Actual speed = `factor × waist_lifting_default_parameter[0]`.  
Send `0.0` to stop.

```bash
ros2 topic pub /body_controller/waist_lifting_command std_msgs/msg/Float64 "data: 0.5"   # rise
ros2 topic pub --once /body_controller/waist_lifting_command std_msgs/msg/Float64 "data: 0.0"  # stop
```

### 5.7 Waist turning — velocity factor

**Topic:** `/{controller_name}/waist_turning_command`  
**Type:** `std_msgs/Float64` (factor range: `[-1.0, 1.0]`)  
**Requires:** `waist_lifting_enabled: true`, active state: MOVEJ

Continuous velocity control. Actual speed = `factor × waist_turning_default_parameter[0]`.  
Send `0.0` to stop.

```bash
ros2 topic pub /body_controller/waist_turning_command std_msgs/msg/Float64 "data: -0.3"  # turn left
```

---

## 6. Topic Summary

Assuming controller name `my_controller`, joint name `j1`:

| Topic | Type | Requires state | Notes |
|---|---|---|---|
| `/fsm_command` | `Int32` | any | FSM state/config switching |
| `/my_controller/target_joint_position` | `Float64MultiArray` | MOVEJ | Direct joint targets |
| `/my_controller/target_joint_trajectory` | `JointTrajectory` | MOVEJ | Multi-waypoint trajectory |
| `/my_controller/target_command` | `Int32` (0/1) | MOVEJ | Hand open/close switch |
| `/my_controller/target_percent` | `Float64` (0~1) | MOVEJ | Hand proportional control |
| `/my_controller/waist_lifting` | `Float64` | MOVEJ | Waist position delta |
| `/my_controller/waist_lifting_command` | `Float64` | MOVEJ | Waist velocity factor |
| `/my_controller/waist_turning_command` | `Float64` | MOVEJ | Waist turning velocity factor |

---

## 7. Demo Launch

```bash
source ~/ros2_ws/install/setup.bash

# Launch head + body controllers (default)
ros2 launch basic_joint_controller demo.launch.py robot:=fiveages_w1

# Head only
ros2 launch basic_joint_controller demo.launch.py robot:=fiveages_w1 enable_body:=false

# Body only
ros2 launch basic_joint_controller demo.launch.py robot:=fiveages_w1 enable_head:=false
```

| Argument | Default | Description |
|---|---|---|
| `robot` | `fiveages_w1` | Robot name |
| `type` | — | Robot type (optional) |
| `hardware` | `mock_components` | `gz` / `isaac` / `mock_components` |
| `world` | `dart` | Gazebo world (only when `hardware=gz`) |
| `enable_head` | `true` | Enable head controllers |
| `enable_body` | `true` | Enable body controllers |
| `use_rviz` | `true` | Launch RViz |

---

## 8. Implementation Notes

### Interpolation

Both Home and MoveJ states support configurable interpolation:

- **`tanh`** (default): smooth S-curve acceleration/deceleration
  ```
  phase = tanh(t/T × scale)
  pos   = start + phase × (target − start)
  ```
- **`linear`**: constant velocity

### Thread Safety

`StateMoveJ::setTargetPosition()` is protected by a mutex, safe to call from any ROS callback thread.
