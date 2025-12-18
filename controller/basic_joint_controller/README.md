# Basic Joint Controller

A basic joint controller with three finite state machine states: Home, Hold, and Move.

## 1. Building

```bash
cd ~/ros2_ws
colcon build --packages-up-to basic_joint_controller
```

## 2. Features

- **StateHome**: Smoothly interpolates from current position to preset home positions
- **StateHold**: Maintains current joint positions unchanged
- **StateMove**: Receives target joint positions and smoothly interpolates from current position to target

## 3. Configuration Parameters

The following parameters need to be set in the controller configuration file:

```yaml
basic_joint_controller:
  ros__parameters:
    update_rate: 1000  # Controller update rate (Hz)
    joints: ["joint1", "joint2", "joint3", ...]  # List of joint names
    command_interfaces: ["position"]  # Command interface types
    state_interfaces: ["position", "velocity"]  # State interface types
    home_1: [0.0, 0.0, 0.0, ...]  # Home configuration 1 joint angles (radians)
    home_2: [0.5, 0.5, 0.5, ...]  # Home configuration 2 joint angles (optional)
    home_3: [-0.5, -0.5, -0.5, ...]  # Home configuration 3 joint angles (optional)
    # Can continue adding home_4, home_5, ... up to 10 configurations
    home_duration: 3.0  # Home state interpolation duration (seconds)
    move_duration: 3.0  # Move state interpolation duration (seconds)
    home_interpolation_type: "tanh"  # "tanh" (default) or "linear"
    home_tanh_scale: 3.0  # Only used when home_interpolation_type == "tanh"
    movej_interpolation_type: "tanh"  # "tanh" (default) or "linear"
    movej_tanh_scale: 3.0  # Only used when movej_interpolation_type == "tanh"
    hold_position_threshold: 0.1  # Hold state position threshold (radians)
    switch_command_base: 100  # Base command value for configuration switching (default: 100, can be set to 4 for backward compatibility)
```

## 4. State Transitions

State transitions are controlled via the `command` field of the `/control_input` topic:

- `command = 1`: Switch to HOME state
- `command = 2`: Switch to HOLD state
- `command = 3`: Switch to MOVE state

### 4.1 Home State Configuration Switching

In the HOME state, you can switch between different home configurations using the following commands:

- `command = switch_command_base` (default 100): Cycle to next configuration
- `command = switch_command_base + 1` (default 101): Switch to configuration 0
- `command = switch_command_base + 2` (default 102): Switch to configuration 1
- `command = switch_command_base + 3` (default 103): Switch to configuration 2
- ... and so on

For example, if `switch_command_base = 4` (for backward compatibility):
- `command = 4`: Cycle to next
- `command = 5`: Switch to configuration 0
- `command = 6`: Switch to configuration 1

## 5. Usage

### 5.1 Demo Launch File

The demo launch file provides a convenient way to launch head controllers or body controllers:

```bash
# Launch both head and body controllers (default)
source ~/ros2_ws/install/setup.bash
ros2 launch basic_joint_controller demo.launch.py robot:=fiveages_w1

# Launch only head controller
ros2 launch basic_joint_controller demo.launch.py robot:=fiveages_w1 enable_body:=false

# Launch only body controller
ros2 launch basic_joint_controller demo.launch.py robot:=fiveages_w1 enable_head:=false
```

Launch file arguments:
- `robot`: Robot name (default: fiveages_w1)
- `type`: Robot type (optional)
- `hardware`: Hardware type, 'gz' for Gazebo simulation, 'isaac' for Isaac simulation, 'mock_components' for mock components (default: mock_components)
- `world`: Gazebo world name (only used when hardware=gz, default: dart)
- `enable_head`: Enable head controllers (default: true)
- `enable_body`: Enable body controllers (default: true)
- `use_rviz`: Launch RViz visualization (default: true)

### 5.2 State Switching

Switch states by publishing to the `/control_input` topic:

```bash
ros2 topic pub /control_input arms_ros2_control_msgs/msg/Inputs "{command: 1}"  # Switch to HOME
ros2 topic pub /control_input arms_ros2_control_msgs/msg/Inputs "{command: 2}"  # Switch to HOLD
ros2 topic pub /control_input arms_ros2_control_msgs/msg/Inputs "{command: 3}"  # Switch to MOVE
```

### 5.3 Setting Target Position (Move State)

In the MOVE state, set target joint positions by publishing to the `target_joint_position` topic (relative to controller namespace):

```bash
# For a controller named "head_joint_controller":
ros2 topic pub /head_joint_controller/target_joint_position std_msgs/msg/Float64MultiArray "{data: [1.0, 0.5, -0.5, ...]}"  # Set target joint angles (radians)

# For a controller named "waist_joint_controller":
ros2 topic pub /waist_joint_controller/target_joint_position std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, ...]}"  # Set target joint angles (radians)
```

**Note**: The topic name is relative to the controller's namespace, allowing multiple controller instances (e.g., one for head, one for waist) to run simultaneously without conflicts.

## 6. Implementation Details

### 6.1 State Machine Architecture

- All states inherit from the `FSMState` base class
- Each state implements `enter()`, `run()`, `exit()`, and `checkChange()` methods
- State transitions are determined by checking control inputs in the `checkChange()` method

### 6.2 Interpolation Algorithm

The Home state uses the `tanh` function for smooth interpolation. The MoveJ state supports configurable interpolation.

```cpp
double phase = std::tanh(percent_ * 3.0);
double interpolated_value = phase * target_pos[i] + (1.0 - phase) * start_pos[i];
```

This ensures smooth acceleration and deceleration.

#### MoveJ interpolation parameters

- **`movej_interpolation_type`**: `"tanh"` (default) or `"linear"`
- **`movej_tanh_scale`**: tanh scale (default `3.0`), only used when `movej_interpolation_type="tanh"`

### 6.3 Thread Safety

StateMove uses mutex locks to protect target position updates, ensuring safety in multi-threaded environments.
