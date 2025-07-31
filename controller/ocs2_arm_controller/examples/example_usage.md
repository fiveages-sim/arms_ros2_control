# OCS2 Arm Controller Usage Example

This document shows how to use the OCS2 Arm Controller in a typical ROS2 Control setup.

## Basic Setup

### 1. Hardware Interface Configuration

Your hardware interface should provide the following interfaces:

```yaml
# In your hardware interface configuration
hardware_interface:
  joints:
    - name: joint1
      command_interfaces:
        - position
      state_interfaces:
        - position
    - name: joint2
      command_interfaces:
        - position
      state_interfaces:
        - position
    # ... repeat for all joints
```

### 2. Controller Configuration

```yaml
# ocs2_arm_controller.yaml
ocs2_arm_controller:
  type: ocs2_arm_controller/Ocs2ArmController
  
  joints:
    - joint1
    - joint2
    - joint3
    - joint4
    - joint5
    - joint6
  
  command_interfaces:
    - position
  
  state_interfaces:
    - position
  
  home_pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  update_rate: 1000
```

### 3. Launch File Example

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ocs2_arm_controller')
    controller_config = os.path.join(pkg_share, 'config', 'ocs2_arm_controller.yaml')
    
    # Controller manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config],
        output='screen',
    )
    
    return LaunchDescription([
        controller_manager_node,
    ])
```

## Usage Commands

### Load the Controller

```bash
ros2 control load_controller ocs2_arm_controller
```

### Start the Controller

```bash
ros2 control set_controller_state ocs2_arm_controller active
```

### Check Controller Status

```bash
ros2 control list_controllers
```

### Send Control Commands

The controller subscribes to `/control_input` topic for control commands:

```bash
# Publish a control command
ros2 topic pub /control_input control_input_msgs/msg/Inputs "{command: 1, lx: 0.0, ly: 0.0, rx: 0.0, ry: 0.0}"
```

### Monitor Joint States

```bash
# Monitor joint states
ros2 topic echo /joint_states
```

## State Transitions

The controller implements a simple state machine:

1. **PASSIVE**: Initial state, no active control
2. **HOME**: Moves to home position
3. **State Transitions**: Switch between HOME and ZERO states

State transitions can be triggered by control input commands or internal logic.

## Troubleshooting

### Common Issues

1. **Controller not loading**: Check that all required interfaces are available
2. **No movement**: Verify that the controller is active and receiving commands
3. **Interface mismatch**: Ensure hardware interface provides position command and state interfaces

### Debug Commands

```bash
# Check available controllers
ros2 control list_controllers

# Check controller info
ros2 control list_controller_types

# Monitor controller logs
ros2 run ocs2_arm_controller ocs2_arm_controller --ros-args --log-level debug
``` 