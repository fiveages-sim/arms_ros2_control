# Topic Based ROS2 Control

A ROS2 control hardware interface that communicates with hardware through ROS2 topics.

## Features

- Communicates with hardware through ROS2 topics
- Supports position, velocity, and effort interfaces
- Configurable initialization behavior for real hardware safety

## Configuration

### Basic Configuration

```xml
<ros2_control name="my_system" type="system">
  <hardware>
    <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
    <param name="joint_commands_topic">/robot_joint_commands</param>
    <param name="joint_states_topic">/robot_joint_states</param>
    <param name="initialize_commands_from_state">true</param>
  </hardware>
  
  <joint name="joint1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

### Safety Parameters

#### `initialize_commands_from_state` (Recommended for Real Hardware)

- **true** (default): Initialize command values from the first received joint state
  - Safe for real hardware - prevents sudden movements on startup
  - Commands are set to match current hardware position
  - Recommended for production systems

- **false**: Use `initial_value` from configuration
  - May cause sudden movements if hardware position differs from `initial_value`
  - Only use for simulation or when you're certain about initial positions

#### Example: Safe Real Hardware Configuration

```xml
<ros2_control name="cr5_system" type="system">
  <hardware>
    <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
    <param name="joint_commands_topic">/joint_command_robot</param>
    <param name="joint_states_topic">/joint_states_robot</param>
    <param name="initialize_commands_from_state">true</param>
  </hardware>
  
  <joint name="joint1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

#### Example: Simulation Configuration

```xml
<ros2_control name="cr5_system" type="system">
  <hardware>
    <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
    <param name="joint_commands_topic">/isaac/joint_command</param>
    <param name="joint_states_topic">/isaac/joint_states</param>
    <param name="initialize_commands_from_state">false</param>
  </hardware>
  
  <joint name="joint1">
    <command_interface name="position"/>
    <state_interface name="position">
      <param name="initial_value">0.0</param>
    </state_interface>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

## Usage

1. Load the controller manager with your configuration
2. The system will automatically initialize based on the `initialize_commands_from_state` parameter
3. If `initialize_commands_from_state` is true, commands will be set to match the first received joint state
4. If `initialize_commands_from_state` is false, commands will use the `initial_value` from configuration

## Safety Notes

- Always use `initialize_commands_from_state=true` for real hardware
- This prevents sudden movements when the controller starts
- The system will log initialization behavior for debugging
