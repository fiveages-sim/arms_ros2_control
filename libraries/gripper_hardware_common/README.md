# Gripper Hardware Common

Common libraries for gripper hardware interfaces, providing shared utilities for position conversion, Modbus configuration, command change detection, and read frequency control.

## Overview

This package provides common utilities used across different gripper hardware implementations in the `arms_ros2_control` project. It helps reduce code duplication and ensures consistency across different gripper implementations.

## Features

### 1. Position Conversion (`PositionConverter`)

Provides position conversion utilities for different gripper types:

- **Changingtek 90**: Converts between normalized position (0.0-1.0) and Modbus position (0-9000)
- **Jodell**: Converts between normalized position (0.0-1.0) and Jodell position (0-255)

All conversions use normalized position where:
- `0.0` = closed
- `1.0` = open

### 2. Modbus Configuration (`ModbusConfig`)

Provides Modbus register addresses, function codes, and other constants for:

- **Changingtek 90**: Register addresses, function codes, default parameters
- **Jodell**: Register addresses, function codes, slave address, control data format

### 3. Command Change Detection (`CommandChangeDetector`)

Utility for detecting significant command changes to avoid sending redundant commands:

- Percentage-based threshold (default: 1%)
- Absolute threshold for physical positions
- Percentage-based threshold relative to max value

### 4. Read Frequency Control (`ReadFrequencyController`)

Utility for controlling read frequency to reduce Modbus communication load:

- Configurable read interval (default: read every 4 cycles)
- Counter-based mechanism
- Reset and interval adjustment support

### 5. Jodell Command Builder (`JodellCommandBuilder`)

Utility for building Jodell gripper Modbus commands:

- Builds 3-register command format
- Supports position, torque, and velocity settings
- Can build from normalized position

## Usage Examples

### Position Conversion

```cpp
#include "gripper_hardware_common/utils/PositionConverter.h"

using namespace gripper_hardware_common;

// Changingtek 90
double normalized = 0.5;  // 50% open
uint16_t modbus_pos = PositionConverter::Changingtek90::normalizedToModbus(normalized);
// modbus_pos = 4500

uint32_t modbus_feedback = 3000;
double normalized_back = PositionConverter::Changingtek90::modbusToNormalized(modbus_feedback);
// normalized_back ≈ 0.667

// Jodell
int jodell_pos = PositionConverter::Jodell::normalizedToJodell(normalized);
// jodell_pos = 127

double normalized_jodell = PositionConverter::Jodell::jodellToNormalized(200);
// normalized_jodell ≈ 0.216
```

### Modbus Configuration

```cpp
#include "gripper_hardware_common/utils/ModbusConfig.h"

using namespace gripper_hardware_common::ModbusConfig;

// Changingtek 90
uint16_t pos_reg = Changingtek90::POS_REG_ADDR;  // 0x0102
uint16_t feedback_reg = Changingtek90::FEEDBACK_REG_ADDR;  // 0x060D

// Jodell
uint16_t init_reg = Jodell::INIT_REG_ADDR;  // 0x03E8
uint16_t status_reg = Jodell::STATUS_REG_ADDR;  // 0x07D0
uint8_t slave_addr = Jodell::SLAVE_ADDRESS;  // 0x09
```

### Command Change Detection

```cpp
#include "gripper_hardware_common/utils/CommandChangeDetector.h"

using namespace gripper_hardware_common;

double current = 0.52;
double last = 0.50;

// Check with default threshold (1%)
if (CommandChangeDetector::hasChanged(current, last)) {
    // Send command
}

// Check with custom threshold (2%)
if (CommandChangeDetector::hasChanged(current, last, 0.02)) {
    // Send command
}

// Check with absolute threshold (for physical positions in meters)
double threshold_m = 0.0004;  // 0.4mm
if (CommandChangeDetector::hasChangedAbsolute(current, last, threshold_m)) {
    // Send command
}
```

### Read Frequency Control

```cpp
#include "gripper_hardware_common/utils/ReadFrequencyController.h"

using namespace gripper_hardware_common;

// Create controller with default interval (4)
ReadFrequencyController read_controller;

// In read loop
if (read_controller.shouldRead()) {
    // Read gripper status
    readGripperStatus();
}

// Or with custom interval
ReadFrequencyController custom_controller(8);  // Read every 8 cycles
```

### Jodell Command Builder

```cpp
#include "gripper_hardware_common/utils/JodellCommandBuilder.h"

using namespace gripper_hardware_common;

// Build command with position, torque, and velocity
int pos_set = 128;  // 50% open
int trq_set = 100;
int vel_set = 50;
auto command = JodellCommandBuilder::buildCommand(pos_set, trq_set, vel_set);
// command = [0x09, 0x8000, 0x6432]

// Build command with default torque and velocity
auto command2 = JodellCommandBuilder::buildCommand(pos_set);
// command2 = [0x09, 0x8000, 0xFFFF]

// Build from normalized position
double normalized = 0.5;
auto command3 = JodellCommandBuilder::buildCommandFromNormalized(normalized);
```

## Dependencies

- `rclcpp`: ROS2 C++ client library (for logging, if needed in future)

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select gripper_hardware_common --symlink-install
```

## Integration

To use this package in other packages, add to `CMakeLists.txt`:

```cmake
find_package(gripper_hardware_common REQUIRED)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
    gripper_hardware_common
)
```

And to `package.xml`:

```xml
<depend>gripper_hardware_common</depend>
```

## Related Packages

This package is designed to be used by:
- `modbus_ros2_control`: Modbus-based gripper hardware interface
- `dobot_ros2_control`: Dobot robot hardware interface (with Changingtek gripper)
- `rokae_ros2_control`: Rokae robot hardware interface (with Jodell gripper)
- `marvin_ros2_control`: Marvin robot hardware interface (with Jodell gripper)

## License

Apache-2.0
