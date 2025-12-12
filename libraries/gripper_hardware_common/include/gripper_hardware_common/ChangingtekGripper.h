//
// Changingtek Gripper - Unified header for Changingtek gripper utilities
//
// This header includes all necessary utilities for working with Changingtek grippers:
// - Position conversion (normalized <-> Modbus)
// - Modbus configuration constants
// - Command change detection
// - Read frequency control
//
#pragma once

#include "gripper_hardware_common/utils/PositionConverter.h"
#include "gripper_hardware_common/utils/ModbusConfig.h"
#include "gripper_hardware_common/utils/CommandChangeDetector.h"
#include "gripper_hardware_common/utils/ReadFrequencyController.h"

namespace gripper_hardware_common
{
    /**
     * @brief Convenience namespace for Changingtek gripper utilities
     * 
     * This header provides easy access to all Changingtek-related utilities.
     * After including this header, you can use:
     * 
     * @code
     * using namespace gripper_hardware_common;
     * 
     * // Position conversion
     * uint16_t modbus_pos = PositionConverter::Changingtek90::normalizedToModbus(0.5);
     * double normalized = PositionConverter::Changingtek90::modbusToNormalized(4500);
     * 
     * // Modbus configuration
     * uint16_t pos_reg = ModbusConfig::Changingtek90::POS_REG_ADDR;
     * 
     * // Command change detection
     * if (CommandChangeDetector::hasChanged(current, last)) { ... }
     * 
     * // Read frequency control
     * ReadFrequencyController controller(4);
     * if (controller.shouldRead()) { ... }
     * @endcode
     */
} // namespace gripper_hardware_common
