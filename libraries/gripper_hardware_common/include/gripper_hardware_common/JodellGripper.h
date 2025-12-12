//
// Jodell Gripper - Unified header for Jodell gripper utilities
//
// This header includes all necessary utilities for working with Jodell grippers:
// - Position conversion (normalized <-> Jodell)
// - Modbus configuration constants
// - Command change detection
// - Read frequency control
// - Command builder
//
#pragma once

#include "gripper_hardware_common/utils/PositionConverter.h"
#include "gripper_hardware_common/utils/ModbusConfig.h"
#include "gripper_hardware_common/utils/CommandChangeDetector.h"
#include "gripper_hardware_common/utils/ReadFrequencyController.h"
#include "gripper_hardware_common/utils/JodellCommandBuilder.h"

namespace gripper_hardware_common
{
    /**
     * @brief Convenience namespace for Jodell gripper utilities
     * 
     * This header provides easy access to all Jodell-related utilities.
     * After including this header, you can use:
     * 
     * @code
     * using namespace gripper_hardware_common;
     * 
     * // Position conversion
     * int jodell_pos = PositionConverter::Jodell::normalizedToJodell(0.5);
     * double normalized = PositionConverter::Jodell::jodellToNormalized(128);
     * 
     * // Modbus configuration
     * uint16_t init_reg = ModbusConfig::Jodell::INIT_REG_ADDR;
     * 
     * // Command building
     * auto command = JodellCommandBuilder::buildCommand(128, 100, 50);
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
