// GripperBase - common abstract interface for gripper implementations
#pragma once

namespace gripper_hardware_common
{
    /**
     * @brief Common gripper interface shared across hardware implementations.
     *
     * Derived classes provide hardware-specific logic while exposing a uniform
     * initialize / move / status API that higher-level components can rely on.
     */
    class GripperBase
    {
    public:
        virtual ~GripperBase() = default;

        /// Perform hardware initialization/configuration.
        virtual bool initialize() = 0;

        /// Move the gripper with the given torque, velocity, and normalized position (0.0=closed, 1.0=open).
        virtual bool move_gripper(int torque, int velocity, double position) = 0;

        /// Send async read request for current status (status updated by recv_thread_func).
        virtual bool getStatus() = 0;

        /// Read data from port and process it (for Modbus grippers).
        /// Returns true if data was successfully read and processed.
        virtual bool readAndProcessData(int& torque, int& velocity, double& position)
        {
            return false;  // Default implementation for non-Modbus grippers
        }

        /// Optional teardown hook.
        virtual void deinitialize() {}

        /// Optional state reset hook.
        virtual void resetState() {}
    };
} // namespace gripper_hardware_common

