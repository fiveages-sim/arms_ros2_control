//
// FSM State Transition Validator - Common utility for validating FSM state transitions
//
#pragma once

#include <string>
#include <cstdint>

namespace arms_controller_common
{
    /**
     * @brief FSM State Transition Validator - Validates state transitions based on FSM command
     * 
     * This utility class provides common functions for validating FSM state transitions
     * to ensure only valid state changes are allowed. This prevents invalid state transitions
     * that could cause system instability.
     * 
     * Valid state transitions:
     * - command = 1: HOLD → HOME (only from HOLD state)
     * - command = 2: HOME/OCS2/MOVEJ → HOLD (only from HOME, OCS2, or MOVEJ states)
     * - command = 3: HOLD → OCS2 (only from HOLD state)
     * - command = 4: HOLD → MOVEJ (only from HOLD state)
     * 
     * Usage example:
     * ```cpp
     * std::string current_state = "HOLD";
     * int32_t command = 3;  // HOLD → OCS2
     * 
     * std::string new_state;
     * bool valid = FSMStateTransitionValidator::validateTransition(
     *     current_state, command, new_state);
     * 
     * if (valid) {
     *     current_state = new_state;
     *     // Process state change
     * } else {
     *     // Ignore invalid transition
     * }
     * ```
     */
    class FSMStateTransitionValidator
    {
    public:
        /**
         * @brief Validate a state transition based on current state and command
         * 
         * @param current_state Current FSM state ("HOME", "HOLD", "OCS2", "MOVEJ")
         * @param command FSM command (1, 2, 3, 4, or special commands like 0, 100)
         * @param new_state Output parameter: new state if transition is valid (unchanged if invalid)
         * @return true if transition is valid, false otherwise
         */
        static bool validateTransition(
            const std::string& current_state,
            int32_t command,
            std::string& new_state)
        {
            // Initialize new_state to current_state (no change by default)
            new_state = current_state;
            
            switch (command)
            {
                case 1: // HOLD → HOME
                    if (current_state == "HOLD")
                    {
                        new_state = "HOME";
                        return true;
                    }
                    break;
                    
                case 2: // HOME → HOLD 或 OCS2 → HOLD 或 MOVEJ → HOLD
                    if (current_state == "HOME" || current_state == "OCS2" || current_state == "MOVEJ")
                    {
                        new_state = "HOLD";
                        return true;
                    }
                    break;
                    
                case 3: // HOLD → OCS2
                    if (current_state == "HOLD")
                    {
                        new_state = "OCS2";
                        return true;
                    }
                    break;
                    
                case 4: // HOLD → MOVEJ
                    if (current_state == "HOLD")
                    {
                        new_state = "MOVEJ";
                        return true;
                    }
                    break;
                    
                default:
                    // 对于其他命令（如 0, 100 等），不进行状态转换验证
                    // 这些命令不影响状态转换
                    return false;
            }
            
            return false;
        }
        
        /**
         * @brief Check if a state transition is valid without modifying state
         * 
         * @param current_state Current FSM state
         * @param command FSM command
         * @return true if transition would be valid, false otherwise
         */
        static bool isValidTransition(const std::string& current_state, int32_t command)
        {
            std::string dummy_state;
            return validateTransition(current_state, command, dummy_state);
        }
    };
} // namespace arms_controller_common

