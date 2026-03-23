/******************************************************************************
 * Minimal robot context for pose-based reference generation (no MobileManipulatorInterface).
 ******************************************************************************/

#pragma once

#include <string>

namespace ocs2::controller_common {

/** Values needed to build TargetTrajectories from EE pose targets without a full OCS2 robot interface. */
struct Ocs2ReferenceTargetContext {
    bool dual_arm{false};
    std::string base_frame;
    int input_dim{0};
};

} // namespace ocs2::controller_common
