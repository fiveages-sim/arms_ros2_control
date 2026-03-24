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

    /// 0 = auto: 7 (single arm) or 14 (dual arm). Use kWheelHumanoidTargetStateDim for wheel-humanoid switched layout.
    int reference_target_state_dim{0};

    /// When reference_target_state_dim == kWheelHumanoidTargetStateDim: fill body block [14:21] from setBodyPoseReference()
    /// each cycle (typically FK). If false, body block uses zero translation + identity quaternion ("empty" body target).
    bool body_pose_from_current_state{true};

    static constexpr int kWheelHumanoidTargetStateDim = 21;
};

} // namespace ocs2::controller_common
