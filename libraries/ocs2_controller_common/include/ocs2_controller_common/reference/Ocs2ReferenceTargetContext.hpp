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

    /// Optional legacy override for non-wheel-humanoid users. Wheel-humanoid layout is derived from capabilities below.
    int reference_target_state_dim{0};

    /// When the body capability is enabled, fill its block from setBodyPoseReference() each cycle
    /// (typically FK). If false, the block uses zero translation + identity quaternion.
    bool body_pose_from_current_state{true};

    /// Enables publication of the cached Cartesian body target.
    /// Set by controllers that explicitly support body tracking EE.
    bool body_target_enabled{false};

    /// Enables the optional 7D head pose block and Head target ROS interfaces.
    bool head_target_enabled{false};

    static constexpr int kPoseTargetDim = 7;
    static constexpr int kDualArmTargetDim = 14;

    [[nodiscard]] int bodyTargetOffset() const { return dual_arm ? kDualArmTargetDim : 7; }
    [[nodiscard]] int headTargetOffset() const {
        return bodyTargetOffset() + (body_target_enabled ? kPoseTargetDim : 0);
    }
    [[nodiscard]] int wheelHumanoidTargetStateDim() const {
        return bodyTargetOffset() +
               (body_target_enabled ? kPoseTargetDim : 0) +
               (head_target_enabled ? kPoseTargetDim : 0);
    }
};

} // namespace ocs2::controller_common
