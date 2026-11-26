/******************************************************************************
 * Minimal robot context for pose-based reference generation.
 ******************************************************************************/

#pragma once

#include <string>

namespace ocs2::controller_common {

/** Values needed to build TargetTrajectories from EE pose targets. */
struct Ocs2ReferenceTargetContext {
    bool dual_arm{false};
    std::string base_frame;
    int input_dim{0};

    int reference_target_state_dim{0};

    bool body_pose_from_current_state{true};

    bool body_target_enabled{false};

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
