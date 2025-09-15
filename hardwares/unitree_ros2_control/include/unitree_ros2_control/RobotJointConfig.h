//
// Created by fa on 2025/9/15.
//

#pragma once
#include <array>

enum G1JointIndex {
    // Left leg
    kLeftHipPitch,
    kLeftHipRoll,
    kLeftHipYaw,
    kLeftKnee,
    kLeftAnkle,
    kLeftAnkleRoll,

    // Right leg
    kRightHipPitch,
    kRightHipRoll,
    kRightHipYaw,
    kRightKnee,
    kRightAnkle,
    kRightAnkleRoll,

    kWaistYaw,
    kWaistRoll,
    kWaistPitch,

    // Left arm
    kLeftShoulderPitch,
    kLeftShoulderRoll,
    kLeftShoulderYaw,
    kLeftElbow,
    kLeftWristRoll,
    kLeftWristPitch,
    kLeftWristYaw,
    // Right arm
    kRightShoulderPitch,
    kRightShoulderRoll,
    kRightShoulderYaw,
    kRightElbow,
    kRightWristRoll,
    kRightWristPitch,
    kRightWristYaw,

    kNotUsedJoint,
    kNotUsedJoint1,
    kNotUsedJoint2,
    kNotUsedJoint3,
    kNotUsedJoint4,
    kNotUsedJoint5
};

inline std::array<G1JointIndex, 17> arm_joints = {
    kLeftShoulderPitch, kLeftShoulderRoll,
    kLeftShoulderYaw, kLeftElbow,
    kLeftWristRoll, kLeftWristPitch,
    kLeftWristYaw,
    kRightShoulderPitch, kRightShoulderRoll,
    kRightShoulderYaw, kRightElbow,
    kRightWristRoll, kRightWristPitch,
    kRightWristYaw,
    kWaistYaw,
    kWaistRoll,
    kWaistPitch
};
