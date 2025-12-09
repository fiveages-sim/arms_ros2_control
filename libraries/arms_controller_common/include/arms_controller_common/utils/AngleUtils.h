//
// Angle Utilities - Common utility functions for angle unwrapping and quaternion operations
//
#pragma once

#include <cmath>
#include <array>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace arms_controller_common
{
    /**
     * @brief Angle utilities for handling angle unwrapping and quaternion operations
     * 
     * This utility class provides common functions for:
     * - Unwrapping angles to maintain continuity (avoiding jumps from 179° to -179°)
     * - Converting between quaternions and RPY angles with continuity preservation
     * 
     * Usage examples:
     * 1. Unwrap RPY angles to maintain continuity:
     *    double roll = 3.14, pitch = 0.0, yaw = -3.14;
     *    double last_roll = 0.0, last_pitch = 0.0, last_yaw = 0.0;
     *    AngleUtils::unwrapRPY(roll, pitch, yaw, last_roll, last_pitch, last_yaw);
     * 
     * 2. Extract RPY from quaternion with continuity:
     *    geometry_msgs::msg::Quaternion quat;
     *    double last_rpy[3] = {0.0, 0.0, 0.0};
     *    bool initialized = false;
     *    auto rpy = AngleUtils::quaternionToRPY(quat, last_rpy, initialized);
     */
    class AngleUtils
    {
    public:
        /**
         * @brief Unwrap a single angle to maintain continuity
         * 
         * Adjusts the angle by adding or subtracting 2π to minimize the difference
         * from the reference angle, avoiding jumps from 179° to -179°.
         * 
         * @param angle Current angle (input/output, modified in place)
         * @param reference Reference angle (typically the previous value)
         * @return Unwrapped angle
         */
        static double unwrapAngle(double angle, double reference)
        {
            const double two_pi = 2.0 * M_PI;
            double diff = angle - reference;
            
            // Normalize the difference to [-π, π] range
            while (diff > M_PI)
            {
                angle -= two_pi;
                diff -= two_pi;
            }
            while (diff < -M_PI)
            {
                angle += two_pi;
                diff += two_pi;
            }
            
            return angle;
        }

        /**
         * @brief Unwrap RPY angles to maintain continuity
         * 
         * Adjusts roll, pitch, and yaw angles to minimize jumps from the reference values.
         * Updates the reference values in place.
         * 
         * @param roll Roll angle (input/output, modified in place)
         * @param pitch Pitch angle (input/output, modified in place)
         * @param yaw Yaw angle (input/output, modified in place)
         * @param last_roll Last roll angle (input/output, updated to current value)
         * @param last_pitch Last pitch angle (input/output, updated to current value)
         * @param last_yaw Last yaw angle (input/output, updated to current value)
         */
        static void unwrapRPY(
            double& roll, double& pitch, double& yaw,
            double& last_roll, double& last_pitch, double& last_yaw)
        {
            roll = unwrapAngle(roll, last_roll);
            pitch = unwrapAngle(pitch, last_pitch);
            yaw = unwrapAngle(yaw, last_yaw);
            
            // Update reference values
            last_roll = roll;
            last_pitch = pitch;
            last_yaw = yaw;
        }

        /**
         * @brief Unwrap RPY angles using array storage
         * 
         * Convenience function that uses arrays for storing last RPY values.
         * 
         * @param roll Roll angle (input/output, modified in place)
         * @param pitch Pitch angle (input/output, modified in place)
         * @param yaw Yaw angle (input/output, modified in place)
         * @param last_rpy Array of [roll, pitch, yaw] for last values (input/output, updated)
         * @param initialized Whether last_rpy has been initialized (input/output, updated)
         */
        static void unwrapRPY(
            double& roll, double& pitch, double& yaw,
            std::array<double, 3>& last_rpy,
            bool& initialized)
        {
            if (!initialized)
            {
                last_rpy[0] = roll;
                last_rpy[1] = pitch;
                last_rpy[2] = yaw;
                initialized = true;
                return;
            }
            
            unwrapRPY(roll, pitch, yaw, last_rpy[0], last_rpy[1], last_rpy[2]);
        }

        /**
         * @brief Extract RPY angles from quaternion with continuity preservation
         * 
         * Converts a quaternion to RPY angles and unwraps them to maintain continuity.
         * 
         * @param quaternion Input quaternion
         * @param last_rpy Array of [roll, pitch, yaw] for last values (input/output, updated)
         * @param initialized Whether last_rpy has been initialized (input/output, updated)
         * @return Array of [roll, pitch, yaw] angles in radians
         */
        static std::array<double, 3> quaternionToRPY(
            const geometry_msgs::msg::Quaternion& quaternion,
            std::array<double, 3>& last_rpy,
            bool& initialized)
        {
            // Convert quaternion to RPY
            tf2::Quaternion tf_quat;
            tf2::fromMsg(quaternion, tf_quat);
            
            double roll, pitch, yaw;
            tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
            
            // Unwrap to maintain continuity
            unwrapRPY(roll, pitch, yaw, last_rpy, initialized);
            
            return {roll, pitch, yaw};
        }

        /**
         * @brief Extract RPY angles from quaternion with continuity preservation (using separate variables)
         * 
         * @param quaternion Input quaternion
         * @param last_roll Last roll angle (input/output, updated)
         * @param last_pitch Last pitch angle (input/output, updated)
         * @param last_yaw Last yaw angle (input/output, updated)
         * @return Array of [roll, pitch, yaw] angles in radians
         */
        static std::array<double, 3> quaternionToRPY(
            const geometry_msgs::msg::Quaternion& quaternion,
            double& last_roll, double& last_pitch, double& last_yaw)
        {
            // Convert quaternion to RPY
            tf2::Quaternion tf_quat;
            tf2::fromMsg(quaternion, tf_quat);
            
            double roll, pitch, yaw;
            tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
            
            // Unwrap to maintain continuity
            unwrapRPY(roll, pitch, yaw, last_roll, last_pitch, last_yaw);
            
            return {roll, pitch, yaw};
        }
    };
} // namespace arms_controller_common

