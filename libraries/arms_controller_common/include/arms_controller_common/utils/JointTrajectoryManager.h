#pragma once

#include "arms_controller_common/utils/Interpolation.h"
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>

// 条件编译：只在 lina_planning 可用时包含相关头文件
#ifdef HAS_LINA_PLANNING
#include "lina_planning/planning/path_planner/movej.h"
#include "lina_planning/planning/path_planner/smooth_curve_of_multi_joints_using_blending.h"
#include "lina_planning/planning/common/trajectory_init_parameters.h"
#endif

namespace arms_controller_common
{
    /**
     * @brief Unified joint trajectory manager for single-node and multi-node trajectories
     * 
     * This class provides a unified interface for managing joint trajectories:
     * - Basic interpolation (TANH, LINEAR, NONE) - no external dependencies
     * - Advanced interpolation (DOUBLES) - requires lina_planning (optional)
     * 
     * Supports both single-node and multi-node trajectories:
     * - Single-node: direct interpolation from start to target
     * - Multi-node: segment-based interpolation (basic) or smooth blending (advanced)
     */
    class JointTrajectoryManager
    {
    public:
        /**
         * @brief Constructor
         * @param logger Optional ROS logger for logging messages
         */
        explicit JointTrajectoryManager(rclcpp::Logger logger = rclcpp::get_logger("joint_trajectory_manager"));

        /**
         * @brief Check if DOUBLES interpolation is available (lina_planning is available)
         * @return True if DOUBLES interpolation can be used
         */
        static bool isDoublesAvailable();

        /**
         * @brief Initialize single-node trajectory
         * @param start_pos Starting joint positions
         * @param target_pos Target joint positions
         * @param duration Trajectory duration in seconds
         * @param type Interpolation type (TANH, LINEAR, NONE, or DOUBLES if available)
         * @param controller_frequency Controller update frequency in Hz
         * @param tanh_scale Scale factor for TANH interpolation (default: 3.0)
         * @return True if initialization successful, false otherwise
         */
        bool initSingleNode(
            const std::vector<double>& start_pos,
            const std::vector<double>& target_pos,
            double duration,
            InterpolationType type,
            double controller_frequency,
            double tanh_scale = 3.0
        );

        /**
         * @brief Initialize multi-node trajectory
         * 
         * Basic mode (TANH/LINEAR/NONE):
         *   Uses segment-based interpolation - each segment independently interpolated
         * 
         * Advanced mode (DOUBLES, requires lina_planning):
         *   Uses SmoothCurveOfMultiJointsUsingBlending for smooth multi-segment trajectory
         * 
         * @param waypoints Vector of waypoints (at least 2 points: start and end)
         *                  For n waypoints, creates n-1 segments
         * @param durations Vector of durations for each segment (size should be waypoints.size() - 1)
         *                  If empty, uses default_duration for all segments
         * @param type Interpolation type (TANH, LINEAR, NONE, or DOUBLES if available)
         * @param controller_frequency Controller update frequency in Hz
         * @param default_duration Default duration for segments if durations vector is empty
         * @param tanh_scale Scale factor for TANH interpolation (default: 3.0)
         * @return True if initialization successful, false otherwise
         */
        bool initMultiNode(
            const std::vector<std::vector<double>>& waypoints,
            const std::vector<double>& durations,
            InterpolationType type,
            double controller_frequency,
            double default_duration = 3.0,
            double tanh_scale = 3.0
        );

        /**
         * @brief Get next trajectory point (unified interface)
         * 
         * Call this method in each control loop iteration to get the next joint positions.
         * Automatically handles progress tracking and segment transitions for multi-node trajectories.
         * 
         * @return Vector of joint positions for the next time step
         *         Returns empty vector if trajectory is not initialized or completed
         */
        std::vector<double> getNextPoint();

        /**
         * @brief Check if trajectory is completed
         * @return True if trajectory has reached the end, false otherwise
         */
        bool isCompleted() const;

        /**
         * @brief Reset trajectory manager
         * 
         * Clears all internal state and prepares for a new trajectory initialization.
         */
        void reset();

        /**
         * @brief Get current progress [0.0, 1.0]
         * @return Progress value from 0.0 (start) to 1.0 (end)
         */
        double getProgress() const;

        /**
         * @brief Check if trajectory is initialized
         * @return True if a trajectory has been initialized
         */
        bool isInitialized() const;

    private:
        // Trajectory mode
        enum class TrajectoryMode
        {
            NONE,           // Not initialized
            SINGLE_NODE,    // Single-node trajectory
            MULTI_NODE_BASIC,  // Multi-node trajectory (basic segment-based interpolation)
            MULTI_NODE_ADVANCED  // Multi-node trajectory (advanced lina_planning)
        };

        // Single-node trajectory computation
        std::vector<double> computeSingleNodePoint();

        // Multi-node trajectory computation - basic mode (segment-based)
        std::vector<double> computeMultiNodeBasic();

#ifdef HAS_LINA_PLANNING
        // Multi-node trajectory computation - advanced mode (lina_planning)
        std::vector<double> computeMultiNodeAdvanced();
#endif

        // Calculate interpolation phase based on type and progress
        double calculatePhase(InterpolationType type, double percent) const;

        // Update interpolation progress for single-node or basic multi-node
        void updateProgress();

        // Check if advanced mode should be used
        bool useAdvancedMode(InterpolationType type) const;

        // Validate initialization parameters
        bool validateSingleNodeParams(const std::vector<double>& start_pos,
                                      const std::vector<double>& target_pos,
                                      double duration,
                                      InterpolationType type) const;

        bool validateMultiNodeParams(const std::vector<std::vector<double>>& waypoints,
                                     const std::vector<double>& durations) const;

        rclcpp::Logger logger_;

        // Trajectory state
        TrajectoryMode mode_{TrajectoryMode::NONE};
        bool initialized_{false};
        bool completed_{false};

        // Single-node trajectory parameters
        std::vector<double> start_pos_;
        std::vector<double> target_pos_;
        double duration_{0.0};
        InterpolationType interpolation_type_{InterpolationType::TANH};
        double tanh_scale_{3.0};
        double percent_{0.0};  // Progress [0.0, 1.0]

        // Multi-node trajectory parameters (basic mode)
        std::vector<std::vector<double>> waypoints_;
        std::vector<double> segment_durations_;
        size_t current_segment_{0};
        std::vector<double> segment_progress_;  // Progress for each segment [0.0, 1.0]

        // Controller parameters
        double controller_frequency_{100.0};
        double period_{0.01};  // 1.0 / controller_frequency_

        // Advanced mode (lina_planning) - conditionally compiled
#ifdef HAS_LINA_PLANNING
        // Single-node DOUBLES planner
        std::unique_ptr<planning::moveJ> movej_planner_;

        // Multi-node DOUBLES planner
        std::unique_ptr<planning::SmoothCurveOfMultiJointsUsingBlending> multi_node_planner_;
#endif
    };
} // namespace arms_controller_common

