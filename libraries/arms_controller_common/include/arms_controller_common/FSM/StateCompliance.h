//
// Common StateCompliance for Arm Controllers
//
// Force/position hybrid compliance control state.
// Entered from HOLD via fsm_command=5; returns to HOLD on fsm_command=2.
//
#pragma once

#include "arms_controller_common/FSM/FSMState.h"
#include "arms_controller_common/utils/GravityCompensation.h"
#include "arms_controller_common/utils/Kinematics.h"
#include <vector>
#include <memory>
#include <mutex>
#include <array>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <Eigen/Dense>

namespace arms_controller_common
{
    /**
     * @brief StateCompliance - Compliance / force-position hybrid control
     *
     * Behavior (per cycle):
     *  1. Position baseline: hold the joint positions captured on enter.
     *  2. (Optional) Admittance drift: integrate the external wrench through a
     *     Cartesian mass-damper law to compute a Cartesian offset dx, map it to
     *     joints via damped-least-squares inverse Jacobian, and add it to the
     *     held position so the arm "yields" along the direction of the applied
     *     force (compliant behavior even on pure position interface hardware).
     *  3. MIX mode: apply soft kp/kd (compliance_gains_) and gravity compensation.
     *  4. Force feed-forward: convert the Cartesian wrench (in end-effector frame)
     *     to the base frame using the current EE rotation, then apply
     *     τ_ff = J^T · W_base on the effort interface (only in MIX mode).
     *
     * Subclasses (per controller) only need to override checkChange() to map
     * fsm_command into the controller-specific next state.
     *
     * Coordinates:
     *  - Subscribed wrench is assumed to be expressed in the EE frame
     *    (frame_id == left_eef/right_eef). It is rotated into the base frame
     *    before being multiplied by J^T (which is in LOCAL_WORLD_ALIGNED /
     *    base frame).
     *  - When no kinematics solver is supplied, the wrench feed-forward and
     *    admittance drift are skipped (degrades gracefully to soft-hold).
     */
    class StateCompliance : public FSMState
    {
    public:
        explicit StateCompliance(CtrlInterfaces& ctrl_interfaces,
                                 std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node = nullptr,
                                 const std::shared_ptr<GravityCompensation>& gravity_compensation = nullptr,
                                 const std::shared_ptr<ArmKinematics>& kinematics = nullptr);

        void enter() override;
        void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        void exit() override;
        FSMStateName checkChange() override;

        /**
         * @brief Override the wrench topics. Empty disables that side's feed-in.
         * Must be called before enter() to take effect.
         */
        void setWrenchTopic(const std::string& left_topic, const std::string& right_topic);

    protected:
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        std::shared_ptr<GravityCompensation> gravity_compensation_;
        std::shared_ptr<ArmKinematics> kinematics_;

    private:
        void updateParam();
        void setupWrenchSubscriptions();

        // ---- helpers ----
        // Returns true if a kinematics solver is configured and joint layout is dual-arm.
        bool kinematicsAvailable() const;

        // Split the controller joint vector into left/right Eigen vectors.
        void splitJoints(const std::vector<double>& all,
                         Eigen::VectorXd& left, Eigen::VectorXd& right) const;

        // Rotate a 6D wrench from EE frame to base frame using current EE rotation.
        Eigen::Matrix<double, 6, 1> wrenchToBase(int side_index,
                                                  const std::array<double, 6>& wrench_ee) const;

        // Compute J^T·W feed-forward joint torques for one side.
        Eigen::VectorXd computeForceFeedforward(int side_index,
                                                 const Eigen::Matrix<double, 6, 1>& wrench_base);

        // Admittance: integrate Cartesian wrench -> Cartesian pose offset -> joint offset.
        Eigen::VectorXd computeAdmittanceOffset(int side_index,
                                                 const Eigen::Matrix<double, 6, 1>& wrench_base,
                                                 double dt);

        // Members
        std::vector<double> hold_positions_;  // Joint positions captured on enter (baseline)

        // Compliance gains [kp, kd]; lower than hold/default gains for soft behavior
        std::vector<double> compliance_gains_;

        // Force feed-forward scale (0 disables; 1.0 = raw J^T·W)
        double force_feedforward_scale_{1.0};

        // Admittance parameters (Cartesian mass-damper: M·ẍ + D·ẋ = W_ext)
        bool admittance_enabled_{false};
        std::vector<double> admittance_mass_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};     // [m_x, m_y, m_z, j_x, j_y, j_z]
        std::vector<double> admittance_damping_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double admittance_max_displacement_{0.05};  // m / rad cap per DOF
        // Per-side admittance state (linear/angular velocity in base frame)
        Eigen::Matrix<double, 6, 1> adm_state_left_{Eigen::Matrix<double, 6, 1>::Zero()};
        Eigen::Matrix<double, 6, 1> adm_state_right_{Eigen::Matrix<double, 6, 1>::Zero()};
        rclcpp::Time last_admittance_time_;

        // Optional external wrench feed-in
        std::string left_wrench_topic_{"/left_arm_external_wrench"};
        std::string right_wrench_topic_{"/right_arm_external_wrench"};
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr left_wrench_sub_;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr right_wrench_sub_;
        std::mutex wrench_mutex_;
        std::array<double, 6> left_wrench_{};
        std::array<double, 6> right_wrench_{};

        // Per-side joint count (derived from kinematics when available, else 0)
        size_t left_joint_count_{0};
        size_t right_joint_count_{0};
    };
} // namespace arms_controller_common
