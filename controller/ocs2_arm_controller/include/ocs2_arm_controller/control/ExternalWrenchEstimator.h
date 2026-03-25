//
// τ-based external wrench estimation (joint effort − gravity RNEA + damped J^T inverse).
//
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <ocs2_core/Types.h>

#include <arms_controller_common/CtrlInterfaces.h>

namespace ocs2::mobile_manipulator
{
    class MobileManipulatorInterface;
    /**
     * Publishes geometry_msgs/WrenchStamped from measured joint torque vs. model gravity torque.
     * Single topic or left/right topics depending on dual_arm_mode at setupPublishers().
     */
    class ExternalWrenchEstimator
    {
    public:
        ExternalWrenchEstimator(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
                                bool enabled,
                                double damping,
                                double filter_alpha,
                                std::string wrench_frame_id);

        void setupPublishers(const std::string& robot_name, bool dual_arm_mode);

        /** Requires joint effort state interfaces; no-op if disabled or layout invalid. */
        void publish(const rclcpp::Time& time,
                     const vector_t& joint_positions,
                     arms_controller_common::CtrlInterfaces& ctrl,
                     const std::shared_ptr<MobileManipulatorInterface>& interface,
                     const std::vector<std::string>& joint_names) const;

    private:
        static Eigen::Matrix<double, 6, 1> dampedWrenchFromTorque(
            const Eigen::Ref<const Eigen::MatrixXd>& J,
            const Eigen::Ref<const Eigen::VectorXd>& tau_segment,
            double lambda);

        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        bool enabled_;
        double damping_;
        double filter_alpha_;
        std::string wrench_frame_id_;

        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_single_;
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_left_;
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_right_;

        mutable Eigen::VectorXd filtered_left_;
        mutable Eigen::VectorXd filtered_right_;
        mutable Eigen::VectorXd filtered_single_;
        mutable bool warned_no_effort_{false};
    };
} // namespace ocs2::mobile_manipulator
