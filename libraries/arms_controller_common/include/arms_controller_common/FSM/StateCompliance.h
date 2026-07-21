//
// Common StateCompliance for Arm Controllers
//
// Force outer-loop / stiff position inner-loop.
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
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>

namespace arms_controller_common
{
    /**
     * Pipeline (always on after enter):
     *   FT_raw (sensor) − tool_gravity(world→sensor via TF) − zero_cal bias
     *   → world/base → admittance → stiff position command.
     *
     * Tool model: left/right_dyn_param = [m, mx,my,mz(mm in tcp), I...]
     * Gravity direction: world −Z (fallback base_link if world TF missing).
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

    protected:
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        std::shared_ptr<GravityCompensation> gravity_compensation_;
        std::shared_ptr<ArmKinematics> kinematics_;

    private:
        void updateParam();
        void setupWrenchSubscriptions();
        bool kinematicsAvailable() const;
        void splitJoints(const std::vector<double>& all,
                         Eigen::VectorXd& left, Eigen::VectorXd& right) const;

        /** Tool gravity wrench in FT sensor frame: F = R^T (−mgẑ_world), τ = r_com×F. */
        bool computeToolGravityWrenchInSensorFrame(
            int side_index, Eigen::Matrix<double, 6, 1>& wrench_sensor) const;

        Eigen::Matrix<double, 6, 1> wrenchToBase(int side_index,
                                                  const std::array<double, 6>& wrench_ee) const;
        /** Integrate Mẍ+Dẋ=F with accumulated x; map q_offset = J⁺ x. Skip if no FT. */
        Eigen::VectorXd computeAdmittanceOffset(int side_index,
                                                 const Eigen::Matrix<double, 6, 1>& wrench_base,
                                                 double dt);

        std::vector<double> hold_positions_;

        // Heavier M/D + deadband: FT noise must not chatter stiff position inner-loop.
        std::vector<double> admittance_mass_{5.0, 5.0, 5.0, 0.5, 0.5, 0.5};
        std::vector<double> admittance_damping_{80.0, 80.0, 80.0, 8.0, 8.0, 8.0};
        double admittance_max_displacement_{0.05};  // total Cartesian |x| cap [m]/rad]
        double force_deadband_{2.0};                // N
        double torque_deadband_{0.15};              // Nm
        double wrench_lpf_alpha_{0.15};             // EMA on tared wrench (0..1)
        Eigen::Matrix<double, 6, 1> adm_vel_left_{Eigen::Matrix<double, 6, 1>::Zero()};
        Eigen::Matrix<double, 6, 1> adm_vel_right_{Eigen::Matrix<double, 6, 1>::Zero()};
        Eigen::Matrix<double, 6, 1> adm_pos_left_{Eigen::Matrix<double, 6, 1>::Zero()};
        Eigen::Matrix<double, 6, 1> adm_pos_right_{Eigen::Matrix<double, 6, 1>::Zero()};
        Eigen::Matrix<double, 6, 1> wrench_filt_left_{Eigen::Matrix<double, 6, 1>::Zero()};
        Eigen::Matrix<double, 6, 1> wrench_filt_right_{Eigen::Matrix<double, 6, 1>::Zero()};

        // Zero-cal (sensor-frame residual after gravity removal)
        double zero_cal_duration_{1.0};
        bool zero_cal_pending_{false};
        bool zero_cal_running_{false};
        bool zero_cal_done_{false};
        rclcpp::Time zero_cal_start_;
        long zero_cal_samples_left_{0};
        long zero_cal_samples_right_{0};
        std::array<double, 6> wrench_bias_left_{};
        std::array<double, 6> wrench_bias_right_{};
        std::array<double, 6> zero_cal_sum_left_{};
        std::array<double, 6> zero_cal_sum_right_{};

        // Tool load [m, mx,my,mz mm in tcp, ...]
        std::vector<double> left_dyn_param_;
        std::vector<double> right_dyn_param_;
        double gravity_accel_{9.81};

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::string gravity_frame_{"world"};  // vertical reference; fallback base_link
        static constexpr const char* kLeftFtFrame = "left_ft_sensor_link";
        static constexpr const char* kRightFtFrame = "right_ft_sensor_link";
        static constexpr const char* kLeftTcpFrame = "left_tcp";
        static constexpr const char* kRightTcpFrame = "right_tcp";
        static constexpr const char* kLeftWrenchTopic = "/left_ft_broadcaster/wrench";
        static constexpr const char* kRightWrenchTopic = "/right_ft_broadcaster/wrench";

        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr left_wrench_sub_;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr right_wrench_sub_;
        std::mutex wrench_mutex_;
        std::array<double, 6> left_wrench_{};
        std::array<double, 6> right_wrench_{};
        bool left_ft_active_{false};
        bool right_ft_active_{false};

        size_t left_joint_count_{0};
        size_t right_joint_count_{0};
    };
} // namespace arms_controller_common
