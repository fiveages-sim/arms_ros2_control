//
// base_pose_sim 节点声明。
//
// 角色：把底盘速度指令积分成平面位姿，向 TF 树补一条 world→base
//      （或 world→odom + odom→base）变换。仅在缺少位姿来源的仿真环境里使用。
// 输入：cmd_vel（机体系速度）与 ROS 参数。
// 输出：/tf、可选 /tf_static 与 /odom。
// 改动注意：这里刻意不做轮系解算（转向角、轮速、打滑），只做刚体运动学积分；
//      把它当"能动的可视占位"，不要用它的轮速/摩擦行为判断控制效果。
//
#pragma once

#include <chrono>
#include <string>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace base_pose_sim
{
    /**
     * @brief 由 cmd_vel 积分出底盘平面位姿并广播 TF。
     *
     * 支持两种运动模型，二者共用同一份积分公式：
     *   - omni：消费 vx、vy、ω（全向轮、舵轮、麦克纳姆等可横移底盘）；
     *   - differential：只用 vx、ω（差速、履带等），vy 被强制忽略。
     * 差速模型下若收到非零 vy，会限频告警，用来暴露"参数配错"而不是静默吞掉。
     *
     * odom 语义：启动瞬间的位姿即 odom 原点，world→odom 用静态变换锁存，
     * 之后只发 odom→base（与常见仿真器的 latch 行为一致），这样 /odom 从零开始。
     */
    class BasePoseSimNode : public rclcpp::Node
    {
    public:
        BasePoseSimNode();

    private:
        /** 底盘可用的运动自由度。 */
        enum class BaseModel
        {
            kOmni,          ///< vx + vy + ω
            kDifferential,  ///< vx + ω（vy 恒为 0）
        };

        /** @brief 缓存最新速度指令并刷新到达时间。 */
        void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);

        /** @brief 定时积分一步，然后发布位姿。 */
        void onTick();

        /** @brief 按当前模型发布 TF（与可选的 /odom）。 */
        void publish();

        /** @brief 速度指令是否仍在有效期内；超时视为零速。 */
        [[nodiscard]] bool commandIsFresh() const;

        /** @brief 限幅后的机体系速度，已按运动模型裁剪 vy。 */
        void resolveVelocity(double& vx, double& vy, double& wz) const;

        /** @brief 把角度折算到 (-π, π]；避免 yaw 累积成大数后精度下降。 */
        static double wrapToPi(double angle);

        /** @brief 只有偏航的旋转四元数。 */
        static geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);

        /** @brief 组装一条 TransformStamped；z 取 base_z_。 */
        geometry_msgs::msg::TransformStamped makeTransform(const std::string& parent,
                                                          const std::string& child,
                                                          const rclcpp::Time& stamp,
                                                          double x,
                                                          double y,
                                                          double yaw) const;

        // ---- 参数 ----
        std::string cmd_vel_topic_;
        std::string world_frame_;
        std::string odom_frame_;
        std::string base_frame_;
        BaseModel model_{BaseModel::kOmni};
        double base_z_{0.0};
        double publish_rate_{50.0};
        double cmd_timeout_sec_{0.25};
        double max_linear_velocity_{0.0};
        double max_angular_velocity_{0.0};
        bool publish_odom_{false};

        // ---- 运行状态 ----
        // 积分状态：world 系下的底盘位姿。
        double x_{0.0};
        double y_{0.0};
        double yaw_{0.0};
        // odom 原点锁存在启动位姿上，据此把世界位姿换算成 odom 位姿。
        double origin_x_{0.0};
        double origin_y_{0.0};
        double origin_yaw_{0.0};

        double target_vx_{0.0};
        double target_vy_{0.0};
        double target_wz_{0.0};
        bool has_command_{false};
        rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
        rclcpp::Time last_tick_time_{0, 0, RCL_ROS_TIME};
        bool has_tick_{false};

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    };
}  // namespace base_pose_sim
