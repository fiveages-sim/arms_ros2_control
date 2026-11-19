//
// base_pose_sim 节点实现。
//
// 角色：cmd_vel → 平面位姿 → TF（纯积分，无轮系解算）。
// 输入：cmd_vel、ROS 参数。
// 输出：/tf、可选 /tf_static 与 /odom。
// 改动注意：
//   * 本节点只发 base 自己那条边，不碰 base→link 的固定关节（那是
//     robot_state_publisher 的职责），重复发布会造成同一父子对双发布者。
//   * 与真实位姿源（物理仿真、定位）不能同时存在：两者都发同一父子对时
//     TF 会来回跳。启动前先确认没有别的东西在发这条变换。
//   * 时钟跳变（重置仿真时间）时 dt 会被钳住，否则一帧积分出巨大位移。
//
#include "base_pose_sim/base_pose_sim_node.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

namespace base_pose_sim
{
    namespace
    {
        // 差速模型下允许的横向速度残差：超过即认为模型配错或上游算错了。
        constexpr double kModelMismatchEps = 1e-3;
        // 单步积分的时间上限：时钟跳变或进程卡顿时防止位姿瞬移。
        constexpr double kMaxStepSec = 0.5;
        // 定时器首帧没有历史 dt 时使用的缺省值。
        constexpr double kDefaultStepSec = 0.02;
        // 限幅后仍小于该值即视为零速，避免在浮点噪声上做无谓的三角函数。
        constexpr double kStopEps = 1e-9;
    }  // namespace

    BasePoseSimNode::BasePoseSimNode()
        : rclcpp::Node("base_pose_sim")
    {
        cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
        world_frame_ = declare_parameter<std::string>("world_frame", "world");
        odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
        base_z_ = declare_parameter<double>("base_z", 0.0);
        publish_rate_ = declare_parameter<double>("publish_rate", 50.0);
        cmd_timeout_sec_ = declare_parameter<double>("cmd_timeout", 0.25);
        max_linear_velocity_ = declare_parameter<double>("max_linear_velocity", 0.0);
        max_angular_velocity_ = declare_parameter<double>("max_angular_velocity", 0.0);
        publish_odom_ = declare_parameter<bool>("publish_odom", false);

        // 运动模型：具体档位由底盘实际自由度决定，非法值直接报错而不是静默降级，
        // 否则"配了差速却在收 vy"这类问题会一直没有迹象。
        const std::string model = declare_parameter<std::string>("base_model", "omni");
        if (model == "omni")
        {
            model_ = BaseModel::kOmni;
        }
        else if (model == "differential")
        {
            model_ = BaseModel::kDifferential;
        }
        else
        {
            RCLCPP_ERROR(get_logger(),
                         "未知 base_model='%s'（可选 omni / differential），回退为 omni",
                         model.c_str());
            model_ = BaseModel::kOmni;
        }

        // 初始位姿：允许仿真把机器人摆在非原点位置时仍与场景对齐。
        origin_x_ = declare_parameter<double>("initial_x", 0.0);
        origin_y_ = declare_parameter<double>("initial_y", 0.0);
        origin_yaw_ = declare_parameter<double>("initial_yaw", 0.0);
        x_ = origin_x_;
        y_ = origin_y_;
        yaw_ = origin_yaw_;

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        if (publish_odom_)
        {
            // world→odom 锁存在启动位姿上：odom 原点即"出生点"。
            // 该变换静态发布，之后只要维护 odom→base，/odom 才不会白送一个初始偏移。
            static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
            static_broadcaster_->sendTransform(
                makeTransform(world_frame_, odom_frame_, now(), origin_x_, origin_y_, origin_yaw_));
            odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        }

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic_, 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) { onCmdVel(msg); });

        const double period = 1.0 / std::max(publish_rate_, 1.0);
        timer_ = create_wall_timer(
            std::chrono::duration<double>(period), [this]() { onTick(); });

        RCLCPP_INFO(get_logger(),
                    "base_pose_sim: %s→%s（%s %s, %.1f Hz，超时 %.2f s%s）",
                    world_frame_.c_str(),
                    publish_odom_ ? odom_frame_.c_str() : base_frame_.c_str(),
                    cmd_vel_topic_.c_str(),
                    model_ == BaseModel::kOmni ? "omni[vx,vy,ω]" : "diff[vx,ω]",
                    publish_rate_,
                    cmd_timeout_sec_,
                    publish_odom_ ? "，另发 odom→base 与 /odom" : "");
    }

    void BasePoseSimNode::onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        target_vx_ = msg->linear.x;
        target_vy_ = msg->linear.y;
        target_wz_ = msg->angular.z;
        has_command_ = true;
        last_command_time_ = now();
    }

    bool BasePoseSimNode::commandIsFresh() const
    {
        if (!has_command_)
        {
            return false;
        }
        if (cmd_timeout_sec_ <= 0.0)
        {
            return true;
        }
        const double age = (now() - last_command_time_).seconds();
        return age <= cmd_timeout_sec_;
    }

    void BasePoseSimNode::resolveVelocity(double& vx, double& vy, double& wz) const
    {
        vx = target_vx_;
        vy = target_vy_;
        wz = target_wz_;

        if (!commandIsFresh())
        {
            // 指令停发后必须归零：订阅端会一直保留最后一帧消息，
            // 不主动清零的话底盘会按旧指令无限走下去。
            vx = 0.0;
            vy = 0.0;
            wz = 0.0;
            return;
        }

        if (model_ == BaseModel::kDifferential && std::abs(vy) > kModelMismatchEps)
        {
            // 差速底盘物理上做不出横向速度。静默丢弃会让人以为"配了也生效"，
            // 所以留一条限频告警指向真正的问题（模型选错或上游指令有误）。
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "base_model=differential 但收到 linear.y=%.3f，已忽略横向分量",
                                 vy);
            vy = 0.0;
        }

        if (max_linear_velocity_ > 0.0)
        {
            const double speed = std::hypot(vx, vy);
            if (speed > max_linear_velocity_)
            {
                // 按比例缩放而不是只砍 vx：保持运动方向不变，避免"被限速后跑偏"。
                const double scale = max_linear_velocity_ / speed;
                vx *= scale;
                vy *= scale;
            }
        }

        if (max_angular_velocity_ > 0.0)
        {
            wz = std::clamp(wz, -max_angular_velocity_, max_angular_velocity_);
        }
    }

    void BasePoseSimNode::onTick()
    {
        const rclcpp::Time tick_time = now();

        double dt = kDefaultStepSec;
        if (has_tick_)
        {
            dt = (tick_time - last_tick_time_).seconds();
            if (!std::isfinite(dt) || dt <= 0.0)
            {
                dt = kDefaultStepSec;
            }
            dt = std::min(dt, kMaxStepSec);
        }
        has_tick_ = true;
        last_tick_time_ = tick_time;

        double vx = 0.0;
        double vy = 0.0;
        double wz = 0.0;
        resolveVelocity(vx, vy, wz);

        if (std::abs(vx) > kStopEps || std::abs(vy) > kStopEps || std::abs(wz) > kStopEps)
        {
            // 机体系速度 → 世界系位移。vy 项在差速模型下恒为 0，
            // 因此 omni 与 differential 共用同一份积分，不需要分支。
            const double cos_yaw = std::cos(yaw_);
            const double sin_yaw = std::sin(yaw_);
            x_ += (vx * cos_yaw - vy * sin_yaw) * dt;
            y_ += (vx * sin_yaw + vy * cos_yaw) * dt;
            yaw_ = wrapToPi(yaw_ + wz * dt);
        }

        publish();
    }

    void BasePoseSimNode::publish()
    {
        const rclcpp::Time stamp = now();

        if (!publish_odom_)
        {
            tf_broadcaster_->sendTransform(
                makeTransform(world_frame_, base_frame_, stamp, x_, y_, yaw_));
            return;
        }

        // odom→base = 启动位姿的逆 复合 当前位姿。
        // 两级复合回来恰好等于 world→base，所以 /odom 与 TF 永远自洽。
        const double dx = x_ - origin_x_;
        const double dy = y_ - origin_y_;
        const double cos_origin = std::cos(origin_yaw_);
        const double sin_origin = std::sin(origin_yaw_);
        const double odom_x = cos_origin * dx + sin_origin * dy;
        const double odom_y = -sin_origin * dx + cos_origin * dy;
        const double odom_yaw = wrapToPi(yaw_ - origin_yaw_);

        tf_broadcaster_->sendTransform(
            makeTransform(odom_frame_, base_frame_, stamp, odom_x, odom_y, odom_yaw));

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;
        odom.pose.pose.position.x = odom_x;
        odom.pose.pose.position.y = odom_y;
        odom.pose.pose.position.z = base_z_;
        odom.pose.pose.orientation = yawToQuaternion(odom_yaw);
        odom_pub_->publish(odom);
    }

    double BasePoseSimNode::wrapToPi(double angle)
    {
        double wrapped = std::fmod(angle + M_PI, 2.0 * M_PI);
        if (wrapped < 0.0)
        {
            wrapped += 2.0 * M_PI;
        }
        return wrapped - M_PI;
    }

    geometry_msgs::msg::Quaternion BasePoseSimNode::yawToQuaternion(double yaw)
    {
        geometry_msgs::msg::Quaternion q;
        q.x = 0.0;
        q.y = 0.0;
        q.z = std::sin(yaw * 0.5);
        q.w = std::cos(yaw * 0.5);
        return q;
    }

    geometry_msgs::msg::TransformStamped BasePoseSimNode::makeTransform(
        const std::string& parent,
        const std::string& child,
        const rclcpp::Time& stamp,
        double x,
        double y,
        double yaw) const
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = stamp;
        tf.header.frame_id = parent;
        tf.child_frame_id = child;
        tf.transform.translation.x = x;
        tf.transform.translation.y = y;
        tf.transform.translation.z = base_z_;
        tf.transform.rotation = yawToQuaternion(yaw);
        return tf;
    }
}  // namespace base_pose_sim

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<base_pose_sim::BasePoseSimNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
