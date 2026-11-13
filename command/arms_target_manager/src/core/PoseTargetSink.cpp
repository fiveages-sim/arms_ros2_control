//
// PoseTargetSink 实现：发送、显示、回显抑制、交接过渡、冻结追赶。
//
// 坐标系换算是外部注入的，因此本文件不直接依赖 TF，便于测试。
//
#include "arms_target_manager/core/PoseTargetSink.h"

#include <algorithm>
#include <utility>

namespace arms_ros2_control::command
{
    namespace
    {
        geometry_msgs::msg::Pose blend(const geometry_msgs::msg::Pose& from,
                                       const geometry_msgs::msg::Pose& to,
                                       double t)
        {
            geometry_msgs::msg::Pose out;
            out.position.x = from.position.x + (to.position.x - from.position.x) * t;
            out.position.y = from.position.y + (to.position.y - from.position.y) * t;
            out.position.z = from.position.z + (to.position.z - from.position.z) * t;

            Eigen::Quaterniond a(from.orientation.w, from.orientation.x,
                                 from.orientation.y, from.orientation.z);
            Eigen::Quaterniond b(to.orientation.w, to.orientation.x,
                                 to.orientation.y, to.orientation.z);
            if (a.norm() < 1e-9)
            {
                a.setIdentity();
            }
            if (b.norm() < 1e-9)
            {
                b.setIdentity();
            }
            const Eigen::Quaterniond q =
                a.normalized().slerp(std::clamp(t, 0.0, 1.0), b.normalized()).normalized();
            out.orientation.x = q.x();
            out.orientation.y = q.y();
            out.orientation.z = q.z();
            out.orientation.w = q.w();
            return out;
        }

        void toEigen(const geometry_msgs::msg::Pose& pose,
                     Eigen::Vector3d& position, Eigen::Quaterniond& orientation)
        {
            position = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
            orientation = Eigen::Quaterniond(
                pose.orientation.w, pose.orientation.x,
                pose.orientation.y, pose.orientation.z);
        }

        void fromEigen(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                       geometry_msgs::msg::Pose& pose)
        {
            pose.position.x = position.x();
            pose.position.y = position.y();
            pose.position.z = position.z();
            pose.orientation.x = orientation.x();
            pose.orientation.y = orientation.y();
            pose.orientation.z = orientation.z();
            pose.orientation.w = orientation.w();
        }
    } // namespace

    PoseTargetSink::PoseTargetSink(std::string control_frame, std::string marker_frame)
        : control_frame_(std::move(control_frame))
          , marker_frame_(std::move(marker_frame))
    {
    }

    void PoseTargetSink::bind(
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub,
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr stamped_pub,
        TransformFn transform, rclcpp::Node* node)
    {
        pose_pub_ = std::move(pose_pub);
        stamped_pub_ = std::move(stamped_pub);
        transform_ = std::move(transform);
        node_ = node;
    }

    bool PoseTargetSink::publish(
        const geometry_msgs::msg::Pose& control_pose, const Options& options)
    {
        if (!hasPublisher())
        {
            return false;
        }

        geometry_msgs::msg::Pose out = control_pose;
        if (options.filter_stale)
        {
            Eigen::Vector3d position;
            Eigen::Quaterniond orientation;
            toEigen(out, position, orientation);
            stale_.apply(position, orientation);
            fromEigen(position, orientation, out);
        }

        const auto now = std::chrono::steady_clock::now();
        if (has_handover_)
        {
            const double duration = std::chrono::duration<double>(handover_duration_).count();
            const double elapsed =
                std::chrono::duration<double>(now - handover_start_).count();
            const double t = duration > 1e-6 ? std::clamp(elapsed / duration, 0.0, 1.0) : 1.0;
            if (t >= 1.0)
            {
                has_handover_ = false;
            }
            else
            {
                out = blend(handover_from_, out, t);
            }
        }

        if (options.stamped)
        {
            if (!stamped_pub_)
            {
                return false;
            }
            geometry_msgs::msg::PoseStamped msg;
            msg.header.stamp = node_ ? node_->now() : rclcpp::Clock().now();
            msg.header.frame_id = control_frame_;
            msg.pose = out;
            stamped_pub_->publish(msg);
        }
        else
        {
            pose_pub_->publish(out);
        }

        last_published_ = out;
        notifyView(out);
        return true;
    }

    bool PoseTargetSink::republish(const Options& options)
    {
        if (!last_command_)
        {
            return false;
        }
        return publish(*last_command_, options);
    }

    void PoseTargetSink::recordCommand(const geometry_msgs::msg::Pose& control_pose)
    {
        last_command_ = control_pose;
        notifyView(control_pose);
    }

    void PoseTargetSink::startHandover(std::chrono::milliseconds duration)
    {
        if (duration.count() <= 0 || !last_published_)
        {
            has_handover_ = false;
            return;
        }
        handover_from_ = *last_published_;
        handover_start_ = std::chrono::steady_clock::now();
        handover_duration_ = duration;
        has_handover_ = true;
    }

    void PoseTargetSink::holdEcho(std::chrono::milliseconds duration)
    {
        if (duration.count() > 0)
        {
            echo_hold_until_ = std::chrono::steady_clock::now() + duration;
        }
    }

    void PoseTargetSink::onReported(const geometry_msgs::msg::PoseStamped& msg)
    {
        last_reported_ = msg;
        if (std::chrono::steady_clock::now() < echo_hold_until_)
        {
            return;
        }
        if (!view_ || !transform_)
        {
            return;
        }
        geometry_msgs::msg::Pose marker_pose;
        if (!transform_(msg.pose, msg.header.frame_id, marker_frame_, marker_pose))
        {
            return;
        }
        view_(marker_pose);
    }

    bool PoseTargetSink::refreshViewFromLastCommand()
    {
        if (!last_command_)
        {
            return false;
        }
        notifyView(*last_command_);
        return true;
    }

    bool PoseTargetSink::refreshViewFromReported()
    {
        if (!last_reported_ || !view_ || !transform_)
        {
            return false;
        }
        geometry_msgs::msg::Pose marker_pose;
        if (!transform_(last_reported_->pose, last_reported_->header.frame_id,
                        marker_frame_, marker_pose))
        {
            return false;
        }
        view_(marker_pose);
        return true;
    }

    void PoseTargetSink::invalidate()
    {
        last_command_.reset();
        last_published_.reset();
        last_reported_.reset();
        has_handover_ = false;
        echo_hold_until_ = std::chrono::steady_clock::time_point{};
        stale_.reset();
    }

    void PoseTargetSink::notifyView(const geometry_msgs::msg::Pose& control_pose)
    {
        if (!view_ || !transform_)
        {
            return;
        }
        geometry_msgs::msg::Pose marker_pose;
        if (!transform_(control_pose, control_frame_, marker_frame_, marker_pose))
        {
            return;
        }
        view_(marker_pose);
    }

} // namespace arms_ros2_control::command
