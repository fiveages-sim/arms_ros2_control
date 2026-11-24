//
// PoseTargetSink 实现：发送、显示、回显抑制、交接过渡、冻结追赶。
//
// 关键点：
//   * 发布前依次做：冻结追赶快照 → 交接过渡插值 → 发消息 → 刷新显示；
//   * 回显抑制期内只记录不显示，避免手动拖动时 marker 被拉回；
//   * invalidate() 会清掉全部缓存（含冻结追赶状态）。
//
// 注意：
//   坐标系换算是外部注入的，因此本文件不直接依赖 TF，便于测试。
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

    /**
     * @brief 绑定发布器与坐标变换器
     * @param pose_pub 连续目标发布器
     * @param stamped_pub 单次（带时间戳）目标发布器
     * @param transform 坐标系换算函数，用于把目标换算回 marker 显示系
     * @param node 节点指针，只用于取时间
     */
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

    /**
     * @brief 发布一个目标位姿
     *
     * 处理顺序：冻结追赶（可选）→ 交接平滑 → 发布 → 刷新显示。
     * 目标始终以控制坐标系传入，发布时也不再换算（下游按同一基准系解释）。
     *
     * @param control_pose 控制坐标系下的目标位姿
     * @param options 本次发布选项（是否带时间戳、是否过滤冻结、是否手动来源等）
     * @return false 表示未绑定发布器，或本次未发出
     */
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

    /**
     * @brief 把上一次缓存的命令目标重新发一次
     * @param options 发布选项
     * @return false 表示还没有任何命令目标
     */
    bool PoseTargetSink::republish(const Options& options)
    {
        if (!last_command_)
        {
            return false;
        }
        return publish(*last_command_, options);
    }

    /**
     * @brief 记录本次命令目标（不一定发布）
     *
     * 单次模式下拖动只更新这里，菜单“发送目标”时才真正发出去。
     */
    void PoseTargetSink::recordCommand(const geometry_msgs::msg::Pose& control_pose)
    {
        last_command_ = control_pose;
        notifyView(control_pose);
    }

    /**
     * @brief 启动一次交接过渡
     *
     * 以“上一次实际发出的位置”为起点，在 duration 内渐变到新目标，
     * 避免手动拖动交还给自动控制时出现位置跳变。
     */
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

    /**
     * @brief 在接下来一段时间内不接受控制器回传值刷新显示
     *
     * 手动拖动期间控制器回传的是旧目标，不抑制会把 marker 拉回去。
     */
    void PoseTargetSink::holdEcho(std::chrono::milliseconds duration)
    {
        if (duration.count() > 0)
        {
            echo_hold_until_ = std::chrono::steady_clock::now() + duration;
        }
    }

    /**
     * @brief 收到控制器回传的实际目标
     *
     * 始终记录，但在回显抑制期内不刷新显示。
     */
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

    /**
     * @brief 清空全部缓存（命令/发布/回显/交接/冻结追赶）
     *
     * 坐标系或控制拓扑变化时调用，避免拿旧值当锚点。
     */
    void PoseTargetSink::invalidate()
    {
        last_command_.reset();
        last_published_.reset();
        last_reported_.reset();
        has_handover_ = false;
        echo_hold_until_ = std::chrono::steady_clock::time_point{};
        stale_.reset();
    }

    /**
     * @brief 把控制系位姿换算到显示系后交给界面回调
     */
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
