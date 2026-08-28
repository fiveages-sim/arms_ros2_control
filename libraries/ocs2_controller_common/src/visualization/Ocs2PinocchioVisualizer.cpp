//
// Created by fiveages on 8/21/25.
//

#include "ocs2_controller_common/visualization/Ocs2PinocchioVisualizer.h"
#include <fstream>
#include <limits>
#include <string>
#include <algorithm>
#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <std_msgs/msg/string.hpp>

namespace ocs2::controller_common
{
    Ocs2PinocchioVisualizer::Ocs2PinocchioVisualizer(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                                                     PinocchioInterface pinocchio_interface, Ocs2VisualizerConfig config)
        : node_(node),
          pinocchio_interface_(std::move(pinocchio_interface)),
          config_(std::move(config)),
          dual_arm_mode_(config_.dual_arm),
          base_frame_(config_.base_frame),
          urdf_file_(config_.urdf_file),
          left_arm_color_({0.0, 0.4470, 0.7410}),
          right_arm_color_({0.6350, 0.0780, 0.1840}),
          rt_data_(pinocchio_interface_.getModel()),
          viz_data_(pinocchio_interface_.getModel())
    {
        const auto& model = pinocchio_interface_.getModel();
        left_ee_frame_id_ = model.getFrameId(config_.left_ee_frame);
        left_ee_frame_valid_ = left_ee_frame_id_ < static_cast<pinocchio::FrameIndex>(model.nframes);
        if (dual_arm_mode_)
        {
            right_ee_frame_id_ = model.getFrameId(config_.right_ee_frame);
            right_ee_frame_valid_ = right_ee_frame_id_ < static_cast<pinocchio::FrameIndex>(model.nframes);
        }
        if (!config_.body_frame.empty())
        {
            body_frame_id_ = model.getFrameId(config_.body_frame);
            body_frame_valid_ = body_frame_id_ < static_cast<pinocchio::FrameIndex>(model.nframes);
        }
    }

    void Ocs2PinocchioVisualizer::initialize()
    {
        // Create publisher
        trajectory_marker_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            "trajectory_markers", 1);
        
        // Create end effector pose publisher
        left_end_effector_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            "left_current_pose", 1);
        
        // Only create right arm publisher in dual arm mode
        if (dual_arm_mode_)
        {
            right_end_effector_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
                "right_current_pose", 1);
        }

        body_frame_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "body_current_pose", 1);
        
        // Create robot description publisher and publish URDF
        if (!urdf_file_.empty())
        {
            robot_description_publisher_ = node_->create_publisher<std_msgs::msg::String>(
                "/ocs2_robot_description", rclcpp::QoS(1).transient_local());
            
            // Read and publish URDF file (ifstream does not throw on open failure by default).
            std::ifstream urdf_stream(urdf_file_);
            if (urdf_stream.is_open())
            {
                std::string urdf_content((std::istreambuf_iterator(urdf_stream)),
                                        std::istreambuf_iterator<char>());
                urdf_stream.close();

                std_msgs::msg::String urdf_msg;
                urdf_msg.data = urdf_content;
                robot_description_publisher_->publish(urdf_msg);

                RCLCPP_INFO(node_->get_logger(), "Published OCS2 robot description from: %s", urdf_file_.c_str());
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "Failed to open URDF file: %s", urdf_file_.c_str());
            }
        }

        try
        {
            auto geometry_opt = std::move(config_.pinocchio_geometry);
            if (geometry_opt)
            {
                const scalar_t activationDistance = config_.self_collision_activation_distance;
                geometry_visualization_ = std::make_unique<GeometryInterfaceVisualization>(
                    pinocchio_interface_, std::move(*geometry_opt), base_frame_, activationDistance);
                RCLCPP_INFO(node_->get_logger(),
                            "Self-collision visualization initialized (activation distance: %.3f m)", activationDistance);
            }
            else
            {
                RCLCPP_INFO(node_->get_logger(),
                            "No Pinocchio geometry provided, self-collision visualization disabled");
            }
        }
        catch (const std::exception& e)
        {
            geometry_visualization_.reset();
            RCLCPP_WARN(node_->get_logger(),
                        "Failed to initialize self-collision visualization: %s, self-collision visualization disabled",
                        e.what());
        }

        RCLCPP_INFO(node_->get_logger(), "Ocs2PinocchioVisualizer initialization completed");
        RCLCPP_INFO(node_->get_logger(), "Self-collision visualization: %s",
                    geometry_visualization_ ? "enabled" : "disabled");
    }

    void Ocs2PinocchioVisualizer::updateEndEffectorTrajectory(const PrimalSolution& policy)
    {
        updateEndEffectorTrajectoryFromStates(policy.stateTrajectory_);
    }

    void Ocs2PinocchioVisualizer::updateEndEffectorTrajectoryFromStates(const vector_array_t& stateTrajectory)
    {
        if (stateTrajectory.empty())
        {
            return;
        }

        if (dual_arm_mode_)
        {
            std::vector<vector_t> left_trajectory, right_trajectory;
            left_trajectory.reserve(stateTrajectory.size());
            right_trajectory.reserve(stateTrajectory.size());

            for (const auto& state : stateTrajectory)
            {
                forwardKinematicsInto(viz_data_, state);
                left_trajectory.push_back(extractFramePose7(viz_data_, left_ee_frame_id_));
                if (right_ee_frame_valid_)
                {
                    right_trajectory.push_back(extractFramePose7(viz_data_, right_ee_frame_id_));
                }
            }

            left_arm_trajectory_history_ = std::move(left_trajectory);
            right_arm_trajectory_history_ = std::move(right_trajectory);
        }
        else
        {
            std::vector<vector_t> trajectory;
            trajectory.reserve(stateTrajectory.size());

            for (const auto& state : stateTrajectory)
            {
                forwardKinematicsInto(viz_data_, state);
                trajectory.push_back(extractFramePose7(viz_data_, left_ee_frame_id_));
            }

            left_arm_trajectory_history_ = std::move(trajectory);
        }
    }


    void Ocs2PinocchioVisualizer::publishEndEffectorTrajectory(const rclcpp::Time& time)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        geometry_msgs::msg::PoseArray pose_array;

        pose_array.header.stamp = time;
        pose_array.header.frame_id = base_frame_;

        // Create left arm trajectory marker
        if (!left_arm_trajectory_history_.empty())
        {
            std::vector<geometry_msgs::msg::Point> left_trajectory_points;
            std::vector<geometry_msgs::msg::Pose> left_poses;

            for (const auto& pose : left_arm_trajectory_history_)
            {
                geometry_msgs::msg::Point point;
                point.x = pose(0);
                point.y = pose(1);
                point.z = pose(2);
                left_trajectory_points.push_back(point);

                geometry_msgs::msg::Pose pose_msg;
                pose_msg.position = point;
                pose_msg.orientation.w = pose(6);
                pose_msg.orientation.x = pose(3);
                pose_msg.orientation.y = pose(4);
                pose_msg.orientation.z = pose(5);
                left_poses.push_back(pose_msg);
            }

            // Create left arm trajectory line
            auto left_trajectory_marker = createTrajectoryLineMarker(
                left_trajectory_points, left_arm_color_, trajectory_line_width_, "Left Arm Trajectory");
            marker_array.markers.push_back(left_trajectory_marker);

            // Add to pose array
            pose_array.poses.insert(pose_array.poses.end(), left_poses.begin(), left_poses.end());
        }

        // Create right arm trajectory marker (if in dual arm mode)
        if (dual_arm_mode_ && !right_arm_trajectory_history_.empty())
        {
            std::vector<geometry_msgs::msg::Point> right_trajectory_points;
            std::vector<geometry_msgs::msg::Pose> right_poses;

            for (const auto& pose : right_arm_trajectory_history_)
            {
                geometry_msgs::msg::Point point;
                point.x = pose(0);
                point.y = pose(1);
                point.z = pose(2);
                right_trajectory_points.push_back(point);

                geometry_msgs::msg::Pose pose_msg;
                pose_msg.position = point;
                pose_msg.orientation.w = pose(6);
                pose_msg.orientation.x = pose(3);
                pose_msg.orientation.y = pose(4);
                pose_msg.orientation.z = pose(5);
                right_poses.push_back(pose_msg);
            }

            // Create right arm trajectory line
            auto right_trajectory_marker = createTrajectoryLineMarker(
                right_trajectory_points, right_arm_color_, trajectory_line_width_, "Right Arm Trajectory");
            marker_array.markers.push_back(right_trajectory_marker);

            // Add to pose array
            pose_array.poses.insert(pose_array.poses.end(), right_poses.begin(), right_poses.end());
        }

        // Set marker header information
        for (auto& marker : marker_array.markers)
        {
            marker.header.stamp = time;
            marker.header.frame_id = base_frame_;
        }

        // Publish marker array and pose array
        trajectory_marker_publisher_->publish(marker_array);
    }

    bool Ocs2PinocchioVisualizer::publishSelfCollisionVisualization(const vector_t& state) const
    {
        if (!geometry_visualization_)
        {
            return false;
        }
        const auto& model = pinocchio_interface_.getModel();
        if (static_cast<size_t>(state.size()) != static_cast<size_t>(model.nq))
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "Self-collision viz skipped: state size %ld != model.nq %u (check controller joints vs OCS2)",
                                 static_cast<long>(state.size()), model.nq);
            return false;
        }
        try
        {
            std::lock_guard<std::mutex> lock(self_collision_mutex_);
            geometry_visualization_->publishDistances(state);
            return true;
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "Self-collision visualization failed (disable selfCollision in task or fix URDF/collision pairs): %s",
                                 e.what());
            return false;
        }
    }

    scalar_t Ocs2PinocchioVisualizer::getLastMinDistance() const
    {
        if (!geometry_visualization_)
        {
            return std::numeric_limits<scalar_t>::max();
        }
        std::lock_guard<std::mutex> lock(self_collision_mutex_);
        return geometry_visualization_->getLastMinDistance();
    }

    void Ocs2PinocchioVisualizer::forwardKinematicsInto(pinocchio::Data& data, const vector_t& state) const
    {
        const auto& model = pinocchio_interface_.getModel();
        pinocchio::forwardKinematics(model, data, state);
        pinocchio::updateFramePlacements(model, data);
    }

    vector_t Ocs2PinocchioVisualizer::extractFramePose7(const pinocchio::Data& data,
                                                        const pinocchio::FrameIndex frame_id) const
    {
        vector_t pose = vector_t::Zero(7);
        pose(6) = 1.0;
        if (frame_id >= static_cast<pinocchio::FrameIndex>(data.oMf.size()))
        {
            return pose;
        }
        const auto& frame_placement = data.oMf[frame_id];
        pose.head<3>() = frame_placement.translation();
        const Eigen::Quaterniond quat(frame_placement.rotation());
        pose(3) = quat.x();
        pose(4) = quat.y();
        pose(5) = quat.z();
        pose(6) = quat.w();
        return pose;
    }

    void Ocs2PinocchioVisualizer::fillPoseStamped(geometry_msgs::msg::PoseStamped& msg, const rclcpp::Time& time,
                                                  const std::string& frame_id, const vector_t& pose7)
    {
        msg.header.stamp = time;
        msg.header.frame_id = frame_id;
        msg.pose.position.x = pose7(0);
        msg.pose.position.y = pose7(1);
        msg.pose.position.z = pose7(2);
        msg.pose.orientation.x = pose7(3);
        msg.pose.orientation.y = pose7(4);
        msg.pose.orientation.z = pose7(5);
        msg.pose.orientation.w = pose7(6);
    }

    void Ocs2PinocchioVisualizer::publishPose(
        const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr& pub,
        const rclcpp::Time& time, const vector_t& pose7) const
    {
        if (!pub)
        {
            return;
        }
        geometry_msgs::msg::PoseStamped msg;
        fillPoseStamped(msg, time, base_frame_, pose7);
        pub->publish(msg);
    }

    void Ocs2PinocchioVisualizer::setPosePublishPeriod(const double period_sec)
    {
        pose_publish_period_sec_ = std::max(0.0, period_sec);
    }

    Ocs2PinocchioVisualizer::EndEffectorPoses
    Ocs2PinocchioVisualizer::computeEndEffectorPoses(const vector_t& state) const
    {
        EndEffectorPoses poses;
        poses.left(6) = 1.0;
        poses.right(6) = 1.0;
        poses.body(6) = 1.0;

        const bool need_fk = left_ee_frame_valid_ ||
                             (dual_arm_mode_ && right_ee_frame_valid_) ||
                             body_frame_valid_;
        if (need_fk)
        {
            forwardKinematicsInto(rt_data_, state);
        }

        if (left_ee_frame_valid_)
        {
            poses.left = extractFramePose7(rt_data_, left_ee_frame_id_);
        }
        else
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "Left EE frame '%s' not in model (nframes=%d)",
                                 config_.left_ee_frame.c_str(), pinocchio_interface_.getModel().nframes);
        }

        if (dual_arm_mode_)
        {
            if (right_ee_frame_valid_)
            {
                poses.right = extractFramePose7(rt_data_, right_ee_frame_id_);
            }
            else
            {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                     "Right EE frame '%s' not in model (nframes=%d)",
                                     config_.right_ee_frame.c_str(), pinocchio_interface_.getModel().nframes);
            }
        }

        if (body_frame_valid_)
        {
            poses.body = extractFramePose7(rt_data_, body_frame_id_);
        }
        else if (!config_.body_frame.empty())
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "Body frame '%s' not in model (nframes=%d)",
                                 config_.body_frame.c_str(), pinocchio_interface_.getModel().nframes);
        }
        return poses;
    }

    void Ocs2PinocchioVisualizer::publishEndEffectorPoses(const rclcpp::Time& time,
                                                          const EndEffectorPoses& poses) const
    {
        if (pose_publish_period_sec_ > 0.0 && last_pose_publish_time_.nanoseconds() != 0 &&
            (time - last_pose_publish_time_).seconds() < pose_publish_period_sec_)
        {
            return;
        }
        last_pose_publish_time_ = time;
        publishPose(left_end_effector_pose_publisher_, time, poses.left);
        if (dual_arm_mode_)
        {
            publishPose(right_end_effector_pose_publisher_, time, poses.right);
        }
        publishPose(body_frame_pose_publisher_, time, poses.body);
    }

    void Ocs2PinocchioVisualizer::publishEndEffectorPose(const rclcpp::Time& time, const vector_t& state) const
    {
        publishEndEffectorPoses(time, computeEndEffectorPoses(state));
    }

    vector_t Ocs2PinocchioVisualizer::computeEndEffectorPose(const vector_t& state) const
    {
        return computeEndEffectorPoses(state).left;
    }

    vector_t Ocs2PinocchioVisualizer::computeRightEndEffectorPose(const vector_t& state) const
    {
        return computeEndEffectorPoses(state).right;
    }

    vector_t Ocs2PinocchioVisualizer::computeBodyFramePose(const vector_t& state) const
    {
        return computeEndEffectorPoses(state).body;
    }

    visualization_msgs::msg::Marker Ocs2PinocchioVisualizer::createTrajectoryLineMarker(
        const std::vector<geometry_msgs::msg::Point>& points,
        const std::array<double, 3>& color,
        const double line_width,
        const std::string& namespace_name) const
    {
        visualization_msgs::msg::Marker marker;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.ns = namespace_name;
        marker.scale.x = line_width;
        marker.color = ros_msg_helpers::getColor(color, 0.8);
        marker.points = points;
        return marker;
    }

    void Ocs2PinocchioVisualizer::clearTrajectoryHistory()
    {
        left_arm_trajectory_history_.clear();
        right_arm_trajectory_history_.clear();
        RCLCPP_INFO(node_->get_logger(), "Trajectory history cleared");
        
        // Publish empty trajectory visualization to clear previous display
        publishEndEffectorTrajectory(node_->now());
    }
}
