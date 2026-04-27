//
// Created by fiveages on 8/21/25.
//

#include "ocs2_controller_common/visualization/Ocs2PinocchioVisualizer.h"
#include <limits>
#include <fstream>
#include <string>
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
          enable_self_collision_(true),
          dual_arm_mode_(false),
          robot_name_(config_.robot_name),
          base_frame_(config_.base_frame),
          urdf_file_(config_.urdf_file),
          trajectory_line_width_(0.005),
          left_arm_color_({0.0, 0.4470, 0.7410}),
          right_arm_color_({0.6350, 0.0780, 0.1840})
    {
        dual_arm_mode_ = config_.dual_arm;
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
            
            // Read and publish URDF file
            try
            {
                std::ifstream urdf_stream(urdf_file_);
                if (urdf_stream.is_open())
                {
                    std::string urdf_content((std::istreambuf_iterator<char>(urdf_stream)),
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
            catch (const std::exception& e)
            {
                RCLCPP_ERROR(node_->get_logger(), "Failed to read and publish URDF: %s", e.what());
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

                enable_self_collision_ = true;
                RCLCPP_INFO(node_->get_logger(),
                            "Self-collision visualization initialized (activation distance: %.3f m)", activationDistance);
            }
            else
            {
                enable_self_collision_ = false;
                RCLCPP_INFO(node_->get_logger(),
                            "No Pinocchio geometry provided, self-collision visualization disabled");
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Failed to initialize self-collision visualization: %s, self-collision visualization disabled",
                        e.what());
            enable_self_collision_ = false;
        }

        RCLCPP_INFO(node_->get_logger(), "Ocs2PinocchioVisualizer initialization completed");
        RCLCPP_INFO(node_->get_logger(), "Self-collision visualization: %s",
                    enable_self_collision_ ? "enabled" : "disabled");
    }

    void Ocs2PinocchioVisualizer::updateEndEffectorTrajectory(const PrimalSolution& policy)
    {
        // Update end effector trajectory history - get from MPC calculation results
        if (!policy.stateTrajectory_.empty())
        {
            const auto& mpc_state_trajectory = policy.stateTrajectory_;

            if (dual_arm_mode_)
            {
                // Dual arm mode: calculate left and right arm end effector poses from MPC trajectory
                std::vector<vector_t> left_trajectory, right_trajectory;

                for (const auto& state : mpc_state_trajectory)
                {
                    vector_t left_ee_pose = computeEndEffectorPose(state);
                    vector_t right_ee_pose = computeRightEndEffectorPose(state);
                    left_trajectory.push_back(left_ee_pose);
                    right_trajectory.push_back(right_ee_pose);
                }

                // Update trajectory history
                left_arm_trajectory_history_ = std::move(left_trajectory);
                right_arm_trajectory_history_ = std::move(right_trajectory);
            }
            else
            {
                // Single arm mode: calculate end effector pose from MPC trajectory
                std::vector<vector_t> trajectory;

                for (const auto& state : mpc_state_trajectory)
                {
                    vector_t ee_pose = computeEndEffectorPose(state);
                    trajectory.push_back(ee_pose);
                }

                // Update trajectory history
                left_arm_trajectory_history_ = std::move(trajectory);
            }
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

    void Ocs2PinocchioVisualizer::publishSelfCollisionVisualization(const vector_t& state) const
    {
        if (!geometry_visualization_)
        {
            return;
        }
        const auto& model = pinocchio_interface_.getModel();
        if (static_cast<size_t>(state.size()) != static_cast<size_t>(model.nq))
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "Self-collision viz skipped: state size %ld != model.nq %u (check controller joints vs OCS2)",
                                 static_cast<long>(state.size()), model.nq);
            return;
        }
        try
        {
            geometry_visualization_->publishDistances(state);
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                 "Self-collision visualization failed (disable selfCollision in task or fix URDF/collision pairs): %s",
                                 e.what());
        }
    }

    bool Ocs2PinocchioVisualizer::isCollisionDetected(scalar_t threshold) const
    {
        if (geometry_visualization_)
        {
            return geometry_visualization_->isCollisionDetected(threshold);
        }
        return false;
    }

    void Ocs2PinocchioVisualizer::publishEndEffectorPose(
        const rclcpp::Time& time,
        const vector_t& state) const
    {
        // 先发布双臂（原有逻辑）
        if (dual_arm_mode_) {
            publishLeftEndEffectorPose(time, state);
            publishRightEndEffectorPose(time, state);
        } else {
            publishLeftEndEffectorPose(time, state);
        }

        // 新增：发布 body current pose
        const auto body_pose = computeBodyFramePose(state);

        geometry_msgs::msg::PoseStamped body_msg;
        body_msg.header.stamp = time;
        body_msg.header.frame_id = base_frame_;

        body_msg.pose.position.x = body_pose(0);
        body_msg.pose.position.y = body_pose(1);
        body_msg.pose.position.z = body_pose(2);

        body_msg.pose.orientation.x = body_pose(3);
        body_msg.pose.orientation.y = body_pose(4);
        body_msg.pose.orientation.z = body_pose(5);
        body_msg.pose.orientation.w = body_pose(6);

        body_frame_pose_publisher_->publish(body_msg);
    }

    vector_t Ocs2PinocchioVisualizer::computeEndEffectorPose(const vector_t& state) const
    {
        vector_t ee_state = vector_t::Zero(7);

        try
        {
            const auto& model = pinocchio_interface_.getModel();
            auto data = pinocchio_interface_.getData();

            pinocchio::forwardKinematics(model, data, state);
            pinocchio::updateFramePlacements(model, data);

            const auto& ee_frame_name = config_.left_ee_frame;
            const auto ee_frame_id = model.getFrameId(ee_frame_name);
            const auto& frame_placement = data.oMf[ee_frame_id];

            ee_state.head<3>() = frame_placement.translation();
            Eigen::Quaterniond quat(frame_placement.rotation());
            ee_state(3) = quat.x();
            ee_state(4) = quat.y();
            ee_state(5) = quat.z();
            ee_state(6) = quat.w();
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(node_->get_logger(), "Failed to compute end-effector pose: %s", e.what());
        }

        return ee_state;
    }

    vector_t Ocs2PinocchioVisualizer::computeRightEndEffectorPose(const vector_t& state) const
    {
        vector_t ee_state = vector_t::Zero(7);

        try
        {
            const auto& model = pinocchio_interface_.getModel();
            auto data = pinocchio_interface_.getData();

            pinocchio::forwardKinematics(model, data, state);
            pinocchio::updateFramePlacements(model, data);

            const auto& ee_frame_name = config_.right_ee_frame;
            const auto ee_frame_id = model.getFrameId(ee_frame_name);
            const auto& frame_placement = data.oMf[ee_frame_id];

            ee_state.head<3>() = frame_placement.translation();
            Eigen::Quaterniond quat(frame_placement.rotation());
            ee_state(3) = quat.x();
            ee_state(4) = quat.y();
            ee_state(5) = quat.z();
            ee_state(6) = quat.w();
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(node_->get_logger(), "Failed to compute right end-effector pose: %s", e.what());
        }

        return ee_state;
    }

    vector_t Ocs2PinocchioVisualizer::computeBodyFramePose(const vector_t& state) const
    {
        vector_t pose = vector_t::Zero(7);
        pose(6) = 1.0; // identity quaternion (x,y,z,w)
        if (config_.body_frame.empty())
        {
            return pose;
        }

        try
        {
            const auto& model = pinocchio_interface_.getModel();
            auto data = pinocchio_interface_.getData();

            pinocchio::forwardKinematics(model, data, state);
            pinocchio::updateFramePlacements(model, data);

            const auto& frame_name = config_.body_frame;
            const auto frame_id = model.getFrameId(frame_name);
            const auto& frame_placement = data.oMf[frame_id];

            pose.head<3>() = frame_placement.translation();
            Eigen::Quaterniond quat(frame_placement.rotation());
            pose(3) = quat.x();
            pose(4) = quat.y();
            pose(5) = quat.z();
            pose(6) = quat.w();
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(node_->get_logger(), "Failed to compute body frame pose: %s", e.what());
        }

        return pose;
    }

    void Ocs2PinocchioVisualizer::publishLeftEndEffectorPose(const rclcpp::Time& time, const vector_t& state) const
    {
        // Calculate left arm end effector pose (left arm uses default eeFrame)
        const auto ee_pose = computeEndEffectorPose(state);

        // Publish left arm pose information
        geometry_msgs::msg::PoseStamped ee_pose_msg;
        ee_pose_msg.header.stamp = time;
        ee_pose_msg.header.frame_id = base_frame_;
        ee_pose_msg.pose.position.x = ee_pose(0);
        ee_pose_msg.pose.position.y = ee_pose(1);
        ee_pose_msg.pose.position.z = ee_pose(2);
        ee_pose_msg.pose.orientation.w = ee_pose(6);
        ee_pose_msg.pose.orientation.x = ee_pose(3);
        ee_pose_msg.pose.orientation.y = ee_pose(4);
        ee_pose_msg.pose.orientation.z = ee_pose(5);

        left_end_effector_pose_publisher_->publish(ee_pose_msg);
    }

    void Ocs2PinocchioVisualizer::publishRightEndEffectorPose(const rclcpp::Time& time, const vector_t& state) const
    {
        // Calculate right arm end effector pose
        const auto ee_pose = computeRightEndEffectorPose(state);
        
        // Publish right arm pose information
        geometry_msgs::msg::PoseStamped ee_pose_msg;
        ee_pose_msg.header.stamp = time;
        ee_pose_msg.header.frame_id = base_frame_;
        ee_pose_msg.pose.position.x = ee_pose(0);
        ee_pose_msg.pose.position.y = ee_pose(1);
        ee_pose_msg.pose.position.z = ee_pose(2);
        ee_pose_msg.pose.orientation.w = ee_pose(6);
        ee_pose_msg.pose.orientation.x = ee_pose(3);
        ee_pose_msg.pose.orientation.y = ee_pose(4);
        ee_pose_msg.pose.orientation.z = ee_pose(5);
        
        right_end_effector_pose_publisher_->publish(ee_pose_msg);
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
