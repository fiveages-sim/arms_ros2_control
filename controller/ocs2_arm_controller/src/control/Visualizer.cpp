//
// Created by fiveages on 8/21/25.
//

#include "ocs2_arm_controller/control/Visualizer.h"
#include <limits>
#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <ocs2_core/misc/LoadData.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

namespace ocs2::mobile_manipulator
{
    Visualizer::Visualizer(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode>& node,
                           const std::shared_ptr<MobileManipulatorInterface>& interface,
                           const std::string& robot_name)
        : node_(node),
          interface_(interface),
          enable_self_collision_(true),
          dual_arm_mode_(false),
          robot_name_(robot_name),
          trajectory_line_width_(0.005),
          left_arm_color_({0.0, 0.4470, 0.7410}),
          right_arm_color_({0.6350, 0.0780, 0.1840})
    {
        // Detect if dual arm mode is enabled
        dual_arm_mode_ = interface_->dual_arm_;
        
        // Get base frame information
        base_frame_ = interface_->getManipulatorModelInfo().baseFrame;
    }

    void Visualizer::initialize()
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

        // Get self-collision geometry interface from MobileManipulatorInterface and initialize visualization
        try
        {
            if (const auto pinocchio_geometry_interface = interface_->getPinocchioGeometryInterface())
            {
                // Get activation distance for visualization filtering
                const scalar_t activationDistance = interface_->getSelfCollisionActivationDistance();
                
                geometry_visualization_ = std::make_unique<GeometryInterfaceVisualization>(
                    interface_->getPinocchioInterface(),
                    std::move(*pinocchio_geometry_interface), base_frame_, activationDistance);

                enable_self_collision_ = true;
                RCLCPP_INFO(node_->get_logger(), 
                    "Self-collision visualization initialized (activation distance: %.3f m)", activationDistance);
            }
            else
            {
                enable_self_collision_ = false;
                RCLCPP_INFO(node_->get_logger(),
                            "Interface has no geometry information, self-collision visualization disabled");
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_WARN(node_->get_logger(),
                        "Failed to initialize self-collision visualization: %s, self-collision visualization disabled",
                        e.what());
            enable_self_collision_ = false;
        }

        RCLCPP_INFO(node_->get_logger(), "Visualizer initialization completed");
        RCLCPP_INFO(node_->get_logger(), "Self-collision visualization: %s",
                    enable_self_collision_ ? "enabled" : "disabled");
    }

    void Visualizer::updateEndEffectorTrajectory(const PrimalSolution& policy)
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


    void Visualizer::publishEndEffectorTrajectory(const rclcpp::Time& time)
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

    void Visualizer::publishSelfCollisionVisualization(const vector_t& state) const
    {
        if (geometry_visualization_)
        {
            // Use GeometryInterfaceVisualization for self-collision visualization
            geometry_visualization_->publishDistances(state);
        }
    }

    bool Visualizer::isCollisionDetected(scalar_t threshold) const
    {
        if (geometry_visualization_)
        {
            return geometry_visualization_->isCollisionDetected(threshold);
        }
        return false;
    }

    void Visualizer::publishEndEffectorPose(const rclcpp::Time& time, const vector_t& state) const
    {
        if (dual_arm_mode_) {
            // Dual arm mode: publish left and right arm end effector poses
            publishLeftEndEffectorPose(time, state);
            publishRightEndEffectorPose(time, state);
        } else {
            // Single arm mode: use left arm publisher
            publishLeftEndEffectorPose(time, state);
        }
    }

    vector_t Visualizer::computeEndEffectorPose(const vector_t& state) const
    {
        vector_t ee_state = vector_t::Zero(7);

        try
        {
            const auto& pinocchio_interface = interface_->getPinocchioInterface();
            const auto& model = pinocchio_interface.getModel();
            auto data = pinocchio_interface.getData();

            pinocchio::forwardKinematics(model, data, state);
            pinocchio::updateFramePlacements(model, data);

            const auto& ee_frame_name = interface_->getManipulatorModelInfo().eeFrame;
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

    vector_t Visualizer::computeRightEndEffectorPose(const vector_t& state) const
    {
        vector_t ee_state = vector_t::Zero(7);

        try
        {
            const auto& pinocchio_interface = interface_->getPinocchioInterface();
            const auto& model = pinocchio_interface.getModel();
            auto data = pinocchio_interface.getData();

            pinocchio::forwardKinematics(model, data, state);
            pinocchio::updateFramePlacements(model, data);

            const auto& ee_frame_name = interface_->getManipulatorModelInfo().eeFrame1;
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

    void Visualizer::publishLeftEndEffectorPose(const rclcpp::Time& time, const vector_t& state) const
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

    void Visualizer::publishRightEndEffectorPose(const rclcpp::Time& time, const vector_t& state) const
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

    visualization_msgs::msg::Marker Visualizer::createTrajectoryLineMarker(
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

    void Visualizer::clearTrajectoryHistory()
    {
        left_arm_trajectory_history_.clear();
        right_arm_trajectory_history_.clear();
        RCLCPP_INFO(node_->get_logger(), "Trajectory history cleared");
        
        // Publish empty trajectory visualization to clear previous display
        publishEndEffectorTrajectory(node_->now());
    }
}
