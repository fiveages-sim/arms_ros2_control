//
// HeadMarker - 头部 Marker 管理类实现
//

#include "arms_target_manager/marker/HeadMarker.h"
#include "arms_target_manager/MarkerFactory.h"
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>
#include <algorithm>
#include <cmath>

namespace arms_ros2_control::command
{
    HeadMarker::HeadMarker(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<MarkerFactory> marker_factory,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        const std::string& frame_id,
        const std::string& target_topic)
        : node_(std::move(node))
          , marker_factory_(std::move(marker_factory))
          , tf_buffer_(std::move(tf_buffer))
          , frame_id_(frame_id)
    {
        // 初始化默认 pose
        head_pose_.position.x = 1.0;
        head_pose_.position.y = 0.0;
        head_pose_.position.z = 1.5;
        head_pose_.orientation.w = 1.0;
        head_pose_.orientation.x = 0.0;
        head_pose_.orientation.y = 0.0;
        head_pose_.orientation.z = 0.0;

        // 创建发布器（在构造函数中创建，topic 名称在 initialize 时可能会从参数读取）
        joint_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(target_topic, 1);
    }

    void HeadMarker::initialize()
    {
        // 读取是否启用头部控制
        node_->declare_parameter<bool>("enable_head_control", false);
        enable_head_control_ = node_->get_parameter("enable_head_control").as_bool();

        if (!enable_head_control_)
        {
            return;
        }

        // 初始化头部关节限位管理器
        head_limits_manager_ = std::make_shared<arms_controller_common::JointLimitsManager>(node_->get_logger());

        // 读取头部link名称配置
        node_->declare_parameter<std::string>("head_link_name", "head_link2");
        head_link_name_ = node_->get_parameter("head_link_name").as_string();
        RCLCPP_INFO(node_->get_logger(),
                    "头部marker所在的link名称: %s", head_link_name_.c_str());

        // 读取头部关节映射配置
        std::string parent_param_name = "head_joint_to_rpy_mapping";
        RCLCPP_INFO(node_->get_logger(),
                    "开始读取头部关节映射配置: %s", parent_param_name.c_str());

        // 先显式声明所有可能的嵌套参数
        std::vector<std::string> common_joint_names = {"head_joint1", "head_joint2", "head_joint3"};
        for (const auto& joint_name : common_joint_names)
        {
            std::string param_name = parent_param_name + "." + joint_name;
            node_->declare_parameter(param_name, "");
        }

        // 使用 list_parameters 获取所有配置的关节映射
        auto result = node_->list_parameters({parent_param_name}, 1);

        // 读取所有关节到RPY的映射
        for (const auto& param_name : result.names)
        {
            size_t dot_pos = param_name.find_last_of('.');
            if (dot_pos != std::string::npos && dot_pos + 1 < param_name.length())
            {
                std::string joint_name = param_name.substr(dot_pos + 1);
                std::string rpy_name = node_->get_parameter(param_name).as_string();

                if (!rpy_name.empty())
                {
                    head_joint_to_rpy_mapping_[joint_name] = rpy_name;
                    RCLCPP_INFO(node_->get_logger(),
                                "配置头部映射: %s -> %s",
                                joint_name.c_str(), rpy_name.c_str());
                }
            }
        }

        // 读取旋转轴方向配置
        std::set<std::string> used_rpy_names;
        for (const auto& [joint_name, rpy_name] : head_joint_to_rpy_mapping_)
        {
            if (!rpy_name.empty())
            {
                used_rpy_names.insert(rpy_name);
            }
        }

        for (const auto& rpy_name : used_rpy_names)
        {
            std::string param_name = "head_rpy_axis_direction." + rpy_name;
            node_->declare_parameter<double>(param_name, 1.0);
            head_rpy_axis_direction_[rpy_name] = node_->get_parameter(param_name).as_double();
            RCLCPP_INFO(node_->get_logger(),
                        "配置头部旋转轴方向: %s = %.1f",
                        param_name.c_str(),
                        head_rpy_axis_direction_[rpy_name]);
        }

        // 按照控制器期望的关节顺序构建发送顺序
        std::vector<std::string> standard_joint_order = {"head_joint1", "head_joint2", "head_joint3"};
        std::string order_str;

        for (const auto& joint_name : standard_joint_order)
        {
            auto it = head_joint_to_rpy_mapping_.find(joint_name);
            if (it != head_joint_to_rpy_mapping_.end() && !it->second.empty())
            {
                head_joint_send_order_.push_back(joint_name);
                if (!order_str.empty())
                {
                    order_str += ", ";
                }
                order_str += joint_name;
            }
        }

        RCLCPP_INFO(node_->get_logger(),
                    "✓ 头部关节发送顺序（按控制器期望）: [%s]", order_str.c_str());

        // 设置限位管理器的关节顺序
        if (head_limits_manager_)
        {
            head_limits_manager_->setJointNames(head_joint_send_order_);
        }

        // 订阅 robot_description 以解析头部关节限位
        if (head_limits_manager_)
        {
            robot_description_subscription_ = node_->create_subscription<std_msgs::msg::String>(
                "/robot_description", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
                [this](const std_msgs::msg::String::SharedPtr msg)
                {
                    if (head_limits_manager_)
                    {
                        head_limits_manager_->parseFromURDF(msg->data, head_joint_send_order_);
                        RCLCPP_INFO(node_->get_logger(),
                                   "头部关节限位已从 /robot_description topic 加载");
                    }
                });
        }
    }

    visualization_msgs::msg::InteractiveMarker HeadMarker::createMarker(
        const std::string& name,
        const geometry_msgs::msg::Pose& pose,
        bool enable_interaction) const
    {
        // 确定要传递给createHeadMarker的关节集合
        std::set<std::string> joints_to_use;

        for (const auto& [joint_name, rpy_name] : head_joint_to_rpy_mapping_)
        {
            if (!rpy_name.empty())
            {
                joints_to_use.insert(rpy_name);
            }
        }

        // 如果映射为空，使用默认的RPY名称（向后兼容）
        if (joints_to_use.empty())
        {
            RCLCPP_WARN(node_->get_logger(),
                        "头部关节映射配置为空，使用默认RPY（head_roll, head_pitch, head_yaw）。"
                        "建议在配置文件中设置 head_joint_to_rpy_mapping");
            joints_to_use.insert("head_roll");
            joints_to_use.insert("head_pitch");
            joints_to_use.insert("head_yaw");
        }

        return marker_factory_->createHeadMarker(name, pose, enable_interaction, joints_to_use);
    }

    std::vector<double> HeadMarker::quaternionToJointAngles(
        const geometry_msgs::msg::Quaternion& quaternion) const
    {
        // 使用 tf2 的 getRPY 从 quaternion 提取欧拉角
        tf2::Quaternion tf_quat;
        tf2::fromMsg(quaternion, tf_quat);

        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

        // 使用通用工具类 unwrap 保持角度连续性，避免跳变
        arms_controller_common::AngleUtils::unwrapRPY(
            roll, pitch, yaw, last_head_rpy_, last_head_rpy_initialized_);

        // 应用旋转轴方向系数
        double roll_with_direction = roll;
        double pitch_with_direction = pitch;
        double yaw_with_direction = yaw;

        auto it_roll_dir = head_rpy_axis_direction_.find("head_roll");
        if (it_roll_dir != head_rpy_axis_direction_.end())
        {
            roll_with_direction = roll * it_roll_dir->second;
        }

        auto it_pitch_dir = head_rpy_axis_direction_.find("head_pitch");
        if (it_pitch_dir != head_rpy_axis_direction_.end())
        {
            pitch_with_direction = pitch * it_pitch_dir->second;
        }

        auto it_yaw_dir = head_rpy_axis_direction_.find("head_yaw");
        if (it_yaw_dir != head_rpy_axis_direction_.end())
        {
            yaw_with_direction = yaw * it_yaw_dir->second;
        }

        // 按照控制器期望的关节顺序组织数据
        if (!head_joint_send_order_.empty())
        {
            std::vector<double> joint_angles;
            std::map<std::string, double> rpy_values = {
                {"head_roll", roll_with_direction},
                {"head_pitch", pitch_with_direction},
                {"head_yaw", yaw_with_direction}
            };

            for (const auto& joint_name : head_joint_send_order_)
            {
                auto joint_it = head_joint_to_rpy_mapping_.find(joint_name);
                if (joint_it != head_joint_to_rpy_mapping_.end())
                {
                    const std::string& rpy_name = joint_it->second;
                    auto rpy_it = rpy_values.find(rpy_name);
                    if (rpy_it != rpy_values.end())
                    {
                        joint_angles.push_back(rpy_it->second);
                    }
                    else
                    {
                        joint_angles.push_back(0.0);
                    }
                }
                else
                {
                    joint_angles.push_back(0.0);
                }
            }

            return joint_angles;
        }

        RCLCPP_ERROR(node_->get_logger(),
                     "头部关节发送顺序配置为空，无法发送目标关节角度。"
                     "请在配置文件中设置 head_joint_to_rpy_mapping");
        return {};
    }

    bool HeadMarker::clampPoseRotation(geometry_msgs::msg::Pose& pose) const
    {
        if (!head_limits_manager_ || !head_limits_manager_->hasAnyLimits())
        {
            return false;
        }

        // 从四元数提取 RPY 角度
        tf2::Quaternion tf_quat;
        tf2::fromMsg(pose.orientation, tf_quat);

        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

        // 使用通用工具类 unwrap 保持角度连续性，避免跳变
        arms_controller_common::AngleUtils::unwrapRPY(
            roll, pitch, yaw, last_head_rpy_, last_head_rpy_initialized_);

        // 应用旋转轴方向系数，得到关节角度
        double roll_with_direction = roll;
        double pitch_with_direction = pitch;
        double yaw_with_direction = yaw;

        auto it_roll_dir = head_rpy_axis_direction_.find("head_roll");
        if (it_roll_dir != head_rpy_axis_direction_.end())
        {
            roll_with_direction = roll * it_roll_dir->second;
        }

        auto it_pitch_dir = head_rpy_axis_direction_.find("head_pitch");
        if (it_pitch_dir != head_rpy_axis_direction_.end())
        {
            pitch_with_direction = pitch * it_pitch_dir->second;
        }

        auto it_yaw_dir = head_rpy_axis_direction_.find("head_yaw");
        if (it_yaw_dir != head_rpy_axis_direction_.end())
        {
            yaw_with_direction = yaw * it_yaw_dir->second;
        }

        // 将 RPY 映射到关节角度，并根据限位限制
        std::map<std::string, double> rpy_to_joint = {
            {"head_roll", roll_with_direction},
            {"head_pitch", pitch_with_direction},
            {"head_yaw", yaw_with_direction}
        };

        bool was_clamped = false;
        std::map<std::string, double> clamped_rpy;

        // 对每个关节进行限位检查
        for (const auto& joint_name : head_joint_send_order_)
        {
            auto joint_it = head_joint_to_rpy_mapping_.find(joint_name);
            if (joint_it != head_joint_to_rpy_mapping_.end())
            {
                const std::string& rpy_name = joint_it->second;
                auto rpy_it = rpy_to_joint.find(rpy_name);

                if (rpy_it != rpy_to_joint.end())
                {
                    double joint_angle = rpy_it->second;
                    double original_angle = joint_angle;

                    // 获取关节限位
                    auto limits = head_limits_manager_->getJointLimits(joint_name);
                    if (limits.initialized)
                    {
                        joint_angle = std::clamp(joint_angle, limits.lower, limits.upper);

                        if (std::abs(joint_angle - original_angle) > 1e-6)
                        {
                            was_clamped = true;
                        }
                    }

                    clamped_rpy[rpy_name] = joint_angle;
                }
            }
        }

        if (!was_clamped)
        {
            return false;
        }

        // 将限制后的关节角度转换回 RPY
        double clamped_roll = clamped_rpy.find("head_roll") != clamped_rpy.end()
                                  ? clamped_rpy["head_roll"]
                                  : roll_with_direction;
        double clamped_pitch = clamped_rpy.find("head_pitch") != clamped_rpy.end()
                                   ? clamped_rpy["head_pitch"]
                                   : pitch_with_direction;
        double clamped_yaw = clamped_rpy.find("head_yaw") != clamped_rpy.end()
                                 ? clamped_rpy["head_yaw"]
                                 : yaw_with_direction;

        // 应用反向的旋转轴方向系数
        if (it_roll_dir != head_rpy_axis_direction_.end() && it_roll_dir->second != 0.0)
        {
            clamped_roll = clamped_roll / it_roll_dir->second;
        }
        if (it_pitch_dir != head_rpy_axis_direction_.end() && it_pitch_dir->second != 0.0)
        {
            clamped_pitch = clamped_pitch / it_pitch_dir->second;
        }
        if (it_yaw_dir != head_rpy_axis_direction_.end() && it_yaw_dir->second != 0.0)
        {
            clamped_yaw = clamped_yaw / it_yaw_dir->second;
        }

        // 将限制后的 RPY 转换回四元数
        tf2::Quaternion clamped_quat;
        clamped_quat.setRPY(clamped_roll, clamped_pitch, clamped_yaw);
        clamped_quat.normalize();

        pose.orientation.w = clamped_quat.w();
        pose.orientation.x = clamped_quat.x();
        pose.orientation.y = clamped_quat.y();
        pose.orientation.z = clamped_quat.z();

        return true;
    }

    geometry_msgs::msg::Pose HeadMarker::updateFromJointState(
        const sensor_msgs::msg::JointState::ConstSharedPtr& joint_msg,
        bool is_state_disabled)
    {
        // 初始化关节索引（如果需要）
        if (head_joint_indices_.empty() && !head_joint_to_rpy_mapping_.empty())
        {
            initializeJointIndices(joint_msg);
        }

        // 如果状态禁用，只更新位置
        if (is_state_disabled)
        {
            try
            {
                geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                    frame_id_, head_link_name_, tf2::TimePointZero);

                head_pose_.position.x = transform.transform.translation.x;
                head_pose_.position.y = transform.transform.translation.y;
                head_pose_.position.z = transform.transform.translation.z;
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_DEBUG(node_->get_logger(),
                             "无法从 TF 获取头部 link %s 的位置: %s",
                             head_link_name_.c_str(), ex.what());
            }
            return head_pose_;
        }

        // 从 TF 获取头部 link 的实际位置
        try
        {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                frame_id_, head_link_name_, tf2::TimePointZero);

            head_pose_.position.x = transform.transform.translation.x;
            head_pose_.position.y = transform.transform.translation.y;
            head_pose_.position.z = transform.transform.translation.z;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_DEBUG(node_->get_logger(),
                         "无法从 TF 获取头部 link %s 的位置: %s，保持当前位置",
                         head_link_name_.c_str(), ex.what());
        }

        // 从关节状态提取 RPY 角度
        double head_roll = 0.0;
        double head_pitch = 0.0;
        double head_yaw = 0.0;

        if (extractRPYFromJointState(joint_msg, head_roll, head_pitch, head_yaw))
        {
            // 更新上一次的 RPY 值
            last_head_rpy_[0] = head_roll;
            last_head_rpy_[1] = head_pitch;
            last_head_rpy_[2] = head_yaw;
            last_head_rpy_initialized_ = true;

            // 从 RPY 创建四元数
            tf2::Quaternion tf_quat;
            tf_quat.setRPY(head_roll, head_pitch, head_yaw);
            tf_quat.normalize();

            geometry_msgs::msg::Quaternion quat;
            quat.w = tf_quat.w();
            quat.x = tf_quat.x();
            quat.y = tf_quat.y();
            quat.z = tf_quat.z();

            head_pose_.orientation = quat;
        }

        return head_pose_;
    }

    void HeadMarker::publishTargetJointAngles(const geometry_msgs::msg::Pose& pose) const
    {
        if (!joint_publisher_)
        {
            return;
        }

        // 使用传入的 pose 或当前 pose
        geometry_msgs::msg::Pose target_pose = pose;
        if (target_pose.orientation.w == 0.0 && target_pose.orientation.x == 0.0 &&
            target_pose.orientation.y == 0.0 && target_pose.orientation.z == 0.0)
        {
            // 如果 pose 为空（默认值），使用当前 pose
            target_pose = head_pose_;
        }

        // 从四元数提取关节角度
        std::vector<double> joint_angles = quaternionToJointAngles(target_pose.orientation);

        // 应用关节限位
        if (head_limits_manager_)
        {
            joint_angles = head_limits_manager_->applyLimits(joint_angles);
        }

        std_msgs::msg::Float64MultiArray msg;
        msg.data = joint_angles;
        joint_publisher_->publish(msg);
    }

    void HeadMarker::initializeJointIndices(const sensor_msgs::msg::JointState::ConstSharedPtr& joint_msg)
    {
        for (const auto& [joint_name, rpy_name] : head_joint_to_rpy_mapping_)
        {
            if (rpy_name.empty())
            {
                continue;
            }

            for (size_t i = 0; i < joint_msg->name.size(); ++i)
            {
                if (joint_msg->name[i] == joint_name)
                {
                    head_joint_indices_[joint_name] = i;
                    break;
                }
            }
        }

        if (!head_joint_indices_.empty())
        {
            std::string joints_str;
            for (const auto& [joint_name, index] : head_joint_indices_)
            {
                if (!joints_str.empty())
                {
                    joints_str += ", ";
                }
                joints_str += joint_name;
            }
            RCLCPP_INFO(node_->get_logger(),
                        "基于配置映射初始化头部关节索引: [%s] (共 %zu 个关节)",
                        joints_str.c_str(), head_joint_indices_.size());
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(),
                        "未在 joint_states 中找到配置的头部关节。请检查配置文件和关节名称是否正确。");
        }
    }

    bool HeadMarker::extractRPYFromJointState(
        const sensor_msgs::msg::JointState::ConstSharedPtr& joint_msg,
        double& head_roll,
        double& head_pitch,
        double& head_yaw) const
    {
        bool found_roll = false;
        bool found_pitch = false;
        bool found_yaw = false;

        for (const auto& [joint_name, rpy_name] : head_joint_to_rpy_mapping_)
        {
            auto it = head_joint_indices_.find(joint_name);
            if (it != head_joint_indices_.end() && it->second < joint_msg->position.size())
            {
                double joint_value = joint_msg->position[it->second];

                // 应用旋转轴方向系数
                auto dir_it = head_rpy_axis_direction_.find(rpy_name);
                if (dir_it != head_rpy_axis_direction_.end())
                {
                    joint_value *= dir_it->second;
                }

                if (rpy_name == "head_roll")
                {
                    head_roll = joint_value;
                    found_roll = true;
                }
                else if (rpy_name == "head_pitch")
                {
                    head_pitch = joint_value;
                    found_pitch = true;
                }
                else if (rpy_name == "head_yaw")
                {
                    head_yaw = joint_value;
                    found_yaw = true;
                }
            }
        }

        if (!found_roll && !found_pitch && !found_yaw)
        {
            if (head_joint_to_rpy_mapping_.empty())
            {
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                     "头部关节映射配置为空，无法更新头部marker方向。"
                                     "请在配置文件中设置 head_joint_to_rpy_mapping");
            }
            else
            {
                RCLCPP_DEBUG_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                                      "未在 joint_states 中找到配置的头部关节，无法更新头部marker方向");
            }
            return false;
        }

        return true;
    }
} // namespace arms_ros2_control::command

