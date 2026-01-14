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
        double publish_rate,
        const std::string& target_topic)
        : node_(std::move(node))
          , marker_factory_(std::move(marker_factory))
          , tf_buffer_(std::move(tf_buffer))
          , frame_id_(frame_id)
          , publish_rate_(publish_rate)
          , last_publish_time_(node_->now())
          , last_subscription_update_time_(node_->now())
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
        // 将世界坐标系的四元数转换为相对于 head_link 的局部四元数
        tf2::Quaternion marker_quat_world;
        tf2::fromMsg(quaternion, marker_quat_world);

        // 获取 head_link 在世界坐标系下的姿态（head_link_name_ 在配置文件中设置）
        tf2::Quaternion head_link_quat;
        try {
            auto transform = tf_buffer_->lookupTransform(
                frame_id_, head_link_name_,
                tf2::TimePointZero);
            tf2::fromMsg(transform.transform.rotation, head_link_quat);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_DEBUG(node_->get_logger(),
                        "无法获取 %s 姿态，使用单位四元数: %s",
                        head_link_name_.c_str(), ex.what());
            head_link_quat.setRPY(0, 0, 0);
        }

        // 计算相对四元数：relative = head_link^(-1) * marker_world
        // 这表示 marker 相对于 head_link 的局部旋转
        tf2::Quaternion relative_quat = head_link_quat.inverse() * marker_quat_world;

        // 从相对四元数提取欧拉角
        double roll, pitch, yaw;
        tf2::Matrix3x3(relative_quat).getRPY(roll, pitch, yaw);

        // 打印世界坐标系和 head_link 的姿态信息（用于调试）
        double head_link_roll, head_link_pitch, head_link_yaw;
        tf2::Matrix3x3(head_link_quat).getRPY(head_link_roll, head_link_pitch, head_link_yaw);
        double marker_world_roll, marker_world_pitch, marker_world_yaw;
        tf2::Matrix3x3(marker_quat_world).getRPY(marker_world_roll, marker_world_pitch, marker_world_yaw);

        RCLCPP_INFO(node_->get_logger(), "════════════════════════════════════════════════════════════════");
        RCLCPP_INFO(node_->get_logger(),
                    "[HeadMarker] Marker世界姿态 RPY: [%.3f, %.3f, %.3f] rad = [%.1f°, %.1f°, %.1f°]",
                    marker_world_roll, marker_world_pitch, marker_world_yaw,
                    marker_world_roll * 180.0 / M_PI, marker_world_pitch * 180.0 / M_PI, marker_world_yaw * 180.0 / M_PI);
        RCLCPP_INFO(node_->get_logger(),
                    "[HeadMarker] HeadLink世界姿态 RPY: [%.3f, %.3f, %.3f] rad = [%.1f°, %.1f°, %.1f°]",
                    head_link_roll, head_link_pitch, head_link_yaw,
                    head_link_roll * 180.0 / M_PI, head_link_pitch * 180.0 / M_PI, head_link_yaw * 180.0 / M_PI);
        RCLCPP_INFO(node_->get_logger(),
                    "[HeadMarker] 相对HeadLink的局部姿态 RPY: [%.3f, %.3f, %.3f] rad = [%.1f°, %.1f°, %.1f°]",
                    roll, pitch, yaw,
                    roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);

        // 使用通用工具类 unwrap 保持角度连续性，避免跳变（使用局部坐标系的历史数据）
        arms_controller_common::AngleUtils::unwrapRPY(
            roll, pitch, yaw, last_head_rpy_local_, last_head_rpy_local_initialized_);

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
                        // 获取相对角度
                        double relative_angle = rpy_it->second;

                        // 获取当前关节角度
                        double current_angle = 0.0;
                        auto current_it = current_joint_positions_.find(joint_name);
                        if (current_it != current_joint_positions_.end())
                        {
                            current_angle = current_it->second;
                        }

                        // 计算绝对目标角度 = 当前角度 + 相对角度
                        double target_angle = current_angle + relative_angle;
                        joint_angles.push_back(target_angle);
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

            // 打印相对角度
            RCLCPP_INFO(node_->get_logger(),
                        "[HeadMarker] 相对角度（局部旋转） RPY: [%.3f, %.3f, %.3f] rad = [%.1f°, %.1f°, %.1f°]",
                        roll_with_direction, pitch_with_direction, yaw_with_direction,
                        roll_with_direction * 180.0 / M_PI, pitch_with_direction * 180.0 / M_PI, yaw_with_direction * 180.0 / M_PI);

            // 打印当前关节角度
            std::string current_angles_str = "[";
            for (size_t i = 0; i < head_joint_send_order_.size(); ++i) {
                auto it = current_joint_positions_.find(head_joint_send_order_[i]);
                double current = (it != current_joint_positions_.end()) ? it->second : 0.0;
                current_angles_str += std::to_string(current) + " rad = " +
                                     std::to_string(current * 180.0 / M_PI) + "°";
                if (i < head_joint_send_order_.size() - 1) current_angles_str += ", ";
            }
            current_angles_str += "]";
            RCLCPP_INFO(node_->get_logger(),
                        "[HeadMarker] 当前关节角度: %s", current_angles_str.c_str());

            // 打印目标绝对角度
            std::string joint_angles_str = "[";
            for (size_t i = 0; i < joint_angles.size(); ++i) {
                joint_angles_str += std::to_string(joint_angles[i]) + " rad = " +
                                   std::to_string(joint_angles[i] * 180.0 / M_PI) + "°";
                if (i < joint_angles.size() - 1) joint_angles_str += ", ";
            }
            joint_angles_str += "]";
            RCLCPP_INFO(node_->get_logger(),
                        "[HeadMarker] 目标绝对角度（发送给控制器）: %s", joint_angles_str.c_str());
            RCLCPP_INFO(node_->get_logger(), "════════════════════════════════════════════════════════════════");

            return joint_angles;
        }

        RCLCPP_ERROR(node_->get_logger(),
                     "头部关节发送顺序配置为空，无法发送目标关节角度。"
                     "请在配置文件中设置 head_joint_to_rpy_mapping");
        return {};
    }

    geometry_msgs::msg::Pose HeadMarker::updateFromJointState(
        const sensor_msgs::msg::JointState::ConstSharedPtr& joint_msg,
        bool is_state_disabled)
    {

        // 节流检查：限制更新频率为最多30Hz（1/30秒间隔）
        auto now = node_->now();
        auto time_since_last = (now - last_subscription_update_time_).seconds();
        if (time_since_last < 1.0 / 30.0)  // 30Hz = 1/30秒
        {
            return head_pose_;  // 跳过此次更新，返回当前pose
        }
        last_subscription_update_time_ = now;

        // 初始化关节索引（如果需要）
        if (head_joint_indices_.empty() && !head_joint_to_rpy_mapping_.empty())
        {
            initializeJointIndices(joint_msg);
        }

        // 更新当前关节角度（用于 quaternionToJointAngles 计算绝对目标角度）
        for (const auto& [joint_name, index] : head_joint_indices_)
        {
            if (index < joint_msg->position.size())
            {
                current_joint_positions_[joint_name] = joint_msg->position[index];
            }
        }

        // 先检查状态：如果状态禁用，根据xyz位置变化来判断是否更新四元数
        if (is_state_disabled)
        {

            try
            {
                geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                    frame_id_, head_link_name_, tf2::TimePointZero);

                // 计算位置变化量
                double position_change = 0.0;
                if (last_position_initialized_)
                {
                    double dx = transform.transform.translation.x - last_position_.x;
                    double dy = transform.transform.translation.y - last_position_.y;
                    double dz = transform.transform.translation.z - last_position_.z;
                    position_change = std::sqrt(dx * dx + dy * dy + dz * dz);
                }

                // 更新位置
                head_pose_.position.x = transform.transform.translation.x;
                head_pose_.position.y = transform.transform.translation.y;
                head_pose_.position.z = transform.transform.translation.z;

                // 只有当位置变化较大时才更新四元数（阈值：0.01米 = 1厘米）
                const double position_change_threshold = 0.0001;
                bool should_update_orientation = !last_position_initialized_ || position_change > position_change_threshold;

                // 保存当前位置
                last_position_.x = head_pose_.position.x;
                last_position_.y = head_pose_.position.y;
                last_position_.z = head_pose_.position.z;
                last_position_initialized_ = true;

                if (should_update_orientation)
                {
                    // 从 TF 获取四元数方向（与 head_link2 完全重合）
                    head_pose_.orientation = transform.transform.rotation;

                    // 从四元数提取 RPY 角度，用于保持角度连续性
                    tf2::Quaternion tf_quat;
                    tf2::fromMsg(transform.transform.rotation, tf_quat);
                    double roll, pitch, yaw;
                    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
                    arms_controller_common::AngleUtils::unwrapRPY(
                        roll, pitch, yaw, last_head_rpy_, last_head_rpy_initialized_);
                    last_head_rpy_[0] = roll;
                    last_head_rpy_[1] = pitch;
                    last_head_rpy_[2] = yaw;
                    last_head_rpy_initialized_ = true;
                }
                // 如果位置变化不大，保持原有的 orientation 不变
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_DEBUG(node_->get_logger(),
                             "无法从 TF 获取头部 link %s 的位置和方向: %s",
                             head_link_name_.c_str(), ex.what());
            }
            return head_pose_;
        }


        // 初始化关节索引（如果需要）
        if (head_joint_indices_.empty() && !head_joint_to_rpy_mapping_.empty())
        {
            initializeJointIndices(joint_msg);
        }

        // 从 TF 获取头部 link 的实际位置和方向
        try
        {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                frame_id_, head_link_name_, tf2::TimePointZero);

            head_pose_.position.x = transform.transform.translation.x;
            head_pose_.position.y = transform.transform.translation.y;
            head_pose_.position.z = transform.transform.translation.z;

            // 直接从 TF 获取四元数方向（与 head_link2 完全重合）
            head_pose_.orientation = transform.transform.rotation;

            // 从四元数提取 RPY 角度，用于保持角度连续性
            tf2::Quaternion tf_quat;
            tf2::fromMsg(transform.transform.rotation, tf_quat);
            double roll, pitch, yaw;
            tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
            arms_controller_common::AngleUtils::unwrapRPY(
                roll, pitch, yaw, last_head_rpy_, last_head_rpy_initialized_);
            last_head_rpy_[0] = roll;
            last_head_rpy_[1] = pitch;
            last_head_rpy_[2] = yaw;
            last_head_rpy_initialized_ = true;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_DEBUG(node_->get_logger(),
                         "无法从 TF 获取头部 link %s 的位置和方向: %s，保持当前位置",
                         head_link_name_.c_str(), ex.what());
        }

        return head_pose_;
    }

    bool HeadMarker::publishTargetJointAngles(bool force) const
    {
        if (!joint_publisher_)
        {
            return false;
        }

        // 如果不是强制发送，检查是否需要节流（用于连续发布模式）
        if (!force)
        {
            if (!shouldThrottle(1.0 / publish_rate_))
            {
                return false;
            }
        }

        // 使用内部管理的 pose（与 ArmMarker 保持一致）
        // 从四元数提取关节角度
        std::vector<double> joint_angles = quaternionToJointAngles(head_pose_.orientation);

        // 应用关节限位
        if (head_limits_manager_)
        {
            joint_angles = head_limits_manager_->applyLimits(joint_angles);
        }

        // 打印实际发布到 topic 的数据
        RCLCPP_INFO(node_->get_logger(), "────────────────────────────────────────────────────────────────");
        std::string publish_str = "[";
        for (size_t i = 0; i < joint_angles.size(); ++i) {
            publish_str += std::to_string(joint_angles[i]) + " rad = " +
                          std::to_string(joint_angles[i] * 180.0 / M_PI) + "°";
            if (i < joint_angles.size() - 1) publish_str += ", ";
        }
        publish_str += "]";
        RCLCPP_INFO(node_->get_logger(),
                    "[HeadMarker] 📤 发布到 topic 的关节角度（应用限位后）: %s", publish_str.c_str());
        RCLCPP_INFO(node_->get_logger(), "────────────────────────────────────────────────────────────────");

        std_msgs::msg::Float64MultiArray msg;
        msg.data = joint_angles;
        joint_publisher_->publish(msg);

        // 即使强制发送，也更新节流时间，避免连续强制发送过于频繁
        if (force)
        {
            last_publish_time_ = node_->now();
        }

        return true;
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

    bool HeadMarker::shouldThrottle(double interval) const
    {
        auto now = node_->now();
        auto time_since_last = (now - last_publish_time_).seconds();

        if (time_since_last >= interval)
        {
            last_publish_time_ = now;
            return true;
        }
        return false;
    }
} // namespace arms_ros2_control::command

