//
// Created for Basic Joint Controller
//

#include "basic_joint_controller/BasicJointController.h"
#include "basic_joint_controller/FSM/StateHold.h"
#include "basic_joint_controller/FSM/StateMoveJ.h"
#include <eigen3/Eigen/Dense>
#include <arms_controller_common/FSM/StateHome.h>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <cmath>
#include <boost/log/attributes/constant.hpp>
#include <tf2/exceptions.h>

namespace basic_joint_controller
{
    using config_type = controller_interface::interface_configuration_type;

    controller_interface::InterfaceConfiguration BasicJointController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * command_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : command_interface_types_)
            {
                if (!command_prefix_.empty())
                {
                    conf.names.push_back(command_prefix_ + "/" + joint_name + "/" + interface_type);
                }
                else
                {
                    conf.names.push_back(joint_name + "/" + interface_type);
                }
            }
        }

        return conf;
    }

    controller_interface::InterfaceConfiguration BasicJointController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}};

        conf.names.reserve(joint_names_.size() * state_interface_types_.size());
        for (const auto& joint_name : joint_names_)
        {
            for (const auto& interface_type : state_interface_types_)
            {
                conf.names.push_back(joint_name + "/" + interface_type);
            }
        }

        return conf;
    }

    controller_interface::return_type BasicJointController::update(const rclcpp::Time& time,
                                                                   const rclcpp::Duration& period)
    {
        (void)time;
        if (mode_ == FSMMode::NORMAL)
        {
            current_state_->run(time, period);
            next_state_name_ = current_state_->checkChange();
            if (next_state_name_ != current_state_->state_name)
            {
                mode_ = FSMMode::CHANGE;
                next_state_ = getNextState(next_state_name_);
                RCLCPP_INFO(get_node()->get_logger(), "Switched from %s to %s",
                            current_state_->state_name_string.c_str(), next_state_->state_name_string.c_str());
            }

            applyPendingTargetPercentPosition();
        }
        else if (mode_ == FSMMode::CHANGE)
        {
            current_state_->exit();
            current_state_ = next_state_;
            current_state_->enter();
            mode_ = FSMMode::NORMAL;
        }

        return controller_interface::return_type::OK;
    }

    void BasicJointController::applyPendingTargetPercentPosition()
    {
        std::vector<double> target;
        {
            std::lock_guard<std::mutex> lock(target_percent_mutex_);
            if (has_pending_target_percent_position_)
            {
                active_target_percent_position_ = pending_target_percent_position_;
                has_active_target_percent_position_ = true;
                has_pending_target_percent_position_ = false;
            }

            if (!has_active_target_percent_position_)
            {
                return;
            }

            target = active_target_percent_position_;
        }

        if (!current_state_ || current_state_->state_name != FSMStateName::MOVEJ)
        {
            std::lock_guard<std::mutex> lock(target_percent_mutex_);
            has_active_target_percent_position_ = false;
            return;
        }

        if (target.size() != ctrl_interfaces_.joint_position_command_interface_.size())
        {
            RCLCPP_WARN(get_node()->get_logger(),
                        "target_percent direct command size(%zu) != command interfaces size(%zu), ignoring",
                        target.size(), ctrl_interfaces_.joint_position_command_interface_.size());
            return;
        }

        for (size_t i = 0; i < target.size(); ++i)
        {
            ctrl_interfaces_.setJointPositionCommand(i, target[i]);
        }
    }

    controller_interface::CallbackReturn BasicJointController::on_init()
    {
        try
        {
            // Get controller name
            controller_name_ = get_node()->get_name();

            // Get update frequency
            get_node()->get_parameter("update_rate", ctrl_interfaces_.frequency_);
            RCLCPP_INFO(get_node()->get_logger(), "Controller Manager Update Rate: %d Hz", ctrl_interfaces_.frequency_);

            // Hardware parameters
            command_prefix_ = auto_declare<std::string>("command_prefix", command_prefix_);
            joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
            command_interface_types_ =
                auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
            state_interface_types_ =
                auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

            // Home state
            auto_declare<double>("home_duration", 3.0);
            auto_declare<std::string>("home_interpolation_type", "linear");
            auto_declare<double>("home_tanh_scale", 3.0);
            auto_declare<long>("switch_command_base", 100);

            state_list_.home = std::make_shared<arms_controller_common::StateHome>(
                ctrl_interfaces_, nullptr, get_node());
            state_list_.home->init([this](const std::string& name, const std::vector<double>& default_value)
            {
                return this->auto_declare<std::vector<double>>(name, default_value);
            });

            // HOLD state
            auto_declare<double>("hold_first_check_position_threshold", 0.1);
            auto_declare<double>("hold_position_threshold", 0.1);
            state_list_.hold = std::make_shared<StateHold>(
                ctrl_interfaces_, get_node());

            // MOVEJ state
            auto_declare<double>("movej_duration", 3.0);
            auto_declare<std::string>("movej_interpolation_type", "linear");
            auto_declare<double>("movej_tanh_scale", 3.0);
            auto_declare<double>("movej_trajectory_duration", 3.0);
            auto_declare<double>("movej_trajectory_blend_ratio", 0.0);

            state_list_.movej = std::make_shared<StateMoveJ>(
                ctrl_interfaces_, get_node(), joint_names_);

            target_command_enabled_ = auto_declare<bool>("target_command_enabled", false);
            target_command_close_config_ = auto_declare<int32_t>("target_command_close_config", 1);
            target_command_open_config_ = auto_declare<int32_t>("target_command_open_config", 0);

            if (target_command_enabled_)
            {
                RCLCPP_INFO(get_node()->get_logger(),
                            "Target command enabled: close_config=%d, open_config=%d",
                            target_command_close_config_, target_command_open_config_);
            }

            // 读取腰部升降配置
            waist_lifting_enabled_ = auto_declare<bool>("waist_lifting_enabled", false);

            // 声明腰部参数
            if (waist_lifting_enabled_)
            {
                auto_declare<double>("waist_lifting_duration", 3.0);
                auto_declare<std::vector<double>>("waist_lifting_default_parameter", {0.25, 1.0, 5.0});
                auto_declare<std::vector<double>>("waist_turning_default_parameter", {0.25, 1.0, 5.0});
                std::string waist_lifting_type_ = auto_declare<std::string>("waist_lifting_type", "three_joint");
                if (waist_lifting_type_ == "three_joint")
                {
                    auto_declare<double>("waist_l1", 0.322);
                    auto_declare<double>("waist_l2", 0.355);
                    auto_declare<std::vector<double>>("waist_rotation_direction", {1.0, 1.0, 1.0});
                    auto_declare<std::vector<double>>("waist_angle_offset", {0.0, 0.0, 0.0});
                }

                RCLCPP_INFO(get_node()->get_logger(),
                            "Waist lifting enabled for controller %s",
                            controller_name_.c_str());
            }

            return CallbackReturn::SUCCESS;
        }
        catch (const std::exception& e)
        {
            fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    controller_interface::CallbackReturn BasicJointController::on_configure(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // Subscribe to FSM command (dedicated topic for state transitions)
        fsm_command_subscription_ = get_node()->create_subscription<std_msgs::msg::Int32>(
            "/fsm_command", 10, [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                ctrl_interfaces_.fsm_command_ = msg->data;
            });


        state_list_.movej->setupSubscriptions("target_joint_position", false);
        state_list_.movej->setupTrajectorySubscription();
        robot_description_subscription_ = get_node()->create_subscription<std_msgs::msg::String>(
            "/robot_description", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                state_list_.movej->updateJointLimitsFromURDF(msg->data);
            });
        state_list_.movej->setupJointTrajectoryService("joint_trajectory_with_para");

        // 创建腰部升降规划器并传递给 StateMoveJ
        if (waist_lifting_enabled_)
        {
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_node()->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // 订阅腰部升降话题
            std::string waist_lifting_topic = "/" + controller_name_ + "/waist_lifting";

            waist_lifting_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64>(
                waist_lifting_topic, 10,
                [this](const std_msgs::msg::Float64::SharedPtr msg)
                {
                    // 只有在 MOVEJ 状态时才处理
                    if (!current_state_ || current_state_->state_name != FSMStateName::MOVEJ)
                    {
                        return;
                    }

                    // 通过 StateMoveJ 启动腰部升降
                    if (state_list_.movej)
                    {
                        bool success = state_list_.movej->moveWaistLifting(
                            Eigen::Vector3d(0.0, msg->data, 0.0)); //兼容旧接口：仅dz

                        if (success)
                        {
                            RCLCPP_INFO(get_node()->get_logger(),
                                        "Waist lifting command received: distance=%.3f",
                                        msg->data);
                        }
                        else
                        {
                            RCLCPP_WARN(get_node()->get_logger(),
                                        "waist lifting command failed");
                        }
                    }
                });
            std::string waist_lifting_pose_relative_topic =
                "/" + controller_name_ + "/waist_lifting_pose_relative";
            waist_lifting_pose_relative_subscription_ =
                get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
                waist_lifting_pose_relative_topic, 10,
                [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
                {
                    if (!current_state_ || current_state_->state_name != FSMStateName::MOVEJ)
                    {
                        return;
                    }
                    if (!state_list_.movej)
                    {
                        return;
                    }
                    if (msg->data.size() < 3)
                    {
                        RCLCPP_WARN(get_node()->get_logger(),
                                    "waist_lifting_pose_relative expects [dx, dz, dphi], got %zu values",
                                    msg->data.size());
                        return;
                    }

                    const Eigen::Vector3d lifting_delta(msg->data[0], msg->data[1], msg->data[2]);
                    bool success = state_list_.movej->moveWaistLifting(lifting_delta);
                    if (!success)
                    {
                        RCLCPP_WARN(get_node()->get_logger(), "waist lifting x/z/phi delta command failed");
                    }
                });
            std::string waist_lifting_pose_absolute_topic =
                "/" + controller_name_ + "/waist_lifting_pose_absolute";
            waist_lifting_pose_absolute_subscription_ =
                get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
                    waist_lifting_pose_absolute_topic, 10,
                    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
                    {
                        if (!current_state_ || current_state_->state_name != FSMStateName::MOVEJ)
                        {
                            return;
                        }
                        if (!state_list_.movej)
                        {
                            return;
                        }
                        if (msg->data.size() < 3)
                        {
                            RCLCPP_WARN(get_node()->get_logger(),
                                        "waist_lifting_pose_absolute expects [x, z, phi], got %zu values",
                                        msg->data.size());
                            return;
                        }
                        if (!tf_buffer_)
                        {
                            RCLCPP_WARN(get_node()->get_logger(), "TF buffer is not initialized");
                            return;
                        }

                        geometry_msgs::msg::PointStamped target_base;
                        target_base.header.frame_id = waist_absolute_source_frame_;
                        target_base.header.stamp = rclcpp::Time(0);
                        target_base.point.x = msg->data[0];
                        target_base.point.y = 0.0;
                        target_base.point.z = msg->data[1];

                        try
                        {
                            const auto transform = tf_buffer_->lookupTransform(
                                waist_absolute_target_frame_,
                                waist_absolute_source_frame_,
                                tf2::TimePointZero);

                            geometry_msgs::msg::PointStamped target_body;
                            tf2::doTransform(target_base, target_body, transform);
                            if (std::abs(target_body.point.y) > 1.0e-4)
                            {
                                RCLCPP_WARN(get_node()->get_logger(),
                                            "Transformed waist absolute target has non-zero y in %s: %.6f; using x/z only",
                                            waist_absolute_target_frame_.c_str(), target_body.point.y);
                            }

                            const Eigen::Vector3d target_xz_phi(
                                target_body.point.x, target_body.point.z, msg->data[2]);
                            if (!state_list_.movej->moveWaistLiftingToBodyBaseXz(target_xz_phi))
                            {
                                RCLCPP_WARN(get_node()->get_logger(),
                                            "waist absolute x/z/phi command failed");
                            }
                        }
                        catch (const tf2::TransformException& ex)
                        {
                            RCLCPP_WARN(get_node()->get_logger(),
                                        "Failed to transform waist absolute target from %s to %s: %s",
                                        waist_absolute_source_frame_.c_str(),
                                        waist_absolute_target_frame_.c_str(),
                                        ex.what());
                        }
                    });
            // 订阅腰部升降speedj 指令
            std::string waist_lifting_command_topic = "/" + controller_name_ + "/waist_lifting_command";

            waist_lifting_command_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64>(
                waist_lifting_command_topic, 10,
                [this](const std_msgs::msg::Float64::SharedPtr msg)
                {
                    // 只有在 MOVEJ 状态时才处理
                    if (!current_state_ || current_state_->state_name != FSMStateName::MOVEJ)
                    {
                        return;
                    }

                    // 通过 StateMoveJ 启动腰部升降
                    if (state_list_.movej)
                    {
                        bool success = state_list_.movej->setWaistLiftingFactor(msg->data);

                        if (success)
                        {
                            // RCLCPP_INFO(get_node()->get_logger(),
                            //             "Waist lifting command received: waist  %s",
                            //             cmd_msg.c_str());
                        }
                        else
                        {
                            RCLCPP_WARN(get_node()->get_logger(),
                                        "waist lifting command failed");
                        }
                    }
                });

            std::string waist_turning_command_topic = "/" + controller_name_ + "/waist_turning_command";
            waist_turning_command_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64>(
                waist_turning_command_topic, 10,
                [this](const std_msgs::msg::Float64::SharedPtr msg)
                {
                    if (!current_state_ || current_state_->state_name != FSMStateName::MOVEJ)
                    {
                        return;
                    }

                    if (state_list_.movej)
                    {
                        bool success = state_list_.movej->setWaistTurningFactor(msg->data);
                        if (!success)
                        {
                            RCLCPP_WARN(get_node()->get_logger(),
                                        "waist turning command failed");
                        }
                    }
                });
        }

        // Subscribe to target_command topic for dexterous hand control (if enabled)
        // Only effective when in MOVEJ state - directly sets target position without switching states
        if (target_command_enabled_)
        {
            std::string target_command_topic = "/" + controller_name_ + "/target_command";
            target_command_subscription_ = get_node()->create_subscription<std_msgs::msg::Int32>(
                target_command_topic, rclcpp::QoS(10),
                [this](const std_msgs::msg::Int32::SharedPtr msg)
                {
                    // Only process if current state is MOVEJ
                    if (current_state_ && current_state_->state_name == FSMStateName::MOVEJ)
                    {
                        // Map command value to configuration index
                        int32_t config_index = -1;
                        if (msg->data == 0)
                        {
                            config_index = target_command_close_config_;
                        }
                        else if (msg->data == 1)
                        {
                            config_index = target_command_open_config_;
                        }
                        else
                        {
                            RCLCPP_WARN(get_node()->get_logger(),
                                        "Invalid target_command value: %d (expected 0 or 1)", msg->data);
                            return;
                        }

                        // Validate configuration index
                        if (config_index < 0)
                        {
                            RCLCPP_WARN(get_node()->get_logger(),
                                        "Invalid configuration index: %d (must be >= 0)", config_index);
                            return;
                        }

                        // Get configuration from StateHome and set as target for StateMoveJ
                        if (state_list_.home && state_list_.movej)
                        {
                            std::vector<double> target_config = state_list_.home->getConfiguration(
                                static_cast<size_t>(config_index));

                            if (!target_config.empty())
                            {
                                std::string movej_interpolation_type = get_node()->get_parameter(
                                    "movej_interpolation_type").as_string();
                                std::transform(movej_interpolation_type.begin(), movej_interpolation_type.end(),
                                               movej_interpolation_type.begin(),
                                               [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

                                if (movej_interpolation_type == "none")
                                {
                                    std::lock_guard<std::mutex> lock(target_percent_mutex_);
                                    pending_target_percent_position_ = target_config;
                                    has_pending_target_percent_position_ = true;
                                    RCLCPP_INFO(get_node()->get_logger(),
                                                "Target command received: %d, queueing direct target configuration %d",
                                                msg->data, config_index);
                                }
                                else
                                {
                                    state_list_.movej->setTargetPosition(target_config);
                                    RCLCPP_INFO(get_node()->get_logger(),
                                                "Target command received: %d, setting MOVEJ target to configuration %d",
                                                msg->data, config_index);
                                }
                            }
                            else
                            {
                                RCLCPP_WARN(get_node()->get_logger(),
                                            "Configuration %d not available or invalid", config_index);
                            }
                        }
                    }
                    else
                    {
                        RCLCPP_DEBUG(get_node()->get_logger(),
                                     "Target command received but ignored (current state is not MOVEJ)");
                    }
                });
            RCLCPP_DEBUG(get_node()->get_logger(),
                         "Subscribed to target_command topic: %s (close_config=%d, open_config=%d)",
                         target_command_topic.c_str(), target_command_close_config_, target_command_open_config_);

            // 百分比控制：在关闭配置和打开配置之间按比例插值
            // 0.0 = 完全关闭（close_config），1.0 = 完全打开（open_config）
            // 逐关节线性插值：target[i] = close[i] + percent * (open[i] - close[i])
            std::string target_percent_topic = "/" + controller_name_ + "/target_percent";
            target_percent_subscription_ = get_node()->create_subscription<std_msgs::msg::Float64>(
                target_percent_topic, rclcpp::QoS(10),
                [this](const std_msgs::msg::Float64::SharedPtr msg)
                {
                    if (!current_state_ || current_state_->state_name != FSMStateName::MOVEJ)
                    {
                        RCLCPP_DEBUG(get_node()->get_logger(),
                                     "target_percent received but ignored (current state is not MOVEJ)");
                        return;
                    }

                    double percent = msg->data;
                    if (percent < 0.0 || percent > 1.0)
                    {
                        RCLCPP_WARN(get_node()->get_logger(),
                                    "target_percent %.3f out of range [0.0, 1.0], clamping",
                                    percent);
                        percent = std::clamp(percent, 0.0, 1.0);
                    }

                    if (!state_list_.home || !state_list_.movej)
                    {
                        return;
                    }

                    const std::vector<double> close_config = state_list_.home->getConfiguration(
                        static_cast<size_t>(target_command_close_config_));
                    const std::vector<double> open_config = state_list_.home->getConfiguration(
                        static_cast<size_t>(target_command_open_config_));

                    if (close_config.empty() || open_config.empty())
                    {
                        RCLCPP_WARN(get_node()->get_logger(),
                                    "target_percent: close_config(%d) or open_config(%d) not available",
                                    target_command_close_config_, target_command_open_config_);
                        return;
                    }

                    if (close_config.size() != open_config.size())
                    {
                        RCLCPP_WARN(get_node()->get_logger(),
                                    "target_percent: close_config size(%zu) != open_config size(%zu), ignoring",
                                    close_config.size(), open_config.size());
                        return;
                    }

	                    // 逐关节线性插值
	                    std::vector<double> target(close_config.size());
	                    for (size_t i = 0; i < target.size(); ++i)
	                    {
	                        target[i] = close_config[i] + percent * (open_config[i] - close_config[i]);
	                    }
	
	                    std::string movej_interpolation_type = get_node()->get_parameter(
	                        "movej_interpolation_type").as_string();
	                    std::transform(movej_interpolation_type.begin(), movej_interpolation_type.end(),
	                                   movej_interpolation_type.begin(),
	                                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

	                    if (movej_interpolation_type == "none")
	                    {
	                        std::lock_guard<std::mutex> lock(target_percent_mutex_);
	                        pending_target_percent_position_ = target;
	                        has_pending_target_percent_position_ = true;
	                        RCLCPP_DEBUG(get_node()->get_logger(),
	                                     "target_percent %.1f%% -> direct target queued (%zu joints)",
	                                     percent * 100.0, target.size());
	                    }
	                    else
	                    {
	                        state_list_.movej->setTargetPosition(target);
	                        RCLCPP_INFO(get_node()->get_logger(),
	                                    "target_percent %.1f%% -> interpolated target set (%zu joints)",
	                                    percent * 100.0, target.size());
	                    }
	                });

            RCLCPP_DEBUG(get_node()->get_logger(),
                         "Subscribed to target_percent topic: %s (0.0=close_config[%d], 1.0=open_config[%d])",
                         target_percent_topic.c_str(), target_command_close_config_, target_command_open_config_);
        }
        else
        {
            RCLCPP_DEBUG(get_node()->get_logger(), "Target command subscription disabled");
        }


        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BasicJointController::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        // clear out vectors in case of restart
        ctrl_interfaces_.clear();

        // assign command interfaces
        for (auto& interface : command_interfaces_)
        {
            std::string interface_name = interface.get_interface_name();
            if (const size_t pos = interface_name.find('/'); pos != std::string::npos)
            {
                command_interface_map_[interface_name.substr(pos + 1)]->push_back(interface);
            }
            else
            {
                command_interface_map_[interface_name]->push_back(interface);
            }
        }

        // assign state interfaces
        for (auto& interface : state_interfaces_)
        {
            state_interface_map_[interface.get_interface_name()]->push_back(interface);
        }

        // Initialize last sent joint positions from current state
        // This ensures we have a baseline for smooth transitions to HOLD state
        ctrl_interfaces_.initializeLastSentPositions();

        // Initialize FSM
        current_state_ = state_list_.hold;
        current_state_->enter();
        next_state_ = current_state_;
        next_state_name_ = current_state_->state_name;
        mode_ = FSMMode::NORMAL;

        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BasicJointController::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        release_interfaces();
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BasicJointController::on_cleanup(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BasicJointController::on_error(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn BasicJointController::on_shutdown(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        return CallbackReturn::SUCCESS;
    }

    std::shared_ptr<FSMState> BasicJointController::getNextState(FSMStateName stateName) const
    {
        switch (stateName)
        {
        case FSMStateName::INVALID:
            return state_list_.invalid;
        case FSMStateName::HOME:
            return state_list_.home;
        case FSMStateName::HOLD:
            return state_list_.hold;
        case FSMStateName::MOVEJ:
            return state_list_.movej;
        default:
            return state_list_.invalid;
        }
    }
} // namespace basic_joint_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(basic_joint_controller::BasicJointController, controller_interface::ControllerInterface);
