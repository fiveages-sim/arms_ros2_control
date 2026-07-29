#include "arxlift2s_ros2_control/arx_x5_hardware.h"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <unistd.h>
#include <sstream>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace arxlift2s_ros2_control {

    static const std::vector<double> kDefaultJointKGains = {20.0, 20.0, 20.0, 20.0, 10.0, 10.0};
    static const std::vector<double> kDefaultJointDGains = {3.5, 3.5, 3.5, 3.5, 1.0, 1.0};
    // position defaults — same as arx-ros2-control
    static const std::vector<double> kPositionJointKGains = {80.0, 70.0, 70.0, 30.0, 30.0, 20.0};
    static const std::vector<double> kPositionJointDGains = {2.0, 2.0, 2.0, 1.0, 1.0, 0.7};
    static const double kDefaultGripperKP = 5.0;
    static const double kDefaultGripperKD = 0.2;

    static constexpr double kGripperPosScaleToRos = 0.5;

    static std::vector<double> parseDoubleArrayLoose(const std::string& str, const std::vector<double>& default_val)
    {
        if (str.empty()) {
            return default_val;
        }
        std::string cleaned = str;
        cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), '['), cleaned.end());
        cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), ']'), cleaned.end());
        std::replace(cleaned.begin(), cleaned.end(), ',', ' ');

        std::istringstream iss(cleaned);
        std::vector<double> result;
        double v = 0.0;
        while (iss >> v) {
            result.push_back(v);
        }
        return result.empty() ? default_val : result;
    }

    static bool vectorsNearlyEqual(const std::vector<double>& a, const std::vector<double>& b, double eps = 1e-9)
    {
        if (a.size() != b.size()) {
            return false;
        }
        for (size_t i = 0; i < a.size(); ++i) {
            if (std::abs(a[i] - b[i]) > eps) {
                return false;
            }
        }
        return true;
    }

void ArxX5Hardware::declare_node_parameters()
{
    const auto hw_find = [this](const std::string& name) -> const std::string* {
        auto it = info_.hardware_parameters.find(name);
        if (it == info_.hardware_parameters.end()) {
            return nullptr;
        }
        return &it->second;
    };

    const auto ensure_string_param = [this](const std::string& name, const std::string& default_val, const std::string* hw_val) {
        if (node_->has_parameter(name)) {
            if (node_->get_parameter(name).get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
                try {
                    node_->undeclare_parameter(name);
                } catch (...) {}
            } else {
                return;
            }
        }

        if (!node_->has_parameter(name)) {
            const std::string val = hw_val ? *hw_val : default_val;
            node_->declare_parameter<std::string>(name, val);
        }
    };

    const auto ensure_double_param = [this](const std::string& name, double default_val, const std::string* hw_val) {
        if (node_->has_parameter(name)) {
            if (node_->get_parameter(name).get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                try {
                    node_->undeclare_parameter(name);
                } catch (...) {}
            } else {
                return;
            }
        }

        if (!node_->has_parameter(name)) {
            double val = default_val;
            if (hw_val) {
                try {
                    val = std::stod(*hw_val);
                } catch (...) {
                    val = default_val;
                }
            }
            node_->declare_parameter<double>(name, val);
        }
    };

    const auto ensure_double_array_sized = [this, &hw_find](const std::string& name,
                                                           const std::vector<double>& default_val,
                                                           size_t expected_size) {
        const std::string* hw_val = hw_find(name);

        if (node_->has_parameter(name)) {
            if (node_->get_parameter(name).get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
                try {
                    node_->undeclare_parameter(name);
                } catch (...) {}
            } else {
                try {
                    const auto current = node_->get_parameter(name).get_value<std::vector<double>>();
                    if (current.size() == expected_size) {
                        return;
                    }
                } catch (...) {}
                try {
                    node_->undeclare_parameter(name);
                } catch (...) {}
            }
        }

        if (!node_->has_parameter(name)) {
            std::vector<double> val = hw_val ? parseDoubleArrayLoose(*hw_val, default_val) : default_val;
            if (val.size() != expected_size) {
                val = default_val;
            }
            node_->declare_parameter<std::vector<double>>(name, val);
        }
    };

    // can_name is accepted as an alias of can_interface (legacy official xacro).
    const std::string * can_hw = hw_find("can_interface");
    if (!can_hw) {
        can_hw = hw_find("can_name");
    }
    ensure_string_param("robot_model", "X5", hw_find("robot_model"));
    ensure_string_param("can_interface", "can0", can_hw);
    ensure_string_param("control_mode", "full_control", hw_find("control_mode"));

    // Code-level fallbacks if URDF omitted gains: position → arx-ros2-control values.
    const std::string mode_raw =
      hw_find("control_mode") ? *hw_find("control_mode") : "full_control";
    const std::string mode_norm = normalizeControlMode(mode_raw);
    const bool use_position_gains = (mode_norm == "position");
    const auto & kp_fallback =
      use_position_gains ? kPositionJointKGains : kDefaultJointKGains;
    const auto & kd_fallback =
      use_position_gains ? kPositionJointDGains : kDefaultJointDGains;
    ensure_double_array_sized("joint_k_gains", kp_fallback, 6);
    ensure_double_array_sized("joint_d_gains", kd_fallback, 6);
    // Dual gain sets (auto-selected by control_mode)
    ensure_double_array_sized("joint_k_gains_full_control", kDefaultJointKGains, 6);
    ensure_double_array_sized("joint_d_gains_full_control", kDefaultJointDGains, 6);
    ensure_double_array_sized("joint_k_gains_position", kPositionJointKGains, 6);
    ensure_double_array_sized("joint_d_gains_position", kPositionJointDGains, 6);
    ensure_double_param("gripper_kp", kDefaultGripperKP, hw_find("gripper_kp"));
    ensure_double_param("gripper_kd", kDefaultGripperKD, hw_find("gripper_kd"));

    const auto ensure_bool_param = [this](const std::string & name, bool default_val,
                                          const std::string * hw_val) {
        if (node_->has_parameter(name)) {
            if (node_->get_parameter(name).get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                try {
                    node_->undeclare_parameter(name);
                } catch (...) {}
            } else {
                return;
            }
        }
        if (!node_->has_parameter(name)) {
            bool val = default_val;
            if (hw_val) {
                const std::string s = *hw_val;
                val = (s == "true" || s == "1" || s == "True" || s == "yes" || s == "on");
            }
            node_->declare_parameter<bool>(name, val);
        }
    };
    ensure_bool_param("status_debug", false, hw_find("status_debug"));
    // position 严格语义：torque=0；如需保留 OCS2 MIX 提供的重力补偿，可设为 true。
    ensure_bool_param("position_forward_effort", false, hw_find("position_forward_effort"));
}

hardware_interface::CallbackReturn ArxX5Hardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {

    if (hardware_interface::SystemInterface::on_init(params) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    node_ = get_node();
    logger_ = get_node()->get_logger();
    declare_node_parameters();
    robot_model_ = get_node_param("robot_model", std::string("X5"));
    can_interface_ = get_node_param("can_interface", std::string("can0"));
    if (info_.hardware_parameters.count("urdf_path") ||
      info_.hardware_parameters.count("end_type"))
    {
        RCLCPP_WARN(
            get_logger(),
            "Ignoring official-arm params urdf_path/end_type — "
            "ArxX5Hardware now uses Stanford arx5-sdk (can_interface + control_mode).");
    }
    // Modes: URDF always exports MIX IFs; mode only changes write().
    //   full_control — OCS2 MIX pos/vel/effort; MIT kp/kd from HI joint_k/d_gains
    //   position     — pos only (vel=0); by default torque=0 (strict reference).
    //                  If ``position_forward_effort=true``, effort will be forwarded as torque.
    //   pd_control   — HT-compatible alias → position
    // Live switch: ros2 param set <arm_hi_node> control_mode full_control|position
    {
        const std::string raw =
          get_node_param("control_mode", std::string("full_control"));
        const std::string normalized = normalizeControlMode(raw);
        if (normalized.empty()) {
            RCLCPP_WARN(
              get_logger(),
              "Unknown control_mode '%s'; using 'full_control'. "
              "Supported: full_control | position | pd_control",
              raw.c_str());
            control_mode_ = "full_control";
        } else {
            control_mode_ = normalized;
        }
    }

    has_gripper_ = false;
    gripper_joint_names_.clear();
    joint_names_.clear();

    for (const auto& joint : params.hardware_info.joints) {
        std::string joint_name_lower = joint.name;
        std::transform(joint_name_lower.begin(), joint_name_lower.end(),
                       joint_name_lower.begin(), ::tolower);

        if (joint_name_lower.find("gripper") != std::string::npos) {
            has_gripper_ = true;
            gripper_joint_names_.push_back(joint.name);
        } else {
            joint_names_.push_back(joint.name);
        }
    }

    joint_count_ = joint_names_.size();

    position_states_.resize(joint_count_, 0.0);
    velocity_states_.resize(joint_count_, 0.0);
    effort_states_.resize(joint_count_, 0.0);
    position_commands_.resize(joint_count_, 0.0);
    velocity_commands_.resize(joint_count_, 0.0);
    effort_commands_.resize(joint_count_, 0.0);
    kp_commands_.resize(joint_count_, 0.0);
    kd_commands_.resize(joint_count_, 0.0);

    if (has_gripper_) {
        const size_t gripper_count = gripper_joint_names_.size();
        gripper_position_states_.resize(gripper_count, 0.0);
        gripper_velocity_states_.resize(gripper_count, 0.0);
        gripper_effort_states_.resize(gripper_count, 0.0);
        gripper_position_commands_.resize(gripper_count, 0.0);
    }

    RCLCPP_INFO(get_logger(),
        "ArxX5Hardware init: model=%s can=%s control_mode=%s joints=%zu gripper=%s",
        robot_model_.c_str(), can_interface_.c_str(), control_mode_.c_str(),
        joint_count_, has_gripper_ ? "yes" : "no");

    controller_.reset();
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr> ArxX5Hardware::on_export_state_interfaces() {
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;

    for (size_t i = 0; i < joint_count_; ++i) {
        state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &position_states_[i]));
        state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
            joint_names_[i], hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));
        state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
            joint_names_[i], hardware_interface::HW_IF_EFFORT, &effort_states_[i]));
    }

    if (has_gripper_) {
        for (size_t i = 0; i < gripper_joint_names_.size(); ++i) {
            state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
                gripper_joint_names_[i], hardware_interface::HW_IF_POSITION, &gripper_position_states_[i]));
            state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
                gripper_joint_names_[i], hardware_interface::HW_IF_VELOCITY, &gripper_velocity_states_[i]));
            state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
                gripper_joint_names_[i], hardware_interface::HW_IF_EFFORT, &gripper_effort_states_[i]));
        }
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr> ArxX5Hardware::on_export_command_interfaces() {
    // Always export the same set as interfaces.xacro (position/velocity/effort/kp/kd for arm).
    // control_mode_ only changes how write() uses those buffers — not which are advertised.
    std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;

    for (size_t i = 0; i < joint_count_; ++i) {
        command_interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
            joint_names_[i], hardware_interface::HW_IF_POSITION, &position_commands_[i]));
        command_interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
            joint_names_[i], hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
        command_interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
            joint_names_[i], hardware_interface::HW_IF_EFFORT, &effort_commands_[i]));
        command_interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
            joint_names_[i], "kp", &kp_commands_[i]));
        command_interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
            joint_names_[i], "kd", &kd_commands_[i]));
    }

    if (has_gripper_) {
        for (size_t i = 0; i < gripper_joint_names_.size(); ++i) {
            command_interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
                gripper_joint_names_[i], hardware_interface::HW_IF_POSITION, &gripper_position_commands_[i]));
        }
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn ArxX5Hardware::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {

    // Keep backward compatibility: old params are still accepted, but active gains will be selected by control_mode.
    std::vector<double> current_kp = get_node_param("joint_k_gains", kDefaultJointKGains);
    std::vector<double> current_kd = get_node_param("joint_d_gains", kDefaultJointDGains);
    if (current_kp.size() != 6) {
        RCLCPP_WARN(get_logger(), "joint_k_gains size is %zu, expected 6; using default for cache.", current_kp.size());
        current_kp = kDefaultJointKGains;
    }
    if (current_kd.size() != 6) {
        RCLCPP_WARN(get_logger(), "joint_d_gains size is %zu, expected 6; using default for cache.", current_kd.size());
        current_kd = kDefaultJointDGains;
    }

    // New split gain sets (auto-selected by control_mode_).
    joint_k_gains_full_control_ = get_node_param("joint_k_gains_full_control", kDefaultJointKGains);
    joint_d_gains_full_control_ = get_node_param("joint_d_gains_full_control", kDefaultJointDGains);
    joint_k_gains_position_ = get_node_param("joint_k_gains_position", kPositionJointKGains);
    joint_d_gains_position_ = get_node_param("joint_d_gains_position", kPositionJointDGains);

    auto fix_size = [this](std::vector<double> & v, const std::vector<double> & fallback, const char* name) {
        if (v.size() != 6) {
            RCLCPP_WARN(get_logger(), "%s size is %zu, expected 6; using default for cache.", name, v.size());
            v = fallback;
        }
    };
    fix_size(joint_k_gains_full_control_, kDefaultJointKGains, "joint_k_gains_full_control");
    fix_size(joint_d_gains_full_control_, kDefaultJointDGains, "joint_d_gains_full_control");
    fix_size(joint_k_gains_position_, kPositionJointKGains, "joint_k_gains_position");
    fix_size(joint_d_gains_position_, kPositionJointDGains, "joint_d_gains_position");

    // Active gains: initially seed from old params to preserve your existing tuning.
    if (control_mode_ == "position") {
        joint_k_gains_ = current_kp;
        joint_d_gains_ = current_kd;
    } else {
        joint_k_gains_ = current_kp;
        joint_d_gains_ = current_kd;
    }
    gripper_kp_ = get_node_param("gripper_kp", kDefaultGripperKP);
    gripper_kd_ = get_node_param("gripper_kd", kDefaultGripperKD);
    status_debug_.store(get_node_param("status_debug", false));
    position_forward_effort_.store(get_node_param("position_forward_effort", false));

    // Seed command gains from parameter defaults (OCS2 may overwrite in full_control).
    for (size_t i = 0; i < joint_count_; ++i) {
        kp_commands_[i] = (i < joint_k_gains_.size()) ? joint_k_gains_[i] : kDefaultJointKGains[std::min(i, kDefaultJointKGains.size() - 1)];
        kd_commands_[i] = (i < joint_d_gains_.size()) ? joint_d_gains_[i] : kDefaultJointDGains[std::min(i, kDefaultJointDGains.size() - 1)];
        velocity_commands_[i] = 0.0;
        effort_commands_[i] = 0.0;
    }

    param_callback_handle_ = node_->add_on_set_parameters_callback(
        std::bind(&ArxX5Hardware::paramCallback, this, std::placeholders::_1));

    try {
        controller_ = std::make_shared<arx::Arx5JointController>(robot_model_, can_interface_);
        hardware_connected_ = true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to connect or probe hardware: %s", e.what());
        hardware_connected_ = false;
        controller_.reset();
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArxX5Hardware::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {

    if (!hardware_connected_) {
        RCLCPP_ERROR(get_logger(), "Cannot activate: hardware not connected (configure first).");
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (!controller_) {
        RCLCPP_ERROR(get_logger(), "Cannot activate: controller is null.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    const int max_read_attempts = 10;
    const int read_interval_ms = 100;
    auto has_invalid_values = [](const arx::JointState& state, size_t count) -> bool {
        for (size_t i = 0; i < count && i < static_cast<size_t>(state.pos.size()); ++i) {
            if (std::isnan(state.pos[i]) || std::isinf(state.pos[i])) {
                return true;
            }
        }
        return false;
    };

    try {
        controller_->reset_to_home();

        bool initial_read_success = false;
        arx::JointState initial_state(joint_count_);

        for (int attempt = 0; attempt < max_read_attempts; ++attempt) {
            try {
                initial_state = controller_->get_joint_state();
                if (has_invalid_values(initial_state, joint_count_)) {
                    RCLCPP_WARN(get_logger(),
                        "Initial joint state contains NaN/Inf (attempt %d/%d), retrying...",
                        attempt + 1, max_read_attempts);
                    usleep(read_interval_ms * 1000);
                    continue;
                }
                initial_read_success = true;
                break;
            } catch (const std::exception& e) {
                RCLCPP_WARN(get_logger(),
                    "Failed to read initial joint state (attempt %d/%d): %s",
                    attempt + 1, max_read_attempts, e.what());
                usleep(read_interval_ms * 1000);
            }
        }

        if (!initial_read_success) {
            RCLCPP_ERROR(get_logger(), "Failed to read valid initial joint state after %d attempts",
                        max_read_attempts);
            controller_.reset();
            return hardware_interface::CallbackReturn::ERROR;
        }

        for (size_t i = 0; i < joint_count_ && i < static_cast<size_t>(initial_state.pos.size()); ++i) {
            position_states_[i] = initial_state.pos[i];
            velocity_states_[i] = initial_state.vel[i];
            effort_states_[i] = initial_state.torque[i];
            position_commands_[i] = initial_state.pos[i];
            velocity_commands_[i] = 0.0;
            effort_commands_[i] = 0.0;
        }
        if (has_gripper_ && gripper_joint_names_.size() >= 1) {
            gripper_position_states_[0] = initial_state.gripper_pos * kGripperPosScaleToRos;
            gripper_position_commands_[0] = initial_state.gripper_pos * kGripperPosScaleToRos;
        }

        cmd_buffer_.emplace(joint_count_);
        last_applied_kp_.clear();
        last_applied_kd_.clear();
        last_applied_gripper_kp_ = -1.0;
        last_applied_gripper_kd_ = -1.0;
        applyGains(joint_k_gains_, joint_d_gains_, gripper_kp_, gripper_kd_, true);

        control_active_ = true;
        RCLCPP_INFO(get_logger(), "ArxX5Hardware activated (control_mode=%s)", control_mode_.c_str());
        return hardware_interface::CallbackReturn::SUCCESS;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to activate: %s", e.what());
        control_active_ = false;
        hardware_connected_ = false;
        controller_.reset();
        return hardware_interface::CallbackReturn::ERROR;
    }
}

hardware_interface::CallbackReturn ArxX5Hardware::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {

    control_active_ = false;
    cmd_buffer_.reset();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArxX5Hardware::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {

    param_callback_handle_.reset();
    control_active_ = false;
    if (hardware_connected_) {
        RCLCPP_WARN(get_logger(), "Hardware still connected during cleanup, disconnecting...");
    }
    controller_.reset();
    hardware_connected_ = false;
    cmd_buffer_.reset();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArxX5Hardware::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/) {

    control_active_ = false;
    if (hardware_connected_) {
        RCLCPP_WARN(get_logger(), "Hardware still connected during shutdown, disconnecting...");
    }
    controller_.reset();
    hardware_connected_ = false;
    cmd_buffer_.reset();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArxX5Hardware::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/) {

    RCLCPP_ERROR(get_logger(), "Error in ArxX5 Hardware Interface");
    control_active_ = false;
    hardware_connected_ = false;
    controller_.reset();
    cmd_buffer_.reset();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArxX5Hardware::read(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

    if (!control_active_) {
        return hardware_interface::return_type::OK;
    }
    if (!hardware_connected_ || !controller_) {
        return hardware_interface::return_type::ERROR;
    }

    auto validate_and_update = [this](double raw, double& state, const char* name, int idx) {
        if (std::isnan(raw) || std::isinf(raw)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *node_->get_clock(), 1000,
                "%s[%d] is NaN/Inf, keeping previous value: %.3f", name, idx, state);
        } else {
            state = raw;
        }
    };

    try {
        arx::JointState state = controller_->get_joint_state();
        for (size_t i = 0; i < joint_count_ && i < static_cast<size_t>(state.pos.size()); ++i) {
            validate_and_update(state.pos[i], position_states_[i], "Joint position", static_cast<int>(i));
            validate_and_update(state.vel[i], velocity_states_[i], "Joint velocity", static_cast<int>(i));
            validate_and_update(state.torque[i], effort_states_[i], "Joint effort", static_cast<int>(i));
        }
        if (has_gripper_ && gripper_joint_names_.size() >= 1) {
            validate_and_update(state.gripper_pos * kGripperPosScaleToRos, gripper_position_states_[0], "Gripper position", 0);
            validate_and_update(state.gripper_vel * kGripperPosScaleToRos, gripper_velocity_states_[0], "Gripper velocity", 0);
            validate_and_update(state.gripper_torque, gripper_effort_states_[0], "Gripper effort", 0);
        }
        return hardware_interface::return_type::OK;

    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *node_->get_clock(), 1000,
                              "Read failed: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }
}

hardware_interface::return_type ArxX5Hardware::write(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

    if (!control_active_) {
        return hardware_interface::return_type::OK;
    }
    if (!hardware_connected_ || !controller_) {
        return hardware_interface::return_type::ERROR;
    }
    if (!cmd_buffer_) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *node_->get_clock(), 1000, "Command buffer not initialized");
        return hardware_interface::return_type::ERROR;
    }

    try {
        arx::JointState& cmd = *cmd_buffer_;

        for (size_t i = 0; i < joint_count_; ++i) {
            double pos = position_commands_[i];
            if (!std::isfinite(pos)) {
                pos = position_states_[i];
                position_commands_[i] = pos;
            }
            cmd.pos[i] = pos;

            if (isFullControl()) {
                double vel = velocity_commands_[i];
                if (!std::isfinite(vel)) {
                    vel = 0.0;
                    velocity_commands_[i] = 0.0;
                }
                double eff = effort_commands_[i];
                if (!std::isfinite(eff)) {
                    eff = 0.0;
                    effort_commands_[i] = 0.0;
                }
                cmd.vel[i] = vel;
                cmd.torque[i] = eff;
            } else {
                cmd.vel[i] = 0.0;
                if (position_forward_effort_.load()) {
                    double eff = effort_commands_[i];
                    if (!std::isfinite(eff)) {
                        eff = 0.0;
                        effort_commands_[i] = 0.0;
                    }
                    cmd.torque[i] = eff;
                } else {
                    cmd.torque[i] = 0.0;
                }
            }
        }

        if (has_gripper_ && gripper_joint_names_.size() >= 1) {
            double gpos = gripper_position_commands_[0];
            if (!std::isfinite(gpos)) {
                gpos = gripper_position_states_[0];
                gripper_position_commands_[0] = gpos;
            }
            cmd.gripper_pos = gpos * 2.0;
        }

        // MIT kp/kd from HI joint_k/d_gains (live: same arm-node rqt path as control_mode).
        std::vector<double> kp;
        std::vector<double> kd;
        double gripper_kp;
        double gripper_kd;
        {
            std::lock_guard<std::mutex> lock(gains_mutex_);
            kp = joint_k_gains_;
            kd = joint_d_gains_;
            gripper_kp = gripper_kp_;
            gripper_kd = gripper_kd_;
        }
        for (size_t i = 0; i < joint_count_; ++i) {
            if (i < kp.size()) {
                kp_commands_[i] = kp[i];
            }
            if (i < kd.size()) {
                kd_commands_[i] = kd[i];
            }
        }
        applyGains(kp, kd, gripper_kp, gripper_kd, false);

        if (status_debug_.load() && joint_count_ > 0) {
            std::string mode;
            bool full_control = false;
            {
                std::lock_guard<std::mutex> lock(gains_mutex_);
                mode = control_mode_;
                full_control = (control_mode_ == "full_control");
            }
            RCLCPP_INFO_THROTTLE(
              get_logger(), *node_->get_clock(), 2000,
              "ArxX5Hardware can=%s mode=%s j1 pos=%.3f cmd=%.3f vel=%.3f "
              "eff=%.3f grip=%.3f",
              can_interface_.c_str(), mode.c_str(),
              position_states_[0], position_commands_[0],
              full_control ? velocity_commands_[0] : 0.0,
              full_control ? effort_commands_[0] : 0.0,
              has_gripper_ ? gripper_position_states_[0] : 0.0);
        }

        // Near-current timestamp → init_fixed (no interpolator lag) for high-rate OCS2.
        cmd.timestamp = controller_->get_timestamp();
        controller_->set_joint_cmd(cmd);
        return hardware_interface::return_type::OK;

    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *node_->get_clock(), 1000,
                              "Write failed: %s", e.what());
        return hardware_interface::return_type::ERROR;
    }
}

std::string ArxX5Hardware::normalizeControlMode(const std::string & raw)
{
    if (raw == "full_control") {
        return "full_control";
    }
    // Backward-compatible alias (deprecated): pd_control -> position
    if (raw == "position" || raw == "pd_control") {
        return "position";
    }
    return {};
}

bool ArxX5Hardware::isFullControl() const
{
    std::lock_guard<std::mutex> lock(gains_mutex_);
    return control_mode_ == "full_control";
}

rcl_interfaces::msg::SetParametersResult ArxX5Hardware::paramCallback(
    const std::vector<rclcpp::Parameter> & params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : params) {
        if (param.get_name() == "control_mode") {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
                result.successful = false;
                result.reason = "control_mode must be a string";
                return result;
            }
            const std::string normalized = normalizeControlMode(param.as_string());
            if (normalized.empty()) {
                result.successful = false;
                result.reason =
                  "control_mode must be 'full_control' or 'position' (pd_control is deprecated alias)";
                return result;
            }
            {
                std::lock_guard<std::mutex> lock(gains_mutex_);
                const bool changed = (control_mode_ != normalized);
                control_mode_ = normalized;

                // Auto-select active gains set.
                if (control_mode_ == "position") {
                    joint_k_gains_ = joint_k_gains_position_;
                    joint_d_gains_ = joint_d_gains_position_;
                } else {
                    joint_k_gains_ = joint_k_gains_full_control_;
                    joint_d_gains_ = joint_d_gains_full_control_;
                }

                if (changed) {
                    RCLCPP_INFO(get_logger(), "control_mode -> %s", control_mode_.c_str());
                }

                if (hardware_connected_ && control_active_) {
                    applyGains(joint_k_gains_, joint_d_gains_, gripper_kp_, gripper_kd_, true);
                }
            }
        }
        else if (param.get_name() == "joint_k_gains") {
            std::vector<double> new_kp = param.as_double_array();
            const size_t expected_count = 6;

            if (new_kp.size() != expected_count) {
                result.successful = false;
                result.reason = "joint_k_gains must have exactly " + std::to_string(expected_count) + " values (for 6-joint robot)";
                return result;
            }

            {
                std::lock_guard<std::mutex> lock(gains_mutex_);
                // Backward-compatible alias: override gains for current control_mode only.
                if (control_mode_ == "position") {
                    joint_k_gains_position_ = new_kp;
                    joint_k_gains_ = new_kp;
                } else {
                    joint_k_gains_full_control_ = new_kp;
                    joint_k_gains_ = new_kp;
                }
            }
            RCLCPP_INFO(get_logger(), "Updated joint_k_gains via param server");
            if (hardware_connected_ && control_active_) {
                applyGains(new_kp, joint_d_gains_, gripper_kp_, gripper_kd_, true);
            }
        }
        else if (param.get_name() == "joint_d_gains") {
            std::vector<double> new_kd = param.as_double_array();
            const size_t expected_count = 6;
            if (new_kd.size() != expected_count) {
                result.successful = false;
                result.reason = "joint_d_gains must have exactly " + std::to_string(expected_count) + " values (for 6-joint robot)";
                return result;
            }
            {
                std::lock_guard<std::mutex> lock(gains_mutex_);
                // Backward-compatible alias: override gains for current control_mode only.
                if (control_mode_ == "position") {
                    joint_d_gains_position_ = new_kd;
                    joint_d_gains_ = new_kd;
                } else {
                    joint_d_gains_full_control_ = new_kd;
                    joint_d_gains_ = new_kd;
                }
            }
            RCLCPP_INFO(get_logger(), "Updated joint_d_gains via param server");
            if (hardware_connected_ && control_active_) {
                applyGains(joint_k_gains_, new_kd, gripper_kp_, gripper_kd_, true);
            }
        }
        else if (param.get_name() == "joint_k_gains_full_control") {
            std::vector<double> new_kp = param.as_double_array();
            const size_t expected_count = 6;
            if (new_kp.size() != expected_count) {
                result.successful = false;
                result.reason = "joint_k_gains_full_control must have exactly " + std::to_string(expected_count) +
                                 " values (for 6-joint robot)";
                return result;
            }
            bool apply_now = false;
            {
                std::lock_guard<std::mutex> lock(gains_mutex_);
                joint_k_gains_full_control_ = new_kp;
                if (control_mode_ == "full_control") {
                    joint_k_gains_ = new_kp;
                    apply_now = true;
                }
            }
            RCLCPP_INFO(get_logger(), "Updated joint_k_gains_full_control via param server");
            if (hardware_connected_ && control_active_ && apply_now) {
                applyGains(new_kp, joint_d_gains_, gripper_kp_, gripper_kd_, true);
            }
        }
        else if (param.get_name() == "joint_d_gains_full_control") {
            std::vector<double> new_kd = param.as_double_array();
            const size_t expected_count = 6;
            if (new_kd.size() != expected_count) {
                result.successful = false;
                result.reason = "joint_d_gains_full_control must have exactly " + std::to_string(expected_count) +
                                 " values (for 6-joint robot)";
                return result;
            }
            bool apply_now = false;
            {
                std::lock_guard<std::mutex> lock(gains_mutex_);
                joint_d_gains_full_control_ = new_kd;
                if (control_mode_ == "full_control") {
                    joint_d_gains_ = new_kd;
                    apply_now = true;
                }
            }
            RCLCPP_INFO(get_logger(), "Updated joint_d_gains_full_control via param server");
            if (hardware_connected_ && control_active_ && apply_now) {
                applyGains(joint_k_gains_, new_kd, gripper_kp_, gripper_kd_, true);
            }
        }
        else if (param.get_name() == "joint_k_gains_position") {
            std::vector<double> new_kp = param.as_double_array();
            const size_t expected_count = 6;
            if (new_kp.size() != expected_count) {
                result.successful = false;
                result.reason = "joint_k_gains_position must have exactly " + std::to_string(expected_count) +
                                 " values (for 6-joint robot)";
                return result;
            }
            bool apply_now = false;
            {
                std::lock_guard<std::mutex> lock(gains_mutex_);
                joint_k_gains_position_ = new_kp;
                if (control_mode_ == "position") {
                    joint_k_gains_ = new_kp;
                    apply_now = true;
                }
            }
            RCLCPP_INFO(get_logger(), "Updated joint_k_gains_position via param server");
            if (hardware_connected_ && control_active_ && apply_now) {
                applyGains(new_kp, joint_d_gains_, gripper_kp_, gripper_kd_, true);
            }
        }
        else if (param.get_name() == "joint_d_gains_position") {
            std::vector<double> new_kd = param.as_double_array();
            const size_t expected_count = 6;
            if (new_kd.size() != expected_count) {
                result.successful = false;
                result.reason = "joint_d_gains_position must have exactly " + std::to_string(expected_count) +
                                 " values (for 6-joint robot)";
                return result;
            }
            bool apply_now = false;
            {
                std::lock_guard<std::mutex> lock(gains_mutex_);
                joint_d_gains_position_ = new_kd;
                if (control_mode_ == "position") {
                    joint_d_gains_ = new_kd;
                    apply_now = true;
                }
            }
            RCLCPP_INFO(get_logger(), "Updated joint_d_gains_position via param server");
            if (hardware_connected_ && control_active_ && apply_now) {
                applyGains(joint_k_gains_, new_kd, gripper_kp_, gripper_kd_, true);
            }
        }
        else if (param.get_name() == "gripper_kp") {
            double new_gripper_kp = param.as_double();
            if (new_gripper_kp < 0.0) {
                result.successful = false;
                result.reason = "gripper_kp must be >= 0";
                return result;
            }
            {
                std::lock_guard<std::mutex> lock(gains_mutex_);
                gripper_kp_ = new_gripper_kp;
            }
            if (hardware_connected_ && control_active_) {
                applyGains(joint_k_gains_, joint_d_gains_, new_gripper_kp, gripper_kd_, true);
            }
        }
        else if (param.get_name() == "gripper_kd") {
            double new_gripper_kd = param.as_double();
            if (new_gripper_kd < 0.0) {
                result.successful = false;
                result.reason = "gripper_kd must be >= 0";
                return result;
            }
            {
                std::lock_guard<std::mutex> lock(gains_mutex_);
                gripper_kd_ = new_gripper_kd;
            }
            if (hardware_connected_ && control_active_) {
                applyGains(joint_k_gains_, joint_d_gains_, gripper_kp_, new_gripper_kd, true);
            }
        }
        else if (param.get_name() == "status_debug") {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                result.successful = false;
                result.reason = "status_debug must be a bool";
                return result;
            }
            status_debug_.store(param.as_bool());
            RCLCPP_INFO(
              get_logger(), "status_debug -> %s",
              status_debug_.load() ? "true" : "false");
        }
        else if (param.get_name() == "position_forward_effort") {
            if (param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                result.successful = false;
                result.reason = "position_forward_effort must be a bool";
                return result;
            }
            position_forward_effort_.store(param.as_bool());
            RCLCPP_INFO(
              get_logger(), "position_forward_effort -> %s",
              position_forward_effort_.load() ? "true" : "false");
        }
    }

    return result;
}

void ArxX5Hardware::applyGains(const std::vector<double>& kp, const std::vector<double>& kd,
                               double gripper_kp, double gripper_kd, bool force)
{
    if (!controller_) {
        RCLCPP_WARN(get_logger(), "Controller not initialized, cannot apply gains");
        return;
    }
    const size_t joint_count = 6;
    if (kp.size() < joint_count || kd.size() < joint_count) {
        RCLCPP_ERROR(get_logger(),
            "Gain arrays too short: kp.size()=%zu, kd.size()=%zu, expected %zu",
            kp.size(), kd.size(), joint_count);
        return;
    }

    if (!force &&
        vectorsNearlyEqual(kp, last_applied_kp_) &&
        vectorsNearlyEqual(kd, last_applied_kd_) &&
        std::abs(gripper_kp - last_applied_gripper_kp_) < 1e-9 &&
        std::abs(gripper_kd - last_applied_gripper_kd_) < 1e-9) {
        return;
    }

    try {
        arx::Gain gain(static_cast<int>(joint_count));
        for (size_t i = 0; i < joint_count; ++i) {
            gain.kp[i] = kp[i];
            gain.kd[i] = kd[i];
        }
        gain.gripper_kp = static_cast<float>(gripper_kp);
        gain.gripper_kd = static_cast<float>(gripper_kd);
        controller_->set_gain(gain);
        last_applied_kp_ = kp;
        last_applied_kd_ = kd;
        last_applied_gripper_kp_ = gripper_kp;
        last_applied_gripper_kd_ = gripper_kd;
        if (status_debug_.load()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *node_->get_clock(), 2000,
                "Applied MIT gains kp=[%.1f, ...] kd=[%.2f, ...] (gripper kp=%.1f kd=%.2f)",
                kp[0], kd[0], gripper_kp, gripper_kd);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to apply gains: %s", e.what());
    }
}

}  // namespace arxlift2s_ros2_control

PLUGINLIB_EXPORT_CLASS(arxlift2s_ros2_control::ArxX5Hardware, hardware_interface::SystemInterface)
