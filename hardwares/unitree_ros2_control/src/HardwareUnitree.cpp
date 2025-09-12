#include "unitree_ros2_control/HardwareUnitree.h"
#include "unitree_ros2_control/QuadrupedCommunicator.h"
using hardware_interface::return_type;

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HardwareUnitree::on_init(
    const hardware_interface::HardwareInfo& info)
{
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    int joint_count = static_cast<int>(info_.joints.size());
    RCLCPP_INFO(get_logger(), "Found %d joints in configuration", joint_count);
    
    joint_torque_command_.assign(joint_count, 0);
    joint_position_command_.assign(joint_count, 0);
    joint_velocities_command_.assign(joint_count, 0);
    joint_kp_command_.assign(joint_count, 0);
    joint_kd_command_.assign(joint_count, 0);

    joint_position_.assign(joint_count, 0);
    joint_velocities_.assign(joint_count, 0);
    joint_effort_.assign(joint_count, 0);

    imu_states_.assign(10, 0);
    foot_force_.assign(4, 0);
    high_states_.assign(6, 0);

    for (const auto& joint : info_.joints)
    {
        for (const auto& interface : joint.state_interfaces)
        {
            joint_interfaces[interface.name].push_back(joint.name);
        }
    }


    if (const auto network_interface_param = info.hardware_parameters.find("network_interface"); network_interface_param
        != info.hardware_parameters.end())
    {
        network_interface_ = network_interface_param->second;
    }
    if (const auto domain_param = info.hardware_parameters.find("domain"); domain_param != info.hardware_parameters.
        end())
    {
        domain_ = std::stoi(domain_param->second);
    }
    if (const auto show_foot_force_param = info.hardware_parameters.find("show_foot_force"); show_foot_force_param !=
        info.hardware_parameters.end())
    {
        show_foot_force_ = show_foot_force_param->second == "true";
    }
    if (const auto robot_type_param = info.hardware_parameters.find("robot_type"); robot_type_param !=
        info.hardware_parameters.end())
    {
        robot_type_ = robot_type_param->second;
    }
    if (const auto enable_high_state_param = info.hardware_parameters.find("enable_high_state"); enable_high_state_param !=
        info.hardware_parameters.end())
    {
        enable_high_state_ = enable_high_state_param->second == "true";
    }

    RCLCPP_INFO(get_logger(), " robot_type: %s, network_interface: %s, domain: %d, enable_high_state: %s", 
                robot_type_.c_str(), network_interface_.c_str(), domain_, enable_high_state_ ? "true" : "false");

    initializeCommunicator();


    return SystemInterface::on_init(info);
}

std::vector<hardware_interface::StateInterface> HardwareUnitree::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    int ind = 0;
    for (const auto& joint_name : joint_interfaces["position"])
    {
        state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["velocity"])
    {
        state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["effort"])
    {
        state_interfaces.emplace_back(joint_name, "effort", &joint_effort_[ind++]);
    }

    exportSensorStateInterfaces(state_interfaces);


    return
        state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HardwareUnitree::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    int ind = 0;
    for (const auto& joint_name : joint_interfaces["position"])
    {
        command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["velocity"])
    {
        command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["effort"])
    {
        command_interfaces.emplace_back(joint_name, "effort", &joint_torque_command_[ind]);
        command_interfaces.emplace_back(joint_name, "kp", &joint_kp_command_[ind]);
        command_interfaces.emplace_back(joint_name, "kd", &joint_kd_command_[ind]);
        ind++;
    }
    return command_interfaces;
}

return_type HardwareUnitree::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    if (!communicator_) {
        RCLCPP_ERROR(get_logger(), "Communicator not initialized");
        return return_type::ERROR;
    }

    UnitreeCommunicator::RobotState robot_state;
    if (!communicator_->readState(robot_state)) {
        return return_type::OK;
    }

    int joint_count = communicator_->getJointCount();
    for (int i = 0; i < joint_count && i < static_cast<int>(joint_position_.size()); ++i) {
        joint_position_[i] = robot_state.joint_position[i];
        joint_velocities_[i] = robot_state.joint_velocity[i];
        joint_effort_[i] = robot_state.joint_effort[i];
    }

    // 第一次读取到状态时，将当前位置设置为command初始值
    if (!commands_initialized_) {
        for (int i = 0; i < joint_count && i < static_cast<int>(joint_position_command_.size()); ++i) {
            joint_position_command_[i] = joint_position_[i];
            joint_velocities_command_[i] = 0.0;  // 速度命令初始化为0
            joint_torque_command_[i] = 0.0;      // 力矩命令初始化为0
            joint_kp_command_[i] = 0.0;          // kp初始化为0
            joint_kd_command_[i] = 0.0;          // kd初始化为0
        }
        commands_initialized_ = true;
        RCLCPP_INFO(get_logger(), "Initialized command values with current joint positions");
    }

    imu_states_[0] = robot_state.imu_quaternion[0];
    imu_states_[1] = robot_state.imu_quaternion[1];
    imu_states_[2] = robot_state.imu_quaternion[2];
    imu_states_[3] = robot_state.imu_quaternion[3];
    imu_states_[4] = robot_state.imu_gyroscope[0];
    imu_states_[5] = robot_state.imu_gyroscope[1];
    imu_states_[6] = robot_state.imu_gyroscope[2];
    imu_states_[7] = robot_state.imu_accelerometer[0];
    imu_states_[8] = robot_state.imu_accelerometer[1];
    imu_states_[9] = robot_state.imu_accelerometer[2];

    if (communicator_->supportsFootForce()) {
        foot_force_[0] = robot_state.foot_force[0];
        foot_force_[1] = robot_state.foot_force[1];
        foot_force_[2] = robot_state.foot_force[2];
        foot_force_[3] = robot_state.foot_force[3];

        if (show_foot_force_) {
            RCLCPP_INFO(get_logger(), "foot_force(): %f, %f, %f, %f", 
                       foot_force_[0], foot_force_[1], foot_force_[2], foot_force_[3]);
        }
    }

    if (enable_high_state_ && communicator_->supportsHighState()) {
        high_states_[0] = robot_state.high_position[0];
        high_states_[1] = robot_state.high_position[1];
        high_states_[2] = robot_state.high_position[2];
        high_states_[3] = robot_state.high_velocity[0];
        high_states_[4] = robot_state.high_velocity[1];
        high_states_[5] = robot_state.high_velocity[2];
    }

    return return_type::OK;
}

return_type HardwareUnitree::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    if (!communicator_) {
        RCLCPP_ERROR(get_logger(), "Communicator not initialized");
        return return_type::ERROR;
    }

    // 如果命令还没有根据当前位置初始化，则不发送任何指令
    if (!commands_initialized_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
                            "Commands not initialized yet, skipping command write");
        return return_type::OK;
    }

    UnitreeCommunicator::RobotCommand command;
    int joint_count = communicator_->getJointCount();
    
    command.joint_position.resize(joint_count);
    command.joint_velocity.resize(joint_count);
    command.joint_effort.resize(joint_count);
    command.joint_kp.resize(joint_count);
    command.joint_kd.resize(joint_count);

    for (int i = 0; i < joint_count && i < static_cast<int>(joint_position_command_.size()); ++i) {
        command.joint_position[i] = joint_position_command_[i];
        command.joint_velocity[i] = joint_velocities_command_[i];
        command.joint_effort[i] = joint_torque_command_[i];
        command.joint_kp[i] = joint_kp_command_[i];
        command.joint_kd[i] = joint_kd_command_[i];
    }

    if (!communicator_->writeCommand(command)) {
        RCLCPP_ERROR(get_logger(), "Failed to write command");
        return return_type::ERROR;
    }

    return return_type::OK;
}

void HardwareUnitree::initializeCommunicator()
{
    communicator_ = UnitreeCommunicatorFactory::createCommunicator(robot_type_);
    if (!communicator_) {
        RCLCPP_ERROR(get_logger(), "Failed to create communicator for robot type: %s", robot_type_.c_str());
        return;
    }

    int config_joint_count = static_cast<int>(joint_position_.size());
    communicator_->setJointCount(config_joint_count);
    RCLCPP_INFO(get_logger(), "Set communicator joint count to: %d", config_joint_count);

    bool init_success = false;
    if (robot_type_ == "quadruped") {
        auto* quadruped_comm = dynamic_cast<QuadrupedCommunicator*>(communicator_.get());
        if (quadruped_comm) {
            init_success = quadruped_comm->initialize(domain_, network_interface_, enable_high_state_);
        } else {
            init_success = communicator_->initialize(domain_, network_interface_);
        }
    } else {
        init_success = communicator_->initialize(domain_, network_interface_);
    }
    
    if (!init_success) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize communicator");
        communicator_.reset();
        return;
    }

    RCLCPP_INFO(get_logger(), "Successfully initialized %s communicator with %d joints", 
                robot_type_.c_str(), config_joint_count);
}

void HardwareUnitree::exportSensorStateInterfaces(std::vector<hardware_interface::StateInterface>& state_interfaces) {
    hardware_interface::ComponentInfo imu_sensor;
    if (findSensorByName("imu", imu_sensor)) {
        for (uint i = 0; i < imu_sensor.state_interfaces.size() && i < imu_states_.size(); i++) {
            state_interfaces.emplace_back(
                imu_sensor.name, imu_sensor.state_interfaces[i].name, &imu_states_[i]);
        }
        RCLCPP_INFO(get_logger(), "Exported %zu IMU state interfaces", imu_sensor.state_interfaces.size());
    } else {
        RCLCPP_WARN(get_logger(), "IMU sensor not found in configuration");
    }

    if (communicator_ && communicator_->supportsFootForce()) {
        hardware_interface::ComponentInfo foot_force_sensor;
        if (findSensorByName("foot_force", foot_force_sensor)) {
            for (uint i = 0; i < foot_force_sensor.state_interfaces.size() && i < foot_force_.size(); i++) {
                state_interfaces.emplace_back(
                    foot_force_sensor.name, foot_force_sensor.state_interfaces[i].name, &foot_force_[i]);
            }
            RCLCPP_INFO(get_logger(), "Exported %zu foot force state interfaces", foot_force_sensor.state_interfaces.size());
        } else {
            RCLCPP_WARN(get_logger(), "Foot force sensor not found in configuration");
        }
    } else {
        RCLCPP_INFO(get_logger(), "Foot force sensor not supported for robot type: %s", robot_type_.c_str());
    }

    if (enable_high_state_ && communicator_ && communicator_->supportsHighState()) {
        hardware_interface::ComponentInfo high_state_sensor;
        if (findSensorByName("odometer", high_state_sensor)) {
            for (uint i = 0; i < high_state_sensor.state_interfaces.size() && i < high_states_.size(); i++) {
                state_interfaces.emplace_back(
                    high_state_sensor.name, high_state_sensor.state_interfaces[i].name, &high_states_[i]);
            }
            RCLCPP_INFO(get_logger(), "Exported %zu high state interfaces", high_state_sensor.state_interfaces.size());
        } else {
            RCLCPP_WARN(get_logger(), "High state sensor (odometer) not found in configuration");
        }
    } else {
        RCLCPP_INFO(get_logger(), "High state sensor disabled or not supported");
    }
}

bool HardwareUnitree::findSensorByName(const std::string& sensor_name, hardware_interface::ComponentInfo& sensor_info) {
    for (const auto& sensor : info_.sensors) {
        if (sensor.name.find(sensor_name) != std::string::npos) {
            sensor_info = sensor;
            return true;
        }
    }
    return false;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    HardwareUnitree, hardware_interface::SystemInterface)
