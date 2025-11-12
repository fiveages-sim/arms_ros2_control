#include "eyou_ros2_control/eyou_hardware.h"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <thread>
#include <chrono>

namespace eyou_ros2_control {

hardware_interface::CallbackReturn EyouHardware::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
    if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    // 解析关节名称
    std::vector<std::string> joint_names;
    for (const auto& joint : info_.joints) {
        joint_names.push_back(joint.name);
    }
    size_t motor_count = joint_names.size();
    
    // 初始化数据存储
    joint_positions_.resize(motor_count, 0.0);
    joint_velocities_.resize(motor_count, 0.0);
    joint_efforts_.resize(motor_count, 0.0);
    joint_position_commands_.resize(motor_count, 0.0);
    raw_position_commands_.resize(motor_count, 0);
    raw_positions_.resize(motor_count, 0);
    raw_efforts_.resize(motor_count, 0);
    last_positions_.resize(motor_count, 0.0);
    last_raw_positions_.resize(motor_count, 0);
    last_read_time_ = rclcpp::Clock().now();
    
    // 解析配置参数
    const auto get_param = [this](const std::string& name, const std::string& default_val) {
        if (auto it = info_.hardware_parameters.find(name); 
            it != info_.hardware_parameters.end()) {
            return it->second;
        }
        return default_val;
    };
    
    // CAN设备配置
    std::string device_type_str = get_param("device_type", "Canable");
    if (device_type_str == "Canable") {
        device_type_ = harmonic_DeviceType_Canable;
    } else if (device_type_str == "USB2CAN") {
        device_type_ = harmonic_DeviceType_USB2CAN;
    } else {
        RCLCPP_ERROR(get_node()->get_logger(), "Unknown device type: %s", device_type_str.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    device_index_ = static_cast<huint8>(std::stoi(get_param("device_index", "0")));
    
    int baudrate_val = std::stoi(get_param("baudrate", "1000"));
    switch (baudrate_val) {
        case 1000: baudrate_ = harmonic_Baudrate_1000; break;
        case 500: baudrate_ = harmonic_Baudrate_500; break;
        case 250: baudrate_ = harmonic_Baudrate_250; break;
        default:
            RCLCPP_ERROR(get_node()->get_logger(), "Unsupported baudrate: %d", baudrate_val);
            return hardware_interface::CallbackReturn::ERROR;
    }
    
    // 电机ID列表
    std::string motor_ids_str = get_param("motor_ids", "");
    if (motor_ids_str.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "motor_ids parameter is required");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    // 解析motor_ids字符串，格式: [5, 6] 或 5,6
    motor_ids_str.erase(std::remove(motor_ids_str.begin(), motor_ids_str.end(), '['), motor_ids_str.end());
    motor_ids_str.erase(std::remove(motor_ids_str.begin(), motor_ids_str.end(), ']'), motor_ids_str.end());
    std::istringstream iss(motor_ids_str);
    std::string token;
    while (std::getline(iss, token, ',')) {
        motor_ids_.push_back(static_cast<huint8>(std::stoi(token)));
    }
    
    if (motor_ids_.size() != motor_count) {
        RCLCPP_ERROR(get_node()->get_logger(), 
                    "Motor count mismatch: %zu motors configured but %zu joints defined",
                    motor_ids_.size(), motor_count);
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    control_period_ms_ = std::stoi(get_param("control_period_ms", "4"));
    report_period_ms_ = std::stoi(get_param("report_period_ms", "100"));
    use_sync_ = (get_param("use_sync", "true") == "true");
    
    // 解析减速比数组（如果提供）
    std::vector<double> reduction_ratios;
    std::string reduction_ratios_str = get_param("reduction_ratios", "");
    if (!reduction_ratios_str.empty()) {
        // 解析数组格式: [50.0, 30.0] 或 50.0,30.0
        std::string cleaned = reduction_ratios_str;
        cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), '['), cleaned.end());
        cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), ']'), cleaned.end());
        std::istringstream iss(cleaned);
        std::string token;
        while (std::getline(iss, token, ',')) {
            reduction_ratios.push_back(std::stod(token));
        }
        
        if (reduction_ratios.size() != motor_ids_.size()) {
            RCLCPP_ERROR(get_node()->get_logger(), 
                        "Reduction ratios count (%zu) doesn't match motor count (%zu)",
                        reduction_ratios.size(), motor_ids_.size());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    
    // 解析电机配置（减速比）
    if (!parseMotorConfigs(reduction_ratios)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse motor configs");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    running_ = false;
    
    RCLCPP_INFO(get_node()->get_logger(), "==============================================");
    RCLCPP_INFO(get_node()->get_logger(), "EyouHardware initialized successfully");
    RCLCPP_INFO(get_node()->get_logger(), "  Device Type: %s", device_type_str.c_str());
    RCLCPP_INFO(get_node()->get_logger(), "  Device Index: %d", device_index_);
    RCLCPP_INFO(get_node()->get_logger(), "  Baudrate: %d kbps", baudrate_val);
    RCLCPP_INFO(get_node()->get_logger(), "  Motor Count: %zu", motor_ids_.size());
    RCLCPP_INFO(get_node()->get_logger(), "  Control Period: %d ms", control_period_ms_);
    RCLCPP_INFO(get_node()->get_logger(), "  Report Period: %d ms", report_period_ms_);
    RCLCPP_INFO(get_node()->get_logger(), "  Use Sync: %s", use_sync_ ? "true" : "false");
    RCLCPP_INFO(get_node()->get_logger(), "==============================================");
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EyouHardware::on_activate(
    const rclcpp_lifecycle::State & /* previous_state */) {
    RCLCPP_INFO(get_node()->get_logger(), "Activating EyouHardware...");
    
    // 1. 初始化CAN设备
    if (!initializeCAN()) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    // 2. 配置PDO
    if (!configurePDO()) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    // 3. 设置电机模式（CSP模式）
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        huint8 id = motor_ids_[i];
        if (HARMONIC_SUCCESS != harmonic_setOperateMode(device_index_, id, 
                                                         harmonic_OperateMode_CyclicSyncPosition)) {
            RCLCPP_ERROR(get_node()->get_logger(), 
                        "Failed to set CSP mode for motor %d", id);
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        if (HARMONIC_SUCCESS != harmonic_setInterpolationTimePeriodValue(device_index_, id, 
                                                                         control_period_ms_)) {
            RCLCPP_ERROR(get_node()->get_logger(), 
                        "Failed to set interpolation period for motor %d", id);
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        if (HARMONIC_SUCCESS != harmonic_setSyncCounter(device_index_, id, 0)) {
            RCLCPP_ERROR(get_node()->get_logger(), 
                        "Failed to set sync counter for motor %d", id);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    
    // 4. 使能电机
    if (!enableMotors()) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    // 5. 读取初始位置
    readMotorStates();
    
    // 将初始位置设置为命令位置（避免启动时突然运动）
    for (size_t i = 0; i < joint_position_commands_.size(); ++i) {
        joint_position_commands_[i] = joint_positions_[i];
        raw_position_commands_[i] = raw_positions_[i];
    }
    
    // 6. 启动CSP控制线程
    running_ = true;
    csp_control_thread_ = std::thread(&EyouHardware::cspControlLoop, this);
    RCLCPP_INFO(get_node()->get_logger(), "✅ EyouHardware activated with control enabled!");
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EyouHardware::on_deactivate(
    const rclcpp_lifecycle::State & /* previous_state */) {
    RCLCPP_INFO(get_node()->get_logger(), "Deactivating EyouHardware...");
    
    // 停止控制线程（如果已启动）
    if (running_) {
        running_ = false;
        if (csp_control_thread_.joinable()) {
            csp_control_thread_.join();
        }
    }
    
    // 停止所有电机
    for (huint8 id : motor_ids_) {
        harmonic_stopControl(device_index_, id);
    }
    
    // 释放CAN设备
    harmonic_freeDLL(device_index_);
    
    RCLCPP_INFO(get_node()->get_logger(), "✅ EyouHardware deactivated");
    
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr> 
EyouHardware::on_export_state_interfaces() {
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
    
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const std::string& joint_name = info_.joints[i].name;
        
        state_interfaces.push_back(
            std::make_shared<hardware_interface::StateInterface>(
                joint_name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
        
        state_interfaces.push_back(
            std::make_shared<hardware_interface::StateInterface>(
                joint_name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));
        
        state_interfaces.push_back(
            std::make_shared<hardware_interface::StateInterface>(
                joint_name, hardware_interface::HW_IF_EFFORT, &joint_efforts_[i]));
    }
    
    RCLCPP_INFO(get_node()->get_logger(), 
               "Exported %zu state interfaces for %zu joints", 
               state_interfaces.size(), info_.joints.size());
    
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr> 
EyouHardware::on_export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
    
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        const std::string& joint_name = info_.joints[i].name;
        
        command_interfaces.push_back(
            std::make_shared<hardware_interface::CommandInterface>(
                joint_name, hardware_interface::HW_IF_POSITION, &joint_position_commands_[i]));
    }
    
    RCLCPP_INFO(get_node()->get_logger(), 
               "Exported %zu command interfaces for %zu joints", 
               command_interfaces.size(), info_.joints.size());
    
    return command_interfaces;
}

hardware_interface::return_type EyouHardware::read(
    const rclcpp::Time & /* time */, 
    const rclcpp::Duration & /* period */) {
    readMotorStates();
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type EyouHardware::write(
    const rclcpp::Time & /* time */, 
    const rclcpp::Duration & /* period */) {
    // 将弧度转换为编码器计数
    for (size_t i = 0; i < joint_position_commands_.size(); ++i) {
        raw_position_commands_[i] = radianToEncoder(i, joint_position_commands_[i]);
    }
    
    return hardware_interface::return_type::OK;
}

// === 辅助函数实现 ===

void EyouHardware::calculateConversionScales(MotorConfig& config) {
    // 电机一圈脉冲分辨率 = 65536 * 减速比
    // 编码器原始值：电机轴转一圈 = 65536 脉冲
    // 关节轴转一圈 = 电机轴转 减速比 圈 = 65536 * 减速比 脉冲
    // encoder = radian × reduction_ratio × encoder_resolution / (2π)
    // 其中 encoder_resolution = 65536（电机轴每圈脉冲数）
    config.position_scale_to_encoder = 
        config.reduction_ratio * ENCODER_RESOLUTION / (2.0 * M_PI);
    
    // radian = encoder / position_scale_to_encoder
    config.position_scale_to_radian = 1.0 / config.position_scale_to_encoder;
}

hint32 EyouHardware::radianToEncoder(size_t motor_index, double radian) {
    const auto& config = motor_configs_[motor_index];
    double encoder_double = radian * config.position_scale_to_encoder;
    return static_cast<hint32>(std::round(encoder_double));
}

double EyouHardware::encoderToRadian(size_t motor_index, hint32 encoder) {
    const auto& config = motor_configs_[motor_index];
    return static_cast<double>(encoder) * config.position_scale_to_radian;
}

double EyouHardware::encoderPerSecToRadianPerSec(size_t motor_index, double encoder_per_sec) {
    const auto& config = motor_configs_[motor_index];
    return encoder_per_sec * config.position_scale_to_radian;
}

bool EyouHardware::parseMotorConfigs(const std::vector<double>& reduction_ratios) {
    motor_configs_.clear();
    motor_configs_.resize(motor_ids_.size());
    
    const auto get_param = [this](const std::string& name, const std::string& default_val) {
        if (auto it = info_.hardware_parameters.find(name); 
            it != info_.hardware_parameters.end()) {
            return it->second;
        }
        return default_val;
    };
    
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        huint8 motor_id = motor_ids_[i];
        motor_configs_[i].motor_id = motor_id;
        
        // 优先使用数组中的减速比，否则从单独的参数读取
        double reduction_ratio = 1.0;
        if (!reduction_ratios.empty() && i < reduction_ratios.size()) {
            // 使用数组中的值
            reduction_ratio = reduction_ratios[i];
        } else {
            // 从单独的参数读取（向后兼容）
            std::string param_name = "motor_" + std::to_string(motor_id) + ".reduction_ratio";
            reduction_ratio = std::stod(get_param(param_name, "1.0"));
        }
        
        motor_configs_[i].reduction_ratio = reduction_ratio;
        
        // 计算转换系数
        calculateConversionScales(motor_configs_[i]);
        
        RCLCPP_INFO(get_node()->get_logger(), 
                   "Motor %d: reduction_ratio=%.1f, encoder_scale=%.2f, radian_scale=%.6f",
                   motor_id,
                   motor_configs_[i].reduction_ratio,
                   motor_configs_[i].position_scale_to_encoder,
                   motor_configs_[i].position_scale_to_radian);
    }
    
    return true;
}

EyouHardware::MotorConfig EyouHardware::parseMotorConfig(huint8 motor_id, double reduction_ratio) {
    MotorConfig config;
    config.motor_id = motor_id;
    config.reduction_ratio = reduction_ratio;
    return config;
}

bool EyouHardware::initializeCAN() {
    if (HARMONIC_SUCCESS != harmonic_initDLL(device_type_, device_index_, baudrate_)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize CAN device");
        return false;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    RCLCPP_INFO(get_node()->get_logger(), "CAN device initialized");
    return true;
}

bool EyouHardware::configurePDO() {
    // 配置TPDO（从站发送给主站）
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        huint8 id = motor_ids_[i];
        
        // 无效化TPDO
        if (HARMONIC_SUCCESS != harmonic_setTPDOCobId(device_index_, id, 0, 
                                                      (0x80 << 24) + 0x180 + id)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to set TPDO COB-ID for motor %d", id);
            return false;
        }
        
        if (HARMONIC_SUCCESS != harmonic_setTPDOTransmitType(device_index_, id, 0, 0xFF)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to set TPDO transmit type for motor %d", id);
            return false;
        }
        
        if (HARMONIC_SUCCESS != harmonic_setTPDOEventTimer(device_index_, id, 0, report_period_ms_)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to set TPDO event timer for motor %d", id);
            return false;
        }
    }
    
    // 配置LocalRPDO（主站接收从站数据）
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        huint8 id = motor_ids_[i];
        size_t local_index = i;
        
        // 无效化LocalRPDO
        harmonic_setLocalRPDOCobId(local_index, (0x80 << 24) + 0x180 + id);
        harmonic_setLocalRPDOMaxMappedCount(local_index, 0);
        harmonic_setLocalRPDOTransmitType(local_index, 0xFF);
        harmonic_setLocalRPDOInhibitTime(local_index, 0);
        harmonic_setLocalRPDOEventTimer(local_index, report_period_ms_);
        harmonic_setLocalRPDOSYNCStartValue(local_index, 0);
        
        // 配置映射：位置(0x5000)和力矩(0x5002)
        harmonic_setLocalRPDOMapped(local_index, 0, 
                                   (0x5000 << 16) + ((0x1 + local_index) << 8) + 0x20);
        harmonic_setLocalRPDOMapped(local_index, 1, 
                                   (0x5002 << 16) + ((0x1 + local_index) << 8) + 0x10);
        harmonic_setLocalRPDOMaxMappedCount(local_index, 2);
        
        // 激活LocalRPDO
        harmonic_setLocalRPDOCobId(local_index, 0x180 + id);
    }
    
    // 配置远程TPDO映射
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        huint8 id = motor_ids_[i];
        
        // 配置位置映射(0x6064)和力矩映射(0x6077)
        if (HARMONIC_SUCCESS != harmonic_setTPDOMapped(device_index_, id, 0, 0, 
                                                        (0x6064 << 16) + 0x20)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to set TPDO position mapping for motor %d", id);
            return false;
        }
        
        if (HARMONIC_SUCCESS != harmonic_setTPDOMapped(device_index_, id, 0, 1, 
                                                        (0x6077 << 16) + 0x10)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to set TPDO torque mapping for motor %d", id);
            return false;
        }
        
        if (HARMONIC_SUCCESS != harmonic_setTPDOMaxMappedCount(device_index_, id, 0, 2)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to set TPDO max mapped count for motor %d", id);
            return false;
        }
        
        // 激活TPDO
        if (HARMONIC_SUCCESS != harmonic_setTPDOCobId(device_index_, id, 0, 0x180 + id)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to activate TPDO for motor %d", id);
            return false;
        }
    }
    
    // 重置和启动节点
    harmonic_setNodeState(device_index_, 0, harmonic_NMTState_Reset_Node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    harmonic_setNodeState(device_index_, 0, harmonic_NMTState_Start_Node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    RCLCPP_INFO(get_node()->get_logger(), "PDO configuration completed");
    return true;
}

bool EyouHardware::enableMotors() {
    huint8 type = use_sync_ ? 1 : 0xFF;
    
    // 配置RPDO（主站发送给从站）
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        huint8 id = motor_ids_[i];
        
        // 无效化RPDO
        if (HARMONIC_SUCCESS != harmonic_setRPDOCobId(device_index_, id, 0, 
                                                      (0x80 << 24) + 0x200 + id)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to invalidate RPDO for motor %d", id);
            return false;
        }
        
        if (HARMONIC_SUCCESS != harmonic_setRPDOMaxMappedCount(device_index_, id, 0, 0)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to clear RPDO mapping for motor %d", id);
            return false;
        }
        
        if (HARMONIC_SUCCESS != harmonic_setRPDOTransmitType(device_index_, id, 0, type)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to set RPDO transmit type for motor %d", id);
            return false;
        }
        
        // 配置位置映射(0x607A)
        if (HARMONIC_SUCCESS != harmonic_setRPDOMapped(device_index_, id, 0, 0, 
                                                        (0x607A << 16) + 0x20)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to set RPDO position mapping for motor %d", id);
            return false;
        }
        
        if (HARMONIC_SUCCESS != harmonic_setRPDOMaxMappedCount(device_index_, id, 0, 1)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to set RPDO max mapped count for motor %d", id);
            return false;
        }
        
        // 激活RPDO
        if (HARMONIC_SUCCESS != harmonic_setRPDOCobId(device_index_, id, 0, 0x200 + id)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to activate RPDO for motor %d", id);
            return false;
        }
    }
    
    // 使能电机（状态机转换）
    for (huint8 id : motor_ids_) {
        if (HARMONIC_SUCCESS != harmonic_setControlword(device_index_, id, 0x06)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to set controlword 0x06 for motor %d", id);
            return false;
        }
    }
    
    for (huint8 id : motor_ids_) {
        if (HARMONIC_SUCCESS != harmonic_setControlword(device_index_, id, 0x07)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to set controlword 0x07 for motor %d", id);
            return false;
        }
    }
    
    for (huint8 id : motor_ids_) {
        if (HARMONIC_SUCCESS != harmonic_setControlword(device_index_, id, 0x0F)) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to set controlword 0x0F for motor %d", id);
            return false;
        }
    }
    
    // 再次重置和启动节点
    harmonic_setNodeState(device_index_, 0, harmonic_NMTState_Reset_Node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    harmonic_setNodeState(device_index_, 0, harmonic_NMTState_Start_Node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    RCLCPP_INFO(get_node()->get_logger(), "Motors enabled");
    return true;
}

void EyouHardware::cspControlLoop() {
    RCLCPP_INFO(get_node()->get_logger(), "CSP control thread started");
    
    auto period = std::chrono::milliseconds(control_period_ms_);
    auto next_time = std::chrono::steady_clock::now() + period;
    
    while (running_) {
        // 发送位置指令到所有电机
        for (size_t i = 0; i < motor_ids_.size(); ++i) {
            setPos(device_index_, motor_ids_[i], raw_position_commands_[i]);
        }
        
        // 发送SYNC帧（如果启用同步）
        if (use_sync_) {
            static const huint8 sync_data[1] = {0};
            harmonic_writeCanData(device_index_, 0x80, sync_data, 1);
        }
        
        // 精确周期控制
        std::this_thread::sleep_until(next_time);
        next_time += period;
    }
    
    RCLCPP_INFO(get_node()->get_logger(), "CSP control thread stopped");
}

void EyouHardware::readMotorStates() {
    for (size_t i = 0; i < motor_ids_.size(); ++i) {
        // 读取位置（从本地字典0x5000）
        hint32 raw_pos = 0;
        if (HARMONIC_SUCCESS == harmonic_getLocalMutiMotorPos(i, &raw_pos)) {
            raw_positions_[i] = raw_pos;
            joint_positions_[i] = encoderToRadian(i, raw_pos);
        }
        
        // 读取力矩（从本地字典0x5002）
        hint16 raw_torque = 0;
        if (HARMONIC_SUCCESS == harmonic_getLocalMutiMotorTorque(i, &raw_torque)) {
            raw_efforts_[i] = raw_torque;
            joint_efforts_[i] = static_cast<double>(raw_torque) * EFFORT_SCALE;
        }
        
        // 速度计算（差分）
        rclcpp::Time current_time = rclcpp::Clock().now();
        double dt = (current_time - last_read_time_).seconds();
        
        if (dt > 0.001) {  // 至少1ms
            double encoder_per_sec = 
                static_cast<double>(raw_positions_[i] - last_raw_positions_[i]) / dt;
            joint_velocities_[i] = encoderPerSecToRadianPerSec(i, encoder_per_sec);
        }
        
        last_positions_[i] = joint_positions_[i];
        last_raw_positions_[i] = raw_positions_[i];
    }
    last_read_time_ = rclcpp::Clock().now();
}

void EyouHardware::setPos(huint8 dev_index, unsigned id, hint32 pos) {
    huint8 data[4];
    data[0] = (unsigned)pos & 0x000000ff;
    data[1] = ((unsigned)pos & 0x0000ff00) >> 8;
    data[2] = ((unsigned)pos & 0x00ff0000) >> 16;
    data[3] = ((unsigned)pos & 0xff000000) >> 24;
    harmonic_writeCanData(dev_index, 0x200 + id, data, 4);
}

} // namespace eyou_ros2_control

PLUGINLIB_EXPORT_CLASS(eyou_ros2_control::EyouHardware, hardware_interface::SystemInterface)

