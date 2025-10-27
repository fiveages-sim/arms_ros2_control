#include "dobot_ros2_control/command.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>

CRCommanderRos2::CRCommanderRos2(const std::string &ip, bool verbose)
    : current_joint_{}, tool_vector_{}, is_running_(false), verbose_(verbose)
{
    is_running_ = false;
    real_time_data_ = std::make_shared<RealTimeData>();
    real_time_tcp_ = std::make_shared<TcpClient>(ip, 30004);
    dash_board_tcp_ = std::make_shared<TcpClient>(ip, 29999);
}

CRCommanderRos2::~CRCommanderRos2()
{
    is_running_ = false;
    if (thread_ && thread_->joinable()) {
        thread_->join();
    }
}

void CRCommanderRos2::getCurrentJointStatus(double *joint)
{
    mutex_.lock();
    memcpy(joint, current_joint_, sizeof(current_joint_));
    mutex_.unlock();
}

void CRCommanderRos2::getToolVectorActual(double *val)
{
    mutex_.lock();
    memcpy(val, tool_vector_, sizeof(tool_vector_));
    mutex_.unlock();
}

void CRCommanderRos2::recvTask()
{
    uint32_t has_read;
    
    // 频率统计变量
    int recv_count = 0;
    auto last_print_time = std::chrono::steady_clock::now();
    
    while (is_running_)
    {
        if (real_time_tcp_->isConnect())
        {
            try
            {
                uint8_t *tmpData = reinterpret_cast<uint8_t *>(real_time_data_.get());
                if (real_time_tcp_->tcpRecv(tmpData, sizeof(RealTimeData), has_read, 5000))
                {

                    if (real_time_data_->len != 1440)
                        continue;

                    mutex_.lock();
                    for (uint32_t i = 0; i < 6; i++)
                        current_joint_[i] = deg2Rad(real_time_data_->q_actual[i]);

                    memcpy(tool_vector_, real_time_data_->tool_vector_actual, sizeof(tool_vector_));
                    mutex_.unlock();
                    
                    // 统计接收频率
                    recv_count++;
                }
                else
                {
                    std::cout << "tcp recv timeout" << std::endl;
                }
            }
            catch (const TcpClientException &err)
            {
                real_time_tcp_->disConnect();
                std::cout << "tcp recv error :" << std::endl;
            }
        }
        else
        {
            try
            {
                real_time_tcp_->connect();
            }
            catch (const TcpClientException &err)
            {
                std::cout << "tcp recv Error : %s" << std::endl;
                sleep(3);
            }
        }

        if (!dash_board_tcp_->isConnect())
        {
            try
            {
                dash_board_tcp_->connect();
            }
            catch (const TcpClientException &err)
            {

                std::cout << "tcp recv ERROR : %s" << std::endl;
                sleep(3);
            }
        }
        
        // 每秒输出一次接收频率（仅在 verbose 模式下显示）
        if (verbose_) {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - last_print_time).count();
            
            if (elapsed >= 1000) {  // 每1000毫秒（1秒）输出一次
                double frequency = recv_count * 1000.0 / elapsed;
                std::cout << "[TCP Recv] Frequency: " << std::fixed << std::setprecision(2) 
                          << frequency << " Hz (" << recv_count << " msgs in " 
                          << elapsed << " ms)" << std::endl;
                
                // 重置计数器
                recv_count = 0;
                last_print_time = current_time;
            }
        }
    }
}

void CRCommanderRos2::init()
{
    try
    {
        is_running_ = true;
        thread_ = std::unique_ptr<std::thread>(new std::thread(&CRCommanderRos2::recvTask, this));
    }
    catch (const TcpClientException &err)
    {
        std::cout << "Commander : %s" << std::endl;
    }
}

int stringToInt(const std::string& str) {
    return std::atoi(str.c_str());
}

void CRCommanderRos2::doTcpCmd(std::shared_ptr<TcpClient> &tcp, const char *cmd, int32_t &err_id,
                               std::vector<std::string> &result)
{
    std::ignore = result;
    try
    {
        uint32_t has_read;
        std::string response;  // 使用动态字符串，无固定大小限制
        char recv_buf[1024];   // 临时接收缓冲区
        
        // 发送命令（静默模式，不输出）
        tcp->tcpSend(cmd, strlen(cmd));
        
        while (true)
        {
            memset(recv_buf, 0, sizeof(recv_buf));
            bool err = tcp->tcpRecv(recv_buf, sizeof(recv_buf) - 1, has_read, 0);
            if (!err)
            {
                sleep(0.01);
                continue;
            }
            
            // 追加到响应字符串
            response.append(recv_buf);
            
            // 检查是否接收到结束符
            if (!response.empty() && response.back() == ';')
                break;
            
            // 防止无限增长（最大10MB）
            if (response.size() > 10 * 1024 * 1024) {
                std::cout << "[TCP Error] Response too large (>10MB)" << std::endl;
                err_id = -1;
                return;
            }
        }
        
        // 解析最后一个命令的响应
        std::string target_cmd(cmd);
        size_t cmd_pos = response.rfind(target_cmd);
        
        if (cmd_pos != std::string::npos) {
            // 找到命令，提取其响应部分
            size_t start_pos = response.rfind(';', cmd_pos);
            if (start_pos == std::string::npos) {
                start_pos = 0;
            } else {
                start_pos++;
            }
            
            size_t end_pos = response.find(';', cmd_pos);
            if (end_pos != std::string::npos) {
                std::string cmd_response = response.substr(start_pos, end_pos - start_pos);
                
                // 解析错误码: "ErrorID,{...},Command"
                size_t first_comma = cmd_response.find(',');
                if (first_comma != std::string::npos) {
                    std::string err_str = cmd_response.substr(0, first_comma);
                    err_id = stringToInt(err_str);
                    
                    // 只在出错时输出
                    if (err_id != 0) {
                        std::cout << "[TCP Error] Command: " << cmd << std::endl;
                        std::cout << "[TCP Error] ErrorID: " << err_id << std::endl;
                        std::cout << "[TCP Error] Response segment: " << cmd_response << std::endl;
                    }
                }
            }
        } else {
            // 没找到命令，尝试解析最后一个响应
            size_t last_semicolon = response.rfind(';');
            size_t second_last_semicolon = response.rfind(';', last_semicolon - 1);
            
            if (second_last_semicolon != std::string::npos) {
                std::string last_response = response.substr(second_last_semicolon + 1, 
                                                            last_semicolon - second_last_semicolon - 1);
                
                size_t first_comma = last_response.find(',');
                if (first_comma != std::string::npos) {
                    std::string err_str = last_response.substr(0, first_comma);
                    err_id = stringToInt(err_str);
                }
            }
            
            if (err_id != 0) {
                std::cout << "[TCP Warning] Command not found in response" << std::endl;
            }
        }
    }
    catch (const std::logic_error &err)
    {
        std::cout << "[TCP Exception] tcpDoCmd failed: " << err.what() << std::endl;
    }
}


void CRCommanderRos2::doTcpCmd_f(std::shared_ptr<TcpClient> &tcp, const char *cmd, int32_t &err_id,std::string &mode_id,
                               std::vector<std::string> &result)
{
    std::ignore = result;
    try
    {
        uint32_t has_read;
        std::string response;  // 使用动态字符串，无固定大小限制
        char recv_buf[1024];   // 临时接收缓冲区
        
        // 发送命令（静默模式，不输出）
        tcp->tcpSend(cmd, strlen(cmd));
        
        while (true)
        {
            memset(recv_buf, 0, sizeof(recv_buf));
            bool err = tcp->tcpRecv(recv_buf, sizeof(recv_buf) - 1, has_read, 0);
            if (!err)
            {
                sleep(0.01);
                continue;
            }
            
            // 追加到响应字符串
            response.append(recv_buf);
            
            // 检查是否接收到结束符
            if (!response.empty() && response.back() == ';')
                break;
            
            // 防止无限增长（最大10MB）
            if (response.size() > 10 * 1024 * 1024) {
                std::cout << "[TCP Error] Response too large (>10MB)" << std::endl;
                err_id = -1;
                return;
            }
        }
        
        // 解析最后一个命令的响应（从后往前找，因为我们的命令通常在最后）
        // 格式: ErrorID,{data},Command;
        std::string target_cmd(cmd);
        size_t cmd_pos = response.rfind(target_cmd);  // 从后往前找命令
        
        if (cmd_pos != std::string::npos) {
            // 找到命令，提取其响应部分
            // 向前找这个命令之前的分号或开头
            size_t start_pos = response.rfind(';', cmd_pos);
            if (start_pos == std::string::npos) {
                start_pos = 0;
            } else {
                start_pos++;  // 跳过分号
            }
            
            // 向后找命令之后的分号
            size_t end_pos = response.find(';', cmd_pos);
            if (end_pos != std::string::npos) {
                std::string cmd_response = response.substr(start_pos, end_pos - start_pos);
                
                // 解析错误码和数据: "ErrorID,{data},Command"
                size_t first_comma = cmd_response.find(',');
                if (first_comma != std::string::npos) {
                    std::string err_str = cmd_response.substr(0, first_comma);
                    err_id = stringToInt(err_str);
                    
                    // 提取 {data} 部分
                    size_t brace_start = cmd_response.find('{', first_comma);
                    size_t brace_end = cmd_response.find('}', brace_start);
                    
                    if (brace_start != std::string::npos && brace_end != std::string::npos) {
                        mode_id = cmd_response.substr(brace_start, brace_end - brace_start + 1);
                    }
                    
                    // 只在出错时输出
                    if (err_id != 0) {
                        std::cout << "[TCP Error] Command: " << cmd << std::endl;
                        std::cout << "[TCP Error] ErrorID: " << err_id << std::endl;
                        std::cout << "[TCP Error] Response segment: " << cmd_response << std::endl;
                    }
                }
            }
        } else {
            // 没找到命令，尝试解析最后一个响应
            size_t last_semicolon = response.rfind(';');
            size_t second_last_semicolon = response.rfind(';', last_semicolon - 1);
            
            if (second_last_semicolon != std::string::npos) {
                std::string last_response = response.substr(second_last_semicolon + 1, 
                                                            last_semicolon - second_last_semicolon - 1);
                
                size_t first_comma = last_response.find(',');
                if (first_comma != std::string::npos) {
                    std::string err_str = last_response.substr(0, first_comma);
                    err_id = stringToInt(err_str);
                    
                    size_t brace_start = last_response.find('{', first_comma);
                    size_t brace_end = last_response.find('}', brace_start);
                    
                    if (brace_start != std::string::npos && brace_end != std::string::npos) {
                        mode_id = last_response.substr(brace_start, brace_end - brace_start + 1);
                    }
                }
            }
            
            if (err_id != 0) {
                std::cout << "[TCP Warning] Command not found in response, parsed last segment" << std::endl;
            }
        }
    }
    catch (const std::logic_error &err)
    {
        std::cout << "[TCP Exception] tcpDoCmd_f failed: " << err.what() << std::endl;
    }
}

bool CRCommanderRos2::callRosService(const std::string cmd, int32_t &err_id)
{
    try
    {
        std::vector<std::string> result_;
        doTcpCmd(this->dash_board_tcp_, cmd.c_str(), err_id, result_);
        return true;
    }
    catch (const TcpClientException &err)
    {
        std::cout << "%s" << std::endl;
        err_id = -1;
        return false;
    }
}

bool CRCommanderRos2::callRosService_async(const std::string &cmd)
{
    try
    {
        if (!dash_board_tcp_ || !dash_board_tcp_->isConnect())
        {
            return false;
        }
        
        // 异步发送：只发送命令，不等待响应
        dash_board_tcp_->tcpSend(cmd.c_str(), cmd.length());
        return true;
    }
    catch (const TcpClientException &err)
    {
        // 发送失败时才记录错误（限流，避免刷屏）
        static auto last_error_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_error_time).count() > 1000)
        {
            std::cout << "[callRosService_async] Send failed: " << err.what() << std::endl;
            last_error_time = now;
        }
        return false;
    }
}

bool CRCommanderRos2::callRosService_f(const std::string cmd, int32_t &err_id,std::string &mode_id)
{
    try
    {
        std::vector<std::string> result_;
        doTcpCmd_f(this->dash_board_tcp_, cmd.c_str(), err_id,mode_id, result_);
        return true;
    }
    catch (const TcpClientException &err)
    {
        std::cout << "%s" << std::endl;
        err_id = -1;
        return false;
    }
}

bool CRCommanderRos2::callRosService(const std::string cmd, int32_t &err_id, std::vector<std::string> &result_)
{
    try
    {
        doTcpCmd(this->dash_board_tcp_, cmd.c_str(), err_id, result_);
        return true;
    }
    catch (const TcpClientException &err)
    {
        std::cout << "%s" << std::endl;
        err_id = -1;
        return false;
    }
}

bool CRCommanderRos2::isEnable() const
{
    return real_time_data_->robot_mode == 5;
}

bool CRCommanderRos2::isConnected() const
{
    return dash_board_tcp_->isConnect() && real_time_tcp_->isConnect();
}

uint16_t CRCommanderRos2::getRobotMode() const
{
    return real_time_data_->robot_mode;
}

std::shared_ptr<RealTimeData> CRCommanderRos2::getRealData() const
{
    return real_time_data_;
}

bool CRCommanderRos2::servoJ(const double joint_positions[6], double servo_time,
                             double aheadtime, double gain)
{
    // 将弧度转换为角度
    double joint_deg[6];
    for (int i = 0; i < 6; i++) {
        joint_deg[i] = rad2Deg(joint_positions[i]);
    }
    
    // 构建ServoJ命令字符串
    // 格式: ServoJ(j1,j2,j3,j4,j5,j6,t=0.03,aheadtime=20,gain=500)
    char cmd[512];
    snprintf(cmd, sizeof(cmd), 
             "ServoJ(%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,t=%.3f,aheadtime=%.1f,gain=%.1f)",
             joint_deg[0], joint_deg[1], joint_deg[2],
             joint_deg[3], joint_deg[4], joint_deg[5],
             servo_time, aheadtime, gain);
    
    // 异步发送命令，不等待响应
    return callRosService_async(std::string(cmd));
}

bool CRCommanderRos2::setSpeedFactor(int ratio)
{
    // 构建SpeedFactor命令字符串
    // 格式: SpeedFactor(5)
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "SpeedFactor(%d)", ratio);
    
    // 通过TCP发送命令
    int32_t err_id = 0;
    bool success = callRosService(std::string(cmd), err_id);
    
    if (!success || err_id != 0) {
        std::cout << "[SpeedFactor] Failed to set speed to " << ratio << "%, Error code: " << err_id << std::endl;
        return false;
    }
    
    std::cout << "[SpeedFactor] Successfully set speed to " << ratio << "%" << std::endl;
    return true;
}

bool CRCommanderRos2::modbusClose(int index)
{
    // 构建ModbusClose命令字符串
    // 格式: ModbusClose(0)
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "ModbusClose(%d)", index);
    
    // 通过TCP发送命令
    int32_t err_id = 0;
    bool success = callRosService(std::string(cmd), err_id);
    
    if (!success || err_id != 0) {
        // 关闭失败不是严重错误，可能连接本来就不存在
        return false;
    }
    
    return true;
}

bool CRCommanderRos2::modbusRTUCreate(int slave_id, int baud, 
                                      const std::string& parity, int data_bit, int stop_bit)
{
    // 构建ModbusRTUCreate命令字符串
    // 格式: ModbusRTUCreate(1,115200,N,8,1)
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "ModbusRTUCreate(%d,%d,%s,%d,%d)", 
             slave_id, baud, parity.c_str(), data_bit, stop_bit);
    
    // 通过TCP发送命令
    int32_t err_id = 0;
    std::string mode_id;
    bool success = callRosService_f(std::string(cmd), err_id, mode_id);
    
    if (!success || err_id != 0) {
        std::cout << "[ModbusRTU] Failed to create Modbus RTU connection, Error code: " << err_id << std::endl;
        return false;
    }
    
    std::cout << "[ModbusRTU] Successfully created Modbus RTU connection (slave_id=" << slave_id << ")" << std::endl;
    return true;
}

bool CRCommanderRos2::setHoldRegs(int index, int addr, int count, 
                                  const std::string& val_tab, const std::string& val_type)
{
    // 构建SetHoldRegs命令字符串
    // 格式: SetHoldRegs(0,258,1,{0},U16)
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "SetHoldRegs(%d,%d,%d,%s,%s)", 
             index, addr, count, val_tab.c_str(), val_type.c_str());
    
    // 异步发送命令，不等待响应
    return callRosService_async(std::string(cmd));
}

bool CRCommanderRos2::getHoldRegs(int index, int addr, int count, 
                                  const std::string& val_type, std::string& result)
{
    // 构建GetHoldRegs命令字符串
    // 格式: GetHoldRegs(0,1549,2,U16)
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "GetHoldRegs(%d,%d,%d,%s)", 
             index, addr, count, val_type.c_str());
    
    // 通过TCP发送命令
    int32_t err_id = 0;
    std::string mode_id;
    bool success = callRosService_f(std::string(cmd), err_id, mode_id);
    
    if (!success || err_id != 0) {
        return false;
    }
    
    result = mode_id;  // mode_id 包含返回的数据
    return true;
}

