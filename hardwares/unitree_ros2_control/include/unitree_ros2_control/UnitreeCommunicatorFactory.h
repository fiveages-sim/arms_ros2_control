#pragma once

#include "UnitreeCommunicator.h"
#include <memory>
#include <string>

/**
 * @brief Unitree通信器工厂类
 * 
 * 根据机器人类型创建相应的通信器实例
 */
class UnitreeCommunicatorFactory {
public:
    /**
     * @brief 创建通信器实例
     * @param robot_type 机器人类型（"go2" 或 "g1"）
     * @return 通信器智能指针，如果类型不支持则返回nullptr
     */
    static std::unique_ptr<UnitreeCommunicator> createCommunicator(const std::string& robot_type);

    /**
     * @brief 获取支持的机器人类型列表
     * @return 支持的机器人类型字符串列表
     */
    static std::vector<std::string> getSupportedRobotTypes();

    /**
     * @brief 检查是否支持指定的机器人类型
     * @param robot_type 机器人类型
     * @return 是否支持
     */
    static bool isRobotTypeSupported(const std::string& robot_type);
};
