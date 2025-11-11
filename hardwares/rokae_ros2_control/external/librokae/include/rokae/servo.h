/**
 * @file servo.h
 * @brief 伺服相关接口
 * @copyright Copyright (C) 2023 ROKAE (Beijing) Technology Co., LTD. All Rights Reserved.
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

#ifndef ROKAEAPI_SERVO_H_
#define ROKAEAPI_SERVO_H_

#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif // defined(_MSC_VER) && (_MSC_VER >= 1200)

#include "base.h"
#include "data_types.h"

namespace rokae {

// forward declarations
class BaseRobot;
class XService;
struct Info;


/**
 * @struct Info
 * @brief 机器人从站信息
 */
struct SlaveInfo {
    uint32_t vendorId;
    uint32_t productCode;
    uint32_t reversionNumber;
    uint16_t slaveAddr;
    uint32_t slaveId;
    uint16_t alStatus;
	std::string slaveName;
};

/**
 * @struct SDOData
 * @brief SDO数据信息
 */
struct SDOData {
    int index;
    int sub_index;
    int length;
    int over_time;
    int wait_time = 0; //ms
    int print_data = 0; //值为1则控制器打印数据
    uint8_t data[50];
};

/**
 * @class BaseEthercat
 * @brief ethercat通用类
 */
class XCORE_API BaseEthercat : public Base<BaseEthercat>{
public:
    explicit BaseEthercat(std::shared_ptr<XService> rpc);
    virtual ~BaseEthercat();

    // **********************************************************************
    // ******************     ethercat从站相关接口          *********************

    /**
     * @brief 读SDO
     * @param[in] 从站地址、索引、子索引、长度、超时时间
     * @param[out] data 数据; ec 错误码
     * @return 成功/失败
     */
    bool ReadSDO(int slave_addr, int index, int sub_index, int length, uint8_t data[], int over_time, error_code& ec);

    /**
     * @brief 写SDO
     * @param[in] 从站地址、索引、子索引、长度、数据、超时时间
     * @param[out] ec 错误码
     * @return 成功/失败
     */
    bool WriteSDO(int slave_addr, int index, int sub_index, int length, uint8_t data[], int over_time, error_code& ec);

    /**
     * @brief 写多个SDO
     * @param[in] SDOData数组
     * @param[out] ec 错误码
     * @return 成功/失败
     */
    bool WriteMultiSDO(uint16_t slave_addr, std::vector<SDOData> SDO_data, error_code& ec);

    /**
     * @brief 获取从站数量
     * @param[out] ec 错误码
     * @return 从站数量
     */
    int GetSlaveCount(error_code& ec);

    /**
     * @brief 获取从站信息
     * @param[in] slave_addr 从站地址
     * @param[out] ec 错误码
     * @return 从站信息
     */
    SlaveInfo GetSlaveInfo(int slave_addr, error_code& ec);

	/**
	 * @brief 获取所有从站信息
	 * @param[out] ec 错误码
	 * @return 从站信息
	 */
	std::vector<SlaveInfo> GetSlavesInfo(error_code& ec);

    /**
     * @brief 从站全部切状态
     * @param[in] state 从站状态
     * @param[out] ec 错误码
     * @return 成功/失败
     */
    bool SetSlavesState(uint16_t state, error_code& ec);

    /**
     * @brief 获取某个从站状态
     * @param[in] slave_addr 从站地址
     * @param[out] ec 错误码
     * @return 从站状态
     */
    uint16_t GetSlaveState(int slave_addr, error_code& ec);

    /**
     * @brief 读PDO
     * @param[in] 从站地址、pdo偏移、pdo长度
     * @param[out] data 数据; ec 错误码
     * @return 成功/失败
     */
    bool ReadPDO(int slave_addr, int offset, int size, uint8_t data[], error_code& ec);

    /**
     * @brief 写PDO
     * @param[in] 从站地址、pdo偏移、pdo长度、数据
     * @param[out] ec 错误码
     * @return 成功/失败
     */
    bool WritePDO(int slave_addr, int offset, int size, uint8_t data[], error_code& ec);

    XCORESDK_DECLARE_IMPL
};
}  // namespace rokae

#endif // ROKAEAPI_SERVO_H