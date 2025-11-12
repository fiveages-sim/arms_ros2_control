/**
 * @file eu_harmonic.h
 * @author ygy (9330432@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-03-19
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef HARMONIC_EU_HARMONIC_H
#define HARMONIC_EU_HARMONIC_H

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef EXTERNFUNC
#ifdef _WIN32
#define EXTERNFUNC __declspec(dllexport)
#else
#define EXTERNFUNC
#endif
#endif

#ifdef _WIN32
    typedef char hint8;
    typedef short hint16;
    typedef long hint24;
    typedef long hint32;
    typedef long long hint40;
    typedef long long hint48;
    typedef long long hint56;
    typedef long long hint64;
    typedef unsigned char huint8;
    typedef unsigned short huint16;
    typedef unsigned long huint24;
    typedef unsigned long huint32;
    typedef unsigned long long huint40;
    typedef unsigned long long huint48;
    typedef unsigned long long huint56;
    typedef unsigned long long huint64;
    typedef float hreal32;
    typedef double hreal64;
#else
typedef signed char hint8;
typedef signed short int hint16;
typedef signed int hint24;
typedef signed int hint32;
typedef signed long int hint40;
typedef signed long int hint48;
typedef signed long int hint56;
typedef signed long int hint64;
typedef unsigned int huint8 __attribute__((__mode__(__QI__)));
typedef unsigned int huint16 __attribute__((__mode__(__HI__)));
typedef unsigned int huint24 __attribute__((__mode__(__SI__)));
typedef unsigned int huint32 __attribute__((__mode__(__SI__)));
typedef unsigned int huint40 __attribute__((__mode__(__DI__)));
typedef unsigned int huint48 __attribute__((__mode__(__DI__)));
typedef unsigned int huint56 __attribute__((__mode__(__DI__)));
typedef unsigned int huint64 __attribute__((__mode__(__DI__)));
typedef float hreal32;
typedef double hreal64;
#endif

#define HARMONIC_SUCCESS (0)               /**< 执行成功 */
#define HARMONIC_FAILED_DEVICEDISABLED (1) /**< 执行失败，设备不存在 */
#define HARMONIC_FAILED_OPENFAILED (2)     /**< 执行失败，设备打开失败 */
#define HARMONIC_FAILED_CANSEND (3)        /**< 执行失败，数据发送失败 */
#define HARMONIC_FAILED_CANRECEIVE (4)     /**< 执行失败，数据接收失败 */
#define HARMONIC_FAILED_ReadLocalDict (5)  /**< 执行失败，读本地字典失败 */
#define HARMONIC_FAILED_WriteLocalDict (6) /**< 执行失败，写本地字典失败 */
#define HARMONIC_FAILED_NoRespondR (7)     /**< 执行失败，sdo'读'请求在规定时间内没有接收到回应数据 */
#define HARMONIC_FAILED_NoRespondW (8)     /**< 执行失败，sdo'写'请求在规定时间内没有接收到回应数据 */
#define HARMONIC_FAILED_UNKNOWN (100)      /**< 执行失败，未知原因 */

    /**
     * @brief 设备类型
     *
     */
    enum harmonic_DeviceType
    {
        harmonic_DeviceType_USB2CAN = 4,            /**< 创芯usb转can设备 */
        harmonic_DeviceType_Canable = 11,           /**< 意优canable设备 */
        harmonic_DeviceType_ZCAN_USBCANFD_100U = 42 /**< 周立功usb转can设备 */
    };

    /**
     * @brief 波特率
     *
     */
    enum harmonic_Baudrate
    {
        harmonic_Baudrate_10 = 10,    /**< 波特率10 */
        harmonic_Baudrate_20 = 20,    /**< 波特率20 */
        harmonic_Baudrate_50 = 50,    /**< 波特率50 */
        harmonic_Baudrate_100 = 100,  /**< 波特率100 */
        harmonic_Baudrate_250 = 250,  /**< 波特率250 */
        harmonic_Baudrate_500 = 500,  /**< 500波特率 */
        harmonic_Baudrate_800 = 800,  /**< 800波特率 */
        harmonic_Baudrate_1000 = 1000 /**< 1000波特率 */
    };

    enum harmonic_DBaudrate
    {
        harmonic_DBaudrate_1000 = 1000, /**< 波特率1000 */
        harmonic_DBaudrate_2000 = 2000, /**< 波特率2000 */
        harmonic_DBaudrate_3000 = 3000, /**< 波特率3000 */
        harmonic_DBaudrate_4000 = 4000, /**< 波特率4000 */
        harmonic_DBaudrate_5000 = 5000, /**< 波特率5000 */
        harmonic_DBaudrate_6000 = 6000, /**< 波特率6000 */
        harmonic_DBaudrate_7000 = 7000, /**< 波特率7000 */
        harmonic_DBaudrate_8000 = 8000  /**< 波特率8000 */
    };
    /**
     * @brief NMT状态
     *
     */
    enum harmonic_NMTState
    {
        harmonic_NMTState_Start_Node = 0x01,
        harmonic_NMTState_Stop_Node = 0x02,
        harmonic_NMTState_Enter_PreOperational = 0x80,
        harmonic_NMTState_Reset_Node = 0x81,
        harmonic_NMTState_Reset_Comunication = 0x82
    };

    /**
     * @brief 节点状态
     *
     */
    enum harmonic_NodeState
    {
        harmonic_NodeState_Initialisation = 0x00,  /**< 初始化状态 */
        harmonic_NodeState_Disconnected = 0x01,    /**< 掉线状态 */
        harmonic_NodeState_Connecting = 0x02,      /**< 在线状态 */
        harmonic_NodeState_Preparing = 0x02,       /**< 准备状态 */
        harmonic_NodeState_Stopped = 0x04,         /**< 停止状态 */
        harmonic_NodeState_Operational = 0x05,     /**< 操作状态 */
        harmonic_NodeState_Pre_operational = 0x7F, /**< 预操作状态 */
        harmonic_NodeState_Unknown_state = 0x0F    /**< 未知状态 */
    };

    /**
     * @brief 数据类型
     *
     */
    enum harmonic_DataType
    {
        //        harmonic_DataType_boolean = 0x01, /**< 布尔类型 */
        harmonic_DataType_int8 = 0x02,   /**< 1字节有符号整形 */
        harmonic_DataType_int16 = 0x03,  /**< 2字节有符号整形 */
        harmonic_DataType_int32 = 0x04,  /**< 4字节有符号整形 */
        harmonic_DataType_uint8 = 0x05,  /**< 1字节无符号整形 */
        harmonic_DataType_uint16 = 0x06, /**< 2字节无符号整形 */
        harmonic_DataType_uint32 = 0x07, /**< 4字节无符号整形 */
        harmonic_DataType_real32 = 0x08  /**< 4字节浮点型 */
    };

    /**
     * @brief 快速停止选项
     *
     */
    enum harmonic_QuickStopOption
    {
        harmonic_QuickStopOption_Disable_Drive = 0,                                         /**< 禁用驱动功能 */
        harmonic_QuickStopOption_Slow_Down_On_Slow_Down_Ramp = 1,                           /**< 在减速斜坡道上减速 */
        harmonic_QuickStopOption_Slow_Down_On_Quick_Stop_Ramp = 2,                          /**< 在急停坡道上减速 */
        harmonic_QuickStopOption_Slow_Down_On_the_Current_Limit = 3,                        /**< 在电流限制上减速 */
        harmonic_QuickStopOption_Slow_Down_On_the_Voltag_Limit = 4,                         /**< 在电压限制上减速 */
        harmonic_QuickStopOption_Slow_Down_On_Quick_Stop_Ramp_And_Stay_In_Quick_Stop = 5,   /**< 在急停坡道上减速并停留在急停位置 */
        harmonic_QuickStopOption_Slow_Down_On_Slow_Down_Ramp_And_Stay_In_Quick_Stop = 6,    /**< 在减速斜坡道上减速并停留在急停位置 */
        harmonic_QuickStopOption_Slow_Down_On_the_Current_Limit_And_Stay_In_Quick_Stop = 7, /**< 在电流限制下减速并停留在急停位置 */
        harmonic_QuickStopOption_Slow_Down_On_the_Voltag_Limit_And_Stay_In_Quick_Stop = 8   /**< 在电压限制下减速并停留在急停位置 */
    };

    /**
     * @brief 关机选项
     *
     */
    enum harmonic_ShutdownOption
    {
        harmonic_ShutdownOption_Disable_Drive_Function = 0,   /**< 禁用驱动功能 */
        harmonic_ShutdownOption_Slown_With_Slow_Down_Ramp = 1 /**< 使用减速斜坡减速，然后禁用驱动功能 */
    };

    /**
     * @brief 下使能操作
     *
     */
    enum harmonic_DisableOperationOption
    {
        harmonic_DisableOperationOption_Disable_Drive_Function = 0,       /**< 禁用驱动功能 */
        harmonic_DisableOperationOption_Slow_Down_With_Slow_Down_Ramp = 1 /**< 使用减速斜坡减速，然后禁用驱动功能 */
    };

    /**
     * @brief 暂停选项
     *
     */
    enum harmonic_HaltOption
    {
        harmonic_HaltOption_Disable_Drive = 0,                  /**< 禁用驱动器，电机可自动旋转 */
        harmonic_HaltOption_Slow_Down_On_Slow_Down_Ramp = 1,    /**< 在减速斜坡上减速 */
        harmonic_HaltOption_Slow_Down_On_Quick_Stop_Ramp = 2,   /**< 在急停坡道上减速 */
        harmonic_HaltOption_Slow_Down_On_the_Current_Limit = 3, /**< 在电流限制下减速 */
        harmonic_HaltOption_Slow_Down_On_the_Voltag_Limit = 4   /**< 在电压限制下减速 */
    };

    /**
     * @brief 故障处理选项
     *
     */
    enum harmonic_FaultReactionOption
    {
        harmonic_FaultReactionOption_Disable_drive_function = 0,       /**< 禁用驱动器，电机可自由旋转 */
        harmonic_FaultReactionOption_Slow_down_on_slow_down_ramp = 1,  /**< 在减速斜坡上减速 */
        harmonic_FaultReactionOption_Slow_down_on_quick_stop_ramp = 2, /**< 在急停坡道上减速 */
        harmonic_FaultReactionOption_Slow_down_on_current_limit = 3,   /**< 在电流限制下减速 */
        harmonic_FaultReactionOption_Slow_down_on_voltage_limit = 4    /**< 在电压限制下减速 */
    };

    /**
     * @brief 操作模式
     *
     */
    enum harmonic_OperateMode
    {
        harmonic_OperateMode_AutoTuning = -4,
        harmonic_OperateMode_INLCalibration = -3,
        harmonic_OperateMode_RotorAligning = -2,
        harmonic_OperateMode_Reserve = 0,              /**< 保留的 */
        harmonic_OperateMode_ProfilePosition = 1,      /**< 轮廓位置模式 */
        harmonic_OperateMode_Velocity = 2,             /**< 速度模式 */
        harmonic_OperateMode_ProfileVelocity = 3,      /**< 轮廓速度模式 */
        harmonic_OperateMode_ProfileTorque = 4,        /**< 轮廓力矩模式 */
        harmonic_OperateMode_Homing = 6,               /**< 归航模式 */
        harmonic_OperateMode_InterpolatedPosition = 7, /**< 内插位置模式 */
        harmonic_OperateMode_CyclicSyncPosition = 8,   /**< 同步位置模式 */
        harmonic_OperateMode_CyclicSyncVelocity = 9,   /**< 同步速度模式 */
        harmonic_OperateMode_CyclicSyncTorque = 10,    /**< 同步力矩模式 */
        harmonic_OperateMode_TorquePositionFixed = 11  /**< 力矩位置混合模式 */
    };

    /**
     * @brief can数据结构
     *
     */
    typedef struct _harmonic_CanMsg
    {
        huint32 canId;   /**< can id */
        huint8 rtr;      /**< 0是数据帧，1是远程帧 */
        huint8 extended; /**< 0是标准帧，1是扩展帧 */
        huint8 len;      /**< 数据长度，0~8 */
        huint8 data[8];  /**< 数据 */
    } harmonic_CanMsg;

    /**
     * @brief 发送数据回调函数
     *
     */
    typedef void (*harmonic_SendDataCallBack)(huint8 devIndex, const harmonic_CanMsg *msg);

    /**
     * @brief 接收数据回调函数
     *
     */
    typedef void (*harmonic_ReceiveDataCallBack)(huint8 devIndex, const harmonic_CanMsg *msg);

    /**
     * @brief 设置发送数据回调函数
     *
     * @param callFunc 回调函数地址
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setSendDataCallBack(harmonic_SendDataCallBack callFunc);

    /**
     * @brief 设置接收数据回调函数
     *
     * @param callFunc 回调函数地址
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setReceiveDataCallBack(harmonic_ReceiveDataCallBack callFunc);

    /**
     * @brief 初始化dll，在调用所有函数前，必须先调用该函数进行初始化，该函数会打开设备。
         【注意】控制多个设备时可多次调用，多设备控制时共享一个主站，所以即使不同的通讯设备上，电机id也不能重复
     *
     * @param devType 设备类型，参见harmonic_DeviceType
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param baudrate
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_initDLL(harmonic_DeviceType devType, huint8 devIndex, harmonic_Baudrate baudrate, bool isCanFD = false, harmonic_DBaudrate dbaudrate = harmonic_DBaudrate::harmonic_DBaudrate_5000);

    /**
     * @brief 关闭设备，释放资源，设备打开成功后需要在适当时机释放资源
        如果存在多个设备，程序退出前，每个设备都需调用一次释放资源
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @return EXTERNFUNC
     */
    EXTERNFUNC int harmonic_freeDLL(huint8 devIndex);

    /**
     * @brief 读从节点状态
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param state 保存读取的从节点的节点状态
     * @param timeout 最大等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getNodeState(huint8 devIndex, huint8 id, harmonic_NodeState *state, huint32 timeout = 100);

    /**
     * @brief 设置本地节点状态
     *
     * @param state 节点状态
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalNodeState(harmonic_NodeState state);

    /**
     * @brief 设置从节点状态
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param state 节点状态
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setNodeState(huint8 devIndex, huint8 id, harmonic_NMTState state);

    /**
     * @brief 读主站字典
     *
     * @param index 主索引
     * @param subIndex 子索引
     * @param dataType 数据类型
     * @param readData 保存读取的数据
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_readLocalDirectory(huint16 index, huint8 subIndex, harmonic_DataType dataType, void *readData);

    /**
     * @brief 读从站字典
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param index 主索引
     * @param subIndex 子索引
     * @param dataType 数据类型
     * @param readData 保存读取的数据
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_readDirectory(huint8 devIndex, huint8 id, huint16 index, huint8 subIndex, harmonic_DataType dataType, void *readData, huint32 timeout = 100);

    /**
     * @brief 写主站字典
     *
     * @param index 主索引
     * @param subIndex 子索引
     * @param dataType 数据类型
     * @param writeData 写入的数据
     * @param sendLen 写入字节长度
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_writeLocalDirectory(huint16 index, huint8 subIndex, harmonic_DataType dataType, void *writeData);

    /**
     * @brief 写从站字典
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param index 主索引
     * @param subIndex 子索引
     * @param dataType 数据类型
     * @param writeData 写入的数据
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_writeDirectory(huint8 devIndex, huint8 id, huint16 index, huint8 subIndex, harmonic_DataType dataType, void *writeData, huint32 timeout = 100);

        /**
     * @brief 发送原始can数据
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param cobId can帧id
     * @param data 发送的can数据
     * @param len 数据长度
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_writeCanData(huint8 devIndex, huint16 cobId, const huint8 *const data, huint8 len);

    /**
     * @brief 发送原始can数据
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param canMsg can帧
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_writeCanMsg(huint8 devIndex, const harmonic_CanMsg *canMsg);

    /**
     * @brief 轮廓位置控制，该模式下电机内部根据给定参数生成轨迹并执行
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param targetPos 目标位置，单位脉冲
     * @param vel 速度，单位脉冲
     * @param acc 加速度，单位脉冲
     * @param dec 减速度，单位脉冲
     * @param isRelative 是否采用相对位置，true:采用相对位置，false:采用绝对位置
     * @param isImmediately 该函数的调用是否立即生效，true:立即生效，false:如果前一次指令没有运行到位，继续执行前一次指令，执行完毕后再执行当前指令
     * @param isUpdate 是否采用更新模式，true:采用更新模式，false:不采用更新模式 (更新模式下，pdo通信参数不会再次配置，一般相同模式下的第一次控制设为false，后面的多次控制设为true)。
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_profilePositionControl(huint8 devIndex, huint8 id, hint32 targetPos, huint32 vel, huint32 acc, huint32 dec, bool isRelative = false, bool isImmediately = true, bool isUpdate = false);

    /**
     * @brief 轮廓速度控制，该模式下电机内部根据给定参数生成轨迹并执行
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param targetVel 目标速度，单位脉冲
     * @param acc 加速度，单位脉冲
     * @param dec 减速度，单位脉冲
     * @param isUpdate 是否采用更新模式，true:采用更新模式，false:不采用更新模式 (更新模式下，pdo通信参数不会再次配置，一般相同模式下的第一次控制设为false，后面的多次控制设为true)。
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_profileVelocityControl(huint8 devIndex, huint8 id, hint32 targetVel, huint32 acc, huint32 dec, bool isUpdate = false);

    /**
     * @brief 轮廓力矩控制，该模式下电机内部根据给定参数生成轨迹并执行
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param targetTorque 目标力矩，单位千分之
     * @param slope 增长斜率，单位千分之/s
     * @param isUpdate 是否采用更新模式，true:采用更新模式，false:不采用更新模式 (更新模式下，pdo通信参数不会再次配置，一般相同模式下的第一次控制设为false，后面的多次控制设为true)。
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_profileTorqueControl(huint8 devIndex, huint8 id, huint16 targetTorque, hint16 slope, bool isUpdate = false);

    /**
     * @brief 停止控制
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_stopControl(huint8 devIndex, huint8 id);

    /**
     * @brief 读本地字典0x1005-0x0
     *
     * @param cobId 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalSyncCOBID(huint32 *cobId);

    /**
     * @brief 读本地字典0x1005-0x0
     *
     * @param cobId 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalSyncCOBID(huint32 cobId);

    /**
     * @brief 读本地字典0x1006-0x0
     *
     * @param cycle 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalSyncCycle(huint32 *cycle);

    /**
     * @brief 写本地字典0x1006-0x0
     *
     * @param cycle
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalSyncCycle(huint32 cycle);

    /**
     * @brief 读本地字典0x1007-0x0
     *
     * @param sWindow 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalSyncWindow(huint32 *sWindow);

    /**
     * @brief 写本地字典0x1007-0x0
     *
     * @param sWindow 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalSyncWindow(huint32 sWindow);

    /**
     * @brief 读本地字典[0x1200+sdoServerIndex]-0x0
     *
     * @param sdoServerIndex sdo索引，为0时主索引为0x1200，为1时主索引为0x1201，以此类推
     * @param num 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalServerSDONumberOfEntries(huint16 sdoServerIndex, huint8 *num);

    /**
     * @brief 写本地字典[0x1200+sdoServerIndex]-0x0
     *
     * @param sdoServerIndex sdo索引，为0时主索引为0x1200，为1时主索引为0x1201，以此类推
     * @param num 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalServerSDONumberOfEntries(huint16 sdoServerIndex, huint8 num);

    /**
     * @brief 读本地字典[0x1200+sdoServerIndex]-0x1
     *
     * @param sdoServerIndex sdo索引，为0时主索引为0x1200，为1时主索引为0x1201，以此类推
     * @param cobId 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalServerSDOCobIdClientToServer(huint16 sdoServerIndex, huint32 *cobId);

    /**
     * @brief 写本地字典[0x1200+sdoServerIndex]-0x1
     *
     * @param sdoServerIndex sdo索引，为0时主索引为0x1200，为1时主索引为0x1201，以此类推
     * @param cobId 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalServerSDOCobIdClientToServer(huint16 sdoServerIndex, huint32 cobId);

    /**
     * @brief 读本地字典[0x1200+sdoServerIndex]-0x2
     *
     * @param sdoServerIndex sdo索引，为0时主索引为0x1200，为1时主索引为0x1201，以此类推
     * @param cobId 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalServerSDOCobIdServerToClient(huint16 sdoServerIndex, huint32 *cobId);

    /**
     * @brief 写本地字典[0x1200+sdoServerIndex]-0x2
     *
     * @param sdoServerIndex sdo索引，为0时主索引为0x1200，为1时主索引为0x1201，以此类推
     * @param cobId 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalServerSDOCobIdServerToClient(huint16 sdoServerIndex, huint32 cobId);

    /**
     * @brief 读本地字典[0x1200+sdoServerIndex]-0x3
     *
     * @param sdoServerIndex sdo索引，为0时主索引为0x1200，为1时主索引为0x1201，以此类推
     * @param nodeId 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalServerSDONodeIdOfClient(huint16 sdoServerIndex, huint8 *nodeId);

    /**
     * @brief 写本地字典[0x1200+sdoServerIndex]-0x3
     *
     * @param sdoServerIndex sdo索引，为0时主索引为0x1200，为1时主索引为0x1201，以此类推
     * @param nodeId 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalServerSDONodeIdOfClient(huint16 sdoServerIndex, huint8 nodeId);

    /**
     * @brief 读本地字典[0x1280+sdoClientIndex]-0x0
     *
     * @param sdoClientIndex sdo索引，为0时主索引为0x1280，为1时主索引为0x1281，以此类推
     * @param number 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalClientSDONumberOfEntries(huint16 sdoClientIndex, huint8 *number);

    /**
     * @brief 写本地字典[0x1280+sdoClientIndex]-0x0
     *
     * @param sdoClientIndex sdo索引，为0时主索引为0x1280，为1时主索引为0x1281，以此类推
     * @param number 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalClientSDONumberOfEntries(huint16 sdoClientIndex, huint8 number);

    /**
     * @brief 读本地字典[0x1280+sdoClientIndex]-0x1
     *
     * @param sdoClientIndex sdo索引，为0时主索引为0x1280，为1时主索引为0x1281，以此类推
     * @param cobId 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalClientSDOCobIdClientToServer(huint16 sdoClientIndex, huint32 *cobId);

    /**
     * @brief 写本地字典[0x1280+sdoClientIndex]-0x1
     *
     * @param sdoClientIndex sdo索引，为0时主索引为0x1280，为1时主索引为0x1281，以此类推
     * @param cobId 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalClientSDOCobIdClientToServer(huint16 sdoClientIndex, huint32 cobId);

    /**
     * @brief 读本地字典[0x1280+sdoClientIndex]-0x2
     *
     * @param sdoClientIndex sdo索引，为0时主索引为0x1280，为1时主索引为0x1281，以此类推
     * @param cobId 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalClientSDOCobIdServerToClient(huint16 sdoClientIndex, huint32 *cobId);

    /**
     * @brief 写本地字典[0x1280+sdoClientIndex]-0x2
     *
     * @param sdoClientIndex sdo索引，为0时主索引为0x1280，为1时主索引为0x1281，以此类推
     * @param cobId 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalClientSDOCobIdServerToClient(huint16 sdoClientIndex, huint32 cobId);

    /**
     * @brief 读本地字典[0x1280+sdoClientIndex]-0x3
     *
     * @param sdoClientIndex sdo索引，为0时主索引为0x1280，为1时主索引为0x1281，以此类推
     * @param nodeId 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalClientSDONodeIdOfSDOServer(huint16 sdoClientIndex, huint8 *nodeId);

    /**
     * @brief 写本地字典[0x1280+sdoClientIndex]-0x3
     *
     * @param sdoClientIndex sdo索引，为0时主索引为0x1280，为1时主索引为0x1281，以此类推
     * @param nodeId 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalClientSDONodeIdOfSDOServer(huint16 sdoClientIndex, huint8 nodeId);

    /**
     * @brief 读本地字典[0x1400+pdoIndex]-0x0
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1400，为1时主索引为0x1401，以此类推
     * @param count 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalRPDOMaxParasCount(huint16 pdoIndex, huint8 *count);

    /**
     * @brief 写本地字典[0x1400+pdoIndex]-0x0
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1400，为1时主索引为0x1401，以此类推
     * @param count 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalRPDOMaxParasCount(huint16 pdoIndex, huint8 count);

    /**
     * @brief 读本地字典[0x1400+pdoIndex]-0x1
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1400，为1时主索引为0x1401，以此类推
     * @param cobId 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalRPDOCobId(huint16 pdoIndex, huint32 *cobId);

    /**
     * @brief 写本地字典[0x1400+pdoIndex]-0x1
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1400，为1时主索引为0x1401，以此类推
     * @param cobId 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalRPDOCobId(huint16 pdoIndex, huint32 cobId);

    /**
     * @brief 读本地字典[0x1400+pdoIndex]-0x2
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1400，为1时主索引为0x1401，以此类推
     * @param type 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalRPDOTransmitType(huint16 pdoIndex, huint8 *type);

    /**
     * @brief 写本地字典[0x1400+pdoIndex]-0x2
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1400，为1时主索引为0x1401，以此类推
     * @param type 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalRPDOTransmitType(huint16 pdoIndex, huint8 type);

    /**
     * @brief 读本地字典[0x1400+pdoIndex]-0x3
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1400，为1时主索引为0x1401，以此类推
     * @param time 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalRPDOInhibitTime(huint16 pdoIndex, huint16 *time);

    /**
     * @brief 写本地字典[0x1400+pdoIndex]-0x3
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1400，为1时主索引为0x1401，以此类推
     * @param time 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalRPDOInhibitTime(huint16 pdoIndex, huint16 time);

    /**
     * @brief 读本地字典[0x1400+pdoIndex]-0x5
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1400，为1时主索引为0x1401，以此类推
     * @param timer 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalRPDOEventTimer(huint16 pdoIndex, huint16 *timer);

    /**
     * @brief 写本地字典[0x1400+pdoIndex]-0x5
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1400，为1时主索引为0x1401，以此类推
     * @param timer 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalRPDOEventTimer(huint16 pdoIndex, huint16 timer);

    /**
     * @brief 读本地字典[0x1400+pdoIndex]-0x6
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1400，为1时主索引为0x1401，以此类推
     * @param value 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalRPDOSYNCStartValue(huint16 pdoIndex, huint8 *value);

    /**
     * @brief 写本地字典[0x1400+pdoIndex]-0x6
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1400，为1时主索引为0x1401，以此类推
     * @param value 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalRPDOSYNCStartValue(huint16 pdoIndex, huint8 value);

    /**
     * @brief 读本地字典[0x1600+pdoIndex]-0x0
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1600，为1时主索引为0x1601，以此类推
     * @param count 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalRPDOMaxMappedCount(huint16 pdoIndex, huint8 *count);

    /**
     * @brief 写本地字典[0x1600+pdoIndex]-0x0
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1600，为1时主索引为0x1601，以此类推
     * @param count 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalRPDOMaxMappedCount(huint16 pdoIndex, huint8 count);

    /**
     * @brief 读本地字典[0x1600+pdoIndex]-[0x1+mapIndex]
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1600，为1时主索引为0x1601，以此类推
     * @param mapIndex pdo映射索引，为0时子索引为0x1，1时子索引为0x2，以此类推
     * @param value 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalRPDOMapped(huint16 pdoIndex, huint8 mapIndex, huint32 *value);

    /**
     * @brief 写本地字典[0x1600+pdoIndex]-[0x1+mapIndex]
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1600，为1时主索引为0x1601，以此类推
     * @param mapIndex pdo映射索引，为0时子索引为0x1，1时子索引为0x2，以此类推
     * @param value 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalRPDOMapped(huint16 pdoIndex, huint8 mapIndex, huint32 value);

    /**
     * @brief 读本地字典[0x1800+pdoIndex]-0x0
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1800，为1时主索引为0x1801，以此类推
     * @param count 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalTPDOMaxParasCount(huint16 pdoIndex, huint8 *count);

    /**
     * @brief 写本地字典[0x1800+pdoIndex]-0x0
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1800，为1时主索引为0x1801，以此类推
     * @param count 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalTPDOMaxParasCount(huint16 pdoIndex, huint8 count);

    /**
     * @brief 读本地字典[0x1800+pdoIndex]-0x1
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1800，为1时主索引为0x1801，以此类推
     * @param cobId 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalTPDOCobId(huint16 pdoIndex, huint32 *cobId);

    /**
     * @brief 写本地字典[0x1800+pdoIndex]-0x1
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1800，为1时主索引为0x1801，以此类推
     * @param cobId 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalTPDOCobId(huint16 pdoIndex, huint32 cobId);

    /**
     * @brief 读本地字典[0x1800+pdoIndex]-0x2
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1800，为1时主索引为0x1801，以此类推
     * @param type 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalTPDOTransmitType(huint16 pdoIndex, huint8 *type);

    /**
     * @brief 写本地字典[0x1800+pdoIndex]-0x2
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1800，为1时主索引为0x1801，以此类推
     * @param type 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalTPDOTransmitType(huint16 pdoIndex, huint8 type);

    /**
     * @brief 读本地字典[0x1800+pdoIndex]-0x3
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1800，为1时主索引为0x1801，以此类推
     * @param time 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalTPDOInhibitTime(huint16 pdoIndex, huint16 *time);

    /**
     * @brief 写本地字典[0x1800+pdoIndex]-0x3
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1800，为1时主索引为0x1801，以此类推
     * @param time 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalTPDOInhibitTime(huint16 pdoIndex, huint16 time);

    /**
     * @brief 读本地字典[0x1800+pdoIndex]-0x5
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1800，为1时主索引为0x1801，以此类推
     * @param timer 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalTPDOEventTimer(huint16 pdoIndex, huint16 *timer);

    /**
     * @brief 写本地字典[0x1800+pdoIndex]-0x5
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1800，为1时主索引为0x1801，以此类推
     * @param timer 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalTPDOEventTimer(huint16 pdoIndex, huint16 timer);

    /**
     * @brief 读本地字典[0x1800+pdoIndex]-0x6
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1800，为1时主索引为0x1801，以此类推
     * @param value 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalTPDOSYNCStartValue(huint16 pdoIndex, huint8 *value);

    /**
     * @brief 写本地字典[0x1800+pdoIndex]-0x6
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1800，为1时主索引为0x1801，以此类推
     * @param value 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalTPDOSYNCStartValue(huint16 pdoIndex, huint8 value);

    /**
     * @brief 读本地字典[0x1A00+pdoIndex]-0x0
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1A00，为1时主索引为0x1A01，以此类推
     * @param count 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalTPDOMaxMappedCount(huint16 pdoIndex, huint8 *count);

    /**
     * @brief 写本地字典[0x1A00+pdoIndex]-0x0
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1A00，为1时主索引为0x1A01，以此类推
     * @param count 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalTPDOMaxMappedCount(huint16 pdoIndex, huint8 count);

    /**
     * @brief 读本地字典[0x1A00+pdoIndex]-[0x1+mapIndex]
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1A00，为1时主索引为0x1A01，以此类推
     * @param mapIndex pdo映射索引，为0时子索引为0x1，为1时子索引为0x2
     * @param value 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalTPDOMapped(huint16 pdoIndex, huint8 mapIndex, huint32 *value);

    /**
     * @brief 写本地字典[0x1A00+pdoIndex]-[0x1+mapIndex]
     *
     * @param pdoIndex pdo索引，为0时主索引为0x1A00，为1时主索引为0x1A01，以此类推
     * @param mapIndex pdo映射索引，为0时子索引为0x1，为1时子索引为0x2
     * @param value 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalTPDOMapped(huint16 pdoIndex, huint8 mapIndex, huint32 value);

    /**
     * @brief 读本地字典0x2106-0x1
     *
     * @param tem 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalServoTemperature(hint8 *tem);

    /**
     * @brief 读本地字典0x2130-0x0
     *
     * @param cobcmd 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalMitCtrlCobcmd1(huint32 *cobcmd);

    /**
     * @brief 写本地字典0x2130-0x0
     *
     * @param cobcmd 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalMitCtrlCobcmd1(huint32 cobcmd);

    /**
     * @brief 读本地字典0x2131-0x0
     *
     * @param cobcmd 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalMitCtrlCobcmd2(huint32 *cobcmd);

    /**
     * @brief 写本地字典0x2131-0x0
     *
     * @param cobcmd 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalMitCtrlCobcmd2(huint32 cobcmd);

    /**
     * @brief 读本地字典0x2132-0x0
     *
     * @param cobdat 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalMitReplyCobdat1(huint32 *cobdat);

    /**
     * @brief 写本地字典0x2132-0x0
     *
     * @param cobdat 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalMitReplyCobdat1(huint32 cobdat);

    /**
     * @brief 读本地字典0x2133-0x0
     *
     * @param cobdat 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalMitReplyCobdat2(huint32 *cobdat);

    /**
     * @brief 写本地字典0x2133-0x0
     *
     * @param cobdat 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalMitReplyCobdat2(huint32 cobdat);

    /**
     * @brief 读本地字典0x5000+[0x1+motorIndex]
     *
     * @param motorIndex 电机索引，为0时子索引为0x1,为1时子索引为0x2，以此类推
     * @param pos 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalMutiMotorPos(huint16 motorIndex, hint32 *pos);

    /**
     * @brief 读本地字典0x5001+[0x1+motorIndex]
     *
     * @param motorIndex 电机索引，为0时子索引为0x1,为1时子索引为0x2，以此类推
     * @param vel 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalMutiMotorVel(huint16 motorIndex, hint32 *vel);

    /**
     * @brief 读本地字典0x5002+[0x1+motorIndex]
     *
     * @param motorIndex 电机索引，为0时子索引为0x1,为1时子索引为0x2，以此类推
     * @param tor 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalMutiMotorTorque(huint16 motorIndex, hint16 *tor);

    /**
     * @brief 写本地字典0x5003+[0x1+motorIndex]
     *
     * @param motorIndex 电机索引，为0时子索引为0x1,为1时子索引为0x2，以此类推
     * @param pos 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalMutiMotorSetPos(huint16 motorIndex, hint32 pos);

    /**
     * @brief 写本地字典0x5004+[0x1+motorIndex]
     *
     * @param motorIndex 电机索引，为0时子索引为0x1,为1时子索引为0x2，以此类推
     * @param vel 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalMutiMotorSetVel(huint16 motorIndex, hint32 vel);

    /**
     * @brief 写本地字典0x5005+[0x1+motorIndex]
     *
     * @param motorIndex 电机索引，为0时子索引为0x1,为1时子索引为0x2，以此类推
     * @param torque 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalMutiMotorSetTorque(huint16 motorIndex, hint16 torque);

    /**
     * @brief 读本地字典0x5006+[0x1+motorIndex]
     *
     * @param motorIndex 电机索引，为0时子索引为0x1,为1时子索引为0x2，以此类推
     * @param status 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalMutiMotorStatusWord(huint16 motorIndex, huint16 *status);

    /**
     * @brief 读本地字典0x5007+[0x1+motorIndex]
     *
     * @param motorIndex 电机索引，为0时子索引为0x1,为1时子索引为0x2，以此类推
     * @param error 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalMutiMotorErrorCode(huint16 motorIndex, huint16 *error);

    /**
     * @brief 读本地字典0x5008+[0x1+motorIndex]
     *
     * @param motorIndex 电机索引，为0时子索引为0x1,为1时子索引为0x2，以此类推
     * @param error 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalMitMutiMotorGetPos(huint16 motorIndex, hreal32 *pos);

    /**
     * @brief 写本地字典0x5008+[0x1+motorIndex]
     *
     * @param motorIndex 电机索引，为0时子索引为0x1,为1时子索引为0x2，以此类推
     * @param error 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalMitMutiMotorGetPos(huint16 motorIndex, hreal32 pos);

    /**
     * @brief 读本地字典0x5009+[0x1+motorIndex]
     *
     * @param motorIndex 电机索引，为0时子索引为0x1,为1时子索引为0x2，以此类推
     * @param error 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalMitMutiMotorGetVel(huint16 motorIndex, hreal32 *vel);

    /**
     * @brief 写本地字典0x5009+[0x1+motorIndex]
     *
     * @param motorIndex 电机索引，为0时子索引为0x1,为1时子索引为0x2，以此类推
     * @param error 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalMitMutiMotorGetVel(huint16 motorIndex, hreal32 vel);

    /**
     * @brief 读本地字典0x500A+[0x1+motorIndex]
     *
     * @param motorIndex 电机索引，为0时子索引为0x1,为1时子索引为0x2，以此类推
     * @param error 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalMitMutiMotorGetTor(huint16 motorIndex, hreal32 *tor);

    /**
     * @brief 写本地字典0x500A+[0x1+motorIndex]
     *
     * @param motorIndex 电机索引，为0时子索引为0x1,为1时子索引为0x2，以此类推
     * @param error 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalMitMutiMotorGetTor(huint16 motorIndex, hreal32 tor);

    /**
     * @brief 读本地字典0x500B+[0x1+motorIndex]
     *
     * @param motorIndex 电机索引，为0时子索引为0x1,为1时子索引为0x2，以此类推
     * @param error 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalMitMutiMotorSetPos(huint16 motorIndex, hreal32 *pos);

    /**
     * @brief 写本地字典0x500B+[0x1+motorIndex]
     *
     * @param motorIndex 电机索引，为0时子索引为0x1,为1时子索引为0x2，以此类推
     * @param error 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalMitMutiMotorSetPos(huint16 motorIndex, hreal32 pos);

    /**
     * @brief 读本地字典0x500C+[0x1+motorIndex]
     *
     * @param motorIndex 电机索引，为0时子索引为0x1,为1时子索引为0x2，以此类推
     * @param error 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalMitMutiMotorSetVel(huint16 motorIndex, hreal32 *vel);

    /**
     * @brief 写本地字典0x500C+[0x1+motorIndex]
     *
     * @param motorIndex 电机索引，为0时子索引为0x1,为1时子索引为0x2，以此类推
     * @param error 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalMitMutiMotorSetVel(huint16 motorIndex, hreal32 vel);

    /**
     * @brief 读本地字典0x500D+[0x1+motorIndex]
     *
     * @param motorIndex 电机索引，为0时子索引为0x1,为1时子索引为0x2，以此类推
     * @param error 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalMitMutiMotorSetTor(huint16 motorIndex, hreal32 *tor);

    /**
     * @brief 写本地字典0x500D+[0x1+motorIndex]
     *
     * @param motorIndex 电机索引，为0时子索引为0x1,为1时子索引为0x2，以此类推
     * @param error 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalMitMutiMotorSetTor(huint16 motorIndex, hreal32 tor);

    /**
     * @brief 读本地字典0x603F-0x0
     *
     * @param err 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalServoErrorCode(huint16 *err);

    /**
     * @brief 读本地字典0x6040-0x0
     *
     * @param word 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalControlword(huint16 *word);

    /**
     * @brief 写本地字典0x6040-0x0
     *
     * @param word 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalControlword(huint16 word);

    /**
     * @brief 写本地字典0x6041-0x0
     *
     * @param word 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalStatusWord(huint16 *word);

    /**
     * @brief 读本地字典0x6064-0x0
     *
     * @param pos 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalActualPos(hint32 *pos);

    /**
     * @brief 读本地字典0x606c-0x0
     *
     * @param velocity 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalActualVelocity(hint32 *velocity);

    /**
     * @brief 读本地字典0x6071-0x0
     *
     * @param torque 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalTargetTorque(hint16 *torque);

    /**
     * @brief 写本地字典0x6071-0x0
     *
     * @param torque 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalTargetTorque(hint16 torque);

    /**
     * @brief 读本地字典0x6077-0x0
     *
     * @param torque 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalActualTorque(hint16 *torque);

    /**
     * @brief 读本地字典0x6079-0x0
     *
     * @param voltage 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalDCLinkCircuitVoltage(huint32 *voltage);

    /**
     * @brief 读本地字典0x607A-0x0
     *
     * @param pos 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalTargetPos(hint32 *pos);

    /**
     * @brief 写本地字典0x607A-0x0
     *
     * @param pos 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalTargetPos(hint32 pos);

    /**
     * @brief 读本地字典0x60C1-0x1
     *
     * @param value 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalInterpolationDataRecord(hint32 *value);

    /**
     * @brief 写本地字典0x60C1-0x1
     *
     * @param value 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalInterpolationDataRecord(hint32 value);

    /**
     * @brief 读本地字典0x60c2-0x1
     *
     * @param value 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalInterpolationTimePeriodValue(huint8 *value);

    /**
     * @brief 写本地字典0x60c2-0x1
     *
     * @param value 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalInterpolationTimePeriodValue(huint8 value);

    /**
     * @brief 读本地字典0x60FF-0x0
     *
     * @param vel 存放读取的值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLocalTargetVelocity(hint32 *vel);

    /**
     * @brief 写本地字典0x60FF-0x0
     *
     * @param vel 写入值
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLocalTargetVelocity(hint32 vel);

    /**
     * @brief 读从站字典0x1000-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param deviceType 保存读取的设备类型
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getDeviceType(huint8 devIndex, huint8 id, huint32 *deviceType, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x1001-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param errReg 保存读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getErrorRegister(huint8 devIndex, huint8 id, huint8 *errReg, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x1003-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param number 保存读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getErrorFieldNumber(huint8 devIndex, huint8 id, huint8 *number, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x1003-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param number 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setErrorFieldNumber(huint8 devIndex, huint8 id, huint8 number, huint32 timeout = 100); // 0x1003-0

    /**
     * @brief 读从站字典0x1003-subIndex
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param subIndex 字典子索引(取值范围0x01~0x7f)
     * @param err 保存读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getErrorField(huint8 devIndex, huint8 id, huint8 subIndex, huint32 *err, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x1005-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param cobid 保存读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getSyncCobid(huint8 devIndex, huint8 id, huint32 *cobid, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x1005-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param cobid 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setSyncCobid(huint8 devIndex, huint8 id, huint32 cobid, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x1006-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param syncCycle 保存读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getSyncCycle(huint8 devIndex, huint8 id, huint32 *syncCycle, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x1006-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param syncCycle 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setSyncCycle(huint8 devIndex, huint8 id, huint32 syncCycle, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x1007-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param syncWindow 保存读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getSyncWindow(huint8 devIndex, huint8 id, huint32 *syncWindow, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x1007-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param syncWindow 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setSyncWindow(huint8 devIndex, huint8 id, huint32 syncWindow, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x1008-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param devName 保存读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getDeviceName(huint8 devIndex, huint8 id, char *devName, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x1009-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param hVersion 保存读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getHardwareVersion(huint8 devIndex, huint8 id, char *version, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x100A-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param version 保存读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getSoftwareVersion(huint8 devIndex, huint8 id, char *version, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x100C-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param time 保存读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getGuardTime(huint8 devIndex, huint8 id, huint16 *time, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x100C-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param time 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setGuardTime(huint8 devIndex, huint8 id, huint16 time, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x100D-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param factor 保存读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getLifeTimeFactor(huint8 devIndex, huint8 id, huint8 *factor, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x100D-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param factor 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setLifeTimeFactor(huint8 devIndex, huint8 id, huint8 factor, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x1010-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param number 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getStoreParametersNumber(huint8 devIndex, huint8 id, huint8 *number, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x1010-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param value 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getStoreParameters(huint8 devIndex, huint8 id, huint32 *value, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x1010-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param value 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setStoreParameters(huint8 devIndex, huint8 id, huint32 value, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x1011-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param number 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getRestoreDefaultParametersNumber(huint8 devIndex, huint8 id, huint8 *number, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x1011-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param value 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getRestoreDefaultParameters(huint8 devIndex, huint8 id, huint32 *value, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x1011-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param subIndex 子索引
     * @param value 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setRestoreDefaultParameters(huint8 devIndex, huint8 id, huint32 value, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x1014-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param cobid 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getEmergencyCobid(huint8 devIndex, huint8 id, huint32 *cobid, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x1014-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param cobid 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setEmergencyCobid(huint8 devIndex, huint8 id, huint32 cobid, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x1015-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param time 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getEmergencyInhibitTime(huint8 devIndex, huint8 id, huint16 *time, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x1015-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param time 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setEmergencyInhibitTime(huint8 devIndex, huint8 id, huint16 time, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x1016-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param number 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getHearbeatConsumerTimeParasNum(huint8 devIndex, huint8 id, huint8 *number, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x1016-subIndex
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param subIndex 子索引
     * @param time 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getHearbeatConsumerTime(huint8 devIndex, huint8 id, huint8 subIndex, huint32 *time, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x1016-subIndex
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param subIndex 子索引
     * @param time 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setHearbeatConsumerTime(huint8 devIndex, huint8 id, huint8 subIndex, huint32 time, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x1017-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param time 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getProducerHeartbeatTime(huint8 devIndex, huint8 id, huint16 *time, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x1017-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param time 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setProducerHeartbeatTime(huint8 devIndex, huint8 id, huint16 time, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x1018-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param number 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getIdentityObjectNumber(huint8 devIndex, huint8 id, huint8 *number, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x1018-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param vId 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getVendorID(huint8 devIndex, huint8 id, huint32 *vId, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x1018-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pCode 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getProductCode(huint8 devIndex, huint8 id, huint32 *pCode, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x1018-0x3
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param revisionNum 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getIdentityObjectRevisionNumber(huint8 devIndex, huint8 id, huint32 *revisionNum, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x1018-0x4
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param serialNum 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getSerialNumber(huint8 devIndex, huint8 id, huint32 *serialNum, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x1019-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param counter 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getSyncCounter(huint8 devIndex, huint8 id, huint8 *counter, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x1019-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param counter 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setSyncCounter(huint8 devIndex, huint8 id, huint8 counter, huint32 timeout = 100);

    /**
     * @brief 读从站字典[0x1200+sdoServerIndex]-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param sdoServerIndex sdo索引，设为0代表从站字典主索引0x1200，设为1代表从站字典主索引0x1201,以此类推
     * @param num 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServerSDONumberOfEntries(huint8 devIndex, huint8 id, huint16 sdoServerIndex, huint8 *num, huint32 timeout = 100);

    /**
     * @brief 写从站字典[0x1200+sdoServerIndex]-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param sdoServerIndex sdo索引，设为0代表从站字典主索引0x1200，设为1代表从站字典主索引0x1201,以此类推
     * @param num 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServerSDONumberOfEntries(huint8 devIndex, huint8 id, huint16 sdoServerIndex, huint8 num, huint32 timeout = 100);

    /**
     * @brief 读从站字典[0x1200+sdoServerIndex]-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param sdoServerIndex sdo索引，设为0代表从站字典主索引0x1200，设为1代表从站字典主索引0x1201,以此类推
     * @param cobId 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServerSDOCobIdClientToServer(huint8 devIndex, huint8 id, huint16 sdoServerIndex, huint32 *cobId, huint32 timeout = 100);

    /**
     * @brief 写从站字典[0x1200+sdoServerIndex]-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param sdoServerIndex sdo索引，设为0代表从站字典主索引0x1200，设为1代表从站字典主索引0x1201,以此类推
     * @param cobId 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServerSDOCobIdClientToServer(huint8 devIndex, huint8 id, huint16 sdoServerIndex, huint32 cobId, huint32 timeout = 100);

    /**
     * @brief 读从站字典[0x1200+sdoServerIndex]-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param sdoServerIndex sdo索引，设为0代表从站字典主索引0x1200，设为1代表从站字典主索引0x1201,以此类推
     * @param cobId 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServerSDOCobIdServerToClient(huint8 devIndex, huint8 id, huint16 sdoServerIndex, huint32 *cobId, huint32 timeout = 100);

    /**
     * @brief 写从站字典[0x1200+sdoServerIndex]-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param sdoServerIndex sdo索引，设为0代表从站字典主索引0x1200，设为1代表从站字典主索引0x1201,以此类推
     * @param cobId 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServerSDOCobIdServerToClient(huint8 devIndex, huint8 id, huint16 sdoServerIndex, huint32 cobId, huint32 timeout = 100);

    /**
     * @brief 读从站字典[0x1200+sdoServerIndex]-0x3
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param sdoServerIndex sdo索引，设为0代表从站字典主索引0x1200，设为1代表从站字典主索引0x1201,以此类推
     * @param nodeId 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServerSDONodeIdOfClient(huint8 devIndex, huint8 id, huint16 sdoServerIndex, huint8 *nodeId, huint32 timeout = 100);

    /**
     * @brief 写从站字典[0x1200+sdoServerIndex]-0x3
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param sdoServerIndex sdo索引，设为0代表从站字典主索引0x1200，设为1代表从站字典主索引0x1201,以此类推
     * @param nodeId 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServerSDONodeIdOfClient(huint8 devIndex, huint8 id, huint16 sdoServerIndex, huint8 nodeId, huint32 timeout = 100);

    /**
     * @brief 读从站字典[0x1280+sdoClientIndex]-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param sdoClientIndex sdo索引，设为0代表从站字典主索引0x1280，设为1代表从站字典主索引0x1281,以此类推
     * @param number 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getClientSDONumberOfEntries(huint8 devIndex, huint8 id, huint16 sdoClientIndex, huint8 *number, huint32 timeout = 100);

    /**
     * @brief 写从站字典[0x1280+sdoClientIndex]-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param sdoClientIndex sdo索引，设为0代表从站字典主索引0x1280，设为1代表从站字典主索引0x1281,以此类推
     * @param number 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setClientSDONumberOfEntries(huint8 devIndex, huint8 id, huint16 sdoClientIndex, huint8 number, huint32 timeout = 100);

    /**
     * @brief 读从站字典[0x1280+sdoClientIndex]-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param sdoClientIndex sdo索引，设为0代表从站字典主索引0x1280，设为1代表从站字典主索引0x1281,以此类推
     * @param cobId 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getClientSDOCobIdClientToServer(huint8 devIndex, huint8 id, huint16 sdoClientIndex, huint32 *cobId, huint32 timeout = 100);

    /**
     * @brief 写从站字典[0x1280+sdoClientIndex]-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param sdoClientIndex sdo索引，设为0代表从站字典主索引0x1280，设为1代表从站字典主索引0x1281,以此类推
     * @param cobId 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setClientSDOCobIdClientToServer(huint8 devIndex, huint8 id, huint16 sdoClientIndex, huint32 cobId, huint32 timeout = 100);

    /**
     * @brief 读从站字典[0x1280+sdoClientIndex]-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param sdoClientIndex sdo索引，设为0代表从站字典主索引0x1280，设为1代表从站字典主索引0x1281,以此类推
     * @param cobId 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getClientSDOCobIdServerToClient(huint8 devIndex, huint8 id, huint16 sdoClientIndex, huint32 *cobId, huint32 timeout = 100);

    /**
     * @brief 写从站字典[0x1280+sdoClientIndex]-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param sdoClientIndex sdo索引，设为0代表从站字典主索引0x1280，设为1代表从站字典主索引0x1281,以此类推
     * @param cobId 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setClientSDOCobIdServerToClient(huint8 devIndex, huint8 id, huint16 sdoClientIndex, huint32 cobId, huint32 timeout = 100);

    /**
     * @brief 读从站字典[0x1280+sdoClientIndex]-0x3
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param sdoClientIndex sdo索引，设为0代表从站字典主索引0x1280，设为1代表从站字典主索引0x1281,以此类推
     * @param nodeId 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getClientSDONodeIdOfSDOServer(huint8 devIndex, huint8 id, huint16 sdoClientIndex, huint8 *nodeId, huint32 timeout = 100);

    /**
     * @brief 写从站字典[0x1280+sdoClientIndex]-0x3
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param sdoClientIndex sdo索引，设为0代表从站字典主索引0x1280，设为1代表从站字典主索引0x1281,以此类推
     * @param nodeId 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setClientSDONodeIdOfSDOServer(huint8 devIndex, huint8 id, huint16 sdoClientIndex, huint8 nodeId, huint32 timeout = 100);

    /**
     * @brief 读从站字典[0x1400+pdoIndex]-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1400，设为1代表从站字典主索引0x1401,以此类推
     * @param count 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getRPDOMaxParasCount(huint8 devIndex, huint8 id, huint16 pdoIndex, huint8 *count, huint32 timeout = 100);

    /**
     * @brief 写从站字典[0x1400+pdoIndex]-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1400，设为1代表从站字典主索引0x1401,以此类推
     * @param count 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setRPDOMaxParasCount(huint8 devIndex, huint8 id, huint16 pdoIndex, huint8 count, huint32 timeout = 100);

    /**
     * @brief 读从站字典[0x1400+pdoIndex]-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1400，设为1代表从站字典主索引0x1401,以此类推
     * @param cobId 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getRPDOCobId(huint8 devIndex, huint8 id, huint16 pdoIndex, huint32 *cobId, huint32 timeout = 100);

    /**
     * @brief 写从站字典[0x1400+pdoIndex]-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1400，设为1代表从站字典主索引0x1401,以此类推
     * @param cobId 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setRPDOCobId(huint8 devIndex, huint8 id, huint16 pdoIndex, huint32 cobId, huint32 timeout = 100);

    /**
     * @brief 读从站字典[0x1400+pdoIndex]-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1400，设为1代表从站字典主索引0x1401,以此类推
     * @param type 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getRPDOTransmitType(huint8 devIndex, huint8 id, huint16 pdoIndex, huint8 *type, huint32 timeout = 100);

    /**
     * @brief 写从站字典[0x1400+pdoIndex]-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1400，设为1代表从站字典主索引0x1401,以此类推
     * @param type 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setRPDOTransmitType(huint8 devIndex, huint8 id, huint16 pdoIndex, huint8 type, huint32 timeout = 100);

    /**
     * @brief 读从站字典[0x1400+pdoIndex]-0x3
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1400，设为1代表从站字典主索引0x1401,以此类推
     * @param time 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getRPDOInhibitTime(huint8 devIndex, huint8 id, huint16 pdoIndex, huint16 *time, huint32 timeout = 100);

    /**
     * @brief 写从站字典[0x1400+pdoIndex]-0x3
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1400，设为1代表从站字典主索引0x1401,以此类推
     * @param time 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setRPDOInhibitTime(huint8 devIndex, huint8 id, huint16 pdoIndex, huint16 time, huint32 timeout = 100);

    /**
     * @brief 读从站字典[0x1400+pdoIndex]-0x5
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1400，设为1代表从站字典主索引0x1401,以此类推
     * @param timer 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getRPDOEventTimer(huint8 devIndex, huint8 id, huint16 pdoIndex, huint16 *timer, huint32 timeout = 100);

    /**
     * @brief 写从站字典[0x1400+pdoIndex]-0x5
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1400，设为1代表从站字典主索引0x1401,以此类推
     * @param timer 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setRPDOEventTimer(huint8 devIndex, huint8 id, huint16 pdoIndex, huint16 timer, huint32 timeout = 100);

    /**
     * @brief 读从站字典[0x1400+pdoIndex]-0x6
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1400，设为1代表从站字典主索引0x1401,以此类推
     * @param value 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getRPDOSYNCStartValue(huint8 devIndex, huint8 id, huint16 pdoIndex, huint8 *value, huint32 timeout = 100);

    /**
     * @brief 写从站字典[0x1400+pdoIndex]-0x6
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1400，设为1代表从站字典主索引0x1401,以此类推
     * @param value 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setRPDOSYNCStartValue(huint8 devIndex, huint8 id, huint16 pdoIndex, huint8 value, huint32 timeout = 100); // 1400-6

    /**
     * @brief 读从站字典[0x1600+pdoIndex]-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1600，设为1代表从站字典主索引0x1601,以此类推
     * @param count 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getRPDOMaxMappedCount(huint8 devIndex, huint8 id, huint16 pdoIndex, huint8 *count, huint32 timeout = 100);

    /**
     * @brief 写从站字典[0x1600+pdoIndex]-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1600，设为1代表从站字典主索引0x1601,以此类推
     * @param count 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setRPDOMaxMappedCount(huint8 devIndex, huint8 id, huint16 pdoIndex, huint8 count, huint32 timeout = 100);

    /**
     * @brief 读从站字典[0x1600+pdoIndex]-[0x1+mapIndex]
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1600，设为1代表从站字典主索引0x1601,以此类推
     * @param mapIndex 映射地址索引，设为0代表从站字典子索引0x1, 设为1代表子索引0x2,以此类推
     * @param value 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getRPDOMapped(huint8 devIndex, huint8 id, huint16 pdoIndex, huint8 mapIndex, huint32 *value, huint32 timeout = 100);

    /**
     * @brief 写从站字典[0x1600+pdoIndex]-[0x1+mapIndex]
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1600，设为1代表从站字典主索引0x1601,以此类推
     * @param mapIndex 映射地址索引，设为0代表从站字典子索引0x1, 设为1代表子索引0x2,以此类推
     * @param value 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setRPDOMapped(huint8 devIndex, huint8 id, huint16 pdoIndex, huint8 mapIndex, huint32 value, huint32 timeout = 100);

    /**
     * @brief 读从站字典[0x1800+pdoIndex]-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1800，设为1代表从站字典主索引0x1801,以此类推
     * @param count 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getTPDOMaxParasCount(huint8 devIndex, huint8 id, huint16 pdoIndex, huint8 *count, huint32 timeout = 100);

    /**
     * @brief 写从站字典[0x1800+pdoIndex]-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1800，设为1代表从站字典主索引0x1801,以此类推
     * @param count 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setTPDOMaxParasCount(huint8 devIndex, huint8 id, huint16 pdoIndex, huint8 count, huint32 timeout = 100);

    /**
     * @brief 读从站字典[0x1800+pdoIndex]-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1800，设为1代表从站字典主索引0x1801,以此类推
     * @param cobId 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getTPDOCobId(huint8 devIndex, huint8 id, huint16 pdoIndex, huint32 *cobId, huint32 timeout = 100);

    /**
     * @brief 写从站字典[0x1800+pdoIndex]-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1800，设为1代表从站字典主索引0x1801,以此类推
     * @param cobId 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setTPDOCobId(huint8 devIndex, huint8 id, huint16 pdoIndex, huint32 cobId, huint32 timeout = 100);

    /**
     * @brief 读从站字典[0x1800+pdoIndex]-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1800，设为1代表从站字典主索引0x1801,以此类推
     * @param type 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getTPDOTransmitType(huint8 devIndex, huint8 id, huint16 pdoIndex, huint8 *type, huint32 timeout = 100);

    /**
     * @brief 写从站字典[0x1800+pdoIndex]-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1800，设为1代表从站字典主索引0x1801,以此类推
     * @param type 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setTPDOTransmitType(huint8 devIndex, huint8 id, huint16 pdoIndex, huint8 type, huint32 timeout = 100);

    /**
     * @brief 读从站字典[0x1800+pdoIndex]-0x3
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1800，设为1代表从站字典主索引0x1801,以此类推
     * @param time 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getTPDOInhibitTime(huint8 devIndex, huint8 id, huint16 pdoIndex, huint16 *time, huint32 timeout = 100);

    /**
     * @brief 写从站字典[0x1800+pdoIndex]-0x3
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1800，设为1代表从站字典主索引0x1801,以此类推
     * @param time 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setTPDOInhibitTime(huint8 devIndex, huint8 id, huint16 pdoIndex, huint16 time, huint32 timeout = 100);

    /**
     * @brief 读从站字典[0x1800+pdoIndex]-0x5
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1800，设为1代表从站字典主索引0x1801,以此类推
     * @param timer 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getTPDOEventTimer(huint8 devIndex, huint8 id, huint16 pdoIndex, huint16 *timer, huint32 timeout = 100);

    /**
     * @brief 写从站字典[0x1800+pdoIndex]-0x5
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1800，设为1代表从站字典主索引0x1801,以此类推
     * @param timer 写入值
     * @param timeout 等待确认时间，单位
     */
    EXTERNFUNC int harmonic_setTPDOEventTimer(huint8 devIndex, huint8 id, huint16 pdoIndex, huint16 timer, huint32 timeout = 100);

    /**
     * @brief 读从站字典[0x1800+pdoIndex]-0x6
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1800，设为1代表从站字典主索引0x1801,以此类推
     * @param value 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getTPDOSYNCStartValue(huint8 devIndex, huint8 id, huint16 pdoIndex, huint8 *value, huint32 timeout = 100);

    /**
     * @brief 写从站字典[0x1800+pdoIndex]-0x6
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1800，设为1代表从站字典主索引0x1801,以此类推
     * @param value 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setTPDOSYNCStartValue(huint8 devIndex, huint8 id, huint16 pdoIndex, huint8 value, huint32 timeout = 100);

    /**
     * @brief 读从站字典[0x1A00+pdoIndex]-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1A00，设为1代表从站字典主索引0x1A01,以此类推
     * @param count 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getTPDOMaxMappedCount(huint8 devIndex, huint8 id, huint16 pdoIndex, huint8 *count, huint32 timeout = 100);

    /**
     * @brief 写从站字典[0x1A00+pdoIndex]-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1a00，设为1代表从站字典主索引0x1a01,以此类推
     * @param count 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setTPDOMaxMappedCount(huint8 devIndex, huint8 id, huint16 pdoIndex, huint8 count, huint32 timeout = 100);

    /**
     * @brief 读从站字典[0x1A00+pdoIndex]-[0x1+mapIndex]
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1A00，设为1代表从站字典主索引0x1A01,以此类推
     * @param mapIndex 映射索引，设为0代表从站字典子索引0x1,设为1代表从站字典子索引0x2,以此类推
     * @param value 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getTPDOMapped(huint8 devIndex, huint8 id, huint16 pdoIndex, huint8 mapIndex, huint32 *value, huint32 timeout = 100);

    /**
     * @brief 写从站字典[0x1A00+pdoIndex]-[0x1+mapIndex]
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pdoIndex pdo索引，设为0代表从站字典主索引0x1A00，设为1代表从站字典主索引0x1A01,以此类推
     * @param mapIndex 映射索引，设为0代表从站字典子索引0x1,设为1代表从站字典子索引0x2,以此类推
     * @param value 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setTPDOMapped(huint8 devIndex, huint8 id, huint16 pdoIndex, huint8 mapIndex, huint32 value, huint32 timeout = 100); // 1A00-1

    /**
     * @brief 读从站字典0x2001-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param num 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoParametersNumber(huint8 devIndex, huint8 id, huint8 *num, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2001-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param num 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServoParametersNumber(huint8 devIndex, huint8 id, huint8 num, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2001-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param servoID 存放读取的伺服id
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoID(huint8 devIndex, huint8 id, huint8 *servoID, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2001-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param servoID 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServoID(huint8 devIndex, huint8 id, huint8 servoID, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2001-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param baudrate 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoCanBaudrate(huint8 devIndex, huint8 id, huint16 *baudrate, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2001-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param baudrate 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServoCanBaudrate(huint8 devIndex, huint8 id, huint16 baudrate, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2003-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param state 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoSoftwareLimitState(huint8 devIndex, huint8 id, huint32 *state, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2003-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param state 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServoSoftwareLimitState(huint8 devIndex, huint8 id, huint32 state, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2004-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param mode 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoCommunicationMode(huint8 devIndex, huint8 id, huint8 *mode, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2004-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param mode 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServoCommunicationMode(huint8 devIndex, huint8 id, huint8 mode, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2010-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param num 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoCurrentLoopPINum(huint8 devIndex, huint8 id, huint8 *num, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2010-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param kp 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getDefaultServoCurrentLoopKP(huint8 devIndex, huint8 id, huint16 *kp, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2010-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param kp 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setDefaultServoCurrentLoopKP(huint8 devIndex, huint8 id, huint16 kp, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2010-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param ki 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getDefaultServoCurrentLoopKI(huint8 devIndex, huint8 id, huint16 *ki, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2010-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param ki 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setDefaultServoCurrentLoopKI(huint8 devIndex, huint8 id, huint16 ki, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2010-0x3
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param kp 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoCurrentLoopKP(huint8 devIndex, huint8 id, huint16 *kp, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2010-0x3
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param kp 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServoCurrentLoopKP(huint8 devIndex, huint8 id, huint16 kp, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2010-0x4
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param ki 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoCurrentLoopKI(huint8 devIndex, huint8 id, huint16 *ki, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2010-0x4
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param ki 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServoCurrentLoopKI(huint8 devIndex, huint8 id, huint16 ki, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2012-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param num 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoVelocityLoopPINum(huint8 devIndex, huint8 id, huint8 *num, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2012-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param kp 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getDefaultServoVelocityLoopKP(huint8 devIndex, huint8 id, huint16 *kp, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2012-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param kp 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setDefaultServoVelocityLoopKP(huint8 devIndex, huint8 id, huint16 kp, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2012-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param ki 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getDefaultServoVelocityLoopKI(huint8 devIndex, huint8 id, huint16 *ki, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2012-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param ki 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setDefaultServoVelocityLoopKI(huint8 devIndex, huint8 id, huint16 ki, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2012-0x3
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param kp 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoVelocityLoopKP(huint8 devIndex, huint8 id, huint16 *kp, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2012-0x3
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param kp 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServoVelocityLoopKP(huint8 devIndex, huint8 id, huint16 kp, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2012-0x4
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param ki 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoVelocityLoopKI(huint8 devIndex, huint8 id, huint16 *ki, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2012-0x4
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param ki 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServoVelocityLoopKI(huint8 devIndex, huint8 id, huint16 ki, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2013-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param num 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoPositionLoopPINum(huint8 devIndex, huint8 id, huint8 *num, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2013-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param kp 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getDefaultServoPositionLoopKP(huint8 devIndex, huint8 id, huint16 *kp, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2013-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param kp 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setDefaultServoPositionLoopKP(huint8 devIndex, huint8 id, huint16 kp, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2013-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param ki 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getDefaultServoPositionLoopKI(huint8 devIndex, huint8 id, huint16 *ki, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2013-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param ki 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setDefaultServoPositionLoopKI(huint8 devIndex, huint8 id, huint16 ki, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2013-0x3
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param kp 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoPositionLoopKP(huint8 devIndex, huint8 id, huint16 *kp, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2013-0x3
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param kp 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServoPositionLoopKP(huint8 devIndex, huint8 id, huint16 kp, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2013-0x4
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param ki 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoPositionLoopKI(huint8 devIndex, huint8 id, huint16 *ki, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2013-0x4
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param ki 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServoPositionLoopKI(huint8 devIndex, huint8 id, huint16 ki, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2014-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param num 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoBrakeParaNum(huint8 devIndex, huint8 id, huint8 *num, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2014-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param status 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoBrakeControl(huint8 devIndex, huint8 id, huint8 *status, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2014-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param status 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServoBrakeControl(huint8 devIndex, huint8 id, huint8 status, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2014-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param state 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getBrakeState(huint8 devIndex, huint8 id, huint8 *state, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2014-0x2(设置抱闸状态)
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param state 写入值，true:为打开抱闸 false:关闭抱闸
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setBrakeState(huint8 devIndex, huint8 id, huint8 state, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2016-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param num 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoTemperatureParasNum(huint8 devIndex, huint8 id, huint8 *num, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2016-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param tem 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoTemperature(huint8 devIndex, huint8 id, hint8 *tem, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2016-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param tem 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoTemperatureLimit(huint8 devIndex, huint8 id, hint8 *tem, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2016-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param tem 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServoTemperatureLimit(huint8 devIndex, huint8 id, huint8 tem, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2016-0x3
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param time 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoTemperatureLimitTime(huint8 devIndex, huint8 id, huint16 *time, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2016-0x3
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param time 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServoTemperatureLimitTime(huint8 devIndex, huint8 id, huint16 time, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2017-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param limit 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoVelocityIntLimit(huint8 devIndex, huint8 id, huint16 *limit, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2017-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param limit 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServoVelocityIntLimit(huint8 devIndex, huint8 id, huint16 limit, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2020-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param num 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoBlockParasNum(huint8 devIndex, huint8 id, huint8 *num, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2020-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param torque 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoBlockTorque(huint8 devIndex, huint8 id, huint16 *torque, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2020-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param torque 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServoBlockTorque(huint8 devIndex, huint8 id, huint16 torque, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2020-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param time 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoBlockTime(huint8 devIndex, huint8 id, huint16 *time, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2020-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param time 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServoBlockTime(huint8 devIndex, huint8 id, huint16 time, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2020-0x3
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param velocity 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoBlockVelocity(huint8 devIndex, huint8 id, huint32 *velocity, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2020-0x3
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param velocity 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServoBlockVelocity(huint8 devIndex, huint8 id, huint32 velocity, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2021-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param num 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoVelocityFlowingErrorParasNum(huint8 devIndex, huint8 id, huint8 *num, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2021-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param error 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoVelocityFlowingErrorWindows(huint8 devIndex, huint8 id, huint32 *error, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2021-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param error 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServoVelocityFlowingErrorWindows(huint8 devIndex, huint8 id, huint32 error, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2021-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param time 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoVelocityFlowingErrorTime(huint8 devIndex, huint8 id, huint16 *time, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2021-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param time 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServoVelocityFlowingErrorTime(huint8 devIndex, huint8 id, huint16 time, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2022-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param window 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoTorqueWindow(huint8 devIndex, huint8 id, huint16 *window, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2022-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param window 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServoTorqueWindow(huint8 devIndex, huint8 id, huint16 window, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2023-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param window 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoTorqueWindowTime(huint8 devIndex, huint8 id, huint16 *window, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2023-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param window 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServoTorqueWindowTime(huint8 devIndex, huint8 id, huint16 window, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2024-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param thre 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoOverSpeedThreshold(huint8 devIndex, huint8 id, huint32 *thre, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2024-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param thre 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServoOverSpeedThreshold(huint8 devIndex, huint8 id, huint32 thre, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2025-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param time 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoOverSpeedTime(huint8 devIndex, huint8 id, huint16 *time, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2025-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param time 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServoOverSpeedTime(huint8 devIndex, huint8 id, huint16 time, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2026-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param time 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoBrakeDelayTime(huint8 devIndex, huint8 id, huint8 *time, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2026-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param time 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setServoBrakeDelayTime(huint8 devIndex, huint8 id, huint8 time, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2028-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param value 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getI2tOverLoadLimit(huint8 devIndex, huint8 id, huint16 *value, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2028-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param value 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setI2tOverLoadLimit(huint8 devIndex, huint8 id, huint16 value, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2029-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param value 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getI2tOverLoadValue(huint8 devIndex, huint8 id, huint16 *value, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x202A-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param value 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoFirstEncoderValue(huint8 devIndex, huint8 id, hint32 *value, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x202B-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param value 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoSecondEncoderValue(huint8 devIndex, huint8 id, hint32 *value, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x202C-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param value 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoThetaBiasValue(huint8 devIndex, huint8 id, hint32 *value, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x202D-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param number 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getDCVolageProtectionParasNumber(huint8 devIndex, huint8 id, huint8 *number, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x202D-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param vol 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getDCUnderVoltageThreshold(huint8 devIndex, huint8 id, huint32 *vol, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x202D-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param vol 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setDCUnderVoltageThreshold(huint8 devIndex, huint8 id, huint32 vol, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x202D-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param time 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getDCUnderVoltageTime(huint8 devIndex, huint8 id, huint32 *time, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x202D-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param time 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setDCUnderVoltageTime(huint8 devIndex, huint8 id, huint32 time, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x202D-0x3
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param vol 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getDCOverVoltageThreshold(huint8 devIndex, huint8 id, huint32 *vol, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x202D-0x3
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param vol 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setDCOverVoltageThreshold(huint8 devIndex, huint8 id, huint32 vol, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x202D-0x4
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param time 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getDCOverVoltageTime(huint8 devIndex, huint8 id, huint32 *time, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x202D-0x4
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param time 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setDCOverVoltageTime(huint8 devIndex, huint8 id, huint32 time, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2030-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param value 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getTorqueSensorValue(huint8 devIndex, huint8 id, hreal32 *value, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2101-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param flag 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getFindPointDoneFlag(huint8 devIndex, huint8 id, huint32 *flag, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2101-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param flag 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setFindPointDoneFlag(huint8 devIndex, huint8 id, huint32 flag, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2102-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param status 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getFindPointStart(huint8 devIndex, huint8 id, huint32 *status, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2102-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param status 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setFindPointStart(huint8 devIndex, huint8 id, huint32 status, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2105-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param angle 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getFindPointMoveMax(huint8 devIndex, huint8 id, huint32 *angle, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2105-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param angle 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setFindPointMoveMax(huint8 devIndex, huint8 id, huint32 angle, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2106-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param acc 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getFindPointMoveAcc(huint8 devIndex, huint8 id, huint32 *acc, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2106-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param acc 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setFindPointMoveAcc(huint8 devIndex, huint8 id, huint32 acc, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2107-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param dec 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getFindPointMoveDec(huint8 devIndex, huint8 id, huint32 *dec, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2107-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param dec 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setFindPointMoveDec(huint8 devIndex, huint8 id, huint32 dec, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2108-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param vel 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getFindPointMoveVelocity(huint8 devIndex, huint8 id, huint32 *vel, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2108-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param vel 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setFindPointMoveVelocity(huint8 devIndex, huint8 id, huint32 vel, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2110-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param factor 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getTorqueFactor(huint8 devIndex, huint8 id, huint16 *factor, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2110-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param factor 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setTorqueFactor(huint8 devIndex, huint8 id, huint16 factor, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2130-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param cobcmd 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getMitCtrlCobcmd1(huint8 devIndex, huint8 id, huint32 *cobcmd, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2130-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param cobcmd 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setMitCtrlCobcmd1(huint8 devIndex, huint8 id, huint32 cobcmd, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2131-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param cobcmd 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getMitCtrlCobcmd2(huint8 devIndex, huint8 id, huint32 *cobcmd, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2131-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param cobcmd 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setMitCtrlCobcmd2(huint8 devIndex, huint8 id, huint32 cobcmd, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2132-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param cobdat 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getMitReplyCobdat1(huint8 devIndex, huint8 id, huint32 *cobdat, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2132-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param cobdat 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setMitReplyCobdat1(huint8 devIndex, huint8 id, huint32 cobdat, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2133-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param cobdat 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getMitReplyCobdat2(huint8 devIndex, huint8 id, huint32 *cobdat, huint32 timeout = 100);

    /**
     * @brief 从站字典0x2133-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param cobdat 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setMitReplyCobdat2(huint8 devIndex, huint8 id, huint32 cobdat, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2140-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pMin 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getMitPMin(huint8 devIndex, huint8 id, hreal32 *pMin, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2140-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pMin 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setMitPMin(huint8 devIndex, huint8 id, hreal32 pMin, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2141-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pMax 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getMitPMax(huint8 devIndex, huint8 id, hreal32 *pMax, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2141-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pMax 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setMitPMax(huint8 devIndex, huint8 id, hreal32 pMax, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2142-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param vMin 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getMitVMin(huint8 devIndex, huint8 id, hreal32 *vMin, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2142-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param vMin 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setMitVMin(huint8 devIndex, huint8 id, hreal32 vMin, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2143-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param vMax 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getMitVMax(huint8 devIndex, huint8 id, hreal32 *vMax, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2143-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param vMax 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setMitVMax(huint8 devIndex, huint8 id, hreal32 vMax, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2144-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param kpMin 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getMitKPMin(huint8 devIndex, huint8 id, hreal32 *kpMin, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2144-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param kpMin 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setMitKPMin(huint8 devIndex, huint8 id, hreal32 kpMin, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2145-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param kpMax 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getMitKPMax(huint8 devIndex, huint8 id, hreal32 *kpMax, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2145-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param kpMax 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setMitKPMax(huint8 devIndex, huint8 id, hreal32 kpMax, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2146-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param kdMin 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getMitKDMin(huint8 devIndex, huint8 id, hreal32 *kdMin, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2146-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param kdMin 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setMitKDMin(huint8 devIndex, huint8 id, hreal32 kdMin, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2147-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param kdMax 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getMitKDMax(huint8 devIndex, huint8 id, hreal32 *kdMax, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2147-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param kdMax 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setMitKDMax(huint8 devIndex, huint8 id, hreal32 kdMax, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2148-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param tMin 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getMitTMin(huint8 devIndex, huint8 id, hreal32 *tMin, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2148-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param tMin 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setMitTMin(huint8 devIndex, huint8 id, hreal32 tMin, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x2149-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param tMin 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getMitTMax(huint8 devIndex, huint8 id, hreal32 *tMin, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x2149-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param tMin 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setMitTMax(huint8 devIndex, huint8 id, hreal32 tMin, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x603F-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param err 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getServoErrorCode(huint8 devIndex, huint8 id, huint16 *err, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x6040-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param word 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getControlword(huint8 devIndex, huint8 id, huint16 *word, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x6040-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param word 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setControlword(huint8 devIndex, huint8 id, huint16 word, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x6041-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param word 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getStatusWord(huint8 devIndex, huint8 id, huint16 *word, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x605A-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param option 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getQuickStopOption(huint8 devIndex, huint8 id, harmonic_QuickStopOption *option, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x605A-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param option 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setQuickStopOption(huint8 devIndex, huint8 id, harmonic_QuickStopOption option, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x605B-0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param code 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getShutdownOptionCode(huint8 devIndex, huint8 id, harmonic_ShutdownOption *code, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x605B-0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param code 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setShutdownOptionCode(huint8 devIndex, huint8 id, harmonic_ShutdownOption code, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x605C-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param code 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getDisableOperationOptionCode(huint8 devIndex, huint8 id, harmonic_DisableOperationOption *code, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x605C-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param code 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setDisableOperationOptionCode(huint8 devIndex, huint8 id, harmonic_DisableOperationOption code, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x605D-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param option 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getHaltOption(huint8 devIndex, huint8 id, harmonic_HaltOption *option, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x605D-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param option 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setHaltOption(huint8 devIndex, huint8 id, harmonic_HaltOption option, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x605E-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param option 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getFaultReactionOption(huint8 devIndex, huint8 id, harmonic_FaultReactionOption *option, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x605E-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param option 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setFaultReactionOption(huint8 devIndex, huint8 id, harmonic_FaultReactionOption option, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x6060-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param mode 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getOperateMode(huint8 devIndex, huint8 id, harmonic_OperateMode *mode, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x6060-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param mode 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setOperateMode(huint8 devIndex, huint8 id, harmonic_OperateMode mode, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x6061-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param mode 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getDisplayOperateMode(huint8 devIndex, huint8 id, harmonic_OperateMode *mode, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x6062-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pos 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getDemandPos(huint8 devIndex, huint8 id, hint32 *pos, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x6062-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pos 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setDemandPos(huint8 devIndex, huint8 id, hint32 pos, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x6064-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pos 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getActualPos(huint8 devIndex, huint8 id, hint32 *pos, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x6065-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param eWindow 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getFollowingErrorWindow(huint8 devIndex, huint8 id, huint32 *eWindow, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x6065-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param eWindow 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setFollowingErrorWindow(huint8 devIndex, huint8 id, huint32 eWindow, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x6067-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pWindow 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getPositionWindow(huint8 devIndex, huint8 id, huint32 *pWindow, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x6067-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pWindow 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setPositionWindow(huint8 devIndex, huint8 id, huint32 pWindow, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x6068-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pWindowTime 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getPositionWindowTime(huint8 devIndex, huint8 id, huint16 *pWindowTime, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x6068-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pWindowTime 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setPositionWindowTime(huint8 devIndex, huint8 id, huint16 pWindowTime, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x606B-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param dVel 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getDemandVelocity(huint8 devIndex, huint8 id, hint32 *dVel, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x606B-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param dvel 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setDemandVelocity(huint8 devIndex, huint8 id, hint32 dvel, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x606C-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param vel 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getActualVelocity(huint8 devIndex, huint8 id, hint32 *vel, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x606D-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param vWindow 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getVelocityWindow(huint8 devIndex, huint8 id, huint16 *vWindow, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x606D-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param vWindow 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setVelocityWindow(huint8 devIndex, huint8 id, huint16 vWindow, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x606E-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param wTime 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getVelocityWindowTime(huint8 devIndex, huint8 id, huint16 *wTime, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x606E-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param wTime 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setVelocityWindowTime(huint8 devIndex, huint8 id, huint16 wTime, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x606F-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param thres 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getVelocityThreshold(huint8 devIndex, huint8 id, huint16 *thres, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x606F-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param thres 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setVelocityThreshold(huint8 devIndex, huint8 id, huint16 thres, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x6070-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param tTime 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getVelocityThresholdTime(huint8 devIndex, huint8 id, huint16 *tTime, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x6070-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param tTime 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setVelocityThresholdTime(huint8 devIndex, huint8 id, huint16 tTime, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x6071-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param torque 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getTargetTorque(huint8 devIndex, huint8 id, hint16 *torque, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x6071-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param torque 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setTargetTorque(huint8 devIndex, huint8 id, hint16 torque, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x6072-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param torque 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getTorqueLimit(huint8 devIndex, huint8 id, hint16 *torque, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x6072-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param torque 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setTorqueLimit(huint8 devIndex, huint8 id, hint16 torque, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x6074-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param torque 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getDemandTorque(huint8 devIndex, huint8 id, hint16 *torque, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x6074-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param torque 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setDemandTorque(huint8 devIndex, huint8 id, hint16 torque, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x6075-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param current 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getRatedCurrent(huint8 devIndex, huint8 id, huint32 *current, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x6075-0x0
     * 
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param current 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setRatedCurrent(huint8 devIndex, huint8 id, huint32 current, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x6076-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param torque 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getMotorRatedTorque(huint8 devIndex, huint8 id, huint32 *torque, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x6076-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param torque 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setMotorRatedTorque(huint8 devIndex, huint8 id, huint32 torque, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x6077-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param torque 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getActualTorque(huint8 devIndex, huint8 id, hint16 *torque, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x6078-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param current 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getActualCurrent(huint8 devIndex, huint8 id, hint16 *current, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x6079-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param voltage 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getDCLinkCircuitVoltage(huint8 devIndex, huint8 id, huint32 *voltage, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x6079-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param voltage 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setDCLinkCircuitVoltage(huint8 devIndex, huint8 id, huint32 voltage, huint32 timeout = 100);

    /**
     * @brief 0x607A-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pos 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getTargetPos(huint8 devIndex, huint8 id, hint32 *pos, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x607A-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pos 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setTargetPos(huint8 devIndex, huint8 id, hint32 pos, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x607C-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param offSet 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getHomeOffset(huint8 devIndex, huint8 id, hint32 *offSet, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x607C-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param offSet 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setHomeOffset(huint8 devIndex, huint8 id, hint32 offSet, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x607D-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pos 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getSoftwareMinimumPosition(huint8 devIndex, huint8 id, hint32 *pos, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x607D-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pos 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setSoftwareMinimumPosition(huint8 devIndex, huint8 id, hint32 pos, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x607D-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pos 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getSoftwareMaxmumPosition(huint8 devIndex, huint8 id, hint32 *pos, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x607D-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param pos 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setSoftwareMaxmumPosition(huint8 devIndex, huint8 id, hint32 pos, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x607F-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param vel 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getMaxProfileVelocity(huint8 devIndex, huint8 id, huint32 *vel, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x607F-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param vel 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setMaxProfileVelocity(huint8 devIndex, huint8 id, huint32 vel, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x6081-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param vel 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getProfileVelocity(huint8 devIndex, huint8 id, huint32 *vel, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x6081-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param vel 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setProfileVelocity(huint8 devIndex, huint8 id, huint32 vel, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x6083-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param acc 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getProfileAcceleration(huint8 devIndex, huint8 id, huint32 *acc, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x6083-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param acc 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setProfileAcceleration(huint8 devIndex, huint8 id, huint32 acc, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x6084-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param dec 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getProfileDeceleration(huint8 devIndex, huint8 id, huint32 *dec, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x6084-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param dec 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setProfileDeceleration(huint8 devIndex, huint8 id, huint32 dec, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x6085-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param dec 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getQuickStopDeceleration(huint8 devIndex, huint8 id, huint32 *dec, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x6085-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param dec 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setQuickStopDeceleration(huint8 devIndex, huint8 id, huint32 dec, huint32 timeout = 100);

    /**
     * @brief 0x6087-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param slope 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getTorqueSlope(huint8 devIndex, huint8 id, huint32 *slope, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x6087-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param slope 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setTorqueSlope(huint8 devIndex, huint8 id, huint32 slope, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x6091-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param rev 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getGearRatioMotorRevolutions(huint8 devIndex, huint8 id, huint32 *rev, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x6091-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param rev 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setGearRatioMotorRevolutions(huint8 devIndex, huint8 id, huint32 rev, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x6091-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param rev 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getGearRatioShaftRevolutions(huint8 devIndex, huint8 id, huint32 *rev, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x6091-0x2
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param rev 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setGearRatioShaftRevolutions(huint8 devIndex, huint8 id, huint32 rev, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x60C1-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param value 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getInterpolationDataRecord(huint8 devIndex, huint8 id, hint32 *value, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x60C1-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param value 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setInterpolationDataRecord(huint8 devIndex, huint8 id, hint32 value, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x60C2-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param value 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getInterpolationTimePeriodValue(huint8 devIndex, huint8 id, huint8 *value, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x60C2-0x1
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param value 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setInterpolationTimePeriodValue(huint8 devIndex, huint8 id, huint8 value, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x60F4-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param error 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getActualFollowingError(huint8 devIndex, huint8 id, hint32 *error, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x60F4-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param error 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setActualFollowingError(huint8 devIndex, huint8 id, hint32 error, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x60FF-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param targetVel 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getTargetVelocity(huint8 devIndex, huint8 id, hint32 *targetVel, huint32 timeout = 100);

    /**
     * @brief 写从站字典0x60FF-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param targetVel 写入值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_setTargetVelocity(huint8 devIndex, huint8 id, hint32 targetVel, huint32 timeout = 100);

    /**
     * @brief 读从站字典0x6502-0x0
     *
     * @param devIndex usb转can设备索引，第1个插入的设备为0，第2个为1，以此类推
     * @param id 从节点id
     * @param modes 存放读取的值
     * @param timeout 超时等待时间，单位ms
     * @return 成功返回0，失败返回其他
     */
    EXTERNFUNC int harmonic_getSupportedDriveModes(huint8 devIndex, huint8 id, huint32 *modes, huint32 timeout = 100);

#ifdef __cplusplus
}
#endif

#endif // EU_HARMONIC_H
