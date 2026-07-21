#pragma once

#include <stdint.h>
#include <pthread.h>


namespace arx
{
    namespace hw_interface
    {
        struct CanFrame
        {
            uint32_t can_id;
            uint8_t can_dlc;
            uint8_t __pad;
            uint8_t __res0;
            uint8_t __res1;
            uint8_t data[8] __attribute__((aligned(8)));
        };

        /// @brief 定义了基本的Can数据类型和信息交换函数
        class CanBase
        {
        public:
            CanFrame frame;

            virtual bool Init(const char *interface) = 0;
            virtual bool ExchangeData(CanFrame *frame) = 0;
            virtual bool WriteData() = 0;
            virtual bool ReadData() = 0;
            virtual bool IsOpen() = 0;
        };
    }
}