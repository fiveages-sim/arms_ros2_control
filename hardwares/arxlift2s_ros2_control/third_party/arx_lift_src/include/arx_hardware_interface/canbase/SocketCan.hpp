#pragma once

#include "arx_hardware_interface/canbase/CanBaseDef.hpp"

// #include "CANAdapter.h"
#include <iostream>
#include <net/if.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>

#include <linux/can.h>

#include <sys/select.h>
#include <sys/socket.h>
// SIOCGIFINDEX
#include <sys/ioctl.h>

#include <functional>
#include <memory>
#include <mutex>
#include <string.h>

#include <atomic>

namespace arx {
namespace hw_interface {
/// @brief 继承CanBase并使用SocketCan进行数据通信
class SocketCan : public CanBase {
public:
  SocketCan();
  ~SocketCan();

  bool Init(const char *interface) override;
  bool ExchangeData(CanFrame *frame) override;
  bool WriteData();
  bool ReadData();
  bool IsOpen();
  bool checkOnline();
  void closeReset();

  void setCallBackFunction(const std::function<void(CanFrame *frame)> func);
  void setGetMsgContentFunction(const std::function<void()> func);

  void LoadMutexMsg();

  /// @brief ----
  void Close();

private:
  class impl;
  std::unique_ptr<impl> pimpl;
};
} // namespace hw_interface
} // namespace arx