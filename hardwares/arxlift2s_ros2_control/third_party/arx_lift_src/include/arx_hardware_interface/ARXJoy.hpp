//
// Created by yezi on 2025/5/4.
//

#include "arx_hardware_interface/canbase/CanBaseDef.hpp"

namespace arx {
namespace hw_interface {
class ARXJoy {
public:
  void read(CanFrame *frame);
  void getValue(int *button);
  void update();

private:
  int button_[8]{0, 0, 0, 0, 0, 0, 0, 0};
  int button_buffer_[8]{0, 0, 0, 0, 0, 0, 0, 0};
};
} // namespace hw_interface
} // namespace arx