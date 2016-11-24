#pragma once

#include "AP_HAL_Linux.h"

namespace Linux {

class CAN_SocketCAN : public AP_HAL::CAN {
  public:
    void init();
    void write();
  private:
    int _socket;
    int read_can_port;
};

}
