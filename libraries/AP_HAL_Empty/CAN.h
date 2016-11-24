#pragma once

#include "AP_HAL_Empty.h"

class Empty::CAN : public AP_HAL::CAN {
    void init();
};
