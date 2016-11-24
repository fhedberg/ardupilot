#pragma once

#include "AP_HAL_Namespace.h"

class AP_HAL::CAN {
public:
    virtual void init() = 0;
    virtual void write() = 0;
};
