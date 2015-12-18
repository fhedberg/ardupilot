// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>

#include "RPM_I2C.h"

extern const AP_HAL::HAL& hal;

/* 
   open the sensor in constructor
*/
AP_RPM_I2C::AP_RPM_I2C(AP_RPM &ap_rpm, uint8_t instance, AP_RPM::RPM_State &_state) :
	AP_RPM_Backend(ap_rpm, instance, _state)
{
  
}

AP_RPM_I2C::~AP_RPM_I2C()
{

}

void AP_RPM_I2C::update(void)
{
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // exit immediately if we can't take the semaphore
    if (i2c_sem == NULL || !i2c_sem->take(1)) {
        return;
    }

    uint8_t buff[4];

    // read first registers
    if (hal.i2c->read(AP_RPM_I2C_ADDRESS, 0x20, &buff[0]) != 0) {
        i2c_sem->give();
        return;
    }
}