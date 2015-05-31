// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU   General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY warrantyNTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_MotorsHeli_Compound.h"

extern const AP_HAL::HAL& hal;

// init_outputs
void AP_MotorsHeli_Compound::init_outputs ()
{
    AP_MotorsHeli_Single::init_outputs();

    _servo_yaw_2.set_angle(4500);
}

// set_boost
void AP_MotorsHeli_Compound::set_boost(float boost_in)
{
   _boost_in = boost_in;
}

// move_yaw
void AP_MotorsHeli_Compound::move_yaw(float yaw_in)
{
    // constrain yaw and update limits
    int16_t yaw_out = constrain_int16(yaw_in, -0.45f, 0.45f);

    if (yaw_in != yaw_out) {
        limit.yaw = true;
    }

    if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_SERVO || _tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_VARPITCH) {
        float boost_available = 0.45f - abs(yaw_out);
        float boost_out = _boost_in * boost_available;

        float _servo_yaw_1_out = boost_out + yaw_out;
        float _servo_yaw_2_out = boost_out - yaw_out;

        rc_write(AP_MOTORS_MOT_4, calc_pwm_output_0to1(_servo_yaw_1_out, _servo_yaw_1));
        rc_write(AP_MOTORS_MOT_5, calc_pwm_output_0to1(_servo_yaw_2_out, _servo_yaw_2));
    } else if (_tail_type == AP_MOTORS_HELI_SINGLE_TAILTYPE_DIRECTDRIVE_FIXEDPITCH) {
        float boost_available = 0.45f - MAX(yaw_out, 0.0f);
        float boost_out = _boost_in * boost_available;

        float _servo_yaw_1_out = boost_out + yaw_out;
        float _servo_yaw_2_out = boost_out;

        rc_write(AP_MOTORS_MOT_4, calc_pwm_output_0to1(_servo_yaw_1_out, _servo_yaw_1));
        rc_write(AP_MOTORS_MOT_5, calc_pwm_output_0to1(_servo_yaw_2_out, _servo_yaw_2));
    }
}
