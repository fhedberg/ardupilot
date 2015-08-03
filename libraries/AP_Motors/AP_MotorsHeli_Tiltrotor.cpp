// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_MotorsHeli_Tiltrotor.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHeli_Tiltrotor::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsHeli_Dual, 0),

    // @Param: TILT_EFFECT
    // @DisplayName: Tilt effect
    // @Description: Feed forward compensation of tilt effect coupling yaw and roll
    // @Range: 0 1
    // @User: Standard
    AP_GROUPINFO("TILT_EFFECT", 1, AP_MotorsHeli_Tiltrotor, _tilt_effect, AP_MOTORS_HELI_TILTROTOR_TILT_EFFECT),

    // @Param: TILT_MODE
    // @DisplayName: Tilt Mode
    // @Description: Sets the tilt mode of the heli, either as TVEC or as AERO.
    // @Values: 0:Aero, 1:TVEC
    // @User: Standard
    AP_GROUPINFO("TILT_MODE", 2, AP_MotorsHeli_Tiltrotor, _tilt_mode, AP_MOTORS_HELI_TILTROTOR_MODE_TVEC),

    // @Group: TSV_
    // @Path: ../RC_Channel/RC_Channel.cpp
    AP_SUBGROUPINFO(_tilt_servo, "TSV_", 3, AP_MotorsHeli_Tiltrotor, RC_Channel),

    AP_GROUPEND
};

// enable - starts allowing signals to be sent to motors
void AP_MotorsHeli_Tiltrotor::enable()
{
    AP_MotorsHeli_Dual::enable();

    rc_enable_ch(CH_7);
}

// set_tilt_angle
void AP_MotorsHeli_Tiltrotor::set_tilt_angle (float tilt_angle)
{
    _tilt_angle = constrain_float(tilt_angle, 0.0f, 90.0f);

    calculate_roll_pitch_collective_factors();
}

// calculate_swash_factors - calculate factors based on swash type and servo position
void AP_MotorsHeli_Tiltrotor::calculate_roll_pitch_collective_factors()
{
    if (_tilt_mode == AP_MOTORS_HELI_TILTROTOR_MODE_TVEC) {
        // roll factors
        _rollFactor[CH_1] = _dcp_scaler * sinf(radians(_tilt_angle)) + cosf(radians(_servo1_pos - _swash1_phase_angle)) * cosf(radians(_tilt_angle));
        _rollFactor[CH_2] = _dcp_scaler * sinf(radians(_tilt_angle)) + cosf(radians(_servo2_pos - _swash1_phase_angle)) * cosf(radians(_tilt_angle));
        _rollFactor[CH_3] = _dcp_scaler * sinf(radians(_tilt_angle)) + cosf(radians(_servo3_pos - _swash1_phase_angle)) * cosf(radians(_tilt_angle));

        _rollFactor[CH_4] = -_dcp_scaler * sinf(radians(_tilt_angle)) + cosf(radians(_servo4_pos + 180 - _swash2_phase_angle)) * cosf(radians(_tilt_angle));
        _rollFactor[CH_5] = -_dcp_scaler * sinf(radians(_tilt_angle)) + cosf(radians(_servo5_pos + 180 - _swash2_phase_angle)) * cosf(radians(_tilt_angle));
        _rollFactor[CH_6] = -_dcp_scaler * sinf(radians(_tilt_angle)) + cosf(radians(_servo6_pos + 180 - _swash2_phase_angle)) * cosf(radians(_tilt_angle));

        // pitch factors
        _pitchFactor[CH_1] = cosf(radians(_servo1_pos - _swash1_phase_angle));
        _pitchFactor[CH_2] = cosf(radians(_servo2_pos - _swash1_phase_angle));
        _pitchFactor[CH_3] = cosf(radians(_servo3_pos - _swash1_phase_angle));

        _pitchFactor[CH_4] = cosf(radians(_servo4_pos - _swash2_phase_angle));
        _pitchFactor[CH_5] = cosf(radians(_servo5_pos - _swash2_phase_angle));
        _pitchFactor[CH_6] = cosf(radians(_servo6_pos - _swash2_phase_angle));

        // yaw factors
        _yawFactor[CH_1] = cosf(radians(_servo1_pos + 180 - _swash1_phase_angle)) * sinf(radians(_tilt_angle)) + _dcp_scaler * cosf(radians(_tilt_angle));
        _yawFactor[CH_2] = cosf(radians(_servo2_pos + 180 - _swash1_phase_angle)) * sinf(radians(_tilt_angle)) + _dcp_scaler * cosf(radians(_tilt_angle));
        _yawFactor[CH_3] = cosf(radians(_servo3_pos + 180 - _swash1_phase_angle)) * sinf(radians(_tilt_angle)) + _dcp_scaler * cosf(radians(_tilt_angle));

        _yawFactor[CH_4] = cosf(radians(_servo4_pos - _swash2_phase_angle)) * sinf(radians(_tilt_angle)) - _dcp_scaler * cosf(radians(_tilt_angle));
        _yawFactor[CH_5] = cosf(radians(_servo5_pos - _swash2_phase_angle)) * sinf(radians(_tilt_angle)) - _dcp_scaler * cosf(radians(_tilt_angle));
        _yawFactor[CH_6] = cosf(radians(_servo6_pos - _swash2_phase_angle)) * sinf(radians(_tilt_angle)) - _dcp_scaler * cosf(radians(_tilt_angle));
    } else { // AP_MOTORS_HELI_TILTROTOR_MODE_AERO
          // roll factors
        _rollFactor[CH_1] = _dcp_scaler * sinf(radians(_tilt_angle));
        _rollFactor[CH_2] = _dcp_scaler * sinf(radians(_tilt_angle));
        _rollFactor[CH_3] = _dcp_scaler * sinf(radians(_tilt_angle));

        _rollFactor[CH_4] = -_dcp_scaler * sinf(radians(_tilt_angle));
        _rollFactor[CH_5] = -_dcp_scaler * sinf(radians(_tilt_angle));
        _rollFactor[CH_6] = -_dcp_scaler * sinf(radians(_tilt_angle));

        // pitch factors
        _pitchFactor[CH_1] = cosf(radians(_servo1_pos - _swash1_phase_angle)) * sinf(radians(_tilt_angle));
        _pitchFactor[CH_2] = cosf(radians(_servo2_pos - _swash1_phase_angle)) * sinf(radians(_tilt_angle));
        _pitchFactor[CH_3] = cosf(radians(_servo3_pos - _swash1_phase_angle)) * sinf(radians(_tilt_angle));

        _pitchFactor[CH_4] = cosf(radians(_servo4_pos - _swash2_phase_angle)) * sinf(radians(_tilt_angle));
        _pitchFactor[CH_5] = cosf(radians(_servo5_pos - _swash2_phase_angle)) * sinf(radians(_tilt_angle));
        _pitchFactor[CH_6] = cosf(radians(_servo6_pos - _swash2_phase_angle)) * sinf(radians(_tilt_angle));

        // yaw factors
        _yawFactor[CH_1] = cosf(radians(_servo1_pos + 180 - _swash1_phase_angle)) * sinf(radians(_tilt_angle));
        _yawFactor[CH_2] = cosf(radians(_servo2_pos + 180 - _swash1_phase_angle)) * sinf(radians(_tilt_angle));
        _yawFactor[CH_3] = cosf(radians(_servo3_pos + 180 - _swash1_phase_angle)) * sinf(radians(_tilt_angle));

        _yawFactor[CH_4] = cosf(radians(_servo4_pos - _swash2_phase_angle)) * sinf(radians(_tilt_angle));
        _yawFactor[CH_5] = cosf(radians(_servo5_pos - _swash2_phase_angle)) * sinf(radians(_tilt_angle));
        _yawFactor[CH_6] = cosf(radians(_servo6_pos - _swash2_phase_angle)) * sinf(radians(_tilt_angle));
    }

    // collective factors
    _collectiveFactor[CH_1] = 1;
    _collectiveFactor[CH_2] = 1;
    _collectiveFactor[CH_3] = 1;

    _collectiveFactor[CH_4] = 1;
    _collectiveFactor[CH_5] = 1;
    _collectiveFactor[CH_6] = 1;
}

// heli_move_swash - moves swash plate to attitude of parameters passed in
void AP_MotorsHeli_Tiltrotor::move_actuators(float roll_out, float pitch_out, float coll_in, float yaw_out)
{
    AP_MotorsHeli_Dual::move_actuators(roll_out, pitch_out, coll_in, yaw_out);

    float tilt_servo_out = _tilt_angle / 90.0f;

    rc_write(AP_MOTORS_HELI_TILTROTOR_TILT_SERVO, calc_pwm_output_0to1(tilt_servo_out, _tilt_servo));
}
