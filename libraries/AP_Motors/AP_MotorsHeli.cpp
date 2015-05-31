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

/*
 *       AP_MotorsHeli.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */
#include <stdlib.h>
#include <AP_HAL.h>
#include "AP_MotorsHeli.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHeli::var_info[] PROGMEM = {
    // variables from parent vehicle
    AP_NESTEDGROUPINFO(AP_Motors, 0),
   
    // @Param: ROL_MAX
    // @DisplayName: Swash Roll Angle Max
    // @Description: Maximum roll angle of the swash plate
    // @Range: 0 18000
    // @Units: Centi-Degrees
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("ROL_MAX", 1,      AP_MotorsHeli,  _roll_max,    AP_MOTORS_HELI_SWASH_ROLL_MAX),

    // @Param: PIT_MAX
    // @DisplayName: Swash Pitch Angle Max
    // @Description: Maximum pitch angle of the swash plate
    // @Range: 0 18000
    // @Units: Centi-Degrees
    // @Increment: 100
    // @User: Advanced
    AP_GROUPINFO("PIT_MAX", 2,      AP_MotorsHeli,  _pitch_max,   AP_MOTORS_HELI_SWASH_PITCH_MAX),

    // @Param: COL_MIN
    // @DisplayName: Collective Pitch Minimum
    // @Description: Lowest possible servo position for the swashplate
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL_MIN", 3,      AP_MotorsHeli,  _collective_min, AP_MOTORS_HELI_COLLECTIVE_MIN),

    // @Param: COL_MAX
    // @DisplayName: Collective Pitch Maximum
    // @Description: Highest possible servo position for the swashplate
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL_MAX", 4,      AP_MotorsHeli,  _collective_max, AP_MOTORS_HELI_COLLECTIVE_MAX),

    // @Param: COL_MID
    // @DisplayName: Collective Pitch Mid-Point
    // @Description: Swash servo position corresponding to zero collective pitch (or zero lift for Assymetrical blades)
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("COL_MID", 5,      AP_MotorsHeli,  _collective_mid, AP_MOTORS_HELI_COLLECTIVE_MID),

    // @Param: SV_MAN
    // @DisplayName: Manual Servo Mode
    // @Description: Pass radio inputs directly to servos for set-up. Do not set this manually!
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("SV_MAN",  6,     AP_MotorsHeli,  _servo_manual,  0),

    // @Param: COLYAW
    // @DisplayName: Collective-Yaw Mixing
    // @Description: Feed-forward compensation to automatically add rudder input when collective pitch is increased. Can be positive or negative depending on mechanics.
    // @Range: -10 10
    // @Increment: 0.1
    AP_GROUPINFO("COLYAW",  7,     AP_MotorsHeli,  _collective_yaw_effect, 0),

    // @Param: GOV_SETPOINT
    // @DisplayName: External Motor Governor Setpoint
    // @Description: PWM passed to the external motor governor when external governor is enabled
    // @Range: 0 1000
    // @Units: PWM
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("RSC_SETPOINT", 8, AP_MotorsHeli, _rsc_setpoint, AP_MOTORS_HELI_RSC_SETPOINT),

    // @Param: RSC_MODE
    // @DisplayName: Rotor Speed Control Mode
    // @Description: Controls the source of the desired rotor speed, either ch8 or RSC_SETPOINT
    // @Values: 0:None, 1:Ch8 Input, 2:SetPoint
    // @User: Standard
    AP_GROUPINFO("RSC_MODE", 9, AP_MotorsHeli,     _rsc_mode, AP_MOTORS_HELI_RSC_MODE_CH8_PASSTHROUGH),

    // @Param: LAND_COL_MIN
    // @DisplayName: Landing Collective Minimum
    // @Description: Minimum collective position while landed or landing
    // @Range: 0 500
    // @Units: pwm
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("LAND_COL_MIN", 10, AP_MotorsHeli, _land_collective_min, AP_MOTORS_HELI_LAND_COLLECTIVE_MIN),

    // @Param: RSC_RAMP_TIME
    // @DisplayName: RSC Ramp Time
    // @Description: Time in seconds for the output to the main rotor's ESC to reach full speed
    // @Range: 0 60
    // @Units: Seconds
    // @User: Standard
    AP_GROUPINFO("RSC_RAMP_TIME", 11, AP_MotorsHeli,_rsc_ramp_time, AP_MOTORS_HELI_RSC_RAMP_TIME),

    // @Param: RSC_RUNUP_TIME
    // @DisplayName: RSC Runup Time
    // @Description: Time in seconds for the main rotor to reach full speed.  Must be longer than RSC_RAMP_TIME
    // @Range: 0 60
    // @Units: Seconds
    // @User: Standard
    AP_GROUPINFO("RSC_RUNUP_TIME", 12, AP_MotorsHeli,_rsc_runup_time, AP_MOTORS_HELI_RSC_RUNUP_TIME),

    // @Param: RSC_CRITICAL
    // @DisplayName: Critical Rotor Speed
    // @Description: Rotor speed below which flight is not possible
    // @Range: 0 1000
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("RSC_CRITICAL", 23, AP_MotorsHeli, _rsc_critical, AP_MOTORS_HELI_RSC_CRITICAL),

    AP_GROUPEND
};

//
// public methods
//

// init
void AP_MotorsHeli::Init()
{
    // set update rate
    set_update_rate(_speed_hz);

    // ensure inputs are not passed through to servos
    _servo_manual = 0;

    // initialise some scalers
    recalc_scalers();

    // initialise swash plate
    init_swash();
}

// output - sends commands to the servos
void AP_MotorsHeli::output()
{
    // update throttle filter
    update_throttle_filter();

    if (_flags.armed) {
        if (!_flags.interlock) {
            output_armed_zero_throttle();
        } else if (_flags.stabilizing) {
            output_armed_stabilizing();
        } else {
            output_armed_not_stabilizing();
        }
    } else {
        output_disarmed();
    }
};

// output_min - sets servos to neutral point
void AP_MotorsHeli::output_min()
{
    // move swash to mid
    move_swash(0, 0, 500, 0);

    // override limits flags
    limit.roll_pitch = true;
    limit.yaw = true;
    limit.throttle_lower = true;
    limit.throttle_upper = false;
}

// return true if the main rotor is up to speed
bool AP_MotorsHeli::rotor_runup_complete() const
{
    // if we have no control of motors, assume pilot has spun them up
    if (_rsc_mode == AP_MOTORS_HELI_RSC_MODE_NONE) {
        return true;
    }

    return _heliflags.rotor_runup_complete;
}

void AP_MotorsHeli::output_armed_not_stabilizing()
{
    // stabilizing servos always operate for helicopters
    output_armed_stabilizing();
}

// sends commands to the motors
void AP_MotorsHeli::output_armed_stabilizing()
{
    // if manual override (i.e. when setting up swash), pass pilot commands straight through to swash
    if (_servo_manual == 1) {
        _roll_control_input = _roll_radio_passthrough;
        _pitch_control_input = _pitch_radio_passthrough;
        _throttle_control_input = _throttle_radio_passthrough;
        _yaw_control_input = _yaw_radio_passthrough;
    }

    move_swash(_roll_control_input, _pitch_control_input, _throttle_control_input, _yaw_control_input);
}

// output_armed_zero_throttle - sends commands to the motors
void AP_MotorsHeli::output_armed_zero_throttle()
{
    // stabilizing servos always operate for helicopters
    // ToDo: Bring RSC Master On/Off into this function
    output_armed_stabilizing();
}

// output_disarmed - sends commands to the motors
void AP_MotorsHeli::output_disarmed()
{
    // stabilizing servos always operate for helicopters
    output_armed_stabilizing();
}

void AP_MotorsHeli::reset_swash_servo (RC_Channel& servo)
{
    servo.radio_min = 1000;
    servo.radio_max = 2000;
}

void AP_MotorsHeli::recalc_scalers()
{
    // range check collective min, max and mid
    if( _collective_min >= _collective_max ) {
        _collective_min = 1000;
        _collective_max = 2000;
    }
    _collective_mid = constrain_int16(_collective_mid, _collective_min, _collective_max);

    // calculate collective mid point as a number from 0 to 1000
    _collective_mid_pwm = ((float)(_collective_mid-_collective_min))/((float)(_collective_max-_collective_min))*1000.0f;

    // determine roll, pitch and collective input scaling
    _roll_scaler = (float)_roll_max/4500.0f;
    _pitch_scaler = (float)_pitch_max/4500.0f;
    _collective_scaler = ((float)(_collective_max-_collective_min))/1000.0f;
    
    // set roll, pitch and throttle scaling
    _roll_scaler = 1.0f;
    _pitch_scaler = 1.0f;
    _collective_scaler = ((float)(_throttle_radio_max - _throttle_radio_min))/1000.0f;
    _collective_scaler_manual = 1.0f;
}

// reset_swash - free up swash for maximum movements. Used for set-up
void AP_MotorsHeli::reset_swash()
{
    // free up servo ranges
    reset_servos();

    // calculate factors based on swash type and servo position
    calculate_swash_factors();

    // we must be in set-up mode so mark swash as uninitialised
    _heliflags.swash_initialised = false;
}

void AP_MotorsHeli::init_swash_servo (RC_Channel& servo)
{
    servo.set_range (0, 1000);
}

// init_swash - initialise the swash plate
void AP_MotorsHeli::init_swash()
{
    // swash servo initialisation
    init_servos();

    // calculate factors based on swash type and servo position
    calculate_swash_factors();

    // servo min/max values
    reset_servos();

        _heliflags.swash_initialised = true;

}

// update the throttle input filter
void AP_MotorsHeli::update_throttle_filter()
{
    _throttle_filter.apply(_throttle_in, 1.0f/_loop_rate);

    // constrain throttle signal to 0-1000
    _throttle_control_input = constrain_float(_throttle_filter.get(),0.0f,1000.0f);
}

// set_radio_passthrough used to pass radio inputs directly to outputs
void AP_MotorsHeli::set_radio_passthrough(int16_t radio_roll_input, int16_t radio_pitch_input, int16_t radio_throttle_input, int16_t radio_yaw_input)
{
    _roll_radio_passthrough = radio_roll_input;
    _pitch_radio_passthrough = radio_pitch_input;
    _throttle_radio_passthrough = radio_throttle_input;
    _yaw_radio_passthrough = radio_yaw_input;
}

// reset_radio_passthrough used to reset all radio inputs to center
void AP_MotorsHeli::reset_radio_passthrough()
{
    _roll_radio_passthrough = 0;
    _pitch_radio_passthrough = 0;
    _throttle_radio_passthrough = 500;
    _yaw_radio_passthrough = 0;
}
