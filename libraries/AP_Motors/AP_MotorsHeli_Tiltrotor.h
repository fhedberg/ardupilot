// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file   AP_MotorsHeli_Tiltrotor.h
/// @brief  Motor control class for swash based tiltrotors
/// @author Fredrik Hedberg

#ifndef __AP_MOTORS_HELI_TILTROTOR_H__
#define __AP_MOTORS_HELI_TILTROTOR_H__

#include "AP_MotorsHeli_Dual.h"

// tiltrotor modes
#define AP_MOTORS_HELI_TILTROTOR_MODE_AERO                 0        // uses aerodynamic controlsurfaces when in airplane mode
#define AP_MOTORS_HELI_TILTROTOR_MODE_TVEC                 1        // uses trust vectoring when in airplane mode

// tilt servo channel
#define AP_MOTORS_HELI_TILTROTOR_TILT_SERVO                CH_7

// default tilt effect
#define AP_MOTORS_HELI_TILTROTOR_TILT_EFFECT               0.5f

/// @class AP_MotorsHeli_Tiltrotor
class AP_MotorsHeli_Tiltrotor : public AP_MotorsHeli_Dual {
public:
    // constructor
    AP_MotorsHeli_Tiltrotor(uint16_t     loop_rate,
                            uint16_t     speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_MotorsHeli_Dual(loop_rate, speed_hz),
        _tilt_servo(CH_7)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // set_tilt_angle - sets the trust vector angle, between 0 degrees (plane) and 90 degrees (helicopter)
    void set_tilt_angle (float tilt_angle);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    // enable - starts allowing signals to be sent to motors
    void enable();

protected:

    // calculate_roll_pitch_collective_factors - calculate factors based on swash type and servo position
    void calculate_roll_pitch_collective_factors ();

    // heli_move_swash - moves swash plate to attitude of parameters passed in
    void move_actuators(float roll_out, float pitch_out, float coll_in, float yaw_out);

private:

    // parameters
    AP_Float            _tilt_effect;
    AP_Int16            _tilt_mode;
    RC_Channel          _tilt_servo;

    // internal variables
    float               _tilt_angle;

};

#endif // __AP_MOTORS_HELI_TILTROTOR_H__
