// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file   AP_MotorsHeli_Tiltrotor.h
/// @brief  Motor control class for swash based tiltrotors
/// @author Fredrik Hedberg

#ifndef __AP_MOTORS_HELI_TILTROTOR_H__
#define __AP_MOTORS_HELI_TILTROTOR_H__

#include <inttypes.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <RC_Channel.h>

#include "AP_MotorsHeli_Dual.h"

// limits
#define AP_MOTORS_HELI_TILTROTOR_TVEC_ANGLE_MIN            0.0f
#define AP_MOTORS_HELI_TILTROTOR_TVEC_ANGLE_MAX            90.0f

// tiltrotor modes
#define AP_MOTORS_HELI_TILTROTOR_MODE_AERO                 0        // uses aerodynamic controlsurfaces when in airplane mode
#define AP_MOTORS_HELI_TILTROTOR_MODE_TVEC                 1        // uses trust vectoring when in airplane mode

// tvec servo channel 
#define AP_MOTORS_HELI_TILTROTOR_TVEC                      CH_7

// default tilt effect
#define AP_MOTORS_HELI_TILTROTOR_TILT_EFFECT               0.5f

/// @class AP_MotorsHeli_Tiltrotor
class AP_MotorsHeli_Tiltrotor : public AP_MotorsHeli_Dual {
public:
    // constructor
    AP_MotorsHeli_Tiltrotor(RC_Channel&  servo_rsc,
                            RC_Channel&  swash_servo_1,
                            RC_Channel&  swash_servo_2,
                            RC_Channel&  swash_servo_3,
                            RC_Channel&  swash_servo_4,
                            RC_Channel&  swash_servo_5,
                            RC_Channel&  swash_servo_6,
                            RC_Channel&  tvec_servo,
                            uint16_t     loop_rate,
                            uint16_t     speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_MotorsHeli_Dual(servo_rsc, swash_servo_1, swash_servo_2, swash_servo_3, swash_servo_4, swash_servo_5, swash_servo_6, loop_rate, speed_hz),
        _tvec_servo(tvec_servo)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // Init
    void Init();

    // set_tvec_angle - sets the trust vector angle, between 0 degrees (plane) and 90 degrees (helicopter)
    void set_tvec_angle (float tvec_angle);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

    // enable - starts allowing signals to be sent to motors
    void enable();

    // get_tvec_max_cd
    float get_tvec_angle_max () const { return _tvec_angle_max; }

    // get_tvec_min_cd
    float get_tvec_angle_min () const { return _tvec_angle_min; }

protected:

    // init_servos
    void init_servos ();
    
    // calculate_roll_pitch_collective_factors - calculate factors based on swash type and servo position
    void calculate_roll_pitch_collective_factors ();

    // heli_move_swash - moves swash plate to attitude of parameters passed in
    void move_swash(int16_t roll_out, int16_t pitch_out, int16_t coll_in, int16_t yaw_out);

private:

    // parameters
    AP_Float            _tvec_angle_max;
    AP_Float            _tvec_angle_min;
    AP_Float            _tilt_effect;
    AP_Int16            _tilt_mode;

    // internal variables
    RC_Channel&         _tvec_servo;
    float               _tvec_angle;

};

#endif // __AP_MOTORS_HELI_TILTROTOR_H__