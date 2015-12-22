// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file   AP_MotorsHeli_Dual.h
/// @brief  Motor control class for dual heli (tandem or transverse)
/// @author Fredrik Hedberg

#ifndef __AP_MOTORS_HELI_DUAL_H__
#define __AP_MOTORS_HELI_DUAL_H__

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>

#include "AP_MotorsHeli.h"
#include "AP_MotorsHeli_RSC.h"

// servo position defaults
#define AP_MOTORS_HELI_DUAL_SERVO1_POS               -60
#define AP_MOTORS_HELI_DUAL_SERVO2_POS                60
#define AP_MOTORS_HELI_DUAL_SERVO3_POS               180
#define AP_MOTORS_HELI_DUAL_SERVO4_POS               -60
#define AP_MOTORS_HELI_DUAL_SERVO5_POS                60
#define AP_MOTORS_HELI_DUAL_SERVO6_POS               180

// rsc function output channel
#define AP_MOTORS_HELI_DUAL_RSC                      CH_8

// tandem modes
#define AP_MOTORS_HELI_DUAL_MODE_TANDEM              0       // tandem mode (rotors front and aft)
#define AP_MOTORS_HELI_DUAL_MODE_TRANSVERSE          1       // transverse mode (rotors side by side)

// default differential-collective-pitch scaler
#define AP_MOTORS_HELI_DUAL_DCP_SCALER               0.25f

/// @class AP_MotorsHeli_Dual
class AP_MotorsHeli_Dual : public AP_MotorsHeli {
public:
    // constructor
    AP_MotorsHeli_Dual(RC_Channel&  servo_rsc,
                       RC_Channel&  swash_servo_1,
                       RC_Channel&  swash_servo_2,
                       RC_Channel&  swash_servo_3,
                       RC_Channel&  swash_servo_4,
                       RC_Channel&  swash_servo_5,
                       RC_Channel&  swash_servo_6,
                       uint16_t     loop_rate,
                       uint16_t     speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_MotorsHeli(loop_rate, speed_hz),
        _swash_servo_1(swash_servo_1),
        _swash_servo_2(swash_servo_2),
        _swash_servo_3(swash_servo_3),
        _swash_servo_4(swash_servo_4),
        _swash_servo_5(swash_servo_5),
        _swash_servo_6(swash_servo_6),
        _rotor(servo_rsc, AP_MOTORS_HELI_DUAL_RSC, loop_rate)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // set_update_rate - set update rate to motors 
    void set_update_rate( uint16_t speed_hz );

    // enable - starts allowing signals to be sent to motors
    void enable();

    // output_test - spin a motor at the pwm value specified
    void output_test(uint8_t motor_seq, int16_t pwm);
  
    // set_desired_rotor_speed - sets target rotor speed as a number from 0 ~ 1000
    void set_desired_rotor_speed(int16_t desired_speed);

    // get_estimated_rotor_speed - gets estimated rotor speed as a number from 0 ~ 1000
    int16_t get_main_rotor_speed() const { return _rotor.get_rotor_speed(); }

    // get_desired_rotor_speed - gets target rotor speed as a number from 0 ~ 1000
    int16_t get_desired_rotor_speed() const { return _rotor.get_rotor_speed(); }

    // rotor_speed_above_critical - return true if rotor speed is above that critical for flight
    bool rotor_speed_above_critical() const { return _rotor.get_rotor_speed() > _rotor.get_critical_speed(); }

    // calculate_scalars - recalculates various scalars used
    void calculate_scalars();

    // calculate_armed_scalars - recalculates scalars that can change while armed
    void calculate_armed_scalars();

    // get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
    uint16_t get_motor_mask();

    // has_flybar - returns true if we have a mechical flybar
    bool has_flybar() const { return AP_MOTORS_HELI_NOFLYBAR; }

    // supports_yaw_passthrought - returns true if we support yaw passthrough
    bool supports_yaw_passthrough() const { return false; }

    // set_delta_phase_angle for setting variable phase angle compensation and force
    // recalculation of collective factors
    void set_delta_phase_angle(int16_t angle);

    // servo_test - move servos through full range of movement
    void servo_test();
    
    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // init_outputs
    void init_outputs ();

    // update_motor_controls - sends commands to motor controllers
    void update_motor_control(RotorControlState state);

    // calculate_roll_pitch_collective_factors - calculate factors based on swash type and servo position
    void calculate_roll_pitch_collective_factors ();

    // move_actuators - moves swash plate to attitude of parameters passed in
    void move_actuators(int16_t roll_out, int16_t pitch_out, int16_t coll_in, int16_t yaw_out);

private:

    //  objects we depend upon
    RC_Channel&                 _swash_servo_1;     // swash plate servo #1
    RC_Channel&                 _swash_servo_2;     // swash plate servo #2
    RC_Channel&                 _swash_servo_3;     // swash plate servo #3
    RC_Channel&                 _swash_servo_4;     // swash plate servo #4
    RC_Channel&                 _swash_servo_5;     // swash plate servo #5
    RC_Channel&                 _swash_servo_6;     // swash plate servo #6
    AP_MotorsHeli_RSC           _rotor;             // main rotor controller

    // internal variables
    float _oscillate_angle = 0.0f;                  // cyclic oscillation angle, used by servo_test function
    float _servo_test_cycle_time = 0.0f;            // cycle time tracker, used by servo_test function
    float _collective_test = 0.0f;                  // over-ride for collective output, used by servo_test function
    float _roll_test = 0.0f;                        // over-ride for roll output, used by servo_test function
    float _pitch_test = 0.0f;                       // over-ride for pitch output, used by servo_test function

    // parameters
    AP_Int16        _servo1_pos;                    // angular location of swash servo #1
    AP_Int16        _servo2_pos;                    // angular location of swash servo #2
    AP_Int16        _servo3_pos;                    // angular location of swash servo #3
    AP_Int16        _servo4_pos;                    // angular location of swash servo #4
    AP_Int16        _servo5_pos;                    // angular location of swash servo #5
    AP_Int16        _servo6_pos;                    // angular location of swash servo #6
    AP_Int16        _swash1_phase_angle;            // phase angle correction for 1st swash.
    AP_Int16        _swash2_phase_angle;            // phase angle correction for 2nd swash.
    AP_Int8         _dual_mode;                     // which dual mode the heli is
    AP_Float        _dcp_scaler;                    // scaling factor applied to the differential-collective-pitch
    AP_Float        _dcp_yaw_effect;                // feed-forward compensation to automatically add yaw input when differential collective pitch is applied.

    // internal variables
    float           _yawFactor[AP_MOTORS_HELI_NUM_SWASHPLATE_SERVOS];
    float           _yaw_scaler;                    // scaler to convert pitch input from radio (i.e. -4500 ~ 4500) to max pitch range
};

#endif  // AP_MotorsHeli_Dual
