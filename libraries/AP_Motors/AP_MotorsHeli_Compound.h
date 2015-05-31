// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file   AP_MotorsHeli_Compound.h
/// @brief  Motor control class for compound helicopters
/// @author Fredrik Hedberg

#ifndef __AP_MOTORS_HELI_COMPOUND_H__
#define __AP_MOTORS_HELI_COMPOUND_H__

#include "AP_MotorsHeli_Single.h"

/// @class AP_MotorsHeli_Compound
class AP_MotorsHeli_Compound : public AP_MotorsHeli_Single {
public:
    // constructor
    AP_MotorsHeli_Compound(uint16_t         loop_rate,
                           uint16_t         speed_hz = AP_MOTORS_HELI_SPEED_DEFAULT) :
        AP_MotorsHeli_Single(loop_rate, speed_hz),
        _servo_yaw_1(CH_4),
        _servo_yaw_2(CH_5)
    {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // set_boost - engage the booster
    void set_boost(float boost_in);

protected:

    // init_servos
    void init_outputs();

    // move_yaw
    void move_yaw(float yaw_out);

private:

    float _boost_in;
    RC_Channel _servo_yaw_1;
    RC_Channel _servo_yaw_2;
};

#endif  // __AP_MOTORS_HELI_COMPOUND_H__
