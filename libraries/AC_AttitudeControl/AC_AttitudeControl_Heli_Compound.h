// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file    AC_AttitudeControl_Heli_Compound.h
/// @brief   Attitude control library for compound helicopters

#ifndef AC_ATTITUDECONTROL_HELI_COMPOUND_H
#define AC_ATTITUDECONTROL_HELI_COMPOUND_H

#include "AC_AttitudeControl_Heli.h"

#define AC_ATTITUDE_COMPOUND_FORWARD_PITCH_ANGLE_DEFAULT    1000.0f

class AC_AttitudeControl_Heli_Compound : public AC_AttitudeControl_Heli {
public:
    AC_AttitudeControl_Heli_Compound(AP_AHRS &ahrs,
                                     const AP_Vehicle::MultiCopter &aparm,
                                     AP_MotorsHeli& motors,
                                     float dt) :
        AC_AttitudeControl_Heli(ahrs, aparm, motors, dt)
		{
            AP_Param::setup_object_defaults(this, var_info);
		}
    //
    // methods to be called by upper controllers to request and implement a desired attitude
    //

    // Command an euler roll and pitch angle and an euler yaw rate with angular velocity feedforward and smoothing
    virtual void input_euler_angle_roll_pitch_euler_rate_yaw_smooth(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds, float smoothing_gain);

    // Command an euler roll and pitch angle and an euler yaw rate
    virtual void input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds);

    // Command an euler roll, pitch and yaw angle
    virtual void input_euler_angle_roll_pitch_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw);

    //
    // rate_controller_run - run lowest level body-frame rate controller and send outputs to the motors
    //      should be called at 100hz or more
    //
    virtual void rate_controller_run();

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:

    // calculate_boost_and_scale_pitch
    float calculate_boost_and_scale_pitch (float pitch_angle);

    // parameters

    AP_Float    _forward_pitch_angle;

    // internal variables

    int16_t     _boost;

};

#endif //AC_ATTITUDECONTROL_HELI_COMPOUND_H
