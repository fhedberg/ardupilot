// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file    AC_AttitudeControl_Heli_Compound.h
/// @brief   Attitude control library for compound helicopters

#ifndef AC_ATTITUDECONTROL_HELI_COMPOUND_H
#define AC_ATTITUDECONTROL_HELI_COMPOUND_H

#include <AC_AttitudeControl_Heli.h>
#include <AP_ControlSurfaces.h>
#include <AC_HELI_PID.h>
#include <Filter.h>

#define AC_ATTITUDE_COMPOUND_FORWARD_PITCH_ANGLE_DEFAULT    1000.0f // earth-frame rate stabilize controller's maximum overshoot angle

class AC_AttitudeControl_Heli_Compound : public AC_AttitudeControl_Heli {
public:
    AC_AttitudeControl_Heli_Compound(AP_AHRS &ahrs,
                                     const AP_Vehicle::MultiCopter &aparm,
                                     AP_MotorsHeli& motors,
                                     AC_P& p_angle_roll, AC_P& p_angle_pitch, AC_P& p_angle_yaw,
                                     AC_HELI_PID& pid_rate_roll, AC_HELI_PID& pid_rate_pitch, AC_HELI_PID& pid_rate_yaw,
                                     AP_ControlSurfaces &control_surfaces) :
        AC_AttitudeControl_Heli(ahrs, aparm, motors,
                                p_angle_roll, p_angle_pitch, p_angle_yaw,
                                pid_rate_roll, pid_rate_pitch, pid_rate_yaw),
        _control_surfaces(control_surfaces)
		{
            AP_Param::setup_object_defaults(this, var_info);
		}
    //
    // methods to be called by upper controllers to request and implement a desired attitude
    //

    // angle_ef_roll_pitch_rate_ef_yaw_smooth - attempts to maintain a roll and pitch angle and yaw rate (all earth frame) while smoothing the attitude based on the feel parameter
    //      smoothing_gain : a number from 1 to 50 with 1 being sluggish and 50 being very crisp
    virtual void angle_ef_roll_pitch_rate_ef_yaw_smooth(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef, float smoothing_gain);

    // angle_ef_roll_pitch_rate_ef_yaw - attempts to maintain a roll and pitch angle and yaw rate (all earth frame)
    virtual void angle_ef_roll_pitch_rate_ef_yaw(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef);

    // angle_ef_roll_pitch_yaw - attempts to maintain a roll, pitch and yaw angle (all earth frame)
    //  if yaw_slew is true then target yaw movement will be gradually moved to the new target based on the YAW_SLEW parameter
    virtual void angle_ef_roll_pitch_yaw(float roll_angle_ef, float pitch_angle_ef, float yaw_angle_ef, bool slew_yaw);

    // rate_ef_roll_pitch_yaw - attempts to maintain a roll, pitch and yaw rate (all earth frame)
    virtual void rate_ef_roll_pitch_yaw(float roll_rate_ef, float pitch_rate_ef, float yaw_rate_ef);

    // rate_bf_roll_pitch_yaw - attempts to maintain a roll, pitch and yaw rate (all body frame)
    virtual void rate_bf_roll_pitch_yaw(float roll_rate_bf, float pitch_rate_bf, float yaw_rate_bf);

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

    AP_Float                _forward_pitch_angle;

    // internal variables

    AP_ControlSurfaces      _control_surfaces;
    int16_t                 _boost;

};

#endif //AC_ATTITUDECONTROL_HELI_COMPOUND_H
