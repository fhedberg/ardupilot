// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "AC_AttitudeControl_Heli_Compound.h"
#include <AP_HAL/AP_HAL.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl_Heli_Compound::var_info[] = {
    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(AC_AttitudeControl_Heli, 0),

    // @Param: FWD_PIT
    // @DisplayName: Forward pitch angle
    // @Description: The forward pitch angle where the booster will take over for fast forward flight.
    // @Range: 0 4500
    // @Units: Centi-degrees
    // @User: Advanced
    AP_GROUPINFO("FWD_PIT", 0, AC_AttitudeControl_Heli_Compound, _forward_pitch_angle, AC_ATTITUDE_COMPOUND_FORWARD_PITCH_ANGLE_DEFAULT),

    AP_GROUPEND
};

void AC_AttitudeControl_Heli_Compound::rate_controller_run()
{
	AC_AttitudeControl_Heli::rate_controller_run();

    ((AP_MotorsHeli_Compound&)_motors).set_boost(_boost);
}

void AC_AttitudeControl_Heli_Compound::input_euler_angle_roll_pitch_euler_rate_yaw_smooth(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds, float smoothing_gain)
{
	float pitch_angle_ef_scaled = calculate_boost_and_scale_pitch (euler_pitch_angle_cd);

	AC_AttitudeControl_Heli::input_euler_angle_roll_pitch_euler_rate_yaw_smooth(euler_roll_angle_cd, pitch_angle_ef_scaled, euler_yaw_rate_cds, smoothing_gain);
}

void AC_AttitudeControl_Heli_Compound::input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds)
{
	float pitch_angle_ef_scaled = calculate_boost_and_scale_pitch (euler_pitch_angle_cd);

	AC_AttitudeControl_Heli::input_euler_angle_roll_pitch_euler_rate_yaw(euler_roll_angle_cd, euler_pitch_angle_cd, euler_yaw_rate_cds);
}

void AC_AttitudeControl_Heli_Compound::input_euler_angle_roll_pitch_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw)
{
	float pitch_angle_ef_scaled = calculate_boost_and_scale_pitch (euler_pitch_angle_cd);

	AC_AttitudeControl_Heli::input_euler_angle_roll_pitch_yaw(euler_roll_angle_cd, pitch_angle_ef_scaled, euler_yaw_angle_cd, slew_yaw);
}

float AC_AttitudeControl_Heli_Compound::calculate_boost_and_scale_pitch(float pitch_angle)
{
	float pitch_angle_out;

	if (pitch_angle < -_forward_pitch_angle)
	{
		_boost = abs(1000 * (pitch_angle + _forward_pitch_angle)/_aparm.angle_max);
		pitch_angle_out = -_forward_pitch_angle;
	}
	else
	{
		_boost = 0;
		pitch_angle_out = pitch_angle;
	}

	return pitch_angle_out;
}