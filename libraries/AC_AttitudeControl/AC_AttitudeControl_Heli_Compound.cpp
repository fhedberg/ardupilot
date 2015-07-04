// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "AC_AttitudeControl_Heli_Compound.h"
#include <AP_HAL.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_AttitudeControl_Heli_Compound::var_info[] PROGMEM = {
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

void AC_AttitudeControl_Heli_Compound::angle_ef_roll_pitch_rate_ef_yaw_smooth(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef, float smoothing_gain)
{
	float pitch_angle_ef_scaled = calculate_boost_and_scale_pitch (pitch_angle_ef);

	AC_AttitudeControl_Heli::angle_ef_roll_pitch_rate_ef_yaw_smooth(roll_angle_ef, pitch_angle_ef_scaled, yaw_rate_ef, smoothing_gain);
}

void AC_AttitudeControl_Heli_Compound::angle_ef_roll_pitch_rate_ef_yaw(float roll_angle_ef, float pitch_angle_ef, float yaw_rate_ef)
{
	float pitch_angle_ef_scaled = calculate_boost_and_scale_pitch (pitch_angle_ef);

	AC_AttitudeControl_Heli::angle_ef_roll_pitch_rate_ef_yaw(roll_angle_ef, pitch_angle_ef_scaled, yaw_rate_ef);
}

void AC_AttitudeControl_Heli_Compound::angle_ef_roll_pitch_yaw(float roll_angle_ef, float pitch_angle_ef, float yaw_angle_ef, bool slew_yaw)
{
	float pitch_angle_ef_scaled = calculate_boost_and_scale_pitch (pitch_angle_ef);

	AC_AttitudeControl_Heli::angle_ef_roll_pitch_yaw(roll_angle_ef, pitch_angle_ef_scaled, yaw_angle_ef, slew_yaw);
}

void AC_AttitudeControl_Heli_Compound::rate_ef_roll_pitch_yaw(float roll_rate_ef, float pitch_rate_ef, float yaw_rate_ef)
{
	// Don't engage forward thrust when in rate mode.
	calculate_boost_and_scale_pitch(0);

	AC_AttitudeControl_Heli::rate_ef_roll_pitch_yaw(roll_rate_ef, pitch_rate_ef, yaw_rate_ef);
}

void AC_AttitudeControl_Heli_Compound::rate_bf_roll_pitch_yaw(float roll_rate_bf, float pitch_rate_bf, float yaw_rate_bf)
{
	// Don't engage forward thrust when in rate mode.
	calculate_boost_and_scale_pitch(0);

	AC_AttitudeControl_Heli::rate_bf_roll_pitch_yaw(roll_rate_bf, pitch_rate_bf, yaw_rate_bf);
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