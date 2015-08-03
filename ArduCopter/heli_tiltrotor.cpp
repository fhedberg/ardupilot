/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#if FRAME_CONFIG == HELI_DUAL_FRAME

void Copter::heli_update_tiltrotor_tvec_angle ()
{
	float tvec_angle_max = motors.get_tvec_angle_max();
    float tvec_angle_min = motors.get_tvec_angle_min();

    float desired_angle = g.rc_7.percent_input()/100.0f * (tvec_angle_max - tvec_angle_min) + tvec_angle_min;

	motors.set_tvec_angle(desired_angle);
}

#endif