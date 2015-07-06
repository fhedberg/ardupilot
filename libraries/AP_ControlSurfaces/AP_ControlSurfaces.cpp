// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

#include "AP_ControlSurfaces.h"

// table of user settable parameters
const AP_Param::GroupInfo AP_ControlSurfaces::var_info[] PROGMEM = {
  	// @Param: SCALING_SPEED
    // @DisplayName: speed used for speed scaling calculations
    // @Description: Airspeed in m/s to use when calculating surface speed scaling. Note that changing this value will affect all PID values
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("SCALING_SPEED", 0, AP_ControlSurfaces, _scaling_speed, 0),

    // @Param: VTAIL_OUTPUT
    // @DisplayName: VTail output
    // @Description: Enable VTail output in software. If enabled then the APM will provide software VTail mixing on the elevator and rudder channels. There are 4 different mixing modes available, which refer to the 4 ways the elevator can be mapped to the two VTail servos. Note that you must not use VTail output mixing with hardware pass-through of RC values, such as with channel 8 manual control on an APM1. So if you use an APM1 then set FLTMODE_CH to something other than 8 before you enable VTAIL_OUTPUT. Please also see the MIXING_GAIN parameter for the output gain of the mixer.
    // @Values: 0:Disabled,1:UpUp,2:UpDown,3:DownUp,4:DownDown
    // @User: User
    AP_GROUPINFO("VTAIL", 1, AP_ControlSurfaces, _vtail_output, 0),

    // @Param: ELEVON_OUTPUT
    // @DisplayName: Elevon output
    // @Description: Enable software elevon output mixer. If enabled then the APM will provide software elevon mixing on the aileron and elevator channels. There are 4 different mixing modes available, which refer to the 4 ways the elevator can be mapped to the two elevon servos. Note that you must not use elevon output mixing with hardware pass-through of RC values, such as with channel 8 manual control on an APM1. So if you use an APM1 then set FLTMODE_CH to something other than 8 before you enable ELEVON_OUTPUT. Please also see the MIXING_GAIN parameter for the output gain of the mixer.
    // @Values: 0:Disabled,1:UpUp,2:UpDown,3:DownUp,4:DownDown
    // @User: User
    AP_GROUPINFO("ELEVON", 2, AP_ControlSurfaces, _elevon_output, 0),

    // @Param: MIXING_GAIN
    // @DisplayName: Mixing Gain
    // @Description: The gain for the Vtail and elevon output mixers. The default is 0.5, which ensures that the mixer doesn't saturate, allowing both input channels to go to extremes while retaining control over the output. Hardware mixers often have a 1.0 gain, which gives more servo throw, but can saturate. If you don't have enough throw on your servos with VTAIL_OUTPUT or ELEVON_OUTPUT enabled then you can raise the gain using MIXING_GAIN. The mixer allows outputs in the range 900 to 2100 microseconds.
    // @Range: 0.5 1.2
    // @User: User
    AP_GROUPINFO("MIXING", 3, AP_ControlSurfaces, _mixing_gain, 0.5f),

    // @Param: FLAPERON_OUTPUT
    // @DisplayName: Flaperon output
    // @Description: Enable flaperon output in software. If enabled then the APM will provide software flaperon mixing on the FLAPERON1 and FLAPERON2 output channels specified using the FUNCTION on two auxillary channels. There are 4 different mixing modes available, which refer to the 4 ways the flap and aileron outputs can be mapped to the two flaperon servos. Note that you must not use flaperon output mixing with hardware pass-through of RC values, such as with channel 8 manual control on an APM1. So if you use an APM1 then set FLTMODE_CH to something other than 8 before you enable FLAPERON_OUTPUT. Please also see the MIXING_GAIN parameter for the output gain of the mixer. FLAPERON_OUTPUT cannot be combined with ELEVON_OUTPUT or ELEVON_MIXING.
    // @Values: 0:Disabled,1:UpUp,2:UpDown,3:DownUp,4:DownDown
    // @User: User
    AP_GROUPINFO("FLAPERON", 4, AP_ControlSurfaces, _flaperon_output, 0),

    AP_GROUPEND
};

/*
  get a speed scaling number for control surfaces. This is applied to
  PIDs to change the scaling of the PID with speed. At high speed we
  move the surfaces less, and at low speeds we move them more.
 */
float AP_ControlSurfaces::get_speed_scaler()
{
    float aspeed, speed_scaler;

    if (_ahrs.airspeed_estimate(&aspeed)) {
        if (aspeed > 0) {
            speed_scaler = _scaling_speed / aspeed;
        } else {
            speed_scaler = 2.0;
        }
        speed_scaler = constrain_float(speed_scaler, 0.5f, 2.0f);
    } else {
        speed_scaler = 1.67f;
    }

    return speed_scaler;
}

/*
  implement a software VTail or elevon mixer. There are 4 different mixing modes
 */
void AP_ControlSurfaces::channel_output_mixer(uint8_t mixing_type, int16_t &chan1_out, int16_t &chan2_out)
{
    int16_t c1, c2;
    int16_t v1, v2;

    // first get desired elevator and rudder as -500..500 values
    c1 = chan1_out - 1500;
    c2 = chan2_out - 1500;

    v1 = (c1 - c2) * _mixing_gain;
    v2 = (c1 + c2) * _mixing_gain;

    // now map to mixed output
    switch (mixing_type) {
    case MIXING_DISABLED:
        return;

    case MIXING_UPUP:
        break;

    case MIXING_UPDN:
        v2 = -v2;
        break;

    case MIXING_DNUP:
        v1 = -v1;
        break;

    case MIXING_DNDN:
        v1 = -v1;
        v2 = -v2;
        break;
    }

    // scale for a 1500 center and 900..2100 range, symmetric
    v1 = constrain_int16(v1, -600, 600);
    v2 = constrain_int16(v2, -600, 600);

    chan1_out = 1500 + v1;
    chan2_out = 1500 + v2;
}

/*
  setup flaperon output channels
 */
void AP_ControlSurfaces::flaperon_update(int8_t flap_percent)
{
    if (!RC_Channel_aux::function_assigned(RC_Channel_aux::k_flaperon1) ||
        !RC_Channel_aux::function_assigned(RC_Channel_aux::k_flaperon2)) {
        return;
    }
    int16_t ch1, ch2;
    /*
      flaperons are implemented as a mixer between aileron and a
      percentage of flaps. Flap input can come from a manual channel
      or from auto flaps.

      Use k_flaperon1 and k_flaperon2 channel trims to center servos.
      Then adjust aileron trim for level flight (note that aileron trim is affected
      by mixing gain). flapin_channel's trim is not used.
     */
     
    ch1 = _channel_roll->radio_out;
    // The *5 is to take a percentage to a value from -500 to 500 for the mixer
    ch2 = 1500 - flap_percent * 5;
    channel_output_mixer(_flaperon_output, ch1, ch2);
    RC_Channel_aux::set_radio_trimmed(RC_Channel_aux::k_flaperon1, ch1);
    RC_Channel_aux::set_radio_trimmed(RC_Channel_aux::k_flaperon2, ch2);
}
