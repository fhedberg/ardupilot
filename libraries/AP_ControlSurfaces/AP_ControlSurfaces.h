// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/// @file    AP_ControlSurfaces.h
/// @brief   Control surfaces control library

#ifndef AP_CONTROL_SURFACES_H
#define AP_CONTROL_SURFACES_H

#include <AP_AHRS.h>
#include <RC_Channel.h>

class AP_ControlSurfaces {
public:
    AP_ControlSurfaces(AP_AHRS &ahrs,
                       RC_Channel *channel_pitch,
                       RC_Channel *channel_roll,
                       RC_Channel *channel_rudder) :
      _ahrs(ahrs),
      _channel_pitch(channel_pitch),
      _channel_roll(channel_roll),
      _channel_rudder(channel_rudder)
	{
        AP_Param::setup_object_defaults(this, var_info);
	}

    void output ();

    void channel_output_mixer(uint8_t mixing_type, int16_t &chan1_out, int16_t &chan2_out);

    void flaperon_update(int8_t flap_percent);
    
    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

private:

    // get_speed_scaler
    float get_speed_scaler();

    // internal variables

    const AP_AHRS&            _ahrs;
    RC_Channel*               _channel_pitch;
    RC_Channel*               _channel_roll;
    RC_Channel*               _channel_rudder;

    // parameters

    AP_Int8                   _elevon_output;
    AP_Int8                   _vtail_output;
    AP_Int8                   _flaperon_output;
    AP_Float                  _scaling_speed;
    AP_Float                  _mixing_gain;

    // defines

    enum ChannelMixing {
      MIXING_DISABLED = 0,
      MIXING_UPUP     = 1,
      MIXING_UPDN     = 2,
      MIXING_DNUP     = 3,
      MIXING_DNDN     = 4
  };

};

#endif //AP_CONTROL_SURFACES_H
