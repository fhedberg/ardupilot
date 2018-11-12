#pragma once

#include <cmath>
#include <stdarg.h>
#include <stdio.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_BattMonitor/AP_BattMonitor.h>

#include "defines.h"
#include "config.h"
#include "Parameters.h"

class Node : public AP_HAL::HAL::Callbacks {
public:
  friend class Parameters;

  Node(void);

  void setup() override;
  void loop() override;

private:
  Parameters g;
  AP_Scheduler scheduler;
  AP_BoardConfig board_config;
  AP_BoardConfig_CAN board_config_can;
  AP_SerialManager serial_manager;

  AP_GPS gps;
  AP_Baro barometer;
  Compass compass;
  AP_InertialSensor ins;
  RangeFinder rangefinder{serial_manager, ROTATION_PITCH_270};

  AP_Notify notify;

  float G_Dt = 0.02f;
  NavEKF2 EKF2{&ahrs, rangefinder};
  NavEKF3 EKF3{&ahrs, rangefinder};
  AP_AHRS_NavEKF ahrs{EKF2, EKF3};

  AP_Param param_loader{var_info};
  static const AP_Param::Info var_info[];
  static const AP_Scheduler::Task scheduler_tasks[];

public:
  void ahrs_update();
};

extern const AP_HAL::HAL& hal;
extern Node node;

using AP_HAL::millis;
using AP_HAL::micros;
