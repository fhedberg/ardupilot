#include "Node.h"

/*
 *  ArduNode parameter definitions
 *
 */

#define GSCALAR(v, name, def) { node.g.v.vtype, name, Parameters::k_param_ ## v, &node.g.v, {def_value : def} }
#define ASCALAR(v, name, def) { node.aparm.v.vtype, name, Parameters::k_param_ ## v, (const void *)&node.aparm.v, {def_value : def} }
#define GGROUP(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, &node.g.v, {group_info : class::var_info} }
#define GOBJECT(v, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## v, (const void *)&node.v, {group_info : class::var_info} }
#define GOBJECTN(v, pname, name, class) { AP_PARAM_GROUP, name, Parameters::k_param_ ## pname, (const void *)&node.v, {group_info : class::var_info} }

const AP_Param::Info Node::var_info[] = {
    // @Param: FORMAT_VERSION
    // @DisplayName: Eeprom format version number
    // @Description: This value is incremented when changes are made to the eeprom format
    // @User: Advanced
    GSCALAR(format_version,         "FORMAT_VERSION", 0),

    // @Param: SYSID_THISMAV
    // @DisplayName: MAVLink system ID of this vehicle
    // @Description: Allows setting an individual MAVLink system id for this vehicle to distinguish it from others on the same network
    // @Range: 1 255
    // @User: Advanced
    //GSCALAR(sysid_this_mav,         "SYSID_THISMAV",  MAV_SYSTEM_ID),

    // @Param: SYSID_MYGCS
    // @DisplayName: Ground station MAVLink system ID
    // @Description: The identifier of the ground station in the MAVLink protocol. Don't change this unless you also modify the ground station to match.
    // @Range: 1 255
    // @User: Advanced
    //GSCALAR(sysid_my_gcs,           "SYSID_MYGCS",    255),

    // @Group: SERIAL
    // @Path: ../libraries/AP_SerialManager/AP_SerialManager.cpp
    GOBJECT(serial_manager, "SERIAL",   AP_SerialManager)
};
