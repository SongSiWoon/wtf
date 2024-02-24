/** @file
 *  @brief MAVLink comm protocol generated from swarm.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_SWARM_H
#define MAVLINK_SWARM_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_SWARM.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_SWARM_XML_HASH -184688163154664767

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{180, 206, 56, 56, 0, 0, 0}, {181, 180, 50, 50, 1, 16, 0}, {182, 5, 5, 5, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_SWARM

// ENUM DEFINITIONS


/** @brief Enumeration of distance sensor types */
#ifndef HAVE_ENUM_SCENARIO_CMD_ENUM
#define HAVE_ENUM_SCENARIO_CMD_ENUM
typedef enum SCENARIO_CMD_ENUM
{
   SCENARIO_CMD_SET_START_TIME=0, /* set start time for scenario  (param1: start_time) | */
   SCENARIO_CMD_STOP_SCENARIO=1, /* stop scenario | */
   SCENARIO_CMD_EMERGENCY_LAND=2, /* land | */
   SCENARIO_CMD_SET_CONFIGS=3, /* set vairous configurations param1: base vehicle pos x, param2: base vehicle pos y, param5: scenario file name | */
   SCENARIO_CMD_RESET_CONFIGS=4, /* reset configurations | */
   SCENARIO_CMD_ENUM_ENUM_END=5, /*  | */
} SCENARIO_CMD_ENUM;
#endif

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_monitoring.h"
#include "./mavlink_msg_scenario_cmd.h"
#include "./mavlink_msg_led_control.h"

// base include



#if MAVLINK_SWARM_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_MONITORING, MAVLINK_MESSAGE_INFO_SCENARIO_CMD, MAVLINK_MESSAGE_INFO_LED_CONTROL}
# define MAVLINK_MESSAGE_NAMES {{ "LED_CONTROL", 182 }, { "MONITORING", 180 }, { "SCENARIO_CMD", 181 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_SWARM_H
