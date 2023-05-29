/** @file
 *  @brief MAVLink comm protocol generated from chassis.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_CHASSIS_H
#define MAVLINK_CHASSIS_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_CHASSIS.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#undef MAVLINK_THIS_XML_HASH
#define MAVLINK_THIS_XML_HASH 8960257948822207091

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{9, 96, 24, 24, 0, 0, 0}, {10, 101, 28, 28, 0, 0, 0}, {11, 95, 28, 28, 0, 0, 0}, {209, 219, 25, 25, 0, 0, 0}, {210, 142, 18, 18, 0, 0, 0}, {211, 56, 21, 21, 0, 0, 0}, {212, 22, 21, 21, 0, 0, 0}, {213, 36, 1, 1, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_CHASSIS

// ENUM DEFINITIONS



// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 3
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 3
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_control.h"
#include "./mavlink_msg_posture.h"
#include "./mavlink_msg_chassis_to_upper.h"
#include "./mavlink_msg_joystick_air.h"
#include "./mavlink_msg_joystick_air_led.h"
#include "./mavlink_msg_joystick_air_dashboard_set_title.h"
#include "./mavlink_msg_joystick_air_dashboard_set_msg.h"
#include "./mavlink_msg_joystick_air_dashboard_del.h"

// base include


#undef MAVLINK_THIS_XML_HASH
#define MAVLINK_THIS_XML_HASH 8960257948822207091

#if MAVLINK_THIS_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_CONTROL, MAVLINK_MESSAGE_INFO_POSTURE, MAVLINK_MESSAGE_INFO_CHASSIS_TO_UPPER, MAVLINK_MESSAGE_INFO_JOYSTICK_AIR, MAVLINK_MESSAGE_INFO_JOYSTICK_AIR_LED, MAVLINK_MESSAGE_INFO_JOYSTICK_AIR_DASHBOARD_SET_TITLE, MAVLINK_MESSAGE_INFO_JOYSTICK_AIR_DASHBOARD_SET_MSG, MAVLINK_MESSAGE_INFO_JOYSTICK_AIR_DASHBOARD_DEL}
# define MAVLINK_MESSAGE_NAMES {{ "CHASSIS_TO_UPPER", 11 }, { "CONTROL", 9 }, { "JOYSTICK_AIR", 209 }, { "JOYSTICK_AIR_DASHBOARD_DEL", 213 }, { "JOYSTICK_AIR_DASHBOARD_SET_MSG", 212 }, { "JOYSTICK_AIR_DASHBOARD_SET_TITLE", 211 }, { "JOYSTICK_AIR_LED", 210 }, { "POSTURE", 10 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_CHASSIS_H
