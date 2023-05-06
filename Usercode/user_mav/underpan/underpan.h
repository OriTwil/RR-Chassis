/** @file
 *  @brief MAVLink comm protocol generated from underpan.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_UNDERPAN_H
#define MAVLINK_UNDERPAN_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_UNDERPAN.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_UNDERPAN_XML_HASH 870125114321110965

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{1, 189, 9, 9, 0, 0, 0}, {9, 207, 24, 24, 0, 0, 0}, {10, 142, 12, 12, 0, 0, 0}, {12, 139, 24, 24, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_UNDERPAN

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
#include "./mavlink_msg_controller.h"
#include "./mavlink_msg_control_set.h"
#include "./mavlink_msg_speed_status.h"
#include "./mavlink_msg_posture.h"

// base include



#if MAVLINK_UNDERPAN_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_Controller, MAVLINK_MESSAGE_INFO_CONTROL_SET, MAVLINK_MESSAGE_INFO_SPEED_STATUS, MAVLINK_MESSAGE_INFO_POSTURE}
# define MAVLINK_MESSAGE_NAMES {{ "CONTROL_SET", 9 }, { "Controller", 1 }, { "POSTURE", 12 }, { "SPEED_STATUS", 10 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_UNDERPAN_H
