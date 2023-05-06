#pragma once
// MESSAGE SPEED_CONTROL_STATUS PACKING

#define MAVLINK_MSG_ID_SPEED_CONTROL_STATUS 10


typedef struct __mavlink_speed_control_status_t {
 float vx_state; /*<  x 方向速度*/
 float vy_state; /*<  y 方向速度*/
 float vw_state; /*<  z 方向角速度*/
} mavlink_speed_control_status_t;

#define MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_LEN 12
#define MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_MIN_LEN 12
#define MAVLINK_MSG_ID_10_LEN 12
#define MAVLINK_MSG_ID_10_MIN_LEN 12

#define MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_CRC 240
#define MAVLINK_MSG_ID_10_CRC 240



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SPEED_CONTROL_STATUS { \
    10, \
    "SPEED_CONTROL_STATUS", \
    3, \
    {  { "vx_state", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_speed_control_status_t, vx_state) }, \
         { "vy_state", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_speed_control_status_t, vy_state) }, \
         { "vw_state", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_speed_control_status_t, vw_state) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SPEED_CONTROL_STATUS { \
    "SPEED_CONTROL_STATUS", \
    3, \
    {  { "vx_state", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_speed_control_status_t, vx_state) }, \
         { "vy_state", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_speed_control_status_t, vy_state) }, \
         { "vw_state", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_speed_control_status_t, vw_state) }, \
         } \
}
#endif

/**
 * @brief Pack a speed_control_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param vx_state  x 方向速度
 * @param vy_state  y 方向速度
 * @param vw_state  z 方向角速度
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_speed_control_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float vx_state, float vy_state, float vw_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_LEN];
    _mav_put_float(buf, 0, vx_state);
    _mav_put_float(buf, 4, vy_state);
    _mav_put_float(buf, 8, vw_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_LEN);
#else
    mavlink_speed_control_status_t packet;
    packet.vx_state = vx_state;
    packet.vy_state = vy_state;
    packet.vw_state = vw_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SPEED_CONTROL_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_LEN, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_CRC);
}

/**
 * @brief Pack a speed_control_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vx_state  x 方向速度
 * @param vy_state  y 方向速度
 * @param vw_state  z 方向角速度
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_speed_control_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float vx_state,float vy_state,float vw_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_LEN];
    _mav_put_float(buf, 0, vx_state);
    _mav_put_float(buf, 4, vy_state);
    _mav_put_float(buf, 8, vw_state);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_LEN);
#else
    mavlink_speed_control_status_t packet;
    packet.vx_state = vx_state;
    packet.vy_state = vy_state;
    packet.vw_state = vw_state;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SPEED_CONTROL_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_LEN, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_CRC);
}

/**
 * @brief Encode a speed_control_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param speed_control_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_speed_control_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_speed_control_status_t* speed_control_status)
{
    return mavlink_msg_speed_control_status_pack(system_id, component_id, msg, speed_control_status->vx_state, speed_control_status->vy_state, speed_control_status->vw_state);
}

/**
 * @brief Encode a speed_control_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param speed_control_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_speed_control_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_speed_control_status_t* speed_control_status)
{
    return mavlink_msg_speed_control_status_pack_chan(system_id, component_id, chan, msg, speed_control_status->vx_state, speed_control_status->vy_state, speed_control_status->vw_state);
}

/**
 * @brief Send a speed_control_status message
 * @param chan MAVLink channel to send the message
 *
 * @param vx_state  x 方向速度
 * @param vy_state  y 方向速度
 * @param vw_state  z 方向角速度
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_speed_control_status_send(mavlink_channel_t chan, float vx_state, float vy_state, float vw_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_LEN];
    _mav_put_float(buf, 0, vx_state);
    _mav_put_float(buf, 4, vy_state);
    _mav_put_float(buf, 8, vw_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS, buf, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_LEN, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_CRC);
#else
    mavlink_speed_control_status_t packet;
    packet.vx_state = vx_state;
    packet.vy_state = vy_state;
    packet.vw_state = vw_state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS, (const char *)&packet, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_LEN, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_CRC);
#endif
}

/**
 * @brief Send a speed_control_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_speed_control_status_send_struct(mavlink_channel_t chan, const mavlink_speed_control_status_t* speed_control_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_speed_control_status_send(chan, speed_control_status->vx_state, speed_control_status->vy_state, speed_control_status->vw_state);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS, (const char *)speed_control_status, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_LEN, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_speed_control_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float vx_state, float vy_state, float vw_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, vx_state);
    _mav_put_float(buf, 4, vy_state);
    _mav_put_float(buf, 8, vw_state);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS, buf, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_LEN, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_CRC);
#else
    mavlink_speed_control_status_t *packet = (mavlink_speed_control_status_t *)msgbuf;
    packet->vx_state = vx_state;
    packet->vy_state = vy_state;
    packet->vw_state = vw_state;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS, (const char *)packet, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_MIN_LEN, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_LEN, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE SPEED_CONTROL_STATUS UNPACKING


/**
 * @brief Get field vx_state from speed_control_status message
 *
 * @return  x 方向速度
 */
static inline float mavlink_msg_speed_control_status_get_vx_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field vy_state from speed_control_status message
 *
 * @return  y 方向速度
 */
static inline float mavlink_msg_speed_control_status_get_vy_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field vw_state from speed_control_status message
 *
 * @return  z 方向角速度
 */
static inline float mavlink_msg_speed_control_status_get_vw_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a speed_control_status message into a struct
 *
 * @param msg The message to decode
 * @param speed_control_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_speed_control_status_decode(const mavlink_message_t* msg, mavlink_speed_control_status_t* speed_control_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    speed_control_status->vx_state = mavlink_msg_speed_control_status_get_vx_state(msg);
    speed_control_status->vy_state = mavlink_msg_speed_control_status_get_vy_state(msg);
    speed_control_status->vw_state = mavlink_msg_speed_control_status_get_vw_state(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_LEN? msg->len : MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_LEN;
        memset(speed_control_status, 0, MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_LEN);
    memcpy(speed_control_status, _MAV_PAYLOAD(msg), len);
#endif
}
