#pragma once
// MESSAGE POSTURE PACKING

#define MAVLINK_MSG_ID_POSTURE 12


typedef struct __mavlink_posture_t {
 float pos_x; /*<  x 坐标*/
 float pos_y; /*<  y 位置*/
 float zangle; /*<  航向角*/
 float xangle; /*<  横滚角*/
 float yangle; /*<  俯仰角*/
 float w_z; /*<  航向角速度*/
} mavlink_posture_t;

#define MAVLINK_MSG_ID_POSTURE_LEN 24
#define MAVLINK_MSG_ID_POSTURE_MIN_LEN 24
#define MAVLINK_MSG_ID_12_LEN 24
#define MAVLINK_MSG_ID_12_MIN_LEN 24

#define MAVLINK_MSG_ID_POSTURE_CRC 139
#define MAVLINK_MSG_ID_12_CRC 139



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_POSTURE { \
    12, \
    "POSTURE", \
    6, \
    {  { "pos_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_posture_t, pos_x) }, \
         { "pos_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_posture_t, pos_y) }, \
         { "zangle", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_posture_t, zangle) }, \
         { "xangle", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_posture_t, xangle) }, \
         { "yangle", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_posture_t, yangle) }, \
         { "w_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_posture_t, w_z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_POSTURE { \
    "POSTURE", \
    6, \
    {  { "pos_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_posture_t, pos_x) }, \
         { "pos_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_posture_t, pos_y) }, \
         { "zangle", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_posture_t, zangle) }, \
         { "xangle", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_posture_t, xangle) }, \
         { "yangle", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_posture_t, yangle) }, \
         { "w_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_posture_t, w_z) }, \
         } \
}
#endif

/**
 * @brief Pack a posture message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pos_x  x 坐标
 * @param pos_y  y 位置
 * @param zangle  航向角
 * @param xangle  横滚角
 * @param yangle  俯仰角
 * @param w_z  航向角速度
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_posture_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float pos_x, float pos_y, float zangle, float xangle, float yangle, float w_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POSTURE_LEN];
    _mav_put_float(buf, 0, pos_x);
    _mav_put_float(buf, 4, pos_y);
    _mav_put_float(buf, 8, zangle);
    _mav_put_float(buf, 12, xangle);
    _mav_put_float(buf, 16, yangle);
    _mav_put_float(buf, 20, w_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_POSTURE_LEN);
#else
    mavlink_posture_t packet;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.zangle = zangle;
    packet.xangle = xangle;
    packet.yangle = yangle;
    packet.w_z = w_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_POSTURE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_POSTURE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_POSTURE_MIN_LEN, MAVLINK_MSG_ID_POSTURE_LEN, MAVLINK_MSG_ID_POSTURE_CRC);
}

/**
 * @brief Pack a posture message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pos_x  x 坐标
 * @param pos_y  y 位置
 * @param zangle  航向角
 * @param xangle  横滚角
 * @param yangle  俯仰角
 * @param w_z  航向角速度
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_posture_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float pos_x,float pos_y,float zangle,float xangle,float yangle,float w_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POSTURE_LEN];
    _mav_put_float(buf, 0, pos_x);
    _mav_put_float(buf, 4, pos_y);
    _mav_put_float(buf, 8, zangle);
    _mav_put_float(buf, 12, xangle);
    _mav_put_float(buf, 16, yangle);
    _mav_put_float(buf, 20, w_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_POSTURE_LEN);
#else
    mavlink_posture_t packet;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.zangle = zangle;
    packet.xangle = xangle;
    packet.yangle = yangle;
    packet.w_z = w_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_POSTURE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_POSTURE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_POSTURE_MIN_LEN, MAVLINK_MSG_ID_POSTURE_LEN, MAVLINK_MSG_ID_POSTURE_CRC);
}

/**
 * @brief Encode a posture struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param posture C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_posture_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_posture_t* posture)
{
    return mavlink_msg_posture_pack(system_id, component_id, msg, posture->pos_x, posture->pos_y, posture->zangle, posture->xangle, posture->yangle, posture->w_z);
}

/**
 * @brief Encode a posture struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param posture C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_posture_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_posture_t* posture)
{
    return mavlink_msg_posture_pack_chan(system_id, component_id, chan, msg, posture->pos_x, posture->pos_y, posture->zangle, posture->xangle, posture->yangle, posture->w_z);
}

/**
 * @brief Send a posture message
 * @param chan MAVLink channel to send the message
 *
 * @param pos_x  x 坐标
 * @param pos_y  y 位置
 * @param zangle  航向角
 * @param xangle  横滚角
 * @param yangle  俯仰角
 * @param w_z  航向角速度
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_posture_send(mavlink_channel_t chan, float pos_x, float pos_y, float zangle, float xangle, float yangle, float w_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POSTURE_LEN];
    _mav_put_float(buf, 0, pos_x);
    _mav_put_float(buf, 4, pos_y);
    _mav_put_float(buf, 8, zangle);
    _mav_put_float(buf, 12, xangle);
    _mav_put_float(buf, 16, yangle);
    _mav_put_float(buf, 20, w_z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POSTURE, buf, MAVLINK_MSG_ID_POSTURE_MIN_LEN, MAVLINK_MSG_ID_POSTURE_LEN, MAVLINK_MSG_ID_POSTURE_CRC);
#else
    mavlink_posture_t packet;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.zangle = zangle;
    packet.xangle = xangle;
    packet.yangle = yangle;
    packet.w_z = w_z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POSTURE, (const char *)&packet, MAVLINK_MSG_ID_POSTURE_MIN_LEN, MAVLINK_MSG_ID_POSTURE_LEN, MAVLINK_MSG_ID_POSTURE_CRC);
#endif
}

/**
 * @brief Send a posture message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_posture_send_struct(mavlink_channel_t chan, const mavlink_posture_t* posture)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_posture_send(chan, posture->pos_x, posture->pos_y, posture->zangle, posture->xangle, posture->yangle, posture->w_z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POSTURE, (const char *)posture, MAVLINK_MSG_ID_POSTURE_MIN_LEN, MAVLINK_MSG_ID_POSTURE_LEN, MAVLINK_MSG_ID_POSTURE_CRC);
#endif
}

#if MAVLINK_MSG_ID_POSTURE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_posture_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float pos_x, float pos_y, float zangle, float xangle, float yangle, float w_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, pos_x);
    _mav_put_float(buf, 4, pos_y);
    _mav_put_float(buf, 8, zangle);
    _mav_put_float(buf, 12, xangle);
    _mav_put_float(buf, 16, yangle);
    _mav_put_float(buf, 20, w_z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POSTURE, buf, MAVLINK_MSG_ID_POSTURE_MIN_LEN, MAVLINK_MSG_ID_POSTURE_LEN, MAVLINK_MSG_ID_POSTURE_CRC);
#else
    mavlink_posture_t *packet = (mavlink_posture_t *)msgbuf;
    packet->pos_x = pos_x;
    packet->pos_y = pos_y;
    packet->zangle = zangle;
    packet->xangle = xangle;
    packet->yangle = yangle;
    packet->w_z = w_z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POSTURE, (const char *)packet, MAVLINK_MSG_ID_POSTURE_MIN_LEN, MAVLINK_MSG_ID_POSTURE_LEN, MAVLINK_MSG_ID_POSTURE_CRC);
#endif
}
#endif

#endif

// MESSAGE POSTURE UNPACKING


/**
 * @brief Get field pos_x from posture message
 *
 * @return  x 坐标
 */
static inline float mavlink_msg_posture_get_pos_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pos_y from posture message
 *
 * @return  y 位置
 */
static inline float mavlink_msg_posture_get_pos_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field zangle from posture message
 *
 * @return  航向角
 */
static inline float mavlink_msg_posture_get_zangle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field xangle from posture message
 *
 * @return  横滚角
 */
static inline float mavlink_msg_posture_get_xangle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field yangle from posture message
 *
 * @return  俯仰角
 */
static inline float mavlink_msg_posture_get_yangle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field w_z from posture message
 *
 * @return  航向角速度
 */
static inline float mavlink_msg_posture_get_w_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a posture message into a struct
 *
 * @param msg The message to decode
 * @param posture C-struct to decode the message contents into
 */
static inline void mavlink_msg_posture_decode(const mavlink_message_t* msg, mavlink_posture_t* posture)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    posture->pos_x = mavlink_msg_posture_get_pos_x(msg);
    posture->pos_y = mavlink_msg_posture_get_pos_y(msg);
    posture->zangle = mavlink_msg_posture_get_zangle(msg);
    posture->xangle = mavlink_msg_posture_get_xangle(msg);
    posture->yangle = mavlink_msg_posture_get_yangle(msg);
    posture->w_z = mavlink_msg_posture_get_w_z(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_POSTURE_LEN? msg->len : MAVLINK_MSG_ID_POSTURE_LEN;
        memset(posture, 0, MAVLINK_MSG_ID_POSTURE_LEN);
    memcpy(posture, _MAV_PAYLOAD(msg), len);
#endif
}
