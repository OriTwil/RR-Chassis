#pragma once
// MESSAGE CHASSIS_TO_UPPER PACKING

#define MAVLINK_MSG_ID_CHASSIS_TO_UPPER 11


typedef struct __mavlink_chassis_to_upper_t {
 float pos_x; /*<  x 坐标*/
 float pos_y; /*<  y 位置*/
 float zangle; /*<   航向角*/
 float xangle; /*<   横滚角*/
 float yangle; /*<   俯仰角*/
 float w_z; /*<  航向角速度*/
 int32_t point; /*<  目标点*/
} mavlink_chassis_to_upper_t;

#define MAVLINK_MSG_ID_CHASSIS_TO_UPPER_LEN 28
#define MAVLINK_MSG_ID_CHASSIS_TO_UPPER_MIN_LEN 28
#define MAVLINK_MSG_ID_11_LEN 28
#define MAVLINK_MSG_ID_11_MIN_LEN 28

#define MAVLINK_MSG_ID_CHASSIS_TO_UPPER_CRC 95
#define MAVLINK_MSG_ID_11_CRC 95



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CHASSIS_TO_UPPER { \
    11, \
    "CHASSIS_TO_UPPER", \
    7, \
    {  { "pos_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_chassis_to_upper_t, pos_x) }, \
         { "pos_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_chassis_to_upper_t, pos_y) }, \
         { "zangle", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_chassis_to_upper_t, zangle) }, \
         { "xangle", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_chassis_to_upper_t, xangle) }, \
         { "yangle", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_chassis_to_upper_t, yangle) }, \
         { "w_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_chassis_to_upper_t, w_z) }, \
         { "point", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_chassis_to_upper_t, point) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CHASSIS_TO_UPPER { \
    "CHASSIS_TO_UPPER", \
    7, \
    {  { "pos_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_chassis_to_upper_t, pos_x) }, \
         { "pos_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_chassis_to_upper_t, pos_y) }, \
         { "zangle", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_chassis_to_upper_t, zangle) }, \
         { "xangle", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_chassis_to_upper_t, xangle) }, \
         { "yangle", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_chassis_to_upper_t, yangle) }, \
         { "w_z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_chassis_to_upper_t, w_z) }, \
         { "point", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_chassis_to_upper_t, point) }, \
         } \
}
#endif

/**
 * @brief Pack a chassis_to_upper message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pos_x  x 坐标
 * @param pos_y  y 位置
 * @param zangle   航向角
 * @param xangle   横滚角
 * @param yangle   俯仰角
 * @param w_z  航向角速度
 * @param point  目标点
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chassis_to_upper_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float pos_x, float pos_y, float zangle, float xangle, float yangle, float w_z, int32_t point)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHASSIS_TO_UPPER_LEN];
    _mav_put_float(buf, 0, pos_x);
    _mav_put_float(buf, 4, pos_y);
    _mav_put_float(buf, 8, zangle);
    _mav_put_float(buf, 12, xangle);
    _mav_put_float(buf, 16, yangle);
    _mav_put_float(buf, 20, w_z);
    _mav_put_int32_t(buf, 24, point);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_LEN);
#else
    mavlink_chassis_to_upper_t packet;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.zangle = zangle;
    packet.xangle = xangle;
    packet.yangle = yangle;
    packet.w_z = w_z;
    packet.point = point;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CHASSIS_TO_UPPER;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_MIN_LEN, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_LEN, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_CRC);
}

/**
 * @brief Pack a chassis_to_upper message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pos_x  x 坐标
 * @param pos_y  y 位置
 * @param zangle   航向角
 * @param xangle   横滚角
 * @param yangle   俯仰角
 * @param w_z  航向角速度
 * @param point  目标点
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_chassis_to_upper_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float pos_x,float pos_y,float zangle,float xangle,float yangle,float w_z,int32_t point)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHASSIS_TO_UPPER_LEN];
    _mav_put_float(buf, 0, pos_x);
    _mav_put_float(buf, 4, pos_y);
    _mav_put_float(buf, 8, zangle);
    _mav_put_float(buf, 12, xangle);
    _mav_put_float(buf, 16, yangle);
    _mav_put_float(buf, 20, w_z);
    _mav_put_int32_t(buf, 24, point);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_LEN);
#else
    mavlink_chassis_to_upper_t packet;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.zangle = zangle;
    packet.xangle = xangle;
    packet.yangle = yangle;
    packet.w_z = w_z;
    packet.point = point;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CHASSIS_TO_UPPER;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_MIN_LEN, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_LEN, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_CRC);
}

/**
 * @brief Encode a chassis_to_upper struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param chassis_to_upper C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chassis_to_upper_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_chassis_to_upper_t* chassis_to_upper)
{
    return mavlink_msg_chassis_to_upper_pack(system_id, component_id, msg, chassis_to_upper->pos_x, chassis_to_upper->pos_y, chassis_to_upper->zangle, chassis_to_upper->xangle, chassis_to_upper->yangle, chassis_to_upper->w_z, chassis_to_upper->point);
}

/**
 * @brief Encode a chassis_to_upper struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param chassis_to_upper C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_chassis_to_upper_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_chassis_to_upper_t* chassis_to_upper)
{
    return mavlink_msg_chassis_to_upper_pack_chan(system_id, component_id, chan, msg, chassis_to_upper->pos_x, chassis_to_upper->pos_y, chassis_to_upper->zangle, chassis_to_upper->xangle, chassis_to_upper->yangle, chassis_to_upper->w_z, chassis_to_upper->point);
}

/**
 * @brief Send a chassis_to_upper message
 * @param chan MAVLink channel to send the message
 *
 * @param pos_x  x 坐标
 * @param pos_y  y 位置
 * @param zangle   航向角
 * @param xangle   横滚角
 * @param yangle   俯仰角
 * @param w_z  航向角速度
 * @param point  目标点
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_chassis_to_upper_send(mavlink_channel_t chan, float pos_x, float pos_y, float zangle, float xangle, float yangle, float w_z, int32_t point)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CHASSIS_TO_UPPER_LEN];
    _mav_put_float(buf, 0, pos_x);
    _mav_put_float(buf, 4, pos_y);
    _mav_put_float(buf, 8, zangle);
    _mav_put_float(buf, 12, xangle);
    _mav_put_float(buf, 16, yangle);
    _mav_put_float(buf, 20, w_z);
    _mav_put_int32_t(buf, 24, point);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASSIS_TO_UPPER, buf, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_MIN_LEN, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_LEN, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_CRC);
#else
    mavlink_chassis_to_upper_t packet;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.zangle = zangle;
    packet.xangle = xangle;
    packet.yangle = yangle;
    packet.w_z = w_z;
    packet.point = point;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASSIS_TO_UPPER, (const char *)&packet, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_MIN_LEN, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_LEN, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_CRC);
#endif
}

/**
 * @brief Send a chassis_to_upper message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_chassis_to_upper_send_struct(mavlink_channel_t chan, const mavlink_chassis_to_upper_t* chassis_to_upper)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_chassis_to_upper_send(chan, chassis_to_upper->pos_x, chassis_to_upper->pos_y, chassis_to_upper->zangle, chassis_to_upper->xangle, chassis_to_upper->yangle, chassis_to_upper->w_z, chassis_to_upper->point);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASSIS_TO_UPPER, (const char *)chassis_to_upper, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_MIN_LEN, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_LEN, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_CRC);
#endif
}

#if MAVLINK_MSG_ID_CHASSIS_TO_UPPER_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_chassis_to_upper_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float pos_x, float pos_y, float zangle, float xangle, float yangle, float w_z, int32_t point)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, pos_x);
    _mav_put_float(buf, 4, pos_y);
    _mav_put_float(buf, 8, zangle);
    _mav_put_float(buf, 12, xangle);
    _mav_put_float(buf, 16, yangle);
    _mav_put_float(buf, 20, w_z);
    _mav_put_int32_t(buf, 24, point);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASSIS_TO_UPPER, buf, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_MIN_LEN, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_LEN, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_CRC);
#else
    mavlink_chassis_to_upper_t *packet = (mavlink_chassis_to_upper_t *)msgbuf;
    packet->pos_x = pos_x;
    packet->pos_y = pos_y;
    packet->zangle = zangle;
    packet->xangle = xangle;
    packet->yangle = yangle;
    packet->w_z = w_z;
    packet->point = point;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CHASSIS_TO_UPPER, (const char *)packet, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_MIN_LEN, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_LEN, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_CRC);
#endif
}
#endif

#endif

// MESSAGE CHASSIS_TO_UPPER UNPACKING


/**
 * @brief Get field pos_x from chassis_to_upper message
 *
 * @return  x 坐标
 */
static inline float mavlink_msg_chassis_to_upper_get_pos_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pos_y from chassis_to_upper message
 *
 * @return  y 位置
 */
static inline float mavlink_msg_chassis_to_upper_get_pos_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field zangle from chassis_to_upper message
 *
 * @return   航向角
 */
static inline float mavlink_msg_chassis_to_upper_get_zangle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field xangle from chassis_to_upper message
 *
 * @return   横滚角
 */
static inline float mavlink_msg_chassis_to_upper_get_xangle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field yangle from chassis_to_upper message
 *
 * @return   俯仰角
 */
static inline float mavlink_msg_chassis_to_upper_get_yangle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field w_z from chassis_to_upper message
 *
 * @return  航向角速度
 */
static inline float mavlink_msg_chassis_to_upper_get_w_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field point from chassis_to_upper message
 *
 * @return  目标点
 */
static inline int32_t mavlink_msg_chassis_to_upper_get_point(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  24);
}

/**
 * @brief Decode a chassis_to_upper message into a struct
 *
 * @param msg The message to decode
 * @param chassis_to_upper C-struct to decode the message contents into
 */
static inline void mavlink_msg_chassis_to_upper_decode(const mavlink_message_t* msg, mavlink_chassis_to_upper_t* chassis_to_upper)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    chassis_to_upper->pos_x = mavlink_msg_chassis_to_upper_get_pos_x(msg);
    chassis_to_upper->pos_y = mavlink_msg_chassis_to_upper_get_pos_y(msg);
    chassis_to_upper->zangle = mavlink_msg_chassis_to_upper_get_zangle(msg);
    chassis_to_upper->xangle = mavlink_msg_chassis_to_upper_get_xangle(msg);
    chassis_to_upper->yangle = mavlink_msg_chassis_to_upper_get_yangle(msg);
    chassis_to_upper->w_z = mavlink_msg_chassis_to_upper_get_w_z(msg);
    chassis_to_upper->point = mavlink_msg_chassis_to_upper_get_point(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CHASSIS_TO_UPPER_LEN? msg->len : MAVLINK_MSG_ID_CHASSIS_TO_UPPER_LEN;
        memset(chassis_to_upper, 0, MAVLINK_MSG_ID_CHASSIS_TO_UPPER_LEN);
    memcpy(chassis_to_upper, _MAV_PAYLOAD(msg), len);
#endif
}
