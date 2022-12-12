#pragma once
// MESSAGE CONTROL_SET PACKING

#define MAVLINK_MSG_ID_CONTROL_SET 9


typedef struct __mavlink_control_set_t {
 float vx_set; /*<  x 方向速度*/
 float vy_set; /*<  y 方向速度*/
 float vw_set; /*<  z 方向角速度*/
 float x_set; /*<  x 方向速度*/
 float y_set; /*<  y 方向速度*/
 float w_set; /*<  z 方向角速度*/
} mavlink_control_set_t;

#define MAVLINK_MSG_ID_CONTROL_SET_LEN 24
#define MAVLINK_MSG_ID_CONTROL_SET_MIN_LEN 24
#define MAVLINK_MSG_ID_9_LEN 24
#define MAVLINK_MSG_ID_9_MIN_LEN 24

#define MAVLINK_MSG_ID_CONTROL_SET_CRC 207
#define MAVLINK_MSG_ID_9_CRC 207



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CONTROL_SET { \
    9, \
    "CONTROL_SET", \
    6, \
    {  { "vx_set", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_control_set_t, vx_set) }, \
         { "vy_set", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_control_set_t, vy_set) }, \
         { "vw_set", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_control_set_t, vw_set) }, \
         { "x_set", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_control_set_t, x_set) }, \
         { "y_set", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_control_set_t, y_set) }, \
         { "w_set", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_control_set_t, w_set) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CONTROL_SET { \
    "CONTROL_SET", \
    6, \
    {  { "vx_set", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_control_set_t, vx_set) }, \
         { "vy_set", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_control_set_t, vy_set) }, \
         { "vw_set", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_control_set_t, vw_set) }, \
         { "x_set", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_control_set_t, x_set) }, \
         { "y_set", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_control_set_t, y_set) }, \
         { "w_set", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_control_set_t, w_set) }, \
         } \
}
#endif

/**
 * @brief Pack a control_set message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param vx_set  x 方向速度
 * @param vy_set  y 方向速度
 * @param vw_set  z 方向角速度
 * @param x_set  x 方向速度
 * @param y_set  y 方向速度
 * @param w_set  z 方向角速度
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_control_set_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float vx_set, float vy_set, float vw_set, float x_set, float y_set, float w_set)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CONTROL_SET_LEN];
    _mav_put_float(buf, 0, vx_set);
    _mav_put_float(buf, 4, vy_set);
    _mav_put_float(buf, 8, vw_set);
    _mav_put_float(buf, 12, x_set);
    _mav_put_float(buf, 16, y_set);
    _mav_put_float(buf, 20, w_set);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CONTROL_SET_LEN);
#else
    mavlink_control_set_t packet;
    packet.vx_set = vx_set;
    packet.vy_set = vy_set;
    packet.vw_set = vw_set;
    packet.x_set = x_set;
    packet.y_set = y_set;
    packet.w_set = w_set;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CONTROL_SET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CONTROL_SET;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CONTROL_SET_MIN_LEN, MAVLINK_MSG_ID_CONTROL_SET_LEN, MAVLINK_MSG_ID_CONTROL_SET_CRC);
}

/**
 * @brief Pack a control_set message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param vx_set  x 方向速度
 * @param vy_set  y 方向速度
 * @param vw_set  z 方向角速度
 * @param x_set  x 方向速度
 * @param y_set  y 方向速度
 * @param w_set  z 方向角速度
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_control_set_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float vx_set,float vy_set,float vw_set,float x_set,float y_set,float w_set)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CONTROL_SET_LEN];
    _mav_put_float(buf, 0, vx_set);
    _mav_put_float(buf, 4, vy_set);
    _mav_put_float(buf, 8, vw_set);
    _mav_put_float(buf, 12, x_set);
    _mav_put_float(buf, 16, y_set);
    _mav_put_float(buf, 20, w_set);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CONTROL_SET_LEN);
#else
    mavlink_control_set_t packet;
    packet.vx_set = vx_set;
    packet.vy_set = vy_set;
    packet.vw_set = vw_set;
    packet.x_set = x_set;
    packet.y_set = y_set;
    packet.w_set = w_set;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CONTROL_SET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CONTROL_SET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CONTROL_SET_MIN_LEN, MAVLINK_MSG_ID_CONTROL_SET_LEN, MAVLINK_MSG_ID_CONTROL_SET_CRC);
}

/**
 * @brief Encode a control_set struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param control_set C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_control_set_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_control_set_t* control_set)
{
    return mavlink_msg_control_set_pack(system_id, component_id, msg, control_set->vx_set, control_set->vy_set, control_set->vw_set, control_set->x_set, control_set->y_set, control_set->w_set);
}

/**
 * @brief Encode a control_set struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param control_set C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_control_set_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_control_set_t* control_set)
{
    return mavlink_msg_control_set_pack_chan(system_id, component_id, chan, msg, control_set->vx_set, control_set->vy_set, control_set->vw_set, control_set->x_set, control_set->y_set, control_set->w_set);
}

/**
 * @brief Send a control_set message
 * @param chan MAVLink channel to send the message
 *
 * @param vx_set  x 方向速度
 * @param vy_set  y 方向速度
 * @param vw_set  z 方向角速度
 * @param x_set  x 方向速度
 * @param y_set  y 方向速度
 * @param w_set  z 方向角速度
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_control_set_send(mavlink_channel_t chan, float vx_set, float vy_set, float vw_set, float x_set, float y_set, float w_set)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CONTROL_SET_LEN];
    _mav_put_float(buf, 0, vx_set);
    _mav_put_float(buf, 4, vy_set);
    _mav_put_float(buf, 8, vw_set);
    _mav_put_float(buf, 12, x_set);
    _mav_put_float(buf, 16, y_set);
    _mav_put_float(buf, 20, w_set);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_SET, buf, MAVLINK_MSG_ID_CONTROL_SET_MIN_LEN, MAVLINK_MSG_ID_CONTROL_SET_LEN, MAVLINK_MSG_ID_CONTROL_SET_CRC);
#else
    mavlink_control_set_t packet;
    packet.vx_set = vx_set;
    packet.vy_set = vy_set;
    packet.vw_set = vw_set;
    packet.x_set = x_set;
    packet.y_set = y_set;
    packet.w_set = w_set;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_SET, (const char *)&packet, MAVLINK_MSG_ID_CONTROL_SET_MIN_LEN, MAVLINK_MSG_ID_CONTROL_SET_LEN, MAVLINK_MSG_ID_CONTROL_SET_CRC);
#endif
}

/**
 * @brief Send a control_set message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_control_set_send_struct(mavlink_channel_t chan, const mavlink_control_set_t* control_set)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_control_set_send(chan, control_set->vx_set, control_set->vy_set, control_set->vw_set, control_set->x_set, control_set->y_set, control_set->w_set);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_SET, (const char *)control_set, MAVLINK_MSG_ID_CONTROL_SET_MIN_LEN, MAVLINK_MSG_ID_CONTROL_SET_LEN, MAVLINK_MSG_ID_CONTROL_SET_CRC);
#endif
}

#if MAVLINK_MSG_ID_CONTROL_SET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_control_set_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float vx_set, float vy_set, float vw_set, float x_set, float y_set, float w_set)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, vx_set);
    _mav_put_float(buf, 4, vy_set);
    _mav_put_float(buf, 8, vw_set);
    _mav_put_float(buf, 12, x_set);
    _mav_put_float(buf, 16, y_set);
    _mav_put_float(buf, 20, w_set);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_SET, buf, MAVLINK_MSG_ID_CONTROL_SET_MIN_LEN, MAVLINK_MSG_ID_CONTROL_SET_LEN, MAVLINK_MSG_ID_CONTROL_SET_CRC);
#else
    mavlink_control_set_t *packet = (mavlink_control_set_t *)msgbuf;
    packet->vx_set = vx_set;
    packet->vy_set = vy_set;
    packet->vw_set = vw_set;
    packet->x_set = x_set;
    packet->y_set = y_set;
    packet->w_set = w_set;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_SET, (const char *)packet, MAVLINK_MSG_ID_CONTROL_SET_MIN_LEN, MAVLINK_MSG_ID_CONTROL_SET_LEN, MAVLINK_MSG_ID_CONTROL_SET_CRC);
#endif
}
#endif

#endif

// MESSAGE CONTROL_SET UNPACKING


/**
 * @brief Get field vx_set from control_set message
 *
 * @return  x 方向速度
 */
static inline float mavlink_msg_control_set_get_vx_set(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field vy_set from control_set message
 *
 * @return  y 方向速度
 */
static inline float mavlink_msg_control_set_get_vy_set(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field vw_set from control_set message
 *
 * @return  z 方向角速度
 */
static inline float mavlink_msg_control_set_get_vw_set(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field x_set from control_set message
 *
 * @return  x 方向速度
 */
static inline float mavlink_msg_control_set_get_x_set(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field y_set from control_set message
 *
 * @return  y 方向速度
 */
static inline float mavlink_msg_control_set_get_y_set(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field w_set from control_set message
 *
 * @return  z 方向角速度
 */
static inline float mavlink_msg_control_set_get_w_set(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a control_set message into a struct
 *
 * @param msg The message to decode
 * @param control_set C-struct to decode the message contents into
 */
static inline void mavlink_msg_control_set_decode(const mavlink_message_t* msg, mavlink_control_set_t* control_set)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    control_set->vx_set = mavlink_msg_control_set_get_vx_set(msg);
    control_set->vy_set = mavlink_msg_control_set_get_vy_set(msg);
    control_set->vw_set = mavlink_msg_control_set_get_vw_set(msg);
    control_set->x_set = mavlink_msg_control_set_get_x_set(msg);
    control_set->y_set = mavlink_msg_control_set_get_y_set(msg);
    control_set->w_set = mavlink_msg_control_set_get_w_set(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CONTROL_SET_LEN? msg->len : MAVLINK_MSG_ID_CONTROL_SET_LEN;
        memset(control_set, 0, MAVLINK_MSG_ID_CONTROL_SET_LEN);
    memcpy(control_set, _MAV_PAYLOAD(msg), len);
#endif
}
