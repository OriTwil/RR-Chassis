/** @file
 *    @brief MAVLink comm protocol testsuite generated from chassis.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef CHASSIS_TESTSUITE_H
#define CHASSIS_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_chassis(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_chassis(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CONTROL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_control_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0
    };
    mavlink_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.vx_set = packet_in.vx_set;
        packet1.vy_set = packet_in.vy_set;
        packet1.vw_set = packet_in.vw_set;
        packet1.x_set = packet_in.x_set;
        packet1.y_set = packet_in.y_set;
        packet1.w_set = packet_in.w_set;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CONTROL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CONTROL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_control_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_control_pack(system_id, component_id, &msg , packet1.vx_set , packet1.vy_set , packet1.vw_set , packet1.x_set , packet1.y_set , packet1.w_set );
    mavlink_msg_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.vx_set , packet1.vy_set , packet1.vw_set , packet1.x_set , packet1.y_set , packet1.w_set );
    mavlink_msg_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_control_send(MAVLINK_COMM_1 , packet1.vx_set , packet1.vy_set , packet1.vw_set , packet1.x_set , packet1.y_set , packet1.w_set );
    mavlink_msg_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CONTROL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CONTROL) != NULL);
#endif
}

static void mavlink_test_posture(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_POSTURE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_posture_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,963498712
    };
    mavlink_posture_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.pos_x = packet_in.pos_x;
        packet1.pos_y = packet_in.pos_y;
        packet1.zangle = packet_in.zangle;
        packet1.xangle = packet_in.xangle;
        packet1.yangle = packet_in.yangle;
        packet1.w_z = packet_in.w_z;
        packet1.point = packet_in.point;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_POSTURE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_POSTURE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_posture_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_posture_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_posture_pack(system_id, component_id, &msg , packet1.pos_x , packet1.pos_y , packet1.zangle , packet1.xangle , packet1.yangle , packet1.w_z , packet1.point );
    mavlink_msg_posture_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_posture_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.pos_x , packet1.pos_y , packet1.zangle , packet1.xangle , packet1.yangle , packet1.w_z , packet1.point );
    mavlink_msg_posture_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_posture_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_posture_send(MAVLINK_COMM_1 , packet1.pos_x , packet1.pos_y , packet1.zangle , packet1.xangle , packet1.yangle , packet1.w_z , packet1.point );
    mavlink_msg_posture_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("POSTURE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_POSTURE) != NULL);
#endif
}

static void mavlink_test_chassis_to_upper(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CHASSIS_TO_UPPER >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_chassis_to_upper_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,963498712
    };
    mavlink_chassis_to_upper_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.pos_x = packet_in.pos_x;
        packet1.pos_y = packet_in.pos_y;
        packet1.zangle = packet_in.zangle;
        packet1.xangle = packet_in.xangle;
        packet1.yangle = packet_in.yangle;
        packet1.w_z = packet_in.w_z;
        packet1.point = packet_in.point;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CHASSIS_TO_UPPER_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CHASSIS_TO_UPPER_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chassis_to_upper_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_chassis_to_upper_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chassis_to_upper_pack(system_id, component_id, &msg , packet1.pos_x , packet1.pos_y , packet1.zangle , packet1.xangle , packet1.yangle , packet1.w_z , packet1.point );
    mavlink_msg_chassis_to_upper_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chassis_to_upper_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.pos_x , packet1.pos_y , packet1.zangle , packet1.xangle , packet1.yangle , packet1.w_z , packet1.point );
    mavlink_msg_chassis_to_upper_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_chassis_to_upper_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_chassis_to_upper_send(MAVLINK_COMM_1 , packet1.pos_x , packet1.pos_y , packet1.zangle , packet1.xangle , packet1.yangle , packet1.w_z , packet1.point );
    mavlink_msg_chassis_to_upper_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CHASSIS_TO_UPPER") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CHASSIS_TO_UPPER) != NULL);
#endif
}

static void mavlink_test_joystick_air(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_JOYSTICK_AIR >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_joystick_air_t packet_in = {
        963497464,{ 45.0, 46.0 },{ 101.0, 102.0 },{ 18275, 18276 },77
    };
    mavlink_joystick_air_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.buttons = packet_in.buttons;
        packet1.switchs = packet_in.switchs;
        
        mav_array_memcpy(packet1.joystickL, packet_in.joystickL, sizeof(float)*2);
        mav_array_memcpy(packet1.joystickR, packet_in.joystickR, sizeof(float)*2);
        mav_array_memcpy(packet1.knobs, packet_in.knobs, sizeof(int16_t)*2);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_JOYSTICK_AIR_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_JOYSTICK_AIR_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_joystick_air_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_joystick_air_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_joystick_air_pack(system_id, component_id, &msg , packet1.buttons , packet1.knobs , packet1.joystickL , packet1.joystickR , packet1.switchs );
    mavlink_msg_joystick_air_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_joystick_air_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.buttons , packet1.knobs , packet1.joystickL , packet1.joystickR , packet1.switchs );
    mavlink_msg_joystick_air_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_joystick_air_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_joystick_air_send(MAVLINK_COMM_1 , packet1.buttons , packet1.knobs , packet1.joystickL , packet1.joystickR , packet1.switchs );
    mavlink_msg_joystick_air_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("JOYSTICK_AIR") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_JOYSTICK_AIR) != NULL);
#endif
}

static void mavlink_test_joystick_air_led(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_JOYSTICK_AIR_LED >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_joystick_air_led_t packet_in = {
        17.0,45.0,73.0,101.0,18067
    };
    mavlink_joystick_air_led_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.r = packet_in.r;
        packet1.g = packet_in.g;
        packet1.b = packet_in.b;
        packet1.lightness = packet_in.lightness;
        packet1.duration = packet_in.duration;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_JOYSTICK_AIR_LED_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_JOYSTICK_AIR_LED_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_joystick_air_led_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_joystick_air_led_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_joystick_air_led_pack(system_id, component_id, &msg , packet1.r , packet1.g , packet1.b , packet1.lightness , packet1.duration );
    mavlink_msg_joystick_air_led_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_joystick_air_led_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.r , packet1.g , packet1.b , packet1.lightness , packet1.duration );
    mavlink_msg_joystick_air_led_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_joystick_air_led_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_joystick_air_led_send(MAVLINK_COMM_1 , packet1.r , packet1.g , packet1.b , packet1.lightness , packet1.duration );
    mavlink_msg_joystick_air_led_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("JOYSTICK_AIR_LED") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_JOYSTICK_AIR_LED) != NULL);
#endif
}

static void mavlink_test_joystick_air_dashboard_set_title(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_JOYSTICK_AIR_DASHBOARD_SET_TITLE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_joystick_air_dashboard_set_title_t packet_in = {
        5,"BCDEFGHIJKLMNOPQRST"
    };
    mavlink_joystick_air_dashboard_set_title_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.id = packet_in.id;
        
        mav_array_memcpy(packet1.title, packet_in.title, sizeof(char)*20);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_JOYSTICK_AIR_DASHBOARD_SET_TITLE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_JOYSTICK_AIR_DASHBOARD_SET_TITLE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_joystick_air_dashboard_set_title_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_joystick_air_dashboard_set_title_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_joystick_air_dashboard_set_title_pack(system_id, component_id, &msg , packet1.id , packet1.title );
    mavlink_msg_joystick_air_dashboard_set_title_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_joystick_air_dashboard_set_title_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.id , packet1.title );
    mavlink_msg_joystick_air_dashboard_set_title_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_joystick_air_dashboard_set_title_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_joystick_air_dashboard_set_title_send(MAVLINK_COMM_1 , packet1.id , packet1.title );
    mavlink_msg_joystick_air_dashboard_set_title_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("JOYSTICK_AIR_DASHBOARD_SET_TITLE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_JOYSTICK_AIR_DASHBOARD_SET_TITLE) != NULL);
#endif
}

static void mavlink_test_joystick_air_dashboard_set_msg(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_JOYSTICK_AIR_DASHBOARD_SET_MSG >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_joystick_air_dashboard_set_msg_t packet_in = {
        5,"BCDEFGHIJKLMNOPQRST"
    };
    mavlink_joystick_air_dashboard_set_msg_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.id = packet_in.id;
        
        mav_array_memcpy(packet1.message, packet_in.message, sizeof(char)*20);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_JOYSTICK_AIR_DASHBOARD_SET_MSG_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_JOYSTICK_AIR_DASHBOARD_SET_MSG_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_joystick_air_dashboard_set_msg_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_joystick_air_dashboard_set_msg_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_joystick_air_dashboard_set_msg_pack(system_id, component_id, &msg , packet1.id , packet1.message );
    mavlink_msg_joystick_air_dashboard_set_msg_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_joystick_air_dashboard_set_msg_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.id , packet1.message );
    mavlink_msg_joystick_air_dashboard_set_msg_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_joystick_air_dashboard_set_msg_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_joystick_air_dashboard_set_msg_send(MAVLINK_COMM_1 , packet1.id , packet1.message );
    mavlink_msg_joystick_air_dashboard_set_msg_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("JOYSTICK_AIR_DASHBOARD_SET_MSG") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_JOYSTICK_AIR_DASHBOARD_SET_MSG) != NULL);
#endif
}

static void mavlink_test_joystick_air_dashboard_del(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_JOYSTICK_AIR_DASHBOARD_DEL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_joystick_air_dashboard_del_t packet_in = {
        5
    };
    mavlink_joystick_air_dashboard_del_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.id = packet_in.id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_JOYSTICK_AIR_DASHBOARD_DEL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_JOYSTICK_AIR_DASHBOARD_DEL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_joystick_air_dashboard_del_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_joystick_air_dashboard_del_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_joystick_air_dashboard_del_pack(system_id, component_id, &msg , packet1.id );
    mavlink_msg_joystick_air_dashboard_del_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_joystick_air_dashboard_del_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.id );
    mavlink_msg_joystick_air_dashboard_del_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_joystick_air_dashboard_del_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_joystick_air_dashboard_del_send(MAVLINK_COMM_1 , packet1.id );
    mavlink_msg_joystick_air_dashboard_del_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("JOYSTICK_AIR_DASHBOARD_DEL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_JOYSTICK_AIR_DASHBOARD_DEL) != NULL);
#endif
}

static void mavlink_test_chassis(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_control(system_id, component_id, last_msg);
    mavlink_test_posture(system_id, component_id, last_msg);
    mavlink_test_chassis_to_upper(system_id, component_id, last_msg);
    mavlink_test_joystick_air(system_id, component_id, last_msg);
    mavlink_test_joystick_air_led(system_id, component_id, last_msg);
    mavlink_test_joystick_air_dashboard_set_title(system_id, component_id, last_msg);
    mavlink_test_joystick_air_dashboard_set_msg(system_id, component_id, last_msg);
    mavlink_test_joystick_air_dashboard_del(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // CHASSIS_TESTSUITE_H
