/** @file
 *    @brief MAVLink comm protocol testsuite generated from underpan.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef UNDERPAN_TESTSUITE_H
#define UNDERPAN_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_underpan(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_underpan(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_controller(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_Controller >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_controller_t packet_in = {
        17235,17339,17443,17547,29
    };
    mavlink_controller_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.left_x = packet_in.left_x;
        packet1.left_y = packet_in.left_y;
        packet1.right_x = packet_in.right_x;
        packet1.right_y = packet_in.right_y;
        packet1.buttons = packet_in.buttons;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_Controller_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_Controller_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_controller_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_controller_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_controller_pack(system_id, component_id, &msg , packet1.left_x , packet1.left_y , packet1.right_x , packet1.right_y , packet1.buttons );
    mavlink_msg_controller_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_controller_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.left_x , packet1.left_y , packet1.right_x , packet1.right_y , packet1.buttons );
    mavlink_msg_controller_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_controller_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_controller_send(MAVLINK_COMM_1 , packet1.left_x , packet1.left_y , packet1.right_x , packet1.right_y , packet1.buttons );
    mavlink_msg_controller_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("Controller") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_Controller) != NULL);
#endif
}

static void mavlink_test_control_set(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CONTROL_SET >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_control_set_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0
    };
    mavlink_control_set_t packet1, packet2;
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
           memset(MAVLINK_MSG_ID_CONTROL_SET_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CONTROL_SET_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_control_set_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_control_set_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_control_set_pack(system_id, component_id, &msg , packet1.vx_set , packet1.vy_set , packet1.vw_set , packet1.x_set , packet1.y_set , packet1.w_set );
    mavlink_msg_control_set_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_control_set_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.vx_set , packet1.vy_set , packet1.vw_set , packet1.x_set , packet1.y_set , packet1.w_set );
    mavlink_msg_control_set_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_control_set_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_control_set_send(MAVLINK_COMM_1 , packet1.vx_set , packet1.vy_set , packet1.vw_set , packet1.x_set , packet1.y_set , packet1.w_set );
    mavlink_msg_control_set_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("CONTROL_SET") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_CONTROL_SET) != NULL);
#endif
}

static void mavlink_test_speed_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SPEED_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_speed_status_t packet_in = {
        17.0,45.0,73.0
    };
    mavlink_speed_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.vx_state = packet_in.vx_state;
        packet1.vy_state = packet_in.vy_state;
        packet1.vw_state = packet_in.vw_state;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SPEED_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SPEED_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speed_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_speed_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speed_status_pack(system_id, component_id, &msg , packet1.vx_state , packet1.vy_state , packet1.vw_state );
    mavlink_msg_speed_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speed_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.vx_state , packet1.vy_state , packet1.vw_state );
    mavlink_msg_speed_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_speed_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speed_status_send(MAVLINK_COMM_1 , packet1.vx_state , packet1.vy_state , packet1.vw_state );
    mavlink_msg_speed_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SPEED_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SPEED_STATUS) != NULL);
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
        17.0,45.0,73.0,101.0,129.0,157.0
    };
    mavlink_posture_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.pos_x = packet_in.pos_x;
        packet1.pos_y = packet_in.pos_y;
        packet1.zangle = packet_in.zangle;
        packet1.xangle = packet_in.xangle;
        packet1.yangle = packet_in.yangle;
        packet1.w_z = packet_in.w_z;
        
        
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
    mavlink_msg_posture_pack(system_id, component_id, &msg , packet1.pos_x , packet1.pos_y , packet1.zangle , packet1.xangle , packet1.yangle , packet1.w_z );
    mavlink_msg_posture_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_posture_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.pos_x , packet1.pos_y , packet1.zangle , packet1.xangle , packet1.yangle , packet1.w_z );
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
    mavlink_msg_posture_send(MAVLINK_COMM_1 , packet1.pos_x , packet1.pos_y , packet1.zangle , packet1.xangle , packet1.yangle , packet1.w_z );
    mavlink_msg_posture_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("POSTURE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_POSTURE) != NULL);
#endif
}

static void mavlink_test_underpan(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_controller(system_id, component_id, last_msg);
    mavlink_test_control_set(system_id, component_id, last_msg);
    mavlink_test_speed_status(system_id, component_id, last_msg);
    mavlink_test_posture(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // UNDERPAN_TESTSUITE_H
