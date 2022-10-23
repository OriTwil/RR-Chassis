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




static void mavlink_test_speed_control_set(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SPEED_CONTROL_SET >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_speed_control_set_t packet_in = {
        17.0,45.0,73.0
    };
    mavlink_speed_control_set_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.vx_set = packet_in.vx_set;
        packet1.vy_set = packet_in.vy_set;
        packet1.vw_set = packet_in.vw_set;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SPEED_CONTROL_SET_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SPEED_CONTROL_SET_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speed_control_set_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_speed_control_set_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speed_control_set_pack(system_id, component_id, &msg , packet1.vx_set , packet1.vy_set , packet1.vw_set );
    mavlink_msg_speed_control_set_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speed_control_set_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.vx_set , packet1.vy_set , packet1.vw_set );
    mavlink_msg_speed_control_set_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_speed_control_set_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speed_control_set_send(MAVLINK_COMM_1 , packet1.vx_set , packet1.vy_set , packet1.vw_set );
    mavlink_msg_speed_control_set_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SPEED_CONTROL_SET") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SPEED_CONTROL_SET) != NULL);
#endif
}

static void mavlink_test_speed_control_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SPEED_CONTROL_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_speed_control_status_t packet_in = {
        17.0,45.0,73.0
    };
    mavlink_speed_control_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.vx_state = packet_in.vx_state;
        packet1.vy_state = packet_in.vy_state;
        packet1.vw_state = packet_in.vw_state;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SPEED_CONTROL_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speed_control_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_speed_control_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speed_control_status_pack(system_id, component_id, &msg , packet1.vx_state , packet1.vy_state , packet1.vw_state );
    mavlink_msg_speed_control_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speed_control_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.vx_state , packet1.vy_state , packet1.vw_state );
    mavlink_msg_speed_control_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_speed_control_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_speed_control_status_send(MAVLINK_COMM_1 , packet1.vx_state , packet1.vy_state , packet1.vw_state );
    mavlink_msg_speed_control_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SPEED_CONTROL_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SPEED_CONTROL_STATUS) != NULL);
#endif
}

static void mavlink_test_underpan(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_speed_control_set(system_id, component_id, last_msg);
    mavlink_test_speed_control_status(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // UNDERPAN_TESTSUITE_H
