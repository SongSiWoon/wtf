/** @file
 *    @brief MAVLink comm protocol testsuite generated from swarm.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef SWARM_TESTSUITE_H
#define SWARM_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_swarm(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_swarm(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_monitoring(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MONITORING >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_monitoring_t packet_in = {
        963497464,45.0,73.0,101.0,129.0,157.0,185.0,963498920,963499128,269.0,297.0,325.0,19731,27,94,161,228,39,106
    };
    mavlink_monitoring_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.tow = packet_in.tow;
        packet1.pos_x = packet_in.pos_x;
        packet1.pos_y = packet_in.pos_y;
        packet1.pos_z = packet_in.pos_z;
        packet1.head = packet_in.head;
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.status1 = packet_in.status1;
        packet1.status2 = packet_in.status2;
        packet1.rtk_n = packet_in.rtk_n;
        packet1.rtk_e = packet_in.rtk_e;
        packet1.rtk_d = packet_in.rtk_d;
        packet1.swarm_id = packet_in.swarm_id;
        packet1.rtk_nbase = packet_in.rtk_nbase;
        packet1.rtk_nrover = packet_in.rtk_nrover;
        packet1.battery = packet_in.battery;
        packet1.r = packet_in.r;
        packet1.g = packet_in.g;
        packet1.b = packet_in.b;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MONITORING_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MONITORING_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_monitoring_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_monitoring_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_monitoring_pack(system_id, component_id, &msg , packet1.swarm_id , packet1.tow , packet1.pos_x , packet1.pos_y , packet1.pos_z , packet1.head , packet1.roll , packet1.pitch , packet1.status1 , packet1.status2 , packet1.rtk_nbase , packet1.rtk_nrover , packet1.battery , packet1.r , packet1.g , packet1.b , packet1.rtk_n , packet1.rtk_e , packet1.rtk_d );
    mavlink_msg_monitoring_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_monitoring_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.swarm_id , packet1.tow , packet1.pos_x , packet1.pos_y , packet1.pos_z , packet1.head , packet1.roll , packet1.pitch , packet1.status1 , packet1.status2 , packet1.rtk_nbase , packet1.rtk_nrover , packet1.battery , packet1.r , packet1.g , packet1.b , packet1.rtk_n , packet1.rtk_e , packet1.rtk_d );
    mavlink_msg_monitoring_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_monitoring_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_monitoring_send(MAVLINK_COMM_1 , packet1.swarm_id , packet1.tow , packet1.pos_x , packet1.pos_y , packet1.pos_z , packet1.head , packet1.roll , packet1.pitch , packet1.status1 , packet1.status2 , packet1.rtk_nbase , packet1.rtk_nrover , packet1.battery , packet1.r , packet1.g , packet1.b , packet1.rtk_n , packet1.rtk_e , packet1.rtk_d );
    mavlink_msg_monitoring_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MONITORING") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MONITORING) != NULL);
#endif
}

static void mavlink_test_scenario_cmd(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SCENARIO_CMD >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_scenario_cmd_t packet_in = {
        17.0,45.0,73.0,963498088,53,120,{ 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218 }
    };
    mavlink_scenario_cmd_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param1 = packet_in.param1;
        packet1.param2 = packet_in.param2;
        packet1.param3 = packet_in.param3;
        packet1.param4 = packet_in.param4;
        packet1.target_system = packet_in.target_system;
        packet1.cmd = packet_in.cmd;
        
        mav_array_memcpy(packet1.param5, packet_in.param5, sizeof(uint8_t)*32);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SCENARIO_CMD_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SCENARIO_CMD_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scenario_cmd_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_scenario_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scenario_cmd_pack(system_id, component_id, &msg , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.target_system , packet1.cmd , packet1.param5 );
    mavlink_msg_scenario_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scenario_cmd_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.target_system , packet1.cmd , packet1.param5 );
    mavlink_msg_scenario_cmd_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_scenario_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_scenario_cmd_send(MAVLINK_COMM_1 , packet1.param1 , packet1.param2 , packet1.param3 , packet1.param4 , packet1.target_system , packet1.cmd , packet1.param5 );
    mavlink_msg_scenario_cmd_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("SCENARIO_CMD") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_SCENARIO_CMD) != NULL);
#endif
}

static void mavlink_test_led_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_LED_CONTROL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_led_control_t packet_in = {
        5,72,139,206,17
    };
    mavlink_led_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.type = packet_in.type;
        packet1.r = packet_in.r;
        packet1.g = packet_in.g;
        packet1.b = packet_in.b;
        packet1.brightness = packet_in.brightness;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_LED_CONTROL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_LED_CONTROL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_led_control_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_led_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_led_control_pack(system_id, component_id, &msg , packet1.type , packet1.r , packet1.g , packet1.b , packet1.brightness );
    mavlink_msg_led_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_led_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.type , packet1.r , packet1.g , packet1.b , packet1.brightness );
    mavlink_msg_led_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_led_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_led_control_send(MAVLINK_COMM_1 , packet1.type , packet1.r , packet1.g , packet1.b , packet1.brightness );
    mavlink_msg_led_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("LED_CONTROL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_LED_CONTROL) != NULL);
#endif
}

static void mavlink_test_swarm(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_monitoring(system_id, component_id, last_msg);
    mavlink_test_scenario_cmd(system_id, component_id, last_msg);
    mavlink_test_led_control(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // SWARM_TESTSUITE_H
