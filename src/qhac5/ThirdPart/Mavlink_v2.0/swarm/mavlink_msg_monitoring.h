#pragma once
// MESSAGE MONITORING PACKING

#define MAVLINK_MSG_ID_MONITORING 180


typedef struct __mavlink_monitoring_t {
 uint32_t tow; /*<  Time Of Week*/
 float pos_x; /*<  current position x*/
 float pos_y; /*<  current position y*/
 float pos_z; /*<  current position z*/
 float head; /*<  current heading*/
 float roll; /*<  Roll*/
 float pitch; /*<  Pitch*/
 uint32_t status1; /*<  status #1*/
 uint32_t status2; /*<  status #2 (RESERVED)*/
 float rtk_n; /*<  Rtk Baseline North coordinate*/
 float rtk_e; /*<  Rtk Baseline East coordinate*/
 float rtk_d; /*<  Rtk Baseline Down coordinate*/
 uint16_t swarm_id; /*<  Swarm id for GCS*/
 uint8_t rtk_nbase; /*<  the number of base satellite*/
 uint8_t rtk_nrover; /*<  the number of rover satellite*/
 uint8_t battery; /*<  Battery (%)*/
 uint8_t r; /*<  red value (0~256)*/
 uint8_t g; /*<  green value (0~256)*/
 uint8_t b; /*<  blue value (0~256)*/
} mavlink_monitoring_t;

#define MAVLINK_MSG_ID_MONITORING_LEN 56
#define MAVLINK_MSG_ID_MONITORING_MIN_LEN 56
#define MAVLINK_MSG_ID_180_LEN 56
#define MAVLINK_MSG_ID_180_MIN_LEN 56

#define MAVLINK_MSG_ID_MONITORING_CRC 206
#define MAVLINK_MSG_ID_180_CRC 206



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MONITORING { \
    180, \
    "MONITORING", \
    19, \
    {  { "swarm_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 48, offsetof(mavlink_monitoring_t, swarm_id) }, \
         { "tow", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_monitoring_t, tow) }, \
         { "pos_x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_monitoring_t, pos_x) }, \
         { "pos_y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_monitoring_t, pos_y) }, \
         { "pos_z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_monitoring_t, pos_z) }, \
         { "head", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_monitoring_t, head) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_monitoring_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_monitoring_t, pitch) }, \
         { "status1", NULL, MAVLINK_TYPE_UINT32_T, 0, 28, offsetof(mavlink_monitoring_t, status1) }, \
         { "status2", NULL, MAVLINK_TYPE_UINT32_T, 0, 32, offsetof(mavlink_monitoring_t, status2) }, \
         { "rtk_nbase", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_monitoring_t, rtk_nbase) }, \
         { "rtk_nrover", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_monitoring_t, rtk_nrover) }, \
         { "battery", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_monitoring_t, battery) }, \
         { "r", NULL, MAVLINK_TYPE_UINT8_T, 0, 53, offsetof(mavlink_monitoring_t, r) }, \
         { "g", NULL, MAVLINK_TYPE_UINT8_T, 0, 54, offsetof(mavlink_monitoring_t, g) }, \
         { "b", NULL, MAVLINK_TYPE_UINT8_T, 0, 55, offsetof(mavlink_monitoring_t, b) }, \
         { "rtk_n", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_monitoring_t, rtk_n) }, \
         { "rtk_e", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_monitoring_t, rtk_e) }, \
         { "rtk_d", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_monitoring_t, rtk_d) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MONITORING { \
    "MONITORING", \
    19, \
    {  { "swarm_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 48, offsetof(mavlink_monitoring_t, swarm_id) }, \
         { "tow", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_monitoring_t, tow) }, \
         { "pos_x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_monitoring_t, pos_x) }, \
         { "pos_y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_monitoring_t, pos_y) }, \
         { "pos_z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_monitoring_t, pos_z) }, \
         { "head", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_monitoring_t, head) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_monitoring_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_monitoring_t, pitch) }, \
         { "status1", NULL, MAVLINK_TYPE_UINT32_T, 0, 28, offsetof(mavlink_monitoring_t, status1) }, \
         { "status2", NULL, MAVLINK_TYPE_UINT32_T, 0, 32, offsetof(mavlink_monitoring_t, status2) }, \
         { "rtk_nbase", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_monitoring_t, rtk_nbase) }, \
         { "rtk_nrover", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_monitoring_t, rtk_nrover) }, \
         { "battery", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_monitoring_t, battery) }, \
         { "r", NULL, MAVLINK_TYPE_UINT8_T, 0, 53, offsetof(mavlink_monitoring_t, r) }, \
         { "g", NULL, MAVLINK_TYPE_UINT8_T, 0, 54, offsetof(mavlink_monitoring_t, g) }, \
         { "b", NULL, MAVLINK_TYPE_UINT8_T, 0, 55, offsetof(mavlink_monitoring_t, b) }, \
         { "rtk_n", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_monitoring_t, rtk_n) }, \
         { "rtk_e", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_monitoring_t, rtk_e) }, \
         { "rtk_d", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_monitoring_t, rtk_d) }, \
         } \
}
#endif

/**
 * @brief Pack a monitoring message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param swarm_id  Swarm id for GCS
 * @param tow  Time Of Week
 * @param pos_x  current position x
 * @param pos_y  current position y
 * @param pos_z  current position z
 * @param head  current heading
 * @param roll  Roll
 * @param pitch  Pitch
 * @param status1  status #1
 * @param status2  status #2 (RESERVED)
 * @param rtk_nbase  the number of base satellite
 * @param rtk_nrover  the number of rover satellite
 * @param battery  Battery (%)
 * @param r  red value (0~256)
 * @param g  green value (0~256)
 * @param b  blue value (0~256)
 * @param rtk_n  Rtk Baseline North coordinate
 * @param rtk_e  Rtk Baseline East coordinate
 * @param rtk_d  Rtk Baseline Down coordinate
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_monitoring_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t swarm_id, uint32_t tow, float pos_x, float pos_y, float pos_z, float head, float roll, float pitch, uint32_t status1, uint32_t status2, uint8_t rtk_nbase, uint8_t rtk_nrover, uint8_t battery, uint8_t r, uint8_t g, uint8_t b, float rtk_n, float rtk_e, float rtk_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MONITORING_LEN];
    _mav_put_uint32_t(buf, 0, tow);
    _mav_put_float(buf, 4, pos_x);
    _mav_put_float(buf, 8, pos_y);
    _mav_put_float(buf, 12, pos_z);
    _mav_put_float(buf, 16, head);
    _mav_put_float(buf, 20, roll);
    _mav_put_float(buf, 24, pitch);
    _mav_put_uint32_t(buf, 28, status1);
    _mav_put_uint32_t(buf, 32, status2);
    _mav_put_float(buf, 36, rtk_n);
    _mav_put_float(buf, 40, rtk_e);
    _mav_put_float(buf, 44, rtk_d);
    _mav_put_uint16_t(buf, 48, swarm_id);
    _mav_put_uint8_t(buf, 50, rtk_nbase);
    _mav_put_uint8_t(buf, 51, rtk_nrover);
    _mav_put_uint8_t(buf, 52, battery);
    _mav_put_uint8_t(buf, 53, r);
    _mav_put_uint8_t(buf, 54, g);
    _mav_put_uint8_t(buf, 55, b);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MONITORING_LEN);
#else
    mavlink_monitoring_t packet;
    packet.tow = tow;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.pos_z = pos_z;
    packet.head = head;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.status1 = status1;
    packet.status2 = status2;
    packet.rtk_n = rtk_n;
    packet.rtk_e = rtk_e;
    packet.rtk_d = rtk_d;
    packet.swarm_id = swarm_id;
    packet.rtk_nbase = rtk_nbase;
    packet.rtk_nrover = rtk_nrover;
    packet.battery = battery;
    packet.r = r;
    packet.g = g;
    packet.b = b;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MONITORING_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MONITORING;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MONITORING_MIN_LEN, MAVLINK_MSG_ID_MONITORING_LEN, MAVLINK_MSG_ID_MONITORING_CRC);
}

/**
 * @brief Pack a monitoring message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 *
 * @param swarm_id  Swarm id for GCS
 * @param tow  Time Of Week
 * @param pos_x  current position x
 * @param pos_y  current position y
 * @param pos_z  current position z
 * @param head  current heading
 * @param roll  Roll
 * @param pitch  Pitch
 * @param status1  status #1
 * @param status2  status #2 (RESERVED)
 * @param rtk_nbase  the number of base satellite
 * @param rtk_nrover  the number of rover satellite
 * @param battery  Battery (%)
 * @param r  red value (0~256)
 * @param g  green value (0~256)
 * @param b  blue value (0~256)
 * @param rtk_n  Rtk Baseline North coordinate
 * @param rtk_e  Rtk Baseline East coordinate
 * @param rtk_d  Rtk Baseline Down coordinate
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_monitoring_pack_status(uint8_t system_id, uint8_t component_id, mavlink_status_t *_status, mavlink_message_t* msg,
                               uint16_t swarm_id, uint32_t tow, float pos_x, float pos_y, float pos_z, float head, float roll, float pitch, uint32_t status1, uint32_t status2, uint8_t rtk_nbase, uint8_t rtk_nrover, uint8_t battery, uint8_t r, uint8_t g, uint8_t b, float rtk_n, float rtk_e, float rtk_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MONITORING_LEN];
    _mav_put_uint32_t(buf, 0, tow);
    _mav_put_float(buf, 4, pos_x);
    _mav_put_float(buf, 8, pos_y);
    _mav_put_float(buf, 12, pos_z);
    _mav_put_float(buf, 16, head);
    _mav_put_float(buf, 20, roll);
    _mav_put_float(buf, 24, pitch);
    _mav_put_uint32_t(buf, 28, status1);
    _mav_put_uint32_t(buf, 32, status2);
    _mav_put_float(buf, 36, rtk_n);
    _mav_put_float(buf, 40, rtk_e);
    _mav_put_float(buf, 44, rtk_d);
    _mav_put_uint16_t(buf, 48, swarm_id);
    _mav_put_uint8_t(buf, 50, rtk_nbase);
    _mav_put_uint8_t(buf, 51, rtk_nrover);
    _mav_put_uint8_t(buf, 52, battery);
    _mav_put_uint8_t(buf, 53, r);
    _mav_put_uint8_t(buf, 54, g);
    _mav_put_uint8_t(buf, 55, b);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MONITORING_LEN);
#else
    mavlink_monitoring_t packet;
    packet.tow = tow;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.pos_z = pos_z;
    packet.head = head;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.status1 = status1;
    packet.status2 = status2;
    packet.rtk_n = rtk_n;
    packet.rtk_e = rtk_e;
    packet.rtk_d = rtk_d;
    packet.swarm_id = swarm_id;
    packet.rtk_nbase = rtk_nbase;
    packet.rtk_nrover = rtk_nrover;
    packet.battery = battery;
    packet.r = r;
    packet.g = g;
    packet.b = b;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MONITORING_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MONITORING;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MONITORING_MIN_LEN, MAVLINK_MSG_ID_MONITORING_LEN, MAVLINK_MSG_ID_MONITORING_CRC);
#else
    return mavlink_finalize_message_buffer(msg, system_id, component_id, _status, MAVLINK_MSG_ID_MONITORING_MIN_LEN, MAVLINK_MSG_ID_MONITORING_LEN);
#endif
}

/**
 * @brief Pack a monitoring message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param swarm_id  Swarm id for GCS
 * @param tow  Time Of Week
 * @param pos_x  current position x
 * @param pos_y  current position y
 * @param pos_z  current position z
 * @param head  current heading
 * @param roll  Roll
 * @param pitch  Pitch
 * @param status1  status #1
 * @param status2  status #2 (RESERVED)
 * @param rtk_nbase  the number of base satellite
 * @param rtk_nrover  the number of rover satellite
 * @param battery  Battery (%)
 * @param r  red value (0~256)
 * @param g  green value (0~256)
 * @param b  blue value (0~256)
 * @param rtk_n  Rtk Baseline North coordinate
 * @param rtk_e  Rtk Baseline East coordinate
 * @param rtk_d  Rtk Baseline Down coordinate
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_monitoring_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t swarm_id,uint32_t tow,float pos_x,float pos_y,float pos_z,float head,float roll,float pitch,uint32_t status1,uint32_t status2,uint8_t rtk_nbase,uint8_t rtk_nrover,uint8_t battery,uint8_t r,uint8_t g,uint8_t b,float rtk_n,float rtk_e,float rtk_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MONITORING_LEN];
    _mav_put_uint32_t(buf, 0, tow);
    _mav_put_float(buf, 4, pos_x);
    _mav_put_float(buf, 8, pos_y);
    _mav_put_float(buf, 12, pos_z);
    _mav_put_float(buf, 16, head);
    _mav_put_float(buf, 20, roll);
    _mav_put_float(buf, 24, pitch);
    _mav_put_uint32_t(buf, 28, status1);
    _mav_put_uint32_t(buf, 32, status2);
    _mav_put_float(buf, 36, rtk_n);
    _mav_put_float(buf, 40, rtk_e);
    _mav_put_float(buf, 44, rtk_d);
    _mav_put_uint16_t(buf, 48, swarm_id);
    _mav_put_uint8_t(buf, 50, rtk_nbase);
    _mav_put_uint8_t(buf, 51, rtk_nrover);
    _mav_put_uint8_t(buf, 52, battery);
    _mav_put_uint8_t(buf, 53, r);
    _mav_put_uint8_t(buf, 54, g);
    _mav_put_uint8_t(buf, 55, b);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MONITORING_LEN);
#else
    mavlink_monitoring_t packet;
    packet.tow = tow;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.pos_z = pos_z;
    packet.head = head;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.status1 = status1;
    packet.status2 = status2;
    packet.rtk_n = rtk_n;
    packet.rtk_e = rtk_e;
    packet.rtk_d = rtk_d;
    packet.swarm_id = swarm_id;
    packet.rtk_nbase = rtk_nbase;
    packet.rtk_nrover = rtk_nrover;
    packet.battery = battery;
    packet.r = r;
    packet.g = g;
    packet.b = b;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MONITORING_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MONITORING;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MONITORING_MIN_LEN, MAVLINK_MSG_ID_MONITORING_LEN, MAVLINK_MSG_ID_MONITORING_CRC);
}

/**
 * @brief Encode a monitoring struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param monitoring C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_monitoring_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_monitoring_t* monitoring)
{
    return mavlink_msg_monitoring_pack(system_id, component_id, msg, monitoring->swarm_id, monitoring->tow, monitoring->pos_x, monitoring->pos_y, monitoring->pos_z, monitoring->head, monitoring->roll, monitoring->pitch, monitoring->status1, monitoring->status2, monitoring->rtk_nbase, monitoring->rtk_nrover, monitoring->battery, monitoring->r, monitoring->g, monitoring->b, monitoring->rtk_n, monitoring->rtk_e, monitoring->rtk_d);
}

/**
 * @brief Encode a monitoring struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param monitoring C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_monitoring_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_monitoring_t* monitoring)
{
    return mavlink_msg_monitoring_pack_chan(system_id, component_id, chan, msg, monitoring->swarm_id, monitoring->tow, monitoring->pos_x, monitoring->pos_y, monitoring->pos_z, monitoring->head, monitoring->roll, monitoring->pitch, monitoring->status1, monitoring->status2, monitoring->rtk_nbase, monitoring->rtk_nrover, monitoring->battery, monitoring->r, monitoring->g, monitoring->b, monitoring->rtk_n, monitoring->rtk_e, monitoring->rtk_d);
}

/**
 * @brief Encode a monitoring struct with provided status structure
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param status MAVLink status structure
 * @param msg The MAVLink message to compress the data into
 * @param monitoring C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_monitoring_encode_status(uint8_t system_id, uint8_t component_id, mavlink_status_t* _status, mavlink_message_t* msg, const mavlink_monitoring_t* monitoring)
{
    return mavlink_msg_monitoring_pack_status(system_id, component_id, _status, msg,  monitoring->swarm_id, monitoring->tow, monitoring->pos_x, monitoring->pos_y, monitoring->pos_z, monitoring->head, monitoring->roll, monitoring->pitch, monitoring->status1, monitoring->status2, monitoring->rtk_nbase, monitoring->rtk_nrover, monitoring->battery, monitoring->r, monitoring->g, monitoring->b, monitoring->rtk_n, monitoring->rtk_e, monitoring->rtk_d);
}

/**
 * @brief Send a monitoring message
 * @param chan MAVLink channel to send the message
 *
 * @param swarm_id  Swarm id for GCS
 * @param tow  Time Of Week
 * @param pos_x  current position x
 * @param pos_y  current position y
 * @param pos_z  current position z
 * @param head  current heading
 * @param roll  Roll
 * @param pitch  Pitch
 * @param status1  status #1
 * @param status2  status #2 (RESERVED)
 * @param rtk_nbase  the number of base satellite
 * @param rtk_nrover  the number of rover satellite
 * @param battery  Battery (%)
 * @param r  red value (0~256)
 * @param g  green value (0~256)
 * @param b  blue value (0~256)
 * @param rtk_n  Rtk Baseline North coordinate
 * @param rtk_e  Rtk Baseline East coordinate
 * @param rtk_d  Rtk Baseline Down coordinate
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_monitoring_send(mavlink_channel_t chan, uint16_t swarm_id, uint32_t tow, float pos_x, float pos_y, float pos_z, float head, float roll, float pitch, uint32_t status1, uint32_t status2, uint8_t rtk_nbase, uint8_t rtk_nrover, uint8_t battery, uint8_t r, uint8_t g, uint8_t b, float rtk_n, float rtk_e, float rtk_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MONITORING_LEN];
    _mav_put_uint32_t(buf, 0, tow);
    _mav_put_float(buf, 4, pos_x);
    _mav_put_float(buf, 8, pos_y);
    _mav_put_float(buf, 12, pos_z);
    _mav_put_float(buf, 16, head);
    _mav_put_float(buf, 20, roll);
    _mav_put_float(buf, 24, pitch);
    _mav_put_uint32_t(buf, 28, status1);
    _mav_put_uint32_t(buf, 32, status2);
    _mav_put_float(buf, 36, rtk_n);
    _mav_put_float(buf, 40, rtk_e);
    _mav_put_float(buf, 44, rtk_d);
    _mav_put_uint16_t(buf, 48, swarm_id);
    _mav_put_uint8_t(buf, 50, rtk_nbase);
    _mav_put_uint8_t(buf, 51, rtk_nrover);
    _mav_put_uint8_t(buf, 52, battery);
    _mav_put_uint8_t(buf, 53, r);
    _mav_put_uint8_t(buf, 54, g);
    _mav_put_uint8_t(buf, 55, b);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MONITORING, buf, MAVLINK_MSG_ID_MONITORING_MIN_LEN, MAVLINK_MSG_ID_MONITORING_LEN, MAVLINK_MSG_ID_MONITORING_CRC);
#else
    mavlink_monitoring_t packet;
    packet.tow = tow;
    packet.pos_x = pos_x;
    packet.pos_y = pos_y;
    packet.pos_z = pos_z;
    packet.head = head;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.status1 = status1;
    packet.status2 = status2;
    packet.rtk_n = rtk_n;
    packet.rtk_e = rtk_e;
    packet.rtk_d = rtk_d;
    packet.swarm_id = swarm_id;
    packet.rtk_nbase = rtk_nbase;
    packet.rtk_nrover = rtk_nrover;
    packet.battery = battery;
    packet.r = r;
    packet.g = g;
    packet.b = b;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MONITORING, (const char *)&packet, MAVLINK_MSG_ID_MONITORING_MIN_LEN, MAVLINK_MSG_ID_MONITORING_LEN, MAVLINK_MSG_ID_MONITORING_CRC);
#endif
}

/**
 * @brief Send a monitoring message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_monitoring_send_struct(mavlink_channel_t chan, const mavlink_monitoring_t* monitoring)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_monitoring_send(chan, monitoring->swarm_id, monitoring->tow, monitoring->pos_x, monitoring->pos_y, monitoring->pos_z, monitoring->head, monitoring->roll, monitoring->pitch, monitoring->status1, monitoring->status2, monitoring->rtk_nbase, monitoring->rtk_nrover, monitoring->battery, monitoring->r, monitoring->g, monitoring->b, monitoring->rtk_n, monitoring->rtk_e, monitoring->rtk_d);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MONITORING, (const char *)monitoring, MAVLINK_MSG_ID_MONITORING_MIN_LEN, MAVLINK_MSG_ID_MONITORING_LEN, MAVLINK_MSG_ID_MONITORING_CRC);
#endif
}

#if MAVLINK_MSG_ID_MONITORING_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_monitoring_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t swarm_id, uint32_t tow, float pos_x, float pos_y, float pos_z, float head, float roll, float pitch, uint32_t status1, uint32_t status2, uint8_t rtk_nbase, uint8_t rtk_nrover, uint8_t battery, uint8_t r, uint8_t g, uint8_t b, float rtk_n, float rtk_e, float rtk_d)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint32_t(buf, 0, tow);
    _mav_put_float(buf, 4, pos_x);
    _mav_put_float(buf, 8, pos_y);
    _mav_put_float(buf, 12, pos_z);
    _mav_put_float(buf, 16, head);
    _mav_put_float(buf, 20, roll);
    _mav_put_float(buf, 24, pitch);
    _mav_put_uint32_t(buf, 28, status1);
    _mav_put_uint32_t(buf, 32, status2);
    _mav_put_float(buf, 36, rtk_n);
    _mav_put_float(buf, 40, rtk_e);
    _mav_put_float(buf, 44, rtk_d);
    _mav_put_uint16_t(buf, 48, swarm_id);
    _mav_put_uint8_t(buf, 50, rtk_nbase);
    _mav_put_uint8_t(buf, 51, rtk_nrover);
    _mav_put_uint8_t(buf, 52, battery);
    _mav_put_uint8_t(buf, 53, r);
    _mav_put_uint8_t(buf, 54, g);
    _mav_put_uint8_t(buf, 55, b);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MONITORING, buf, MAVLINK_MSG_ID_MONITORING_MIN_LEN, MAVLINK_MSG_ID_MONITORING_LEN, MAVLINK_MSG_ID_MONITORING_CRC);
#else
    mavlink_monitoring_t *packet = (mavlink_monitoring_t *)msgbuf;
    packet->tow = tow;
    packet->pos_x = pos_x;
    packet->pos_y = pos_y;
    packet->pos_z = pos_z;
    packet->head = head;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->status1 = status1;
    packet->status2 = status2;
    packet->rtk_n = rtk_n;
    packet->rtk_e = rtk_e;
    packet->rtk_d = rtk_d;
    packet->swarm_id = swarm_id;
    packet->rtk_nbase = rtk_nbase;
    packet->rtk_nrover = rtk_nrover;
    packet->battery = battery;
    packet->r = r;
    packet->g = g;
    packet->b = b;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MONITORING, (const char *)packet, MAVLINK_MSG_ID_MONITORING_MIN_LEN, MAVLINK_MSG_ID_MONITORING_LEN, MAVLINK_MSG_ID_MONITORING_CRC);
#endif
}
#endif

#endif

// MESSAGE MONITORING UNPACKING


/**
 * @brief Get field swarm_id from monitoring message
 *
 * @return  Swarm id for GCS
 */
static inline uint16_t mavlink_msg_monitoring_get_swarm_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  48);
}

/**
 * @brief Get field tow from monitoring message
 *
 * @return  Time Of Week
 */
static inline uint32_t mavlink_msg_monitoring_get_tow(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field pos_x from monitoring message
 *
 * @return  current position x
 */
static inline float mavlink_msg_monitoring_get_pos_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field pos_y from monitoring message
 *
 * @return  current position y
 */
static inline float mavlink_msg_monitoring_get_pos_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pos_z from monitoring message
 *
 * @return  current position z
 */
static inline float mavlink_msg_monitoring_get_pos_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field head from monitoring message
 *
 * @return  current heading
 */
static inline float mavlink_msg_monitoring_get_head(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field roll from monitoring message
 *
 * @return  Roll
 */
static inline float mavlink_msg_monitoring_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field pitch from monitoring message
 *
 * @return  Pitch
 */
static inline float mavlink_msg_monitoring_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field status1 from monitoring message
 *
 * @return  status #1
 */
static inline uint32_t mavlink_msg_monitoring_get_status1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  28);
}

/**
 * @brief Get field status2 from monitoring message
 *
 * @return  status #2 (RESERVED)
 */
static inline uint32_t mavlink_msg_monitoring_get_status2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  32);
}

/**
 * @brief Get field rtk_nbase from monitoring message
 *
 * @return  the number of base satellite
 */
static inline uint8_t mavlink_msg_monitoring_get_rtk_nbase(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  50);
}

/**
 * @brief Get field rtk_nrover from monitoring message
 *
 * @return  the number of rover satellite
 */
static inline uint8_t mavlink_msg_monitoring_get_rtk_nrover(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  51);
}

/**
 * @brief Get field battery from monitoring message
 *
 * @return  Battery (%)
 */
static inline uint8_t mavlink_msg_monitoring_get_battery(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  52);
}

/**
 * @brief Get field r from monitoring message
 *
 * @return  red value (0~256)
 */
static inline uint8_t mavlink_msg_monitoring_get_r(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  53);
}

/**
 * @brief Get field g from monitoring message
 *
 * @return  green value (0~256)
 */
static inline uint8_t mavlink_msg_monitoring_get_g(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  54);
}

/**
 * @brief Get field b from monitoring message
 *
 * @return  blue value (0~256)
 */
static inline uint8_t mavlink_msg_monitoring_get_b(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  55);
}

/**
 * @brief Get field rtk_n from monitoring message
 *
 * @return  Rtk Baseline North coordinate
 */
static inline float mavlink_msg_monitoring_get_rtk_n(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field rtk_e from monitoring message
 *
 * @return  Rtk Baseline East coordinate
 */
static inline float mavlink_msg_monitoring_get_rtk_e(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field rtk_d from monitoring message
 *
 * @return  Rtk Baseline Down coordinate
 */
static inline float mavlink_msg_monitoring_get_rtk_d(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Decode a monitoring message into a struct
 *
 * @param msg The message to decode
 * @param monitoring C-struct to decode the message contents into
 */
static inline void mavlink_msg_monitoring_decode(const mavlink_message_t* msg, mavlink_monitoring_t* monitoring)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    monitoring->tow = mavlink_msg_monitoring_get_tow(msg);
    monitoring->pos_x = mavlink_msg_monitoring_get_pos_x(msg);
    monitoring->pos_y = mavlink_msg_monitoring_get_pos_y(msg);
    monitoring->pos_z = mavlink_msg_monitoring_get_pos_z(msg);
    monitoring->head = mavlink_msg_monitoring_get_head(msg);
    monitoring->roll = mavlink_msg_monitoring_get_roll(msg);
    monitoring->pitch = mavlink_msg_monitoring_get_pitch(msg);
    monitoring->status1 = mavlink_msg_monitoring_get_status1(msg);
    monitoring->status2 = mavlink_msg_monitoring_get_status2(msg);
    monitoring->rtk_n = mavlink_msg_monitoring_get_rtk_n(msg);
    monitoring->rtk_e = mavlink_msg_monitoring_get_rtk_e(msg);
    monitoring->rtk_d = mavlink_msg_monitoring_get_rtk_d(msg);
    monitoring->swarm_id = mavlink_msg_monitoring_get_swarm_id(msg);
    monitoring->rtk_nbase = mavlink_msg_monitoring_get_rtk_nbase(msg);
    monitoring->rtk_nrover = mavlink_msg_monitoring_get_rtk_nrover(msg);
    monitoring->battery = mavlink_msg_monitoring_get_battery(msg);
    monitoring->r = mavlink_msg_monitoring_get_r(msg);
    monitoring->g = mavlink_msg_monitoring_get_g(msg);
    monitoring->b = mavlink_msg_monitoring_get_b(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MONITORING_LEN? msg->len : MAVLINK_MSG_ID_MONITORING_LEN;
        memset(monitoring, 0, MAVLINK_MSG_ID_MONITORING_LEN);
    memcpy(monitoring, _MAV_PAYLOAD(msg), len);
#endif
}
