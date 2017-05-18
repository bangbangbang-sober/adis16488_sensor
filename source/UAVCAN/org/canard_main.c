/*
 * Copyright (c) 2016 UAVCAN Team
 *
 * Distributed under the MIT License, available in the file LICENSE.
 *
 * Author: Michael Sierra <sierramichael.a@gmail.com>
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include "canard.h"


#define CLEANUP_STALE_TRANSFERS 2000000
#define CANARD_AVAILABLE_BLOCKS 32


#define TIME_TO_SEND_NODE_STATUS 101000000
#define TIME_TO_SEND_AIRSPEED 51000000000
#define TIME_TO_SEND_MULTI 1000000
#define TIME_TO_SEND_REQUEST 1000000
#define TIME_TO_SEND_NODE_INFO 2000000


// / Arbitrary priority values
static const uint8_t PRIORITY_HIGHEST = 0;
static const uint8_t PRIORITY_HIGH    = 8;
static const uint8_t PRIORITY_MEDIUM  = 16;
static const uint8_t PRIORITY_LOW     = 24;
static const uint8_t PRIORITY_LOWEST  = 31;

uint8_t uavcan_node_id=0x81;

// / Defined for the standard data type uavcan.protocol.NodeStatus
enum node_health
{
    HEALTH_OK       = 0,
    HEALTH_WARNING  = 1,
    HEALTH_ERROR    = 2,
    HEALTH_CRITICAL = 3
};

// / Defined for the standard data type uavcan.protocol.NodeStatus
enum node_mode
{
    MODE_OPERATIONAL     = 0,
    MODE_INITIALIZATION  = 1,
    MODE_MAINTENANCE     = 2,
    MODE_SOFTWARE_UPDATE = 3,
    MODE_OFFLINE         = 7
};

// / Standard data type: uavcan.protocol.NodeStatus
int publish_node_status(enum node_health health, enum node_mode mode,
                        uint16_t vendor_specific_status_code)
{
    static uint64_t startup_timestamp_usec;
    if (startup_timestamp_usec == 0)
    {
        startup_timestamp_usec = clock();
    }

    uint8_t payload[7];

    // Uptime in seconds
    const uint32_t uptime_sec = (clock() - startup_timestamp_usec) / 1000000ULL;
    payload[0] = (uptime_sec >> 0)  & 0xFF;
    payload[1] = (uptime_sec >> 8)  & 0xFF;
    payload[2] = (uptime_sec >> 16) & 0xFF;
    payload[3] = (uptime_sec >> 24) & 0xFF;

    // Health and mode
    payload[4] = ((uint8_t)health << 6) | ((uint8_t)mode << 3);

    // Vendor-specific status code
    payload[5] = (vendor_specific_status_code >> 0) & 0xFF;
    payload[6] = (vendor_specific_status_code >> 8) & 0xFF;

    static const uint16_t data_type_id = 341;
    static uint8_t transfer_id, transfer_type;
		transfer_type = 0x00;
    uint64_t data_type_signature = 0x8899AABBCCDDEEFF;
    return canardBroadcast(uavcan_node_id, data_type_signature,
                           data_type_id, transfer_type, &transfer_id, payload, sizeof(payload));
}

/*
 * Float16 support
 */
uint16_t make_float16(float value)
{
    union fp32
    {
        uint32_t u;
        float f;
    };

    const union fp32 f32infty = { 255U << 23 };
    const union fp32 f16infty = { 31U << 23 };
    const union fp32 magic = { 15U << 23 };
    const uint32_t sign_mask = 0x80000000U;
    const uint32_t round_mask = ~0xFFFU;

    union fp32 in;

    uint16_t out = 0;

    in.f = value;

    uint32_t sign = in.u & sign_mask;
    in.u ^= sign;

    if (in.u >= f32infty.u)
    {
        out = (in.u > f32infty.u) ? 0x7FFFU : 0x7C00U;
    }
    else
    {
        in.u &= round_mask;
        in.f *= magic.f;
        in.u -= round_mask;
        if (in.u > f16infty.u)
        {
            in.u = f16infty.u;
        }
        out = (uint16_t)(in.u >> 13);
    }

    out |= (uint16_t)(sign >> 16);

    return out;
}

// / Standard data type: uavcan.equipment.IMU.gyro
int publish_IMU_gyro(uint16_t x_gyro, uint16_t y_gyro, uint16_t z_gyro)
{
    uint8_t payload[6];
    payload[0] = (x_gyro >> 0) & 0xFF;
    payload[1] = (x_gyro >> 8) & 0xFF;
    payload[2] = (y_gyro >> 0) & 0xFF;
    payload[3] = (y_gyro >> 8) & 0xFF;
    payload[4] = (z_gyro >> 0) & 0xFF;
    payload[5] = (z_gyro >> 8) & 0xFF;

    static const uint16_t data_type_id = 1050;
    static uint8_t transfer_id;
    static uint8_t transfer_type = 0x00;
    uint64_t data_type_signature = 0x8899AABBCCDDEEFF;
    return canardBroadcast(uavcan_node_id, data_type_signature,
                           data_type_id, transfer_type, &transfer_id, payload, sizeof(payload));
}

// / Standard data type: uavcan.equipment.IMU.accl
int publish_IMU_accl(uint16_t x_accl, uint16_t y_accl, uint16_t z_accl)
{
    uint8_t payload[6];
    payload[0] = (x_accl >> 0) & 0xFF;
    payload[1] = (x_accl >> 8) & 0xFF;
    payload[2] = (y_accl >> 0) & 0xFF;
    payload[3] = (y_accl >> 8) & 0xFF;
    payload[4] = (z_accl >> 0) & 0xFF;
    payload[5] = (z_accl >> 8) & 0xFF;

    static const uint16_t data_type_id = 1051;
    static uint8_t transfer_id;
    static uint8_t transfer_type = 0x00;
    uint64_t data_type_signature = 0x8899AABBCCDDEEFF;
    return canardBroadcast(uavcan_node_id, data_type_signature,
                           data_type_id, transfer_type, &transfer_id, payload, sizeof(payload));
}

// / Standard data type: uavcan.equipment.IMU.magn
int publish_IMU_magn(uint16_t x_magn, uint16_t y_magn, uint16_t z_magn)
{
    uint8_t payload[6];
    payload[0] = (x_magn >> 0) & 0xFF;
    payload[1] = (x_magn >> 8) & 0xFF;
    payload[2] = (y_magn >> 0) & 0xFF;
    payload[3] = (y_magn >> 8) & 0xFF;
    payload[4] = (z_magn >> 0) & 0xFF;
    payload[5] = (z_magn >> 8) & 0xFF;

    static const uint16_t data_type_id = 1052;
    static uint8_t transfer_id;
    static uint8_t transfer_type = 0x00;
    uint64_t data_type_signature = 0x8899AABBCCDDEEFF;
    return canardBroadcast(uavcan_node_id, data_type_signature,
                           data_type_id, transfer_type, &transfer_id, payload, sizeof(payload));
}

// / Standard data type: uavcan.equipment.IMU.barom
int publish_IMU_barom(uint16_t barom_low, uint16_t barom_out)
{
    uint8_t payload[4];
    payload[0] = (barom_low >> 0) & 0xFF;
    payload[1] = (barom_low >> 8) & 0xFF;
    payload[2] = (barom_out >> 0) & 0xFF;
    payload[3] = (barom_out >> 8) & 0xFF;

    static const uint16_t data_type_id = 1053;
    static uint8_t transfer_id;
    static uint8_t transfer_type = 0x00;
    uint64_t data_type_signature = 0x8899AABBCCDDEEFF;
    return canardBroadcast(uavcan_node_id, data_type_signature,
                           data_type_id, transfer_type, &transfer_id, payload, sizeof(payload));
}

// / Standard data type: uavcan.equipment.IMU.deltang
int publish_IMU_deltang(uint16_t x_deltang, uint16_t y_deltang, uint16_t z_deltang)
{
    uint8_t payload[6];
    payload[0] = (x_deltang >> 0) & 0xFF;
    payload[1] = (x_deltang >> 8) & 0xFF;
    payload[2] = (y_deltang >> 0) & 0xFF;
    payload[3] = (y_deltang >> 8) & 0xFF;
    payload[4] = (z_deltang >> 0) & 0xFF;
    payload[5] = (z_deltang >> 8) & 0xFF;

    static const uint16_t data_type_id = 1054;
    static uint8_t transfer_id;
    static uint8_t transfer_type = 0x00;
    uint64_t data_type_signature = 0x8899AABBCCDDEEFF;
    return canardBroadcast(uavcan_node_id, data_type_signature,
                           data_type_id, transfer_type, &transfer_id, payload, sizeof(payload));
}
// / Standard data type: uavcan.equipment.IMU.deltvel
int publish_IMU_deltvel(uint16_t x_deltvel, uint16_t y_deltvel, uint16_t z_deltvel)
{
    uint8_t payload[6];
    payload[0] = (x_deltvel >> 0) & 0xFF;
    payload[1] = (x_deltvel >> 8) & 0xFF;
    payload[2] = (y_deltvel >> 0) & 0xFF;
    payload[3] = (y_deltvel >> 8) & 0xFF;
    payload[4] = (z_deltvel >> 0) & 0xFF;
    payload[5] = (z_deltvel >> 8) & 0xFF;

    static const uint16_t data_type_id = 1055;
    static uint8_t transfer_id;
    static uint8_t transfer_type = 0x00;
    uint64_t data_type_signature = 0x8899AABBCCDDEEFF;
    return canardBroadcast(uavcan_node_id, data_type_signature,
                           data_type_id, transfer_type, &transfer_id, payload, sizeof(payload));
}


// returns true with a probability of probability
bool random_drop(double probability)
{
    return rand() <  probability * ((double)RAND_MAX + 1.0);
}

