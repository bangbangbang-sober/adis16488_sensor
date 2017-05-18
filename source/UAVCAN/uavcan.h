/*
 * Copyright (c) 2016 UAVCAN Team
 *
 * Distributed under the MIT License, available in the file LICENSE.
 *
 * Author: Michael Sierra <sierramichael.a@gmail.com>
 *
 */


#ifndef CANARD_MAIN_H
#define CANARD_MAIN_H

#include <stdint.h>

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


int publish_node_status(enum node_health health, enum node_mode mode,
                        uint16_t vendor_specific_status_code);

 
int publish_IMU_gyro(uint16_t x_gyro, uint16_t y_gyro, uint16_t z_gyro);
int publish_IMU_accl(uint16_t x_accl, uint16_t y_accl, uint16_t z_accl); 
int publish_IMU_magn(uint16_t x_magn, uint16_t y_magn, uint16_t z_magn); 
int publish_IMU_barom(uint16_t barom_low, uint16_t barom_out); 
int publish_IMU_deltang(uint16_t x_deltang, uint16_t y_deltang, uint16_t z_deltang);
int publish_IMU_deltvel(uint16_t x_deltvel, uint16_t y_deltvel, uint16_t z_deltvel);
 

#endif
