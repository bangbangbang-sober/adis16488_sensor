/*
 * Common library for ADIS16XXX devices
 *
 * Copyright 2012 Analog Devices Inc.
 *   Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2 or later.
 */

#include <stdint.h>
#include <stm32f4xx.h>


extern uint16_t x_gyro, y_gyro, z_gyro;
extern uint16_t x_accl, y_accl, z_accl;
extern uint16_t x_magn, y_magn, z_magn;
extern uint16_t barom_low, barom_out;
extern uint32_t barom;
extern uint16_t x_deltang, y_deltang, z_deltang;
extern uint16_t x_deltvel, y_deltvel, z_deltvel;

int adis16480_initial_setup(void);
int adis_self_test(void);

unsigned int adis16480_show_firmware_date(void);
int adis16480_show_serial_number(uint16_t *val);
int adis16480_show_product_id(uint16_t *val);
int adis16480_show_flash_count(uint32_t *val);
int adis16480_set_freq(int val, int val2);
int adis16480_get_freq(int *val, int *val2);
int adis16480_set_calibbias(unsigned int reg, int bias);
int adis16480_set_calibbias(unsigned int reg, int bias);
int adis16480_set_calibscale(unsigned int reg, int scale);
int adis16480_get_calibscale(unsigned int reg, int *scale);

unsigned int adis16480_StatusReg(void);

int readgyro(void);
int readaccl(void);
int readmagn(void);
int readbarom(void);
int readdeltang(void);
int readdeltvel(void);
