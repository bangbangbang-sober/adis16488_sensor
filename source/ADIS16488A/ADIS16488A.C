/*
 * ADIS16480 and similar IMUs driver
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */


#include <stdint.h>
#include <time.h>

#include "adis.h"

static int adis16480_read_raw32(unsigned int reg, uint32_t *val);
static int adis16480_read_raw16(unsigned int reg, uint16_t *val);
extern SPI_TypeDef spi1;
int adis_enable_irq(bool enable);

int adis16480_stop_device(void);


#define ADIS_MSC_CTRL_DATA_RDY_EN	BIT(2)
#define ADIS_MSC_CTRL_DATA_RDY_POL_HIGH	BIT(1)
#define ADIS_MSC_CTRL_DATA_RDY_DIO2	BIT(0)
#define ADIS_GLOB_CMD_SW_RESET		BIT(7)

#define ADIS16480_PAGE_SIZE 0x80

#define ADIS16480_REG(page, reg) ((page) * ADIS16480_PAGE_SIZE + (reg))

#define ADIS16480_REG_PAGE_ID 0x00 /* Same address on each page */
#define ADIS16480_REG_SEQ_CNT			ADIS16480_REG(0x00, 0x06)
#define ADIS16480_REG_SYS_E_FLA			ADIS16480_REG(0x00, 0x08)
#define ADIS16480_REG_DIAG_STS			ADIS16480_REG(0x00, 0x0A)
#define ADIS16480_REG_ALM_STS			ADIS16480_REG(0x00, 0x0C)
#define ADIS16480_REG_TEMP_OUT			ADIS16480_REG(0x00, 0x0E)
#define ADIS16480_REG_X_GYRO_OUT		ADIS16480_REG(0x00, 0x12)//0x10
#define ADIS16480_REG_Y_GYRO_OUT		ADIS16480_REG(0x00, 0x16)//0x14
#define ADIS16480_REG_Z_GYRO_OUT		ADIS16480_REG(0x00, 0x1A)//0x18
#define ADIS16480_REG_X_ACCEL_OUT		ADIS16480_REG(0x00, 0x1E)//0x1C
#define ADIS16480_REG_Y_ACCEL_OUT		ADIS16480_REG(0x00, 0x22)//0x20
#define ADIS16480_REG_Z_ACCEL_OUT		ADIS16480_REG(0x00, 0x26)//0x24
#define ADIS16480_REG_X_MAGN_OUT		ADIS16480_REG(0x00, 0x28)//
#define ADIS16480_REG_Y_MAGN_OUT		ADIS16480_REG(0x00, 0x2A)//
#define ADIS16480_REG_Z_MAGN_OUT		ADIS16480_REG(0x00, 0x2C)//
#define ADIS16480_REG_BAROM_OUT			ADIS16480_REG(0x00, 0x2E)
#define ADIS16480_REG_X_DELTAANG_OUT		ADIS16480_REG(0x00, 0x42)//0x40
#define ADIS16480_REG_Y_DELTAANG_OUT		ADIS16480_REG(0x00, 0x46)//0x44
#define ADIS16480_REG_Z_DELTAANG_OUT		ADIS16480_REG(0x00, 0x4A)//0x48
#define ADIS16480_REG_X_DELTAVEL_OUT		ADIS16480_REG(0x00, 0x4E)//0x4C
#define ADIS16480_REG_Y_DELTAVEL_OUT		ADIS16480_REG(0x00, 0x52)//0x50
#define ADIS16480_REG_Z_DELTAVEL_OUT		ADIS16480_REG(0x00, 0x56)//0x54
#define ADIS16480_REG_PROD_ID			ADIS16480_REG(0x00, 0x7E)

#define ADIS16480_REG_X_GYRO_SCALE		ADIS16480_REG(0x02, 0x04)
#define ADIS16480_REG_Y_GYRO_SCALE		ADIS16480_REG(0x02, 0x06)
#define ADIS16480_REG_Z_GYRO_SCALE		ADIS16480_REG(0x02, 0x08)
#define ADIS16480_REG_X_ACCEL_SCALE		ADIS16480_REG(0x02, 0x0A)
#define ADIS16480_REG_Y_ACCEL_SCALE		ADIS16480_REG(0x02, 0x0C)
#define ADIS16480_REG_Z_ACCEL_SCALE		ADIS16480_REG(0x02, 0x0E)
#define ADIS16480_REG_X_GYRO_BIAS		ADIS16480_REG(0x02, 0x10)
#define ADIS16480_REG_Y_GYRO_BIAS		ADIS16480_REG(0x02, 0x14)
#define ADIS16480_REG_Z_GYRO_BIAS		ADIS16480_REG(0x02, 0x18)
#define ADIS16480_REG_X_ACCEL_BIAS		ADIS16480_REG(0x02, 0x1C)
#define ADIS16480_REG_Y_ACCEL_BIAS		ADIS16480_REG(0x02, 0x20)
#define ADIS16480_REG_Z_ACCEL_BIAS		ADIS16480_REG(0x02, 0x24)
#define ADIS16480_REG_X_HARD_IRON		ADIS16480_REG(0x02, 0x28)
#define ADIS16480_REG_Y_HARD_IRON		ADIS16480_REG(0x02, 0x2A)
#define ADIS16480_REG_Z_HARD_IRON		ADIS16480_REG(0x02, 0x2C)
#define ADIS16480_REG_BAROM_BIAS		ADIS16480_REG(0x02, 0x40)
#define ADIS16480_REG_FLASH_CNT			ADIS16480_REG(0x02, 0x7C)

#define ADIS16480_REG_GLOB_CMD			ADIS16480_REG(0x03, 0x02)
#define ADIS16480_REG_FNCTIO_CTRL		ADIS16480_REG(0x03, 0x06)
#define ADIS16480_REG_GPIO_CTRL			ADIS16480_REG(0x03, 0x08)
#define ADIS16480_REG_CONFIG			ADIS16480_REG(0x03, 0x0A)
#define ADIS16480_REG_DEC_RATE			ADIS16480_REG(0x03, 0x0C)
#define ADIS16480_REG_SLP_CNT			ADIS16480_REG(0x03, 0x10)
#define ADIS16480_REG_FILTER_BNK0		ADIS16480_REG(0x03, 0x16)
#define ADIS16480_REG_FILTER_BNK1		ADIS16480_REG(0x03, 0x18)
#define ADIS16480_REG_ALM_CNFG0			ADIS16480_REG(0x03, 0x20)
#define ADIS16480_REG_ALM_CNFG1			ADIS16480_REG(0x03, 0x22)
#define ADIS16480_REG_ALM_CNFG2			ADIS16480_REG(0x03, 0x24)
#define ADIS16480_REG_XG_ALM_MAGN		ADIS16480_REG(0x03, 0x28)
#define ADIS16480_REG_YG_ALM_MAGN		ADIS16480_REG(0x03, 0x2A)
#define ADIS16480_REG_ZG_ALM_MAGN		ADIS16480_REG(0x03, 0x2C)
#define ADIS16480_REG_XA_ALM_MAGN		ADIS16480_REG(0x03, 0x2E)
#define ADIS16480_REG_YA_ALM_MAGN		ADIS16480_REG(0x03, 0x30)
#define ADIS16480_REG_ZA_ALM_MAGN		ADIS16480_REG(0x03, 0x32)
#define ADIS16480_REG_XM_ALM_MAGN		ADIS16480_REG(0x03, 0x34)
#define ADIS16480_REG_YM_ALM_MAGN		ADIS16480_REG(0x03, 0x36)
#define ADIS16480_REG_ZM_ALM_MAGN		ADIS16480_REG(0x03, 0x38)
#define ADIS16480_REG_BR_ALM_MAGN		ADIS16480_REG(0x03, 0x3A)
#define ADIS16480_REG_FIRM_REV			ADIS16480_REG(0x03, 0x78)
#define ADIS16480_REG_FIRM_DM			ADIS16480_REG(0x03, 0x7A)
#define ADIS16480_REG_FIRM_Y			ADIS16480_REG(0x03, 0x7C)

#define ADIS16480_REG_SERIAL_NUM		ADIS16480_REG(0x04, 0x20)

/* Each filter coefficent bank spans two pages */
#define ADIS16480_FIR_COEF(page) (x < 60 ? ADIS16480_REG(page, (x) + 8) : \
		ADIS16480_REG((page) + 1, (x) - 60 + 8))
#define ADIS16480_FIR_COEF_A(x)			ADIS16480_FIR_COEF(0x05, (x))
#define ADIS16480_FIR_COEF_B(x)			ADIS16480_FIR_COEF(0x07, (x))
#define ADIS16480_FIR_COEF_C(x)			ADIS16480_FIR_COEF(0x09, (x))
#define ADIS16480_FIR_COEF_D(x)			ADIS16480_FIR_COEF(0x0B, (x))


//extern SPI_HandleTypeDef hspi1;

uint16_t x_gyro, y_gyro, z_gyro;
uint16_t x_accl, y_accl, z_accl;
uint16_t x_magn, y_magn, z_magn;
uint16_t barom_low, barom_out;
uint32_t barom;
uint16_t x_deltang, y_deltang, z_deltang;
uint16_t x_deltvel, y_deltvel, z_deltvel;



struct adis16480_chip_info {
	unsigned int num_channels;
	const struct iio_chan_spec *channels;
	unsigned int gyro_max_val;
	unsigned int gyro_max_scale;
	unsigned int accel_max_val;
	unsigned int accel_max_scale;
};


struct adis16480 {
	
	const struct adis16480_chip_info *chip_info;
	
};


unsigned int adis16480_show_firmware_revision(void)
{

	char buf[7];
	uint16_t rev;
	int ret;

	ret = adis_read_reg_16(&spi1, ADIS16480_REG_FIRM_REV, &rev);
	if (ret < 0)
		return ret;

	return rev;
}

unsigned int adis16480_StatusReg(void)
{

	char buf[7];
	uint16_t sta;
	int ret;

	ret = adis_read_reg_16(&spi1, ADIS16480_REG_SYS_E_FLA, &sta);
	if (ret < 0)
		return ret;

	return sta;
}


unsigned int adis16480_show_firmware_date(void)
{
	
	uint16_t md, year;
	char buf[12];
	size_t len;
	int ret;

	ret = adis_read_reg_16(&spi1, ADIS16480_REG_FIRM_Y, &year);
	if (ret < 0)
		return ret;

	ret = adis_read_reg_16(&spi1, ADIS16480_REG_FIRM_DM, &md);
	if (ret < 0)
		return ret;

	return 1;
}


int adis16480_show_serial_number(uint16_t *val)
{
	uint16_t serial;
	int ret;

	ret = adis_read_reg_16(&spi1, ADIS16480_REG_SERIAL_NUM, &serial);
	if (ret < 0)
		return ret;

	*val = serial;

	return 0;
}


int adis16480_show_product_id(uint16_t *val)
{
	uint16_t prod_id;
	int ret;

	ret = adis_read_reg_16(&spi1, ADIS16480_REG_PROD_ID, &prod_id);
	if (ret < 0)
		return ret;

	*val = prod_id;

	return 0;
}


int adis16480_show_flash_count(uint32_t *val)
{
	uint32_t flash_count;
	int ret;

	ret = adis_read_reg_32(&spi1, ADIS16480_REG_FLASH_CNT,
		&flash_count);
	if (ret < 0)
		return ret;

	*val = flash_count;

	return 0;
}



int adis16480_set_freq(int val, int val2)
{
	
	unsigned int t;

	t =  val * 1000 + val2 / 1000;
	if (t <= 0)
		return -1;

	t = 2460000 / t;
	if (t > 2048)
		t = 2048;

	if (t != 0)
		t--;

	return adis_write_reg_16(&spi1, ADIS16480_REG_DEC_RATE, t);
}


int adis16480_get_freq(int *val, int *val2)
{
	uint16_t t;
	int ret;
	unsigned freq;

	ret = adis_read_reg_16(&spi1, ADIS16480_REG_DEC_RATE, &t);
	if (ret < 0)
		return ret;

	freq = 2460000 / (t + 1);
	*val = freq / 1000;
	*val2 = (freq % 1000) * 1000;

	return 1;
}

enum {
	ADIS16480_SCAN_GYRO_X,
	ADIS16480_SCAN_GYRO_Y,
	ADIS16480_SCAN_GYRO_Z,
	ADIS16480_SCAN_ACCEL_X,
	ADIS16480_SCAN_ACCEL_Y,
	ADIS16480_SCAN_ACCEL_Z,
	ADIS16480_SCAN_MAGN_X,
	ADIS16480_SCAN_MAGN_Y,
	ADIS16480_SCAN_MAGN_Z,
	ADIS16480_SCAN_BARO,
	ADIS16480_SCAN_TEMP,
};

static const unsigned int adis16480_calibbias_regs[] = {
	[ADIS16480_SCAN_GYRO_X] = ADIS16480_REG_X_GYRO_BIAS,
	[ADIS16480_SCAN_GYRO_Y] = ADIS16480_REG_Y_GYRO_BIAS,
	[ADIS16480_SCAN_GYRO_Z] = ADIS16480_REG_Z_GYRO_BIAS,
	[ADIS16480_SCAN_ACCEL_X] = ADIS16480_REG_X_ACCEL_BIAS,
	[ADIS16480_SCAN_ACCEL_Y] = ADIS16480_REG_Y_ACCEL_BIAS,
	[ADIS16480_SCAN_ACCEL_Z] = ADIS16480_REG_Z_ACCEL_BIAS,
	[ADIS16480_SCAN_MAGN_X] = ADIS16480_REG_X_HARD_IRON,
	[ADIS16480_SCAN_MAGN_Y] = ADIS16480_REG_Y_HARD_IRON,
	[ADIS16480_SCAN_MAGN_Z] = ADIS16480_REG_Z_HARD_IRON,
	[ADIS16480_SCAN_BARO] = ADIS16480_REG_BAROM_BIAS,
};

static const unsigned int adis16480_calibscale_regs[] = {
	[ADIS16480_SCAN_GYRO_X] = ADIS16480_REG_X_GYRO_SCALE,
	[ADIS16480_SCAN_GYRO_Y] = ADIS16480_REG_Y_GYRO_SCALE,
	[ADIS16480_SCAN_GYRO_Z] = ADIS16480_REG_Z_GYRO_SCALE,
	[ADIS16480_SCAN_ACCEL_X] = ADIS16480_REG_X_ACCEL_SCALE,
	[ADIS16480_SCAN_ACCEL_Y] = ADIS16480_REG_Y_ACCEL_SCALE,
	[ADIS16480_SCAN_ACCEL_Z] = ADIS16480_REG_Z_ACCEL_SCALE,
};

int adis16480_set_calibbias(unsigned int reg, int bias)
{
	
	return adis_write_reg_16(&spi1, reg, bias);

}

int adis16480_get_calibbias(unsigned int reg, int *bias)
{

	uint16_t val16;
	uint32_t val32;
	int ret;

	ret = adis_read_reg_16(&spi1, reg, &val16);
	*bias =(int)(uint32_t)val16;

	return 1;
}

int adis16480_set_calibscale(unsigned int reg, int scale)
{

	return adis_write_reg_16(&spi1, reg, scale);
	
}

int adis16480_get_calibscale(unsigned int reg, int *scale)
{
	uint16_t val16;
	int ret;

	ret = adis_read_reg_16(&spi1, reg, &val16);
	if (ret < 0)
		return ret;

	*scale = (int)(uint32_t)val16;
	return 1;
}

static const unsigned int adis16480_def_filter_freqs[] = {
	310,
	55,
	275,
	63,
};

static const unsigned int ad16480_filter_data[][2] = {
	[ADIS16480_SCAN_GYRO_X]		= { ADIS16480_REG_FILTER_BNK0, 0 },
	[ADIS16480_SCAN_GYRO_Y]		= { ADIS16480_REG_FILTER_BNK0, 3 },
	[ADIS16480_SCAN_GYRO_Z]		= { ADIS16480_REG_FILTER_BNK0, 6 },
	[ADIS16480_SCAN_ACCEL_X]	= { ADIS16480_REG_FILTER_BNK0, 9 },
	[ADIS16480_SCAN_ACCEL_Y]	= { ADIS16480_REG_FILTER_BNK0, 12 },
	[ADIS16480_SCAN_ACCEL_Z]	= { ADIS16480_REG_FILTER_BNK1, 0 },
	[ADIS16480_SCAN_MAGN_X]		= { ADIS16480_REG_FILTER_BNK1, 3 },
	[ADIS16480_SCAN_MAGN_Y]		= { ADIS16480_REG_FILTER_BNK1, 6 },
	[ADIS16480_SCAN_MAGN_Z]		= { ADIS16480_REG_FILTER_BNK1, 9 },
};


static int adis16480_read_raw16(unsigned int reg, uint16_t *val)
{
	return adis_read_reg_16(&spi1, reg, val);
}

static int adis16480_read_raw32(unsigned int reg, uint32_t *val)
{
	return adis_read_reg_32(&spi1, reg, val);
}

int adis16480_stop_device(void)
{
	int ret;

	ret = adis_write_reg_16(&spi1, ADIS16480_REG_SLP_CNT, BIT(9));
	if (ret)
		return -1;

	return ret;
}

int adis16480_enable_irq(bool enable)
{
	return adis_write_reg_16(&spi1, ADIS16480_REG_FNCTIO_CTRL,
		enable ? BIT(3) : 0);
}

int adis16480_initial_setup(void)
{
	uint16_t prod_id;
	unsigned int device_id;
	int ret;

	adis_reset();
  //HAL_Delay(70);
  
	ret = adis_write_reg_16(&spi1, ADIS16480_REG_GLOB_CMD, BIT(1));
	if (ret)
		return ret;
	//HAL_Delay(30);

	ret = adis_read_reg_16(&spi1, ADIS16480_REG_PROD_ID, &prod_id);
	if (ret)
		return ret;
	
	adis_enable_irq( 1 );
	
	return 0;
}

#define ADIS16480_DIAG_STAT_XGYRO_FAIL 0
#define ADIS16480_DIAG_STAT_YGYRO_FAIL 1
#define ADIS16480_DIAG_STAT_ZGYRO_FAIL 2
#define ADIS16480_DIAG_STAT_XACCL_FAIL 3
#define ADIS16480_DIAG_STAT_YACCL_FAIL 4
#define ADIS16480_DIAG_STAT_ZACCL_FAIL 5
#define ADIS16480_DIAG_STAT_XMAGN_FAIL 8
#define ADIS16480_DIAG_STAT_YMAGN_FAIL 9
#define ADIS16480_DIAG_STAT_ZMAGN_FAIL 10
#define ADIS16480_DIAG_STAT_BARO_FAIL 11

static const char * const adis16480_status_error_msgs[] = {
	[ADIS16480_DIAG_STAT_XGYRO_FAIL] = "X-axis gyroscope self-test failure",
	[ADIS16480_DIAG_STAT_YGYRO_FAIL] = "Y-axis gyroscope self-test failure",
	[ADIS16480_DIAG_STAT_ZGYRO_FAIL] = "Z-axis gyroscope self-test failure",
	[ADIS16480_DIAG_STAT_XACCL_FAIL] = "X-axis accelerometer self-test failure",
	[ADIS16480_DIAG_STAT_YACCL_FAIL] = "Y-axis accelerometer self-test failure",
	[ADIS16480_DIAG_STAT_ZACCL_FAIL] = "Z-axis accelerometer self-test failure",
	[ADIS16480_DIAG_STAT_XMAGN_FAIL] = "X-axis magnetometer self-test failure",
	[ADIS16480_DIAG_STAT_YMAGN_FAIL] = "Y-axis magnetometer self-test failure",
	[ADIS16480_DIAG_STAT_ZMAGN_FAIL] = "Z-axis magnetometer self-test failure",
	[ADIS16480_DIAG_STAT_BARO_FAIL] = "Barometer self-test failure",
};



/**
 * adis_enable_irq() - Enable or disable data ready IRQ
 * @adis: The adis device
 * @enable: Whether to enable the IRQ
 *
 * Returns 0 on success, negative error code otherwise
 */
int adis_enable_irq(bool enable)
{
	int ret = 0;
	uint16_t msc;

	ret = adis_read_reg_16(&spi1, ADIS16480_REG_GLOB_CMD, &msc);
	if (ret)
		goto error_ret;

	msc |= ADIS_MSC_CTRL_DATA_RDY_POL_HIGH;
	msc &= ~ADIS_MSC_CTRL_DATA_RDY_DIO2;
	if (enable)
		msc |= ADIS_MSC_CTRL_DATA_RDY_EN;
	else
		msc &= ~ADIS_MSC_CTRL_DATA_RDY_EN;

	ret = adis_write_reg_16(&spi1, ADIS16480_REG_GLOB_CMD, msc);

error_ret:
	return ret;
}


/**
 * adis_reset() - Reset the device
 * @adis: The adis device
 *
 * Returns 0 on success, a negative error code otherwise
 */
int adis_reset(void)
{
	int ret;

	ret = adis_write_reg_8(&spi1,  ADIS16480_REG_GLOB_CMD,
			ADIS_GLOB_CMD_SW_RESET);

	return ret;
}


int adis_self_test(void)
{
	int ret;

	ret = adis_write_reg_16(&spi1, ADIS16480_REG_GLOB_CMD, 0x02);

	//HAL_Delay(20);

	return ret;
}


/**
 * ¶ÁIMUÊý¾Ý 
 * @adis: The adis device
 *
 * Returns 0 on success, a negative error code otherwise
 */
int readgyro(void)
{
	int ret;
	//adis16480_read_raw16(ADIS16480_REG_X_GYRO_OUT, &x_gyro);
	adis16480_read_raw16(ADIS16480_REG_X_GYRO_OUT, &x_gyro);
	adis16480_read_raw16(ADIS16480_REG_Y_GYRO_OUT, &y_gyro);
	adis16480_read_raw16(ADIS16480_REG_Z_GYRO_OUT, &z_gyro);
	
	return 1;
}

int readaccl(void)
{
	int ret;
	
	adis16480_read_raw16(ADIS16480_REG_X_ACCEL_OUT, &x_accl);
	adis16480_read_raw16(ADIS16480_REG_Y_ACCEL_OUT, &y_accl);
	adis16480_read_raw16(ADIS16480_REG_Z_ACCEL_OUT, &z_accl);
	
	return 1;
}

int readmagn(void)
{
	int ret;
	//adis16480_read_raw16(ADIS16480_REG_X_MAGN_OUT, &x_magn);
	adis16480_read_raw16(ADIS16480_REG_X_MAGN_OUT, &x_magn);
	adis16480_read_raw16(ADIS16480_REG_Y_MAGN_OUT, &y_magn);
	adis16480_read_raw16(ADIS16480_REG_Z_MAGN_OUT, &z_magn);
	
	return 1;
}

int readbarom(void)
{
	int ret;
	
	adis16480_read_raw32(ADIS16480_REG_BAROM_OUT, &barom);
	barom_low = (uint16_t)(barom & 0xff);
	barom_out = (uint16_t)((barom >> 16) & 0xff);	
	
	return 1;
}

int readdeltang(void)
{
	int ret;
	//adis16480_read_raw16(ADIS16480_REG_X_DELTAANG_OUT, &x_deltang);
	adis16480_read_raw16(ADIS16480_REG_X_DELTAANG_OUT, &x_deltang);
	adis16480_read_raw16(ADIS16480_REG_Y_DELTAANG_OUT, &y_deltang);
	adis16480_read_raw16(ADIS16480_REG_Z_DELTAANG_OUT, &z_deltang);
	
	return 1;
}


int readdeltvel(void)
{
	int ret;
	//adis16480_read_raw16(ADIS16480_REG_X_DELTAVEL_OUT, &x_deltvel);
	adis16480_read_raw16(ADIS16480_REG_X_DELTAVEL_OUT, &x_deltvel);
	adis16480_read_raw16(ADIS16480_REG_Y_DELTAVEL_OUT, &y_deltvel);
	adis16480_read_raw16(ADIS16480_REG_Z_DELTAVEL_OUT, &z_deltvel);
	
	return 1;
}


