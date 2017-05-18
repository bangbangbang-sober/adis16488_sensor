 /*
 * Common library for ADIS16XXX devices
 *
 * Copyright 2012 Analog Devices Inc.
 *   Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __ADIS_H__
#define __ADIS_H__

#include <stdint.h>
#include <stdbool.h>
#include "spi.h"
#include "stm32f4xx.h"
#define NSS_H   GPIO_SetBits(GPIOA, GPIO_Pin_4)
#define NSS_L   GPIO_ResetBits(GPIOA, GPIO_Pin_4)  
#define ARRAY_SIZE(x) (sizeof(x) ? sizeof(x) / sizeof((x)[0]) : 0)
#define BIT(x) (1 << (x))
#define BIT_MASK(bit) BIT((bit) % 32)
#define BIT_WORD(bit) ((bit) / 32)
#define TEST_BIT(addr, bit) (!!(*(((uint32_t *) addr) + BIT_WORD(bit)) \
		& BIT_MASK(bit)))


#define ADIS_WRITE_REG(reg) ((0x80 | (reg)))
#define ADIS_READ_REG(reg) ((reg) & 0x7f)

#define ADIS_PAGE_SIZE 0x80
#define ADIS_REG_PAGE_ID 0x00


#define ADIS16480_REG_PAGE(addr) (uint8_t)(addr / ADIS_PAGE_SIZE)
#define ADIS16480_REG_REG(addr) (uint8_t)(addr % ADIS_PAGE_SIZE)

extern SPI_TypeDef spi1;

/**
 * struct adis_data - ADIS chip variant specific data
 * @read_delay: SPI delay for read operations in us
 * @write_delay: SPI delay for write operations in us
 * @glob_cmd_reg: Register address of the GLOB_CMD register
 * @msc_ctrl_reg: Register address of the MSC_CTRL register
 * @diag_stat_reg: Register address of the DIAG_STAT register
 * @status_error_msgs: Array of error messgaes
 * @status_error_mask:
 */
struct adis_data {
	unsigned int read_delay;
	unsigned int write_delay;

	unsigned int glob_cmd_reg;
	unsigned int msc_ctrl_reg;
	unsigned int diag_stat_reg;

	unsigned int self_test_mask;
	unsigned int startup_delay;

	const char * const *status_error_msgs;
	unsigned int status_error_mask;

	int (*enable_irq)(struct adis *adis, bool enable);

	bool has_paging;
};

/**
 * adis_write_reg_page() - Write page  register
 * @adis: The adis device
 * @reg: The address of the register to be written

 */
static inline int adis_write_reg_page(SPI_TypeDef  *hspi, unsigned int reg)
{
  uint8_t pData[2];
  uint8_t aPage;
	uint16_t Ptdata;
  aPage = ADIS16480_REG_PAGE(reg);
  
  // Ð´Ò³Âë
  pData[0] = 0x80;
  pData[1] = aPage;
	Ptdata = (((uint16_t)pData[0] << 8) | ((uint16_t)pData[1]));
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);{}
	SPI_I2S_SendData(SPI1,Ptdata);
  //HAL_SPI_Transmit(hspi, pData, 1, 10000);
	return 1;
}

/**
 * adis_write_reg_8() - Write single byte to a register
 * @adis: The adis device
 * @reg: The address of the register to be written
 * @value: The value to write
 */
static inline int adis_write_reg_8(SPI_TypeDef  *hspi, unsigned int reg,
	uint8_t val)
{
  uint8_t pData[2];
	uint16_t Ptdata;
  uint8_t aReg;
  aReg = ADIS16480_REG_REG(reg);
  
  // Ð´Ò³Âë
  adis_write_reg_page(hspi, reg);
  
  // Ð´Êý¾Ý
  pData[0] = 0x80|aReg;
  pData[1] = val;
	Ptdata = (((uint16_t)pData[0] << 8) | ((uint16_t)pData[1]));
  SPI_I2S_SendData(SPI1,Ptdata);
	
	//HAL_SPI_Transmit(hspi, pData, 1, 10000);
	return 1;
}

/**
 * adis_write_reg_16() - Write 2 bytes to a pair of registers
 * @adis: The adis device
 * @reg: The address of the lower of the two registers
 * @value: Value to be written
 */
static inline int adis_write_reg_16(SPI_TypeDef *hspi, unsigned int reg,
	uint16_t val)
{
  uint8_t pData[4];
  uint8_t aReg;
	uint16_t Ptdata;
  aReg = ADIS16480_REG_REG(reg);
  
  // Ð´Ò³Âë
  adis_write_reg_page(hspi, reg);
  
  // Ð´Êý¾Ý   
  pData[0] = 0x80|aReg;
  pData[1] = (uint8_t)(val&0xff);
	Ptdata = (((uint16_t)pData[0] << 8) | ((uint16_t)pData[1]));
  SPI_I2S_SendData(SPI1,Ptdata);
  pData[2] = 0x80|(uint8_t)(aReg++);
  pData[3] = (uint8_t)((val>>8)&0xff);
  Ptdata = (((uint16_t)pData[2] << 8) | ((uint16_t)pData[3]));
  SPI_I2S_SendData(SPI1,Ptdata);
  //HAL_SPI_Transmit(hspi, pData, 2, 10000);
  
	return 1;
}

/**
 * adis_write_reg_32() - write 4 bytes to four registers
 * @adis: The adis device
 * @reg: The address of the lower of the four register
 * @value: Value to be written
 */
static inline int adis_write_reg_32(SPI_TypeDef  *hspi, unsigned int reg,
	uint32_t val)
{
  uint8_t pData[8];
  uint8_t aReg;
	uint16_t Ptdata;
  aReg = ADIS16480_REG_REG(reg);
  
  // Ð´Ò³Âë
  adis_write_reg_page(hspi, reg);
  
  // Ð´Êý¾Ý   
  pData[0] = 0x80|aReg;
  pData[1] = (uint8_t)(val&0xff);
	Ptdata = (((uint16_t)pData[0] << 8) | ((uint16_t)pData[1]));
  SPI_I2S_SendData(SPI1,Ptdata);
  pData[2] = 0x80|(aReg++);
  pData[3] = (uint8_t)((val>>8)&0xff);
	Ptdata = (((uint16_t)pData[2] << 8) | ((uint16_t)pData[3]));
  SPI_I2S_SendData(SPI1,Ptdata);
  pData[4] = 0x80||(aReg++);
  pData[5] = (uint8_t)((val>>16)&0xff);  
	Ptdata = (((uint16_t)pData[4] << 8) | ((uint16_t)pData[5]));
  SPI_I2S_SendData(SPI1,Ptdata);
  pData[6] = 0x80|(aReg++);
  pData[7] = (uint8_t)((val>>24)&0xff);
	Ptdata = (((uint16_t)pData[6] << 8) | ((uint16_t)pData[7]));
  SPI_I2S_SendData(SPI1,Ptdata);
  
  //HAL_SPI_Transmit(hspi, pData, 4, 10000);

	return 1;
}

/**
 * adis_read_reg_16() - read 2 bytes from a 16-bit register
 * @adis: The adis device
 * @reg: The address of the lower of the two registers
 * @val: The value read back from the device
 */
static inline int adis_read_reg_16(SPI_TypeDef  *hspi, unsigned int reg,
	uint16_t *val)
{
	unsigned int tmp;
	int ret;
  uint8_t pTxData[2], pRxData[2];
	uint16_t  ppRxdata,ppRxdata1,ppRxdata2,ppRxdata3,ppRxdata4;
	uint16_t  Ptdata;
  uint8_t aReg;
  aReg = ADIS16480_REG_REG(reg);
  
  // Ð´Ò³Âë
 // adis_write_reg_page(hspi, reg);
  
  // Ð´Êý¾Ý   
  pTxData[0] = aReg;//0x80|aReg;
  pTxData[1] = 0x00;  
	Ptdata = (((uint16_t)pTxData[0] << 8) | ((uint16_t)pTxData[1]));
  ppRxdata = SPI1_ReadWrite16Bit(Ptdata);
	//ppRxdata1 = SPI1_ReadWrite16Bit(Ptdata);

	//HAL_SPI_TransmitReceive(hspi, pTxData, pRxData, 1, 10000); 
	*val = ppRxdata;//(pRxData[0]<<8)|pRxData[1];

	return 1;
}

/**
 * adis_read_reg_32() - read 4 bytes from a 32-bit register
 * @adis: The adis device
 * @reg: The address of the lower of the two registers
 * @val: The value read back from the device
 */
static inline int adis_read_reg_32(SPI_TypeDef  *hspi, unsigned int reg,
	uint32_t *val)
{
	unsigned int tmp;
	int ret;
  uint8_t addr;
  uint16_t  ppRxdata[2];
	uint16_t  Ptdata;
  uint8_t pTxData[4], pRxData[4];
  uint8_t aReg;
  aReg = ADIS16480_REG_REG(reg);
  
  // Ð´Ò³Âë
  adis_write_reg_page(hspi, reg);
  
  // Ð´Êý¾Ý   
  pTxData[0] = 0x80|aReg;
  pTxData[1] = 0x00;  
	Ptdata = (((uint16_t)pTxData[0] << 8) | ((uint16_t)pTxData[1]));
  ppRxdata[0] = SPI1_ReadWrite16Bit(Ptdata);
  pTxData[2] = 0x80|(aReg++);
  pTxData[3] = 0x00;
  Ptdata = (((uint16_t)pTxData[2] << 8) | ((uint16_t)pTxData[3]));
  ppRxdata[1] = SPI1_ReadWrite16Bit(Ptdata);
	
  //	HAL_SPI_TransmitReceive(hspi, pTxData, pRxData, 2, 10000); 
  *val = ((uint32_t)ppRxdata[0]<<16)|((uint32_t)ppRxdata[1]);
	//*val = ((uint32_t)pRxData[2]<<24)|((uint32_t)pRxData[3]<<16)|((uint32_t)pRxData[0]<<8)|(uint32_t)pRxData[1];

	return 1;
}



int adis_reset(void);

#endif
