#ifndef __I2C_H__
#define __I2C_H__

#include "system.h"

void I2C_Config_Init(void);
void I2C_Write(u8 devAddr_WR, u8 reg, u8 data);
void I2C_Read_Buffer(u8 devAddr_RD, u8 reg, u8 *pBuffer, u16 NumByteToRead);

#endif	// __I2C_H__

