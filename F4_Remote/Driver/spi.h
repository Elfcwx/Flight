#ifndef __SPI_H__
#define __SPI_H__

#include "system.h"

void SPI1_Config_Init(void);
void SPI2_Config_Init(void);

void SPI1_SetSpeed(u8 SPI_DivideFrequency);
u8 SPI1_SwapByte(u8 TxData);
u8 SPI2_ReadByte(void);
void SPI2_WriteByte(u8 TxData);

#endif	// __SPI_H__

