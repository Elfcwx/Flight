#ifndef __SPI_H__
#define __SPI_H__

#include "system.h"

void SPI_Config_Init(void);
u8 SPI1_SwapByte(u8 TxData);
void SPI1_SetSpeed(u8 SPI_DivideFrequency);

#endif	// __SPI_H__

