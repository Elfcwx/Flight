#ifndef __USART6_H__
#define __USART6_H__

#include "system.h"
#include "ringbuffer.h"

void USART6_Init(void);
uint16_t USART6_GetData(uint8_t *buf, uint16_t len);
#endif	//__USART6_H__
