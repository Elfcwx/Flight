#ifndef _SYSTICK_H
#define _SYSTICK_H

#include "system.h"

void SysTick_Init(uint16_t SYSCLK);
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);
#endif
