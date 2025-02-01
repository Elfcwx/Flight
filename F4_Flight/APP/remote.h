#ifndef __REMOTE_H__
#define __REMOTE_H__

#include "system.h"
#include "nrf24l01.h"
#include "adc.h"
#include "usart1.h"

extern float nrf_Roll;
extern float nrf_Pitch;
extern float nrf_Yaw;
extern uint16_t MOTO_Speed;

void Remote_Receive(void);

extern uint8_t takeOffF;
extern uint8_t unlockF;

#endif	//__REMOTE_H__
