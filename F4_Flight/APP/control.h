#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "system.h"
#include "posture.h"
#include "remote.h"
#include "pid.h"
#include "pwm.h"
#include "led.h"
#include "usart1.h"

void Control(uint8_t outerEN);
void DEBugFly_Error(void);
#endif	//__CONTROL_H__
