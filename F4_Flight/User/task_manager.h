#ifndef __TASK_MANAGER_H__
#define __TASK_MANAGER_H__

#include "system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart1.h"
#include "led.h"
#include "posture.h"
#include "optical.h"
#include "adc.h"
#include "remote.h"
#include "control.h"


#define CONTROL_TASK_PRIO			3
#define CONTROL_STK_SIZE			256 

#define LIGHTFLOW_TASK_PRIO			2
#define LIGHTFLOW_STK_SIZE			256 

#define SEND_TASK_PRIO				1
#define SEND_STK_SIZE				256 


void Flight_task(void);


#endif	//__TASK_MANAGER_H__

