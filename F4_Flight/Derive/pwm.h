#ifndef __PWM_H__
#define __PWM_H__

#include "system.h"

void PWM_Init(void);
void PWM_SetCompare1(uint16_t Compare);
void PWM_SetCompare2(uint16_t Compare);
void PWM_SetCompare3(uint16_t Compare);
void PWM_SetCompare4(uint16_t Compare);
void Motor_PWM_Out(int16_t motor_1, int16_t motor_2, int16_t motor_3, int16_t motor_4);


#endif	// __PWM_H__
