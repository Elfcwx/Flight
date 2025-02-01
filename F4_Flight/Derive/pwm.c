#include "pwm.h"

#define motor_pwm_max	2000
#define motor_pwm_min	1000

void PWM_Init(void)
{	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_TIM2);
	
	TIM_InternalClockConfig(TIM2);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 4000 - 1;		//ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler = 100 - 1;	//PSC			pwm周期为 250hz--- 4ms脉宽
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStacture;
	TIM_OCInitStacture.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStacture.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStacture.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStacture.TIM_Pulse = 0;		//CCR	四轴电调需要1ms-2ms高电平脉宽 -- 1ms高电平脉宽表示油门停转， 2ms高电平脉宽表示最大油门量 ---- 故 CCR 的值需要在 1000 和 2000 之间表示
	TIM_OC1Init(TIM2, &TIM_OCInitStacture);
	TIM_OC2Init(TIM2, &TIM_OCInitStacture);
	TIM_OC3Init(TIM2, &TIM_OCInitStacture);
	TIM_OC4Init(TIM2, &TIM_OCInitStacture);
	TIM_OCStructInit(&TIM_OCInitStacture);
	TIM_Cmd(TIM2, ENABLE);
}

void Motor_PWM_Out(int16_t motor_1, int16_t motor_2, int16_t motor_3, int16_t motor_4)
{
	motor_1 = motor_1 > motor_pwm_max ? motor_pwm_max : (motor_1 < motor_pwm_min ? motor_pwm_min : motor_1);
	motor_2 = motor_2 > motor_pwm_max ? motor_pwm_max : (motor_2 < motor_pwm_min ? motor_pwm_min : motor_2);
	motor_3 = motor_3 > motor_pwm_max ? motor_pwm_max : (motor_3 < motor_pwm_min ? motor_pwm_min : motor_3);
	motor_4 = motor_4 > motor_pwm_max ? motor_pwm_max : (motor_4 < motor_pwm_min ? motor_pwm_min : motor_4);
	TIM_SetCompare1(TIM2, motor_1);
	TIM_SetCompare2(TIM2, motor_2);
	TIM_SetCompare3(TIM2, motor_3);
	TIM_SetCompare4(TIM2, motor_4);
}
